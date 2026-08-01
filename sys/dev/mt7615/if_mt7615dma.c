/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * MediaTek MT7615E: the packet DMA engine and its rings.
 *
 * The chip has one descriptor ring per direction and purpose, each
 * described by four registers: the descriptor base, the number of
 * descriptors, a CPU index the driver advances as it hands descriptors
 * over, and a DMA index the engine advances as it consumes them.  A
 * descriptor addresses up to two buffers; both lengths live in the
 * control word, and the engine sets DMA_DONE there when it is finished.
 *
 * Three Tx rings are used: one for data, one for MCU commands and one
 * for the firmware download, which needs its own because it runs before
 * the MCU is answering.  Two Rx rings come back: data and MCU events.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/systm.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_media.h>
#include <net/ethernet.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "if_mt7615reg.h"
#include "if_mt7615var.h"

static int	mt7615_rx_fill(struct mt7615_softc *, struct mt7615_rx_ring *);

static void
mt7615_dma_map_addr(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{

        if (error != 0)
                return;
        KASSERT(nsegs == 1, ("%d DMA segments, expected 1", nsegs));
        *(bus_addr_t *)arg = segs[0].ds_addr;
}

static int
mt7615_dma_contig_alloc(bus_dma_tag_t parent, struct mt7615_dma_info *dma,
                        bus_size_t size, bus_size_t alignment)
{
        int error;

        dma->tag = NULL;
        dma->map = NULL;
        dma->vaddr = NULL;
        dma->paddr = 0;
        dma->size = size;

        error = bus_dma_tag_create(parent, alignment, 0,
            BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL, size, 1,
            size, 0, NULL, NULL, &dma->tag);
        if (error != 0)
                goto fail;

        error = bus_dmamem_alloc(dma->tag, &dma->vaddr,
            BUS_DMA_NOWAIT | BUS_DMA_ZERO | BUS_DMA_COHERENT, &dma->map);
        if (error != 0)
                goto fail;

        error = bus_dmamap_load(dma->tag, dma->map, dma->vaddr, size,
            mt7615_dma_map_addr, &dma->paddr, BUS_DMA_NOWAIT);
        if (error != 0) {
                bus_dmamem_free(dma->tag, dma->vaddr, dma->map);
                dma->vaddr = NULL;
                goto fail;
        }

        bus_dmamap_sync(dma->tag, dma->map, BUS_DMASYNC_PREWRITE);
        return (0);

fail:
        if (dma->tag != NULL) {
                bus_dma_tag_destroy(dma->tag);
                dma->tag = NULL;
        }
        return (error);
}

static void
mt7615_dma_contig_free(struct mt7615_dma_info *dma)
{

        if (dma->vaddr != NULL) {
                bus_dmamap_sync(dma->tag, dma->map,
                    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
                bus_dmamap_unload(dma->tag, dma->map);
                bus_dmamem_free(dma->tag, dma->vaddr, dma->map);
                dma->vaddr = NULL;
        }
        if (dma->tag != NULL) {
                bus_dma_tag_destroy(dma->tag);
                dma->tag = NULL;
        }
}

/*
 * Ring register helpers.  Every ring's four registers sit at a fixed
 * offset from the ring's own base, so the ring only has to remember the
 * base it was given.
 */
static inline void
mt7615_q_write(struct mt7615_softc *sc, uint32_t regbase, uint32_t off,
               uint32_t val)
{

        mt7615_wr(sc, regbase + off, val);
}

static inline uint32_t
mt7615_q_read(struct mt7615_softc *sc, uint32_t regbase, uint32_t off)
{

        return (mt7615_rr(sc, regbase + off));
}

/*
 * Tx rings.
 */

static int
mt7615_alloc_tx_ring(struct mt7615_softc *sc, struct mt7615_tx_ring *ring,
                     int hw_idx, int ndesc, bool need_txd)
{
        int error, i;

        ring->hw_idx = hw_idx;
        ring->ndesc = ndesc;
        ring->regbase = MT_TX_RING_BASE + hw_idx * MT_RING_SIZE;
        ring->queued = 0;
        ring->cur = 0;
        ring->next = 0;

        error = mt7615_dma_contig_alloc(sc->sc_dmat, &ring->desc_dma,
            ndesc * sizeof(struct mt7615_desc), 16);
        if (error != 0) {
                device_printf(sc->sc_dev,
                    "could not allocate Tx ring %d descriptors: %d\n", hw_idx,
                    error);
                return (error);
        }
        ring->desc = ring->desc_dma.vaddr;

        ring->data = malloc(ndesc * sizeof(struct mt7615_tx_data), M_DEVBUF,
            M_NOWAIT | M_ZERO);
        if (ring->data == NULL)
                return (ENOMEM);

        if (need_txd) {
                error = mt7615_dma_contig_alloc(sc->sc_dmat, &ring->txd_dma,
                    ndesc * MT7615_TXD_STRIDE, MT7615_TXD_STRIDE);
                if (error != 0) {
                        device_printf(sc->sc_dev,
                            "could not allocate the Tx %d descriptor "
                            "scratch: %d\n", hw_idx, error);
                        return (error);
                }
        }

        /*
         * The data ring maps mbuf payloads, which can be scattered; the
         * MCU rings only ever queue one contiguous command buffer, but
         * the same tag serves both.
         */
        error = bus_dma_tag_create(sc->sc_dmat, 1, 0,
            BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
            MCLBYTES * MT_TXP_MAX_BUF_NUM, MT_TXP_MAX_BUF_NUM, MCLBYTES, 0,
            NULL, NULL, &ring->data_tag);
        if (error != 0) {
                device_printf(sc->sc_dev,
                    "could not create the Tx %d data tag: %d\n", hw_idx, error);
                return (error);
        }

        for (i = 0; i < ndesc; i++) {
                error = bus_dmamap_create(ring->data_tag, 0,
                    &ring->data[i].map);
                if (error != 0) {
                        device_printf(sc->sc_dev,
                            "could not create Tx %d map %d: %d\n", hw_idx, i,
                            error);
                        return (error);
                }
        }

        return (0);
}

static void
mt7615_reset_tx_ring(struct mt7615_softc *sc, struct mt7615_tx_ring *ring)
{
        struct mt7615_tx_data *data;
        int i;

        for (i = 0; i < ring->ndesc; i++) {
                data = &ring->data[i];

                if (data->m != NULL) {
                        bus_dmamap_sync(ring->data_tag, data->map,
                            BUS_DMASYNC_POSTWRITE);
                        bus_dmamap_unload(ring->data_tag, data->map);
                        m_freem(data->m);
                        data->m = NULL;
                }
                if (data->ni != NULL) {
                        ieee80211_free_node(data->ni);
                        data->ni = NULL;
                }
                if (data->cmd != NULL) {
                        bus_dmamap_sync(data->cmd_tag, data->cmd_map,
                            BUS_DMASYNC_POSTWRITE);
                        bus_dmamap_unload(data->cmd_tag, data->cmd_map);
                        bus_dmamem_free(data->cmd_tag, data->cmd,
                            data->cmd_map);
                        bus_dma_tag_destroy(data->cmd_tag);
                        data->cmd = NULL;
                        data->cmd_tag = NULL;
                }

                /*
                 * DMA_DONE marks a descriptor as spent, which is the
                 * state an unused Tx descriptor should start in.
                 */
                if (ring->desc != NULL)
                        ring->desc[i].ctrl = htole32(MT_DMA_CTL_DMA_DONE);
        }

        ring->queued = 0;
        ring->cur = 0;
        ring->next = 0;
}

static void
mt7615_free_tx_ring(struct mt7615_softc *sc, struct mt7615_tx_ring *ring)
{
        int i;

        if (ring->data != NULL) {
                mt7615_reset_tx_ring(sc, ring);
                for (i = 0; i < ring->ndesc; i++) {
                        if (ring->data[i].map != NULL)
                                bus_dmamap_destroy(ring->data_tag,
                                    ring->data[i].map);
                }
                free(ring->data, M_DEVBUF);
                ring->data = NULL;
        }
        if (ring->data_tag != NULL) {
                bus_dma_tag_destroy(ring->data_tag);
                ring->data_tag = NULL;
        }
        mt7615_dma_contig_free(&ring->txd_dma);
        mt7615_dma_contig_free(&ring->desc_dma);
        ring->desc = NULL;
}

/*
 * Rx rings.
 */

/*
 * Put a descriptor back into service without changing its buffer.  The
 * engine clears nothing on its own, so the length has to be rewritten
 * along with the now-cleared DMA_DONE bit.
 */
static void
mt7615_rx_rearm_desc(struct mt7615_rx_ring *ring, int idx)
{

        ring->desc[idx].info = 0;
        ring->desc[idx].ctrl = htole32((ring->buf_size <<
                                                       MT_DMA_CTL_SD_LEN0_S) & MT_DMA_CTL_SD_LEN0);
}

static int
mt7615_rx_alloc_mbuf(struct mt7615_softc *sc, struct mt7615_rx_ring *ring,
                     int idx)
{
        struct mt7615_rx_data *data;
        bus_dma_segment_t seg;
        bus_dmamap_t map;
        struct mbuf *m;
        int error, nsegs;

        data = &ring->data[idx];

        m = m_getjcl(M_NOWAIT, MT_DATA, M_PKTHDR, ring->buf_size);
        if (m == NULL)
                return (ENOBUFS);
        m->m_len = m->m_pkthdr.len = ring->buf_size;

        error = bus_dmamap_load_mbuf_sg(ring->data_tag, ring->spare_map, m,
            &seg, &nsegs, BUS_DMA_NOWAIT);
        if (error != 0) {
                m_freem(m);
                return (error);
        }
        KASSERT(nsegs == 1, ("%d Rx segments, expected 1", nsegs));

        if (data->m != NULL) {
                bus_dmamap_sync(ring->data_tag, data->map,
                    BUS_DMASYNC_POSTREAD);
                bus_dmamap_unload(ring->data_tag, data->map);
        }

        /* Swap the spare map in so the loaded one stays with the mbuf. */
        map = data->map;
        data->map = ring->spare_map;
        ring->spare_map = map;

        bus_dmamap_sync(ring->data_tag, data->map, BUS_DMASYNC_PREREAD);

        data->m = m;
        data->paddr = seg.ds_addr;

        ring->desc[idx].buf0 = htole32((uint32_t)data->paddr);
        ring->desc[idx].buf1 = 0;
        ring->desc[idx].info = 0;
        /*
         * No DMA_DONE: the engine sets it once it has written into the
         * buffer, and that is what tells the driver a frame arrived.
         */
        ring->desc[idx].ctrl = htole32((ring->buf_size <<
                                                       MT_DMA_CTL_SD_LEN0_S) & MT_DMA_CTL_SD_LEN0);

        return (0);
}

static int
mt7615_alloc_rx_ring(struct mt7615_softc *sc, struct mt7615_rx_ring *ring,
                     int hw_idx, int ndesc)
{
        int error, i;

        ring->hw_idx = hw_idx;
        ring->ndesc = ndesc;
        ring->regbase = MT_RX_RING_BASE + hw_idx * MT_RING_SIZE;
        ring->buf_size = MT7615_RX_BUF_SIZE;
        ring->cur = 0;

        error = mt7615_dma_contig_alloc(sc->sc_dmat, &ring->desc_dma,
            ndesc * sizeof(struct mt7615_desc), 16);
        if (error != 0) {
                device_printf(sc->sc_dev,
                    "could not allocate Rx ring %d descriptors: %d\n", hw_idx,
                    error);
                return (error);
        }
        ring->desc = ring->desc_dma.vaddr;

        ring->data = malloc(ndesc * sizeof(struct mt7615_rx_data), M_DEVBUF,
            M_NOWAIT | M_ZERO);
        if (ring->data == NULL)
                return (ENOMEM);

        error = bus_dma_tag_create(sc->sc_dmat, 1, 0,
            BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
            ring->buf_size, 1, ring->buf_size, 0, NULL, NULL,
            &ring->data_tag);
        if (error != 0) {
                device_printf(sc->sc_dev,
                    "could not create the Rx %d data tag: %d\n", hw_idx, error);
                return (error);
        }

        error = bus_dmamap_create(ring->data_tag, 0, &ring->spare_map);
        if (error != 0) {
                device_printf(sc->sc_dev,
                    "could not create the Rx %d spare map: %d\n", hw_idx,
                    error);
                return (error);
        }

        for (i = 0; i < ndesc; i++) {
                error = bus_dmamap_create(ring->data_tag, 0,
                    &ring->data[i].map);
                if (error != 0) {
                        device_printf(sc->sc_dev,
                            "could not create Rx %d map %d: %d\n", hw_idx, i,
                            error);
                        return (error);
                }
                error = mt7615_rx_alloc_mbuf(sc, ring, i);
                if (error != 0) {
                        device_printf(sc->sc_dev,
                            "could not fill Rx %d slot %d: %d\n", hw_idx, i,
                            error);
                        return (error);
                }
        }

        bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
            BUS_DMASYNC_PREWRITE);

        return (0);
}

static void
mt7615_free_rx_ring(struct mt7615_softc *sc, struct mt7615_rx_ring *ring)
{
        struct mt7615_rx_data *data;
        int i;

        if (ring->data != NULL) {
                for (i = 0; i < ring->ndesc; i++) {
                        data = &ring->data[i];

                        if (data->m != NULL) {
                                bus_dmamap_sync(ring->data_tag, data->map,
                                    BUS_DMASYNC_POSTREAD);
                                bus_dmamap_unload(ring->data_tag, data->map);
                                m_freem(data->m);
                                data->m = NULL;
                        }
                        if (data->map != NULL)
                                bus_dmamap_destroy(ring->data_tag, data->map);
                }
                free(ring->data, M_DEVBUF);
                ring->data = NULL;
        }
        if (ring->spare_map != NULL) {
                bus_dmamap_destroy(ring->data_tag, ring->spare_map);
                ring->spare_map = NULL;
        }
        if (ring->data_tag != NULL) {
                bus_dma_tag_destroy(ring->data_tag);
                ring->data_tag = NULL;
        }
        mt7615_dma_contig_free(&ring->desc_dma);
        ring->desc = NULL;
}

/*
 * Ring set-up and teardown.
 */

int
mt7615_dma_alloc(struct mt7615_softc *sc)
{
        static const struct {
            int	hw_idx;
            int	ndesc;
            bool	need_txd;
        } txq[MT7615_TXQ_COUNT] = {
            [MT7615_TXQ_DATA] = { MT7615_TXQ_HW_MAIN, MT7615_TX_RING_SIZE,
                                  true },
            [MT7615_TXQ_MCU]  = { MT7615_TXQ_HW_MCU, MT7615_TX_MCU_RING_SIZE,
                                  false },
            [MT7615_TXQ_FWDL] = { MT7615_TXQ_HW_FWDL,
                                  MT7615_TX_FWDL_RING_SIZE, false },
        };
        static const struct {
            int	hw_idx;
            int	ndesc;
        } rxq[MT7615_RXQ_COUNT] = {
            [MT7615_RXQ_DATA] = { MT7615_RXQ_HW_MAIN, MT7615_RX_RING_SIZE },
            [MT7615_RXQ_MCU]  = { MT7615_RXQ_HW_MCU, MT7615_RX_MCU_RING_SIZE },
        };
        int error, i;

        for (i = 0; i < MT7615_TXQ_COUNT; i++) {
                error = mt7615_alloc_tx_ring(sc, &sc->sc_txq[i], txq[i].hw_idx,
                    txq[i].ndesc, txq[i].need_txd);
                if (error != 0)
                        return (error);
                mt7615_reset_tx_ring(sc, &sc->sc_txq[i]);
        }

        for (i = 0; i < MT7615_RXQ_COUNT; i++) {
                error = mt7615_alloc_rx_ring(sc, &sc->sc_rxq[i], rxq[i].hw_idx,
                    rxq[i].ndesc);
                if (error != 0)
                        return (error);
        }

        error = mt7615_token_alloc(sc);
        if (error != 0)
                return (error);

        return (0);
}

/*
 * Tokens.
 *
 * A frame handed to the firmware stays alive until the firmware says
 * it is done, which it does by name rather than in order, so the
 * frames cannot be tracked by the ring they went out on.  Each gets a
 * slot here, and the slot owns the mapping too: the ring slot it was
 * queued from is reused as soon as the descriptor is read.
 */
int
mt7615_token_alloc(struct mt7615_softc *sc)
{
        bus_dma_tag_t tag;
        int error, i;

        sc->sc_token = malloc(MT7615_TOKEN_SIZE * sizeof(*sc->sc_token),
            M_DEVBUF, M_NOWAIT | M_ZERO);
        if (sc->sc_token == NULL)
                return (ENOMEM);

        tag = sc->sc_txq[MT7615_TXQ_DATA].data_tag;
        for (i = 0; i < MT7615_TOKEN_SIZE; i++) {
                error = bus_dmamap_create(tag, 0, &sc->sc_token[i].map);
                if (error != 0)
                        return (error);
        }
        sc->sc_token_next = 0;
        sc->sc_token_used = 0;

        return (0);
}

void
mt7615_token_free(struct mt7615_softc *sc)
{
        struct mt7615_token *tok;
        bus_dma_tag_t tag;
        int i;

        if (sc->sc_token == NULL)
                return;

        tag = sc->sc_txq[MT7615_TXQ_DATA].data_tag;
        if (tag == NULL)
                return;

        for (i = 0; i < MT7615_TOKEN_SIZE; i++) {
                tok = &sc->sc_token[i];
                if (tok->map == NULL)
                        continue;
                if (tok->m != NULL) {
                        bus_dmamap_unload(tag, tok->map);
                        m_freem(tok->m);
                        tok->m = NULL;
                }
                if (tok->ni != NULL) {
                        ieee80211_free_node(tok->ni);
                        tok->ni = NULL;
                }
                bus_dmamap_destroy(tag, tok->map);
                tok->map = NULL;
        }

        free(sc->sc_token, M_DEVBUF);
        sc->sc_token = NULL;
        sc->sc_token_used = 0;
}

/* Claim a free slot, or -1 if the firmware is holding all of them. */
int
mt7615_token_get(struct mt7615_softc *sc)
{
        int i, idx;

        MT7615_ASSERT_LOCKED(sc);

        if (sc->sc_token_used >= MT7615_TOKEN_SIZE)
                return (-1);

        for (i = 0; i < MT7615_TOKEN_SIZE; i++) {
                idx = (sc->sc_token_next + i) % MT7615_TOKEN_SIZE;
                if (sc->sc_token[idx].m == NULL) {
                        sc->sc_token_next = (idx + 1) % MT7615_TOKEN_SIZE;
                        sc->sc_token_used++;
                        return (idx);
                }
        }

        return (-1);
}

/* Give a slot back, releasing whatever it was holding. */
void
mt7615_token_put(struct mt7615_softc *sc, int idx)
{
        struct mt7615_token *tok;
        bus_dma_tag_t tag;

        MT7615_ASSERT_LOCKED(sc);

        if (idx < 0 || idx >= MT7615_TOKEN_SIZE)
                return;

        tok = &sc->sc_token[idx];
        if (tok->m == NULL)
                return;

        tag = sc->sc_txq[MT7615_TXQ_DATA].data_tag;
        bus_dmamap_sync(tag, tok->map, BUS_DMASYNC_POSTWRITE);
        bus_dmamap_unload(tag, tok->map);
        m_freem(tok->m);
        tok->m = NULL;

        if (tok->ni != NULL) {
                ieee80211_free_node(tok->ni);
                tok->ni = NULL;
        }
        sc->sc_token_used--;
}

void
mt7615_dma_free(struct mt7615_softc *sc)
{
        int i;

        mt7615_token_free(sc);
        for (i = 0; i < MT7615_TXQ_COUNT; i++)
                mt7615_free_tx_ring(sc, &sc->sc_txq[i]);
        for (i = 0; i < MT7615_RXQ_COUNT; i++)
                mt7615_free_rx_ring(sc, &sc->sc_rxq[i]);
}

static void
mt7615_tx_ring_program(struct mt7615_softc *sc, struct mt7615_tx_ring *ring)
{

        mt7615_q_write(sc, ring->regbase, MT_RING_DESC_BASE,
            (uint32_t)ring->desc_dma.paddr);
        mt7615_q_write(sc, ring->regbase, MT_RING_COUNT, ring->ndesc);
        mt7615_q_write(sc, ring->regbase, MT_RING_CPU_IDX, 0);
        mt7615_q_write(sc, ring->regbase, MT_RING_DMA_IDX, 0);

        /*
         * The engine may have kept an index of its own across a reset,
         * so take whatever it reports as the starting point.
         */
        ring->cur = ring->next =
            mt7615_q_read(sc, ring->regbase, MT_RING_DMA_IDX) % ring->ndesc;
}

static void
mt7615_rx_ring_program(struct mt7615_softc *sc, struct mt7615_rx_ring *ring)
{
        int i;

        /*
         * Buffers survive a reset, but their descriptors may be left
         * marked done, so re-arm every one before handing the ring back.
         */
        for (i = 0; i < ring->ndesc; i++) {
                if (ring->data[i].m != NULL)
                        mt7615_rx_rearm_desc(ring, i);
        }
        bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
            BUS_DMASYNC_PREWRITE);

        mt7615_q_write(sc, ring->regbase, MT_RING_DESC_BASE,
            (uint32_t)ring->desc_dma.paddr);
        mt7615_q_write(sc, ring->regbase, MT_RING_COUNT, ring->ndesc);
        mt7615_q_write(sc, ring->regbase, MT_RING_DMA_IDX, 0);
        /* One descriptor stays behind the read pointer as the stop mark. */
        mt7615_q_write(sc, ring->regbase, MT_RING_CPU_IDX, ring->ndesc - 1);
        ring->cur = 0;
}

void
mt7615_dma_reset_rings(struct mt7615_softc *sc)
{
        int i;

        for (i = 0; i < MT7615_TXQ_COUNT; i++) {
                mt7615_reset_tx_ring(sc, &sc->sc_txq[i]);
                bus_dmamap_sync(sc->sc_txq[i].desc_dma.tag,
                    sc->sc_txq[i].desc_dma.map, BUS_DMASYNC_PREWRITE);
                mt7615_tx_ring_program(sc, &sc->sc_txq[i]);
        }
        for (i = 0; i < MT7615_RXQ_COUNT; i++) {
                /* Slots emptied before the reset need buffers again. */
                (void)mt7615_rx_fill(sc, &sc->sc_rxq[i]);
                mt7615_rx_ring_program(sc, &sc->sc_rxq[i]);
        }
}

/*
 * Bring the DMA engine up.  The magic constants come straight from the
 * Linux mt76 driver's mt7615_dma_init(); MediaTek has never documented
 * them, and the chip does not work without them.
 */
int
mt7615_dma_init(struct mt7615_softc *sc)
{
        uint32_t val;

        MT7615_ASSERT_LOCKED(sc);

        mt7615_wr(sc, MT_WPDMA_GLO_CFG,
            MT_WPDMA_GLO_CFG_TX_WRITEBACK_DONE |
            MT_WPDMA_GLO_CFG_FIFO_LITTLE_ENDIAN |
            MT_WPDMA_GLO_CFG_OMIT_TX_INFO);

        val = mt7615_rr(sc, MT_WPDMA_GLO_CFG);
        val &= ~(MT_WPDMA_GLO_CFG_TX_BT_SIZE_BIT21 |
                 MT_WPDMA_GLO_CFG_DMA_BURST_SIZE |
                 MT_WPDMA_GLO_CFG_MULTI_DMA_EN);
        val |= MT_WPDMA_GLO_CFG_TX_BT_SIZE_BIT0;
        val |= (1U << MT_WPDMA_GLO_CFG_TX_BT_SIZE_BIT21_S) &
               MT_WPDMA_GLO_CFG_TX_BT_SIZE_BIT21;
        val |= (3U << MT_WPDMA_GLO_CFG_DMA_BURST_SIZE_S) &
               MT_WPDMA_GLO_CFG_DMA_BURST_SIZE;
        val |= (3U << MT_WPDMA_GLO_CFG_MULTI_DMA_EN_S) &
               MT_WPDMA_GLO_CFG_MULTI_DMA_EN;
        val |= MT_WPDMA_GLO_CFG_FIRST_TOKEN_ONLY;
        mt7615_wr(sc, MT_WPDMA_GLO_CFG, val);

        mt7615_wr(sc, MT_WPDMA_GLO_CFG1, 0x1);
        mt7615_wr(sc, MT_WPDMA_TX_PRE_CFG, 0xf0000);
        mt7615_wr(sc, MT_WPDMA_RX_PRE_CFG, 0xf7f0000);
        mt7615_wr(sc, MT_WPDMA_ABT_CFG, 0x4000026);
        mt7615_wr(sc, MT_WPDMA_ABT_CFG1, 0x18811881);
        mt7615_set(sc, 0x7158, (1U << 16));
        mt7615_clear(sc, 0x7000, (1U << 23));

        mt7615_wr(sc, MT_WPDMA_RST_IDX, ~0U);

        mt7615_dma_reset_rings(sc);

        mt7615_wr(sc, MT_DELAY_INT_CFG, 0);

        if (!mt7615_poll(sc, MT_WPDMA_GLO_CFG,
            MT_WPDMA_GLO_CFG_TX_DMA_BUSY | MT_WPDMA_GLO_CFG_RX_DMA_BUSY, 0,
            1000))
                device_printf(sc->sc_dev, "the DMA engine is still busy\n");

        /* Start it.  Interrupts stay masked until the caller asks. */
        mt7615_set(sc, MT_WPDMA_GLO_CFG,
            MT_WPDMA_GLO_CFG_TX_DMA_EN | MT_WPDMA_GLO_CFG_RX_DMA_EN |
            MT_WPDMA_GLO_CFG_TX_WRITEBACK_DONE);

        sc->sc_flags |= MT7615_FLAG_DMA_INITED;

        return (0);
}

void
mt7615_dma_stop(struct mt7615_softc *sc)
{

        MT7615_ASSERT_LOCKED(sc);

        if ((sc->sc_flags & MT7615_FLAG_DMA_INITED) == 0)
                return;

        mt7615_clear(sc, MT_WPDMA_GLO_CFG,
            MT_WPDMA_GLO_CFG_TX_DMA_EN | MT_WPDMA_GLO_CFG_RX_DMA_EN);
        mt7615_set(sc, MT_WPDMA_GLO_CFG, MT_WPDMA_GLO_CFG_SW_RESET);

        sc->sc_flags &= ~MT7615_FLAG_DMA_INITED;
}

/*
 * Interrupt mask bookkeeping.  The chip's mask register is write-only
 * as far as the driver is concerned, so the wanted value is kept in the
 * softc and rewritten whenever part of it changes.
 */
void
mt7615_irq_enable(struct mt7615_softc *sc, uint32_t mask)
{

        MT7615_ASSERT_LOCKED(sc);

        sc->sc_intmask |= mask;
        mt7615_wr(sc, MT_INT_MASK_CSR, sc->sc_intmask);
}

void
mt7615_irq_disable(struct mt7615_softc *sc, uint32_t mask)
{

        MT7615_ASSERT_LOCKED(sc);

        sc->sc_intmask &= ~mask;
        mt7615_wr(sc, MT_INT_MASK_CSR, sc->sc_intmask);
}

/*
 * Hand a buffer to a Tx ring.
 *
 * Either an mbuf chain or a flat command buffer is queued, never both:
 * MCU commands arrive as a physically contiguous block, data frames as
 * an already-mapped mbuf.  The caller owns the mapping until the ring
 * reclaims the descriptor.
 *
 * A descriptor carries two buffers.  A command uses only the first and
 * passes len1 as zero; a data frame follows it with the head of the
 * frame itself, which the firmware parses the header out of.
 *
 * Returns the descriptor index used, or -1 if the ring is full.
 */
int
mt7615_tx_queue_buf(struct mt7615_softc *sc, struct mt7615_tx_ring *ring,
                    bus_addr_t paddr, int len, bus_addr_t paddr1, int len1, void *cookie,
                    struct mbuf *m, struct ieee80211_node *ni)
{
        struct mt7615_desc *desc;
        struct mt7615_tx_data *data;
        uint32_t ctrl;
        int idx;

        MT7615_ASSERT_LOCKED(sc);

        /* -1 rather than an errno: any value >= 0 is a valid index. */
        if (ring->queued >= ring->ndesc - 1)
                return (-1);

        idx = ring->cur;
        desc = &ring->desc[idx];
        data = &ring->data[idx];

        ctrl = (len << MT_DMA_CTL_SD_LEN0_S) & MT_DMA_CTL_SD_LEN0;
        if (len1 > 0) {
                ctrl |= ((len1 << MT_DMA_CTL_SD_LEN1_S) & MT_DMA_CTL_SD_LEN1) |
                        MT_DMA_CTL_LAST_SEC1;
                desc->buf1 = htole32((uint32_t)paddr1);
        } else {
                ctrl |= MT_DMA_CTL_LAST_SEC0;
                desc->buf1 = 0;
        }

        desc->buf0 = htole32((uint32_t)paddr);
        desc->info = 0;
        desc->ctrl = htole32(ctrl);

        data->m = m;
        data->ni = ni;
        data->cmd = cookie;

        bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
            BUS_DMASYNC_PREWRITE);

        ring->cur = (idx + 1) % ring->ndesc;
        ring->queued++;

        mt7615_q_write(sc, ring->regbase, MT_RING_CPU_IDX, ring->cur);

        return (idx);
}

/*
 * Reclaim descriptors the engine has finished with.  The DMA index is
 * the first descriptor the engine has *not* consumed, so everything
 * from the driver's reclaim pointer up to it is done.
 */
void
mt7615_tx_cleanup(struct mt7615_softc *sc, struct mt7615_tx_ring *ring)
{
        struct mt7615_tx_data *data;
        uint32_t dma_idx;

        MT7615_ASSERT_LOCKED(sc);

        dma_idx = mt7615_q_read(sc, ring->regbase, MT_RING_DMA_IDX) %
                  ring->ndesc;

        bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
            BUS_DMASYNC_POSTREAD);

        while (ring->queued > 0 && ring->next != dma_idx) {
                data = &ring->data[ring->next];

                if (data->m != NULL) {
                        bus_dmamap_sync(ring->data_tag, data->map,
                            BUS_DMASYNC_POSTWRITE);
                        bus_dmamap_unload(ring->data_tag, data->map);
                        m_freem(data->m);
                        data->m = NULL;
                }
                if (data->ni != NULL) {
                        ieee80211_free_node(data->ni);
                        data->ni = NULL;
                }
                if (data->cmd != NULL) {
                        bus_dmamap_sync(data->cmd_tag, data->cmd_map,
                            BUS_DMASYNC_POSTWRITE);
                        bus_dmamap_unload(data->cmd_tag, data->cmd_map);
                        bus_dmamem_free(data->cmd_tag, data->cmd,
                            data->cmd_map);
                        bus_dma_tag_destroy(data->cmd_tag);
                        data->cmd = NULL;
                        data->cmd_tag = NULL;
                }

                ring->next = (ring->next + 1) % ring->ndesc;
                ring->queued--;
        }
}

/*
 * Refill an Rx ring after its buffers have been consumed, and tell the
 * engine how far it may now write.
 */
static int
mt7615_rx_fill(struct mt7615_softc *sc, struct mt7615_rx_ring *ring)
{
        int i;

        MT7615_ASSERT_LOCKED(sc);

        for (i = 0; i < ring->ndesc; i++) {
                if (ring->data[i].m != NULL)
                        continue;
                if (mt7615_rx_alloc_mbuf(sc, ring, i) != 0)
                        return (ENOBUFS);
        }

        bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
            BUS_DMASYNC_PREWRITE);

        return (0);
}

/*
 * Drain one Rx ring.  Frames are pulled off, replaced with fresh
 * buffers and passed to the MCU or MAC handler depending on the ring;
 * both take ownership of the mbuf.
 */
void
mt7615_rx_poll(struct mt7615_softc *sc, int qid)
{
        struct mt7615_rx_ring *ring;
        struct mt7615_rx_data *data;
        struct mt7615_desc *desc;
        struct mbuf *m;
        uint32_t ctrl;
        int len, nframes;

        MT7615_ASSERT_LOCKED(sc);

        ring = &sc->sc_rxq[qid];
        if (ring->polling)
                return;
        ring->polling = true;
        nframes = 0;

        bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
            BUS_DMASYNC_POSTREAD);

        for (; nframes < ring->ndesc; nframes++) {
                desc = &ring->desc[ring->cur];
                ctrl = le32toh(desc->ctrl);

                if ((ctrl & MT_DMA_CTL_DMA_DONE) == 0)
                        break;

                data = &ring->data[ring->cur];
                if (data->m == NULL)
                        break;

                len = (ctrl & MT_DMA_CTL_SD_LEN0) >> MT_DMA_CTL_SD_LEN0_S;
                if (len == 0 || len > ring->buf_size)
                        len = 0;

                /*
                 * Hand the slot a fresh buffer first.  If none can be
                 * had the received frame is dropped and its buffer put
                 * straight back, which keeps the ring full at the cost
                 * of a packet.
                 */
                m = data->m;
                if (mt7615_rx_alloc_mbuf(sc, ring, ring->cur) != 0) {
                        /*
                         * The slot is untouched on failure, so only the
                         * control word has to be re-armed.
                         */
                        mt7615_rx_rearm_desc(ring, ring->cur);
                        ring->cur = (ring->cur + 1) % ring->ndesc;
                        continue;
                }

                ring->cur = (ring->cur + 1) % ring->ndesc;

                if (len == 0) {
                        m_freem(m);
                        continue;
                }

                m->m_len = m->m_pkthdr.len = len;

                if (qid == MT7615_RXQ_MCU)
                        mt7615_mcu_rx_event(sc, m);
                else
                        mt7615_mac_rx_event(sc, m);
        }

        if (nframes > 0) {
                bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
                    BUS_DMASYNC_PREWRITE);
                /*
                 * The engine may write up to, but not including, the CPU
                 * index, so hand it everything behind the read pointer.
                 */
                mt7615_q_write(sc, ring->regbase, MT_RING_CPU_IDX,
                    (ring->cur + ring->ndesc - 1) % ring->ndesc);
        }

        ring->polling = false;
}
