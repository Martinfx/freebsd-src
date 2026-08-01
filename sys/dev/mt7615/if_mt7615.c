/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * MediaTek MT7615E PCIe driver.
 *
 * STATE.  The chip is brought all the way up: BAR0 and the interrupt
 * are claimed, the DMA rings are built, the MCU command transport is
 * established over them and the three firmware images are downloaded
 * and started.  Once the MCU answers, the MAC is initialised and the
 * interface is registered with net80211, so an mt7615 device appears
 * and can be configured.
 *
 * A network is described to the firmware in three nested pieces: a
 * device is a hardware MAC slot, a BSS is a network on that slot, and a
 * station record is one peer inside it.  Reaching RUN builds all three;
 * an access point then hands over a beacon for the firmware to repeat,
 * and each peer that associates gets a record of its own.
 *
 * WHAT IS NOT DONE YET.  Encryption keys are not programmed into the
 * hardware, so an access point can only be run open - net80211 will do
 * WPA in software for management frames, but data frames are neither
 * encrypted nor decrypted by the chip, and there is no key command to
 * make it.  Rate control is left at whatever the firmware picks, since
 * the rate table command is not written either, and A-MPDU is not
 * negotiated.  Those are all further MCU commands over the transport
 * that is already here rather than new infrastructure.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/eventhandler.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sbuf.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/taskqueue.h>

#include <machine/atomic.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/ethernet.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_ratectl.h>
#include <net80211/ieee80211_regdomain.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "if_mt7615reg.h"
#include "if_mt7615var.h"

static int	mt7615_probe(device_t);
static int	mt7615_attach(device_t);
static int	mt7615_detach(device_t);
static int	mt7615_suspend(device_t);
static int	mt7615_resume(device_t);

static void	mt7615_attach_hook(void *);
static int	mt7615_intr(void *);
static void	mt7615_intr_thread(void *);
static void	mt7615_watchdog(void *);

static struct ieee80211vap *mt7615_vap_create(struct ieee80211com *,
                                              const char [IFNAMSIZ], int, enum ieee80211_opmode, int,
                                              const uint8_t [IEEE80211_ADDR_LEN],
                                              const uint8_t [IEEE80211_ADDR_LEN]);
static void	mt7615_vap_delete(struct ieee80211vap *);
static int	mt7615_newstate(struct ieee80211vap *, enum ieee80211_state,
                                  int);
static void	mt7615_parent(struct ieee80211com *);
static int	mt7615_transmit(struct ieee80211com *, struct mbuf *);
static int	mt7615_raw_xmit(struct ieee80211_node *, struct mbuf *,
                                  const struct ieee80211_bpf_params *);
static void	mt7615_scan_start(struct ieee80211com *);
static void	mt7615_scan_end(struct ieee80211com *);
static void	mt7615_set_channel(struct ieee80211com *);
static void	mt7615_update_mcast(struct ieee80211com *);
static void	mt7615_update_promisc(struct ieee80211com *);
static void	mt7615_getradiocaps(struct ieee80211com *, int, int *,
                                       struct ieee80211_channel[]);
static int	mt7615_setregdomain(struct ieee80211com *,
                                      struct ieee80211_regdomain *, int,
                                      struct ieee80211_channel[]);
static void	mt7615_newassoc(struct ieee80211_node *, int);
static int	mt7615_key_alloc(struct ieee80211vap *, struct ieee80211_key *,
                                   ieee80211_keyix *, ieee80211_keyix *);
static int	mt7615_key_set(struct ieee80211vap *,
                                 const struct ieee80211_key *);
static int	mt7615_key_delete(struct ieee80211vap *,
                                    const struct ieee80211_key *);
static void	mt7615_sta_task(void *, int);
static void	mt7615_txs_task(void *, int);
static void	mt7615_update_beacon(struct ieee80211vap *, int);
static void	mt7615_beacon_task(void *, int);
static int	mt7615_wme_update(struct ieee80211com *);
static void	mt7615_radiotap_attach(struct mt7615_softc *);

static int	mt7615_init(struct mt7615_softc *);
static void	mt7615_stop(struct mt7615_softc *);
static int	mt7615_tx(struct mt7615_softc *, struct mbuf *,
                            struct ieee80211_node *);

/*
 * Register access.
 *
 * Everything below MT7615_DIRECT_LIMIT is addressed straight in BAR0.
 * Anything above needs the remap window moved first, which is why the
 * lock is required: two threads sliding the window under each other
 * would each read the wrong register.
 */
static uint32_t
mt7615_reg_addr(struct mt7615_softc *sc, uint32_t addr)
{
        uint32_t base, offset;

        if (addr < MT7615_DIRECT_LIMIT)
                return (addr);

        MT7615_ASSERT_LOCKED(sc);

        base = addr & MT_MCU_PCIE_REMAP_2_BASE;
        offset = addr & MT_MCU_PCIE_REMAP_2_OFFSET;

        bus_write_4(sc->sc_mem, MT_MCU_PCIE_REMAP_2, base);
        /* Push the window change out ahead of the access using it. */
        (void)bus_read_4(sc->sc_mem, MT_MCU_PCIE_REMAP_2);

        return (MT_PCIE_REMAP_BASE_2 + offset);
}

uint32_t
mt7615_rr(struct mt7615_softc *sc, uint32_t addr)
{

        return (bus_read_4(sc->sc_mem, mt7615_reg_addr(sc, addr)));
}

void
mt7615_wr(struct mt7615_softc *sc, uint32_t addr, uint32_t val)
{

        bus_write_4(sc->sc_mem, mt7615_reg_addr(sc, addr), val);
}

uint32_t
mt7615_rmw(struct mt7615_softc *sc, uint32_t addr, uint32_t mask, uint32_t val)
{
        uint32_t reg, cur;

        reg = mt7615_reg_addr(sc, addr);
        cur = bus_read_4(sc->sc_mem, reg);
        val |= cur & ~mask;
        bus_write_4(sc->sc_mem, reg, val);

        return (val);
}

/*
 * Wait for a field to take a value, in milliseconds.  Returns nonzero
 * if it did.
 */
int
mt7615_poll(struct mt7615_softc *sc, uint32_t addr, uint32_t mask,
            uint32_t val, int timeout_ms)
{
        int i;

        /* 10us a turn, so a hundred turns to the millisecond. */
        timeout_ms *= 100;
        for (i = 0; i < timeout_ms; i++) {
                if ((mt7615_rr(sc, addr) & mask) == val)
                        return (1);
                DELAY(10);
        }

        return (0);
}

/*
 * Factory data.
 */

static int
mt7615_efuse_read(struct mt7615_softc *sc, uint16_t addr, uint8_t *data)
{
        uint32_t val;
        int i;

        MT7615_ASSERT_LOCKED(sc);

        val = mt7615_rr(sc, MT_EFUSE_BASE + MT_EFUSE_CTRL);
        val &= ~(MT_EFUSE_CTRL_AIN | MT_EFUSE_CTRL_MODE);
        val |= ((uint32_t)(addr & ~0xf) << MT_EFUSE_CTRL_AIN_SHIFT) &
               MT_EFUSE_CTRL_AIN;
        val |= MT_EFUSE_CTRL_KICK;
        mt7615_wr(sc, MT_EFUSE_BASE + MT_EFUSE_CTRL, val);

        for (i = 0; i < 1000; i++) {
                val = mt7615_rr(sc, MT_EFUSE_BASE + MT_EFUSE_CTRL);
                if ((val & MT_EFUSE_CTRL_KICK) == 0)
                        break;
                DELAY(10);
        }
        if ((val & MT_EFUSE_CTRL_KICK) != 0)
                return (ETIMEDOUT);

        DELAY(2);

        val = mt7615_rr(sc, MT_EFUSE_BASE + MT_EFUSE_CTRL);
        if ((val & MT_EFUSE_CTRL_AOUT) == MT_EFUSE_CTRL_AOUT ||
            (val & MT_EFUSE_CTRL_VALID) == 0) {
                /* Never programmed; report zeroes rather than failing. */
                memset(data, 0, MT7615_EFUSE_BLOCK);
                return (0);
        }

        for (i = 0; i < MT7615_EFUSE_BLOCK / 4; i++) {
                val = mt7615_rr(sc, MT_EFUSE_BASE + MT_EFUSE_RDATA(i));
                le32enc(data + 4 * i, val);
        }

        return (0);
}

/*
 * Boards that keep the address in their own flash rather than in the
 * card leave the efuse blank.  The blank cases do not report an error:
 * the whole efuse announces itself as empty through one bit, and an
 * individual block that was never programmed reads back as zeroes with
 * a clean status.  So judge the address that came back rather than the
 * fact that a read completed.
 */
static int
mt7615_read_macaddr(struct mt7615_softc *sc)
{
        uint8_t block[MT7615_EFUSE_BLOCK];
        uint32_t val;
        int error;

        MT7615_ASSERT_LOCKED(sc);

        val = mt7615_rr(sc, MT_EFUSE_BASE + MT_EFUSE_BASE_CTRL);
        if ((val & MT_EFUSE_BASE_CTRL_EMPTY) != 0) {
                device_printf(sc->sc_dev, "the efuse is empty; this board "
                                          "keeps the MAC address elsewhere\n");
                return (0);
        }

        error = mt7615_efuse_read(sc, 0, block);
        if (error != 0)
                return (error);

        memcpy(sc->sc_macaddr, block + MT_EE_MAC_ADDR, sizeof(sc->sc_macaddr));

        /*
         * All zeroes means the block was never written, and a group
         * address is not a station address at all; either way there is
         * nothing usable here and the caller has to make one up.
         */
        if (ETHER_IS_ZERO(sc->sc_macaddr) ||
            ETHER_IS_MULTICAST(sc->sc_macaddr))
                return (0);

        sc->sc_have_macaddr = true;
        device_printf(sc->sc_dev, "MAC address %6D\n", sc->sc_macaddr, ":");

        /*
         * How many chains this board wired up.  Claiming more than there
         * are is not harmless: the network advertises rates that need
         * them, a peer agrees to be sent at those rates, and every such
         * frame then fails.
         *
         * Only read once the address above has proved the block was
         * programmed - a board that carries no address of its own carries
         * no chain count either, whatever the bits happen to say.  Even
         * then it describes the part rather than the board: a four by four
         * chip with one antenna fitted answers the same as one with four.
         */
        error = mt7615_efuse_read(sc, MT_EE_NIC_CONF_0 & ~0xf, block);
        if (error == 0) {
                uint8_t nss;

                nss = (block[MT_EE_NIC_CONF_0 & 0xf] &
                       MT_EE_NIC_CONF_TX_MASK) >> MT_EE_NIC_CONF_TX_MASK_S;
                if (nss >= 1 && nss <= MT7615_MAX_CHAINS)
                        sc->sc_nss = nss;
        }

        return (0);
}

/*
 * Interrupts.
 *
 * Split in two, the way a driver that has real work to do must be.
 * The filter runs in a context that may not block on anything: no
 * mutex, so no driver lock and no taskqueue either, since enqueueing
 * takes the queue's own lock.  All it may do is touch the registers
 * and hand the work on.  The rest runs in the interrupt thread, where
 * locking is allowed and the Rx path can call into net80211 - which
 * needs to be able to block, and to re-enter this driver.
 */
static int
mt7615_intr(void *arg)
{
        struct mt7615_softc *sc;
        uint32_t status;

        sc = arg;

        status = bus_read_4(sc->sc_mem, MT_INT_SOURCE_CSR);
        if (status == 0 || status == 0xffffffff)
                return (FILTER_STRAY);

        status &= sc->sc_intmask;
        if (status == 0)
                return (FILTER_STRAY);

        /* Acknowledge and mask; the thread re-enables what it handled. */
        bus_write_4(sc->sc_mem, MT_INT_SOURCE_CSR, status);
        bus_write_4(sc->sc_mem, MT_INT_MASK_CSR, 0);

        /*
         * The thread collects this without the filter being able to take
         * the driver lock, so the two have to meet atomically.
         */
        atomic_set_int(&sc->sc_intstatus, status);
        atomic_add_long(&sc->sc_intr_count, 1);

        return (FILTER_SCHEDULE_THREAD);
}

static void
mt7615_intr_thread(void *arg)
{
        struct mt7615_softc *sc;
        uint32_t status, mcu_int;

        sc = arg;

        MT7615_LOCK(sc);

        status = atomic_readandclear_int(&sc->sc_intstatus);

        if ((status & MT_INT_RX_DONE(MT7615_RXQ_HW_MAIN)) != 0)
                mt7615_rx_poll(sc, MT7615_RXQ_DATA);

        if ((status & MT_INT_RX_DONE(MT7615_RXQ_HW_MCU)) != 0)
                mt7615_rx_poll(sc, MT7615_RXQ_MCU);

        if ((status & MT_INT_TX_DONE_ALL) != 0) {
                int i;

                for (i = 0; i < MT7615_TXQ_COUNT; i++)
                        mt7615_tx_cleanup(sc, &sc->sc_txq[i]);

                /* Room may have appeared for frames that were held back. */
                mt7615_start(sc);
        }

        if ((status & MT_INT_MCU_CMD) != 0) {
                mcu_int = mt7615_rr(sc, MT_MCU_CMD);
                mt7615_wr(sc, MT_MCU_CMD, mcu_int);
                if ((mcu_int & MT_MCU_CMD_ERROR_MASK) != 0)
                        device_printf(sc->sc_dev,
                            "the MCU reported an error: %#x\n",
                            mcu_int & MT_MCU_CMD_ERROR_MASK);
        }

        /* Put the mask back the way it was. */
        mt7615_wr(sc, MT_INT_MASK_CSR, sc->sc_intmask);

        MT7615_UNLOCK(sc);
}

/*
 * Transmit.
 */

static int
mt7615_tx(struct mt7615_softc *sc, struct mbuf *m, struct ieee80211_node *ni)
{
        struct ieee80211com *ic;
        struct ieee80211_frame *wh;
        struct mt7615_tx_ring *ring;
        struct mt7615_fw_txp *txp;
        bus_dma_segment_t segs[MT_TXP_MAX_BUF_NUM];
        bus_addr_t txd_paddr;
        uint32_t *txd;
        uint16_t flags;
        int ac, error, i, nsegs, idx, parse_len, tok;

        MT7615_ASSERT_LOCKED(sc);

        ic = &sc->sc_ic;
        ring = &sc->sc_txq[MT7615_TXQ_DATA];

        if (ring->queued >= ring->ndesc - 2) {
                m_freem(m);
                return (ENOBUFS);
        }

        ac = M_WME_GETAC(m);

        /*
         * A frame that may be aggregated left net80211 without a sequence
         * number: it holds them back so that whatever queues the
         * aggregate can hand them out in order, and here that is this
         * function.  Numbering it now keeps the block-ack window on
         * net80211's side of the fence, which is where it is kept.
         */
        if ((m->m_flags & M_AMPDU_MPDU) != 0)
                ieee80211_output_seqno_assign(ni, -1, m);

        if (ieee80211_radiotap_active_vap(ni->ni_vap)) {
                struct mt7615_tx_radiotap_header *tap;

                tap = &sc->sc_txtap;
                tap->wt_flags = 0;
                tap->wt_rate = 0;
                tap->wt_chan_freq = htole16(ic->ic_curchan->ic_freq);
                tap->wt_chan_flags = htole16(ic->ic_curchan->ic_flags);
                ieee80211_radiotap_tx(ni->ni_vap, m);
        }

        /*
         * The frame itself is mapped where it lies, so nothing is
         * copied; only the descriptor and the pointer block are written,
         * into this slot's scratch area.  They need a buffer of their
         * own rather than a header mbuf, because the engine is handed
         * them through the ring descriptor while the payload is reached
         * through the pointer list inside them.
         */
        idx = ring->cur;

        /*
         * The frame outlives the descriptor: the engine reads only the
         * block below, and the firmware fetches the body afterwards from
         * the addresses in it.  So the mapping is taken against a token
         * rather than the ring slot, and neither is released until the
         * firmware hands the token back.
         */
        tok = mt7615_token_get(sc);
        if (tok < 0) {
                m_freem(m);
                return (ENOBUFS);
        }

        error = bus_dmamap_load_mbuf_sg(ring->data_tag, sc->sc_token[tok].map,
            m, segs, &nsegs, BUS_DMA_NOWAIT);
        if (error == EFBIG) {
                struct mbuf *mn;

                mn = m_collapse(m, M_NOWAIT, MT_TXP_MAX_BUF_NUM);
                if (mn == NULL) {
                        m_freem(m);
                        return (ENOBUFS);
                }
                m = mn;
                error = bus_dmamap_load_mbuf_sg(ring->data_tag,
                    sc->sc_token[tok].map, m, segs, &nsegs, BUS_DMA_NOWAIT);
        }
        if (error != 0) {
                sc->sc_token_used--;
                m_freem(m);
                return (error);
        }

        txd = (uint32_t *)((uint8_t *)ring->txd_dma.vaddr +
                           idx * MT7615_TXD_STRIDE);
        txd_paddr = ring->txd_dma.paddr + idx * MT7615_TXD_STRIDE;

        mt7615_mac_write_txd(txd, m, ac, MT7615_VAP(ni->ni_vap), ni);

        wh = mtod(m, struct ieee80211_frame *);

        /*
         * The pointer block lists where the rest of the frame lies.  Its
         * flags are what make the firmware send the descriptor built
         * above rather than one of its own, so they are not optional.
         */
        txp = (struct mt7615_fw_txp *)((uint8_t *)txd + MT_TXD_SIZE);
        memset(txp, 0, sizeof(*txp));
        flags = MT_CT_INFO_APPLY_TXD | MT_CT_INFO_NONE_CIPHER_FRAME;
        if ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_MGT)
                flags |= MT_CT_INFO_MGMT_FRAME;
        txp->flags = htole16(flags);
        txp->token = htole16(tok);
        txp->bss_idx = MT7615_VAP(ni->ni_vap)->omac_idx;
        /* No WDS repeater, which the low byte alone says. */
        txp->rept_wds_wcid = htole16(0x00ff);
        /*
         * How many pieces the frame is in and where each one lies.  The
         * count alone marks the end of the list; the lengths are plain
         * byte counts with nothing else folded into them.
         */
        txp->nbuf = MIN(nsegs, MT_TXP_MAX_BUF_NUM);
        for (i = 0; i < txp->nbuf; i++) {
                txp->buf[i] = htole32((uint32_t)segs[i].ds_addr);
                txp->len[i] = htole16(segs[i].ds_len);
        }

        bus_dmamap_sync(ring->data_tag, sc->sc_token[tok].map,
            BUS_DMASYNC_PREWRITE);
        bus_dmamap_sync(ring->txd_dma.tag, ring->txd_dma.map,
            BUS_DMASYNC_PREWRITE);

        /*
         * The descriptor points at the block just built and then at the
         * head of the frame, which the firmware reads the header out of
         * without having to fetch it.  The same bytes appear again in the
         * pointer list above; there is one mapping behind both, unloaded
         * once when the ring reclaims the descriptor.
         */
        parse_len = MIN(MT_CT_PARSE_LEN, (int)segs[0].ds_len);

        /*
         * The ring slot is handed nothing to own.  It is free again as
         * soon as the descriptor is read, and by then the frame is still
         * the firmware's.
         */
        if (mt7615_tx_queue_buf(sc, ring, txd_paddr, MT_TXD_TXP_SIZE,
            segs[0].ds_addr, parse_len, NULL, NULL, NULL) < 0) {
                bus_dmamap_unload(ring->data_tag, sc->sc_token[tok].map);
                sc->sc_token_used--;
                m_freem(m);
                return (ENOBUFS);
        }

        sc->sc_token[tok].m = m;
        sc->sc_token[tok].ni = ni;

        return (0);
}

void
mt7615_start(struct mt7615_softc *sc)
{
        struct ieee80211_node *ni;
        struct mbuf *m;

        MT7615_ASSERT_LOCKED(sc);

        if ((sc->sc_flags & MT7615_FLAG_HW_INITED) == 0)
                return;

        /*
         * Stop while there is still a ring slot and a token to be had.
         * Handing a frame down without one means dropping it after it has
         * already been accepted, and a lost frame costs a sender far more
         * than the wait for the queue to drain would have.
         */
        while (sc->sc_txq[MT7615_TXQ_DATA].queued <
               sc->sc_txq[MT7615_TXQ_DATA].ndesc - 2 &&
               sc->sc_token_used < MT7615_TOKEN_SIZE &&
               (m = mbufq_dequeue(&sc->sc_snd)) != NULL) {
                ni = (struct ieee80211_node *)m->m_pkthdr.rcvif;
                m->m_pkthdr.rcvif = NULL;
                if (mt7615_tx(sc, m, ni) != 0) {
                        if_inc_counter(ni->ni_vap->iv_ifp, IFCOUNTER_OERRORS,
                            1);
                        ieee80211_free_node(ni);
                }
        }
}

static int
mt7615_transmit(struct ieee80211com *ic, struct mbuf *m)
{
        struct mt7615_softc *sc;
        int error;

        sc = ic->ic_softc;

        MT7615_LOCK(sc);
        if ((sc->sc_flags & MT7615_FLAG_HW_INITED) == 0) {
                MT7615_UNLOCK(sc);
                return (ENXIO);
        }

        error = mbufq_enqueue(&sc->sc_snd, m);
        if (error != 0) {
                MT7615_UNLOCK(sc);
                return (error);
        }

        mt7615_start(sc);
        MT7615_UNLOCK(sc);

        return (0);
}

static int
mt7615_raw_xmit(struct ieee80211_node *ni, struct mbuf *m,
                const struct ieee80211_bpf_params *params)
{
        struct mt7615_softc *sc;
        int error;

        sc = ni->ni_ic->ic_softc;

        MT7615_LOCK(sc);
        if ((sc->sc_flags & MT7615_FLAG_HW_INITED) == 0) {
                MT7615_UNLOCK(sc);
                return (ENETDOWN);
        }

        error = mt7615_tx(sc, m, ni);
        MT7615_UNLOCK(sc);

        return (error);
}

/*
 * Bring the hardware up and take it down.
 */
static int
mt7615_init(struct mt7615_softc *sc)
{
        int error;

        MT7615_ASSERT_LOCKED(sc);

        if ((sc->sc_flags & MT7615_FLAG_HW_INITED) != 0)
                return (0);

        /*
         * The images were fetched at attach, where sleeping to load a
         * module was allowed; there is nothing to be done about a
         * missing set from here.
         */
        if ((sc->sc_flags & MT7615_FLAG_FW_LOADED) == 0) {
                device_printf(sc->sc_dev,
                    "no firmware; install wifi-firmware-mt76-kmod and "
                    "reload the driver\n");
                return (ENXIO);
        }

        error = mt7615_dma_init(sc);
        if (error != 0)
                return (error);

        /*
         * The MCU has to be able to answer before the download's own
         * commands are sent, so open the interrupt first.
         *
         * Only if there is a handler for it, though.  The chip drives a
         * level-triggered line, and the bridge has already mapped that
         * line into the interrupt controller even when the vector could
         * not be claimed, so unmasking a source nothing will service
         * leaves the line asserted with no way to clear it.  Keeping
         * every source masked costs nothing here: the command wait
         * polls the event ring anyway, which is what carries the
         * download when there is no interrupt to carry it.
         */
        if (sc->sc_ih != NULL) {
                mt7615_irq_enable(sc, MT_INT_RX_DONE_ALL |
                                      MT_INT_TX_DONE_ALL | MT_INT_MCU_CMD);
        } else {
                device_printf(sc->sc_dev, "no interrupt: the firmware will "
                                          "load, but the receive path stays dead\n");
        }

        error = mt7615_mcu_init(sc);
        if (error != 0)
                goto fail;

        error = mt7615_mac_init(sc);
        if (error != 0)
                goto fail;

        (void)mt7615_mcu_set_eeprom(sc);
        (void)mt7615_mcu_del_wtbl_all(sc);
        (void)mt7615_mcu_set_rts_thresh(sc, 0x92b);
        (void)mt7615_mcu_set_mac_enable(sc, 0, true);

        sc->sc_flags |= MT7615_FLAG_HW_INITED;
        callout_reset(&sc->sc_watchdog_to, hz, mt7615_watchdog, sc);

        return (0);

fail:
        mt7615_irq_disable(sc, ~0U);
        mt7615_dma_stop(sc);
        return (error);
}

static void
mt7615_stop(struct mt7615_softc *sc)
{

        MT7615_ASSERT_LOCKED(sc);

        sc->sc_flags &= ~MT7615_FLAG_HW_INITED;

        callout_stop(&sc->sc_watchdog_to);

        if ((sc->sc_flags & MT7615_FLAG_FW_RUNNING) != 0)
                (void)mt7615_mcu_set_mac_enable(sc, 0, false);

        mt7615_irq_disable(sc, ~0U);
        mt7615_mcu_exit(sc);
        mt7615_dma_stop(sc);
        mt7615_dma_reset_rings(sc);

        /* The station table is gone with the hardware; so is what it said. */
        mt7615_txs_flush(sc);

        mbufq_drain(&sc->sc_snd);
}

static void
mt7615_watchdog(void *arg)
{
        struct mt7615_softc *sc;

        sc = arg;

        MT7615_ASSERT_LOCKED(sc);

        /*
         * Nothing is stuck-detected yet; the callout exists so the Tx
         * rings are reclaimed even when the completion interrupt is
         * lost, which a busy ring otherwise papers over.
         */
        mt7615_tx_cleanup(sc, &sc->sc_txq[MT7615_TXQ_DATA]);
        mt7615_start(sc);

        /*
         * Hand the second's worth of transmit status reports over to
         * net80211.  Once a second is the interval a rate control
         * algorithm expects to be fed at, and it bounds the cost: the
         * reports themselves arrive far faster than any node lookup
         * should be done.
         */
        taskqueue_enqueue(sc->sc_tq, &sc->sc_txs_task);

        callout_reset(&sc->sc_watchdog_to, hz, mt7615_watchdog, sc);
}

/*
 * net80211 glue.
 */

static struct ieee80211vap *
mt7615_vap_create(struct ieee80211com *ic, const char name[IFNAMSIZ], int unit,
                  enum ieee80211_opmode opmode, int flags,
                  const uint8_t bssid[IEEE80211_ADDR_LEN],
                  const uint8_t mac[IEEE80211_ADDR_LEN])
{
        struct mt7615_vap *mvp;
        struct ieee80211vap *vap;

        /* One at a time: the driver has no multi-BSS support yet. */
        if (!TAILQ_EMPTY(&ic->ic_vaps))
                return (NULL);

        mvp = malloc(sizeof(*mvp), M_80211_VAP, M_WAITOK | M_ZERO);
        vap = &mvp->iv_vap;

        if (ieee80211_vap_setup(ic, vap, name, unit, opmode,
            flags | IEEE80211_CLONE_NOBEACONS, bssid) != 0) {
                free(mvp, M_80211_VAP);
                return (NULL);
        }

        mvp->omac_idx = 0;
        mvp->band_idx = 0;
        mvp->wmm_idx = 0;
        /*
         * The group-traffic slot is counted down from the top of the
         * table, one per network.  With a single network that is the
         * topmost slot; it must not be slot 0, which belongs to the
         * beacon.
         */
        mvp->bmc_wcid = MT7615_WTBL_RESERVED;

        /*
         * How much may go into one aggregate.  net80211 starts at eight
         * kilobytes, which is about five frames, and says in as many words
         * that a driver is meant to raise it; the lower of this and what
         * the peer offers is what applies, and a peer offers sixty-four.
         * The firmware is told the peer's own limit separately, so this is
         * only ever the ceiling on our side.
         */
        vap->iv_ampdu_rxmax = IEEE80211_HTCAP_MAXRXAMPDU_64K;
        vap->iv_ampdu_limit = vap->iv_ampdu_rxmax;

        mvp->iv_newstate = vap->iv_newstate;
        vap->iv_newstate = mt7615_newstate;
        vap->iv_update_beacon = mt7615_update_beacon;
        vap->iv_key_alloc = mt7615_key_alloc;
        vap->iv_key_set = mt7615_key_set;
        vap->iv_key_delete = mt7615_key_delete;

        ieee80211_ratectl_init(vap);

        ieee80211_vap_attach(vap, ieee80211_media_change,
            ieee80211_media_status, mac);
        ic->ic_opmode = opmode;

        return (vap);
}

static void
mt7615_vap_delete(struct ieee80211vap *vap)
{
        struct mt7615_softc *sc;
        struct mt7615_vap *mvp;
        struct mt7615_sta_pending *sp, *tmp;

        mvp = MT7615_VAP(vap);
        sc = vap->iv_ic->ic_softc;

        /*
         * Work still queued names this vap, so it has to be finished
         * before the vap goes.  The beacon is disowned first, or the
         * thread would go looking for a frame from a network that is on
         * its way out.
         */
        MT7615_LOCK(sc);
        if (sc->sc_bcn_vap == mvp)
                sc->sc_bcn_vap = NULL;
        MT7615_UNLOCK(sc);

        taskqueue_drain(sc->sc_tq, &sc->sc_sta_task);
        taskqueue_drain(sc->sc_tq, &sc->sc_bcn_task);
        taskqueue_drain(sc->sc_tq, &sc->sc_txs_task);

        MT7615_LOCK(sc);
        STAILQ_FOREACH_SAFE(sp, &sc->sc_sta_pending, next, tmp) {
                if (sp->mvp != mvp)
                        continue;
                STAILQ_REMOVE(&sc->sc_sta_pending, sp, mt7615_sta_pending,
                    next);
                free(sp, M_DEVBUF);
        }
        MT7615_UNLOCK(sc);

        ieee80211_ratectl_deinit(vap);
        ieee80211_vap_detach(vap);
        free(mvp, M_80211_VAP);
}

static int
mt7615_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg)
{
        struct mt7615_vap *mvp;
        struct mt7615_softc *sc;
        struct ieee80211com *ic;
        struct mt7615_peer bmc;
        int error;

        mvp = MT7615_VAP(vap);
        ic = vap->iv_ic;
        sc = ic->ic_softc;

        MT7615_DPRINTF(sc, MT7615_DEBUG_STATE, "%s -> %s\n",
            ieee80211_state_name[vap->iv_state], ieee80211_state_name[nstate]);

        /*
         * Tearing a network down has to reach the firmware while what it
         * describes is still current, so that runs before net80211 lets
         * go of the state.
         */
        if (nstate == IEEE80211_S_INIT && vap->iv_state == IEEE80211_S_RUN) {
                IEEE80211_UNLOCK(ic);
                MT7615_LOCK(sc);
                if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
                        sc->sc_bcn_vap = NULL;
                        (void)mt7615_mcu_add_beacon(sc, mvp, false);
                }
                (void)mt7615_mcu_add_bss(sc, mvp, false);
                (void)mt7615_mcu_add_dev(sc, mvp, false);
                MT7615_UNLOCK(sc);
                IEEE80211_LOCK(ic);
        }

        /*
         * Everything else runs after net80211 has moved, because the
         * hardware is being told about a state it has to finish entering
         * first: iv_bss is not a usable node until then, and a beacon
         * built from a half-finished one describes nothing.
         */
        error = mvp->iv_newstate(vap, nstate, arg);
        if (error != 0)
                return (error);

        IEEE80211_UNLOCK(ic);
        MT7615_LOCK(sc);

        switch (nstate) {
                case IEEE80211_S_SCAN:
                case IEEE80211_S_AUTH:
                case IEEE80211_S_ASSOC:
                        if (ic->ic_curchan != NULL)
                                (void)mt7615_mcu_set_channel(sc, ic->ic_curchan);
                        mt7615_mac_set_rxfilter(sc);
                        break;
                case IEEE80211_S_RUN:
                        if (ic->ic_curchan != NULL)
                                (void)mt7615_mcu_set_channel(sc, ic->ic_curchan);
                        mt7615_mac_set_rxfilter(sc);

                        /*
                         * Running means the network exists, so build it in the
                         * firmware: the hardware MAC slot, the network on it,
                         * and the broadcast pseudo-station every network needs
                         * for anything not addressed to one peer.  An access
                         * point then hands over the beacon and the firmware
                         * repeats it from there on.
                         */
                        error = mt7615_mcu_add_dev(sc, mvp, true);
                        if (error != 0) {
                                device_printf(sc->sc_dev,
                                    "could not create the device context: %d\n", error);
                                break;
                        }
                        error = mt7615_mcu_add_bss(sc, mvp, true);
                        if (error != 0) {
                                device_printf(sc->sc_dev,
                                    "could not create the BSS: %d\n", error);
                                break;
                        }
                        bmc = (struct mt7615_peer){
                            .addr = ieee80211broadcastaddr,
                            .wcid = mvp->bmc_wcid,
                        };
                        (void)mt7615_mcu_sta_add(sc, mvp, &bmc, true);

                        if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
                                sc->sc_bcn_vap = mvp;
                                error = mt7615_mcu_add_beacon(sc, mvp, true);
                                if (error != 0)
                                        device_printf(sc->sc_dev,
                                            "could not hand over the beacon: %d\n",
                                            error);
                        }
                        break;
                default:
                        break;
        }

        MT7615_UNLOCK(sc);
        IEEE80211_LOCK(ic);

        return (0);
}

/*
 * Let go of the driver's thread, and of anything it had left to do.
 */
static void
mt7615_sta_task_free(struct mt7615_softc *sc)
{
        struct mt7615_sta_pending *sp;
        struct mt7615_ba_pending *bp;

        if (sc->sc_tq == NULL)
                return;

        taskqueue_drain(sc->sc_tq, &sc->sc_sta_task);
        taskqueue_drain(sc->sc_tq, &sc->sc_bcn_task);
        taskqueue_drain(sc->sc_tq, &sc->sc_ba_task);
        taskqueue_drain(sc->sc_tq, &sc->sc_txs_task);
        taskqueue_free(sc->sc_tq);
        sc->sc_tq = NULL;

        while ((sp = STAILQ_FIRST(&sc->sc_sta_pending)) != NULL) {
                STAILQ_REMOVE_HEAD(&sc->sc_sta_pending, next);
                free(sp, M_DEVBUF);
        }
        while ((bp = STAILQ_FIRST(&sc->sc_ba_pending)) != NULL) {
                STAILQ_REMOVE_HEAD(&sc->sc_ba_pending, next);
                free(bp, M_DEVBUF);
        }
}

/*
 * Block-acknowledgement sessions.
 *
 * net80211 negotiates them: it decides when a traffic identifier is
 * busy enough to be worth aggregating, exchanges the request and the
 * response with the peer, and keeps the window afterwards.  All the
 * driver has to do is tell the firmware what was agreed, because
 * otherwise the transmit engine goes on asking for an acknowledgement
 * per frame and the peer goes on sending one, which is most of what
 * the air was being spent on.
 *
 * All four of these arrive from the receive path, so the firmware is
 * told from the driver's own thread rather than here.
 */
static void
mt7615_ba_queue(struct mt7615_softc *sc, struct ieee80211_node *ni,
                uint8_t tid, uint16_t ssn, uint16_t winsize, bool tx, bool enable)
{
        struct mt7615_vap *mvp;
        struct mt7615_ba_pending *bp;
        uint8_t wcid;

        mvp = MT7615_VAP(ni->ni_vap);
        if (ni->ni_associd == 0)
                return;
        wcid = IEEE80211_AID(ni->ni_associd);
        if (wcid < MT7615_WTBL_STA_FIRST || wcid >= MT7615_WTBL_STA_LIMIT)
                return;

        bp = malloc(sizeof(*bp), M_DEVBUF, M_NOWAIT | M_ZERO);
        if (bp == NULL) {
                device_printf(sc->sc_dev,
                    "no room to record the session for slot %u tid %u\n",
                    wcid, tid);
                return;
        }

        bp->mvp = mvp;
        IEEE80211_ADDR_COPY(bp->addr, ni->ni_macaddr);
        bp->wcid = wcid;
        bp->tid = tid;
        bp->ssn = ssn;
        bp->winsize = winsize;
        bp->tx = tx;
        bp->enable = enable;

        MT7615_LOCK(sc);
        STAILQ_INSERT_TAIL(&sc->sc_ba_pending, bp, next);
        MT7615_UNLOCK(sc);

        taskqueue_enqueue(sc->sc_tq, &sc->sc_ba_task);
}

static void
mt7615_ba_task(void *arg, int npending __unused)
{
        struct mt7615_softc *sc;
        struct mt7615_ba_pending *bp;
        struct mt7615_ba ba;
        int error;

        sc = arg;

        MT7615_LOCK(sc);
        while ((bp = STAILQ_FIRST(&sc->sc_ba_pending)) != NULL) {
                STAILQ_REMOVE_HEAD(&sc->sc_ba_pending, next);

                if ((sc->sc_flags & MT7615_FLAG_HW_INITED) != 0) {
                        ba = (struct mt7615_ba){
                            .addr = bp->addr,
                            .wcid = bp->wcid,
                            .tid = bp->tid,
                            .ssn = bp->ssn,
                            .winsize = bp->winsize,
                        };
                        error = mt7615_mcu_sta_ba(sc, bp->mvp, &ba, bp->enable,
                            bp->tx);
                        if (error != 0)
                                device_printf(sc->sc_dev,
                                    "could not %s the %s session for slot %u "
                                    "tid %u: %d\n",
                                    bp->enable ? "open" : "close",
                                    bp->tx ? "sending" : "receiving",
                                    bp->wcid, bp->tid, error);
                }

                free(bp, M_DEVBUF);
        }
        MT7615_UNLOCK(sc);
}

/*
 * The peer answered our request to send it aggregates.  net80211 has
 * already checked the answer; a refusal leaves the session closed and
 * there is nothing for the firmware to hear about.
 */
static int
mt7615_addba_response(struct ieee80211_node *ni,
                      struct ieee80211_tx_ampdu *tap, int status, int baparamset, int batimeout)
{
        struct mt7615_softc *sc;
        int ret;

        sc = ni->ni_ic->ic_softc;

        ret = sc->sc_addba_response(ni, tap, status, baparamset, batimeout);
        /*
         * The window is read back rather than taken from the answer,
         * because net80211 caps it at what its own bookkeeping can hold
         * and the firmware has to be told the same number.
         */
        if (status == IEEE80211_STATUS_SUCCESS)
                mt7615_ba_queue(sc, ni, tap->txa_tid, tap->txa_start,
                    tap->txa_wnd, true, true);

        return (ret);
}

static void
mt7615_addba_stop(struct ieee80211_node *ni, struct ieee80211_tx_ampdu *tap)
{
        struct mt7615_softc *sc;

        sc = ni->ni_ic->ic_softc;

        mt7615_ba_queue(sc, ni, tap->txa_tid, 0, 0, true, false);
        sc->sc_addba_stop(ni, tap);
}

/*
 * The peer asked to send us aggregates.  The traffic identifier is only
 * in the request; the window and its starting point are read back out
 * of the reorder state, so that the firmware and net80211 agree on the
 * numbers net80211 settled on rather than on the ones the peer asked
 * for.
 */
static int
mt7615_ampdu_rx_start(struct ieee80211_node *ni, struct ieee80211_rx_ampdu *rap,
                      int baparamset, int batimeout, int baseqctl)
{
        struct mt7615_softc *sc;
        uint8_t tid;
        int ret;

        sc = ni->ni_ic->ic_softc;

        ret = sc->sc_ampdu_rx_start(ni, rap, baparamset, batimeout, baseqctl);
        if (ret != 0)
                return (ret);

        tid = _IEEE80211_MASKSHIFT(baparamset, IEEE80211_BAPS_TID);
        mt7615_ba_queue(sc, ni, tid, rap->rxa_start, rap->rxa_wnd, false, true);

        return (ret);
}

static void
mt7615_ampdu_rx_stop(struct ieee80211_node *ni, struct ieee80211_rx_ampdu *rap)
{
        struct mt7615_softc *sc;
        int tid;

        sc = ni->ni_ic->ic_softc;

        /*
         * The reorder state does not carry its own identifier, so it is
         * recovered from where it sits in the node's array.
         */
        tid = rap - ni->ni_rx_ampdu;
        if (tid >= 0 && tid < WME_NUM_TID)
                mt7615_ba_queue(sc, ni, tid, 0, 0, false, false);

        sc->sc_ampdu_rx_stop(ni, rap);
}

/*
 * The beacon the firmware repeats has gone stale.
 *
 * net80211 says so whenever anything that belongs in the frame
 * changes, and the most consequential of those is hostapd installing
 * its own information elements: the RSN element that announces the
 * network is protected arrives this way, after the network is already
 * running.  A firmware still repeating the frame built at RUN
 * advertises an open network, so a peer joins expecting no
 * encryption, hostapd waits for a handshake that peer will never
 * start, and the two give up on each other.
 *
 * Rebuilding means allocating a frame and waiting for the firmware to
 * take it, and this is called from wherever the change happened -
 * including the receive path - so all that happens here is a nudge.
 */
static void
mt7615_update_beacon(struct ieee80211vap *vap, int item __unused)
{
        struct mt7615_softc *sc;

        sc = vap->iv_ic->ic_softc;
        taskqueue_enqueue(sc->sc_tq, &sc->sc_bcn_task);
}

static void
mt7615_beacon_task(void *arg, int npending __unused)
{
        struct mt7615_softc *sc;
        struct mt7615_vap *mvp;
        int error;

        sc = arg;

        MT7615_LOCK(sc);
        mvp = sc->sc_bcn_vap;
        if (mvp != NULL && (sc->sc_flags & MT7615_FLAG_HW_INITED) != 0 &&
            mvp->iv_vap.iv_state == IEEE80211_S_RUN) {
                error = mt7615_mcu_add_beacon(sc, mvp, true);
                if (error != 0)
                        device_printf(sc->sc_dev,
                            "could not hand over the new beacon: %d\n", error);
        }
        MT7615_UNLOCK(sc);
}

/*
 * Push the station records that have piled up.  Runs on the driver's
 * own thread, where waiting for the firmware to answer is allowed.
 */
static void
mt7615_sta_task(void *arg, int npending __unused)
{
        struct mt7615_softc *sc;
        struct mt7615_sta_pending *sp;
        struct mt7615_peer peer;
        int error;

        sc = arg;

        MT7615_LOCK(sc);
        while ((sp = STAILQ_FIRST(&sc->sc_sta_pending)) != NULL) {
                STAILQ_REMOVE_HEAD(&sc->sc_sta_pending, next);

                if ((sc->sc_flags & MT7615_FLAG_HW_INITED) != 0) {
                        peer = (struct mt7615_peer){
                            .addr = sp->addr,
                            .wcid = sp->wcid,
                            .aid = sp->aid,
                            .htcap = sp->htcap,
                            .htparam = sp->htparam,
                            .maxmcs = sp->maxmcs,
                            .ht40 = sp->ht40,
                            .bw80 = sp->bw80,
                            .vhtcap = sp->vhtcap,
                            .vht_rx_mcs = sp->vht_rx_mcs,
                            .vht_tx_mcs = sp->vht_tx_mcs,
                            .vht_nss = sp->vht_nss,
                            .qos = sp->qos,
                        };
                        error = mt7615_mcu_sta_add(sc, sp->mvp, &peer, true);
                        if (error != 0)
                                device_printf(sc->sc_dev,
                                    "could not add the station in slot %u: %d\n",
                                    sp->wcid, error);
                        else {
                                mt7615_mac_set_rates(sc, &peer);
                                mt7615_txs_claim(sc, sp->wcid, sp->addr);
                        }
                }

                free(sp, M_DEVBUF);
        }
        MT7615_UNLOCK(sc);
}

/*
 * Name the peer that a transmit-status slot belongs to.  The reports
 * arrive with nothing but a slot number, and the thread that reads them
 * has to get from there to a node; this is what makes that possible.
 *
 * The counters are cleared whenever the peer changes, so what one peer
 * did is never charged to the next one to take the slot.
 */
void
mt7615_txs_claim(struct mt7615_softc *sc, uint8_t wcid,
                 const uint8_t addr[IEEE80211_ADDR_LEN])
{
        struct mt7615_txs_stats *st;

        MT7615_ASSERT_LOCKED(sc);

        if (wcid < MT7615_WTBL_STA_FIRST || wcid >= MT7615_WTBL_STA_LIMIT)
                return;

        st = &sc->sc_txs[wcid];
        if (st->valid && IEEE80211_ADDR_EQ(st->addr, addr))
                return;

        memset(st, 0, sizeof(*st));
        IEEE80211_ADDR_COPY(st->addr, addr);
        st->valid = true;
}

/* Every slot, for when the network goes down and the table means nothing. */
void
mt7615_txs_flush(struct mt7615_softc *sc)
{

        MT7615_ASSERT_LOCKED(sc);
        memset(sc->sc_txs, 0, sizeof(sc->sc_txs));
        sc->sc_txs_sample = 0;
}

/*
 * Turn a rate as the hardware names it into one net80211 understands and
 * hang it on the node, which is where "ifconfig list sta" reads it from.
 *
 * The hardware numbers the legacy rates its own way rather than by the
 * standard's, so those two need a table; the modulated ones carry their
 * index directly.
 */
static void
mt7615_txs_node_rate(struct ieee80211_node *ni, uint16_t rate)
{
        /* 1, 2, 5.5 and 11 Mbit/s, in the half-megabit the standard uses. */
        static const uint8_t cck[4] = { 2, 4, 11, 22 };
        /* Hardware indices eight through fifteen, which are not in order. */
        static const uint8_t ofdm[8] = { 96, 48, 24, 12, 108, 72, 36, 18 };
        uint8_t idx, nss;

        idx = rate & MT_TX_RATE_IDX;

        switch ((rate & MT_TX_RATE_MODE) >> MT_TX_RATE_MODE_S) {
                case MT_PHY_TYPE_CCK:
                        /* Four rates named twice over: long preamble, then short. */
                        ieee80211_node_set_txrate_dot11rate(ni, cck[idx & 3]);
                        break;
                case MT_PHY_TYPE_OFDM:
                        if (idx < 8 || idx > 15)
                                return;
                        ieee80211_node_set_txrate_dot11rate(ni, ofdm[idx - 8]);
                        break;
                case MT_PHY_TYPE_HT:
                case MT_PHY_TYPE_HT_GF:
                        if (idx > 31)
                                return;
                        ieee80211_node_set_txrate_ht_mcsrate(ni, idx);
                        break;
                case MT_PHY_TYPE_VHT:
                        if (idx > 9)
                                return;
                        /*
                         * The field holds one less than the number of streams.
                         * Space-time coding spreads one stream over two, so the
                         * count it reports is one higher than the data carries.
                         */
                        nss = ((rate & MT_TX_RATE_NSS) >> MT_TX_RATE_NSS_S) + 1;
                        if ((rate & MT_TX_RATE_STBC) != 0 && nss > 1)
                                nss--;
                        ieee80211_node_set_txrate_vht_rate(ni, nss, idx);
                        break;
                default:
                        break;
        }
}

/*
 * Carry what the transmit status reports have said over to net80211.
 *
 * Runs on the driver's own thread because finding a node takes a
 * net80211 lock, and net80211 holds that one across calls into this
 * driver - taking it from under the driver lock would invert the order.
 * So each slot is copied and emptied with the driver lock held, and
 * everything after that is done without it.
 */
static void
mt7615_txs_task(void *arg, int npending __unused)
{
        struct mt7615_softc *sc;
        struct ieee80211com *ic;
        struct ieee80211_node *ni;
        struct ieee80211_ratectl_tx_stats stats;
        struct mt7615_txs_stats snap;
        int wcid;

        sc = arg;
        ic = &sc->sc_ic;

        for (wcid = MT7615_WTBL_STA_FIRST; wcid < MT7615_WTBL_STA_LIMIT;
             wcid++) {
                MT7615_LOCK(sc);
                snap = sc->sc_txs[wcid];
                if (snap.valid && snap.nframes != 0) {
                        sc->sc_txs[wcid].nframes = 0;
                        sc->sc_txs[wcid].nsuccess = 0;
                        sc->sc_txs[wcid].nretries = 0;
                }
                MT7615_UNLOCK(sc);

                if (!snap.valid || snap.nframes == 0)
                        continue;

                ni = ieee80211_find_node(&ic->ic_sta, snap.addr);
                if (ni == NULL)
                        continue;

                MT7615_DPRINTF(sc, MT7615_DEBUG_TX,
                    "slot %d: rate %#x, %u frame%s, %u acked, %u retries\n",
                    wcid, snap.rate, snap.nframes,
                    snap.nframes == 1 ? "" : "s", snap.nsuccess,
                    snap.nretries);

                if (snap.rate != 0)
                        mt7615_txs_node_rate(ni, snap.rate);

                /*
                 * What a rate control algorithm wants is the shape of an
                 * interval rather than one frame at a time, which is also
                 * the only shape available here: the reports are sampled,
                 * so they stand for the traffic rather than account for it.
                 */
                memset(&stats, 0, sizeof(stats));
                stats.flags = IEEE80211_RATECTL_TX_STATS_NODE |
                              IEEE80211_RATECTL_TX_STATS_RETRIES;
                stats.ni = ni;
                stats.nframes = snap.nframes;
                stats.nsuccess = snap.nsuccess;
                stats.nretries = snap.nretries;
                ieee80211_ratectl_tx_update(ni->ni_vap, &stats);

                ieee80211_free_node(ni);
        }
}

/*
 * A peer has associated.  It needs a slot in the hardware table and a
 * station record before the firmware will carry its traffic; slot 0 is
 * reserved for the beacon, so peers start above it.
 *
 * This is called from the receive path, which runs in an interrupt
 * thread inside the network epoch and so may not sleep.  Telling the
 * firmware means waiting for it to answer, so all that happens here is
 * that the record is copied and a thread is woken to send it.
 */
/*
 * Keys.
 *
 * net80211 decides between hardware and software from ic_cryptocaps,
 * so only the ciphers named there arrive here; everything else it does
 * itself.  What is left is to say which slot a key takes and to write
 * it into the peer's table entry.
 *
 * A key never carries the encryption itself into this driver: net80211
 * still builds the cipher header and leaves the body alone, and the
 * hardware encrypts on the way out.  That is the whole point - the AES
 * stops running on the processor, which at these rates it has no
 * business doing.
 */
static int
mt7615_key_alloc(struct ieee80211vap *vap, struct ieee80211_key *k,
                 ieee80211_keyix *keyix, ieee80211_keyix *rxkeyix)
{

        /*
         * A group key is one of the four the network holds and keeps the
         * index it already has.  A peer's own key sits in that peer's
         * entry, where there is only ever one, so it takes the first.
         */
        if (&vap->iv_nw_keys[0] <= k &&
            k < &vap->iv_nw_keys[IEEE80211_WEP_NKID])
                *keyix = k - vap->iv_nw_keys;
        else
                *keyix = 0;
        *rxkeyix = *keyix;

        return (1);
}

/*
 * Which slot in the hardware table a key belongs to: the peer's own for
 * a pairwise key, the network's group slot for a group key.
 */
static int
mt7615_key_wcid(struct ieee80211vap *vap, const struct ieee80211_key *k,
                uint8_t *wcid)
{
        struct mt7615_vap *mvp;
        struct ieee80211_node *ni;
        uint8_t idx;

        mvp = MT7615_VAP(vap);

        if ((k->wk_flags & IEEE80211_KEY_GROUP) != 0 ||
            IEEE80211_IS_MULTICAST(k->wk_macaddr)) {
                *wcid = mvp->bmc_wcid;
                return (0);
        }

        ni = ieee80211_find_vap_node(&vap->iv_ic->ic_sta, vap, k->wk_macaddr);
        if (ni == NULL)
                return (ENOENT);
        idx = IEEE80211_AID(ni->ni_associd);
        ieee80211_free_node(ni);

        if (idx < MT7615_WTBL_STA_FIRST || idx >= MT7615_WTBL_STA_LIMIT)
                return (ERANGE);

        *wcid = idx;

        return (0);
}

static int
mt7615_key_set(struct ieee80211vap *vap, const struct ieee80211_key *k)
{
        struct mt7615_softc *sc;
        uint8_t cipher, wcid;
        int error;

        sc = vap->iv_ic->ic_softc;

        switch (k->wk_cipher->ic_cipher) {
                case IEEE80211_CIPHER_AES_CCM:
                        cipher = MT_CIPHER_AES_CCMP;
                        break;
                default:
                        /*
                         * Not one this was asked to take.  Saying so is not the
                         * same as failing: net80211 keeps the key and does the
                         * cipher itself.
                         */
                        device_printf(sc->sc_dev,
                            "cipher %u stays in software, for %6D\n",
                            k->wk_cipher->ic_cipher, k->wk_macaddr, ":");
                        return (1);
        }

        error = mt7615_key_wcid(vap, k, &wcid);
        if (error != 0) {
                device_printf(sc->sc_dev,
                    "no station table slot (%d) for the key for %6D\n",
                    error, k->wk_macaddr, ":");
                return (0);
        }

        MT7615_LOCK(sc);
        error = mt7615_mac_set_key(sc, wcid, cipher, k->wk_keyix & 3,
            k->wk_key, k->wk_keylen);
        MT7615_UNLOCK(sc);
        if (error != 0) {
                device_printf(sc->sc_dev,
                    "could not install the key in slot %u: %d\n", wcid, error);
                return (0);
        }

        device_printf(sc->sc_dev,
            "key %u, slot %u, %s, %u bytes, for %6D\n", k->wk_keyix, wcid,
            (k->wk_flags & IEEE80211_KEY_GROUP) != 0 ? "group" : "pairwise",
            k->wk_keylen, k->wk_macaddr, ":");

        return (1);
}

static int
mt7615_key_delete(struct ieee80211vap *vap, const struct ieee80211_key *k)
{
        struct mt7615_softc *sc;
        uint8_t wcid;
        int error;

        sc = vap->iv_ic->ic_softc;

        /*
         * A peer that has already gone takes its table entry with it, so
         * there is nothing left to clear and nothing wrong with that.
         */
        if (mt7615_key_wcid(vap, k, &wcid) != 0)
                return (1);

        MT7615_LOCK(sc);
        error = mt7615_mac_set_key(sc, wcid, MT_CIPHER_NONE, k->wk_keyix & 3,
            NULL, 0);
        MT7615_UNLOCK(sc);

        return (error == 0);
}

static void
mt7615_newassoc(struct ieee80211_node *ni, int isnew)
{
        struct ieee80211vap *vap;
        struct mt7615_softc *sc;
        struct mt7615_vap *mvp;
        struct mt7615_sta_pending *sp;
        uint8_t wcid;
        bool qos;

        vap = ni->ni_vap;
        sc = ni->ni_ic->ic_softc;
        mvp = MT7615_VAP(vap);

        if (!isnew)
                return;

        /*
         * net80211 hands out association IDs from 1, which maps onto the
         * table directly.  A station has only ever one peer, so it takes
         * the first slot regardless of what the access point called it.
         */
        if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
                if (ni->ni_associd == 0)
                        return;
                wcid = IEEE80211_AID(ni->ni_associd);
                if (wcid < MT7615_WTBL_STA_FIRST ||
                    wcid >= MT7615_WTBL_STA_LIMIT) {
                        device_printf(sc->sc_dev,
                            "no room in the station table for aid %u\n",
                            IEEE80211_AID(ni->ni_associd));
                        return;
                }
        } else {
                wcid = MT7615_WTBL_STA_FIRST;
        }

        qos = (ni->ni_flags & IEEE80211_NODE_QOS) != 0;

        /*
         * M_NOWAIT because of the epoch.  A peer whose record cannot be
         * parked here does not get one; it will try to associate again.
         */
        sp = malloc(sizeof(*sp), M_DEVBUF, M_NOWAIT | M_ZERO);
        if (sp == NULL) {
                device_printf(sc->sc_dev,
                    "no room to record the station in slot %u\n", wcid);
                return;
        }

        sp->mvp = mvp;
        IEEE80211_ADDR_COPY(sp->addr, ni->ni_macaddr);
        sp->wcid = wcid;
        sp->aid = IEEE80211_AID(ni->ni_associd);
        sp->qos = qos;
        /*
         * The firmware has to be told the peer does HT and what
         * aggregation limits it asked for, or it sends bursts the peer
         * cannot take.  A peer that did not negotiate HT leaves this at
         * zero, which is what says so.
         */
        if ((ni->ni_flags & IEEE80211_NODE_HT) != 0) {
                int i;

                sp->htcap = ni->ni_htcap;
                sp->htparam = ni->ni_htparam;
                /*
                 * The width this peer settled on, which is not the width
                 * of the channel: a peer that cannot take forty megahertz,
                 * or would not have it, associates at twenty on a channel
                 * that is forty wide, and sending to it any wider than it
                 * agreed to means it hears nothing.
                 */
                sp->ht40 = ni->ni_chw == NET80211_STA_RX_BW_40;

                /*
                 * The highest index the two sides agreed on, which is what
                 * the top of the peer's rate ladder is built from.  The set
                 * is in ascending order, and indices above thirty-one name
                 * unequal modulations across the streams, which nothing here
                 * sends.
                 */
                for (i = 0; i < ni->ni_htrates.rs_nrates; i++) {
                        if (ni->ni_htrates.rs_rates[i] > 31)
                                break;
                        sp->maxmcs = ni->ni_htrates.rs_rates[i];
                }
        }

        /*
         * 802.11ac, where the rate on five gigahertz actually is: eighty
         * megahertz and 256-QAM carry four times what a twenty megahertz
         * 802.11n channel does on the same chains.
         *
         * How many streams the peer will receive is two bits per stream in
         * its own map, counting up until one of them says the stream is
         * not supported.
         */
        if ((ni->ni_flags & IEEE80211_NODE_VHT) != 0 && ni->ni_vhtcap != 0) {
                uint16_t map;
                int i;

                sp->vhtcap = ni->ni_vhtcap;
                sp->vht_rx_mcs = ni->ni_vht_mcsinfo.rx_mcs_map;
                sp->vht_tx_mcs = ni->ni_vht_mcsinfo.tx_mcs_map;
                sp->bw80 = ni->ni_vht_chanwidth >= IEEE80211_VHT_CHANWIDTH_80MHZ;

                map = ni->ni_vht_mcsinfo.rx_mcs_map;
                for (i = 0; i < 8; i++) {
                        if (((map >> (i * 2)) & 0x3) ==
                            IEEE80211_VHT_MCS_NOT_SUPPORTED)
                                break;
                        sp->vht_nss = i + 1;
                }
                if (sp->vht_nss == 0)
                        sp->vhtcap = 0;
        }

        MT7615_LOCK(sc);
        STAILQ_INSERT_TAIL(&sc->sc_sta_pending, sp, next);
        MT7615_UNLOCK(sc);

        taskqueue_enqueue(sc->sc_tq, &sc->sc_sta_task);
}

/*
 * Push the channel access parameters down.  net80211 calls this
 * unconditionally once a network with WME is running - there is no
 * null check on its side - and a host access point sets them up as it
 * starts, so it has to be here whether or not the values are used.
 */
static int
mt7615_wme_update(struct ieee80211com *ic)
{
        struct mt7615_softc *sc;
        struct chanAccParams cap;
        int ac, error;

        sc = ic->ic_softc;

        ieee80211_wme_ic_getparams(ic, &cap);

        error = 0;
        MT7615_LOCK(sc);
        if ((sc->sc_flags & MT7615_FLAG_HW_INITED) != 0) {
                for (ac = 0; ac < WME_NUM_AC; ac++) {
                        error = mt7615_mcu_set_wmm(sc, ac,
                            &cap.cap_wmeParams[ac]);
                        if (error != 0)
                                break;
                }
        }
        MT7615_UNLOCK(sc);

        return (error);
}

static void
mt7615_parent(struct ieee80211com *ic)
{
        struct mt7615_softc *sc;
        int error, start;

        sc = ic->ic_softc;
        start = 0;

        MT7615_LOCK(sc);
        if (ic->ic_nrunning > 0) {
                error = mt7615_init(sc);
                if (error == 0)
                        start = 1;
                else
                        device_printf(sc->sc_dev,
                            "could not bring the hardware up: %d\n", error);
        } else {
                mt7615_stop(sc);
        }
        MT7615_UNLOCK(sc);

        if (start)
                ieee80211_start_all(ic);
}

static void
mt7615_scan_start(struct ieee80211com *ic)
{
        struct mt7615_softc *sc;

        sc = ic->ic_softc;

        MT7615_LOCK(sc);
        sc->sc_flags |= MT7615_FLAG_SCANNING;
        if ((sc->sc_flags & MT7615_FLAG_HW_INITED) != 0)
                mt7615_mac_set_rxfilter(sc);
        MT7615_UNLOCK(sc);
}

static void
mt7615_scan_end(struct ieee80211com *ic)
{
        struct mt7615_softc *sc;

        sc = ic->ic_softc;

        MT7615_LOCK(sc);
        sc->sc_flags &= ~MT7615_FLAG_SCANNING;
        if ((sc->sc_flags & MT7615_FLAG_HW_INITED) != 0)
                mt7615_mac_set_rxfilter(sc);
        MT7615_UNLOCK(sc);
}

static void
mt7615_set_channel(struct ieee80211com *ic)
{
        struct mt7615_softc *sc;

        sc = ic->ic_softc;

        /*
         * Never key up on a channel shared with radar.  Nothing should be
         * able to get one this far - they are not offered and a list
         * containing one is refused - but this is the last place the
         * transmitter can still be held back, and the cost of being wrong
         * here is transmitting over somebody's radar.
         */
        if (ic->ic_curchan != NULL && IEEE80211_IS_CHAN_DFS(ic->ic_curchan) &&
            (ic->ic_caps & IEEE80211_C_DFS) == 0) {
                device_printf(sc->sc_dev,
                    "refusing to transmit on channel %u: radar channel, and "
                    "this driver cannot listen for radar\n",
                    ieee80211_chan2ieee(ic, ic->ic_curchan));
                return;
        }

        MT7615_LOCK(sc);
        if ((sc->sc_flags & MT7615_FLAG_HW_INITED) != 0) {
                int error;

                error = mt7615_mcu_set_channel(sc, ic->ic_curchan);
                if (error != 0)
                        device_printf(sc->sc_dev,
                            "could not move to channel %u: %d\n",
                            ieee80211_chan2ieee(ic, ic->ic_curchan), error);
                /*
                 * The switch leaves the interframe spacing at reset values,
                 * so it has to be put back whether the move succeeded or
                 * not.
                 */
                mt7615_mac_set_timing(sc);
        }
        MT7615_UNLOCK(sc);
}

static void
mt7615_update_mcast(struct ieee80211com *ic)
{
        struct mt7615_softc *sc;

        sc = ic->ic_softc;

        MT7615_LOCK(sc);
        if ((sc->sc_flags & MT7615_FLAG_HW_INITED) != 0)
                mt7615_mac_set_rxfilter(sc);
        MT7615_UNLOCK(sc);
}

/*
 * Promiscuous mode is the same job: the receive filter already reads
 * ic_promisc, so it only has to be written out again.  Without this
 * net80211 has nowhere to say the mode changed, and a capture on the
 * interface quietly sees no more than usual.
 */
static void
mt7615_update_promisc(struct ieee80211com *ic)
{
        struct mt7615_softc *sc;

        sc = ic->ic_softc;

        MT7615_LOCK(sc);
        if ((sc->sc_flags & MT7615_FLAG_HW_INITED) != 0)
                mt7615_mac_set_rxfilter(sc);
        MT7615_UNLOCK(sc);
}

/*
 * The regulatory limits come from the firmware once it is running; for
 * now advertise the bands the radio covers and let net80211's own
 * regulatory database do the trimming.
 */
/*
 * Which modulations each stream may carry.  Two bits a stream, the same
 * answer for all the chains the board has and "not supported" for the
 * rest, since nothing here treats one chain differently from another.
 */
static void
mt7615_set_vht_mcs(struct ieee80211com *ic)
{
        uint32_t rx, tx;
        int i;

        rx = tx = 0;
        for (i = 0; i < 8; i++) {
                rx |= (i < ic->ic_rxstream ? IEEE80211_VHT_MCS_SUPPORT_0_9 :
                       IEEE80211_VHT_MCS_NOT_SUPPORTED) << (i * 2);
                tx |= (i < ic->ic_txstream ? IEEE80211_VHT_MCS_SUPPORT_0_9 :
                       IEEE80211_VHT_MCS_NOT_SUPPORTED) << (i * 2);
        }

        ic->ic_vht_cap.supp_mcs.rx_mcs_map = rx;
        ic->ic_vht_cap.supp_mcs.rx_highest = 0;
        ic->ic_vht_cap.supp_mcs.tx_mcs_map = tx;
        ic->ic_vht_cap.supp_mcs.tx_highest = 0;
}

static void
mt7615_getradiocaps(struct ieee80211com *ic, int maxchans, int *nchans,
                    struct ieee80211_channel chans[])
{
        static const uint8_t chan_2ghz[] = {
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
        };
        /*
         * Five gigahertz, less everything that needs radar detection.
         *
         * Channels 52 to 64 and 100 to 140 are shared with radar - weather,
         * air traffic and military - and every regulator that allows them
         * at all allows them only to a device that listens for radar and
         * vacates the channel when it hears one.  This driver does not
         * listen: there is no radar detection anywhere in it, and
         * IEEE80211_C_DFS is not among its capabilities.
         *
         * A device that cannot do that has no business offering those
         * channels, and offering them is not harmless.  Nothing downstream
         * will catch it - the regulatory database hands them back marked
         * IEEE80211_CHAN_DFS, and net80211 only ever reads that flag to
         * decide what to put in a channel-switch announcement; no part of
         * it refuses a DFS channel to a driver without the capability.  So
         * the list is the only gate there is, and an access point left on
         * one of these transmits over radar for as long as it is up.
         *
         * They come back when radar detection does, and not before.
         */
        static const uint8_t chan_5ghz[] = {
            36, 40, 44, 48,
            149, 153, 157, 161, 165
        };
        struct mt7615_softc *sc;
        uint8_t bands[IEEE80211_MODE_BYTES];
        int cbw, i;

        sc = ic->ic_softc;
        *nchans = 0;

        memset(bands, 0, sizeof(bands));
        setbit(bands, IEEE80211_MODE_11B);
        setbit(bands, IEEE80211_MODE_11G);
        setbit(bands, IEEE80211_MODE_11NG);
        /*
         * The last argument is which channel widths to build entries for,
         * and it is not a hint: a width left out of it has no channels at
         * all, so asking for one afterwards fails with the channel
         * reported as undefined.  Forty megahertz on both bands, and
         * eighty on five where 802.11ac lives.
         */
        ieee80211_add_channel_list_2ghz(chans, maxchans, nchans, chan_2ghz,
            nitems(chan_2ghz), bands, NET80211_CBW_FLAG_HT40);

        memset(bands, 0, sizeof(bands));
        setbit(bands, IEEE80211_MODE_11A);
        setbit(bands, IEEE80211_MODE_11NA);
        setbit(bands, IEEE80211_MODE_VHT_5GHZ);
        /*
         * Eighty megahertz is offered unless it is turned off.  It was the
         * other way round while a wide channel took the radio down with
         * it; that turned out to be the receive chain being left at twenty
         * megahertz behind it, and once told, the radio holds a wide
         * channel.  The switch stays for a board where it does not.
         */
        cbw = NET80211_CBW_FLAG_HT40;
        if (sc->sc_vht80 != 0)
                cbw |= NET80211_CBW_FLAG_VHT80;
        ieee80211_add_channel_list_5ghz(chans, maxchans, nchans, chan_5ghz,
            nitems(chan_5ghz), bands, cbw);

        /*
         * The list helpers leave every channel at zero regulatory power,
         * and zero is a limit rather than an absence of one: it is what
         * the radio is then told it may use.  Nothing else fills these in
         * for a driver that builds its own list, so give them a ceiling
         * that holds in every domain this is likely to run in.  Whatever
         * ifconfig is told still applies on top, since the lower of the
         * two is what reaches the hardware.
         */
        for (i = 0; i < *nchans; i++) {
                chans[i].ic_maxregpower = MT7615_MAX_REG_POWER;
                chans[i].ic_maxpower = 2 * MT7615_MAX_REG_POWER;
                chans[i].ic_minpower = 0;
        }
}

/*
 * A regulatory domain being set.
 *
 * The channel list arrives from userland, where it was worked out from
 * the regulatory database, and net80211 hands it down having checked
 * only that the frequencies and power limits make sense.  It does not
 * check it against what this driver said it could do, and it has no
 * opinion about radar: a list naming channels 52 to 140 is accepted and
 * committed whether or not anything is able to listen for radar on them.
 * The database will name them, too - they are legal in this country, to
 * a device that does radar detection.
 *
 * This driver does not, so this is where that list is refused.  Setting
 * a country whose rules allow more than the hardware can honour should
 * fail loudly rather than quietly widen what the radio will transmit on.
 */
static int
mt7615_setregdomain(struct ieee80211com *ic, struct ieee80211_regdomain *rd,
                    int nchans, struct ieee80211_channel chans[])
{
        struct mt7615_softc *sc;
        int i;

        sc = ic->ic_softc;

        if ((ic->ic_caps & IEEE80211_C_DFS) != 0)
                return (0);

        for (i = 0; i < nchans; i++) {
                if (!IEEE80211_IS_CHAN_DFS(&chans[i]))
                        continue;
                device_printf(sc->sc_dev,
                    "refusing channel %u: it is shared with radar and this "
                    "driver cannot listen for it\n",
                    ieee80211_mhz2ieee(chans[i].ic_freq, chans[i].ic_flags));
                return (EINVAL);
        }

        return (0);
}

static void
mt7615_radiotap_attach(struct mt7615_softc *sc)
{

        ieee80211_radiotap_attach(&sc->sc_ic,
            &sc->sc_txtap.wt_ihdr, sizeof(sc->sc_txtap),
            MT7615_TX_RADIOTAP_PRESENT,
            &sc->sc_rxtap.wr_ihdr, sizeof(sc->sc_rxtap),
            MT7615_RX_RADIOTAP_PRESENT);
}

/*
 * Firmware fetch.
 *
 * firmware(9) looks the images up in /boot/firmware, so it needs the
 * root filesystem: try_binary_file() goes straight to vn_open() with
 * nothing to fall back on if the root vnode is not there yet.  A
 * driver compiled into the kernel attaches long before that, and the
 * interrupt hooks are no help - SI_SUB_INT_CONFIG_HOOKS runs ahead of
 * SI_SUB_ROOT_CONF - so the only safe moment is the root mount itself.
 *
 * A driver loaded later has missed that event and can simply fetch the
 * images on the spot.  Either way this runs in a sleepable context
 * with no lock held, which firmware(9) requires.
 *
 * The images are kept for the life of the driver and released in
 * detach, so bringing the interface down and back up does not have to
 * fetch them again.  Failure is not fatal: the interface still
 * appears, and the attempt to bring it up says why it cannot.
 */
static void
mt7615_firmware_fetch(struct mt7615_softc *sc)
{

        if (mt7615_load_firmware(sc) != 0)
                device_printf(sc->sc_dev,
                    "continuing without firmware; the interface will not "
                    "come up until it is installed\n");
}

static void
mt7615_mountroot(void *arg)
{
        struct mt7615_softc *sc;

        sc = arg;

        /*
         * NOWAIT, because this runs from inside the event's own
         * invocation: EVENTHANDLER_INVOKE() holds the list's runcount
         * raised across the callbacks, so the waiting form would mark
         * this entry dead and then sleep for a deadcount that only the
         * invocation can clear - and the invocation is this callback.
         * That deadlocks vfs_mountroot() outright.  Marking the entry
         * dead is enough; the invocation prunes it on the way out.
         */
        EVENTHANDLER_DEREGISTER_NOWAIT(mountroot, sc->sc_mountroot_eh);
        sc->sc_mountroot_eh = NULL;

        mt7615_firmware_fetch(sc);
}

static void
mt7615_firmware_schedule(struct mt7615_softc *sc)
{

        if (root_mounted()) {
                mt7615_firmware_fetch(sc);
                return;
        }

        sc->sc_mountroot_eh = EVENTHANDLER_REGISTER(mountroot,
            mt7615_mountroot, sc, EVENTHANDLER_PRI_ANY);
        if (sc->sc_mountroot_eh == NULL)
                device_printf(sc->sc_dev,
                    "could not defer the firmware load to the root mount\n");
}

/*
 * Deferred attach.  net80211 needs nothing from the filesystem, so it
 * is brought up from the interrupt hook as usual; only the firmware
 * has to wait for the root mount.
 */
static void
mt7615_attach_hook(void *arg)
{
        struct mt7615_softc *sc;
        struct ieee80211com *ic;

        sc = arg;
        ic = &sc->sc_ic;

        ieee80211_ifattach(ic);

        ic->ic_vap_create = mt7615_vap_create;
        ic->ic_vap_delete = mt7615_vap_delete;
        ic->ic_parent = mt7615_parent;
        ic->ic_transmit = mt7615_transmit;
        ic->ic_raw_xmit = mt7615_raw_xmit;
        ic->ic_scan_start = mt7615_scan_start;
        ic->ic_scan_end = mt7615_scan_end;
        ic->ic_set_channel = mt7615_set_channel;
        ic->ic_update_mcast = mt7615_update_mcast;
        ic->ic_update_promisc = mt7615_update_promisc;
        ic->ic_getradiocaps = mt7615_getradiocaps;
        ic->ic_setregdomain = mt7615_setregdomain;
        ic->ic_newassoc = mt7615_newassoc;
        ic->ic_wme.wme_update = mt7615_wme_update;

        /*
         * net80211 runs the block-ack sessions; the driver only passes on
         * what was agreed, so its handlers sit in front of the ones
         * ieee80211_ifattach() just installed rather than in place of
         * them.
         */
        sc->sc_addba_response = ic->ic_addba_response;
        sc->sc_addba_stop = ic->ic_addba_stop;
        sc->sc_ampdu_rx_start = ic->ic_ampdu_rx_start;
        sc->sc_ampdu_rx_stop = ic->ic_ampdu_rx_stop;
        ic->ic_addba_response = mt7615_addba_response;
        ic->ic_addba_stop = mt7615_addba_stop;
        ic->ic_ampdu_rx_start = mt7615_ampdu_rx_start;
        ic->ic_ampdu_rx_stop = mt7615_ampdu_rx_stop;

        mt7615_radiotap_attach(sc);

        sc->sc_flags |= MT7615_FLAG_ATTACHED;

        ieee80211_announce(ic);

        config_intrhook_disestablish(&sc->sc_preinit_hook);
}

/*
 * Bus glue.
 */

static int
mt7615_probe(device_t dev)
{

        if (pci_get_vendor(dev) != MT7615_VENDOR_MEDIATEK ||
            pci_get_device(dev) != MT7615_DEVICE_MT7615E)
                return (ENXIO);

        device_set_desc(dev, "MediaTek MT7615E");

        return (BUS_PROBE_DEFAULT);
}

static int	mt7615_sysctl_aggr(SYSCTL_HANDLER_ARGS);

static void
mt7615_add_sysctls(struct mt7615_softc *sc)
{
        struct sysctl_ctx_list *ctx;
        struct sysctl_oid_list *tree;

        ctx = device_get_sysctl_ctx(sc->sc_dev);
        tree = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->sc_dev));

        SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "debug", CTLFLAG_RWTUN,
            &sc->sc_debug, 0, "Bitmask controlling debug output");
        SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "vht80", CTLFLAG_RD,
            &sc->sc_vht80, 0,
            "Eighty megahertz channels offered (hw.mt7615.vht80=0 at load "
            "turns them off)");
        SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "chipid", CTLFLAG_RD,
            &sc->sc_chipid, 0, "Chip ID from MT_HW_CHIPID");
        SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "hwrev", CTLFLAG_RD,
            &sc->sc_hwrev, 0, "Revision from MT_HW_REV");
        SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "flags", CTLFLAG_RD,
            &sc->sc_flags, 0, "Driver state flags");
        SYSCTL_ADD_ULONG(ctx, tree, OID_AUTO, "interrupts", CTLFLAG_RD,
            &sc->sc_intr_count, "Interrupts taken");
        SYSCTL_ADD_STRING(ctx, tree, OID_AUTO, "firmware_version", CTLFLAG_RD,
            sc->sc_fwver, 0, "Running firmware version");
        SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "txqueued", CTLFLAG_RD,
            &sc->sc_txq[MT7615_TXQ_DATA].queued, 0,
            "Descriptors outstanding on the data Tx ring");
        SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "tokens", CTLFLAG_RD,
            &sc->sc_token_used, 0, "Frames the firmware has not returned");
        SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "txs_div", CTLFLAG_RW,
            &sc->sc_txs_div, 0,
            "Ask for a transmit status report on every Nth frame (0 = never)");
        SYSCTL_ADD_ULONG(ctx, tree, OID_AUTO, "txs_events", CTLFLAG_RD,
            &sc->sc_txs_events, "Transmit status reports taken apart");
        SYSCTL_ADD_ULONG(ctx, tree, OID_AUTO, "txs_dropped", CTLFLAG_RD,
            &sc->sc_txs_dropped,
            "Transmit status reports naming a slot with no peer in it");
        SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "aggr",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            mt7615_sysctl_aggr, "A",
            "Transmit aggregate sizes and the rate they are acknowledged at");
}

/*
 * What the transmit side is actually managing to do.
 *
 * The histogram says how many frames each aggregate carried, in the
 * hardware's own buckets, and the two counters beside it say how many
 * frames went out inside aggregates and how many of those came back
 * acknowledged.  Aggregates of one, or a delivery rate well short of
 * everything, is what a link that is not living up to its rate looks
 * like from here; the counters clear as they are read, so each call
 * covers the time since the last one.
 */
static int
mt7615_sysctl_aggr(SYSCTL_HANDLER_ARGS)
{
        struct mt7615_softc *sc;
        struct sbuf sb;
        uint32_t val, sent, acked, agg[MT7615_AGG_BUCKETS];
        int error, i;

        sc = arg1;

        MT7615_LOCK(sc);
        if ((sc->sc_flags & MT7615_FLAG_HW_INITED) == 0) {
                MT7615_UNLOCK(sc);
                return (ENXIO);
        }
        sent = mt7615_rr(sc, MT_MIB_SDR14) & MT_MIB_AMPDU_MPDU_COUNT;
        acked = mt7615_rr(sc, MT_MIB_SDR15) & MT_MIB_AMPDU_ACK_COUNT;
        for (i = 0; i < MT7615_AGG_CNT_REGS; i++) {
                val = mt7615_rr(sc, MT_TX_AGG_CNT(i));
                agg[i * 2] = val & 0xffff;
                agg[i * 2 + 1] = val >> 16;
        }
        MT7615_UNLOCK(sc);

        sbuf_new_for_sysctl(&sb, NULL, 256, req);
        sbuf_printf(&sb, "in aggregates %u acknowledged %u", sent, acked);
        if (sent != 0)
                sbuf_printf(&sb, " (%u%%)", 100 * acked / sent);
        sbuf_cat(&sb, "\nsizes");
        for (i = 0; i < MT7615_AGG_BUCKETS; i++)
                sbuf_printf(&sb, " %u", agg[i]);
        sbuf_putc(&sb, '\n');

        error = sbuf_finish(&sb);
        sbuf_delete(&sb);

        return (error);
}

static void
mt7615_teardown(struct mt7615_softc *sc)
{
        device_t dev;

        dev = sc->sc_dev;

        if (sc->sc_mountroot_eh != NULL) {
                EVENTHANDLER_DEREGISTER(mountroot, sc->sc_mountroot_eh);
                sc->sc_mountroot_eh = NULL;
        }
        mt7615_free_firmware(sc);
        mt7615_dma_free(sc);
        if (sc->sc_ih != NULL) {
                bus_teardown_intr(dev, sc->sc_irq, sc->sc_ih);
                sc->sc_ih = NULL;
        }
        if (sc->sc_irq != NULL) {
                bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
                    sc->sc_irq);
                sc->sc_irq = NULL;
        }
        if (sc->sc_msi > 0) {
                pci_release_msi(dev);
                sc->sc_msi = 0;
        }
        if (sc->sc_dmat != NULL) {
                bus_dma_tag_destroy(sc->sc_dmat);
                sc->sc_dmat = NULL;
        }
        if (sc->sc_mem != NULL) {
                bus_release_resource(dev, SYS_RES_MEMORY, sc->sc_mem_rid,
                    sc->sc_mem);
                sc->sc_mem = NULL;
        }
        mbufq_drain(&sc->sc_snd);
}

static int
mt7615_attach(device_t dev)
{
        struct mt7615_softc *sc;
        struct ieee80211com *ic;
        uint32_t val;
        int error, nss;

        sc = device_get_softc(dev);
        ic = &sc->sc_ic;

        sc->sc_dev = dev;
        sc->sc_noise = -95;
        sc->sc_debug = 0;

        MT7615_LOCK_INIT(sc);
        /*
         * Deep enough to ride out the wait for the firmware to give
         * transmit tokens back.  The default of fifty frames is a couple
         * of milliseconds at the rates this runs at, so a burst that
         * arrives while the tokens are out gets dropped rather than sent.
         */
        mbufq_init(&sc->sc_snd, MT7615_TX_RING_SIZE);
        callout_init_mtx(&sc->sc_watchdog_to, &sc->sc_mtx, 0);

        /*
         * A thread of the driver's own, for the work net80211 hands over
         * from contexts that may not wait for the firmware.
         */
        STAILQ_INIT(&sc->sc_sta_pending);
        STAILQ_INIT(&sc->sc_ba_pending);
        TASK_INIT(&sc->sc_sta_task, 0, mt7615_sta_task, sc);
        TASK_INIT(&sc->sc_bcn_task, 0, mt7615_beacon_task, sc);
        TASK_INIT(&sc->sc_ba_task, 0, mt7615_ba_task, sc);
        TASK_INIT(&sc->sc_txs_task, 0, mt7615_txs_task, sc);
        sc->sc_tq = taskqueue_create("mt7615_tq", M_NOWAIT,
            taskqueue_thread_enqueue, &sc->sc_tq);
        if (sc->sc_tq == NULL) {
                device_printf(dev, "could not create the task queue\n");
                error = ENOMEM;
                goto fail;
        }
        error = taskqueue_start_threads(&sc->sc_tq, 1, PI_NET, "%s taskq",
            device_get_nameunit(dev));
        if (error != 0) {
                device_printf(dev, "could not start the task thread: %d\n",
                    error);
                goto fail;
        }

        sc->sc_mem_rid = PCIR_BAR(0);
        sc->sc_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
            &sc->sc_mem_rid, RF_ACTIVE);
        if (sc->sc_mem == NULL) {
                device_printf(dev, "could not allocate BAR0\n");
                error = ENXIO;
                goto fail;
        }

        /* Directly addressable, so no remapping is needed yet. */
        sc->sc_hwrev = bus_read_4(sc->sc_mem, MT_HW_REV);
        sc->sc_chipid = bus_read_4(sc->sc_mem, MT_HW_CHIPID);
        device_printf(dev, "chip id %#x, revision %#x\n", sc->sc_chipid,
            sc->sc_hwrev);
        if (sc->sc_chipid != MT7615_DEVICE_MT7615E) {
                device_printf(dev,
                    "expected chip id %#x; the memory window is wrong\n",
                    MT7615_DEVICE_MT7615E);
                error = ENXIO;
                goto fail;
        }

        pci_enable_busmaster(dev);

        error = bus_dma_tag_create(bus_get_dma_tag(dev), 1, 0,
            BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
            BUS_SPACE_MAXSIZE, BUS_SPACE_UNRESTRICTED, BUS_SPACE_MAXSIZE,
            0, NULL, NULL, &sc->sc_dmat);
        if (error != 0) {
                device_printf(dev, "could not create the DMA tag: %d\n",
                    error);
                goto fail;
        }

        /*
         * What a board that keeps no factory data is taken to be.  One
         * chain: it is what the smallest of these boards fits, and asking
         * the firmware for chains that are not there is worse than
         * leaving one unused.  hw.mt7615.nss overrides both this and
         * whatever the factory data says.
         */
        sc->sc_nss = 1;

        /* Reaching the efuse exercises the remap window as well. */
        MT7615_LOCK(sc);
        error = mt7615_read_macaddr(sc);
        MT7615_UNLOCK(sc);
        if (error != 0) {
                device_printf(dev, "could not read the efuse: %d\n", error);
                goto fail;
        }
        nss = sc->sc_nss;
        TUNABLE_INT_FETCH("hw.mt7615.nss", &nss);
        if (nss < 1 || nss > MT7615_MAX_CHAINS)
                nss = 1;
        sc->sc_nss = nss;
        device_printf(dev, "%d spatial stream%s%s\n", sc->sc_nss,
            sc->sc_nss == 1 ? "" : "s",
            sc->sc_have_macaddr ? "" : ", no factory data");

        /*
         * Read here rather than leaving it to the sysctl, which is set up
         * after the channel list has already been built and so would
         * arrive too late to decide what goes in it.
         */
        sc->sc_vht80 = 1;
        TUNABLE_INT_FETCH("hw.mt7615.vht80", &sc->sc_vht80);

        /*
         * How often a frame is marked for a transmit status report.  One
         * in sixteen is a few hundred reports a second on a busy network,
         * which keeps the rate estimate current without crowding the MCU
         * ring that the command replies share.
         */
        sc->sc_txs_div = 16;
        TUNABLE_INT_FETCH("hw.mt7615.txs_div", &sc->sc_txs_div);
        if (sc->sc_txs_div < 0)
                sc->sc_txs_div = 0;

        if (!sc->sc_have_macaddr) {
                /*
                 * Nothing to go on, so make one up rather than let
                 * net80211 register an all-zero address.
                 *
                 * Not with ether_gen_addr_byname(): that hashes the host
                 * UUID, which rc.d sets long after a driver attaches, and
                 * falls back to a random address until it does.  The card
                 * would come up different every boot, so an access point
                 * on it would hand out a new BSSID each time.  Hash where
                 * the card is instead - stable across reboots, which is
                 * what matters, and locally administered so it cannot
                 * collide with a real assignment.
                 */
                val = (pci_get_domain(dev) << 16) | (pci_get_bus(dev) << 8) |
                      (pci_get_slot(dev) << 3) | pci_get_function(dev);

                sc->sc_macaddr[0] = 0x02;
                sc->sc_macaddr[1] = (sc->sc_chipid >> 8) & 0xff;
                sc->sc_macaddr[2] = sc->sc_chipid & 0xff;
                sc->sc_macaddr[3] = (val >> 16) & 0xff;
                sc->sc_macaddr[4] = (val >> 8) & 0xff;
                sc->sc_macaddr[5] = val & 0xff;

                device_printf(dev, "no factory MAC address; using %6D\n",
                    sc->sc_macaddr, ":");
        }

        /*
         * Prefer MSI, but fall back to the legacy line rather than
         * giving up: a host bridge can report MSI as supported and
         * still not hand a vector over, and the two paths fail
         * independently, so both have to be tried.
         */
        sc->sc_msi = 1;
        error = pci_alloc_msi(dev, &sc->sc_msi);
        if (error == 0 && sc->sc_msi > 0) {
                sc->sc_irq_rid = 1;
                sc->sc_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ,
                    &sc->sc_irq_rid, RF_ACTIVE);
                if (sc->sc_irq == NULL) {
                        device_printf(dev, "could not claim the MSI vector; "
                                           "falling back to the legacy interrupt\n");
                        pci_release_msi(dev);
                        sc->sc_msi = 0;
                }
        } else {
                if (bootverbose)
                        device_printf(dev,
                            "no MSI (error %d); using the legacy interrupt\n",
                            error);
                sc->sc_msi = 0;
        }

        if (sc->sc_irq == NULL) {
                sc->sc_irq_rid = 0;
                sc->sc_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ,
                    &sc->sc_irq_rid, RF_ACTIVE | RF_SHAREABLE);
        }
        /*
         * A host bridge that can supply neither is not fatal.  Every
         * MCU command waits by polling the event ring as well as
         * sleeping, so the firmware still downloads and the chip still
         * comes up; what is lost is the receive path, since nothing
         * else drains the data ring.  Attaching anyway leaves the
         * device visible and the bring-up testable instead of failing
         * with only an errno to go on.
         */
        if (sc->sc_irq == NULL) {
                device_printf(dev, "warning: no interrupt; the host bridge "
                                   "supplied neither an MSI vector nor a legacy line\n");
                device_printf(dev, "the firmware will still load, but no "
                                   "traffic can be received\n");
        } else {
                error = bus_setup_intr(dev, sc->sc_irq,
                    INTR_TYPE_NET | INTR_MPSAFE, mt7615_intr,
                    mt7615_intr_thread, sc,
                    &sc->sc_ih);
                if (error != 0) {
                        device_printf(dev,
                            "could not set up the interrupt: %d\n", error);
                        goto fail;
                }
                device_printf(dev, "using %s interrupt (irq %ju)\n",
                    sc->sc_msi > 0 ? "MSI" : "legacy",
                    rman_get_start(sc->sc_irq));
        }

        /* Mask everything until the rings exist to service it. */
        sc->sc_intmask = 0;
        bus_write_4(sc->sc_mem, MT_INT_MASK_CSR, 0);
        bus_write_4(sc->sc_mem, MT_INT_SOURCE_CSR, ~0U);

        error = mt7615_dma_alloc(sc);
        if (error != 0) {
                device_printf(dev, "could not allocate the DMA rings: %d\n",
                    error);
                goto fail;
        }

        ic->ic_softc = sc;
        ic->ic_name = device_get_nameunit(dev);
        ic->ic_phytype = IEEE80211_T_OFDM;
        ic->ic_opmode = IEEE80211_M_STA;

        IEEE80211_ADDR_COPY(ic->ic_macaddr, sc->sc_macaddr);

        ic->ic_caps =
            IEEE80211_C_STA |
            IEEE80211_C_HOSTAP |
            IEEE80211_C_MONITOR |
            IEEE80211_C_WPA |
            IEEE80211_C_WME |
            IEEE80211_C_SHSLOT |
            IEEE80211_C_SHPREAMBLE |
            IEEE80211_C_BGSCAN;

        ic->ic_txstream = sc->sc_nss;
        ic->ic_rxstream = sc->sc_nss;
        ic->ic_htcaps =
            IEEE80211_HTC_HT |
            IEEE80211_HTC_AMPDU |
            IEEE80211_HTCAP_SHORTGI20 |
            IEEE80211_HTCAP_SHORTGI40 |
            IEEE80211_HTCAP_CHWIDTH40 |
            IEEE80211_HTCAP_MAXAMSDU_3839 |
            IEEE80211_HTCAP_SMPS_OFF;

        /*
         * The chip is an 802.11ac part, and on five gigahertz that is
         * where the rate is: eighty megahertz and 256-QAM carry four times
         * what a twenty megahertz 802.11n channel does on the same number
         * of chains.  Eighty is as wide as this goes; the wider channels
         * the chip lists need a second centre frequency, which nothing
         * here fills in.
         */
        /*
         * The one cipher the hardware is told to do.  Everything else
         * net80211 keeps for itself, which is what the missing bits mean.
         */
        ic->ic_cryptocaps = IEEE80211_CRYPTO_AES_CCM;

        ic->ic_flags_ext |= IEEE80211_FEXT_VHT;
        ic->ic_vht_cap.vht_cap_info =
            IEEE80211_VHTCAP_MAX_MPDU_LENGTH_3895 |
            _IEEE80211_SHIFTMASK(IEEE80211_VHTCAP_SUPP_CHAN_WIDTH_NO160,
                IEEE80211_VHTCAP_SUPP_CHAN_WIDTH_MASK) |
            IEEE80211_VHTCAP_SHORT_GI_80 |
            IEEE80211_VHTCAP_RXLDPC |
            _IEEE80211_SHIFTMASK(7,
                IEEE80211_VHTCAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK);
        mt7615_set_vht_mcs(ic);

        mt7615_getradiocaps(ic, IEEE80211_CHAN_MAX, &ic->ic_nchans,
            ic->ic_channels);

        mt7615_add_sysctls(sc);

        /* Fetch the images now, or at the root mount if it has not happened. */
        mt7615_firmware_schedule(sc);

        /*
         * Registering with net80211 has to wait until interrupts are
         * running, so hand it to an interrupt hook and let the boot
         * carry on.
         */
        sc->sc_preinit_hook.ich_func = mt7615_attach_hook;
        sc->sc_preinit_hook.ich_arg = sc;
        if (config_intrhook_establish(&sc->sc_preinit_hook) != 0) {
                device_printf(dev, "could not establish the attach hook\n");
                error = ENXIO;
                goto fail;
        }

        return (0);

fail:
        mt7615_teardown(sc);
        callout_drain(&sc->sc_watchdog_to);
        mt7615_sta_task_free(sc);
        MT7615_LOCK_DESTROY(sc);
        return (error);
}

static int
mt7615_detach(device_t dev)
{
        struct mt7615_softc *sc;

        sc = device_get_softc(dev);

        if ((sc->sc_flags & MT7615_FLAG_ATTACHED) != 0) {
                MT7615_LOCK(sc);
                mt7615_stop(sc);
                MT7615_UNLOCK(sc);
                ieee80211_ifdetach(&sc->sc_ic);
        }

        callout_drain(&sc->sc_watchdog_to);
        mt7615_sta_task_free(sc);
        mt7615_teardown(sc);
        pci_disable_busmaster(dev);
        MT7615_LOCK_DESTROY(sc);

        return (0);
}

static int
mt7615_suspend(device_t dev)
{
        struct mt7615_softc *sc;

        sc = device_get_softc(dev);

        if ((sc->sc_flags & MT7615_FLAG_ATTACHED) == 0)
                return (0);

        ieee80211_suspend_all(&sc->sc_ic);

        MT7615_LOCK(sc);
        mt7615_stop(sc);
        MT7615_UNLOCK(sc);

        return (0);
}

static int
mt7615_resume(device_t dev)
{
        struct mt7615_softc *sc;

        sc = device_get_softc(dev);

        if ((sc->sc_flags & MT7615_FLAG_ATTACHED) == 0)
                return (0);

        pci_enable_busmaster(dev);

        /* The VAPs bring the hardware back up through ic_parent. */
        ieee80211_resume_all(&sc->sc_ic);

        return (0);
}

static device_method_t mt7615_methods[] = {
    DEVMETHOD(device_probe,		mt7615_probe),
    DEVMETHOD(device_attach,	mt7615_attach),
    DEVMETHOD(device_detach,	mt7615_detach),
    DEVMETHOD(device_suspend,	mt7615_suspend),
    DEVMETHOD(device_resume,	mt7615_resume),

    DEVMETHOD_END
};

static driver_t mt7615_driver = {
    "mt7615",
    mt7615_methods,
    sizeof(struct mt7615_softc)
};

DRIVER_MODULE(mt7615, pci, mt7615_driver, NULL, NULL);
MODULE_DEPEND(mt7615, firmware, 1, 1, 1);
MODULE_DEPEND(mt7615, pci, 1, 1, 1);
MODULE_DEPEND(mt7615, wlan, 1, 1, 1);
MODULE_VERSION(mt7615, 1);
