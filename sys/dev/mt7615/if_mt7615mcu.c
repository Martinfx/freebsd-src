/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * MediaTek MT7615E: the MCU command transport and the firmware
 * download that runs over it.
 *
 * A command is a Tx descriptor followed by struct mt7615_mcu_txd and
 * then the command payload, pushed onto one of two Tx rings: FWDL while
 * the firmware is being downloaded, and the general MCU ring once it is
 * running.  Replies come back on the MCU Rx ring carrying the sequence
 * number they answer, which is how the sender is matched and woken.
 *
 * The download itself is three images.  The ROM patch is guarded by a
 * semaphore, since another driver instance may already have pushed it;
 * N9 and CR4 are sent region by region, each region announced with its
 * load address and length before its payload is scattered across
 * FW_SCATTER commands.  Each image is then started, and the chip
 * reports FW_STATE_RDY once the MCU is alive.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/socket.h>
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

#define	MT7615_MCU_TIMEOUT	(20 * hz)

/*
 * Command framing.
 */

static void
mt7615_mcu_dma_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{

        if (error != 0)
                return;
        KASSERT(nsegs == 1, ("%d MCU segments, expected 1", nsegs));
        *(bus_addr_t *)arg = segs[0].ds_addr;
}

static uint8_t
mt7615_mcu_next_seq(struct mt7615_softc *sc)
{
        uint8_t seq;

        seq = ++sc->sc_mcu_seq & 0xf;
        if (seq == 0)
                seq = ++sc->sc_mcu_seq & 0xf;

        return (seq);
}

/*
 * Push one already-built command buffer onto a Tx ring.  Ownership of
 * the DMA allocation passes to the ring, which frees it when the engine
 * is done with the descriptor.
 */
static int
mt7615_mcu_queue_cmd(struct mt7615_softc *sc, struct mt7615_tx_ring *ring,
                     void *buf, size_t len, bus_dma_tag_t tag, bus_dmamap_t map,
                     bus_addr_t paddr)
{
        struct mt7615_tx_data *data;
        int idx;

        MT7615_ASSERT_LOCKED(sc);

        bus_dmamap_sync(tag, map, BUS_DMASYNC_PREWRITE);

        idx = mt7615_tx_queue_buf(sc, ring, paddr, len, 0, 0, buf, NULL, NULL);
        if (idx < 0)
                return (ENOBUFS);

        data = &ring->data[idx];
        data->cmd_tag = tag;
        data->cmd_map = map;

        return (0);
}

/*
 * Build and send one MCU command.
 *
 * cmd is the plain command id; ext_cid is nonzero for the extended
 * commands, which are sent as MCU_CMD_EXT_CID with the real id in the
 * header.  wait says whether to block for the reply.
 */
int
mt7615_mcu_send_msg(struct mt7615_softc *sc, int cmd, int ext_cid,
                    const void *payload, size_t payload_len, bool wait)
{
        struct mt7615_mcu_txd *txd;
        struct mt7615_tx_ring *ring;
        bus_dma_tag_t tag;
        bus_dmamap_t map;
        bus_addr_t paddr;
        uint8_t *buf;
        size_t len;
        uint32_t val;
        uint8_t seq, q_idx, pkt_fmt;
        int deadline, error;

        MT7615_ASSERT_LOCKED(sc);

        len = sizeof(*txd) + payload_len;
        /* The engine wants whole words. */
        len = roundup2(len, 4);

        error = bus_dma_tag_create(sc->sc_dmat, 4, 0,
            BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL, len, 1,
            len, 0, NULL, NULL, &tag);
        if (error != 0)
                return (error);

        error = bus_dmamem_alloc(tag, (void **)&buf,
            BUS_DMA_NOWAIT | BUS_DMA_ZERO | BUS_DMA_COHERENT, &map);
        if (error != 0) {
                bus_dma_tag_destroy(tag);
                return (error);
        }

        paddr = 0;
        error = bus_dmamap_load(tag, map, buf, len, mt7615_mcu_dma_cb, &paddr,
            BUS_DMA_NOWAIT);
        if (error != 0 || paddr == 0) {
                bus_dmamem_free(tag, buf, map);
                bus_dma_tag_destroy(tag);
                return (error != 0 ? error : ENOMEM);
        }

        txd = (struct mt7615_mcu_txd *)buf;
        if (payload_len != 0)
                memcpy(buf + sizeof(*txd), payload, payload_len);

        seq = mt7615_mcu_next_seq(sc);

        /*
         * The firmware download runs on its own queue and is tagged as
         * firmware rather than command; everything else is a command on
         * the MCU queue.
         */
        if (cmd == MCU_CMD_FW_SCATTER) {
                q_idx = MT_TX_PQ_MCU_PORT_RX_FWDL;
                pkt_fmt = MT_TX_TYPE_FW;
        } else {
                q_idx = MT_TX_PQ_MCU_PORT_RX_Q0;
                pkt_fmt = MT_TX_TYPE_CMD;
        }

        val = (len & MT_TXD0_TX_BYTES) |
              ((q_idx << MT_TXD0_PQ_IDX_S) & MT_TXD0_PQ_IDX);
        txd->txd[0] = htole32(val);

        val = MT_TXD1_LONG_FORMAT |
              ((MT_HDR_FORMAT_CMD << MT_TXD1_HDR_FORMAT_S) &
               MT_TXD1_HDR_FORMAT) |
              ((pkt_fmt << MT_TXD1_PKT_FMT_S) & MT_TXD1_PKT_FMT);
        txd->txd[1] = htole32(val);

        txd->len = htole16(len - sizeof(txd->txd));
        /*
         * pq_id wants the port and queue as separate fields, so undo the
         * combination the descriptor word uses.
         */
        txd->pq_id = htole16(MCU_PQ_ID(MT_TX_PORT_IDX_MCU, q_idx & 0x1f));
        txd->s2d_index = MCU_S2D_H2N;
        txd->pkt_type = MCU_PKT_ID;
        txd->seq = seq;
        txd->cid = ext_cid != 0 ? MCU_CMD_EXT_CID : cmd;
        txd->ext_cid = ext_cid;

        if (ext_cid != 0) {
                txd->set_query = MCU_Q_SET;
                txd->ext_cid_ack = 1;
        } else {
                txd->set_query = MCU_Q_NA;
        }

        /*
         * Arm the wait before queueing: the reply can land in the Rx
         * ring as soon as the descriptor is handed over, and the
         * interrupt path takes the same lock this thread is holding.
         */
        if (wait) {
                sc->sc_mcu_wait_seq = seq;
                sc->sc_mcu_wait_cmd = ext_cid != 0 ? ext_cid : cmd;
                sc->sc_mcu_done = false;
                sc->sc_mcu_resp = 0;
        }

        ring = (sc->sc_flags & MT7615_FLAG_FW_RUNNING) != 0 ?
               &sc->sc_txq[MT7615_TXQ_MCU] : &sc->sc_txq[MT7615_TXQ_FWDL];

        error = mt7615_mcu_queue_cmd(sc, ring, buf, len, tag, map, paddr);
        if (error != 0) {
                bus_dmamap_unload(tag, map);
                bus_dmamem_free(tag, buf, map);
                bus_dma_tag_destroy(tag);
                if (wait)
                        sc->sc_mcu_wait_seq = 0;
                return (error);
        }

        if (!wait)
                return (0);

        /*
         * Poll the event ring alongside the sleep rather than relying on
         * the interrupt: the download runs before the interrupt handler
         * is worth trusting, and a card whose interrupt never arrived
         * would otherwise stall here for the full timeout on every
         * command.
         */
        for (deadline = ticks + MT7615_MCU_TIMEOUT; !sc->sc_mcu_done; ) {
                mt7615_rx_poll(sc, MT7615_RXQ_MCU);
                if (sc->sc_mcu_done)
                        break;
                if ((int)(ticks - deadline) >= 0)
                        break;
                (void)msleep(&sc->sc_mcu_done, &sc->sc_mtx, 0, "mt76mcu",
                    hz / 100);
        }

        sc->sc_mcu_wait_seq = 0;

        if (!sc->sc_mcu_done) {
                device_printf(sc->sc_dev,
                    "MCU command %#x (seq %u) timed out\n",
                    ext_cid != 0 ? ext_cid : cmd, seq);
                return (ETIMEDOUT);
        }

        return (sc->sc_mcu_resp);
}

/*
 * Reply handling.  The event carries the sequence number of the command
 * it answers; anything else is an unsolicited event from the firmware,
 * which this driver does not act on yet.
 */
void
mt7615_mcu_rx_event(struct mt7615_softc *sc, struct mbuf *m)
{
        const struct mt7615_mcu_rxd *rxd;
        const uint8_t *payload;
        int resp, type;

        MT7615_ASSERT_LOCKED(sc);

        if (m->m_pkthdr.len < (int)sizeof(uint32_t)) {
                m_freem(m);
                return;
        }

        /*
         * Not everything arriving here is a reply.  The firmware returns
         * finished frames on this ring too, and that has to be told apart
         * from a reply before anything else: such an event is far shorter
         * than a reply header, so a length check would throw it away, and
         * it carries no sequence number to match, so the reply path would
         * dismiss it as unsolicited.  Either way nothing would ever be
         * freed and the transmit path would seize up once every frame the
         * firmware can hold is outstanding.
         */
        type = (le32toh(*mtod(m, const uint32_t *)) & MT_RXD0_PKT_TYPE) >>
        MT_RXD0_PKT_TYPE_S;
        if (type == MT_PKT_TYPE_TXRX_NOTIFY) {
                mt7615_mac_tx_free(sc, m);
                return;
        }
        if (type == MT_PKT_TYPE_TXS) {
                mt7615_mac_rx_txs(sc, m);
                return;
        }

        if (m->m_pkthdr.len < (int)sizeof(*rxd)) {
                m_freem(m);
                return;
        }

        rxd = mtod(m, const struct mt7615_mcu_rxd *);

        if (sc->sc_mcu_wait_seq == 0 || rxd->seq != sc->sc_mcu_wait_seq) {
                MT7615_DPRINTF(sc, MT7615_DEBUG_MCU,
                    "unsolicited MCU event, eid %#x seq %u\n", rxd->eid,
                    rxd->seq);
                m_freem(m);
                return;
        }

        /*
         * Most replies are only interesting for having arrived.  The
         * patch semaphore is the exception: its status byte says
         * whether the patch still has to be downloaded, and it sits
         * four bytes into the tail of the header.
         */
        resp = 0;
        payload = (const uint8_t *)rxd;
        if (sc->sc_mcu_wait_cmd == MCU_CMD_PATCH_SEM_CONTROL &&
            m->m_pkthdr.len >= (int)sizeof(*rxd))
                resp = payload[sizeof(*rxd) - 4];

        sc->sc_mcu_resp = resp;
        sc->sc_mcu_done = true;
        wakeup(&sc->sc_mcu_done);

        m_freem(m);
}

/*
 * Ownership handshake.  The MCU parks the chip in firmware-own when it
 * is idle; the driver has to ask for it back before touching the MAC.
 */
static int
mt7615_mcu_drv_own(struct mt7615_softc *sc)
{

        MT7615_ASSERT_LOCKED(sc);

        mt7615_wr(sc, MT_CFG_LPCR_HOST, MT_CFG_LPCR_HOST_DRV_OWN);

        if (!mt7615_poll(sc, MT_CFG_LPCR_HOST, MT_CFG_LPCR_HOST_FW_OWN, 0,
            3000)) {
                device_printf(sc->sc_dev, "could not take ownership\n");
                return (ETIMEDOUT);
        }

        return (0);
}

static void
mt7615_mcu_fw_own(struct mt7615_softc *sc)
{

        MT7615_ASSERT_LOCKED(sc);

        mt7615_wr(sc, MT_CFG_LPCR_HOST, MT_CFG_LPCR_HOST_FW_OWN);
}

/*
 * Download primitives.
 */

static int
mt7615_mcu_patch_sem_ctrl(struct mt7615_softc *sc, bool get)
{
        struct {
            uint32_t	op;
        } __packed req;

        req.op = htole32(get ? MT_PATCH_SEM_GET : MT_PATCH_SEM_RELEASE);

        return (mt7615_mcu_send_msg(sc, MCU_CMD_PATCH_SEM_CONTROL, 0, &req,
            sizeof(req), true));
}

static int
mt7615_mcu_start_patch(struct mt7615_softc *sc)
{
        struct {
            uint8_t		check_crc;
            uint8_t		reserved[3];
        } __packed req;

        memset(&req, 0, sizeof(req));

        return (mt7615_mcu_send_msg(sc, MCU_CMD_PATCH_FINISH_REQ, 0, &req,
            sizeof(req), true));
}

static int
mt7615_mcu_start_firmware(struct mt7615_softc *sc, uint32_t addr,
                          uint32_t option)
{
        struct {
            uint32_t	option;
            uint32_t	addr;
        } __packed req;

        req.option = htole32(option);
        req.addr = htole32(addr);

        return (mt7615_mcu_send_msg(sc, MCU_CMD_FW_START_REQ, 0, &req,
            sizeof(req), true));
}

static int
mt7615_mcu_init_download(struct mt7615_softc *sc, uint32_t addr, uint32_t len,
                         uint32_t mode)
{
        struct {
            uint32_t	addr;
            uint32_t	len;
            uint32_t	mode;
        } __packed req;

        req.addr = htole32(addr);
        req.len = htole32(len);
        req.mode = htole32(mode);

        return (mt7615_mcu_send_msg(sc, MCU_CMD_TARGET_ADDRESS_LEN_REQ, 0,
            &req, sizeof(req), true));
}

/*
 * Push one region's payload across as a run of FW_SCATTER commands.
 * Only the last chunk is waited on; the earlier ones are pipelined so
 * the ring stays busy, which is what makes the download tolerable.
 */
static int
mt7615_mcu_send_firmware(struct mt7615_softc *sc, const uint8_t *data,
                         uint32_t len)
{
        struct mt7615_tx_ring *ring;
        uint32_t chunk, maxchunk;
        int error, i;

        ring = &sc->sc_txq[MT7615_TXQ_FWDL];
        maxchunk = MT7615_FW_CHUNK_SIZE - sizeof(struct mt7615_mcu_txd);

        while (len > 0) {
                chunk = MIN(len, maxchunk);

                /*
                 * Never wait on one of these.  FW_SCATTER is the one
                 * command the MCU answers with nothing at all, so a wait
                 * can only ever time out; the next command that does
                 * expect a reply is what confirms the payload landed,
                 * since the MCU works through the queue in order.
                 */
                error = mt7615_mcu_send_msg(sc, MCU_CMD_FW_SCATTER, 0, data,
                    chunk, false);
                if (error != 0)
                        return (error);

                data += chunk;
                len -= chunk;

                /*
                 * With nothing waiting, the ring fills as fast as this
                 * loop can push, so reclaim finished descriptors every
                 * time round and give the engine a chance to catch up
                 * before running it dry.
                 */
                mt7615_tx_cleanup(sc, ring);
                for (i = 0; ring->queued >= ring->ndesc - 4 && i < 200; i++) {
                        (void)msleep(sc, &sc->sc_mtx, 0, "mt76fwdl", 1);
                        mt7615_tx_cleanup(sc, ring);
                }
                if (ring->queued >= ring->ndesc - 4) {
                        device_printf(sc->sc_dev,
                            "the firmware download queue stalled\n");
                        return (ETIMEDOUT);
                }
        }

        return (0);
}

/*
 * Firmware images.
 *
 * firmware(9) can end up loading a module to satisfy a request, which
 * sleeps, so everything from here down to mt7615_load_firmware() must
 * be called with the driver lock NOT held.  Attach and detach are the
 * only callers, and neither races with anything.
 *
 * The images are named with a path the way Linux wants them.  How the
 * firmware port ends up registering them varies, so try the same
 * spellings the linuxkpi compat layer does before giving up.
 */
static const struct firmware *
mt7615_firmware_get(const char *name)
{
        char buf[64];
        const struct firmware *fw;
        const char *base;
        char *p;

        fw = firmware_get_flags(name, FIRMWARE_GET_NOWARN);
        if (fw != NULL)
                return (fw);

        base = strrchr(name, '/');
        if (base != NULL && base[1] != '\0') {
                fw = firmware_get_flags(base + 1, FIRMWARE_GET_NOWARN);
                if (fw != NULL)
                        return (fw);
        }

        strlcpy(buf, name, sizeof(buf));
        for (p = buf; (p = strchr(p, '/')) != NULL; )
                *p = '_';
        fw = firmware_get_flags(buf, FIRMWARE_GET_NOWARN);
        if (fw != NULL)
                return (fw);

        for (p = buf; (p = strchr(p, '.')) != NULL; )
                *p = '_';
        return (firmware_get_flags(buf, FIRMWARE_GET_NOWARN));
}

/*
 * Copy one of the fixed-width strings out of a firmware header into a
 * buffer that is safe to print.  The fields are padded rather than
 * terminated, and nothing stops a vendor from leaving a stray control
 * character in one: a newline in the middle of a build date cuts the
 * rest of the line off, which is how the ROM patch's length and load
 * address went missing from the boot log.  Anything unprintable
 * becomes a space.
 */
static const char *
mt7615_fwstr(char *dst, size_t dstlen, const char *src, size_t srclen)
{
        size_t i, n;

        n = MIN(srclen, dstlen - 1);
        for (i = 0; i < n && src[i] != '\0'; i++)
                dst[i] = (src[i] >= 0x20 && src[i] < 0x7f) ? src[i] : ' ';
        dst[i] = '\0';

        return (dst);
}

#define	MT7615_FWSTR(_b, _f)						\
	mt7615_fwstr((_b), sizeof(_b), (_f), sizeof(_f))

static void
mt7615_firmware_put(struct mt7615_fw *f)
{

        if (f->fw != NULL) {
                firmware_put(f->fw, FIRMWARE_UNLOAD);
                f->fw = NULL;
        }
        f->data = NULL;
        f->len = 0;
        f->nregions = 0;
}

void
mt7615_free_firmware(struct mt7615_softc *sc)
{

        mt7615_firmware_put(&sc->sc_patch);
        mt7615_firmware_put(&sc->sc_n9);
        mt7615_firmware_put(&sc->sc_cr4);
        sc->sc_flags &= ~MT7615_FLAG_FW_LOADED;
}

/*
 * The ROM patch carries its header at the front; the payload is
 * everything after it.
 */
static int
mt7615_read_patch(struct mt7615_softc *sc)
{
        const struct mt7615_patch_hdr *hdr;
        struct mt7615_fw *f;
        char date[sizeof(hdr->build_date) + 1];

        f = &sc->sc_patch;
        f->name = MT7615_ROM_PATCH;
        f->fw = mt7615_firmware_get(f->name);
        if (f->fw == NULL) {
                device_printf(sc->sc_dev, "firmware %s not found; install "
                                          "wifi-firmware-mt76-kmod\n", f->name);
                return (ENOENT);
        }
        if (f->fw->datasize <= sizeof(*hdr)) {
                device_printf(sc->sc_dev, "firmware %s is truncated\n",
                    f->name);
                mt7615_firmware_put(f);
                return (EINVAL);
        }

        hdr = (const struct mt7615_patch_hdr *)f->fw->data;
        f->data = (const uint8_t *)f->fw->data + sizeof(*hdr);
        f->len = f->fw->datasize - sizeof(*hdr);
        f->addr = MT7615_PATCH_ADDRESS;

        device_printf(sc->sc_dev,
            "ROM patch: version %#x, built %s, %u bytes for %#x\n",
            be32toh(hdr->hw_sw_ver), MT7615_FWSTR(date, hdr->build_date),
            f->len, f->addr);

        return (0);
}

/*
 * N9 and CR4 carry one trailer per region at the end of the file, so
 * the first trailer sits nregions entries back from the end.  Each
 * region's payload is stored in order from the start of the file, and
 * every one is followed by a four byte CRC that the MCU expects to
 * receive along with it.
 */
static int
mt7615_read_ram(struct mt7615_softc *sc, struct mt7615_fw *f, const char *name,
                const char *what, int nregions)
{
        const struct mt7615_fw_trailer *hdr;
        char ver[sizeof(hdr->fw_ver) + 1];
        char date[sizeof(hdr->build_date) + 1];
        size_t tsize, need;
        int i;

        tsize = nregions * sizeof(*hdr);

        f->name = name;
        f->nregions = nregions;
        f->fw = mt7615_firmware_get(name);
        if (f->fw == NULL) {
                device_printf(sc->sc_dev, "firmware %s not found; install "
                                          "wifi-firmware-mt76-kmod\n", name);
                return (ENOENT);
        }
        if (f->fw->datasize <= tsize) {
                device_printf(sc->sc_dev, "firmware %s is truncated\n", name);
                mt7615_firmware_put(f);
                return (EINVAL);
        }

        hdr = (const struct mt7615_fw_trailer *)((const uint8_t *)f->fw->data +
                                                 f->fw->datasize - tsize);

        f->data = f->fw->data;
        f->addr = le32toh(hdr[0].addr);
        f->len = le32toh(hdr[0].len);
        f->feature_set = hdr[0].feature_set;

        need = 0;
        for (i = 0; i < nregions; i++) {
                f->region[i].addr = le32toh(hdr[i].addr);
                f->region[i].len = le32toh(hdr[i].len) + MT7615_IMG_CRC_LEN;
                f->region[i].feature_set = hdr[i].feature_set;
                need += f->region[i].len;
        }

        device_printf(sc->sc_dev,
            "%s firmware: version %s, built %s, %zu bytes for %#x\n", what,
            MT7615_FWSTR(ver, hdr[0].fw_ver),
            MT7615_FWSTR(date, hdr[0].build_date), need, f->addr);

        if (need > f->fw->datasize - tsize) {
                device_printf(sc->sc_dev,
                    "%s firmware: the trailers claim %zu bytes but the file "
                    "holds %zu\n", what, need, f->fw->datasize - tsize);
                mt7615_firmware_put(f);
                return (EINVAL);
        }

        if (f == &sc->sc_n9)
                snprintf(sc->sc_fwver, sizeof(sc->sc_fwver), "%s-%s", ver,
                    date);

        return (0);
}

int
mt7615_load_firmware(struct mt7615_softc *sc)
{
        int error;

        if ((sc->sc_flags & MT7615_FLAG_FW_LOADED) != 0)
                return (0);

        error = mt7615_read_patch(sc);
        if (error != 0)
                goto fail;

        error = mt7615_read_ram(sc, &sc->sc_n9, MT7615_FIRMWARE_N9, "N9",
            MT7615_N9_REGION_NUM);
        if (error != 0)
                goto fail;

        error = mt7615_read_ram(sc, &sc->sc_cr4, MT7615_FIRMWARE_CR4, "CR4",
            MT7615_CR4_REGION_NUM);
        if (error != 0)
                goto fail;

        sc->sc_flags |= MT7615_FLAG_FW_LOADED;
        return (0);

fail:
        mt7615_free_firmware(sc);
        return (error);
}

/*
 * Download.
 */

static uint32_t
mt7615_mcu_gen_dl_mode(uint8_t feature_set, bool is_cr4)
{
        uint32_t ret;

        ret = (feature_set & MT7615_FW_FEATURE_SET_ENCRYPT) != 0 ?
              (MT_DL_MODE_ENCRYPT | MT_DL_MODE_RESET_SEC_IV) : 0;
        ret |= (((feature_set & MT7615_FW_FEATURE_SET_KEY_IDX) >>
                                                               MT7615_FW_FEATURE_SET_KEY_IDX_S) << MT_DL_MODE_KEY_IDX_S) &
               MT_DL_MODE_KEY_IDX;
        ret |= MT_DL_MODE_NEED_RSP;
        ret |= is_cr4 ? MT_DL_MODE_WORKING_PDA_CR4 : 0;

        return (ret);
}

static int
mt7615_mcu_download_patch(struct mt7615_softc *sc)
{
        struct mt7615_fw *f;
        int error, sem;

        f = &sc->sc_patch;

        sem = mt7615_mcu_patch_sem_ctrl(sc, true);
        switch (sem) {
                case MT_PATCH_IS_DL:
                        /* Someone got there first; nothing to do. */
                        MT7615_DPRINTF(sc, MT7615_DEBUG_FIRMWARE,
                            "the ROM patch is already loaded\n");
                        return (0);
                case MT_PATCH_NOT_DL_SEM_SUCCESS:
                        break;
                default:
                        device_printf(sc->sc_dev,
                            "could not take the patch semaphore (%d)\n", sem);
                        return (EAGAIN);
        }

        error = mt7615_mcu_init_download(sc, f->addr, f->len,
            MT_DL_MODE_NEED_RSP);
        if (error != 0) {
                device_printf(sc->sc_dev, "the patch download request "
                                          "failed: %d\n", error);
                goto out;
        }

        error = mt7615_mcu_send_firmware(sc, f->data, f->len);
        if (error != 0) {
                device_printf(sc->sc_dev, "could not send the patch: %d\n",
                    error);
                goto out;
        }

        error = mt7615_mcu_start_patch(sc);
        if (error != 0)
                device_printf(sc->sc_dev, "could not start the patch: %d\n",
                    error);

out:
        sem = mt7615_mcu_patch_sem_ctrl(sc, false);
        if (sem != MT_PATCH_REL_SEM_SUCCESS) {
                device_printf(sc->sc_dev,
                    "could not release the patch semaphore (%d)\n", sem);
                if (error == 0)
                        error = EAGAIN;
        }

        return (error);
}

static int
mt7615_mcu_download_ram(struct mt7615_softc *sc, struct mt7615_fw *f,
                        bool is_cr4)
{
        uint32_t mode, offset;
        int error, i;

        offset = 0;
        for (i = 0; i < f->nregions; i++) {
                mode = mt7615_mcu_gen_dl_mode(f->region[i].feature_set,
                    is_cr4);

                error = mt7615_mcu_init_download(sc, f->region[i].addr,
                    f->region[i].len, mode);
                if (error != 0) {
                        device_printf(sc->sc_dev, "the download request for "
                                                  "%s region %d failed: %d\n", f->name, i, error);
                        return (error);
                }

                error = mt7615_mcu_send_firmware(sc, f->data + offset,
                    f->region[i].len);
                if (error != 0) {
                        device_printf(sc->sc_dev, "could not send %s region "
                                                  "%d: %d\n", f->name, i, error);
                        return (error);
                }

                offset += f->region[i].len;
        }

        return (0);
}

static int
mt7615_mcu_download_all(struct mt7615_softc *sc)
{
        uint32_t val;
        int error;

        MT7615_ASSERT_LOCKED(sc);

        val = mt7615_rr(sc, MT_TOP_MISC2) & MT_TOP_MISC2_FW_STATE;
        if (val != MT_FW_STATE_FW_DOWNLOAD) {
                device_printf(sc->sc_dev,
                    "the chip is not ready for a download (state %u)\n", val);
                return (EIO);
        }

        error = mt7615_mcu_download_patch(sc);
        if (error != 0)
                return (error);

        error = mt7615_mcu_download_ram(sc, &sc->sc_n9, false);
        if (error != 0)
                return (error);

        error = mt7615_mcu_start_firmware(sc, sc->sc_n9.addr,
            MT_FW_START_OVERRIDE);
        if (error != 0) {
                device_printf(sc->sc_dev, "could not start N9: %d\n", error);
                return (error);
        }

        error = mt7615_mcu_download_ram(sc, &sc->sc_cr4, true);
        if (error != 0)
                return (error);

        error = mt7615_mcu_start_firmware(sc, 0, MT_FW_START_WORKING_PDA_CR4);
        if (error != 0) {
                device_printf(sc->sc_dev, "could not start CR4: %d\n", error);
                return (error);
        }

        if (!mt7615_poll(sc, MT_TOP_MISC2, MT_TOP_MISC2_FW_STATE,
            MT_FW_STATE_RDY, 500)) {
                device_printf(sc->sc_dev,
                    "the firmware did not come up (state %u)\n",
                    mt7615_rr(sc, MT_TOP_MISC2) & MT_TOP_MISC2_FW_STATE);
                return (EIO);
        }

        device_printf(sc->sc_dev, "firmware %s running\n", sc->sc_fwver);

        return (0);
}

/*
 * Bring the MCU up: take ownership, start the DMA engine so commands
 * have somewhere to go, download, then switch the command queue over to
 * the running firmware's ring.
 */
int
mt7615_mcu_init(struct mt7615_softc *sc)
{
        int error;

        MT7615_ASSERT_LOCKED(sc);

        error = mt7615_mcu_drv_own(sc);
        if (error != 0)
                return (error);

        error = mt7615_mcu_download_all(sc);
        if (error != 0)
                return (error);

        sc->sc_flags |= MT7615_FLAG_FW_RUNNING;

        /*
         * The images stay resident rather than being released here:
         * firmware_put() sleeps, this runs under the driver lock, and
         * bringing the interface back up would need them again anyway.
         * Detach gives them back.
         */
        return (0);
}

void
mt7615_mcu_exit(struct mt7615_softc *sc)
{

        MT7615_ASSERT_LOCKED(sc);

        if ((sc->sc_flags & MT7615_FLAG_FW_RUNNING) != 0) {
                (void)mt7615_mcu_send_msg(sc, MCU_CMD_RESTART_DL_REQ, 0, NULL,
                    0, true);
                sc->sc_flags &= ~MT7615_FLAG_FW_RUNNING;
        }

        mt7615_mcu_fw_own(sc);
}

/*
 * Configuration commands.
 */

int
mt7615_mcu_set_rts_thresh(struct mt7615_softc *sc, uint32_t val)
{
        struct {
            uint8_t		prot_idx;
            uint8_t		band;
            uint8_t		rsv[2];
            uint32_t	len_thresh;
            uint32_t	pkt_thresh;
        } __packed req;

        memset(&req, 0, sizeof(req));
        req.prot_idx = 1;
        req.band = 0;
        req.len_thresh = htole32(val);
        req.pkt_thresh = htole32(0x2);

        return (mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_PROTECT_CTRL, &req,
            sizeof(req), true));
}

int
mt7615_mcu_set_mac_enable(struct mt7615_softc *sc, int band, bool enable)
{
        struct {
            uint8_t		enable;
            uint8_t		band;
            uint8_t		rsv[2];
        } __packed req;

        memset(&req, 0, sizeof(req));
        req.enable = enable ? 1 : 0;
        req.band = band;

        return (mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_MAC_INIT_CTRL, &req,
            sizeof(req), true));
}

int
mt7615_mcu_del_wtbl_all(struct mt7615_softc *sc)
{
        struct {
            uint8_t		operation;
            uint8_t		count;
            uint8_t		rsv[2];
            uint8_t		index;
            uint8_t		rsv1[3];
        } __packed req;

        memset(&req, 0, sizeof(req));
        req.operation = 3;	/* WTBL_RESET_ALL */
        req.count = 1;

        return (mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_WTBL_UPDATE, &req,
            sizeof(req), true));
}

/*
 * Calibrate the radio for where it is about to sit.
 *
 * The receiver's DC offset and the transmitter's pre-distortion both
 * depend on the centre of the band and on how wide it is, and the
 * channel switch does not work them out on its own.  The reference
 * driver hands over values cached in the factory data when it finds
 * them and otherwise asks the firmware to measure; asking is what
 * happens here, since nothing reads the cached blocks out.
 *
 * The two commands carry a large block of results back and forth, all
 * of which is left at zero on the way out.
 */
static int
mt7615_mcu_calibrate(struct mt7615_softc *sc, uint16_t center_freq,
                     uint8_t bw)
{
        struct mt7615_cal_hdr {
            uint8_t		direction;
            uint8_t		runtime_calibration;
            uint8_t		rsv[2];
            uint16_t	center_freq;
            uint8_t		bw;
            uint8_t		band;
            uint8_t		is_freq2;
            uint8_t		success;
            uint8_t		dbdc_en;
            uint8_t		rsv2;
        } hdr;
        struct {
            struct mt7615_cal_hdr	hdr;
            uint32_t		dcoc_data[4][16];
        } __packed dcoc;
        struct {
            struct mt7615_cal_hdr	hdr;
            uint8_t			dpd_data[216];
        } __packed dpd;
        int error;

        MT7615_ASSERT_LOCKED(sc);

        memset(&hdr, 0, sizeof(hdr));
        hdr.direction = 1;
        hdr.runtime_calibration = 1;	/* Measure it rather than recall it. */
        hdr.center_freq = htole16(center_freq);
        hdr.bw = bw;
        hdr.band = center_freq > 4000 ? 1 : 0;

        memset(&dcoc, 0, sizeof(dcoc));
        dcoc.hdr = hdr;
        error = mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_RXDCOC_CAL, &dcoc,
            sizeof(dcoc), true);
        if (error != 0) {
                device_printf(sc->sc_dev,
                    "the receiver would not calibrate at %u MHz: %d\n",
                    center_freq, error);
                return (error);
        }

        memset(&dpd, 0, sizeof(dpd));
        dpd.hdr = hdr;
        error = mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_TXDPD_CAL, &dpd,
            sizeof(dpd), true);
        if (error != 0)
                device_printf(sc->sc_dev,
                    "the transmitter would not calibrate at %u MHz: %d\n",
                    center_freq, error);

        return (error);
}

/*
 * Channel switch.  The MCU wants the channel in its own numbering,
 * along with the bandwidth and which half of it the control channel
 * sits in; a 20MHz STA only ever uses the simplest case.
 */
static int
mt7615_mcu_set_channel_bw(struct mt7615_softc *sc, struct ieee80211_channel *c,
                          uint8_t want_bw)
{
        struct {
            uint8_t		control_chan;
            uint8_t		center_chan;
            uint8_t		bw;
            uint8_t		tx_streams;
            uint8_t		rx_streams_mask;
            uint8_t		switch_reason;
            uint8_t		band_idx;
            uint8_t		center_chan2;
            uint16_t	cac_case;
            uint8_t		channel_band;
            uint8_t		rsv0;
            uint32_t	outband_freq;
            uint8_t		txpower_drop;
            uint8_t		rsv1[3];
            uint8_t		txpower_sku[53];
            uint8_t		rsv2[3];
        } __packed req;
        struct ieee80211com *ic;
        int error, power;
        uint8_t chan;

        ic = &sc->sc_ic;
        chan = ieee80211_chan2ieee(ic, c);

        memset(&req, 0, sizeof(req));
        req.control_chan = chan;
        /*
         * A forty megahertz channel is a pair, and the firmware wants
         * both the one the network is announced on and the middle of the
         * pair.  The second half sits either above or below, two channel
         * numbers away, so the middle is one step in that direction.
         */
        if (want_bw == MT_CH_BW_80) {
                /*
                 * Eighty megahertz spans four channels, and net80211 has
                 * already worked out which one sits in the middle.
                 */
                req.center_chan = c->ic_vht_ch_freq1;
                req.bw = MT_CH_BW_80;
        } else if (want_bw == MT_CH_BW_40 && IEEE80211_IS_CHAN_HT40U(c)) {
                req.center_chan = chan + 2;
                req.bw = MT_CH_BW_40;
        } else if (want_bw == MT_CH_BW_40 && IEEE80211_IS_CHAN_HT40D(c)) {
                req.center_chan = chan - 2;
                req.bw = MT_CH_BW_40;
        } else {
                req.center_chan = chan;
                req.bw = MT_CH_BW_20;
        }
        req.tx_streams = sc->sc_nss;
        req.rx_streams_mask = (1U << sc->sc_nss) - 1;
        /*
         * A scan wants the channel changed without the calibration a
         * settled channel gets, which is most of what makes a switch
         * slow.  Anything else is an ordinary move.
         */
        req.switch_reason = (sc->sc_flags & MT7615_FLAG_SCANNING) != 0 ?
                            MT_CH_SWITCH_SCAN_BYPASS_DPD : MT_CH_SWITCH_NORMAL;
        req.band_idx = 0;
        /*
         * The band is not stated: the firmware takes it from the channel
         * number, and the reference driver leaves this field at zero for
         * both bands.
         */

        /*
         * How hard the radio may drive each rate, in half decibel-milliwatts.
         *
         * This block is not a set of flags with a sensible default: an entry
         * left at zero is a limit of zero, a milliwatt, and the whole array
         * used to go out that way.  So the radio ran at the bottom of its
         * range and only the direction the peer transmitted in performed.
         *
         * The ceiling is what net80211 works out from the regulatory domain
         * and whatever ifconfig was told, and it applies to the whole
         * radio; the entries here are per chain, so the split between the
         * chains comes off first - three decibels for the two this uses.
         */
        power = ic->ic_txpowlimit;
        power = MIN(power, 2 * c->ic_maxregpower);
        power = MIN(power, c->ic_maxpower);
        power -= MT7615_TX_POWER_PATH_DELTA;
        power = MAX(power, 0);
        power = MIN(power, MT7615_TX_POWER_MAX);

        memset(req.txpower_sku, power, MT_SKU_1SS_DELTA);
        /*
         * What a single chain may add back, now that it does not have to
         * share the budget.  Only the first entry applies with two chains.
         */
        req.txpower_sku[MT_SKU_1SS_DELTA] = MT7615_TX_POWER_PATH_DELTA;

        MT7615_DPRINTF(sc, MT7615_DEBUG_STATE,
            "channel %u centre %u, %u MHz, %d.%u dBm a chain, reason %u\n",
            chan, req.center_chan, 20 << req.bw, power / 2, (power & 1) * 5,
            req.switch_reason);

        /*
         * Calibrate for where the radio is going before telling it to go
         * there, which is the order the reference driver uses.  A failure
         * is not fatal on its own - the channel switch may still work -
         * so it is reported and the move goes ahead.
         */
        (void)mt7615_mcu_calibrate(sc,
            ieee80211_ieee2mhz(req.center_chan, c->ic_flags), req.bw);

        error = mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_CHANNEL_SWITCH, &req,
            sizeof(req), true);
        if (error != 0)
                return (error);

        /*
         * The same description again, to the receive side.
         *
         * The switch above moves the radio; this sets up the chain that
         * listens on it, and the two are separate commands taking the same
         * request.  Without it the receiver keeps whatever width it was
         * last set to, which is what the firmware came up with - twenty
         * megahertz.  The reference driver sends it when the interface
         * starts; sending it on every move is what keeps it in step with
         * a channel that changes width underneath it.
         */
        req.switch_reason = MT_CH_SWITCH_NORMAL;

        return (mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_SET_RX_PATH, &req,
            sizeof(req), true));
}

/*
 * Move to a channel, as wide as it is meant to be used.
 *
 * A width the firmware will not take leaves the radio on no channel at
 * all - it stops transmitting, the network stops being announced and
 * nothing says why - so a refusal is narrowed and tried again.  Coming
 * back at twenty megahertz is worth more than being right about eighty.
 */
int
mt7615_mcu_set_channel(struct mt7615_softc *sc, struct ieee80211_channel *c)
{
        uint8_t bw;
        int error;

        if (IEEE80211_IS_CHAN_VHT80(c) && c->ic_vht_ch_freq1 != 0)
                bw = MT_CH_BW_80;
        else if (IEEE80211_IS_CHAN_HT40(c))
                bw = MT_CH_BW_40;
        else
                bw = MT_CH_BW_20;

        for (;;) {
                error = mt7615_mcu_set_channel_bw(sc, c, bw);
                if (error == 0 || bw == MT_CH_BW_20)
                        return (error);

                device_printf(sc->sc_dev,
                    "the firmware would not take channel %u at %u MHz (%d); "
                    "trying %u\n", ieee80211_chan2ieee(&sc->sc_ic, c),
                    20 << bw, error, 20 << (bw - 1));
                bw--;
        }
}

/*
 * Hand the factory calibration data over.  The MCU can read the efuse
 * itself, which is what "buffer mode 0" asks it to do; boards that keep
 * their data elsewhere would need the driver to send the block instead.
 */
int
mt7615_mcu_set_eeprom(struct mt7615_softc *sc)
{
        struct {
            uint8_t		buffer_mode;
            uint8_t		content_format;
            uint16_t	len;
        } __packed req;

        memset(&req, 0, sizeof(req));
        req.buffer_mode = 0;	/* Read from the efuse. */

        return (mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_EFUSE_BUFFER_MODE,
            &req, sizeof(req), true));
}

/*
 * Describing a network to the firmware.
 *
 * Three things have to exist before the radio will carry traffic, and
 * they nest: a device is a hardware MAC slot, a BSS is a network built
 * on that slot, and a station record is one peer inside the network.
 * Bringing an access point up means creating all three and then handing
 * over a beacon for the firmware to repeat; tearing it down means
 * undoing them in reverse.
 *
 * The MT7615 without offload firmware wants the station's hardware
 * table entry programmed separately from its station record, so each
 * peer takes two commands.
 */

/*
 * The PHY mode byte is a bitmap of the modulations the network runs,
 * which the firmware uses to pick its basic rates.
 */
static uint8_t
mt7615_mcu_phy_mode(struct ieee80211com *ic)
{
        uint8_t mode;

        if (IEEE80211_IS_CHAN_2GHZ(ic->ic_curchan)) {
                mode = MT_PHY_MODE_11B | MT_PHY_MODE_11G;
                if (IEEE80211_IS_CHAN_HT(ic->ic_curchan))
                        mode |= MT_PHY_MODE_11GN;
        } else {
                mode = MT_PHY_MODE_11A;
                if (IEEE80211_IS_CHAN_HT(ic->ic_curchan))
                        mode |= MT_PHY_MODE_11AN;
                if (IEEE80211_IS_CHAN_VHT(ic->ic_curchan))
                        mode |= MT_PHY_MODE_11AC;
        }

        return (mode);
}

int
mt7615_mcu_add_dev(struct mt7615_softc *sc, struct mt7615_vap *mvp, bool enable)
{
        struct mt7615_dev_info_req req;

        MT7615_ASSERT_LOCKED(sc);

        memset(&req, 0, sizeof(req));
        req.hdr.omac_idx = mvp->omac_idx;
        req.hdr.band_idx = mvp->band_idx;
        req.hdr.tlv_num = htole16(1);
        req.hdr.is_tlv_append = 1;

        req.tlv.tag = htole16(MT_DEV_INFO_ACTIVE);
        req.tlv.len = htole16(sizeof(req.tlv));
        req.tlv.active = enable ? 1 : 0;
        req.tlv.band_idx = mvp->band_idx;
        IEEE80211_ADDR_COPY(req.tlv.omac_addr, mvp->iv_vap.iv_myaddr);

        return (mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_DEV_INFO_UPDATE, &req,
            sizeof(req), true));
}

int
mt7615_mcu_add_bss(struct mt7615_softc *sc, struct mt7615_vap *mvp, bool enable)
{
        struct ieee80211vap *vap;
        struct ieee80211com *ic;
        struct mt7615_bss_info_req req;
        uint32_t conn_type;

        MT7615_ASSERT_LOCKED(sc);

        vap = &mvp->iv_vap;
        ic = vap->iv_ic;

        conn_type = vap->iv_opmode == IEEE80211_M_HOSTAP ?
                    MT_CONNECTION_INFRA_AP : MT_CONNECTION_INFRA_STA;

        memset(&req, 0, sizeof(req));
        req.hdr.bss_idx = mvp->omac_idx;
        req.hdr.wlan_idx_lo = mvp->bmc_wcid;
        req.hdr.tlv_num = htole16(2);
        req.hdr.is_tlv_append = 1;
        req.hdr.muar_idx = mvp->omac_idx;

        req.omac.tag = htole16(MT_BSS_INFO_OMAC);
        req.omac.len = htole16(sizeof(req.omac));
        req.omac.hw_bss_idx = mvp->omac_idx;
        req.omac.omac_idx = mvp->omac_idx;
        req.omac.band_idx = mvp->band_idx;
        req.omac.conn_type = htole32(conn_type);

        req.basic.tag = htole16(MT_BSS_INFO_BASIC);
        req.basic.len = htole16(sizeof(req.basic));
        req.basic.network_type = htole32(MT_NETWORK_INFRA);
        req.basic.active = enable ? 1 : 0;
        req.basic.wmm_idx = mvp->wmm_idx;
        /* The broadcast pseudo-station carries anything not addressed. */
        req.basic.bmc_wcid_lo = mvp->bmc_wcid;
        req.basic.dtim_period = vap->iv_dtim_period != 0 ?
                                vap->iv_dtim_period : 1;
        req.basic.bcn_interval = htole16(ic->ic_bintval != 0 ?
                                         ic->ic_bintval : IEEE80211_BINTVAL_DEFAULT);
        req.basic.phy_mode = mt7615_mcu_phy_mode(ic);

        /*
         * An access point announces its own address; a station has to be
         * told the one it is joining.
         */
        if (vap->iv_opmode == IEEE80211_M_HOSTAP || vap->iv_bss == NULL)
                IEEE80211_ADDR_COPY(req.basic.bssid, vap->iv_myaddr);
        else
                IEEE80211_ADDR_COPY(req.basic.bssid, vap->iv_bss->ni_bssid);

        return (mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_BSS_INFO_UPDATE, &req,
            sizeof(req), true));
}

/*
 * A station's hardware table entry, which is what the MAC consults for
 * every frame to or from that peer.
 */
static int
mt7615_mcu_set_wtbl(struct mt7615_softc *sc, struct mt7615_vap *mvp,
                    const struct mt7615_peer *peer)
{
        struct mt7615_wtbl_req req;
        size_t len;
        bool bmc;

        MT7615_ASSERT_LOCKED(sc);

        /* The network's own pseudo-station rather than a peer. */
        bmc = (peer->wcid == mvp->bmc_wcid);

        memset(&req, 0, sizeof(req));
        req.hdr.wlan_idx_lo = peer->wcid;
        req.hdr.operation = MT_WTBL_RESET_AND_SET;

        req.generic.tag = htole16(MT_WTBL_GENERIC);
        req.generic.len = htole16(sizeof(req.generic));
        IEEE80211_ADDR_COPY(req.generic.peer_addr, peer->addr);
        req.generic.muar_idx = bmc ? MT_WTBL_MUAR_NONE : mvp->omac_idx;
        req.generic.qos = peer->qos ? 1 : 0;
        req.generic.partial_aid = htole16(peer->aid);

        req.rx.tag = htole16(MT_WTBL_RX);
        req.rx.len = htole16(sizeof(req.rx));
        /*
         * rca1 says a frame whose receiver address matches this entry is
         * ours.  An access point's peers are addressed to the access
         * point, not to themselves, so their entries do not claim it.
         */
        req.rx.rca1 = (bmc ||
                       mvp->iv_vap.iv_opmode != IEEE80211_M_HOSTAP) ? 1 : 0;
        req.rx.rca2 = 1;
        req.rx.rv = 1;

        req.spe.tag = htole16(MT_WTBL_SPE);
        req.spe.len = htole16(sizeof(req.spe));
        req.spe.spe_idx = MT7615_SPE_IDX_DEFAULT;

        /*
         * A peer that negotiated HT needs the aggregation limits it asked
         * for, or the hardware sends it bursts it cannot take.
         */
        if (peer->htcap != 0) {
                req.ht.tag = htole16(MT_WTBL_HT);
                req.ht.len = htole16(sizeof(req.ht));
                req.ht.ht = 1;
                req.ht.ldpc =
                    (peer->htcap & IEEE80211_HTCAP_LDPC) != 0 ? 1 : 0;
                req.ht.af = (peer->htparam & IEEE80211_HTCAP_MAXRXAMPDU) >>
                                                                         IEEE80211_HTCAP_MAXRXAMPDU_S;
                req.ht.mm = (peer->htparam & IEEE80211_HTCAP_MPDUDENSITY) >>
                                                                          IEEE80211_HTCAP_MPDUDENSITY_S;
        }

        /*
         * The records are appended in the order they are declared, so a
         * request that leaves one out is short by exactly that much and
         * one that leaves out the last two is short by both.
         */
        if (peer->vhtcap != 0) {
                req.vht.tag = htole16(MT_WTBL_VHT);
                req.vht.len = htole16(sizeof(req.vht));
                req.vht.vht = 1;
                req.vht.ldpc =
                    (peer->vhtcap & IEEE80211_VHTCAP_RXLDPC) != 0 ? 1 : 0;
                req.vht.dyn_bw = 1;
                req.hdr.tlv_num = htole16(5);
                len = sizeof(req);
        } else if (peer->htcap != 0) {
                req.hdr.tlv_num = htole16(4);
                len = sizeof(req) - sizeof(req.vht);
        } else {
                req.hdr.tlv_num = htole16(3);
                len = sizeof(req) - sizeof(req.ht) - sizeof(req.vht);
        }

        return (mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_WTBL_UPDATE, &req,
            len, true));
}

int
mt7615_mcu_sta_add(struct mt7615_softc *sc, struct mt7615_vap *mvp,
                   const struct mt7615_peer *peer, bool enable)
{
        struct mt7615_sta_rec_req req;
        uint32_t conn_type;
        size_t len;
        int error;

        MT7615_ASSERT_LOCKED(sc);

        if (enable) {
                error = mt7615_mcu_set_wtbl(sc, mvp, peer);
                if (error != 0)
                        return (error);
        }

        /*
         * The peer's type is the mirror of ours: an access point's peers
         * are stations, a station's only peer is an access point.
         */
        if (peer->wcid == mvp->bmc_wcid)
                conn_type = MT_CONNECTION_INFRA_BC;
        else if (mvp->iv_vap.iv_opmode == IEEE80211_M_HOSTAP)
                conn_type = MT_CONNECTION_INFRA_STA;
        else
                conn_type = MT_CONNECTION_INFRA_AP;

        memset(&req, 0, sizeof(req));
        req.hdr.bss_idx = mvp->omac_idx;
        req.hdr.wlan_idx_lo = peer->wcid;
        req.hdr.is_tlv_append = 1;
        req.hdr.muar_idx = mvp->omac_idx;

        req.basic.tag = htole16(MT_STA_REC_BASIC);
        req.basic.len = htole16(sizeof(req.basic));
        req.basic.conn_type = htole32(conn_type);
        req.basic.qos = peer->qos ? 1 : 0;
        req.basic.aid = htole16(peer->aid);
        IEEE80211_ADDR_COPY(req.basic.peer_addr, peer->addr);

        if (enable) {
                req.basic.conn_state = MT_CONN_STATE_PORT_SECURE;
                req.basic.extra_info = htole16(MT_EXTRA_INFO_VER |
                                               MT_EXTRA_INFO_NEW);
        } else {
                req.basic.conn_state = MT_CONN_STATE_DISCONNECT;
                req.basic.extra_info = htole16(MT_EXTRA_INFO_VER);
        }

        /*
         * Only a peer that is joining and negotiated HT gets the HT
         * record; on the way out the basic record alone says enough.
         */
        if (enable && peer->htcap != 0) {
                req.ht.tag = htole16(MT_STA_REC_HT);
                req.ht.len = htole16(sizeof(req.ht));
                req.ht.ht_cap = htole16(peer->htcap);
        }

        if (enable && peer->vhtcap != 0) {
                req.vht.tag = htole16(MT_STA_REC_VHT);
                req.vht.len = htole16(sizeof(req.vht));
                req.vht.vht_cap = htole32(peer->vhtcap);
                req.vht.vht_rx_mcs_map = htole16(peer->vht_rx_mcs);
                req.vht.vht_tx_mcs_map = htole16(peer->vht_tx_mcs);
                req.hdr.tlv_num = htole16(3);
                len = sizeof(req);
        } else if (enable && peer->htcap != 0) {
                req.hdr.tlv_num = htole16(2);
                len = sizeof(req) - sizeof(req.vht);
        } else {
                req.hdr.tlv_num = htole16(1);
                len = sizeof(req) - sizeof(req.ht) - sizeof(req.vht);
        }

        return (mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_STA_REC_UPDATE, &req,
            len, true));
}

/*
 * The hardware table's half of a block-acknowledgement session.
 *
 * The originator half carries the window: the sequence number the
 * window opens at and how far it reaches, so the transmit engine knows
 * how many frames it may have outstanding before it has to stop and
 * ask.  The recipient half instead names the peer, because tearing a
 * session down is done by matching address and traffic identifier
 * rather than by the table slot.
 */
static int
mt7615_mcu_wtbl_ba(struct mt7615_softc *sc, const struct mt7615_ba *ba,
                   bool enable, bool tx)
{
        static const uint8_t range[MT7615_BA_RANGE_COUNT] = MT7615_BA_RANGE;
        struct mt7615_wtbl_ba_req req;
        int i;

        MT7615_ASSERT_LOCKED(sc);

        memset(&req, 0, sizeof(req));
        req.hdr.wlan_idx_lo = ba->wcid;
        req.hdr.operation = MT_WTBL_SET;
        req.hdr.tlv_num = htole16(1);

        req.ba.tag = htole16(MT_WTBL_BA);
        req.ba.len = htole16(sizeof(req.ba));
        req.ba.tid = ba->tid;

        if (tx) {
                req.ba.ba_type = MT_BA_TYPE_ORIGINATOR;
                req.ba.sn = enable ? htole16(ba->ssn) : 0;
                req.ba.ba_winsize = enable ? htole16(ba->winsize) : 0;
                req.ba.ba_en = enable ? 1 : 0;

                /*
                 * Alongside the size itself the firmware wants the largest
                 * of its own steps that the window still covers.
                 */
                if (enable) {
                        for (i = MT7615_BA_RANGE_COUNT - 1; i > 0; i--) {
                                if (ba->winsize >= range[i])
                                        break;
                        }
                        req.ba.ba_winsize_idx = i;
                }
        } else {
                req.ba.ba_type = MT_BA_TYPE_RECIPIENT;
                IEEE80211_ADDR_COPY(req.ba.peer_addr, ba->addr);
                req.ba.rst_ba_tid = ba->tid;
                req.ba.rst_ba_sel = MT_RST_BA_MAC_TID_MATCH;
                req.ba.rst_ba_sb = 1;
        }

        return (mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_WTBL_UPDATE, &req,
            sizeof(req), true));
}

/*
 * Open or close a block-acknowledgement session, both halves of it.
 *
 * tx says which end this is: true when the peer has agreed to receive
 * aggregates from us, false when the peer asked to send them to us.
 * Without this the firmware acknowledges every frame on its own, which
 * is correct but leaves most of the air to the acknowledgements.
 */
int
mt7615_mcu_sta_ba(struct mt7615_softc *sc, struct mt7615_vap *mvp,
                  const struct mt7615_ba *ba, bool enable, bool tx)
{
        struct mt7615_sta_rec_ba_req req;
        int error;

        MT7615_ASSERT_LOCKED(sc);

        error = mt7615_mcu_wtbl_ba(sc, ba, enable, tx);
        if (error != 0)
                return (error);

        memset(&req, 0, sizeof(req));
        req.hdr.bss_idx = mvp->omac_idx;
        req.hdr.wlan_idx_lo = ba->wcid;
        req.hdr.tlv_num = htole16(1);
        req.hdr.is_tlv_append = 1;
        req.hdr.muar_idx = mvp->omac_idx;

        req.ba.tag = htole16(MT_STA_REC_BA);
        req.ba.len = htole16(sizeof(req.ba));
        req.ba.tid = ba->tid;
        req.ba.ba_type = tx ? MT_BA_TYPE_ORIGINATOR : MT_BA_TYPE_RECIPIENT;
        req.ba.ssn = htole16(ba->ssn);
        req.ba.winsize = htole16(ba->winsize);
        /* A bit per traffic identifier rather than a plain yes or no. */
        req.ba.ba_en = enable ? (1U << ba->tid) : 0;

        return (mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_STA_REC_UPDATE, &req,
            sizeof(req), true));
}

/*
 * Hand the beacon over.  The firmware repeats it on its own from here
 * on, so this only runs when the contents change; it is told where the
 * TIM sits so it can keep that current between handovers.
 */
int
mt7615_mcu_add_beacon(struct mt7615_softc *sc, struct mt7615_vap *mvp,
                      bool enable)
{
        struct ieee80211vap *vap;
        struct mt7615_bcn_offload_req *req;
        struct ieee80211_node *ni;
        struct mbuf *m;
        int error, len;

        MT7615_ASSERT_LOCKED(sc);

        vap = &mvp->iv_vap;

        req = malloc(sizeof(*req), M_DEVBUF, M_NOWAIT | M_ZERO);
        if (req == NULL)
                return (ENOMEM);

        req->omac_idx = mvp->omac_idx;
        req->enable = enable ? 1 : 0;
        /* The beacon comes from the firmware's own slot, not the network's. */
        req->wlan_idx = MT7615_WTBL_GLOBAL;
        req->band_idx = mvp->band_idx;

        if (!enable)
                goto send;

        ni = ieee80211_ref_node(vap->iv_bss);
        MT7615_UNLOCK(sc);
        m = ieee80211_beacon_alloc(ni);
        MT7615_LOCK(sc);
        ieee80211_free_node(ni);

        if (m == NULL) {
                free(req, M_DEVBUF);
                return (ENOBUFS);
        }

        len = m->m_pkthdr.len;
        if (MT_TXD_SIZE + len > MT7615_BCN_PKT_MAX) {
                device_printf(sc->sc_dev,
                    "the beacon is %d bytes, which will not fit in %d\n",
                    (int)MT_TXD_SIZE + len, MT7615_BCN_PKT_MAX);
                m_freem(m);
                free(req, M_DEVBUF);
                return (E2BIG);
        }

        mt7615_mac_write_txd_bcn(sc, (uint32_t *)req->pkt, mvp, len);
        m_copydata(m, 0, len, (caddr_t)(req->pkt + MT_TXD_SIZE));

        req->pkt_len = htole16(MT_TXD_SIZE + len);
        /*
         * bo_tim points into the frame rather than counting from its
         * start, so turn it into the offset the firmware wants.
         */
        if (vap->iv_bcn_off.bo_tim != NULL)
                req->tim_ie_pos = htole16(MT_TXD_SIZE +
                                          (vap->iv_bcn_off.bo_tim - mtod(m, uint8_t *)));

        m_freem(m);

send:
        error = mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_BCN_OFFLOAD, req,
            sizeof(*req), true);
        free(req, M_DEVBUF);

        return (error);
}

/*
 * Channel access parameters for one access category.
 *
 * net80211 hands over the contention window as a power-of-two minus
 * one, the way it goes on the air; the firmware wants the exponent
 * back, so each bound is turned into its bit position.
 */
int
mt7615_mcu_set_wmm(struct mt7615_softc *sc, int ac,
                   const struct wmeParams *wmep)
{
        struct {
            uint8_t		number;
            uint8_t		rsv[3];
            uint8_t		queue;
            uint8_t		valid;
            uint8_t		aifs;
            uint8_t		cw_min;
            uint16_t	cw_max;
            uint16_t	txop;
        } __packed req;

        MT7615_ASSERT_LOCKED(sc);

        memset(&req, 0, sizeof(req));
        req.number = 1;
        req.queue = ac;
        req.valid = MT_WMM_PARAM_SET;
        req.aifs = wmep->wmep_aifsn;
        req.cw_min = wmep->wmep_logcwmin;
        req.cw_max = htole16(wmep->wmep_logcwmax);
        req.txop = htole16(wmep->wmep_txopLimit);

        return (mt7615_mcu_send_msg(sc, 0, MCU_EXT_CMD_EDCA_UPDATE, &req,
            sizeof(req), true));
}
