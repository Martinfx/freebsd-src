/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * MediaTek MT7615E: MAC bring-up, the Rx descriptor parser and the Tx
 * descriptor builder.
 *
 * Rx frames arrive with a variable-length descriptor in front of them.
 * The first four words are always there and say what else follows: a
 * group 4 block with the frame control and sequence fields, a group 1
 * block with the addresses, a group 2 block with timestamps, and a
 * group 3 block with the receive vector the signal strength comes from.
 * The parser walks those in order and then hands the 802.11 frame that
 * follows to net80211.
 *
 * Tx frames go the other way: a descriptor is built in front of the
 * frame describing the queue, the station, the rate policy and the
 * sequence number, followed by a pointer block listing the payload's
 * physical fragments.
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

/*
 * MAC initialisation.  The values come from the Linux mt76 driver's
 * mt7615_mac_init(); MediaTek documents none of them.
 */
static void
mt7615_init_mac_chain(struct mt7615_softc *sc, int chain)
{
        uint32_t val;

        if (chain == 0)
                val = MT_CFG_CCR_MAC_D0_1X_GC_EN | MT_CFG_CCR_MAC_D0_2X_GC_EN;
        else
                val = MT_CFG_CCR_MAC_D1_1X_GC_EN | MT_CFG_CCR_MAC_D1_2X_GC_EN;

        /* Enable the band's clock. */
        mt7615_set(sc, MT_CFG_CCR, val);

        mt7615_rmw(sc, MT_TMAC_TRCR(chain),
            MT_TMAC_TRCR_CCA_SEL | MT_TMAC_TRCR_SEC_CCA_SEL,
            (2U << MT_TMAC_TRCR_CCA_SEL_S) & MT_TMAC_TRCR_CCA_SEL);

        mt7615_wr(sc, MT_AGG_ACR(chain),
            MT_AGG_ACR_PKT_TIME_EN | MT_AGG_ACR_NO_BA_AR_RULE |
            ((MT7615_CFEND_RATE_DEFAULT << MT_AGG_ACR_CFEND_RATE_S) &
             MT_AGG_ACR_CFEND_RATE) |
            ((MT7615_BAR_RATE_DEFAULT << MT_AGG_ACR_BAR_RATE_S) &
             MT_AGG_ACR_BAR_RATE));

        mt7615_wr(sc, MT_AGG_ARUCR(chain),
            MT_AGG_ARxCR_LIMIT(0, 7) | MT_AGG_ARxCR_LIMIT(1, 2) |
            MT_AGG_ARxCR_LIMIT(2, 2) | MT_AGG_ARxCR_LIMIT(3, 2) |
            MT_AGG_ARxCR_LIMIT(4, 1) | MT_AGG_ARxCR_LIMIT(5, 1) |
            MT_AGG_ARxCR_LIMIT(6, 1) | MT_AGG_ARxCR_LIMIT(7, 1));

        mt7615_wr(sc, MT_AGG_ARDCR(chain),
            MT_AGG_ARxCR_LIMIT(0, MT7615_RATE_RETRY - 1) |
            MT_AGG_ARxCR_LIMIT(1, MT7615_RATE_RETRY - 1) |
            MT_AGG_ARxCR_LIMIT(2, MT7615_RATE_RETRY - 1) |
            MT_AGG_ARxCR_LIMIT(3, MT7615_RATE_RETRY - 1) |
            MT_AGG_ARxCR_LIMIT(4, MT7615_RATE_RETRY - 1) |
            MT_AGG_ARxCR_LIMIT(5, MT7615_RATE_RETRY - 1) |
            MT_AGG_ARxCR_LIMIT(6, MT7615_RATE_RETRY - 1) |
            MT_AGG_ARxCR_LIMIT(7, MT7615_RATE_RETRY - 1));

        mt7615_clear(sc, MT_DMA_RCFR0(chain), MT_DMA_RCFR0_MCU_RX_TDLS);

        /*
         * Without the offload firmware the MCU wants management and
         * control frames routed to it, and unwanted unicast and
         * multicast dropped in hardware.
         */
        mt7615_rmw(sc, MT_DMA_RCFR0(chain),
            MT_DMA_RCFR0_MCU_RX_MGMT | MT_DMA_RCFR0_MCU_RX_CTL_NON_BAR |
            MT_DMA_RCFR0_MCU_RX_CTL_BAR | MT_DMA_RCFR0_MCU_RX_BYPASS |
            MT_DMA_RCFR0_RX_DROPPED_UCAST | MT_DMA_RCFR0_RX_DROPPED_MCAST,
            ((2U << MT_DMA_RCFR0_RX_DROPPED_UCAST_S) &
             MT_DMA_RCFR0_RX_DROPPED_UCAST) |
            ((2U << MT_DMA_RCFR0_RX_DROPPED_MCAST_S) &
             MT_DMA_RCFR0_RX_DROPPED_MCAST));
}

int
mt7615_mac_init(struct mt7615_softc *sc)
{

        MT7615_ASSERT_LOCKED(sc);

        mt7615_init_mac_chain(sc, 0);

        mt7615_rmw(sc, MT_TMAC_CTCR0,
            MT_TMAC_CTCR0_INS_DDLMT_REFTIME | MT_TMAC_CTCR0_INS_DDLMT_DENSITY,
            ((0x3fU << MT_TMAC_CTCR0_INS_DDLMT_REFTIME_S) &
             MT_TMAC_CTCR0_INS_DDLMT_REFTIME) |
            ((0x3U << MT_TMAC_CTCR0_INS_DDLMT_DENSITY_S) &
             MT_TMAC_CTCR0_INS_DDLMT_DENSITY));
        mt7615_set(sc, MT_TMAC_CTCR0,
            MT_TMAC_CTCR0_INS_DDLMT_VHT_SMPDU_EN |
            MT_TMAC_CTCR0_INS_DDLMT_EN);

        mt7615_set(sc, MT_AGG_SCR, MT_AGG_SCR_NLNAV_MID_PTEC_DIS);

        mt7615_wr(sc, MT_AGG_ARCR,
            ((2U << MT_AGG_ARCR_RTS_RATE_THR_S) & MT_AGG_ARCR_RTS_RATE_THR) |
            MT_AGG_ARCR_RATE_DOWN_RATIO_EN |
            ((1U << MT_AGG_ARCR_RATE_DOWN_RATIO_S) &
             MT_AGG_ARCR_RATE_DOWN_RATIO) |
            ((4U << MT_AGG_ARCR_RATE_UP_EXTRA_TH_S) &
             MT_AGG_ARCR_RATE_UP_EXTRA_TH));

        mt7615_set(sc, MT_WF_RMAC_MIB_TIME0, MT_WF_RMAC_MIB_RXTIME_EN);
        mt7615_set(sc, MT_WF_RMAC_MIB_AIRTIME0, MT_WF_RMAC_MIB_RXTIME_EN);

        mt7615_wr(sc, MT_DMA_DCR0,
            ((3072U << MT_DMA_DCR0_MAX_RX_LEN_S) & MT_DMA_DCR0_MAX_RX_LEN) |
            MT_DMA_DCR0_RX_VEC_DROP | MT_DMA_DCR0_DAMSDU_EN);

        /* No TDLS support, so leave its filtering out of the way. */
        mt7615_clear(sc, MT_WF_PFCR, MT_WF_PFCR_TDLS_EN);
        mt7615_set(sc, MT_WF_MIB_SCR0, MT_MIB_SCR0_AGG_CNT_RANGE_EN);

        mt7615_init_mac_chain(sc, 1);

        /* The radio must not sleep between beacons while associating. */
        mt7615_set(sc, MT_WF_PHY_WF2_RFCTRL0(0),
            MT_WF_PHY_WF2_RFCTRL0_LPBCN_EN);
        mt7615_set(sc, MT_WF_PHY_WF2_RFCTRL0(1),
            MT_WF_PHY_WF2_RFCTRL0_LPBCN_EN);

        mt7615_mac_set_rxfilter(sc);
        mt7615_mac_set_timing(sc);

        return (0);
}

/*
 * Interframe spacing and the detection timeouts.
 *
 * These are not set up by anything else: the firmware leaves them at
 * whatever the reset left, and the channel switch command puts them
 * back there, so they have to be written after every move.  The values
 * belong to the band rather than to the width - five gigahertz waits
 * longer between frames than two and four does - and the arbiter is
 * stopped while they change so that nothing is mid-frame under them.
 */
void
mt7615_mac_set_timing(struct mt7615_softc *sc)
{
        const struct ieee80211_channel *c;
        uint32_t cck, ofdm, val;
        int sifs, slot;

        MT7615_ASSERT_LOCKED(sc);

        c = sc->sc_ic.ic_curchan;
        if (c == NULL)
                return;

        if (IEEE80211_IS_CHAN_2GHZ(c)) {
                sifs = 10;
                slot = 20;
        } else {
                sifs = 16;
                slot = 9;
        }

        cck = ((231U << MT_TIMEOUT_VAL_PLCP_S) & MT_TIMEOUT_VAL_PLCP) |
              ((48U << MT_TIMEOUT_VAL_CCA_S) & MT_TIMEOUT_VAL_CCA);
        ofdm = ((60U << MT_TIMEOUT_VAL_PLCP_S) & MT_TIMEOUT_VAL_PLCP) |
               ((28U << MT_TIMEOUT_VAL_CCA_S) & MT_TIMEOUT_VAL_CCA);

        mt7615_set(sc, MT_ARB_SCR,
            MT_ARB_SCR_TX0_DISABLE | MT_ARB_SCR_RX0_DISABLE);
        DELAY(1);

        mt7615_wr(sc, MT_TMAC_CDTR, cck);
        mt7615_wr(sc, MT_TMAC_ODTR, ofdm);

        val = ((360U << MT_IFS_EIFS_S) & MT_IFS_EIFS) |
              ((2U << MT_IFS_RIFS_S) & MT_IFS_RIFS) |
              (((uint32_t)sifs << MT_IFS_SIFS_S) & MT_IFS_SIFS) |
              (((uint32_t)slot << MT_IFS_SLOT_S) & MT_IFS_SLOT);
        mt7615_wr(sc, MT_TMAC_ICR(0), val);

        /*
         * What a contention-free period ends at.  Only a two gigahertz
         * network with the long slot time falls back to the 11b rate.
         */
        mt7615_rmw(sc, MT_AGG_ACR(0), MT_AGG_ACR_CFEND_RATE,
            ((slot < 20 || !IEEE80211_IS_CHAN_2GHZ(c) ?
              MT7615_CFEND_RATE_DEFAULT : MT7615_CFEND_RATE_11B) <<
                                                                 MT_AGG_ACR_CFEND_RATE_S) & MT_AGG_ACR_CFEND_RATE);

        mt7615_clear(sc, MT_ARB_SCR,
            MT_ARB_SCR_TX0_DISABLE | MT_ARB_SCR_RX0_DISABLE);
}

/*
 * Receive filter.  net80211 asks for promiscuous or monitor operation
 * through the com flags, so translate those into what the RMAC drops.
 */
void
mt7615_mac_set_rxfilter(struct mt7615_softc *sc)
{
        struct ieee80211com *ic;
        uint32_t filter;

        MT7615_ASSERT_LOCKED(sc);

        ic = &sc->sc_ic;

        /*
         * What the hardware throws away before the driver ever sees it.
         *
         * Note what is deliberately absent.  Probe requests have to reach
         * an access point or it can never answer one, which is most of
         * what a peer does before it tries to join.  Duplicates, group
         * addresses and frames from other networks are net80211's to sort
         * out, and the reference driver lets all of them through in every
         * mode; dropping them here only hides frames the stack wants.
         */
        filter = MT_WF_RFCR_DROP_FCSFAIL |
                 MT_WF_RFCR_DROP_A3_MAC |
                 MT_WF_RFCR_DROP_A3_BSSID |
                 MT_WF_RFCR_DROP_OTHER_TIM |
                 MT_WF_RFCR_DROP_CTS |
                 MT_WF_RFCR_DROP_RTS |
                 MT_WF_RFCR_DROP_CTL_RSV |
                 MT_WF_RFCR_DROP_NDPA;

        /*
         * Beacons from other networks are worth having while scanning,
         * and to an access point keeping an eye on its neighbours.
         */
        if (ic->ic_opmode != IEEE80211_M_HOSTAP &&
            ic->ic_opmode != IEEE80211_M_MONITOR &&
            (sc->sc_flags & MT7615_FLAG_SCANNING) == 0)
                filter |= MT_WF_RFCR_DROP_OTHER_BEACON;

        /* Monitor mode, a scan and promiscuous mode want the lot. */
        if (ic->ic_opmode == IEEE80211_M_MONITOR ||
            (sc->sc_flags & MT7615_FLAG_SCANNING) != 0 ||
            ic->ic_promisc > 0)
                filter &= ~(MT_WF_RFCR_DROP_A3_MAC |
                            MT_WF_RFCR_DROP_A3_BSSID | MT_WF_RFCR_DROP_OTHER_TIM);

        mt7615_wr(sc, MT_WF_RFCR(0), filter);

        /*
         * Control frames the hardware answers by itself.  Passing them up
         * would fill the receive ring with frames net80211 discards.
         */
        mt7615_wr(sc, MT_WF_RFCR1(0),
            MT_WF_RFCR1_DROP_ACK | MT_WF_RFCR1_DROP_BF_POLL |
            MT_WF_RFCR1_DROP_BA | MT_WF_RFCR1_DROP_CFEND |
            MT_WF_RFCR1_DROP_CFACK);
}

/*
 * Rx.
 */

static int
mt7615_mac_rx_rate(uint32_t rxv1)
{
        static const uint8_t cck_rates[] = { 2, 4, 11, 22 };
        static const uint8_t ofdm_rates[] = {
            96, 48, 24, 12, 108, 72, 36, 18
        };
        uint8_t idx, mode;

        idx = rxv1 & MT_RXV1_TX_RATE;
        mode = (rxv1 & MT_RXV1_TX_MODE) >> MT_RXV1_TX_MODE_S;

        switch (mode) {
                case 0:	/* CCK */
                        if ((idx & 0x3) < nitems(cck_rates))
                                return (cck_rates[idx & 0x3]);
                        break;
                case 1:	/* OFDM */
                        if ((idx & 0x7) < nitems(ofdm_rates))
                                return (ofdm_rates[idx & 0x7]);
                        break;
                default:
                        /* HT and VHT report an MCS, which net80211 wants separately. */
                        break;
        }

        return (0);
}

/*
 * Strip the receive descriptor off the front of a frame, pulling the
 * signal strength and the channel out of it on the way, and leave the
 * mbuf pointing at the 802.11 header.  Returns nonzero if the frame
 * should be dropped.
 */
static int
mt7615_mac_fill_rx(struct mt7615_softc *sc, struct mbuf *m, int *rssi,
                   int *rate, bool *decrypted)
{
        const uint32_t *rxd;
        uint32_t rxd0, rxd1, rxd2;
        int hdrlen, len, off;

        *rssi = sc->sc_noise;
        *rate = 0;
        *decrypted = false;

        if (m->m_pkthdr.len < 4 * (int)sizeof(uint32_t))
                return (EINVAL);

        rxd = mtod(m, const uint32_t *);
        rxd0 = le32toh(rxd[0]);
        rxd1 = le32toh(rxd[1]);
        rxd2 = le32toh(rxd[2]);

        if ((rxd2 & (MT_RXD2_NORMAL_AMSDU_ERR |
                     MT_RXD2_NORMAL_MAX_LEN_ERROR | MT_RXD2_NORMAL_LEN_MISMATCH)) != 0)
                return (EINVAL);

        /*
         * Whether the hardware has already done the cipher.  It takes the
         * header and the trailer off with it, so net80211 has to be told
         * not to take them off a second time - and taking sixteen bytes
         * out of the middle of every frame is not something that shows up
         * as a decryption failure, it just quietly ruins the traffic.
         */
        if (((rxd2 & MT_RXD2_NORMAL_SEC_MODE) >> MT_RXD2_NORMAL_SEC_MODE_S) !=
            MT_CIPHER_NONE &&
            (rxd2 & (MT_RXD2_NORMAL_CLM | MT_RXD2_NORMAL_CM)) == 0)
                *decrypted = true;

        if ((rxd2 & MT_RXD2_NORMAL_FCS_ERR) != 0)
                return (EINVAL);

        off = 4 * sizeof(uint32_t);

        /* Group 4: frame control, addresses and sequence control. */
        if ((rxd0 & MT_RXD0_NORMAL_GROUP_4) != 0)
                off += 4 * sizeof(uint32_t);
        /* Group 1: the addresses again, in the LMAC's own order. */
        if ((rxd0 & MT_RXD0_NORMAL_GROUP_1) != 0)
                off += 4 * sizeof(uint32_t);
        /* Group 2: timestamps. */
        if ((rxd0 & MT_RXD0_NORMAL_GROUP_2) != 0)
                off += 2 * sizeof(uint32_t);

        /* Group 3: the receive vector, which carries the signal strength. */
        if ((rxd0 & MT_RXD0_NORMAL_GROUP_3) != 0) {
                if (m->m_pkthdr.len < off + 6 * (int)sizeof(uint32_t))
                        return (EINVAL);

                rxd = mtod(m, const uint32_t *) + off / sizeof(uint32_t);
                *rate = mt7615_mac_rx_rate(le32toh(rxd[0]));
                /*
                 * The in-band RSSI is reported as a positive magnitude
                 * below zero dBm.
                 */
                *rssi = -(int8_t)((le32toh(rxd[2]) & MT_RXV3_IB_RSSI) >>
                                                                      MT_RXV3_IB_RSSI_S);

                off += 6 * sizeof(uint32_t);
        }

        /*
         * The LMAC pads the 802.11 header out to a word boundary when
         * asked to, and says so in the descriptor.
         */
        if ((rxd1 & MT_RXD1_NORMAL_HDR_OFFSET) != 0)
                off += 2;

        len = (rxd0 & MT_RXD0_LENGTH);
        if (len < off || len > m->m_pkthdr.len)
                return (EINVAL);

        hdrlen = len - off;
        if (hdrlen < (int)sizeof(struct ieee80211_frame_min))
                return (EINVAL);

        m_adj(m, off);
        m->m_pkthdr.len = m->m_len = hdrlen;

        return (0);
}

/*
 * Tokens the firmware is handing back after a frame has been sent.  The
 * driver reclaims the Tx ring by descriptor index instead, so nothing
 * is owed here beyond noticing the event.
 */
void
mt7615_mac_tx_free(struct mt7615_softc *sc, struct mbuf *m)
{
        const struct mt7615_tx_free *ev;
        const uint16_t *token;
        int i, count, len;

        MT7615_ASSERT_LOCKED(sc);

        len = m->m_pkthdr.len;
        if (len < (int)sizeof(*ev))
                goto out;

        ev = mtod(m, const struct mt7615_tx_free *);
        count = le16toh(ev->ctrl) & MT_TX_FREE_MSDU_ID_CNT;

        /* A run of tokens follows the header; do not read past the end. */
        if ((int)sizeof(*ev) + count * (int)sizeof(uint16_t) > len)
                goto out;

        MT7615_DPRINTF(sc, MT7615_DEBUG_TX, "the firmware returned %d "
                                            "frame%s\n", count, count == 1 ? "" : "s");

        token = (const uint16_t *)(const void *)(ev + 1);
        for (i = 0; i < count; i++)
                mt7615_token_put(sc, le16toh(token[i]));

        /* Room may have come free, so anything held back can go now. */
        mt7615_start(sc);

out:
        m_freem(m);
}

/*
 * One transmit status report.
 *
 * Runs in the interrupt thread under the driver lock, so it does no
 * more than take the report apart and add it to the slot's running
 * totals; carrying those over to net80211 is another thread's job.
 *
 * A report describes one MPDU, aggregated or not, and says how many
 * attempts it took and what rate the last of them used.  Two of the
 * failure flags mean the frame never reached the air on its own terms -
 * the medium was never won, or the frame aged out of the queue - and
 * those say nothing about whether the rate was right, so they are left
 * out of the totals rather than counted as the rate's fault.
 */
static void
mt7615_mac_add_txs(struct mt7615_softc *sc, const uint32_t *raw)
{
        struct mt7615_txs_stats *st;
        uint32_t txs0, txs1, txs3;
        uint16_t rate;
        uint8_t wcid, pid, count;
        bool ampdu, acked;

        MT7615_ASSERT_LOCKED(sc);

        txs0 = le32toh(raw[0]);
        txs1 = le32toh(raw[1]);
        txs3 = le32toh(raw[3]);

        pid = (txs0 & MT_TXS0_PID) >> MT_TXS0_PID_S;
        if (pid == MT7615_PID_NO_STATUS)
                return;

        wcid = (le32toh(raw[2]) & MT_TXS2_WCID) >> MT_TXS2_WCID_S;
        if (wcid < MT7615_WTBL_STA_FIRST || wcid >= MT7615_WTBL_STA_LIMIT) {
                sc->sc_txs_dropped++;
                return;
        }

        st = &sc->sc_txs[wcid];
        sc->sc_txs_events++;

        ampdu = (txs1 & MT_TXS1_AMPDU) != 0;

        /*
         * A frame that timed out waiting for the medium, or that sat in
         * the queue until it expired, was never given a fair try at the
         * rate it names.  Counting it would make the rate look worse than
         * it is and drag the estimate down for a reason that has nothing
         * to do with the rate.
         */
        if ((txs0 & MT_TXS0_QUEUE_TIMEOUT) != 0)
                return;
        if (!ampdu && (txs0 & MT_TXS0_RTS_TIMEOUT) != 0)
                return;

        acked = (txs0 & MT_TXS0_ACK_TIMEOUT) == 0 &&
                (txs0 & MT_TXS0_BA_ERROR) == 0;

        /*
         * The count is attempts, not retries, and the hardware reports at
         * least one for a frame it sent.  A report claiming none is not
         * something to subtract from.
         */
        count = (txs3 & MT_TXS3_TX_COUNT) >> MT_TXS3_TX_COUNT_S;
        if (count == 0)
                count = 1;

        st->nframes++;
        if (acked)
                st->nsuccess++;
        st->nretries += count - 1;

        /*
         * Which rate the frame ended on.  A frame the descriptor pinned to
         * one rate says nothing about where the ladder has got to - that
         * is management and group traffic, which is deliberately sent slow
         * - so it is counted but not allowed to move the reported rate.
         */
        if ((txs0 & MT_TXS0_FIXED_RATE) != 0)
                return;

        rate = txs0 & MT_TXS0_TX_RATE;
        switch ((rate & MT_TX_RATE_MODE) >> MT_TX_RATE_MODE_S) {
                case MT_PHY_TYPE_CCK:
                case MT_PHY_TYPE_OFDM:
                case MT_PHY_TYPE_HT:
                case MT_PHY_TYPE_HT_GF:
                case MT_PHY_TYPE_VHT:
                        st->rate = rate;
                        break;
                default:
                        /* A modulation this chip does not have; leave the last one. */
                        break;
        }
}

/*
 * A batch of transmit status reports.  They arrive packed behind the
 * one word of receive descriptor that names the event, as many as the
 * firmware had ready; a partial one at the end is not a report.
 */
void
mt7615_mac_rx_txs(struct mt7615_softc *sc, struct mbuf *m)
{
        const uint32_t *raw;
        int i, n;

        MT7615_ASSERT_LOCKED(sc);

        if (sc->sc_txs_div == 0)
                goto out;

        /*
         * Everything is read straight out of the mbuf, so it has to be
         * one piece.  The receive ring hands over single clusters, but a
         * short read past the end is not worth risking on that alone.
         */
        if (m->m_len != m->m_pkthdr.len)
                goto out;

        n = (m->m_pkthdr.len - (int)sizeof(uint32_t)) / MT7615_TXS_SIZE;
        raw = mtod(m, const uint32_t *) + 1;

        for (i = 0; i < n; i++, raw += MT7615_TXS_WORDS)
                mt7615_mac_add_txs(sc, raw);

out:
        m_freem(m);
}

/*
 * One frame off the data Rx ring.  The lock is held on entry and is
 * dropped around the hand-off to net80211, which may re-enter the
 * driver through the transmit path.
 */
void
mt7615_mac_rx_event(struct mt7615_softc *sc, struct mbuf *m)
{
        struct ieee80211com *ic;
        struct ieee80211_node *ni;
        struct ieee80211_frame *wh;
        struct ieee80211_rx_stats rxs;
        const uint32_t *rxd;
        uint32_t rxd0;
        int rate, rssi, type;
        bool decrypted;

        MT7615_ASSERT_LOCKED(sc);

        ic = &sc->sc_ic;

        if (m->m_pkthdr.len < (int)sizeof(uint32_t)) {
                m_freem(m);
                return;
        }

        rxd = mtod(m, const uint32_t *);
        rxd0 = le32toh(rxd[0]);
        type = (rxd0 & MT_RXD0_PKT_TYPE) >> MT_RXD0_PKT_TYPE_S;

        /*
         * An event with flag 1 is an ordinary frame the MCU forwarded
         * rather than a reply, so it is parsed like any other.
         */
        if (type == MT_PKT_TYPE_RX_EVENT &&
            ((rxd0 & MT_RXD0_PKT_FLAG) >> MT_RXD0_PKT_FLAG_S) == 1)
                type = MT_PKT_TYPE_NORMAL_MCU;

        switch (type) {
                case MT_PKT_TYPE_TXRX_NOTIFY:
                        mt7615_mac_tx_free(sc, m);
                        return;
                case MT_PKT_TYPE_RX_EVENT:
                        mt7615_mcu_rx_event(sc, m);
                        return;
                case MT_PKT_TYPE_TXS:
                        /*
                         * The firmware puts these on the command ring, not this
                         * one, but nothing says it has to, and dropping them here
                         * would lose the reports rather than misread them.
                         */
                        mt7615_mac_rx_txs(sc, m);
                        return;
                case MT_PKT_TYPE_NORMAL:
                case MT_PKT_TYPE_NORMAL_MCU:
                        break;
                default:
                        m_freem(m);
                        return;
        }

        if (mt7615_mac_fill_rx(sc, m, &rssi, &rate, &decrypted) != 0) {
                m_freem(m);
                return;
        }

        if (ieee80211_radiotap_active(ic)) {
                struct mt7615_rx_radiotap_header *tap;

                tap = &sc->sc_rxtap;
                tap->wr_tsft = 0;
                tap->wr_flags = 0;
                tap->wr_rate = rate;
                tap->wr_chan_freq = htole16(ic->ic_curchan->ic_freq);
                tap->wr_chan_flags = htole16(ic->ic_curchan->ic_flags);
                tap->wr_dbm_antsignal = rssi;
                tap->wr_dbm_antnoise = sc->sc_noise;
        }

        /*
         * What the hardware did to the frame before handing it over.
         * Without this net80211 assumes it did nothing.
         */
        memset(&rxs, 0, sizeof(rxs));
        rxs.r_flags = IEEE80211_R_NF | IEEE80211_R_RSSI;
        rxs.c_nf = sc->sc_noise;
        rxs.c_rssi = rssi - sc->sc_noise;
        if (decrypted)
                rxs.c_pktflags = IEEE80211_RX_F_DECRYPTED |
                                 IEEE80211_RX_F_IV_STRIP | IEEE80211_RX_F_MIC_STRIP;
        if (ieee80211_add_rx_params(m, &rxs) == 0) {
                m_freem(m);
                return;
        }

        wh = mtod(m, struct ieee80211_frame *);

        MT7615_UNLOCK(sc);
        ni = ieee80211_find_rxnode(ic, (struct ieee80211_frame_min *)wh);
        if (ni != NULL) {
                (void)ieee80211_input(ni, m, rssi - sc->sc_noise,
                    sc->sc_noise);
                ieee80211_free_node(ni);
        } else {
                (void)ieee80211_input_all(ic, m, rssi - sc->sc_noise,
                    sc->sc_noise);
        }
        MT7615_LOCK(sc);
}

/*
 * Tx.
 *
 * The LMAC numbers its access-category queues in a different order
 * than net80211 does, so the two have to be translated rather than
 * used interchangeably.
 */
static const uint8_t mt7615_lmac_ac[WME_NUM_AC] = {
    [WME_AC_BK] = 0,
    [WME_AC_BE] = 1,
    [WME_AC_VI] = 2,
    [WME_AC_VO] = 3,
};

/*
 * Rate control.
 *
 * The hardware runs its own rate adaptation, but only over a ladder it
 * is given: eight slots in the station's table entry, which it works
 * its way down as a frame fails and back up as frames succeed.  An
 * entry that was never filled in leaves it with nothing to choose
 * from, and it sends everything at the slowest rate it has - which is
 * why an otherwise healthy link crawls in the direction the driver
 * sends.
 *
 * Fixing a rate in the descriptor instead would take the adaptation
 * away entirely: that says this rate and no other, so a frame that
 * does not get through is retried at the same rate until its budget
 * runs out rather than dropping to something the air can carry.  That
 * is right only for a frame the ladder cannot describe, and the ladder
 * describes one peer, so the exceptions are management frames and
 * group-addressed traffic.
 */
static uint16_t
mt7615_rate_val(uint8_t mode, uint8_t idx, uint8_t nss)
{

        return ((((nss - 1) << MT_TX_RATE_NSS_S) & MT_TX_RATE_NSS) |
                ((mode << MT_TX_RATE_MODE_S) & MT_TX_RATE_MODE) |
                (idx & MT_TX_RATE_IDX));
}

/*
 * The rate for a frame that cannot use the ladder.
 *
 * Management frames and group-addressed traffic go to peers whose
 * capabilities are either unknown or various, so they are sent at the
 * slowest rate the band has.  This is also what keeps them out of the
 * ladder: the ladder belongs to one peer and is built from what that
 * peer negotiated, and an authentication reply sent at a peer's
 * negotiated modulation reaches a peer that has not yet agreed to it
 * only by luck.
 */
static uint16_t
mt7615_basic_rate_val(const struct ieee80211vap *vap)
{
        const struct ieee80211_channel *c;

        c = vap->iv_ic->ic_curchan;
        if (c != NULL && IEEE80211_IS_CHAN_2GHZ(c))
                return (mt7615_rate_val(MT_PHY_TYPE_CCK,
                    MT7615_CCK_RATE_1M, 1));

        return (mt7615_rate_val(MT_PHY_TYPE_OFDM, MT7615_OFDM_RATE_6M, 1));
}

/*
 * Build the ladder for one peer, fastest first.  A peer that
 * negotiated HT gets modulation and coding indices; one that did not
 * gets the OFDM rates, whose indices follow the signalling field
 * rather than ascending speed and so are written out rather than
 * computed.
 *
 * An index above seven means a second spatial stream, which roughly
 * doubles what the peer can take but needs the better of the two
 * antennas' paths to be good.  So a peer that offered one gets the top
 * two rungs on two streams and the lower two on one: a link that
 * cannot hold the second stream falls off it in a single step rather
 * than working its way down through rungs it will never make.
 */
static void
mt7615_rate_ladder(const struct mt7615_peer *peer, uint16_t val[4])
{
        static const uint8_t ofdm[4] = { 12, 13, 9, 11 };	/* 54, 36, 24, 6 */
        static const uint8_t mcs_2ss[4] = { 15, 13, 7, 0 };
        static const uint8_t mcs_1ss[4] = { 7, 5, 3, 0 };
        const uint8_t *mcs;
        uint8_t idx;
        int i;

        /*
         * A peer that negotiated 802.11ac is described by its own
         * modulations rather than by an index that carries the stream
         * count with it, so the ladder walks the indices down and holds
         * the streams at what the peer agreed to until the last rung,
         * which drops to one for the same reason the HT ladder does.
         */
        if (peer->vhtcap != 0 && peer->vht_nss >= 1) {
                static const uint8_t vht[4] = { 9, 7, 4, 0 };

                for (i = 0; i < 4; i++)
                        val[i] = mt7615_rate_val(MT_PHY_TYPE_VHT, vht[i],
                            i < 2 ? peer->vht_nss : 1);
                return;
        }

        /*
         * Every HT peer has to understand the first eight indices, so a
         * ceiling below that is a rate set that did not come out right
         * and the legacy rates are the safe answer.
         */
        if (peer->htcap == 0 || peer->maxmcs < 7) {
                for (i = 0; i < 4; i++)
                        val[i] = mt7615_rate_val(MT_PHY_TYPE_OFDM, ofdm[i], 1);
                return;
        }

        mcs = peer->maxmcs >= 15 ? mcs_2ss : mcs_1ss;
        for (i = 0; i < 4; i++) {
                idx = MIN(mcs[i], peer->maxmcs);
                val[i] = mt7615_rate_val(MT_PHY_TYPE_HT, idx,
                    1 + (idx / 8));
        }
}

/*
 * Hand the ladder to the hardware.  This is registers rather than a
 * command, but the table is shared with the firmware, so the busy bit
 * has to be clear before and the update announced after.
 */

/*
 * Install a peer's key in its table entry, or take it out again.
 *
 * The key material is eight words at the end of the entry.  What says
 * it is there sits in the first two words, and those cannot be written
 * where they lie: they go through a pair of staging registers and are
 * copied in when the update is announced.  The cipher goes in the third
 * word, which is written directly.
 *
 * With this in place the hardware does the encryption, and every frame
 * stops passing through the AES that the stack would otherwise run on
 * the processor.
 */
int
mt7615_mac_set_key(struct mt7615_softc *sc, uint8_t wcid, uint8_t cipher,
                   uint8_t keyidx, const uint8_t *key, int keylen)
{
        uint32_t addr, w0, w1;
        uint8_t buf[MT_WTBL_KEY_SIZE];
        int i;

        MT7615_ASSERT_LOCKED(sc);

        if (keylen > (int)sizeof(buf))
                return (EINVAL);

        addr = MT_WTBL_ENTRY(wcid);

        if (!mt7615_poll(sc, MT_WTBL_UPDATE, MT_WTBL_UPDATE_BUSY, 0, 50))
                return (ETIMEDOUT);

        /* The cipher, in the word that holds it. */
        mt7615_rmw(sc, addr + MT_WTBL_W2, MT_WTBL_W2_KEY_TYPE,
            ((uint32_t)cipher << MT_WTBL_W2_KEY_TYPE_S) & MT_WTBL_W2_KEY_TYPE);

        /* The key itself, zero-padded to the width of the field. */
        memset(buf, 0, sizeof(buf));
        if (key != NULL && keylen > 0)
                memcpy(buf, key, keylen);
        for (i = 0; i < MT_WTBL_KEY_SIZE; i += 4) {
                uint32_t val;

                memcpy(&val, buf + i, sizeof(val));
                mt7615_wr(sc, addr + MT_WTBL_W30 + i, le32toh(val));
        }

        /*
         * The first two words, through the staging pair.  Taking a key
         * out clears the index along with the valid bit, so nothing is
         * left pointing at a key that is no longer there.
         */
        w0 = mt7615_rr(sc, addr + MT_WTBL_W0);
        w1 = mt7615_rr(sc, addr + MT_WTBL_W1);
        if (cipher != MT_CIPHER_NONE) {
                w0 &= ~MT_WTBL_W0_KEY_IDX;
                w0 |= MT_WTBL_W0_RX_KEY_VALID |
                      (((uint32_t)keyidx << MT_WTBL_W0_KEY_IDX_S) &
                       MT_WTBL_W0_KEY_IDX);
        } else
                w0 &= ~(MT_WTBL_W0_RX_KEY_VALID | MT_WTBL_W0_KEY_IDX);

        mt7615_wr(sc, MT_WTBL_RICR0, w0);
        mt7615_wr(sc, MT_WTBL_RICR1, w1);

        mt7615_wr(sc, MT_WTBL_UPDATE,
            ((uint32_t)wcid & MT_WTBL_UPDATE_WLAN_IDX) |
            MT_WTBL_UPDATE_RXINFO_UPDATE);

        if (!mt7615_poll(sc, MT_WTBL_UPDATE, MT_WTBL_UPDATE_BUSY, 0, 50))
                return (ETIMEDOUT);

        MT7615_DPRINTF(sc, MT7615_DEBUG_STATE,
            "slot %u key %u cipher %u, %d bytes\n", wcid, keyidx, cipher,
            keylen);

        return (0);
}

void
mt7615_mac_set_rates(struct mt7615_softc *sc, const struct mt7615_peer *peer)
{
        const struct ieee80211_channel *c;
        uint16_t val[4];
        uint32_t addr, w5, w27;
        uint8_t bw, wcid;
        bool ht40, bw80;

        MT7615_ASSERT_LOCKED(sc);

        wcid = peer->wcid;
        if (!mt7615_poll(sc, MT_WTBL_UPDATE, MT_WTBL_UPDATE_BUSY, 0, 50)) {
                device_printf(sc->sc_dev,
                    "the station table is busy; no rates for slot %u\n", wcid);
                return;
        }

        mt7615_rate_ladder(peer, val);
        addr = MT_WTBL_ENTRY(wcid);

        /*
         * Both ends have to have agreed to the width.  Taking it from the
         * channel alone says forty megahertz for a peer that associated
         * at twenty, and everything sent to that peer then goes out over
         * a band it is not listening to.
         */
        c = sc->sc_ic.ic_curchan;
        bw80 = peer->bw80 && c != NULL && IEEE80211_IS_CHAN_VHT80(c);
        ht40 = bw80 ||
               (peer->ht40 && c != NULL && IEEE80211_IS_CHAN_HT40(c));
        bw = bw80 ? MT_CH_BW_80 : (ht40 ? MT_CH_BW_40 : MT_CH_BW_20);

        /*
         * How wide the peer may be sent at, and the rung at which the
         * hardware narrows back down.  Nothing here narrows, so that rung
         * is put out of reach.
         */
        w27 = mt7615_rr(sc, addr + MT_WTBL_W27) & ~MT_WTBL_W27_CC_BW_SEL;
        w27 |= (bw << MT_WTBL_W27_CC_BW_SEL_S) & MT_WTBL_W27_CC_BW_SEL;

        w5 = mt7615_rr(sc, addr + MT_WTBL_W5);
        w5 &= ~(MT_WTBL_W5_BW_CAP | MT_WTBL_W5_CHANGE_BW_RATE |
                MT_WTBL_W5_SHORT_GI_20 | MT_WTBL_W5_SHORT_GI_40 |
                MT_WTBL_W5_SHORT_GI_80 |
                MT_WTBL_W5_MPDU_OK_COUNT | MT_WTBL_W5_MPDU_FAIL_COUNT |
                MT_WTBL_W5_RATE_IDX);
        w5 |= ((bw << MT_WTBL_W5_BW_CAP_S) & MT_WTBL_W5_BW_CAP) |
              ((7U << MT_WTBL_W5_CHANGE_BW_RATE_S) & MT_WTBL_W5_CHANGE_BW_RATE);

        /*
         * The short guard interval shortens the gap between symbols by a
         * tenth, which is a tenth off the time on air for nothing, but
         * only for a peer that said it can hear it.
         */
        if ((peer->htcap & IEEE80211_HTCAP_SHORTGI20) != 0)
                w5 |= MT_WTBL_W5_SHORT_GI_20;
        if (ht40 && (peer->htcap & IEEE80211_HTCAP_SHORTGI40) != 0)
                w5 |= MT_WTBL_W5_SHORT_GI_40;
        if (bw80 && (peer->vhtcap & IEEE80211_VHTCAP_SHORT_GI_80) != 0)
                w5 |= MT_WTBL_W5_SHORT_GI_80;

        mt7615_wr(sc, MT_WTBL_RIUCR0, w5);

        /*
         * Eight slots from four rates: the first is also the one the
         * hardware probes with, and each of the rest is given twice so a
         * fall-back does not skip a step.
         */
        mt7615_wr(sc, MT_WTBL_RIUCR1,
            ((val[0] << MT_WTBL_RIUCR1_RATE0_S) & MT_WTBL_RIUCR1_RATE0) |
            ((val[0] << MT_WTBL_RIUCR1_RATE1_S) & MT_WTBL_RIUCR1_RATE1) |
            ((val[1] << MT_WTBL_RIUCR1_RATE2_LO_S) & MT_WTBL_RIUCR1_RATE2_LO));

        mt7615_wr(sc, MT_WTBL_RIUCR2,
            (((val[1] >> 8) << MT_WTBL_RIUCR2_RATE2_HI_S) &
             MT_WTBL_RIUCR2_RATE2_HI) |
            ((val[1] << MT_WTBL_RIUCR2_RATE3_S) & MT_WTBL_RIUCR2_RATE3) |
            ((val[2] << MT_WTBL_RIUCR2_RATE4_S) & MT_WTBL_RIUCR2_RATE4) |
            ((val[2] << MT_WTBL_RIUCR2_RATE5_LO_S) & MT_WTBL_RIUCR2_RATE5_LO));

        mt7615_wr(sc, MT_WTBL_RIUCR3,
            (((val[2] >> 4) << MT_WTBL_RIUCR3_RATE5_HI_S) &
             MT_WTBL_RIUCR3_RATE5_HI) |
            ((val[3] << MT_WTBL_RIUCR3_RATE6_S) & MT_WTBL_RIUCR3_RATE6) |
            ((val[3] << MT_WTBL_RIUCR3_RATE7_S) & MT_WTBL_RIUCR3_RATE7));

        mt7615_wr(sc, MT_WTBL_UPDATE,
            ((uint32_t)wcid & MT_WTBL_UPDATE_WLAN_IDX) |
            MT_WTBL_UPDATE_RATE_UPDATE | MT_WTBL_UPDATE_TX_COUNT_CLEAR);

        mt7615_wr(sc, addr + MT_WTBL_W27, w27);

        /*
         * Wait for the copy into the table entry to finish.  Until it does
         * the entry holds neither the old ladder nor the new one, and a
         * frame charged to it in that window has no rate to go out at.
         */
        if (!mt7615_poll(sc, MT_WTBL_UPDATE, MT_WTBL_UPDATE_BUSY, 0, 50))
                device_printf(sc->sc_dev,
                    "the rates for slot %u were not taken up\n", wcid);

        /*
         * One line per peer as it joins.  What the two sides settled on is
         * the first thing worth knowing when the link is slower than it
         * ought to be, and none of it can be read back from outside once
         * the ladder is in the table.
         */
        device_printf(sc->sc_dev,
            "slot %u: %s, %u MHz, %u stream%s, mcs<=%u, short gi %s\n", wcid,
            peer->vhtcap != 0 ? "11ac" : "11n",
            bw80 ? 80 : (ht40 ? 40 : 20),
            peer->vhtcap != 0 ? peer->vht_nss :
            (peer->maxmcs >= 15 ? 2 : 1),
            (peer->vhtcap != 0 ? peer->vht_nss : (peer->maxmcs >= 15 ? 2 : 1))
            > 1 ? "s" : "",
            peer->vhtcap != 0 ? 9 : peer->maxmcs,
            ((w5 & MT_WTBL_W5_SHORT_GI_20) != 0 ||
             (w5 & MT_WTBL_W5_SHORT_GI_40) != 0) ? "on" : "off");

        MT7615_DPRINTF(sc, MT7615_DEBUG_TX,
            "slot %u rates %#x %#x %#x %#x\n", wcid, val[0], val[1], val[2],
            val[3]);
}

/*
 * Which slot in the hardware station table a frame is charged to.
 * Group-addressed traffic, and anything for a peer that has not
 * associated, goes out from the network's own pseudo-station.  Slot 0
 * is never used here: it belongs to the beacon.
 */
static uint8_t
mt7615_node_wcid(struct mt7615_vap *mvp, struct ieee80211_node *ni)
{
        uint8_t wcid;

        if (mvp->iv_vap.iv_opmode != IEEE80211_M_HOSTAP) {
                /* The one peer a station has sits in the first slot. */
                return (ni->ni_associd != 0 ? MT7615_WTBL_STA_FIRST :
                        mvp->bmc_wcid);
        }

        if (ni->ni_associd == 0)
                return (mvp->bmc_wcid);

        wcid = IEEE80211_AID(ni->ni_associd);
        if (wcid < MT7615_WTBL_STA_FIRST || wcid >= MT7615_WTBL_STA_LIMIT)
                return (mvp->bmc_wcid);

        return (wcid);
}

/*
 * Build the descriptor for one outbound frame.  txd points at the eight
 * words the engine will read ahead of the payload; the caller has
 * already decided which access category the frame belongs to.
 */
void
mt7615_mac_write_txd(uint32_t *txd, struct mbuf *m, int ac,
                     struct mt7615_vap *mvp, struct ieee80211_node *ni)
{
        struct ieee80211_frame *wh;
        struct mt7615_softc *sc;
        uint32_t val;
        uint16_t seqno;
        uint8_t type, subtype, tid, wcid, q_idx;
        int hdrlen, mcast, qos;

        sc = mvp->iv_vap.iv_ic->ic_softc;
        wh = mtod(m, struct ieee80211_frame *);
        type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
        subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
        mcast = IEEE80211_IS_MULTICAST(wh->i_addr1);
        qos = IEEE80211_QOS_HAS_SEQ(wh);

        tid = 0;
        if (qos) {
                const struct ieee80211_qosframe *qwh;

                qwh = (const struct ieee80211_qosframe *)wh;
                tid = qwh->i_qos[0] & IEEE80211_QOS_TID;
        }

        /*
         * DW0: how many bytes follow the descriptor, and which LMAC
         * queue the frame belongs in.
         *
         * Management goes to the alternate queue rather than to an access
         * category.  That is where the hardware expects the frames it has
         * to place around the beacon - an authentication or association
         * reply queued behind ordinary traffic does not reach the peer in
         * time for it to still be listening.
         */
        if (type == IEEE80211_FC0_TYPE_MGT)
                q_idx = MT_LMAC_ALTX0;
        else
                q_idx = mvp->wmm_idx * MT7615_MAX_WMM_SETS +
                        mt7615_lmac_ac[ac & (WME_NUM_AC - 1)];

        /*
         * The byte count covers the descriptor and the frame, and stops
         * there.  The pointer block that sits between the two on this
         * path is not part of it - counting it makes the engine read
         * past the end of the frame and the frame is never sent.
         */
        val = ((m->m_pkthdr.len + MT_TXD_SIZE) & MT_TXD0_TX_BYTES) |
              ((q_idx << MT_TXD0_PQ_IDX_S) & MT_TXD0_PQ_IDX);
        txd[0] = htole32(val);

        /*
         * DW1: an 802.11 frame handed over whole, charged to the peer's
         * slot in the hardware station table and sent from this
         * network's hardware MAC.
         *
         * HDR_INFO tells the engine where the payload starts, counted in
         * pairs of bytes.  Without it the engine takes the header to be
         * zero-length and the frame goes out malformed, so it has to
         * follow the actual header including the QoS and 4-address cases.
         */
        hdrlen = ieee80211_anyhdrsize(wh);
        wcid = mt7615_node_wcid(mvp, ni);
        val = MT_TXD1_LONG_FORMAT |
              ((MT_HDR_FORMAT_802_11 << MT_TXD1_HDR_FORMAT_S) &
               MT_TXD1_HDR_FORMAT) |
              (((hdrlen / 2) << MT_TXD1_HDR_INFO_S) & MT_TXD1_HDR_INFO) |
              ((MT_TX_TYPE_CT << MT_TXD1_PKT_FMT_S) & MT_TXD1_PKT_FMT) |
              ((mvp->omac_idx << MT_TXD1_OWN_MAC_S) & MT_TXD1_OWN_MAC) |
              ((tid << MT_TXD1_TID_S) & MT_TXD1_TID) |
              (wcid & MT_TXD1_WLAN_IDX);
        txd[1] = htole32(val);

        /* DW2: the frame type, which the LMAC needs for its own timing. */
        val = (((type >> IEEE80211_FC0_TYPE_SHIFT) << MT_TXD2_FRAME_TYPE_S) &
               MT_TXD2_FRAME_TYPE) |
              ((subtype >> IEEE80211_FC0_SUBTYPE_SHIFT) & MT_TXD2_SUB_TYPE);
        if (mcast)
                val |= MT_TXD2_MULTICAST;
        /*
         * The bit says this frame is not part of an aggregate, and it
         * belongs on every frame that is not: left clear without a
         * block-ack session the hardware waits for an acknowledgement
         * from an agreement neither side opened, and the frame never
         * reaches the peer.  net80211 marks the ones that may aggregate
         * once it has negotiated a session for their traffic identifier.
         */
        if ((m->m_flags & M_AMPDU_MPDU) == 0)
                val |= MT_TXD2_BA_DISABLE;
        txd[2] = htole32(val);

        /*
         * DW3: the sequence number net80211 already assigned, and the
         * retry budget.
         */
        seqno = le16toh(*(const uint16_t *)wh->i_seq) >> IEEE80211_SEQ_SEQ_SHIFT;
        val = MT_TXD3_SN_VALID |
              ((seqno << MT_TXD3_SEQ_S) & MT_TXD3_SEQ) |
              ((MT7615_TX_RETRY << MT_TXD3_REM_TX_COUNT_S) &
               MT_TXD3_REM_TX_COUNT);
        if (mcast || (wh->i_fc[1] & IEEE80211_FC1_DIR_MASK) ==
                     IEEE80211_FC1_DIR_NODS)
                val |= MT_TXD3_NO_ACK;
        /*
         * net80211 has put the cipher header in and left the body alone,
         * so the frame still has to be encrypted; this is what asks for
         * it.  The key is the one in the peer's table entry.
         */
        if ((wh->i_fc[1] & IEEE80211_FC1_PROTECTED) != 0)
                val |= MT_TXD3_PROTECT_FRAME;
        txd[3] = htole32(val);

        txd[4] = 0;

        /*
         * DW5 asks the firmware to report on what became of the frame.
         *
         * The report is the only way to learn where the hardware's rate
         * ladder has got to, but one per frame would put an event on the
         * MCU ring for every frame sent, and that ring also carries the
         * command replies the driver waits on.  Every so many frames is
         * enough: the ladder moves in steps, not per frame.
         *
         * Group and management frames are left out.  They go at a fixed
         * rate that says nothing about the ladder, and the group slot is
         * shared by every peer, so their reports could not be charged to
         * anyone in particular anyway.
         */
        if (sc->sc_txs_div > 0 && type == IEEE80211_FC0_TYPE_DATA && !mcast &&
            ++sc->sc_txs_sample >= (uint32_t)sc->sc_txs_div) {
                sc->sc_txs_sample = 0;
                txd[5] = htole32(MT_TXD5_TX_STATUS_HOST |
                                 (MT7615_PID_STATUS & MT_TXD5_PID));
        } else
                txd[5] = 0;

        /*
         * DW6 names a rate, and whether naming one is right depends on the
         * frame.  MT_TXD2_FIX_RATE means "this rate and no other", so for
         * ordinary unicast data it is wrong: a frame that does not get
         * through is then retried at the same rate until its budget runs
         * out rather than dropping to something the air can carry.  Those
         * frames are left to the ladder in the peer's table entry.
         *
         * Management and group-addressed frames have no ladder to use -
         * the first go to peers that have not negotiated anything yet, the
         * second to all of them at once - so they are fixed at the band's
         * slowest rate, which every peer can hear.
         */
        if (type == IEEE80211_FC0_TYPE_MGT || mcast) {
                txd[2] |= htole32(MT_TXD2_FIX_RATE);
                val = MT_TXD6_FIXED_BW |
                      ((mt7615_basic_rate_val(&mvp->iv_vap) <<
                                                            MT_TXD6_TX_RATE_S) & MT_TXD6_TX_RATE);
                txd[6] = htole32(val);
        } else
                txd[6] = 0;

        /*
         * DW7 carries the frame type a second time.  The transmit path
         * reads DW2 and DW7 at different stages and drops a frame whose
         * two copies disagree, so this is not redundant.
         */
        val = (((type >> IEEE80211_FC0_TYPE_SHIFT) << MT_TXD7_TYPE_S) &
               MT_TXD7_TYPE) |
              (((subtype >> IEEE80211_FC0_SUBTYPE_SHIFT) << MT_TXD7_SUB_TYPE_S) &
               MT_TXD7_SUB_TYPE) |
              ((MT7615_SPE_IDX_DEFAULT << MT_TXD7_SPE_IDX_S) & MT_TXD7_SPE_IDX);
        txd[7] = htole32(val);
}

/*
 * The descriptor for a beacon the firmware repeats on its own.  It goes
 * to the LMAC's dedicated beacon queue rather than an access category,
 * is tagged as firmware-owned rather than a host frame, and is sent
 * from the broadcast pseudo-station with no acknowledgement expected.
 */
void
mt7615_mac_write_txd_bcn(struct mt7615_softc *sc, uint32_t *txd,
                         struct mt7615_vap *mvp, int len)
{
        uint32_t val;

        val = ((len + MT_TXD_SIZE) & MT_TXD0_TX_BYTES) |
              ((MT_LMAC_BCN0 << MT_TXD0_PQ_IDX_S) & MT_TXD0_PQ_IDX);
        txd[0] = htole32(val);

        /*
         * A beacon header is the plain three-address form: 24 bytes, or
         * twelve of the two-byte units HDR_INFO counts in.  The engine
         * needs it to find where the header ends and the body begins.
         */
        val = MT_TXD1_LONG_FORMAT |
              ((MT_HDR_FORMAT_802_11 << MT_TXD1_HDR_FORMAT_S) &
               MT_TXD1_HDR_FORMAT) |
              (((sizeof(struct ieee80211_frame) / 2) << MT_TXD1_HDR_INFO_S) &
               MT_TXD1_HDR_INFO) |
              ((MT_TX_TYPE_FW << MT_TXD1_PKT_FMT_S) & MT_TXD1_PKT_FMT) |
              ((mvp->omac_idx << MT_TXD1_OWN_MAC_S) & MT_TXD1_OWN_MAC) |
              (MT7615_WTBL_GLOBAL & MT_TXD1_WLAN_IDX);
        txd[1] = htole32(val);

        /* A management frame, subtype beacon. */
        val = MT_TXD2_MULTICAST | MT_TXD2_BA_DISABLE |
              ((IEEE80211_FC0_SUBTYPE_BEACON >> IEEE80211_FC0_SUBTYPE_SHIFT) &
               MT_TXD2_SUB_TYPE);
        /*
         * A beacon is what a peer finds the network by, so it has to go
         * out where every peer is listening: the primary twenty
         * megahertz, at the slowest rate the band has.  Left to itself
         * the engine sends it as wide as the channel is, and a station
         * scanning the primary channel cannot hear that at all - which is
         * a network that simply does not appear.
         */
        val |= MT_TXD2_FIX_RATE;
        txd[2] = htole32(val);

        /*
         * The LMAC fills in the sequence number for each repeat, so no
         * sequence is supplied here.  A beacon gets the largest retry
         * budget the field can hold, which is what the reference driver
         * uses to mean "as many as it takes".
         */
        txd[3] = htole32(MT_TXD3_NO_ACK |
                         ((0x1fU << MT_TXD3_REM_TX_COUNT_S) & MT_TXD3_REM_TX_COUNT));

        txd[4] = 0;
        txd[5] = 0;
        txd[6] = htole32(MT_TXD6_FIXED_BW |
                         ((mt7615_basic_rate_val(&mvp->iv_vap) << MT_TXD6_TX_RATE_S) &
                          MT_TXD6_TX_RATE));

        /* The type again, for the same reason as in the data path. */
        val = (((IEEE80211_FC0_TYPE_MGT >> IEEE80211_FC0_TYPE_SHIFT) <<
                                                                     MT_TXD7_TYPE_S) & MT_TXD7_TYPE) |
              (((IEEE80211_FC0_SUBTYPE_BEACON >> IEEE80211_FC0_SUBTYPE_SHIFT) <<
                                                                              MT_TXD7_SUB_TYPE_S) & MT_TXD7_SUB_TYPE) |
              ((MT7615_SPE_IDX_DEFAULT << MT_TXD7_SPE_IDX_S) & MT_TXD7_SPE_IDX);
        txd[7] = htole32(val);
}
