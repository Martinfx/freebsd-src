/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Soft context for the MediaTek MT7615E driver.
 */

#ifndef _IF_MT7615VAR_H_
#define	_IF_MT7615VAR_H_

/* One loaded firmware image plus what was parsed out of it. */
struct mt7615_fw {
    const struct firmware	*fw;
    const char		*name;
    uint32_t		addr;		/* Load address in the chip. */
    uint32_t		len;		/* Payload length. */
    const uint8_t		*data;		/* Payload start. */
    uint8_t			feature_set;	/* From the trailer. */
    int			nregions;
    struct {
        uint32_t	addr;
        uint32_t	len;
        uint8_t		feature_set;
    } region[MT7615_MAX_REGION_NUM];
};

/* A block of coherent memory plus the tag and map that own it. */
struct mt7615_dma_info {
    bus_dma_tag_t		tag;
    bus_dmamap_t		map;
    bus_addr_t		paddr;
    void			*vaddr;
    bus_size_t		size;
};

/*
 * The driver's own Tx queue numbering.  The four access categories map
 * onto the single hardware data ring, which the LMAC then sorts by the
 * queue index in the Tx descriptor; MCU and FWDL get rings of their own.
 */
enum mt7615_txq {
    MT7615_TXQ_DATA = 0,
    MT7615_TXQ_MCU,
    MT7615_TXQ_FWDL,
    MT7615_TXQ_COUNT
};

enum mt7615_rxq {
    MT7615_RXQ_DATA = 0,
    MT7615_RXQ_MCU,
    MT7615_RXQ_COUNT
};

struct mt7615_tx_data {
    bus_dmamap_t		map;
    struct mbuf		*m;
    struct ieee80211_node	*ni;
    /* Command buffer for the MCU rings, NULL on the data ring. */
    void			*cmd;
    bus_dma_tag_t		cmd_tag;
    bus_dmamap_t		cmd_map;
};

struct mt7615_tx_ring {
    struct mt7615_dma_info	desc_dma;
    struct mt7615_desc	*desc;
    struct mt7615_tx_data	*data;
    bus_dma_tag_t		data_tag;	/* For mbuf payloads. */

    /*
     * Descriptor scratch for the data ring: one fixed-size slot per
     * descriptor holding the TXD and the buffer pointer block.  It
     * has to be its own DMA buffer rather than the head of the mbuf
     * chain, because the engine is told about it through the
     * descriptor while the payload is listed inside it.
     */
    struct mt7615_dma_info	txd_dma;

    uint32_t		regbase;	/* Ring register block. */
    int			hw_idx;
    int			ndesc;
    int			queued;
    int			cur;		/* Next descriptor to fill. */
    int			next;		/* Next to reclaim. */
};

struct mt7615_rx_data {
    bus_dmamap_t		map;
    struct mbuf		*m;
    bus_addr_t		paddr;
};

struct mt7615_rx_ring {
    struct mt7615_dma_info	desc_dma;
    struct mt7615_desc	*desc;
    struct mt7615_rx_data	*data;
    bus_dma_tag_t		data_tag;
    bus_dmamap_t		spare_map;

    uint32_t		regbase;
    int			hw_idx;
    int			ndesc;
    int			cur;
    int			buf_size;
    /*
     * Set while the ring is being drained.  Handing a frame to
     * net80211 means dropping the driver lock, so a second thread
     * can arrive mid-loop; it sees this and leaves the ring to the
     * thread already working on it.
     */
    bool			polling;
};

/* Radiotap headers, laid out the way net80211 wants them. */
struct mt7615_rx_radiotap_header {
    struct ieee80211_radiotap_header wr_ihdr;
    uint64_t		wr_tsft;
    uint8_t			wr_flags;
    uint8_t			wr_rate;
    uint16_t		wr_chan_freq;
    uint16_t		wr_chan_flags;
    int8_t			wr_dbm_antsignal;
    int8_t			wr_dbm_antnoise;
} __packed __aligned(8);

#define	MT7615_RX_RADIOTAP_PRESENT					\
	((1 << IEEE80211_RADIOTAP_TSFT) |				\
	 (1 << IEEE80211_RADIOTAP_FLAGS) |				\
	 (1 << IEEE80211_RADIOTAP_RATE) |				\
	 (1 << IEEE80211_RADIOTAP_CHANNEL) |				\
	 (1 << IEEE80211_RADIOTAP_DBM_ANTSIGNAL) |			\
	 (1 << IEEE80211_RADIOTAP_DBM_ANTNOISE))

struct mt7615_tx_radiotap_header {
    struct ieee80211_radiotap_header wt_ihdr;
    uint8_t			wt_flags;
    uint8_t			wt_rate;
    uint16_t		wt_chan_freq;
    uint16_t		wt_chan_flags;
} __packed __aligned(8);

#define	MT7615_TX_RADIOTAP_PRESENT					\
	((1 << IEEE80211_RADIOTAP_FLAGS) |				\
	 (1 << IEEE80211_RADIOTAP_RATE) |				\
	 (1 << IEEE80211_RADIOTAP_CHANNEL))

struct mt7615_vap {
    struct ieee80211vap	iv_vap;
    int			(*iv_newstate)(struct ieee80211vap *,
                                              enum ieee80211_state, int);
    uint8_t			omac_idx;	/* Hardware MAC slot. */
    uint8_t			band_idx;
    uint8_t			wmm_idx;
    uint8_t			bmc_wcid;	/* Group-traffic slot. */
};
#define	MT7615_VAP(vap)		((struct mt7615_vap *)(vap))

/* Softc flags. */
#define	MT7615_FLAG_FW_LOADED	0x00000001	/* Images in memory. */
#define	MT7615_FLAG_FW_RUNNING	0x00000002	/* MCU answered. */
#define	MT7615_FLAG_DMA_INITED	0x00000004
#define	MT7615_FLAG_HW_INITED	0x00000008
#define	MT7615_FLAG_ATTACHED	0x00000010	/* net80211 is up. */
#define	MT7615_FLAG_SCANNING	0x00000020

/*
 * What the firmware needs to know about one peer.  Gathered in one
 * place because both the station record and the hardware table entry
 * are built from the same handful of values.
 */
struct mt7615_peer {
    const uint8_t	*addr;
    uint8_t		wcid;		/* Slot in the hardware table. */
    uint16_t	aid;
    uint16_t	htcap;		/* Zero if the peer is not HT. */
    uint8_t		htparam;
    uint8_t		maxmcs;		/* Highest negotiated HT index. */
    bool		ht40;		/* Peer settled on forty megahertz. */
    bool		bw80;		/* Peer settled on eighty megahertz. */
    uint32_t	vhtcap;		/* Zero if the peer is not VHT. */
    uint16_t	vht_rx_mcs;
    uint16_t	vht_tx_mcs;
    uint8_t		vht_nss;	/* Streams the peer will receive. */
    bool		qos;
};

/*
 * A station record the firmware has not been told about yet.
 *
 * net80211 announces a new peer from the receive path, and that runs
 * in an interrupt thread inside the network epoch, where sleeping is
 * forbidden - while telling the firmware means waiting for it to
 * answer.  So the record is copied here and pushed from a thread of
 * the driver's own.  It is a copy rather than a reference because the
 * node it came from may be gone by the time the thread runs.
 */
struct mt7615_sta_pending {
    STAILQ_ENTRY(mt7615_sta_pending)	next;
    struct mt7615_vap	*mvp;
    uint8_t			addr[IEEE80211_ADDR_LEN];
    uint8_t			wcid;
    uint16_t		aid;
    uint16_t		htcap;
    uint8_t			htparam;
    uint8_t			maxmcs;
    bool			ht40;
    bool			bw80;
    uint32_t		vhtcap;
    uint16_t		vht_rx_mcs;
    uint16_t		vht_tx_mcs;
    uint8_t			vht_nss;
    bool			qos;
};

/*
 * One end of a block-acknowledgement session, as the firmware wants it
 * described.  A session covers one traffic identifier of one peer, and
 * the two ends are told about separately: the originator when the peer
 * has agreed to receive aggregates from us, the recipient when the peer
 * asks to send them to us.
 */
struct mt7615_ba {
    const uint8_t	*addr;
    uint8_t		wcid;
    uint8_t		tid;
    uint16_t	ssn;		/* Sequence the window opens at. */
    uint16_t	winsize;
};

/*
 * A session the firmware has not been told about yet.  Block-ack
 * negotiation happens in the receive path, inside the network epoch,
 * where the wait for the firmware to answer is not allowed; so the
 * request is parked here the same way a new station's record is.
 */
struct mt7615_ba_pending {
    STAILQ_ENTRY(mt7615_ba_pending)		next;
    struct mt7615_vap	*mvp;
    uint8_t			addr[IEEE80211_ADDR_LEN];
    uint8_t			wcid;
    uint8_t			tid;
    uint16_t		ssn;
    uint16_t		winsize;
    bool			tx;		/* We send the aggregates. */
    bool			enable;
};

/*
 * One frame the firmware has been given and not yet handed back.  The
 * mapping lives here rather than in the ring slot because the slot is
 * reused as soon as the descriptor is read, which happens well before
 * the firmware has finished with the frame itself.
 */
struct mt7615_token {
    struct mbuf		*m;
    struct ieee80211_node	*ni;
    bus_dmamap_t		map;
};

/*
 * What the transmit status reports have said about one peer since the
 * last time anyone looked.
 *
 * The reports arrive in the interrupt thread, where the node they
 * belong to may not be looked up: finding a node takes a net80211 lock
 * that is held across calls into this driver, so taking it from under
 * the driver lock would invert the order.  So the reports are only
 * counted here, and a thread of the driver's own carries the totals
 * over to net80211 and empties them again.
 *
 * The address is what makes that possible.  It is recorded when the
 * peer's record is pushed to the firmware, which is the same thread
 * that reads these, so a slot never names a peer that has gone.
 */
struct mt7615_txs_stats {
    uint8_t		addr[IEEE80211_ADDR_LEN];
    bool		valid;		/* A peer occupies this slot. */

    uint32_t	nframes;	/* Reports counted. */
    uint32_t	nsuccess;	/* Of those, acknowledged. */
    uint32_t	nretries;	/* Attempts beyond the first. */

    /*
     * The rate the last report named, in the hardware's own coding.
     * Zero means nothing has been reported yet, which is not a rate
     * the hardware ever names for a frame it has sent.
     */
    uint16_t	rate;
};

struct mt7615_softc {
    struct ieee80211com	sc_ic;
    device_t		sc_dev;
    struct mtx		sc_mtx;

    struct resource		*sc_mem;
    int			sc_mem_rid;
    struct resource		*sc_irq;
    int			sc_irq_rid;
    void			*sc_ih;
    int			sc_msi;		/* MSI vectors held. */

    bus_dma_tag_t		sc_dmat;

    struct callout		sc_watchdog_to;
    struct mbufq		sc_snd;		/* Frames waiting for room. */

    /* Work that has to leave the interrupt thread before it can sleep. */
    struct taskqueue	*sc_tq;
    struct task		sc_sta_task;
    struct task		sc_bcn_task;
    struct task		sc_ba_task;
    struct task		sc_txs_task;
    STAILQ_HEAD(, mt7615_sta_pending) sc_sta_pending;
    STAILQ_HEAD(, mt7615_ba_pending) sc_ba_pending;
    /* The network whose beacon the firmware repeats, NULL if none. */
    struct mt7615_vap	*sc_bcn_vap;

    /*
     * The block-ack handlers net80211 installed, which this driver
     * wraps rather than replaces: net80211 keeps running the session,
     * the driver only tells the firmware about it.
     */
    int			(*sc_addba_response)(struct ieee80211_node *,
                                                    struct ieee80211_tx_ampdu *, int, int, int);
    void			(*sc_addba_stop)(struct ieee80211_node *,
                                                 struct ieee80211_tx_ampdu *);
    int			(*sc_ampdu_rx_start)(struct ieee80211_node *,
                                                    struct ieee80211_rx_ampdu *, int, int, int);
    void			(*sc_ampdu_rx_stop)(struct ieee80211_node *,
                                                    struct ieee80211_rx_ampdu *);

    uint32_t		sc_flags;
    uint32_t		sc_chipid;
    uint32_t		sc_hwrev;
    uint8_t			sc_macaddr[IEEE80211_ADDR_LEN];
    bool			sc_have_macaddr;
    uint8_t			sc_nss;		/* Chains the board wired up. */

    /* Firmware images, released once the download has finished. */
    struct mt7615_fw	sc_patch;
    struct mt7615_fw	sc_n9;
    struct mt7615_fw	sc_cr4;
    char			sc_fwver[32];

    /*
     * Frames the firmware still owns, by the token each carries.
     * Held out of line: the table is a hundred kilobytes, and the
     * softc itself is allocated where waiting is not allowed.
     */
    struct mt7615_token	*sc_token;	/* MT7615_TOKEN_SIZE entries. */
    int			sc_token_next;
    int			sc_token_used;

    /* DMA rings. */
    struct mt7615_tx_ring	sc_txq[MT7615_TXQ_COUNT];
    struct mt7615_rx_ring	sc_rxq[MT7615_RXQ_COUNT];

    /* MCU state. */
    uint8_t			sc_mcu_seq;
    uint8_t			sc_mcu_wait_seq;
    int			sc_mcu_wait_cmd;
    int			sc_mcu_resp;	/* Result of the last reply. */
    bool			sc_mcu_done;

    /* Interrupt bookkeeping. */
    uint32_t		sc_intmask;	/* What we asked the chip for. */
    uint32_t		sc_intstatus;	/* Latched in the filter. */
    u_long			sc_intr_count;

    /*
     * Transmit status.  Asking for a report on every frame would put
     * one event on the MCU ring for each frame sent, and that ring is
     * shared with the command replies the driver waits on; so only
     * every sc_txs_div'th frame is marked, which is plenty to keep
     * the rate estimate current.  Zero turns the reports off.
     */
    struct mt7615_txs_stats	sc_txs[MT7615_WTBL_STA_LIMIT];
    uint32_t		sc_txs_sample;	/* Frames since the last mark. */
    int			sc_txs_div;
    u_long			sc_txs_events;	/* Reports seen. */
    u_long			sc_txs_dropped;	/* Reports for unknown slots. */

    int			sc_debug;
    int			sc_vht80;	/* Offer eighty MHz channels. */
    int			sc_noise;

    struct intr_config_hook	sc_preinit_hook;
    eventhandler_tag	sc_mountroot_eh;

    union {
        struct mt7615_rx_radiotap_header th;
        uint8_t	pad[IEEE80211_RADIOTAP_HDRLEN];
    } sc_rxtapu;
#define	sc_rxtap	sc_rxtapu.th
    union {
        struct mt7615_tx_radiotap_header th;
        uint8_t	pad[IEEE80211_RADIOTAP_HDRLEN];
    } sc_txtapu;
#define	sc_txtap	sc_txtapu.th
};

#define	MT7615_LOCK(sc)			mtx_lock(&(sc)->sc_mtx)
#define	MT7615_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define	MT7615_ASSERT_LOCKED(sc)	mtx_assert(&(sc)->sc_mtx, MA_OWNED)
#define	MT7615_LOCK_INIT(sc)						\
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->sc_dev),	\
	    MTX_NETWORK_LOCK, MTX_DEF)
#define	MT7615_LOCK_DESTROY(sc)		mtx_destroy(&(sc)->sc_mtx)

#define	MT7615_DEBUG_ANY	0xffffffff
#define	MT7615_DEBUG_RESET	0x00000001
#define	MT7615_DEBUG_FIRMWARE	0x00000002
#define	MT7615_DEBUG_MCU	0x00000004
#define	MT7615_DEBUG_DMA	0x00000008
#define	MT7615_DEBUG_TX		0x00000010
#define	MT7615_DEBUG_RX		0x00000020
#define	MT7615_DEBUG_INTR	0x00000040
#define	MT7615_DEBUG_STATE	0x00000080

#define	MT7615_DPRINTF(sc, m, ...) do {					\
	if (((sc)->sc_debug & (m)) != 0)				\
		device_printf((sc)->sc_dev, __VA_ARGS__);		\
} while (0)

/* Register access, if_mt7615.c. */
uint32_t	mt7615_rr(struct mt7615_softc *, uint32_t);
void		mt7615_wr(struct mt7615_softc *, uint32_t, uint32_t);
uint32_t	mt7615_rmw(struct mt7615_softc *, uint32_t, uint32_t, uint32_t);
int		mt7615_poll(struct mt7615_softc *, uint32_t, uint32_t,
                               uint32_t, int);

static inline void
mt7615_set(struct mt7615_softc *sc, uint32_t addr, uint32_t bits)
{

        (void)mt7615_rmw(sc, addr, bits, bits);
}

static inline void
mt7615_clear(struct mt7615_softc *sc, uint32_t addr, uint32_t bits)
{

        (void)mt7615_rmw(sc, addr, bits, 0);
}

/* DMA, if_mt7615_dma.c. */
int		mt7615_dma_alloc(struct mt7615_softc *);
void		mt7615_dma_free(struct mt7615_softc *);
int		mt7615_dma_init(struct mt7615_softc *);
void		mt7615_dma_stop(struct mt7615_softc *);
void		mt7615_dma_reset_rings(struct mt7615_softc *);
int		mt7615_tx_queue_buf(struct mt7615_softc *, struct mt7615_tx_ring *,
                                       bus_addr_t, int, bus_addr_t, int, void *, struct mbuf *,
                                       struct ieee80211_node *);
void		mt7615_tx_cleanup(struct mt7615_softc *, struct mt7615_tx_ring *);
int		mt7615_token_alloc(struct mt7615_softc *);
void		mt7615_token_free(struct mt7615_softc *);
int		mt7615_token_get(struct mt7615_softc *);
void		mt7615_start(struct mt7615_softc *);
void		mt7615_token_put(struct mt7615_softc *, int);
void		mt7615_rx_poll(struct mt7615_softc *, int);
void		mt7615_irq_enable(struct mt7615_softc *, uint32_t);
void		mt7615_irq_disable(struct mt7615_softc *, uint32_t);

/* MCU, if_mt7615_mcu.c. */
int		mt7615_mcu_send_msg(struct mt7615_softc *, int, int,
                                       const void *, size_t, bool);
void		mt7615_mcu_rx_event(struct mt7615_softc *, struct mbuf *);
int		mt7615_mcu_init(struct mt7615_softc *);
void		mt7615_mcu_exit(struct mt7615_softc *);
int		mt7615_load_firmware(struct mt7615_softc *);
void		mt7615_free_firmware(struct mt7615_softc *);
int		mt7615_mcu_set_channel(struct mt7615_softc *,
                                          struct ieee80211_channel *);
int		mt7615_mcu_set_eeprom(struct mt7615_softc *);
int		mt7615_mcu_set_mac_enable(struct mt7615_softc *, int, bool);
int		mt7615_mcu_set_rts_thresh(struct mt7615_softc *, uint32_t);
int		mt7615_mcu_del_wtbl_all(struct mt7615_softc *);
int		mt7615_mcu_set_wmm(struct mt7615_softc *, int,
                                      const struct wmeParams *);
int		mt7615_mcu_add_dev(struct mt7615_softc *, struct mt7615_vap *,
                                      bool);
int		mt7615_mcu_add_bss(struct mt7615_softc *, struct mt7615_vap *,
                                      bool);
int		mt7615_mcu_sta_add(struct mt7615_softc *, struct mt7615_vap *,
                                      const struct mt7615_peer *, bool);
int		mt7615_mcu_sta_ba(struct mt7615_softc *, struct mt7615_vap *,
                                     const struct mt7615_ba *, bool, bool);
int		mt7615_mcu_add_beacon(struct mt7615_softc *,
                                         struct mt7615_vap *, bool);

/* MAC, if_mt7615_mac.c. */
int		mt7615_mac_init(struct mt7615_softc *);
void		mt7615_mac_set_rxfilter(struct mt7615_softc *);
void		mt7615_mac_set_timing(struct mt7615_softc *);
int		mt7615_mac_set_key(struct mt7615_softc *, uint8_t, uint8_t,
                                      uint8_t, const uint8_t *, int);
void		mt7615_mac_write_txd(uint32_t *, struct mbuf *, int,
                                         struct mt7615_vap *, struct ieee80211_node *);
void		mt7615_mac_write_txd_bcn(struct mt7615_softc *, uint32_t *,
                                             struct mt7615_vap *, int);
void		mt7615_mac_rx_event(struct mt7615_softc *, struct mbuf *);
void		mt7615_mac_tx_free(struct mt7615_softc *, struct mbuf *);
void		mt7615_mac_rx_txs(struct mt7615_softc *, struct mbuf *);

/* Transmit status bookkeeping, if_mt7615.c. */
void		mt7615_txs_claim(struct mt7615_softc *, uint8_t,
                                     const uint8_t [IEEE80211_ADDR_LEN]);
void		mt7615_txs_flush(struct mt7615_softc *);
void		mt7615_mac_set_rates(struct mt7615_softc *,
                                         const struct mt7615_peer *);

#endif /* _IF_MT7615VAR_H_ */
