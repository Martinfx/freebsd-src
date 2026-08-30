/*
 * Copyright (c) 2025, 2026 Martin Filla <freebsd@sysctl.cz>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef _IF_MTGEVAR_H_
#define	_IF_MTGEVAR_H_

#include <sys/param.h>
#include <sys/sysctl.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/taskqueue.h>
#include <sys/buf_ring.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/endian.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/rman.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <netinet/tcp_lro.h>
#include <net/if_types.h>
#include <dev/clk/clk.h>
#include <dev/syscon/syscon.h>

#define	MTGE_LOCK(sc)		mtx_lock(&(sc)->mtge_lock)
#define	MTGE_UNLOCK(sc)		mtx_unlock(&(sc)->mtge_lock)
#define	MTGE_ASSERT_LOCKED(sc)	mtx_assert(&(sc)->mtge_lock, MA_OWNED)

#define	MTGE_TX_RING_LOCK(ring)		mtx_lock(&(ring)->lock)
#define	MTGE_TX_RING_UNLOCK(ring)		mtx_unlock(&(ring)->lock)
#define	MTGE_TX_RING_TRYLOCK(ring)		mtx_trylock(&(ring)->lock)
#define	MTGE_TX_RING_ASSERT_LOCKED(ring)	\
		    mtx_assert(&(ring)->lock, MA_OWNED)

/*
 * The frame engine drains four Tx rings independently.  Each gets its own
 * lock and its own buf_ring, so two cores can hand frames to the hardware at
 * the same time instead of serializing on one queue.
 */
#define	MT_SOFTC_TX_RING_COUNT		4

/*
 * Depth of the software queue in front of each Tx ring.  Independent of
 * tx_ring_size, which is how many frames the hardware side holds.
 * buf_ring_alloc(9) wants a power of two.
 */
#define	MT_SOFTC_TX_BR_DEPTH		1024
#define	MT_SOFTC_RX_RING_COUNT		2

/*
 * DMA segments one packet may occupy.  A TSO frame is a whole window of
 * data in one mbuf chain, so it needs far more of these than a single
 * segment does; if_sethwtsomaxsegcount() tells the stack to stay below
 * this, and mtge_tx_data() collapses anything that arrives above it.
 */
#define	MT_NCLKS			11
#define	MT_SOFTC_MAX_SCATTER		12

/*
 * What the driver asks tcp_output(9) for.  The count leaves a segment of
 * headroom under MT_SOFTC_MAX_SCATTER for a chain that busdma splits one
 * more time than the stack counted on, and the size is a page because that
 * is the largest run busdma is sure to hand back as one segment.
 */
#define	MT_SOFTC_TSO_MAX_SEGS		(MT_SOFTC_MAX_SCATTER - 2)
#define	MT_SOFTC_TSO_MAX		(MT_SOFTC_MAX_SCATTER * PAGE_SIZE)
/* What the stack may hand down: its own segments, less its own header. */
#define	MT_SOFTC_TSO_MAX_PAYLOAD					\
	(MT_SOFTC_TSO_MAX_SEGS * PAGE_SIZE - ETHER_HDR_LEN -		\
	ETHER_VLAN_ENCAP_LEN)

/*
 * Descriptors one packet can occupy: mtge_tx_data() packs two DMA segments
 * into each descriptor and always starts a fresh one per packet.
 */
#define	MT_SOFTC_TX_DESC_PER_PKT	(1 + MT_SOFTC_MAX_SCATTER / 2)

/*
 * Ring sizes.  Picked at attach from the hw.mtge.rx_ring_size and
 * hw.mtge.tx_ring_size tunables; these are the bounds.
 *
 * The Rx entry count and the Tx *descriptor* count both reach the frame
 * engine's MAX_CNT registers, and the engine wraps a ring by masking the
 * index, so both have to be powers of two.  A Tx descriptor count of 1280
 * (128 * MT_SOFTC_MAX_SCATTER) once left the hardware and the driver
 * disagreeing about where the ring ended: the engine walked into stale
 * descriptors and the Tx-done index the driver polls never lined up again,
 * so every transmit ended in a watchdog timeout.  Receive was the half that
 * kept working, because its count had always been a power of two.
 *
 * The Tx *packet* count is only ever used in software and is free of that
 * constraint.
 *
 * The per-entry arrays live in the softc, so the maxima below are what the
 * structure is sized for.  How large a ring this frame engine really
 * accepts is not something the driver can know, so mtge_init_locked() reads
 * MAX_CNT back and says which tunable to lower when a value does not stick.
 */
#define	MT_RING_DATA_MIN		32
#define	MT_RING_DATA_DEFAULT		256

/*
 * TX_MAX_CNT holds the descriptor count in twelve bits, so 4096 lands in it
 * as zero and the engine and the driver then disagree about where the ring
 * ends -- which is the disagreement the note above describes.  2048 is
 * therefore the ceiling, and the packet count that may be asked for is
 * whatever fits under it once each packet is given its descriptors.
 */
#define	MT_SOFTC_RX_RING_DATA_MAX	1024
#define	MT_SOFTC_TX_RING_DESC_MAX	2048
#define	MT_SOFTC_TX_RING_DATA_MAX					\
	(MT_SOFTC_TX_RING_DESC_MAX / MT_SOFTC_TX_DESC_PER_PKT / 2 * 2)

/*
 * The MT7622 frame engine exposes three interrupt lines and the frame
 * engine interrupt grouping decides which of them carries the PDMA
 * Tx-done / Rx-done events, so hook every line the device tree lists.
 */
#define	MT_MAX_INTRS			3


#define	MT_TXDESC_SDL1_BURST		(1 << 15)
#define	MT_TXDESC_SDL1_LASTSEG		(1 << 14)
#define	MT_TXDESC_SDL0_DDONE		(1 << 15)
#define	MT_TXDESC_SDL0_LASTSEG		(1 << 14)
struct mtge_txdesc {
	uint32_t sdp0;
	uint16_t sdl1;
	uint16_t sdl0;
	uint32_t sdp1;
	uint8_t vid;
#define	TXDSCR_INS_VLAN_TAG	0x80
#define	TXDSCR_VLAN_PRIO_MASK	0x70
#define	TXDSCR_VLAN_IDX_MASK	0x0f
	uint8_t	pppoe;
#define	TXDSCR_USR_DEF_FLD	0x80
#define	TXDSCR_INS_PPPOE_HDR	0x10
#define	TXDSCR_PPPOE_SID_MASK	0x0f
	uint8_t qn;
#define	TXDSCR_QUEUE_MASK	0x07
	uint8_t	dst;
#define	TXDSCR_IP_CSUM_GEN	0x80
#define	TXDSCR_UDP_CSUM_GEN	0x40
#define	TXDSCR_TCP_CSUM_GEN	0x20
/* Segment this frame; the size to cut it into comes from the TCP checksum. */
#define	TXDSCR_TSO		0x10
#define	TXDSCR_DST_PORT_MASK	0x07
#define	TXDSCR_DST_PORT_CPU	0x00
#define	TXDSCR_DST_PORT_GDMA1	0x01
#define	TXDSCR_DST_PORT_GDMA2	0x02
#define	TXDSCR_DST_PORT_PPE	0x06
#define	TXDSCR_DST_PORT_DISC	0x07
} __packed;

#define	MT_RXDESC_SDL0_DDONE		(1 << 15)

/*
 * word3 is the descriptor's rxd4.  The frame engine reports the result of the
 * checksum it already computes -- GDM_IG_CTRL has ICS/TCS/UCS enabled -- and
 * this is where it lands.  The bit position is the one this generation of the
 * frame engine (netsys v1) uses; dev.mtge.<unit>.stats.rx_csum_valid and the
 * raw word in the dump are there to confirm it against real traffic before
 * IFCAP_RXCSUM is trusted with it.
 */
#define	MT_RXDESC_W3_L4_VALID		(1 << 24)

/*
 * Source port of a received frame, also in rxd4.  This generation of the
 * frame engine stamps which GDMA the frame came in through, which is what
 * lets one PDMA serve two MACs: the rings are shared and this field is the
 * only place the origin survives to.
 */
/*
 * The source port lives in bits 21:19 and nothing more: bit 22 belongs to
 * the next field, which the engine fills on punted frames, so a wider mask
 * misreads GDM2 as something else and WAN ingress lands on the wrong
 * interface the moment the PPE is switched on.
 */
#define	MT_RXD4_SPORT_SHIFT		19
#define	MT_RXD4_SPORT_MASK		0x7
#define	MT_RXD4_SPORT_GDM1		1
#define	MT_RXD4_SPORT_GDM2		2

#define	MT_TX_WATCHDOG_TIMEOUT		5
#define	MII_BUSY_RETRY			1000

struct mtge_rxdesc {
	uint32_t sdp0;
	uint16_t sdl1;
	uint16_t sdl0;
	uint32_t sdp1;
	uint32_t word3;
} __packed;

struct mtge_softc_rx_data {
	bus_dmamap_t dma_map;
	struct mbuf *m;
};

struct mtge_softc_rx_ring {
	bus_dma_tag_t desc_dma_tag;
	bus_dmamap_t desc_dma_map;
	bus_addr_t desc_phys_addr;
	struct mtge_rxdesc *desc;
	bus_dma_tag_t data_dma_tag;
	bus_dmamap_t spare_dma_map;
	struct mtge_softc_rx_data data[MT_SOFTC_RX_RING_DATA_MAX];
	int cur;
	int qid;
};

struct mtge_softc_tx_data {
	bus_dmamap_t dma_map;
	struct mbuf *m;
};

/*
 * How the Rx half of flow learning tells the Tx half which flow a packet
 * belongs to.  The engine names the FOE slot in rxd4, but only the Tx side
 * sees where the flow goes and what NAT did to it, and by then the header
 * no longer resembles the one that was received.  Nothing in the rewritten
 * packet reliably identifies it -- eight connections to the same server and
 * port share every field NAT leaves alone -- so the answer travels with the
 * packet instead: the Rx path attaches this tag, routing and NAT carry it
 * along untouched, and the Tx path reads the slot straight out of it.
 */
#define	MT_PPE_TAG_COOKIE	0x6d746765	/* "mtge" */
#define	MT_PPE_TAG_TYPE		0

/* One parsed frame, as the flow-learning code hands it around. */
struct mtge_ppe_flow {
	uint32_t	 sip;
	uint32_t	 dip;
	uint16_t	 sport;
	uint16_t	 dport;
	uint16_t	 ip_id;
	uint16_t	 ip_len;
	uint8_t	 proto;
};

struct mtge_ppe_tag {
	struct m_tag	 tag;
	uint32_t	 hash;		/* FOE slot the engine hashed this to */
	struct mtge_ppe_flow f;	/* tuple as received, before NAT */
};

struct mtge_softc;

struct mtge_softc_tx_ring {
    struct mtx lock;
    /*
     * Software queue ahead of the hardware ring.  mtge_transmit() is the
     * only producer path but runs on any core, so it is multi-producer;
     * the consumer side is single, under the ring lock.
     */
    struct buf_ring	*br;
    /* Drains br when mtge_transmit() could not take the ring lock. */
    struct task	 tx_task;
    struct mtge_softc *sc;
    bus_dma_tag_t desc_dma_tag;
    bus_dmamap_t desc_dma_map;
    bus_addr_t desc_phys_addr;
    struct mtge_txdesc *desc;
    int desc_queued;
    int desc_cur;
    int desc_next;
    bus_dma_tag_t data_dma_tag;
    struct mtge_softc_tx_data data[MT_SOFTC_TX_RING_DATA_MAX];
    int data_queued;
    int data_cur;
    int data_next;
    int qid;
};

struct mtge_softc {
	device_t	 dev;
	/*
	 * Lock order: mtge_lock, then a Tx ring lock, then ppe_mtx.  No path
	 * takes mtge_lock while holding a ring lock, and nothing sleeps under
	 * any of the three -- the task drains that do are done in process
	 * context by mtge_detach() and mtge_resize_rings() after the lock is
	 * dropped.  tx_watchdog_ch runs with mtge_lock held (callout_init_mtx);
	 * periodic_ch has no lock, so it is drained, not just stopped.
	 */
	struct mtx	 mtge_lock;

	int		 mem_rid;
	struct resource	*mem;
	int		 irq_rid[MT_MAX_INTRS];
	struct resource	*irq[MT_MAX_INTRS];
	void		*irqh[MT_MAX_INTRS];

	bus_space_tag_t	 bst;
	bus_space_handle_t bsh;
	bus_dma_tag_t	 parent_tag;

	struct ifnet	*ifp;
	int		 if_flags;
	struct ifmedia	 ifmedia;
	int		 link_up;
	/*
	 * Flow control on the trunks, and the media word each MAC was last
	 * programmed with, so dev.mtge.N.flowctl can reprogram them in place.
	 */
	int		 flowctl;
	uint32_t	 trunk_media[2];

	/*
	 * gmac1, when the device tree carries a mac@1 node -- on the BPI-R64
	 * the RGMII trunk to switch port 5.  It is a second station on the
	 * same frame engine, not a second engine: the PDMA rings, task queues
	 * and interrupts are shared, frames part ways only at the GDMAs.  The
	 * engine itself follows mtge0; this interface just gates its own
	 * traffic.
	 */
	struct ifnet	*ifp1;
	struct ifmedia	 ifmedia1;
	uint8_t		 mac_addr1[ETHER_ADDR_LEN];

	/*
	 * Carrier as the switch sees it.  A GMAC's own link (MSR_LINK) is
	 * the trunk to the switch and stays up whether or not any jack
	 * behind it has a cable, so on its own it tells the stack nothing
	 * useful: dhclient(8) and rc.d/defaultroute wait on an interface
	 * that will never deliver.  mtge_switch_poll() asks the switch once
	 * a second, from the system task queue so the data path never waits
	 * on MDIO, and keeps the answer here: -1 until the first answer,
	 * then 0 or 1.  Written under mtge_lock; mtge_ifmedia_sts() reads it
	 * without, and can only ever be one poll behind.
	 */
	device_t	 sw_dev;
	struct task	 sw_task;
	/* The switch port each GMAC is wired to, -1 when not known. */
	int		 sw_cpu_port[2];
	int		 sw_link[2];

	/*
	 * Effective ring sizes for this instance, within the bounds above.
	 * rx_ring_size and tx_desc_size are what the hardware is told; both
	 * are powers of two.  tx_ring_size is software-only.
	 */
	int		 rx_ring_size;
	int		 tx_ring_size;
	int		 tx_desc_size;
	/*
	 * Set while mtge_resize_rings() has the rings torn down.  The softc
	 * lock cannot be held across that (it drains the task queues), so this
	 * is what keeps a concurrent ifconfig(8) up out of the window.
	 */
	int		 resizing;
	/* Set under the lock by mtge_detach() before it stops the engine. */
	int		 detaching;

	/*
	 * Interrupt coalescing, in the units MT_DELAY_INT_CFG takes: a packet
	 * count and a timer tick count per direction, either of which fires
	 * the delayed interrupt.  Zero on a pair turns that direction off.
	 */
	int		 rx_coal_pkts;
	int		 rx_coal_ticks;
	int		 tx_coal_pkts;
	int		 tx_coal_ticks;

	uint8_t		 mac_addr[ETHER_ADDR_LEN];
	device_t	 miibus;

	uint32_t	 intr_enable_mask;
	uint32_t	 intr_disable_mask;
	uint32_t	 intr_pending_mask;

	struct task	 rx_done_task;
	int		 rx_process_limit;
	struct task	 tx_done_task;
	struct task	 periodic_task;
	struct callout	 periodic_ch;
	unsigned long	 periodic_round;
	/*
	 * Two queues, one thread each: the Rx path is the expensive one
	 * (descriptor walk, cluster allocation and, with direct netisr
	 * dispatch, the whole IP/TCP stack), and pinning Tx completion
	 * behind it on a single thread capped the interface at one core.
	 * The periodic task rides the Tx queue -- it is a tenth-of-a-
	 * second callout and belongs with the watchdog it feeds.
	 */
	struct taskqueue *rx_taskqueue;
	struct taskqueue *tx_taskqueue;

	struct mtge_softc_rx_ring rx_ring[MT_SOFTC_RX_RING_COUNT];
	struct mtge_softc_tx_ring tx_ring[MT_SOFTC_TX_RING_COUNT];
	int		 tx_ring_mgtqid;

	/*
	 * Key for m_ether_tcpip_hash(9).  Picking the Tx ring by flow keeps a
	 * connection on one ring -- the four drain independently, so spreading
	 * one flow across them would reorder it.
	 */
	uint32_t	 hash_key;

	/*
	 * Software LRO.  Only mtge_rx_done_task() touches it, and only one
	 * thread runs that, so it needs no lock of its own.  Off unless
	 * IFCAP_LRO is enabled: merging segments is right for a box that
	 * terminates traffic and wrong for one that forwards it, since what
	 * comes out the other side would have to be split up again.
	 */
	struct lro_ctrl	 lro;
	int		 lro_ok;

	/*
	 * Packet processing engine.  The FOE table is uncached and physically
	 * contiguous - the engine walks it by physical address behind the
	 * CPU's back.  ppe_enabled is the wanted state; mtge_ppe_reapply()
	 * keeps the hardware following it across interface cycles.
	 */
	bus_dma_tag_t	 ppe_foe_tag;
	bus_dmamap_t	 ppe_foe_map;
	void		*ppe_foe;
	bus_addr_t	 ppe_foe_phys;
	int		 ppe_enabled;
	/* How eagerly to bind flows into the engine; see mtge_ppe_rx_note(). */
#define	MT_PPE_LEARN_OFF	0
#define	MT_PPE_LEARN_RATE	1
#define	MT_PPE_LEARN_TRACKED	2
	int		 ppe_bind;

	/*
	 * Flows the engine asked to have bound, waiting for the Rx'd packet
	 * to come back through the Tx path.  The Rx side sees the original
	 * tuple and the FOE slot the engine hashed it to; only the Tx side
	 * sees the routed (and NATed) frame that says where the flow goes and
	 * what it must look like on the way out.  A slot lives for a second;
	 * a flow that does not route within that is not worth binding.
	 *
	 * ppe_mtx is a leaf lock: the Rx task and the Tx paths meet here and
	 * hold their own locks when they do, so this must nest inside
	 * anything and take nothing.
	 */
	struct mtx	 ppe_mtx;
	/* Times the engine asked for a bind, flows actually bound, and
	 * Tx frames that matched a note whose FOE slot no longer held the
	 * flow (stale note, hash collision, or the engine recycled it). */
	unsigned long	 ppe_rate_hits;
	unsigned long	 ppe_bound;
	unsigned long	 ppe_slot_mismatch;
	/* Frames the engine asked about that could not be tagged. */
	unsigned long	 ppe_tag_fail;
	/*
	 * Why the engine handed each frame to the CPU, indexed by the reason
	 * field of rxd4.  With the engine on, this is its running commentary:
	 * 0x0e/0x0f while it tracks a flow, and -- the ones worth watching --
	 * 0x11 (TTL hit 1), 0x12 (VLAN violation on a bound entry) or 0x16
	 * (bound entry forced to the CPU) when a binding is wrong.
	 */
	unsigned long	 ppe_reason[MT_PPE_REASON_COUNT];

	struct callout	 tx_watchdog_ch;
	int		 tx_timer;

	/* statistic counters */
	unsigned long	 interrupts;
	unsigned long	 tx_coherent_interrupts;
	unsigned long	 rx_coherent_interrupts;
	unsigned long	 rx_interrupts[MT_SOFTC_RX_RING_COUNT];
	unsigned long	 rx_delay_interrupts;
	/* Times mtge_periodic_task() found the Rx ring unserviced. */
	unsigned long	 rx_stall_kicks;
	/* Times it found a Tx ring the engine had already finished. */
	unsigned long	 tx_stall_kicks;
	/* Tx completions that arrived while the Tx sources were masked. */
	unsigned long	 tx_intr_lost;
	/* Frames that came in through GDM2, credited to mtge1. */
	unsigned long	 rx_gdm2_packets;
	/* GDM2 frames dropped because mtge1 is absent or down. */
	unsigned long	 rx_sport_drops;
	/* Frames the engine reported a good L4 checksum for, and not. */
	unsigned long	 rx_csum_valid;
	unsigned long	 rx_csum_none;
	/* rxd4 of the last frame received, so the bits can be eyeballed. */
	uint32_t	 rx_last_word3;
	unsigned long	 tx_interrupts[MT_SOFTC_TX_RING_COUNT];
	unsigned long	 tx_delay_interrupts;
	unsigned long	 tx_data_queue_full[MT_SOFTC_TX_RING_COUNT];
	/* Frames handed to the Tx task because the ring lock was taken. */
	unsigned long	 tx_deferred[MT_SOFTC_TX_RING_COUNT];
	/* Frames dropped because the software queue was full. */
	unsigned long	 tx_br_full[MT_SOFTC_TX_RING_COUNT];
	/*
	 * The same GDMA counters for gmac1.  A bound flow leaves through a
	 * GDMA without the CPU seeing it, so these are the only place its
	 * packets are counted at all.
	 */
	unsigned long	 rx_packets1;
	unsigned long	 tx_packets1;
	unsigned long	 tx_watchdog_timeouts;
	unsigned long	 tx_defrag_packets;
	unsigned long	 no_tx_desc_avail;
	unsigned long	 rx_mbuf_alloc_errors;
	unsigned long	 rx_mbuf_dmamap_errors;

	uint64_t	 rx_bytes;
	unsigned long	 rx_packets;
	/*
	 * Frames the PDMA rings actually handed to the stack.  Kept apart
	 * from rx_packets, which accumulates the GDMA (frame engine) packet
	 * counter: when the two diverge, frames reach the frame engine from
	 * the switch but die before or inside the descriptor rings.
	 */
	unsigned long	 rx_pdma_packets;
	unsigned long	 rx_crc_err;
	unsigned long	 rx_phy_err;
	unsigned long	 rx_fifo_overflows;
	unsigned long	 rx_short_err;
	unsigned long	 rx_long_err;
	uint64_t	 tx_bytes;
	unsigned long	 tx_packets;
	unsigned long	 tx_skip;
	unsigned long	 tx_collision;


	/* chip specific registers config */
	int		rx_ring_count;
	/*
	 * Bookkeeping bits: which ring has work waiting, tracked per queue
	 * in intr_pending_mask and consumed by the done tasks.
	 */
	uint32_t	int_rx_done_mask;
	uint32_t	int_tx_done_mask;

	/*
	 * Hardware bits the engine actually interrupts with, and that the
	 * done tasks mask and unmask.  These are the per-queue done bits
	 * when a direction is uncoalesced and the single delayed-interrupt
	 * bit when it is not; see mtge_set_intr_masks().
	 */
	uint32_t	int_rx_mask;
	uint32_t	int_tx_mask;
	uint32_t	pdma_delay_int_cfg;
	uint32_t	pdma_int_status;
	uint32_t	pdma_int_enable;
	uint32_t	pdma_glo_cfg;
	uint32_t	pdma_rst_idx;
	uint32_t	tx_base_ptr[MT_SOFTC_TX_RING_COUNT];
	uint32_t	tx_max_cnt[MT_SOFTC_TX_RING_COUNT];
	uint32_t	tx_ctx_idx[MT_SOFTC_TX_RING_COUNT];
	uint32_t	tx_dtx_idx[MT_SOFTC_TX_RING_COUNT];
	uint32_t	rx_base_ptr[MT_SOFTC_RX_RING_COUNT];
	uint32_t	rx_max_cnt[MT_SOFTC_RX_RING_COUNT];
	uint32_t	rx_calc_idx[MT_SOFTC_RX_RING_COUNT];
	uint32_t	rx_drx_idx[MT_SOFTC_RX_RING_COUNT];

	clk_t		 clks[MT_NCLKS];
};

/*
 * Per-child instance data for the OF devices we enumerate on our MDIO bus
 * (the MT7531 switch described under the "mdio-bus" device-tree subnode).
 */
struct mtge_ofw_devinfo {
	struct ofw_bus_devinfo	di_dinfo;
	struct resource_list	di_rl;
};

#endif /* #ifndef _IF_MTGEVAR_H_ */

