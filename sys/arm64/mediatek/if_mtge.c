/*
 * Copyright (c) 2025, 2026 Martin Filla <freebsd@sysctl.cz>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kenv.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/sbuf.h>
#include <sys/socket.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_param.h>

#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <machine/pmap.h>
#include <machine/resource.h>

#include <net/bpf.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>
#include <net/if_vlan_var.h>

#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>
#endif

#include <dev/clk/clk.h>
#include <dev/etherswitch/etherswitch.h>
#include <dev/etherswitch/miiproxy.h>
#include <dev/mdio/mdio.h>
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include "etherswitch_if.h"
#include "mdio_if.h"
#include "if_mtgereg.h"
#include "if_mtgevar.h"
#include "ofw_bus_if.h"

/* more specific and new models should go first */
static const struct ofw_compat_data mtge_compat_data[] = {
	{ "mediatek,mt7622-eth",	1 },
	{ NULL,				0 }
};

static int	mtge_probe(device_t dev);
static int	mtge_attach(device_t dev);
static int	mtge_detach(device_t dev);
static void	mtge_init_locked(void *priv);
static void	mtge_init(void *priv);
static void	mtge_stop_locked(void *priv);
static int	mtge_ioctl(if_t ifp, u_long cmd, caddr_t data);
static void	mtge_periodic(void *arg);
static void	mtge_tx_watchdog(void *arg);
static void	mtge_intr(void *arg);
static void	mtge_tx_coherent_intr(struct mtge_softc *sc);
static void	mtge_rx_coherent_intr(struct mtge_softc *sc);
static void	mtge_rx_delay_intr(struct mtge_softc *sc);
static void	mtge_tx_delay_intr(struct mtge_softc *sc);
static void	mtge_rx_intr(struct mtge_softc *sc, int qid);
static void	mtge_tx_intr(struct mtge_softc *sc, int qid);
static void	mtge_rx_done_task(void *context, int pending);
static void	mtge_tx_done_task(void *context, int pending);
static void	mtge_periodic_task(void *context, int pending);
static void	mtge_tx_task(void *context, int pending);
static int	mtge_transmit(if_t ifp, struct mbuf *m);
static void	mtge_qflush(if_t ifp);
static void	mtge_tx_start_locked(struct mtge_softc *sc,
		    struct mtge_softc_tx_ring *ring);
static int	mtge_select_queue(struct mtge_softc *sc, struct mbuf *m);
static void	mtge_gdm_fwd_cpu(struct mtge_softc *sc, int gmac);
static void	mtge_ppe_gdm_dst(struct mtge_softc *sc, int dst);
static int	mtge_ppe_attach(struct mtge_softc *sc);
static void	mtge_ppe_detach(struct mtge_softc *sc);
static int	mtge_ppe_enable(struct mtge_softc *sc);
static void	mtge_ppe_disable(struct mtge_softc *sc);
static void	mtge_ppe_reapply(struct mtge_softc *sc);
static int	mtge_ppe_sysctl_enable(SYSCTL_HANDLER_ARGS);
static int	mtge_ppe_sysctl_dump(SYSCTL_HANDLER_ARGS);
static int	mtge_ppe_parse(const uint8_t *fp, int len,
		    struct mtge_ppe_flow *f);
static void	mtge_ppe_rx_note(struct mtge_softc *sc, struct mbuf *m,
		    uint32_t word3);
static void	mtge_ppe_tx_learn(struct mtge_softc *sc, struct mbuf *m);
static struct mbuf *mtge_tx_tso_prepare(struct mbuf *m);
static void	mtge_attach_port1(struct mtge_softc *sc);
static int	mtge_rx_eof(struct mtge_softc *sc,
    struct mtge_softc_rx_ring *ring, int limit);
static void	mtge_tx_eof(struct mtge_softc *sc,
    struct mtge_softc_tx_ring *ring);
static uint32_t	mtge_tx_unreclaimed(struct mtge_softc *sc);
static void	mtge_update_stats(struct mtge_softc *sc);
static void	mtge_watchdog(struct mtge_softc *sc);
static void	mtge_update_raw_counters(struct mtge_softc *sc);
static void	mtge_intr_enable(struct mtge_softc *sc, uint32_t intr_mask);
static void	mtge_intr_disable(struct mtge_softc *sc, uint32_t intr_mask);
static int	mtge_txrx_enable(struct mtge_softc *sc);
static int	mtge_alloc_rx_ring(struct mtge_softc *sc,
    struct mtge_softc_rx_ring *ring, int qid);
static void	mtge_reset_rx_ring(struct mtge_softc *sc,
    struct mtge_softc_rx_ring *ring);
static void	mtge_free_rx_ring(struct mtge_softc *sc,
    struct mtge_softc_rx_ring *ring);
static int	mtge_alloc_tx_ring(struct mtge_softc *sc,
    struct mtge_softc_tx_ring *ring, int qid);
static void	mtge_reset_tx_ring(struct mtge_softc *sc,
    struct mtge_softc_tx_ring *ring);
static void	mtge_free_tx_ring(struct mtge_softc *sc,
    struct mtge_softc_tx_ring *ring);
static void	mtge_dma_map_addr(void *arg, bus_dma_segment_t *segs,
    int nseg, int error);
static void	mtge_sysctl_attach(struct mtge_softc *sc);
static int	mtge_sysctl_dump(SYSCTL_HANDLER_ARGS);
static int	mtge_sysctl_flowctl(SYSCTL_HANDLER_ARGS);
static int	mtge_sysctl_ring_size(SYSCTL_HANDLER_ARGS);
static int	mtge_sysctl_default_ring_size(SYSCTL_HANDLER_ARGS);
static void	mtge_set_coal(struct mtge_softc *sc);
static void	mtge_set_intr_masks(struct mtge_softc *sc);
static bool	mtge_ring_size_ok(int size, int max);
static void	mtge_set_tx_desc_size(struct mtge_softc *sc);
static int	mtge_resize_rings(struct mtge_softc *sc, int rx_size,
    int tx_size);
static int	mtge_ifmedia_upd(if_t );
static void	mtge_ifmedia_sts(if_t , struct ifmediareq *);
static int	mtge_attach_mdio_bus(struct mtge_softc *sc);
static void	mtge_switch_find(struct mtge_softc *sc);
static void	mtge_switch_poll(struct mtge_softc *sc);
static void	mtge_switch_task(void *context, int pending);
static int	mtge_link_state(struct mtge_softc *sc, if_t ifp);
static int	mtge_ring_size(device_t dev, const char *what, int want,
    int max);
static void	mtge_mac_change(struct mtge_softc *sc, uint32_t media,
    int gmac);
static void	mtge_ether_request_mac(device_t dev, uint8_t *eaddr);
static void	mtge_mac_addr(struct mtge_softc *sc, int gmac);
static uint32_t	mtge_fixed_link_media(device_t dev, uint32_t which);

static SYSCTL_NODE(_hw, OID_AUTO, mtge, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "mtge driver parameters");

/*
 * Ring sizes.  These carry the default a new instance attaches with, and a
 * write also resizes the instances that are already attached, so the same
 * name works from loader.conf, from /etc/sysctl.conf and by hand.  A single
 * device can still be set on its own through dev.mtge.<unit>.
 */
static int mtge_rx_ring_size = MT_RING_DATA_DEFAULT;
SYSCTL_PROC(_hw_mtge, OID_AUTO, rx_ring_size,
    CTLTYPE_INT | CTLFLAG_RWTUN | CTLFLAG_MPSAFE, &mtge_rx_ring_size, 0,
    mtge_sysctl_default_ring_size, "I",
    "Rx ring entries per queue; power of two, 32..1024");

static int mtge_tx_ring_size = MT_RING_DATA_DEFAULT;
SYSCTL_PROC(_hw_mtge, OID_AUTO, tx_ring_size,
    CTLTYPE_INT | CTLFLAG_RWTUN | CTLFLAG_MPSAFE, &mtge_tx_ring_size, 1,
    mtge_sysctl_default_ring_size, "I",
    "Tx packets in flight; power of two, 32..512");

/*
 * Interrupt coalescing defaults.  The driver used to write zero here, which
 * is one interrupt per Rx burst; at the 2.5Gbit/s this trunk runs that is a
 * lot of trips through a single-threaded taskqueue.  Batch a few frames or
 * ~160us, whichever comes first, and leave every value reachable at runtime
 * so a regression can be undone without a rebuild - zero on a pair restores
 * the old behaviour exactly.
 */
static int mtge_rx_coal_pkts = 8;
SYSCTL_INT(_hw_mtge, OID_AUTO, rx_coal_pkts, CTLFLAG_RDTUN,
    &mtge_rx_coal_pkts, 0, "Rx frames to batch before interrupting, 0..127");

static int mtge_rx_coal_ticks = 8;
SYSCTL_INT(_hw_mtge, OID_AUTO, rx_coal_ticks, CTLFLAG_RDTUN,
    &mtge_rx_coal_ticks, 0, "Rx coalescing timer in 20us ticks, 0..255");

static int mtge_tx_coal_pkts = 8;
SYSCTL_INT(_hw_mtge, OID_AUTO, tx_coal_pkts, CTLFLAG_RDTUN,
    &mtge_tx_coal_pkts, 0, "Tx frames to batch before interrupting, 0..127");

static int mtge_tx_coal_ticks = 8;
SYSCTL_INT(_hw_mtge, OID_AUTO, tx_coal_ticks, CTLFLAG_RDTUN,
    &mtge_tx_coal_ticks, 0, "Tx coalescing timer in 20us ticks, 0..255");

/*
 * Whether the MACs honour and send pause frames on their trunks to the
 * switch, when the device tree asks for it.  Runtime copy in
 * dev.mtge.N.flowctl, so the two can be compared without a rebuild.
 */
static int mtge_flowctl = 1;
SYSCTL_INT(_hw_mtge, OID_AUTO, flowctl, CTLFLAG_RDTUN,
    &mtge_flowctl, 0, "Flow control on the trunks at attach, 0 or 1");

static const char * const mtge_clk_names[MT_NCLKS] = {
	"ethif",
	"esw",
	"gp0",
	"gp1",
	"gp2",
	"sgmii_tx250m",
	"sgmii_rx250m",
	"sgmii_cdr_ref",
	"sgmii_cdr_fb",
	"sgmii_ck",
	"eth2pll",
};

static int
mtge_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev)) {
		return (ENXIO);
	}

	if (ofw_bus_search_compatible(dev, mtge_compat_data)->ocd_data == 0) {
		return (ENXIO);
	}

	device_set_desc(dev, "Mediatek MT7622 Ethernet driver");
	return (BUS_PROBE_DEFAULT);
}

static int
mtge_attach(device_t dev)
{
	struct mtge_softc *sc;
	if_t ifp;
	int error, i;
	int gmac = 0;
	uint32_t media;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtge_lock, device_get_nameunit(dev), MTX_NETWORK_LOCK,
	    MTX_DEF);
	sc->sw_cpu_port[0] = sc->sw_cpu_port[1] = -1;
	sc->sw_link[0] = sc->sw_link[1] = -1;
	sc->flowctl = (mtge_flowctl != 0);
	TASK_INIT(&sc->sw_task, 0, mtge_switch_task, sc);
        NET_TASK_INIT(&sc->rx_done_task, 0, mtge_rx_done_task, sc);
        TASK_INIT(&sc->tx_done_task, 0, mtge_tx_done_task, sc);
        TASK_INIT(&sc->periodic_task, 0, mtge_periodic_task, sc);
        callout_init(&sc->periodic_ch, 0);
        callout_init_mtx(&sc->tx_watchdog_ch, &sc->mtge_lock, 0);

        sc->mem_rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->mem == NULL) {
		device_printf(dev, "could not allocate memory resource\n");
		error = ENXIO;
		goto fail;
	}

	sc->bst = rman_get_bustag(sc->mem);
	sc->bsh = rman_get_bushandle(sc->mem);

	/* Forced trunk-link parameters from the gmac0 "fixed-link" node. */
	media = mtge_fixed_link_media(dev, 0);

        for (i = 0; i < MT_NCLKS; i++) {
                error = clk_get_by_ofw_name(sc->dev, 0, mtge_clk_names[i],
                    &sc->clks[i]);
                if (error != 0) {
                        device_printf(sc->dev, "cannot get '%s' clock\n",
                            mtge_clk_names[i]);
                        goto fail;
                }

                error = clk_enable(sc->clks[i]);
                if (error != 0) {
                        device_printf(sc->dev, "cannot enable '%s' clock\n",
                            mtge_clk_names[i]);
                        goto fail;
                }
        }

	for (i = 0; i < MT_MAX_INTRS; i++) {
		sc->irq_rid[i] = i;
		sc->irq[i] = bus_alloc_resource_any(dev, SYS_RES_IRQ,
		    &sc->irq_rid[i], RF_ACTIVE);
                if (sc->irq[i] == NULL) {
                        device_printf(dev, "cannot allocate interrupt\n");
                        error = ENXIO;
                        goto fail;
                }
	}

        for (i = 0; i < MT_MAX_INTRS; i++) {
                error = bus_setup_intr(dev, sc->irq[i],
                    INTR_TYPE_NET | INTR_MPSAFE, NULL, mtge_intr, sc,
                    &sc->irqh[i]);
                if (error != 0) {
                        device_printf(dev,
                            "could not set up interrupt %d\n", i);
                        goto fail;
                }
        }

	sc->pdma_delay_int_cfg=MT_DELAY_INT_CFG;
	sc->pdma_int_status=MT_PDMA_INT_STATUS;
	sc->pdma_int_enable=MT_PDMA_INT_ENABLE;
	sc->pdma_glo_cfg=MT_PDMA_GLO_CFG;
	sc->pdma_rst_idx=MT_PDMA_RST_IDX;

	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++) {
		sc->tx_base_ptr[i]=MT_TX_BASE_PTR(i);
		sc->tx_max_cnt[i]=MT_TX_MAX_CNT(i);
		sc->tx_ctx_idx[i]=MT_TX_CTX_IDX(i);
		sc->tx_dtx_idx[i]=MT_TX_DTX_IDX(i);
	}

	sc->rx_ring_count=2;
	for (i = 0; i < sc->rx_ring_count; i++) {
		sc->rx_base_ptr[i]=MT_RX_BASE_PTR(i);
		sc->rx_max_cnt[i]=MT_RX_MAX_CNT(i);
		sc->rx_calc_idx[i]=MT_RX_CALC_IDX(i);
		sc->rx_drx_idx[i]=MT_RX_DRX_IDX(i);
	}
	sc->int_rx_done_mask=MT_INT_RXQ0_DONE;
	sc->int_tx_done_mask=MT_INT_TXQ0_DONE;

	mtge_gdm_fwd_cpu(sc, gmac);
	if (sc->ifp1 != NULL)
		mtge_gdm_fwd_cpu(sc, 1);

	mtge_mac_change(sc, media, gmac);

	/* Create parent DMA tag. */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* parent */
	    1, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    BUS_SPACE_MAXSIZE_32BIT,	/* maxsize */
	    0,				/* nsegments */
	    BUS_SPACE_MAXSIZE_32BIT,	/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->parent_tag);
	if (error != 0) {
		device_printf(dev, "could not create parent DMA tag\n");
		goto fail;
	}

	sc->rx_coal_pkts = min(max(mtge_rx_coal_pkts, 0), DLY_PINT_MAX);
	sc->rx_coal_ticks = min(max(mtge_rx_coal_ticks, 0), DLY_PTIME_MAX);
	sc->tx_coal_pkts = min(max(mtge_tx_coal_pkts, 0), DLY_PINT_MAX);
	sc->tx_coal_ticks = min(max(mtge_tx_coal_ticks, 0), DLY_PTIME_MAX);

	sc->rx_ring_size = mtge_ring_size(dev, "hw.mtge.rx_ring_size",
	    mtge_rx_ring_size, MT_SOFTC_RX_RING_DATA_MAX);
	sc->tx_ring_size = mtge_ring_size(dev, "hw.mtge.tx_ring_size",
	    mtge_tx_ring_size, MT_SOFTC_TX_RING_DATA_MAX);

	mtge_set_tx_desc_size(sc);

	if (bootverbose)
		device_printf(dev, "Rx ring %d, Tx ring %d packets / %d "
		    "descriptors\n", sc->rx_ring_size, sc->tx_ring_size,
		    sc->tx_desc_size);

	/*
	 * Per-ring state that has to outlive mtge_resize_rings(): it tears the
	 * rings down and builds them again, but mtge_transmit() reaches the
	 * lock and the software queue from any core in the meantime.
	 */
	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++) {
		mtx_init(&sc->tx_ring[i].lock, device_get_nameunit(dev), NULL,
		    MTX_DEF);
		sc->tx_ring[i].sc = sc;
		sc->tx_ring[i].br = buf_ring_alloc(MT_SOFTC_TX_BR_DEPTH,
		    M_DEVBUF, M_WAITOK, &sc->tx_ring[i].lock);
		TASK_INIT(&sc->tx_ring[i].tx_task, 0, mtge_tx_task,
		    &sc->tx_ring[i]);
	}

	/* allocate Tx and Rx rings */
	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++) {
		error = mtge_alloc_tx_ring(sc, &sc->tx_ring[i], i);
		if (error != 0) {
			device_printf(dev, "could not allocate Tx ring #%d\n",
			    i);
			goto fail;
		}
	}

	sc->hash_key = m_ether_tcpip_hash_init();

	sc->tx_ring_mgtqid = 5;
	for (i = 0; i < sc->rx_ring_count; i++) {
		error = mtge_alloc_rx_ring(sc, &sc->rx_ring[i], i);
		if (error != 0) {
			device_printf(dev, "could not allocate Rx ring\n");
			goto fail;
		}
	}

	ifp = sc->ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "could not if_alloc()\n");
		error = ENOMEM;
		goto fail;
	}

	if_setsoftc(ifp, sc);
	if_initname(ifp, device_get_name(sc->dev), device_get_unit(sc->dev));
	if_setflags(ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setinitfn(ifp, mtge_init);
	if_setioctlfn(ifp, mtge_ioctl);
	if_settransmitfn(ifp, mtge_transmit);
	if_setqflushfn(ifp, mtge_qflush);

	ifmedia_init(&sc->ifmedia, 0, mtge_ifmedia_upd, mtge_ifmedia_sts);
	ifmedia_add(&sc->ifmedia, media, 0, NULL);
	ifmedia_set(&sc->ifmedia, media);

	/*
	 * Prefer creating the dedicated OFW MDIO bus (mtkmdio) from the
	 * "mdio-bus" device-tree subnode; its children (e.g. the MT7531
	 * switch at switch@1f) attach there as proper OF devices.  If the
	 * DT has no such node, fall back to the legacy hinted "mdio" bus,
	 * so boards and device trees without one keep working as before.
	 */
	if (mtge_attach_mdio_bus(sc) == 0) {
		device_t child;

		child = device_add_child(dev, "mdio", DEVICE_UNIT_ANY);
		bus_attach_children(sc->dev);
		if (child != NULL)
			bus_attach_children(child);
	}

	mtge_ether_request_mac(dev, sc->mac_addr);
	if (bootverbose)
		device_printf(dev, "Ethernet address %6D\n", sc->mac_addr, ":");

	/* Attach ethernet interface */
	ether_ifattach(ifp, sc->mac_addr);
	mtge_mac_addr(sc, gmac);

	/*
	 * Tell the upper layer(s) we support long frames.
	 */
	if_setifheaderlen(ifp, sizeof(struct ether_vlan_header));
	if_setcapabilitiesbit(ifp, IFCAP_VLAN_MTU, 0);
	if_setcapenablebit(ifp, IFCAP_VLAN_MTU, 0);

	if_setcapabilitiesbit(ifp, IFCAP_TXCSUM, 0);
	if_setcapenablebit(ifp, IFCAP_TXCSUM, 0);

	/*
	 * Rx checksum offload.  The frame engine already verifies the checksum
	 * -- GDM_IG_CTRL enables ICS, TCS and UCS -- so this only reads back a
	 * result that is being computed either way and saves the stack walking
	 * 1460 bytes again.
	 *
	 * The descriptor bit has been confirmed against real traffic on a
	 * BPI-R64: 1.6M frames marked valid over an iperf3 run with 169
	 * unmarked (the non-TCP/UDP rest), and net.inet.tcp.lro.lro_badcsum
	 * agreeing.  It is also what gates tcp_lro_rx(9), which refuses any
	 * frame the hardware has not vouched for -- with this off, LRO
	 * aggregates nothing.  So it defaults to on;
	 * dev.mtge.<unit>.stats.rx_csum_valid keeps counting for anyone who
	 * needs to re-verify, and "ifconfig mtge0 -rxcsum" turns it off.
	 */
	if_setcapabilitiesbit(ifp, IFCAP_RXCSUM, 0);
	if_setcapenablebit(ifp, IFCAP_RXCSUM, 0);

	/*
	 * Software LRO.  Note this does nothing on its own: tcp_lro_rx(9)
	 * rejects any frame the hardware has not already checksummed --
	 * CSUM_DATA_VALID, CSUM_PSEUDO_HDR and csum_data 0xffff -- and counts
	 * it in net.inet.tcp.lro.lro_badcsum.  So IFCAP_RXCSUM above has to be
	 * on and its descriptor bit confirmed before enabling this buys
	 * anything at all; with RXCSUM off it is a call and a counter per
	 * frame and nothing else.
	 *
	 * Left off for that reason rather than out of any worry about
	 * forwarding: tcp_lro_rx(9) already refuses to aggregate while
	 * net.inet.ip.forwarding is set, so a router is safe either way.
	 */
	if (tcp_lro_init_args(&sc->lro, ifp, TCP_LRO_ENTRIES, 0) == 0) {
		sc->lro_ok = 1;
		if_setcapabilitiesbit(ifp, IFCAP_LRO, 0);
	} else
		device_printf(dev, "could not set up LRO\n");

	/*
	 * Advertise the Tx offloads through if_hwassist so the stack leaves
	 * those checksums to us; mtge_tx_data() then asks the GDMA to fill in
	 * exactly the ones the stack flagged on each frame.
	 */
	if_sethwassistbits(ifp, CSUM_IP | CSUM_TCP | CSUM_UDP, 0);

	/*
	 * Segmentation offload.  The engine cuts a window-sized frame into
	 * MSS-sized ones itself, which is the piece the Rx side has had all
	 * along in LRO and the Tx side has not: without it the stack builds
	 * every segment on the CPU, and this board sends at half the rate it
	 * receives.  The limits below are what tcp_output(9) is asked to
	 * respect when it assembles the chain.
	 */
	if_setcapabilitiesbit(ifp, IFCAP_TSO4, 0);
	if_setcapenablebit(ifp, IFCAP_TSO4, 0);
	if_sethwassistbits(ifp, CSUM_TSO, 0);
	if_sethwtsomax(ifp, MT_SOFTC_TSO_MAX_PAYLOAD);
	if_sethwtsomaxsegcount(ifp, MT_SOFTC_TSO_MAX_SEGS);
	if_sethwtsomaxsegsize(ifp, PAGE_SIZE);

	sc->rx_process_limit = 100;

	sc->rx_taskqueue = taskqueue_create("mtge_rxq", M_NOWAIT,
	    taskqueue_thread_enqueue, &sc->rx_taskqueue);
	sc->tx_taskqueue = taskqueue_create("mtge_txq", M_NOWAIT,
	    taskqueue_thread_enqueue, &sc->tx_taskqueue);
	if (sc->rx_taskqueue == NULL || sc->tx_taskqueue == NULL) {
		device_printf(dev, "could not create task queues\n");
		error = ENOMEM;
		goto fail;
	}

	taskqueue_start_threads(&sc->rx_taskqueue, 1, PI_NET, "%s rxq",
	    device_get_nameunit(sc->dev));
	taskqueue_start_threads(&sc->tx_taskqueue, 1, PI_NET, "%s txq",
	    device_get_nameunit(sc->dev));

	mtge_sysctl_attach(sc);
	mtge_attach_port1(sc);

	/* Optional: the router works without it, so failure is not fatal. */
	if (mtge_ppe_attach(sc) != 0)
		device_printf(dev, "PPE unavailable, continuing without it\n");

	return (0);

fail:
	if (sc->lro_ok) {
		tcp_lro_free(&sc->lro);
		sc->lro_ok = 0;
	}

	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++) {
		if (sc->tx_ring[i].br != NULL) {
			buf_ring_free(sc->tx_ring[i].br, M_DEVBUF);
			sc->tx_ring[i].br = NULL;
			mtx_destroy(&sc->tx_ring[i].lock);
		}
	}

	if (sc->rx_taskqueue != NULL)
		taskqueue_free(sc->rx_taskqueue);
	if (sc->tx_taskqueue != NULL)
		taskqueue_free(sc->tx_taskqueue);

	/* free Tx and Rx rings */
	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++)
		mtge_free_tx_ring(sc, &sc->tx_ring[i]);

	for (i = 0; i < sc->rx_ring_count; i++)
		mtge_free_rx_ring(sc, &sc->rx_ring[i]);

	mtx_destroy(&sc->mtge_lock);

	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid,
		    sc->mem);

	for (i = 0; i < MT_MAX_INTRS; i++) {
		if (sc->irq[i] == NULL)
			continue;
		if (sc->irqh[i] != NULL)
			bus_teardown_intr(dev, sc->irq[i], sc->irqh[i]);
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid[i],
		    sc->irq[i]);
	}

        for (i = 0; i < MT_NCLKS; i++) {
                if(sc->clks[i] != NULL) {
                        error = clk_disable(sc->clks[i]);
                        if(error != 0 && bootverbose) {
                                device_printf(sc->dev,
                                    "failed to disable mtge clock: %d\n", error);
                        }
                }
        }

	return (error);
}

/*
 * Build MT_DELAY_INT_CFG from the softc and hand it to the engine.  A pair
 * whose packet count and timer are both zero leaves that direction
 * uncoalesced, which is what the driver did unconditionally before.
 */
static void
mtge_set_coal(struct mtge_softc *sc)
{
	uint32_t val;

	val = 0;
	if (sc->rx_coal_pkts != 0 || sc->rx_coal_ticks != 0)
		val |= RXDLY_INT_EN |
		       (sc->rx_coal_pkts << RXMAX_PINT_SHIFT) |
		       (sc->rx_coal_ticks << RXMAX_PTIME_SHIFT);
	if (sc->tx_coal_pkts != 0 || sc->tx_coal_ticks != 0)
		val |= TXDLY_INT_EN |
		       (sc->tx_coal_pkts << TXMAX_PINT_SHIFT) |
		       (sc->tx_coal_ticks << TXMAX_PTIME_SHIFT);

	MT_WRITE(sc, sc->pdma_delay_int_cfg, val);
}

/*
 * Pick the interrupt bits that stand for "Rx work" and "Tx work", and rebuild
 * intr_enable_mask from them.
 *
 * MT_DELAY_INT_CFG only decides when the engine raises MT_RX_DLY_INT /
 * MT_TX_DLY_INT.  The undelayed per-queue done bits keep firing regardless, so
 * enabling those alongside a delay configuration makes the configuration inert:
 * the driver still takes one interrupt per burst and the delayed bit never
 * reaches the CPU at all.  Coalescing a direction therefore means switching
 * which bit is enabled, not just programming the register.
 *
 * A direction only moves to the delayed bit when its timer is non-zero.  The
 * packet threshold on its own can be left unmet forever by a burst that stops
 * short of it, and then nothing would ever tell the driver to drain the ring.
 */
static void
mtge_set_intr_masks(struct mtge_softc *sc)
{
	int i;

	/*
	 * Uncoalesced, only the rings that are actually drained are worth an
	 * interrupt: mtge_rx_done_task() reads ring 0 and
	 * MT_SOFTC_TX_RING_COUNT is one.  A done bit for a ring nothing
	 * services would just leave a pending bit set that no task ever
	 * clears.
	 */
	if (sc->rx_coal_ticks != 0)
		sc->int_rx_mask = MT_RX_DLY_INT;
	else
		sc->int_rx_mask = MT_INT_RXQ0_DONE;

	if (sc->tx_coal_ticks != 0) {
		sc->int_tx_mask = MT_TX_DLY_INT;
	} else {
		sc->int_tx_mask = 0;
		for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++)
			sc->int_tx_mask |= sc->int_tx_done_mask << i;
	}

	sc->intr_enable_mask = MT_INT_TX_COHERENT | MT_INT_RX_COHERENT |
	    sc->int_rx_mask | sc->int_tx_mask;
}

/*
 * The frame engine wraps a ring by masking the index, so a count that is not
 * a power of two makes it disagree with us about where the ring ends.
 */
static bool
mtge_ring_size_ok(int size, int max)
{

	return (size >= MT_RING_DATA_MIN && size <= max &&
	    (size & (size - 1)) == 0);
}

/*
 * Validate one ring size at attach.  Fall back to the default rather than
 * refusing to attach.
 */
static int
mtge_ring_size(device_t dev, const char *what, int want, int max)
{

	if (mtge_ring_size_ok(want, max))
		return (want);

	device_printf(dev, "%s %d is not a power of two in [%d, %d], using "
	    "%d\n", what, want, MT_RING_DATA_MIN, max, MT_RING_DATA_DEFAULT);

	return (MT_RING_DATA_DEFAULT);
}

/*
 * Point one GDMA at the CPU: every frame class forwarded to the PDMA, CRC
 * stripped, and the checksum engines on.  Shared by both MACs; only the GDMA
 * index differs.
 */
static void
mtge_gdm_fwd_cpu(struct mtge_softc *sc, int gmac)
{

	MT_WRITE(sc, MT_GDM_IG_CTRL(gmac),
	    (
		GDM_ICS_EN | /* Enable IP Csum */
		GDM_TCS_EN | /* Enable TCP Csum */
		GDM_UCS_EN | /* Enable UDP Csum */
		GDM_STRPCRC | /* Strip CRC from packet */
		GDM_DST_PORT_CPU << GDM_UFRC_P_SHIFT | /* fwd UCast to CPU */
		GDM_DST_PORT_CPU << GDM_BFRC_P_SHIFT | /* fwd BCast to CPU */
		GDM_DST_PORT_CPU << GDM_MFRC_P_SHIFT | /* fwd MCast to CPU */
		GDM_DST_PORT_CPU << GDM_OFRC_P_SHIFT   /* fwd Other to CPU */
		));
}

/*
 * Packet processing engine, phase one: the engine in its pass-through state.
 *
 * With the FOE table empty every packet misses, and SEARCH_MISS is set to
 * forward-and-build: the PPE creates an UNBIND entry for the flow and punts
 * the packet to the PDMA, so traffic behaves exactly as without the PPE
 * while the table quietly fills with flow candidates.  Nothing is bound yet
 * - binding is the next phase - so the CPU still forwards everything; what
 * this phase buys is the engine programmed, the table live, and the rxd4
 * FOE fields observable on real traffic.  dev.mtge.<unit>.ppe_enable flips
 * the GDMAs between CPU and PPE at runtime and defaults to off.
 *
 * Register values follow the vendor's Linux driver (mtk_ppe.c, netsys v1
 * paths); ppe_dump prints readbacks to check them against the hardware.
 */
static void
mtge_ppe_gdm_dst(struct mtge_softc *sc, int dst)
{
	uint32_t val;

	val = GDM_ICS_EN | GDM_TCS_EN | GDM_UCS_EN | GDM_STRPCRC |
	    dst << GDM_UFRC_P_SHIFT |
	    dst << GDM_BFRC_P_SHIFT |
	    dst << GDM_MFRC_P_SHIFT |
	    dst << GDM_OFRC_P_SHIFT;

	MT_WRITE(sc, MT_GDM_IG_CTRL(0), val);
	if (sc->ifp1 != NULL)
		MT_WRITE(sc, MT_GDM_IG_CTRL(1), val);

	/*
	 * Pulse the PSE.  Frames it accepted for the old destination are
	 * still queued inside it, and the vendor driver resets it on every
	 * forwarding change rather than let those cross the switchover.
	 */
	MT_WRITE(sc, MT_RST_GL, RST_GL_PSE);
	MT_WRITE(sc, MT_RST_GL, 0);
}

static int
mtge_ppe_attach(struct mtge_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	int error;

	error = bus_dma_tag_create(sc->parent_tag, PAGE_SIZE, 0,
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL,
	    MT_PPE_ENTRIES * MT_PPE_ENTRY_SIZE, 1,
	    MT_PPE_ENTRIES * MT_PPE_ENTRY_SIZE,
	    0, NULL, NULL, &sc->ppe_foe_tag);
	if (error != 0) {
		device_printf(sc->dev, "could not create FOE DMA tag\n");
		return (error);
	}

	/*
	 * The engine walks the table by physical address, so it has to be one
	 * contiguous block; BUS_DMA_COHERENT makes the mapping uncached, the
	 * same reasoning as the descriptor rings.
	 */
	error = bus_dmamem_alloc(sc->ppe_foe_tag, &sc->ppe_foe,
	    BUS_DMA_NOWAIT | BUS_DMA_ZERO | BUS_DMA_COHERENT,
	    &sc->ppe_foe_map);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not allocate the FOE table (%d KB contiguous)\n",
		    MT_PPE_ENTRIES * MT_PPE_ENTRY_SIZE / 1024);
		bus_dma_tag_destroy(sc->ppe_foe_tag);
		sc->ppe_foe_tag = NULL;
		return (error);
	}

	error = bus_dmamap_load(sc->ppe_foe_tag, sc->ppe_foe_map,
	    sc->ppe_foe, MT_PPE_ENTRIES * MT_PPE_ENTRY_SIZE,
	    mtge_dma_map_addr, &sc->ppe_foe_phys, 0);
	if (error != 0) {
		device_printf(sc->dev, "could not load the FOE table map\n");
		bus_dmamem_free(sc->ppe_foe_tag, sc->ppe_foe, sc->ppe_foe_map);
		bus_dma_tag_destroy(sc->ppe_foe_tag);
		sc->ppe_foe = NULL;
		sc->ppe_foe_tag = NULL;
		return (error);
	}

	mtx_init(&sc->ppe_mtx, "mtge ppe", NULL, MTX_DEF);
	sc->ppe_bind = MT_PPE_LEARN_TRACKED;

	/*
	 * A warm reboot does not reset the frame engine, so the engine can
	 * come up still enabled and still pointed at the previous kernel's
	 * table -- memory this one now owns.  Put it down before anything
	 * can feed it.
	 */
	MT_WRITE(sc, MT_PPE_GLO_CFG, 0);
	MT_WRITE(sc, MT_PPE_FLOW_CFG, 0);
	MT_WRITE(sc, MT_PPE_TB_BASE, 0);

	ctx = device_get_sysctl_ctx(sc->dev);
	tree = device_get_sysctl_tree(sc->dev);
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "ppe_enable",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    mtge_ppe_sysctl_enable, "I",
	    "Route ingress through the packet processing engine");
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "ppe_bind",
	    CTLFLAG_RW, &sc->ppe_bind, 0,
	    "Bind flows into the engine: 0 never, 1 once a flow is busy, "
	    "2 as soon as the engine tracks it");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "ppe_dump",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    mtge_ppe_sysctl_dump, "A", "PPE registers and FOE table state");

	if (bootverbose)
		device_printf(sc->dev, "PPE: FOE table %d entries at 0x%jx\n",
		    MT_PPE_ENTRIES, (uintmax_t)sc->ppe_foe_phys);

	return (0);
}

static void
mtge_ppe_detach(struct mtge_softc *sc)
{

	if (sc->ppe_foe == NULL)
		return;
	if (sc->ppe_enabled) {
		MTGE_LOCK(sc);
		mtge_ppe_disable(sc);
                MTGE_UNLOCK(sc);
	}
	bus_dmamap_unload(sc->ppe_foe_tag, sc->ppe_foe_map);
	bus_dmamem_free(sc->ppe_foe_tag, sc->ppe_foe, sc->ppe_foe_map);
	bus_dma_tag_destroy(sc->ppe_foe_tag);
	sc->ppe_foe = NULL;
	sc->ppe_foe_tag = NULL;
	mtx_destroy(&sc->ppe_mtx);
}

static int
mtge_ppe_enable(struct mtge_softc *sc)
{
	uint32_t val;

        MTGE_ASSERT_LOCKED(sc);

	memset(sc->ppe_foe, 0, MT_PPE_ENTRIES * MT_PPE_ENTRY_SIZE);
	bus_dmamap_sync(sc->ppe_foe_tag, sc->ppe_foe_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	MT_WRITE(sc, MT_PPE_TB_BASE, (uint32_t)sc->ppe_foe_phys);

	val = MT_PPE_ENTRIES_SHIFT |
	    PPE_TB_CFG_ENTRY_80B |
	    PPE_SEARCH_MISS_FORWARD_BUILD << PPE_TB_CFG_SEARCH_MISS_SHIFT |
	    PPE_TB_CFG_AGE_NON_L4 |
	    PPE_TB_CFG_AGE_UNBIND |
	    PPE_TB_CFG_AGE_TCP |
	    PPE_TB_CFG_AGE_UDP |
	    PPE_TB_CFG_AGE_TCP_FIN |
	    PPE_KEEPALIVE_DISABLE << PPE_TB_CFG_KEEPALIVE_SHIFT |
	    1 << PPE_TB_CFG_HASH_MODE_SHIFT |
	    PPE_SCAN_MODE_KEEPALIVE_AGE << PPE_TB_CFG_SCAN_MODE_SHIFT;
	MT_WRITE(sc, MT_PPE_TB_CFG, val);

	MT_WRITE(sc, MT_PPE_IP_PROTO_CHK,
	    PPE_IP_PROTO_CHK_IPV4 | PPE_IP_PROTO_CHK_IPV6);

	MT_WRITE(sc, MT_PPE_CACHE_CTL, PPE_CACHE_CTL_CLEAR);
	MT_WRITE(sc, MT_PPE_CACHE_CTL, PPE_CACHE_CTL_EN);

	MT_WRITE(sc, MT_PPE_FLOW_CFG,
	    PPE_FLOW_CFG_IP4_TCP_FRAG |
	    PPE_FLOW_CFG_IP4_UDP_FRAG |
	    PPE_FLOW_CFG_IP6_3T_ROUTE |
	    PPE_FLOW_CFG_IP6_5T_ROUTE |
	    PPE_FLOW_CFG_IP6_6RD |
	    PPE_FLOW_CFG_IP4_NAT |
	    PPE_FLOW_CFG_IP4_NAPT |
	    PPE_FLOW_CFG_IP4_DSLITE |
	    PPE_FLOW_CFG_IP4_NAT_FRAG);

	MT_WRITE(sc, MT_PPE_UNBIND_AGE,
	    1000 << PPE_UNBIND_AGE_MIN_PKT_SHIFT |
	    3 << PPE_UNBIND_AGE_DELTA_SHIFT);
	MT_WRITE(sc, MT_PPE_BIND_AGE0,
	    1 << PPE_BIND_AGE0_DELTA_NON_L4_SHIFT |
	    12 << PPE_BIND_AGE0_DELTA_UDP_SHIFT);
	MT_WRITE(sc, MT_PPE_BIND_AGE1,
	    1 << PPE_BIND_AGE1_DELTA_TCP_FIN_SHIFT |
	    7 << PPE_BIND_AGE1_DELTA_TCP_SHIFT);

	MT_WRITE(sc, MT_PPE_BIND_LIMIT0,
	    (MT_PPE_ENTRIES / 4) << PPE_BIND_LIMIT0_QUARTER_SHIFT |
	    (MT_PPE_ENTRIES / 2) << PPE_BIND_LIMIT0_HALF_SHIFT);
	MT_WRITE(sc, MT_PPE_BIND_LIMIT1,
	    MT_PPE_ENTRIES << PPE_BIND_LIMIT1_FULL_SHIFT |
	    1 << PPE_BIND_LIMIT1_NON_L4_SHIFT);
	MT_WRITE(sc, MT_PPE_BIND_RATE,
	    30 << PPE_BIND_RATE_BIND_SHIFT |
	    1 << PPE_BIND_RATE_PREBIND_SHIFT);

	MT_WRITE(sc, MT_PPE_GLO_CFG,
	    PPE_GLO_CFG_EN |
	    PPE_GLO_CFG_IP4_L4_CS_DROP |
	    PPE_GLO_CFG_IP4_CS_DROP |
	    PPE_GLO_CFG_FLOW_DROP_UPDATE);

	/* Punted packets go to PDMA ring 0, same as direct delivery. */
	MT_WRITE(sc, MT_PPE_DEFAULT_CPU_PORT, 0);

	mtge_ppe_gdm_dst(sc, GDM_DST_PORT_PPE);

	mtx_lock(&sc->ppe_mtx);
	sc->ppe_enabled = 1;
	mtx_unlock(&sc->ppe_mtx);

	return (0);
}

static void
mtge_ppe_disable(struct mtge_softc *sc)
{
	uint32_t val;
	int ntries;

	MTGE_ASSERT_LOCKED(sc);

	/* Traffic back to the CPU first; the engine drains, then stops. */
	mtge_ppe_gdm_dst(sc, GDM_DST_PORT_CPU);

	/*
	 * Flag down first, under the learn lock: mtge_ppe_tx_learn() checks
	 * it there before writing an entry, so once this is out no Tx path
	 * can be writing into the table being cleared below.
	 */
	mtx_lock(&sc->ppe_mtx);
	sc->ppe_enabled = 0;
	mtx_unlock(&sc->ppe_mtx);

	memset(sc->ppe_foe, 0, MT_PPE_ENTRIES * MT_PPE_ENTRY_SIZE);
	bus_dmamap_sync(sc->ppe_foe_tag, sc->ppe_foe_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	MT_WRITE(sc, MT_PPE_CACHE_CTL, PPE_CACHE_CTL_CLEAR);

	val = MT_READ(sc, MT_PPE_GLO_CFG);
	val &= ~PPE_GLO_CFG_EN;
	MT_WRITE(sc, MT_PPE_GLO_CFG, val);
	MT_WRITE(sc, MT_PPE_FLOW_CFG, 0);

	val = MT_READ(sc, MT_PPE_TB_CFG);
	val &= ~(PPE_TB_CFG_AGE_PREBIND | PPE_TB_CFG_AGE_NON_L4 |
	    PPE_TB_CFG_AGE_UNBIND | PPE_TB_CFG_AGE_TCP |
	    PPE_TB_CFG_AGE_UDP | PPE_TB_CFG_AGE_TCP_FIN);
	MT_WRITE(sc, MT_PPE_TB_CFG, val);

	for (ntries = 0; ntries < 1000; ntries++) {
		if ((MT_READ(sc, MT_PPE_GLO_CFG) & PPE_GLO_CFG_BUSY) == 0)
			break;
		DELAY(20);
	}
	if (ntries == 1000)
		device_printf(sc->dev, "PPE would not go idle\n");
}

/*
 * mtge_init_locked() reprograms the GDMAs to the CPU on every interface
 * cycle; this puts them back on the engine when it is meant to be on.
 */
static void
mtge_ppe_reapply(struct mtge_softc *sc)
{

	if (sc->ppe_foe != NULL && sc->ppe_enabled)
		mtge_ppe_gdm_dst(sc, GDM_DST_PORT_PPE);
}

/*
 * Pull the IPv4 5-tuple out of a frame.  Fills the tuple and returns zero
 * only for frames the engine can carry on its own: plain Ethernet, IPv4
 * without options, not a fragment, TCP or UDP.  Addresses and ports come
 * back in host byte order, which is how the FOE table wants them; the
 * identification and total length ride along to pin a note to one packet.
 */
static int
mtge_ppe_parse(const uint8_t *fp, int len, struct mtge_ppe_flow *f)
{

	if (len < ETHER_HDR_LEN + 20 + 4)
		return (1);
	if (be16dec(fp + 12) != ETHERTYPE_IP)
		return (1);
	fp += ETHER_HDR_LEN;
	if (fp[0] != 0x45)			/* IPv4, no options */
		return (1);
	if ((be16dec(fp + 6) & 0x3fff) != 0)	/* a fragment */
		return (1);
	f->proto = fp[9];
	if (f->proto != IPPROTO_TCP && f->proto != IPPROTO_UDP)
		return (1);
	f->ip_len = be16dec(fp + 2);
	f->ip_id = be16dec(fp + 4);
	f->sip = be32dec(fp + 12);
	f->dip = be32dec(fp + 16);
	f->sport = be16dec(fp + 20);
	f->dport = be16dec(fp + 22);
	return (0);
}

/*
 * Rx half of flow learning.  The engine punts frames it is tracking and
 * names the FOE slot it hashed them to; attach that slot to the packet so
 * the Tx side, which sees the routed and translated frame, knows without
 * guessing which flow it is looking at.
 */
static void
mtge_ppe_rx_note(struct mtge_softc *sc, struct mbuf *m, uint32_t word3)
{
	struct mtge_ppe_tag *pt;
	struct mtge_ppe_flow f;
	uint32_t hash, reason;

	reason = (word3 >> MT_RXD4_REASON_SHIFT) & MT_RXD4_REASON_MASK;
	sc->ppe_reason[reason]++;
	if (reason == MT_PPE_REASON_HIT_UNBIND_RATE)
		sc->ppe_rate_hits++;
	else if (reason != MT_PPE_REASON_HIT_UNBIND)
		return;

	/*
	 * Which punts are worth a binding.
	 *
	 * The engine asks politely once a flow gets busy (HIT_UNBIND_RATE),
	 * but waiting for that binds the two directions of a connection
	 * minutes apart in packet terms, and a connection with only one
	 * direction in hardware does not survive a stateful firewall: pf(4)
	 * keeps window-tracking the half it still sees, and acknowledgements
	 * for data that no longer passes the CPU fall outside the window it
	 * remembers and are dropped.  So the useful punt is the plain
	 * HIT_UNBIND one, which the engine raises from the first packets it
	 * tracks: both directions reach hardware within a packet or two of
	 * each other, while that state is still fresh.
	 *
	 * Which leaves the rate punt worth nothing here, and not free: the
	 * engine raises it for every packet of a flow it is still tracking
	 * once the flow passes a thousand packets, and a flow that could be
	 * bound was bound long before that.  What is left raising it is
	 * traffic that ends on this box, which has no egress frame to learn
	 * from and never will -- and tagging every one of those packets is a
	 * per-packet allocation on the fastest path there is.  Nearly a
	 * million such punts showed up in one afternoon of testing.
	 */
	if (reason != (sc->ppe_bind == MT_PPE_LEARN_RATE ?
	    MT_PPE_REASON_HIT_UNBIND_RATE : MT_PPE_REASON_HIT_UNBIND))
		return;
	if (sc->ppe_bind == MT_PPE_LEARN_OFF)
		return;

	hash = word3 & MT_RXD4_FOE_ENTRY_MASK;
	if (hash >= MT_PPE_ENTRIES)
		return;
	if (mtge_ppe_parse(mtod(m, const uint8_t *), m->m_len, &f) != 0)
		return;

	pt = (struct mtge_ppe_tag *)m_tag_alloc(MT_PPE_TAG_COOKIE,
	    MT_PPE_TAG_TYPE, sizeof(*pt) - sizeof(struct m_tag), M_NOWAIT);
	if (pt == NULL) {
		sc->ppe_tag_fail++;
		return;
	}
	pt->hash = hash;
	pt->f = f;
	m_tag_prepend(m, &pt->tag);
}

/*
 * Tx half of flow learning.  The packet carries the FOE slot the engine
 * chose for it on the way in; what it looks like now -- after routing and
 * NAT -- is what the engine has to rewrite it into.  Fill the rest of the
 * entry from this frame and flip it to BIND.
 */
static void
mtge_ppe_tx_learn(struct mtge_softc *sc, struct mbuf *m)
{
	struct mtge_ppe_tag *pt;
	struct mtge_ppe_flow txf;
	uint32_t *e;
	uint32_t ib1;
	int state;
	uint8_t fp[ETHER_HDR_LEN + 20 + 4];

	pt = (struct mtge_ppe_tag *)m_tag_locate(m, MT_PPE_TAG_COOKIE,
	    MT_PPE_TAG_TYPE, NULL);
	if (pt == NULL)
		return;
	if (m->m_pkthdr.len < (int)sizeof(fp))
		return;
	m_copydata(m, 0, sizeof(fp), (caddr_t)fp);
	if (mtge_ppe_parse(fp, sizeof(fp), &txf) != 0)
		return;
	/* A packet that came in as one protocol cannot leave as another. */
	if (txf.proto != pt->f.proto)
		return;

	mtx_lock(&sc->ppe_mtx);
	if (sc->ppe_enabled == 0 || sc->ppe_foe == NULL) {
		mtx_unlock(&sc->ppe_mtx);
		return;
	}

	/*
	 * The slot must still hold the engine's own UNBIND record of this
	 * flow: same state, same protocol, same original tuple.  Already
	 * bound means another packet of the flow got here first, which is
	 * the common case once a flow is going and not a disagreement.
	 */
	e = (uint32_t *)((char *)sc->ppe_foe + pt->hash * MT_PPE_ENTRY_SIZE);
	ib1 = le32toh(e[MT_FOE_W_IB1]);
	state = (ib1 & MT_FOE_IB1_STATE_MASK) >> MT_FOE_IB1_STATE_SHIFT;
	if (state == MT_FOE_STATE_BIND) {
		mtx_unlock(&sc->ppe_mtx);
		return;
	}
	if (state != MT_FOE_STATE_UNBIND ||
	    le32toh(e[MT_FOE_W_ORIG_SIP]) != pt->f.sip ||
	    le32toh(e[MT_FOE_W_ORIG_DIP]) != pt->f.dip ||
	    le32toh(e[MT_FOE_W_ORIG_PORTS]) !=
	    ((uint32_t)pt->f.sport << 16 | pt->f.dport)) {
		sc->ppe_slot_mismatch++;
		mtx_unlock(&sc->ppe_mtx);
		return;
	}

	/*
	 * Fill the body, then flip the first word to BIND, in that order:
	 * the engine must never see a half-written bound entry.  The table
	 * is uncached, so the syncs are pure store ordering.  Everything
	 * past the tuple is written, the tail included -- an entry that has
	 * carried a flow before still holds its remains.
	 */
	e[MT_FOE_W_NEW_SIP] = htole32(txf.sip);
	e[MT_FOE_W_NEW_DIP] = htole32(txf.dip);
	e[MT_FOE_W_NEW_PORTS] = htole32((uint32_t)txf.sport << 16 |
	    txf.dport);
	e[8] = 0;		/* engine bookkeeping from the unbound life */
	e[9] = 0;
	e[10] = 0;
	e[MT_FOE_W_VLAN_ETYPE] = htole32((uint32_t)ETHERTYPE_IP << 16);
	e[MT_FOE_W_DMAC_HI] = htole32((uint32_t)fp[0] << 24 |
	    fp[1] << 16 | fp[2] << 8 | fp[3]);
	e[MT_FOE_W_VLAN2_DMAC_LO] = htole32(
	    ((uint32_t)fp[4] << 8 | fp[5]) << 16);
	e[MT_FOE_W_SMAC_HI] = htole32((uint32_t)fp[6] << 24 |
	    fp[7] << 16 | fp[8] << 8 | fp[9]);
	e[MT_FOE_W_PPPOE_SMAC_LO] = htole32(
	    ((uint32_t)fp[10] << 8 | fp[11]) << 16);
	e[16] = 0;		/* tail: newer chips only, zero as they do */
	e[17] = 0;
	e[18] = 0;
	e[19] = 0;
	e[MT_FOE_W_IB2] = htole32(
	    (m->m_pkthdr.rcvif == sc->ifp1 ? MT_FOE_PSE_PORT_GDM2 :
	    MT_FOE_PSE_PORT_GDM1) << MT_FOE_IB2_DEST_PORT_SHIFT |
	    0x1f << MT_FOE_IB2_PORT_AG_SHIFT);

	ib1 = (MT_READ(sc, MT_FE_TIMESTAMP) & MT_FOE_IB1_TS_MASK) |
	    MT_FOE_IB1_BIND_CACHE | MT_FOE_IB1_BIND_TTL |
	    MT_FOE_PKT_IPV4_HNAPT << MT_FOE_IB1_PKT_TYPE_SHIFT |
	    MT_FOE_STATE_BIND << MT_FOE_IB1_STATE_SHIFT;
	if (txf.proto == IPPROTO_UDP)
		ib1 |= MT_FOE_IB1_UDP;

	bus_dmamap_sync(sc->ppe_foe_tag, sc->ppe_foe_map,
	    BUS_DMASYNC_PREWRITE);
	e[MT_FOE_W_IB1] = htole32(ib1);
	bus_dmamap_sync(sc->ppe_foe_tag, sc->ppe_foe_map,
	    BUS_DMASYNC_PREWRITE);

	/* The entry cache does not watch memory; make it look again. */
	MT_WRITE(sc, MT_PPE_CACHE_CTL,
	    PPE_CACHE_CTL_EN | PPE_CACHE_CTL_CLEAR);
	MT_WRITE(sc, MT_PPE_CACHE_CTL, PPE_CACHE_CTL_EN);

	sc->ppe_bound++;
	mtx_unlock(&sc->ppe_mtx);
}

static int
mtge_ppe_sysctl_enable(SYSCTL_HANDLER_ARGS)
{
	struct mtge_softc *sc = arg1;
	int error, val;

	val = sc->ppe_enabled;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (sc->ppe_foe == NULL)
		return (ENXIO);
	val = (val != 0);

	MTGE_LOCK(sc);
	if (val != sc->ppe_enabled) {
		if (val)
			error = mtge_ppe_enable(sc);
		else
			mtge_ppe_disable(sc);
	}
	MTGE_UNLOCK(sc);

	return (error);
}

static int
mtge_ppe_sysctl_dump(SYSCTL_HANDLER_ARGS)
{
	struct mtge_softc *sc = arg1;
	struct sbuf *sb;
	uint32_t *entry;
	const char *name;
	uint32_t ib1;
	int counts[4], error, i, shown, state;

	if (sc->ppe_foe == NULL)
		return (ENXIO);

	sb = sbuf_new_for_sysctl(NULL, NULL, 512, req);
	if (sb == NULL)
		return (ENOMEM);

	sbuf_printf(sb, "\nGLO_CFG  0x%08x  TB_CFG 0x%08x  FLOW_CFG 0x%08x\n",
	    MT_READ(sc, MT_PPE_GLO_CFG), MT_READ(sc, MT_PPE_TB_CFG),
	    MT_READ(sc, MT_PPE_FLOW_CFG));
	sbuf_printf(sb, "TB_BASE  0x%08x (table at 0x%jx, %d x %d B)\n",
	    MT_READ(sc, MT_PPE_TB_BASE), (uintmax_t)sc->ppe_foe_phys,
	    MT_PPE_ENTRIES, MT_PPE_ENTRY_SIZE);
	sbuf_printf(sb, "GDM1_IG  0x%08x  GDM2_IG 0x%08x\n",
	    MT_READ(sc, MT_GDM_IG_CTRL(0)), MT_READ(sc, MT_GDM_IG_CTRL(1)));
	sbuf_printf(sb, "enabled %d bind %d  fe timestamp 0x%04x\n",
	    sc->ppe_enabled, sc->ppe_bind,
	    MT_READ(sc, MT_FE_TIMESTAMP) & MT_FOE_IB1_TS_MASK);
	sbuf_printf(sb, "learn rate_hits %lu bound %lu mismatch %lu "
	    "tag_fail %lu\n", sc->ppe_rate_hits, sc->ppe_bound,
	    sc->ppe_slot_mismatch, sc->ppe_tag_fail);

	/*
	 * What the engine did with the frames it handed over, and what the
	 * two GDMAs have carried.  A working binding shows up as gdm2 tx
	 * climbing while the punt reasons stop climbing.
	 */
	sbuf_cat(sb, "punts ");
	for (i = 0; i < MT_PPE_REASON_COUNT; i++) {
		if (sc->ppe_reason[i] == 0)
			continue;
		switch (i) {
		case MT_PPE_REASON_NO_FLOW:
			name = "no-flow";
			break;
		case MT_PPE_REASON_UN_HIT:
			name = "un-hit";
			break;
		case MT_PPE_REASON_HIT_UNBIND:
			name = "unbind";
			break;
		case MT_PPE_REASON_HIT_UNBIND_RATE:
			name = "rate";
			break;
		case MT_PPE_REASON_HIT_BIND_TCP_FIN:
			name = "fin";
			break;
		case MT_PPE_REASON_HIT_TTL_1:
			name = "ttl1";
			break;
		case MT_PPE_REASON_HIT_BIND_VLAN_VIOL:
			name = "vlan-viol";
			break;
		case MT_PPE_REASON_HIT_BIND_FORCE_CPU:
			name = "force-cpu";
			break;
		default:
			name = "?";
			break;
		}
		sbuf_printf(sb, "%02x/%s %lu  ", i, name, sc->ppe_reason[i]);
	}
	sbuf_printf(sb, "\nlast  rxd4 0x%08x (foe %u reason 0x%02x sport %u)\n",
	    sc->rx_last_word3, sc->rx_last_word3 & MT_RXD4_FOE_ENTRY_MASK,
	    (sc->rx_last_word3 >> MT_RXD4_REASON_SHIFT) & MT_RXD4_REASON_MASK,
	    (sc->rx_last_word3 >> MT_RXD4_SPORT_SHIFT) & MT_RXD4_SPORT_MASK);
	sbuf_printf(sb, "gdm   gdm1 rx %lu tx %lu  gdm2 rx %lu tx %lu\n",
	    sc->rx_packets, sc->tx_packets, sc->rx_packets1, sc->tx_packets1);

	/*
	 * Histogram the entry states.  With the engine in pass-through,
	 * UNBIND entries appearing here prove it sees the flows.
	 */
	counts[0] = counts[1] = counts[2] = counts[3] = 0;
	bus_dmamap_sync(sc->ppe_foe_tag, sc->ppe_foe_map,
	    BUS_DMASYNC_POSTREAD);
	for (i = 0; i < MT_PPE_ENTRIES; i++) {
		entry = (uint32_t *)((char *)sc->ppe_foe +
		    i * MT_PPE_ENTRY_SIZE);
		state = (le32toh(entry[0]) & MT_FOE_IB1_STATE_MASK) >>
		    MT_FOE_IB1_STATE_SHIFT;
		counts[state]++;
	}
	sbuf_printf(sb, "foe   invalid %d unbind %d bind %d fin %d\n",
	    counts[MT_FOE_STATE_INVALID], counts[MT_FOE_STATE_UNBIND],
	    counts[MT_FOE_STATE_BIND], counts[MT_FOE_STATE_FIN]);

	/*
	 * Show the live entries themselves -- what the engine wrote while
	 * tracking and what a binding put there, straight from the table.
	 */
	shown = 0;
	for (i = 0; i < MT_PPE_ENTRIES && shown < 8; i++) {
		entry = (uint32_t *)((char *)sc->ppe_foe +
		    i * MT_PPE_ENTRY_SIZE);
		ib1 = le32toh(entry[MT_FOE_W_IB1]);
		if (((ib1 & MT_FOE_IB1_STATE_MASK) >>
		    MT_FOE_IB1_STATE_SHIFT) == MT_FOE_STATE_INVALID)
			continue;
		sbuf_printf(sb, "[%04x] ib1 %08x ib2 %08x\n"
		    "       orig %08x:%04x > %08x:%04x  "
		    "new %08x:%04x > %08x:%04x\n",
		    i, ib1, le32toh(entry[MT_FOE_W_IB2]),
		    le32toh(entry[MT_FOE_W_ORIG_SIP]),
		    le32toh(entry[MT_FOE_W_ORIG_PORTS]) >> 16,
		    le32toh(entry[MT_FOE_W_ORIG_DIP]),
		    le32toh(entry[MT_FOE_W_ORIG_PORTS]) & 0xffff,
		    le32toh(entry[MT_FOE_W_NEW_SIP]),
		    le32toh(entry[MT_FOE_W_NEW_PORTS]) >> 16,
		    le32toh(entry[MT_FOE_W_NEW_DIP]),
		    le32toh(entry[MT_FOE_W_NEW_PORTS]) & 0xffff);
		shown++;
	}

	error = sbuf_finish(sb);
	sbuf_delete(sb);
	return (error);
}

static void
mtge_mac_change(struct mtge_softc *sc, uint32_t media, int gmac)
{

	uint32_t reg;
	/*
	 * MAC_RX_PKT_LEN lives in bits 25:24, not next to IPG_CFG: with the
	 * old shift the frame length selector was OR'd straight into the
	 * IPG_CFG field and silently changed the inter-packet gap.
	 */
	reg = (IPG_CFG_96BIT_SHORT << IPG_CFG_SHIFT) |
	    (MAC_RX_PKT_LEN_1536 << MAC_RX_PKT_LEN_SHIFT) |
	    MAC_MODE | FORCE_MODE |
	    MAC_TX_EN | MAC_RX_EN |
	    BKOFF_EN | BACKPR_EN |
	    FORCE_LINK;

	switch (IFM_SUBTYPE(media)) {
	case IFM_10_T:
		reg |= (FORCE_SPD_10M << FORCE_SPD_SHIFT);
		break;
	case IFM_100_TX:
		reg |= (FORCE_SPD_100M << FORCE_SPD_SHIFT);
		break;
	case IFM_1000_T:
	case IFM_1000_SX:
	case IFM_2500_T:
	case IFM_2500_SX:
	case IFM_2500_X:
	case IFM_2500_KX:
		/*
		 * FORCE_SPD only encodes 10, 100 and 1000 -- there is no 2500
		 * setting in this register, and none is wanted.  On a
		 * 2500base-X trunk the rate comes from the serdes PCS; the MAC
		 * side stays at the gigabit selector, which is also what MSR
		 * then reads back.  A "speed=1000" there is not a 1G trunk.
		 */
		reg |= (FORCE_SPD_1000M << FORCE_SPD_SHIFT);
		break;
	default:
		return;
	}

	if ((IFM_OPTIONS(media) & IFM_FDX))
		reg |= FORCE_DPX;
	else
		reg &= ~FORCE_DPX;

	/*
	 * FORCE_MODE is set above, so the MAC takes flow control from these
	 * bits rather than from an autonegotiation that never runs on a
	 * forced link.  Both trunks ask for it in their "fixed-link" node,
	 * and the 2500base-X one needs it: four gigabit jacks feed the
	 * switch behind a single trunk, and without a pause frame the
	 * overflow is a drop.
	 */
	if (sc->flowctl) {
		if ((IFM_OPTIONS(media) & IFM_ETH_RXPAUSE))
			reg |= FORCE_RX_FC;
		if ((IFM_OPTIONS(media) & IFM_ETH_TXPAUSE))
			reg |= FORCE_TX_FC;
	}

	sc->trunk_media[gmac] = media;
	MT_WRITE(sc, MAC_P_MCR(gmac), reg);

	device_printf(sc->dev, "%s MAC_%iMCR  0x%x\n", __func__,
	    gmac, MT_READ(sc, MAC_P_MCR(gmac)));
}

/*
 * Locate a "mac@<which>" subnode, which is where a device tree describes one
 * of the frame engine's two MACs.
 */
#ifdef FDT
static phandle_t
mtge_gmac_node(device_t dev, uint32_t which)
{
	phandle_t node, child;
	uint32_t reg;

	node = ofw_bus_get_node(dev);
	if (node == -1)
		return (0);

	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		if (!ofw_bus_node_is_compatible(child, "mediatek,eth-mac"))
			continue;
		if (OF_getencprop(child, "reg", &reg, sizeof(reg)) <= 0)
			continue;
		if (reg == which)
			return (child);
	}

	return (0);
}

/*
 * Take the address from a device-tree node, under either of the two names
 * boards and boot loaders use.  All-zeroes and multicast are what a node
 * that was never filled in looks like.
 */
static bool
mtge_mac_from_node(phandle_t node, uint8_t *eaddr)
{
	static const char *props[] = { "local-mac-address", "mac-address" };
	int i;
	uint8_t buf[ETHER_ADDR_LEN];

	if (node == 0)
		return (false);

	for (i = 0; i < nitems(props); i++) {
		if (OF_getprop(node, props[i], buf, sizeof(buf)) !=
		    ETHER_ADDR_LEN)
			continue;
		if (ETHER_IS_MULTICAST(buf))
			continue;
		if ((buf[0] | buf[1] | buf[2] | buf[3] | buf[4] | buf[5]) == 0)
			continue;
		memcpy(eaddr, buf, ETHER_ADDR_LEN);
		return (true);
	}

	return (false);
}

/* Hash the board serial number, so a synthesized address at least stays put. */
static uint32_t
mtge_board_hash(void)
{
	phandle_t root;
	uint32_t h;
	int i, len;
	char serial[64];

	h = 2166136261u;				/* FNV-1a basis */
	root = OF_finddevice("/");
	if (root == -1)
		return (h);

	len = OF_getprop(root, "serial-number", serial, sizeof(serial) - 1);
	if (len <= 0)
		return (h);

	serial[len] = '\0';
	for (i = 0; serial[i] != '\0'; i++)
		h = (h ^ (uint8_t)serial[i]) * 16777619u;

	return (h);
}
#endif

static int
mtge_hexval(char c)
{

	if (c >= '0' && c <= '9')
		return (c - '0');
	if (c >= 'a' && c <= 'f')
		return (c - 'a' + 10);
	if (c >= 'A' && c <= 'F')
		return (c - 'A' + 10);

	return (-1);
}

/* Parse "xx:xx:xx:xx:xx:xx", also accepting '-' as the separator. */
static bool
mtge_mac_parse(const char *s, uint8_t *eaddr)
{
	int hi, i, lo;

	for (i = 0; i < ETHER_ADDR_LEN; i++) {
		hi = mtge_hexval(*s++);
		lo = mtge_hexval(*s++);
		if (hi < 0 || lo < 0)
			return (false);
		eaddr[i] = (hi << 4) | lo;
		if (i == ETHER_ADDR_LEN - 1)
			break;
		if (*s != ':' && *s != '-')
			return (false);
		s++;
	}

	return (*s == '\0' && !ETHER_IS_MULTICAST(eaddr));
}

/*
 * mtge_ether_request_mac - find this port's MAC address.
 *
 * It used to be drawn from arc4random() on every attach, which gave the
 * interface a different address on every boot: a DHCP server sees a new
 * client each time, a reservation never matches, and a DHCPREQUEST for the
 * lease the previous boot took is simply ignored.
 *
 * Take it from the device tree, which is where a board description or a boot
 * loader's fixup puts it, then from the hw.mtge.macaddr tunable for a device
 * tree that carries none.  Only if both are missing synthesize one, and do it
 * from the board serial number so that it at least stays the same across
 * boots of this board.
 */
static void
mtge_ether_request_mac(device_t dev, uint8_t *eaddr)
{
	uint32_t h;
	char buf[32];

#ifdef FDT
	if (mtge_mac_from_node(mtge_gmac_node(dev, 0), eaddr))
		return;
	if (mtge_mac_from_node(ofw_bus_get_node(dev), eaddr))
		return;
#endif

	if (TUNABLE_STR_FETCH("hw.mtge.macaddr", buf, sizeof(buf)) != 0) {
		if (mtge_mac_parse(buf, eaddr))
			return;
		device_printf(dev, "hw.mtge.macaddr=\"%s\" is not a unicast "
		    "MAC address, ignoring\n", buf);
	}

	h = 2166136261u;			/* FNV-1a basis */
#ifdef FDT
	h = mtge_board_hash();
#endif
	if (h == 2166136261u)
		device_printf(dev, "no MAC address in the device tree and no "
		    "board serial number to derive one from; set "
		    "hw.mtge.macaddr to keep this board's address unique\n");
	h = (h ^ (uint32_t)device_get_unit(dev)) * 16777619u;

	/* Locally administered, unicast. */
	eaddr[0] = 0x02;
	eaddr[1] = 0x00;
	eaddr[2] = (h >> 24) & 0xff;
	eaddr[3] = (h >> 16) & 0xff;
	eaddr[4] = (h >> 8) & 0xff;
	eaddr[5] = h & 0xff;
}

/*
 * Set mac addr
 */
static void
mtge_mac_addr(struct mtge_softc *sc, int gmac)
{

	if_t ifp = sc->ifp;
	const uint8_t *eaddr;
	uint32_t val;

	/* Write our unicast address */
	eaddr = if_getlladdr(ifp);

	val = eaddr[1] | (eaddr[0] << 8);
	MT_WRITE(sc, MT_GDM_MAC_MSB(gmac), val);

	val = eaddr[5] | (eaddr[4] << 8) | (eaddr[3] << 16) |
	    (eaddr[2] << 24);
	MT_WRITE(sc, MT_GDM_MAC_LSB(gmac), val);
}

#ifdef FDT
static uint32_t
mtge_fixed_link_media(device_t dev, uint32_t which)
{
	phandle_t node, child, fixedlink;
	uint32_t reg, speed;
	uint32_t media = IFM_ETHER;

	node = ofw_bus_get_node(dev);
	if (node == -1)
		return (IFM_ETHER | IFM_1000_T | IFM_FDX);

	/* Locate the wanted MAC subnode and its fixed-link child. */
	fixedlink = 0;
	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		if (!ofw_bus_node_is_compatible(child, "mediatek,eth-mac"))
			continue;
		if (OF_getencprop(child, "reg", &reg, sizeof(reg)) <= 0)
			continue;
		if (reg != which)
			continue;
		fixedlink = ofw_bus_find_child(child, "fixed-link");
		break;
	}
	if (fixedlink == 0) {
		return (IFM_ETHER | IFM_1000_T | IFM_FDX);
	}

	if (OF_getencprop(fixedlink, "speed", &speed, sizeof(speed)) <= 0) {
		speed = 1000;
	}

	switch (speed) {
	case 10:
		media |= IFM_10_T;
		break;
	case 100:
		media |= IFM_100_TX;
		break;
	case 2500:
		/*
		 * IFM_2500_X, not IFM_2500_T: this trunk is a serdes to the
		 * switch, not NBaseT over twisted pair.  Both are extended
		 * subtypes, so an ifconfig(8) older than the media table shows
		 * either as "Other".
		 */
		media |= IFM_2500_X;
		break;
	case 1000:
	default:
		media |= IFM_1000_T;
		break;
	}

	if (OF_hasprop(fixedlink, "full-duplex")) {
		media |= IFM_FDX;
	}
	else {
		media |= IFM_HDX;
	}
	if (OF_hasprop(fixedlink, "pause")) {
		media |= IFM_ETH_RXPAUSE | IFM_ETH_TXPAUSE;
	}

	return (media);
}
#else
static uint32_t
mtge_fixed_link_media(device_t dev, uint32_t which)
{
	return (IFM_ETHER | IFM_1000_T | IFM_FDX);
}
#endif

/*
 * Bring up gmac1 as a second interface when the device tree describes it.
 * On the BPI-R64 that is the RGMII link to switch port 5, which makes a
 * natural WAN trunk: LAN traffic rides gmac0 and port 6, WAN gets its own
 * MAC, and routed traffic stops crossing one trunk twice.
 *
 * It is a second station on the same engine, not a second engine -- rings,
 * task queues and interrupts stay shared, frames part ways at the GDMAs.
 * The engine follows mtge0: mtge1 up/down only gates its own traffic.
 */
static void
mtge_attach_port1(struct mtge_softc *sc)
{
#ifdef FDT
	device_t dev = sc->dev;
	if_t ifp;
	uint32_t media, val;

	if (mtge_gmac_node(dev, 1) == 0)
		return;

	if (!mtge_mac_from_node(mtge_gmac_node(dev, 1), sc->mac_addr1)) {
		/*
		 * No address of its own in the tree: derive one from port 0's,
		 * locally administered so it cannot collide with a real OUI.
		 */
		memcpy(sc->mac_addr1, sc->mac_addr, ETHER_ADDR_LEN);
		sc->mac_addr1[0] |= 0x02;
		sc->mac_addr1[5] += 1;
	}

	ifp = sc->ifp1 = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "could not if_alloc() gmac1\n");
		return;
	}

	if_setsoftc(ifp, sc);
	if_initname(ifp, "mtge", 1);
	if_setflags(ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setinitfn(ifp, mtge_init);
	if_setioctlfn(ifp, mtge_ioctl);
	if_settransmitfn(ifp, mtge_transmit);
	if_setqflushfn(ifp, mtge_qflush);

	media = mtge_fixed_link_media(dev, 1);
	ifmedia_init(&sc->ifmedia1, 0, mtge_ifmedia_upd, mtge_ifmedia_sts);
	ifmedia_add(&sc->ifmedia1, media, 0, NULL);
	ifmedia_set(&sc->ifmedia1, media);

	ether_ifattach(ifp, sc->mac_addr1);

	if_setifheaderlen(ifp, sizeof(struct ether_vlan_header));
	if_setcapabilitiesbit(ifp, IFCAP_VLAN_MTU, 0);
	if_setcapenablebit(ifp, IFCAP_VLAN_MTU, 0);
	if_setcapabilitiesbit(ifp, IFCAP_TXCSUM, 0);
	if_setcapenablebit(ifp, IFCAP_TXCSUM, 0);
	if_sethwassistbits(ifp, CSUM_IP | CSUM_TCP | CSUM_UDP, 0);

	/*
	 * Segmentation offload.  The engine cuts a window-sized frame into
	 * MSS-sized ones itself, which is the piece the Rx side has had all
	 * along in LRO and the Tx side has not: without it the stack builds
	 * every segment on the CPU, and this board sends at half the rate it
	 * receives.  The limits below are what tcp_output(9) is asked to
	 * respect when it assembles the chain.
	 */
	if_setcapabilitiesbit(ifp, IFCAP_TSO4, 0);
	if_setcapenablebit(ifp, IFCAP_TSO4, 0);
	if_sethwassistbits(ifp, CSUM_TSO, 0);
	if_sethwtsomax(ifp, MT_SOFTC_TSO_MAX_PAYLOAD);
	if_sethwtsomaxsegcount(ifp, MT_SOFTC_TSO_MAX_SEGS);
	if_sethwtsomaxsegsize(ifp, PAGE_SIZE);
	/*
	 * Rx checksum yes -- the bit is per-frame and GDMA-agnostic.  LRO no:
	 * a WAN trunk carries forwarded traffic, and one lro_ctrl cannot
	 * serve two interfaces anyway.
	 */
	if_setcapabilitiesbit(ifp, IFCAP_RXCSUM, 0);
	if_setcapenablebit(ifp, IFCAP_RXCSUM, 0);

	/* MAC 1 line parameters and its GDMA: forward to CPU, own station. */
	mtge_mac_change(sc, media, 1);
	mtge_gdm_fwd_cpu(sc, 1);
	val = sc->mac_addr1[1] | (sc->mac_addr1[0] << 8);
	MT_WRITE(sc, MT_GDM_MAC_MSB(1), val);
	val = sc->mac_addr1[5] | (sc->mac_addr1[4] << 8) |
	    (sc->mac_addr1[3] << 16) | (sc->mac_addr1[2] << 24);
	MT_WRITE(sc, MT_GDM_MAC_LSB(1), val);

	if (bootverbose)
		device_printf(dev, "gmac1 attached as mtge1, "
		    "Ethernet address %6D\n", sc->mac_addr1, ":");
#endif
}

/*
 * Set media options.
 */
static int
mtge_ifmedia_upd(if_t ifp)
{
	struct mtge_softc *sc;
	struct ifmedia *ifm;
	struct ifmedia_entry *ife;

	sc = if_getsoftc(ifp);
	ifm = &sc->ifmedia;
	ife = ifm->ifm_cur;

	if (IFM_TYPE(ifm->ifm_media) != IFM_ETHER)
		return (EINVAL);

	if (IFM_SUBTYPE(ife->ifm_media) == IFM_AUTO) {
		device_printf(sc->dev,
		    "AUTO is not supported for multiphy MAC");
		return (EINVAL);
	}

	/*
	 * Ignore everything
	 */
	return (0);
}

/*
 * Report current media status.
 */
static void
mtge_ifmedia_sts(if_t ifp, struct ifmediareq *ifmr)
{
	struct mtge_softc *sc = if_getsoftc(ifp);

	/*
	 * ifmedia_ioctl() has already filled in ifm_active/ifm_current from
	 * the configured (fixed-link) media; leave those alone.  Rebuilding
	 * them from MSR would be a step backwards, because MSR reports the
	 * MAC-side speed selector -- 1000 even on a 2500base-X trunk, where
	 * the rate lives in the serdes -- while the configured word names the
	 * medium correctly.
	 *
	 * The link state is the part worth reading from hardware.  It used to
	 * come from sc->link_up, which mtge_init_locked() sets
	 * unconditionally, so "status: active" meant only that the interface
	 * had been brought up.  MSR_LINK is what the MAC actually sees on the
	 * line to the switch -- and that line is always up, so the switch's
	 * word on the jacks behind it is folded in too (mtge_link_state()).
	 */
	ifmr->ifm_status = IFM_AVALID;
	if (mtge_link_state(sc, ifp) == LINK_STATE_UP)
		ifmr->ifm_status |= IFM_ACTIVE;
}

static int
mtge_detach(device_t dev)
{
	struct mtge_softc *sc;
	struct mbuf *m;
	if_t ifp;
	int error = 0, i;

	sc = device_get_softc(dev);
	ifp = sc->ifp;

	/*
	 * Take the engine down the way "ifconfig down" does, and keep it
	 * down: an "up" racing this would otherwise bring it back between
	 * here and the ring frees.  Then wait for whatever was still running,
	 * in process context and without the lock -- the drains sleep, and
	 * periodic_ch has no lock of its own, so callout_stop() alone does not
	 * wait for a firing already under way.  The callout is drained first
	 * so that anything it enqueues is caught by the task drains after it.
	 */
	MTGE_LOCK(sc);
	sc->detaching = 1;
	mtge_stop_locked(sc);
	MTGE_UNLOCK(sc);

	callout_drain(&sc->periodic_ch);

	/*
	 * mtge_stop_locked() blocked the queues, and a blocked queue does
	 * not wake its thread for a task that is already pending -- a drain
	 * would wait for that task forever.  Let them run: every task checks
	 * IFF_DRV_RUNNING first and that is off now, and nothing enqueues
	 * again with the engine's interrupts disabled.
	 */
	taskqueue_unblock(sc->rx_taskqueue);
	taskqueue_unblock(sc->tx_taskqueue);
	taskqueue_drain(sc->rx_taskqueue, &sc->rx_done_task);
	taskqueue_drain(sc->tx_taskqueue, &sc->tx_done_task);
	taskqueue_drain(sc->tx_taskqueue, &sc->periodic_task);
	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++)
		taskqueue_drain(sc->tx_taskqueue, &sc->tx_ring[i].tx_task);
	taskqueue_drain(taskqueue_thread, &sc->sw_task);

	/*
	 * Nothing runs against the rings now: the engine is stopped, the
	 * tasks are drained and IFF_DRV_RUNNING is off.  The ring lock still
	 * covers a transmit that passed that flag before it was cleared.
	 */
	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++) {
		MTGE_TX_RING_LOCK(&sc->tx_ring[i]);
		mtge_free_tx_ring(sc, &sc->tx_ring[i]);
		MTGE_TX_RING_UNLOCK(&sc->tx_ring[i]);
	}
	for (i = 0; i < sc->rx_ring_count; i++)
		mtge_free_rx_ring(sc, &sc->rx_ring[i]);

	ether_ifdetach(ifp);
	if_free(ifp);

	if (sc->ifp1 != NULL) {
		ether_ifdetach(sc->ifp1);
		ifmedia_removeall(&sc->ifmedia1);
		if_free(sc->ifp1);
		sc->ifp1 = NULL;
	}

	mtge_ppe_detach(sc);

	if (sc->lro_ok)
		tcp_lro_free(&sc->lro);

	taskqueue_free(sc->rx_taskqueue);
	taskqueue_free(sc->tx_taskqueue);

	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++) {
		if (sc->tx_ring[i].br == NULL)
			continue;
		while ((m = buf_ring_dequeue_sc(sc->tx_ring[i].br)) != NULL)
			m_freem(m);
		buf_ring_free(sc->tx_ring[i].br, M_DEVBUF);
		sc->tx_ring[i].br = NULL;
		mtx_destroy(&sc->tx_ring[i].lock);
	}

	bus_generic_detach(dev);
	for (i = 0; i < MT_MAX_INTRS; i++) {
                if(sc->irq[i] != NULL) {
                        bus_teardown_intr(dev,
                            sc->irq[i], sc->irqh[i]);
                        bus_release_resource(dev,
                            SYS_RES_IRQ, sc->irq_rid[i], sc->irq[i]);
                }
	}

	/* After the interrupt handlers are gone: they take this lock. */
	mtx_destroy(&sc->mtge_lock);

        if(sc->mem != NULL) {
                bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);
        }

        for (i = 0; i < MT_NCLKS; i++) {
                if(sc->clks[i] != NULL) {
                        error = clk_disable(sc->clks[i]);
                        if(error != 0 && bootverbose) {
                                device_printf(sc->dev,
                                    "failed to disable mtge clock: %d\n", error);
                        }
                }
        }

	return (error);
}

/*
 * mtge_init_locked - Run initialization process having locked mtx.
 */
static void
mtge_init_locked(void *priv)
{
	struct mtge_softc *sc;
	if_t ifp;
	int i, ntries;
	uint32_t tmp;
	int gmac = 0;

	sc = priv;
	ifp = sc->ifp;

	MTGE_ASSERT_LOCKED(sc);

	/* The rings are torn down; mtge_resize_rings() will init when done. */
	if (sc->resizing || sc->detaching)
		return;

	/* Fwd to CPU (uni|broad|multi)cast and Unknown */
	mtge_gdm_fwd_cpu(sc, gmac);
	if (sc->ifp1 != NULL)
		mtge_gdm_fwd_cpu(sc, 1);
	mtge_ppe_reapply(sc);

	/* disable DMA engine */
	MT_WRITE(sc, sc->pdma_glo_cfg, 0);
	MT_WRITE(sc, sc->pdma_rst_idx, 0xffffffff);

	/* wait while DMA engine is busy */
	for (ntries = 0; ntries < 100; ntries++) {
		tmp = MT_READ(sc, sc->pdma_glo_cfg);
		if (!(tmp & (FE_TX_DMA_BUSY | FE_RX_DMA_BUSY)))
			break;
		DELAY(1000);
	}

	if (ntries == 100) {
		device_printf(sc->dev, "timeout waiting for DMA engine\n");
		goto fail;
	}

	/* reset Rx and Tx rings */
	tmp = FE_RST_DRX_IDX1 |
	    FE_RST_DRX_IDX0 |
	    FE_RST_DTX_IDX3 |
	    FE_RST_DTX_IDX2 |
	    FE_RST_DTX_IDX1 |
	    FE_RST_DTX_IDX0;

	MT_WRITE(sc, sc->pdma_rst_idx, tmp);

	/* XXX switch set mac address */
	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++)
		mtge_reset_tx_ring(sc, &sc->tx_ring[i]);

	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++) {
		/* update TX_BASE_PTRx */
		MT_WRITE(sc, sc->tx_base_ptr[i],
		    sc->tx_ring[i].desc_phys_addr);
		MT_WRITE(sc, sc->tx_max_cnt[i], sc->tx_desc_size);
		if (MT_READ(sc, sc->tx_max_cnt[i]) !=
		    (uint32_t)sc->tx_desc_size)
			device_printf(sc->dev, "TX_MAX_CNT %d did not stick "
			    "(reads back %u); lower hw.mtge.tx_ring_size\n",
			    sc->tx_desc_size,
			    MT_READ(sc, sc->tx_max_cnt[i]));
		MT_WRITE(sc, sc->tx_ctx_idx[i], 0);
	}

	/* init Rx ring */
	for (i = 0; i < sc->rx_ring_count; i++)
		mtge_reset_rx_ring(sc, &sc->rx_ring[i]);

	/* update RX_BASE_PTRx */
	for (i = 0; i < sc->rx_ring_count; i++) {
		MT_WRITE(sc, sc->rx_base_ptr[i],
		    sc->rx_ring[i].desc_phys_addr);
		MT_WRITE(sc, sc->rx_max_cnt[i], sc->rx_ring_size);
		if (MT_READ(sc, sc->rx_max_cnt[i]) !=
		    (uint32_t)sc->rx_ring_size)
			device_printf(sc->dev, "RX_MAX_CNT %d did not stick "
			    "(reads back %u); lower hw.mtge.rx_ring_size\n",
			    sc->rx_ring_size,
			    MT_READ(sc, sc->rx_max_cnt[i]));
		MT_WRITE(sc, sc->rx_calc_idx[i], sc->rx_ring_size - 1);
	}

	/* write back DDONE, 16byte burst enable RX/TX DMA */
	tmp = FE_TX_WB_DDONE | FE_DMA_BT_SIZE16 | FE_RX_DMA_EN | FE_TX_DMA_EN;
	tmp |= FE_RX_2B_OFFSET;
	MT_WRITE(sc, sc->pdma_glo_cfg, tmp);

	mtge_set_coal(sc);

	/* clear pending interrupts */
	MT_WRITE(sc, sc->pdma_int_status, 0xffffffff);

	/*
	 * Start from a clean interrupt state.  mtge_stop_locked() leaves
	 * intr_disable_mask holding whatever a done-task had masked off when
	 * the interface went down, and mtge_tx_intr() / mtge_rx_intr() only
	 * enqueue their task while the bit is *not* in that mask - so a stale
	 * bit means the interrupt arrives, finds itself "already disabled",
	 * and no task ever runs to reclaim the ring.
	 */
	sc->intr_disable_mask = 0;
	sc->intr_pending_mask = 0;

	/* enable interrupts */
	mtge_set_intr_masks(sc);

	MT_WRITE(sc, sc->pdma_int_enable, sc->intr_enable_mask);

	/*
	 * mtge_stop_locked() blocked the task queues and nothing else lets
	 * them run again, so without this an interface that has been down
	 * once never reclaims another Tx descriptor or drains another Rx ring.
	 */
	taskqueue_unblock(sc->rx_taskqueue);
	taskqueue_unblock(sc->tx_taskqueue);

	if (mtge_txrx_enable(sc) != 0)
		goto fail;

	if_setdrvflagbits(ifp, 0, IFF_DRV_OACTIVE);
	if_setdrvflagbits(ifp, IFF_DRV_RUNNING, 0);

	/*
	 * Announce the link.  Without this the interface stays in
	 * LINK_STATE_UNKNOWN, ifconfig(8) shows an empty "status:" line and
	 * devd(8) never gets the IFNET/LINK_UP event that starts dhclient on
	 * a DHCP-configured interface at boot.
	 */
	sc->link_up = 1;
	if_link_state_change(ifp, mtge_link_state(sc, ifp));

	/* The engine is up; open the second port too if it is configured up. */
	if (sc->ifp1 != NULL && (if_getflags(sc->ifp1) & IFF_UP) != 0) {
		if_setdrvflagbits(sc->ifp1, IFF_DRV_RUNNING, 0);
		if_link_state_change(sc->ifp1, mtge_link_state(sc, sc->ifp1));
	}

	sc->periodic_round = 0;

	callout_reset(&sc->periodic_ch, hz / 10, mtge_periodic, sc);

	/*
	 * The Tx watchdog runs with the softc lock held and re-arms itself
	 * for as long as the interface is up; this is the one place it is
	 * started, with the lock its callout was initialized with.
	 */
	sc->tx_timer = 0;
	callout_reset(&sc->tx_watchdog_ch, hz, mtge_tx_watchdog, sc);

	return;

fail:
	mtge_stop_locked(sc);
}

/*
 * mtge_init - lock and initialize device.
 */
static void
mtge_init(void *priv)
{
	struct mtge_softc *sc;

	sc = priv;
	MTGE_LOCK(sc);
	mtge_init_locked(sc);
	MTGE_UNLOCK(sc);
}

/*
 * mtge_stop_locked - stop TX/RX w/ lock
 */
static void
mtge_stop_locked(void *priv)
{
	struct mtge_softc *sc;
	if_t ifp;
	int gmac, ntries;

	sc = priv;
	ifp = sc->ifp;
	gmac = 0;

	MTGE_ASSERT_LOCKED(sc);
	sc->tx_timer = 0;
	if_setdrvflagbits(ifp, 0, (IFF_DRV_RUNNING | IFF_DRV_OACTIVE));
	sc->link_up = 0;
	if_link_state_change(ifp, LINK_STATE_DOWN);
	if (sc->ifp1 != NULL) {
		if_setdrvflagbits(sc->ifp1, 0,
		    (IFF_DRV_RUNNING | IFF_DRV_OACTIVE));
		if_link_state_change(sc->ifp1, LINK_STATE_DOWN);
	}
	callout_stop(&sc->periodic_ch);
	callout_stop(&sc->tx_watchdog_ch);

	/*
	 * Keep the lock throughout.  taskqueue_block() does not sleep; the
	 * drain that would is left to the callers that go on to free the
	 * rings -- mtge_detach() and mtge_resize_rings() do it in process
	 * context once they have dropped the lock.  Dropping it here instead
	 * opened a window in which another "ifconfig up" could bring the
	 * engine back between this function's two halves, only to have its
	 * interrupts switched off again on return with IFF_DRV_RUNNING set.
	 */
	taskqueue_block(sc->rx_taskqueue);
	taskqueue_block(sc->tx_taskqueue);

	/* disable interrupts */
	MT_WRITE(sc, sc->pdma_int_enable, 0);

	/*
	 * Stop the engine as well, not only its interrupts: a ring freed
	 * behind a running PDMA is memory the engine keeps writing to.
	 */
	MT_WRITE(sc, sc->pdma_glo_cfg, 0);
	for (ntries = 0; ntries < 100; ntries++) {
		if ((MT_READ(sc, sc->pdma_glo_cfg) &
		    (FE_TX_DMA_BUSY | FE_RX_DMA_BUSY)) == 0)
			break;
		DELAY(1000);
	}
	if (ntries == 100)
		device_printf(sc->dev, "DMA engine would not stop\n");

	mtge_gdm_fwd_cpu(sc, gmac);
	if (sc->ifp1 != NULL)
		mtge_gdm_fwd_cpu(sc, 1);
}

/*
 * Prepare a frame the stack wants segmented.  This engine takes the segment
 * size from the TCP checksum field rather than from a descriptor field, and
 * works out the real checksum itself -- so the pseudo-header sum that
 * tcp_output(9) leaves there is of no use to it and gets overwritten, and
 * the checksum bits have to be set alongside the segmentation bit.
 *
 * Returns the frame to send, which m_pullup(9) may have moved, or NULL if
 * the headers could not be made contiguous.
 */
static struct mbuf *
mtge_tx_tso_prepare(struct mbuf *m)
{
	struct ether_header *eh;
	struct ip *ip;
	struct tcphdr *th;
	int hlen, off;

	off = ETHER_HDR_LEN;
	if ((m = m_pullup(m, off + sizeof(*ip))) == NULL)
		return (NULL);
	eh = mtod(m, struct ether_header *);
	if (ntohs(eh->ether_type) == ETHERTYPE_VLAN) {
		off = sizeof(struct ether_vlan_header);
		if ((m = m_pullup(m, off + sizeof(*ip))) == NULL)
			return (NULL);
	}

	ip = (struct ip *)(mtod(m, char *) + off);
	hlen = ip->ip_hl << 2;
	if ((m = m_pullup(m, off + hlen + sizeof(*th))) == NULL)
		return (NULL);

	th = (struct tcphdr *)(mtod(m, char *) + off + hlen);
	th->th_sum = htons(m->m_pkthdr.tso_segsz);

	return (m);
}

/*
 * mtge_tx_data - transmit packet.
 */
static int
mtge_tx_data(struct mtge_softc *sc, struct mbuf *m, int qid)
{
	if_t ifp;
	struct mtge_softc_tx_ring *ring;
	struct mtge_softc_tx_data *data;
	struct mtge_txdesc *desc;
	struct mbuf *m_d;
	bus_dma_segment_t dma_seg[MT_SOFTC_MAX_SCATTER];
	int error, ndmasegs, ndescs, i, tso;
	uint8_t csum_gen;

	KASSERT(qid >= 0 && qid < MT_SOFTC_TX_RING_COUNT,
	    ("%s: Tx data: invalid qid=%d\n",
		device_get_nameunit(sc->dev), qid));

	MTGE_TX_RING_ASSERT_LOCKED(&sc->tx_ring[qid]);

	ifp = sc->ifp;
	ring = &sc->tx_ring[qid];
	desc = &ring->desc[ring->desc_cur];
	data = &ring->data[ring->data_cur];

	/* Before the mapping, because this may move the frame. */
	tso = (m->m_pkthdr.csum_flags & CSUM_TSO) != 0;
	if (tso && (m = mtge_tx_tso_prepare(m)) == NULL)
		return (ENOBUFS);

	/* Under the ring lock; the mbuf variant forces NOWAIT anyway. */
	error = bus_dmamap_load_mbuf_sg(ring->data_dma_tag, data->dma_map, m,
	    dma_seg, &ndmasegs, BUS_DMA_NOWAIT);
	if (error != 0) {
		/* too many fragments, linearize */

		m_d = m_collapse(m, M_NOWAIT, MT_SOFTC_MAX_SCATTER);
		if (m_d == NULL) {
			m_freem(m);
			m = NULL;
			return (ENOMEM);
		}
		m = m_d;

		sc->tx_defrag_packets++;

		error = bus_dmamap_load_mbuf_sg(ring->data_dma_tag,
		    data->dma_map, m, dma_seg, &ndmasegs, BUS_DMA_NOWAIT);
		if (error != 0) {
			device_printf(sc->dev, "could not load mbuf DMA map: "
			    "ndmasegs=%d, len=%d, error=%d\n", ndmasegs,
			    m->m_pkthdr.len, error);
			m_freem(m);
			return (error);
		}
	}

	if (m->m_pkthdr.len == 0)
		ndmasegs = 0;

	csum_gen = 0;
	if (tso) {
		/*
		 * The engine rewrites both headers as it cuts the frame up,
		 * so both checksums are its to compute; the stack flagged
		 * neither of them.
		 */
		csum_gen = TXDSCR_TSO | TXDSCR_IP_CSUM_GEN |
		    TXDSCR_TCP_CSUM_GEN;
	} else if ((if_getcapenable(ifp) & IFCAP_TXCSUM) != 0) {
		if ((m->m_pkthdr.csum_flags & CSUM_IP) != 0)
			csum_gen |= TXDSCR_IP_CSUM_GEN;
		if ((m->m_pkthdr.csum_flags & CSUM_TCP) != 0)
			csum_gen |= TXDSCR_TCP_CSUM_GEN;
		if ((m->m_pkthdr.csum_flags & CSUM_UDP) != 0)
			csum_gen |= TXDSCR_UDP_CSUM_GEN;
	}

	/* determine how many Tx descs are required */
	ndescs = 1 + ndmasegs / 2;
	if ((ring->desc_queued + ndescs) >
	    (sc->tx_desc_size - 2)) {

		sc->no_tx_desc_avail++;

		bus_dmamap_unload(ring->data_dma_tag, data->dma_map);
		m_freem(m);
		return (EFBIG);
	}

	data->m = m;

	/* A tagged frame carries the flow the engine wants bound. */
	if (__predict_false(sc->ppe_enabled))
		mtge_ppe_tx_learn(sc, m);

	/* set up Tx descs */
	for (i = 0; i < ndmasegs; i += 2) {
		/* TODO: this needs to be refined as MT7620 for example has
		 * a different word3 layout than RT305x and RT5350 (the last
		 * one doesn't use word3 at all). And so does MT7621...
		 */

		/* Destination: the GDMA of the interface the frame left on. */
		desc->dst = (((m->m_pkthdr.rcvif == sc->ifp1) ?
		    TXDSCR_DST_PORT_GDMA2 : TXDSCR_DST_PORT_GDMA1) << 1);
		desc->dst |= csum_gen;
		/* Set queue id */
		desc->qn = qid;
		/* No PPPoE */
		desc->pppoe = 0;
		/* No VLAN */
		desc->vid = 0;

		desc->sdp0 = htole32(dma_seg[i].ds_addr);
		desc->sdl0 = htole16(dma_seg[i].ds_len |
		    ( ((i+1) == ndmasegs )?MT_TXDESC_SDL0_LASTSEG:0 ));

		if ((i+1) < ndmasegs) {
			desc->sdp1 = htole32(dma_seg[i+1].ds_addr);
			desc->sdl1 = htole16(dma_seg[i+1].ds_len |
			    ( ((i+2) == ndmasegs )?MT_TXDESC_SDL1_LASTSEG:0 ));
		} else {
			desc->sdp1 = 0;
			desc->sdl1 = 0;
		}

		if ((i+2) < ndmasegs) {
			ring->desc_queued++;
			ring->desc_cur = (ring->desc_cur + 1) %
			    sc->tx_desc_size;
		}
		desc = &ring->desc[ring->desc_cur];
	}

	bus_dmamap_sync(ring->data_dma_tag, data->dma_map,
	    BUS_DMASYNC_PREWRITE);

	ring->desc_queued++;
	ring->desc_cur = (ring->desc_cur + 1) % sc->tx_desc_size;

	ring->data_queued++;
	ring->data_cur = (ring->data_cur + 1) % sc->tx_ring_size;

	/*
	 * No doorbell here: the caller rings once per burst.  The engine only
	 * fetches descriptors it has been told about, so descriptors written
	 * ahead of the doorbell are invisible until then.
	 */

	return (0);
}

/*
 * Pick the Tx ring for a frame.
 *
 * The four rings are drained independently by the frame engine, so a flow has
 * to stay on one of them or its packets get reordered.  Use whatever hash the
 * stack already attached, and compute one from the headers when it did not --
 * that is the common case for locally generated traffic.
 */
static int
mtge_select_queue(struct mtge_softc *sc, struct mbuf *m)
{
	uint32_t hash;

	if (M_HASHTYPE_GET(m) != M_HASHTYPE_NONE)
		hash = m->m_pkthdr.flowid;
	else
		hash = m_ether_tcpip_hash(MBUF_HASHFLAG_L3 | MBUF_HASHFLAG_L4,
		    m, sc->hash_key);

	return (hash % MT_SOFTC_TX_RING_COUNT);
}

/*
 * Move frames from one ring's software queue into its descriptors.  Single
 * consumer, under that ring's lock; nothing here touches another ring.
 */
static void
mtge_tx_start_locked(struct mtge_softc *sc, struct mtge_softc_tx_ring *ring)
{
	if_t ifp;
	struct mbuf *m;
	int enqueued;

	MTGE_TX_RING_ASSERT_LOCKED(ring);

	ifp = sc->ifp;
	enqueued = 0;

	if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) == 0)
		return;

	while ((m = buf_ring_peek_clear_sc(ring->br)) != NULL) {
		if (ring->data_queued >= sc->tx_ring_size) {
			/*
			 * Hardware ring is full.  Leave the frame at the head
			 * of the software queue rather than dropping it;
			 * mtge_tx_done_task() comes back here once descriptors
			 * have been reclaimed.
			 */
			buf_ring_putback_sc(ring->br, m);
			sc->tx_data_queue_full[ring->qid]++;
			break;
		}

		buf_ring_advance_sc(ring->br);

		/*
		 * Tap the frame BEFORE handing it to the hardware.  The Tx ring
		 * takes ownership of the mbuf below and the Tx-completion path
		 * may m_freem() it before we return here -- tapping afterwards
		 * is a use-after-free.
		 */
		ETHER_BPF_MTAP((m->m_pkthdr.rcvif != NULL) ?
		    m->m_pkthdr.rcvif : ifp, m);

		if (mtge_tx_data(sc, m, ring->qid) != 0) {
			/* The mbuf has already been freed on that path. */
			if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
			continue;
		}

		enqueued++;
	}

	if (enqueued != 0) {
		/*
		 * One barrier and one doorbell for the whole burst.  The ring
		 * is uncached, so the sync is the ordering barrier before the
		 * engine is told where the new tail is; ringing per packet cost
		 * an uncached MMIO write per frame for nothing.
		 */
		bus_dmamap_sync(ring->desc_dma_tag, ring->desc_dma_map,
		    BUS_DMASYNC_PREWRITE);
		MT_WRITE(sc, sc->tx_ctx_idx[ring->qid], ring->desc_cur);

		/*
		 * Only the ring lock is held here, and tx_watchdog_ch was
		 * initialized with the softc lock, which callout(9) wants
		 * held to reschedule it.  So the watchdog is not touched
		 * from this path: mtge_init_locked() started it and it keeps
		 * itself running; all a transmit does is wind the timer up.
		 * A store of one int, racing only the watchdog's decrement.
		 */
		sc->tx_timer = MT_TX_WATCHDOG_TIMEOUT;
	}
}

/*
 * mtge_tx_task - drain a ring whose lock mtge_transmit() could not get.
 */
static void
mtge_tx_task(void *context, int pending)
{
	struct mtge_softc_tx_ring *ring;

	ring = context;

	MTGE_TX_RING_LOCK(ring);
	mtge_tx_start_locked(ring->sc, ring);
	MTGE_TX_RING_UNLOCK(ring);
}

/*
 * mtge_transmit - hand one frame to the driver.
 *
 * Replaces if_start(9) and its single IFQ under one mutex: each ring has its
 * own queue and its own lock, so senders on different cores only contend when
 * their flows hash to the same ring.
 */
static int
mtge_transmit(if_t ifp, struct mbuf *m)
{
	struct mtge_softc *sc;
	struct mtge_softc_tx_ring *ring;
	int error, qid;

	sc = if_getsoftc(ifp);

	if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) == 0) {
		m_freem(m);
		return (ENETDOWN);
	}

	/*
	 * Remember which interface this frame leaves through.  The Tx rings
	 * are shared between both ports, so this is what carries the GDMA
	 * choice to mtge_tx_data() and the right ifnet to the BPF tap and the
	 * output counter.
	 */
	m->m_pkthdr.rcvif = ifp;

	qid = mtge_select_queue(sc, m);
	ring = &sc->tx_ring[qid];

	error = buf_ring_enqueue(ring->br, m);
	if (error != 0) {
		sc->tx_br_full[qid]++;
		if_inc_counter(ifp, IFCOUNTER_OQDROPS, 1);
		m_freem(m);
		return (error);
	}

	/*
	 * Send it now if this ring is free, and leave it to the Tx task if
	 * someone else holds the ring -- blocking here would stall the
	 * caller's core behind a ring it has no reason to wait for.
	 */
	if (MTGE_TX_RING_TRYLOCK(ring)) {
		mtge_tx_start_locked(sc, ring);
		MTGE_TX_RING_UNLOCK(ring);
	} else {
		sc->tx_deferred[qid]++;
		taskqueue_enqueue(sc->tx_taskqueue, &ring->tx_task);
	}

	return (0);
}

/*
 * mtge_qflush - drop everything still queued in software.
 */
static void
mtge_qflush(if_t ifp)
{
	struct mtge_softc *sc;
	struct mbuf *m;
	int i;

	sc = if_getsoftc(ifp);

	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++) {
		MTGE_TX_RING_LOCK(&sc->tx_ring[i]);
		while ((m = buf_ring_dequeue_sc(sc->tx_ring[i].br)) != NULL)
			m_freem(m);
		MTGE_TX_RING_UNLOCK(&sc->tx_ring[i]);
	}

	if_qflush(ifp);
}
/*
 * mtge_update_promisc - set/clear promiscuous mode. Unused yet, because
 * filtering done by attached Ethernet switch.
 */
static void
mtge_update_promisc(if_t ifp)
{
	struct mtge_softc *sc;

	sc = if_getsoftc(ifp);
	printf("%s: %s promiscuous mode\n",
	    device_get_nameunit(sc->dev),
	    (if_getflags(ifp) & IFF_PROMISC) ? "entering" : "leaving");
}

/*
 * mtge_ioctl - ioctl handler.
 */
static int
mtge_ioctl(if_t ifp, u_long cmd, caddr_t data)
{
	struct mtge_softc *sc;
	struct ifreq *ifr;
	int error, mask;

	sc = if_getsoftc(ifp);
	ifr = (struct ifreq *) data;

	error = 0;

	switch (cmd) {
	case SIOCSIFFLAGS:
		/*
		 * mtge1 is subordinate: the engine follows mtge0, so up/down
		 * here only gates this port's traffic.  It comes up for real
		 * the moment the engine runs (or right now if it already does).
		 */
		if (ifp == sc->ifp1) {
			MTGE_LOCK(sc);
			if ((if_getflags(ifp) & IFF_UP) != 0) {
				if ((if_getdrvflags(sc->ifp) &
				    IFF_DRV_RUNNING) != 0) {
					if_setdrvflagbits(ifp,
					    IFF_DRV_RUNNING, 0);
					if_link_state_change(ifp,
					    mtge_link_state(sc, ifp));
				}
			} else {
				if_setdrvflagbits(ifp, 0, IFF_DRV_RUNNING);
				if_link_state_change(ifp, LINK_STATE_DOWN);
			}
			MTGE_UNLOCK(sc);
			break;
		}
		MTGE_LOCK(sc);
		if (if_getflags(ifp) & IFF_UP) {
			if (if_getdrvflags(ifp) & IFF_DRV_RUNNING) {
				if ((if_getflags(ifp) ^ sc->if_flags) &
				    IFF_PROMISC)
					mtge_update_promisc(ifp);
			} else {
				mtge_init_locked(sc);
			}
		} else {
			if (if_getdrvflags(ifp) & IFF_DRV_RUNNING)
				mtge_stop_locked(sc);
		}
		sc->if_flags = if_getflags(ifp);
		MTGE_UNLOCK(sc);
		break;
	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr, (ifp == sc->ifp1) ?
		    &sc->ifmedia1 : &sc->ifmedia, cmd);
		break;
	case SIOCSIFCAP:
		/*
		 * Keep if_hwassist in step with the Tx offload capability so
		 * that turning it off with ifconfig(8) makes the stack
		 * compute the checksums again instead of leaving frames
		 * without one.
		 */
		mask = ifr->ifr_reqcap ^ if_getcapenable(ifp);
		if ((mask & IFCAP_TXCSUM) != 0 &&
		    (if_getcapabilities(ifp) & IFCAP_TXCSUM) != 0) {
			if_togglecapenable(ifp, IFCAP_TXCSUM);
			if ((if_getcapenable(ifp) & IFCAP_TXCSUM) != 0)
				if_sethwassistbits(ifp,
				    CSUM_IP | CSUM_TCP | CSUM_UDP, 0);
			else
				if_sethwassistbits(ifp, 0,
				    CSUM_IP | CSUM_TCP | CSUM_UDP);
		}

		if ((mask & IFCAP_TSO4) != 0 &&
		    (if_getcapabilities(ifp) & IFCAP_TSO4) != 0) {
			if_togglecapenable(ifp, IFCAP_TSO4);
			if ((if_getcapenable(ifp) & IFCAP_TSO4) != 0)
				if_sethwassistbits(ifp, CSUM_TSO, 0);
			else
				if_sethwassistbits(ifp, 0, CSUM_TSO);
		}

		/*
		 * Segmentation needs the checksums the same pass computes,
		 * so it cannot outlive them.
		 */
		if ((if_getcapenable(ifp) & IFCAP_TXCSUM) == 0 &&
		    (if_getcapenable(ifp) & IFCAP_TSO4) != 0) {
			if_setcapenablebit(ifp, 0, IFCAP_TSO4);
			if_sethwassistbits(ifp, 0, CSUM_TSO);
		}
		if ((mask & IFCAP_VLAN_MTU) != 0 &&
		    (if_getcapabilities(ifp) & IFCAP_VLAN_MTU) != 0)
			if_togglecapenable(ifp, IFCAP_VLAN_MTU);
		if ((mask & IFCAP_LRO) != 0 &&
		    (if_getcapabilities(ifp) & IFCAP_LRO) != 0)
			if_togglecapenable(ifp, IFCAP_LRO);
		if ((mask & IFCAP_RXCSUM) != 0 &&
		    (if_getcapabilities(ifp) & IFCAP_RXCSUM) != 0)
			if_togglecapenable(ifp, IFCAP_RXCSUM);
		VLAN_CAPABILITIES(ifp);
		break;
	default:
		error = ether_ioctl(ifp, cmd, data);
		break;
	}
	return (error);
}

/*
 * mtge_periodic - Handler of PERIODIC interrupt
 */
static void
mtge_periodic(void *arg)
{
	struct mtge_softc *sc;

	sc = arg;
	taskqueue_enqueue(sc->tx_taskqueue, &sc->periodic_task);
}

/*
 * mtge_tx_watchdog - Handler of TX Watchdog
 *
 * Runs with the softc lock held (callout_init_mtx) once a second for as long
 * as the interface is up; mtge_init_locked() starts it and mtge_stop_locked()
 * stops it, both under that lock.
 */
static void
mtge_tx_watchdog(void *arg)
{
	struct mtge_softc *sc;
	if_t ifp;

	sc = arg;
	ifp = sc->ifp;

	MTGE_ASSERT_LOCKED(sc);

	if (sc->tx_timer != 0 && --sc->tx_timer == 0) {
		uint32_t glo;

		glo = MT_READ(sc, sc->pdma_glo_cfg);

		device_printf(sc->dev, "Tx watchdog timeout: resetting\n");

		/*
		 * sc->tx_timer is cleared only by mtge_tx_done_task(), which
		 * runs solely from the Tx-done interrupt, so this fires
		 * whenever a Tx
		 * completion fails to arrive - equally when the engine sent
		 * nothing and when it sent the frames but never signalled.  The
		 * counters cannot tell those apart (TXQ0_interrupts stays 0
		 * either way), so dump the Tx state at the moment of the stall:
		 *
		 *   tx_en reads back 0          the PDMA Tx ring is not usable
		 *                               on this SoC
		 *   tx_en=1 tx_busy=0 and       enabled but idle: the engine
		 *   dtx_idx stuck at 0          never fetched a descriptor
		 *   tx_busy=1, dtx_idx stuck    running but held by a
		 *                               backpressured GDMA egress, i.e.
		 *                               the MAC side (see MCR/MSR)
		 *   max_cnt != what we wrote    the ring size was rejected
		 */
		device_printf(sc->dev, "  GLO_CFG 0x%08x (tx_en=%d tx_busy=%d "
		    "rx_en=%d rx_busy=%d) INT_STATUS 0x%08x ENABLE 0x%08x\n",
		    glo, (glo & FE_TX_DMA_EN) ? 1 : 0,
		    (glo & FE_TX_DMA_BUSY) ? 1 : 0,
		    (glo & FE_RX_DMA_EN) ? 1 : 0,
		    (glo & FE_RX_DMA_BUSY) ? 1 : 0,
		    MT_READ(sc, sc->pdma_int_status),
		    MT_READ(sc, sc->pdma_int_enable));
		device_printf(sc->dev, "  TX0 base 0x%08x max_cnt %u "
		    "ctx_idx %u dtx_idx %u  sw cur %d next %d queued %d\n",
		    MT_READ(sc, sc->tx_base_ptr[0]),
		    MT_READ(sc, sc->tx_max_cnt[0]),
		    MT_READ(sc, sc->tx_ctx_idx[0]),
		    MT_READ(sc, sc->tx_dtx_idx[0]),
		    sc->tx_ring[0].desc_cur, sc->tx_ring[0].desc_next,
		    sc->tx_ring[0].desc_queued);
		device_printf(sc->dev, "  MAC0 MCR 0x%08x MSR 0x%08x (link=%s)"
		    "  GDM0 IG_CTRL 0x%08x\n",
		    MT_READ(sc, MAC_P_MCR(0)), MT_READ(sc, MAC_P_MSR(0)),
		    (MT_READ(sc, MAC_P_MSR(0)) & MSR_LINK) ? "UP" : "DOWN",
		    MT_READ(sc, MT_GDM_IG_CTRL(0)));
#ifdef notyet
		/*
		 * XXX: Commented out, because reset break input.
		 */
		mtge_stop_locked(sc);
		mtge_init_locked(sc);
#endif
		if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
		sc->tx_watchdog_timeouts++;
	}
	callout_reset(&sc->tx_watchdog_ch, hz, mtge_tx_watchdog, sc);
}

/*
 * mtge_intr - main ISR
 */
static void
mtge_intr(void *arg)
{
	struct mtge_softc *sc;
	if_t ifp;
	uint32_t status;

	sc = arg;
	ifp = sc->ifp;

	/* acknowledge interrupts */
	status = MT_READ(sc, sc->pdma_int_status);
	MT_WRITE(sc, sc->pdma_int_status, status);

	if (status == 0xffffffff ||     /* device likely went away */
	    status == 0)            /* not for us */
		return;

	sc->interrupts++;

	/*
	 * INT_STATUS reports every event the engine saw, whether or not that
	 * source is enabled -- with coalescing on, both the delayed bit and the
	 * per-queue done bits are set.  Act only on what mtge_set_intr_masks()
	 * chose, so exactly one of the two reaches the done tasks.
	 */
	status &= sc->intr_enable_mask;

	if (!(if_getdrvflags(ifp) & IFF_DRV_RUNNING))
		return;

	if (status & MT_INT_TX_COHERENT)
		mtge_tx_coherent_intr(sc);
	if (status & MT_INT_RX_COHERENT)
		mtge_rx_coherent_intr(sc);
	if (status & MT_RX_DLY_INT)
		mtge_rx_delay_intr(sc);
	if (status & MT_TX_DLY_INT)
		mtge_tx_delay_intr(sc);
	if (status & MT_INT_RXQ1_DONE)
		mtge_rx_intr(sc, 1);
	if (status & MT_INT_RXQ0_DONE)
		mtge_rx_intr(sc, 0);
	if (status & MT_INT_TXQ3_DONE)
		mtge_tx_intr(sc, 3);
	if (status & MT_INT_TXQ2_DONE)
		mtge_tx_intr(sc, 2);
	if (status & MT_INT_TXQ1_DONE)
		mtge_tx_intr(sc, 1);
	if (status & MT_INT_TXQ0_DONE)
		mtge_tx_intr(sc, 0);
}

static void
mtge_tx_coherent_intr(struct mtge_softc *sc)
{
	uint32_t tmp;
	int i;

	sc->tx_coherent_interrupts++;

	/* restart DMA engine */
	tmp = MT_READ(sc, sc->pdma_glo_cfg);
	tmp &= ~(FE_TX_WB_DDONE | FE_TX_DMA_EN);
	MT_WRITE(sc, sc->pdma_glo_cfg, tmp);

	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++)
		mtge_reset_tx_ring(sc, &sc->tx_ring[i]);

	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++) {
		MT_WRITE(sc, sc->tx_base_ptr[i],
		    sc->tx_ring[i].desc_phys_addr);
		MT_WRITE(sc, sc->tx_max_cnt[i], sc->tx_desc_size);
		if (MT_READ(sc, sc->tx_max_cnt[i]) !=
		    (uint32_t)sc->tx_desc_size)
			device_printf(sc->dev, "TX_MAX_CNT %d did not stick "
			    "(reads back %u); lower hw.mtge.tx_ring_size\n",
			    sc->tx_desc_size,
			    MT_READ(sc, sc->tx_max_cnt[i]));
		MT_WRITE(sc, sc->tx_ctx_idx[i], 0);
	}

	mtge_txrx_enable(sc);
}

/*
 * mtge_rx_coherent_intr
 */
static void
mtge_rx_coherent_intr(struct mtge_softc *sc)
{
	uint32_t tmp;
	int i;

	sc->rx_coherent_interrupts++;

	/* restart DMA engine */
	tmp = MT_READ(sc, sc->pdma_glo_cfg);
	tmp &= ~(FE_RX_DMA_EN);
	MT_WRITE(sc, sc->pdma_glo_cfg, tmp);

	/* init Rx ring */
	for (i = 0; i < sc->rx_ring_count; i++)
		mtge_reset_rx_ring(sc, &sc->rx_ring[i]);

	for (i = 0; i < sc->rx_ring_count; i++) {
		MT_WRITE(sc, sc->rx_base_ptr[i],
		    sc->rx_ring[i].desc_phys_addr);
		MT_WRITE(sc, sc->rx_max_cnt[i], sc->rx_ring_size);
		if (MT_READ(sc, sc->rx_max_cnt[i]) !=
		    (uint32_t)sc->rx_ring_size)
			device_printf(sc->dev, "RX_MAX_CNT %d did not stick "
			    "(reads back %u); lower hw.mtge.rx_ring_size\n",
			    sc->rx_ring_size,
			    MT_READ(sc, sc->rx_max_cnt[i]));
		MT_WRITE(sc, sc->rx_calc_idx[i], sc->rx_ring_size - 1);
	}

	mtge_txrx_enable(sc);
}

/*
 * mtge_rx_intr - a packet received
 */
static void
mtge_rx_intr(struct mtge_softc *sc, int qid)
{
	KASSERT(qid >= 0 && qid < sc->rx_ring_count,
	    ("%s: Rx interrupt: invalid qid=%d\n",
		device_get_nameunit(sc->dev), qid));

	sc->rx_interrupts[qid]++;
	MTGE_LOCK(sc);

	/*
	 * Rx is masked as a whole: with coalescing the enabled source is one
	 * aggregate bit, so there is nothing per-queue left to mask.
	 */
	if (!(sc->intr_disable_mask & sc->int_rx_mask)) {
		mtge_intr_disable(sc, sc->int_rx_mask);
		taskqueue_enqueue(sc->rx_taskqueue, &sc->rx_done_task);
	}

	sc->intr_pending_mask |= (sc->int_rx_done_mask << qid);
	MTGE_UNLOCK(sc);
}

/*
 * The delayed interrupts are what a coalesced direction reports through; they
 * are aggregate and name no queue, so hand them to the same per-queue paths
 * the undelayed bits use.  Rx marks ring 0 because that is the only ring
 * mtge_rx_done_task() drains.
 */
static void
mtge_rx_delay_intr(struct mtge_softc *sc)
{

	sc->rx_delay_interrupts++;
	mtge_rx_intr(sc, 0);
}

static void
mtge_tx_delay_intr(struct mtge_softc *sc)
{
	int i;

	sc->tx_delay_interrupts++;
	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++)
		mtge_tx_intr(sc, i);
}

/*
 * mtge_tx_intr - Transsmition of packet done
 */
static void
mtge_tx_intr(struct mtge_softc *sc, int qid)
{

	KASSERT(qid >= 0 && qid < MT_SOFTC_TX_RING_COUNT,
	    ("%s: Tx interrupt: invalid qid=%d\n",
		device_get_nameunit(sc->dev), qid));

	sc->tx_interrupts[qid]++;
	MTGE_LOCK(sc);

	if (!(sc->intr_disable_mask & sc->int_tx_mask)) {
		mtge_intr_disable(sc, sc->int_tx_mask);
		taskqueue_enqueue(sc->tx_taskqueue, &sc->tx_done_task);
	}

	sc->intr_pending_mask |= (sc->int_tx_done_mask << qid);
	MTGE_UNLOCK(sc);
}

/*
 * mtge_rx_done_task - run RX task
 */
static void
mtge_rx_done_task(void *context, int pending)
{
	struct mtge_softc *sc;
	if_t ifp;
	int again;

	sc = context;
	ifp = sc->ifp;

	if (!(if_getdrvflags(ifp) & IFF_DRV_RUNNING))
		return;

	MTGE_LOCK(sc);
	sc->intr_pending_mask &= ~sc->int_rx_done_mask;
	MTGE_UNLOCK(sc);

	again = mtge_rx_eof(sc, &sc->rx_ring[0], sc->rx_process_limit);

	MTGE_LOCK(sc);

	if ((sc->intr_pending_mask & sc->int_rx_done_mask) || again) {
		taskqueue_enqueue(sc->rx_taskqueue, &sc->rx_done_task);
	} else {
		mtge_intr_enable(sc, sc->int_rx_mask);
	}

	MTGE_UNLOCK(sc);
}

/*
 * mtge_tx_unreclaimed - interrupt bits for the Tx rings the engine has
 * finished with and the driver has not drained yet.
 */
static uint32_t
mtge_tx_unreclaimed(struct mtge_softc *sc)
{
	uint32_t pending;
	int i;

	pending = 0;
	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++) {
		if (sc->tx_ring[i].desc_next !=
		    (int)MT_READ(sc, sc->tx_dtx_idx[i]))
			pending |= sc->int_tx_done_mask << i;
	}

	return (pending);
}

/*
 * mtge_tx_done_task - check for pending TX task in all queues
 */
static void
mtge_tx_done_task(void *context, int pending)
{
	struct mtge_softc *sc;
	if_t ifp;
	uint32_t intr_mask, tx_pending;
	int i;

	sc = context;
	ifp = sc->ifp;

	if (!(if_getdrvflags(ifp) & IFF_DRV_RUNNING))
		return;

	intr_mask = (
	    MT_INT_TXQ3_DONE |
	    MT_INT_TXQ2_DONE |
	    MT_INT_TXQ1_DONE |
	    MT_INT_TXQ0_DONE);

	MTGE_LOCK(sc);
	tx_pending = sc->intr_pending_mask & intr_mask;
	sc->intr_pending_mask &= ~intr_mask;
	MTGE_UNLOCK(sc);

	for (i = MT_SOFTC_TX_RING_COUNT - 1; i >= 0; i--) {
		if (tx_pending & (sc->int_tx_done_mask << i))
			mtge_tx_eof(sc, &sc->tx_ring[i]);
	}

	MTGE_LOCK(sc);

	/* mtge_tx_watchdog() decrements this under the same lock. */
	sc->tx_timer = 0;

	/* Same all-or-nothing unmask as the Rx side, for the same reason. */
	if (sc->intr_pending_mask & intr_mask) {
		taskqueue_enqueue(sc->tx_taskqueue, &sc->tx_done_task);
	} else {
		mtge_intr_enable(sc, sc->int_tx_mask);

		/*
		 * Everything above ran with the Tx sources masked, and
		 * coalescing reports through the delayed bit alone: a
		 * completion the engine finished inside that window is
		 * not waiting for us now that the mask is back, and it
		 * has no work left to raise another interrupt with.
		 * Look at the rings instead of trusting the bits, and
		 * run another pass ourselves if any of them moved.
		 */
		tx_pending = mtge_tx_unreclaimed(sc);
		if (tx_pending != 0) {
			sc->intr_pending_mask |= tx_pending;
			sc->tx_intr_lost++;
			taskqueue_enqueue(sc->tx_taskqueue,
			    &sc->tx_done_task);
		}
	}

	MTGE_UNLOCK(sc);

	/*
	 * Descriptors are free again, so anything a full ring pushed back
	 * into its software queue can go out now.
	 */
	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++) {
		if (buf_ring_empty(sc->tx_ring[i].br))
			continue;
		MTGE_TX_RING_LOCK(&sc->tx_ring[i]);
		mtge_tx_start_locked(sc, &sc->tx_ring[i]);
		MTGE_TX_RING_UNLOCK(&sc->tx_ring[i]);
	}
}

/* Ports a switch can have, as far as the poll below cares. */
#define	MT_SW_MAX_PORTS		16

/*
 * mtge_switch_find - locate the switch behind the MDIO bus and learn which
 * of its ports are wired to the GMACs.
 *
 * The switch driver attaches under the MDIO bus this driver created and
 * carries an "etherswitch" child; that child is what marks it as speaking
 * etherswitch(9), whichever driver it is.  Which of its ports is a GMAC's
 * trunk comes from the device tree: a port node whose "ethernet" phandle
 * points at one of the mac@N nodes here.
 */
static void
mtge_switch_find(struct mtge_softc *sc)
{
#ifdef FDT
	device_t *buses, *devs, sw;
	phandle_t mac, node, port, ports;
	uint32_t reg, xref;
	int g, i, j, nbuses, ndevs;

	sw = NULL;
	if (device_get_children(sc->dev, &buses, &nbuses) != 0)
		return;
	for (i = 0; i < nbuses && sw == NULL; i++) {
		if (device_get_children(buses[i], &devs, &ndevs) != 0)
			continue;
		for (j = 0; j < ndevs; j++) {
			if (device_is_attached(devs[j]) &&
			    device_find_child(devs[j], "etherswitch",
			    DEVICE_UNIT_ANY) != NULL) {
				sw = devs[j];
				break;
			}
		}
		free(devs, M_TEMP);
	}
	free(buses, M_TEMP);
	if (sw == NULL)
		return;

	node = ofw_bus_get_node(sw);
	if (node <= 0)
		return;
	ports = ofw_bus_find_child(node, "ports");
	if (ports == 0)
		ports = ofw_bus_find_child(node, "ethernet-ports");
	if (ports == 0)
		return;

	for (port = OF_child(ports); port != 0; port = OF_peer(port)) {
		if (OF_getencprop(port, "reg", &reg, sizeof(reg)) <= 0 ||
		    OF_getencprop(port, "ethernet", &xref, sizeof(xref)) <= 0)
			continue;
		mac = OF_node_from_xref(xref);
		if (mac == 0)
			continue;
		for (g = 0; g < 2; g++) {
			if (mac == mtge_gmac_node(sc->dev, g))
				sc->sw_cpu_port[g] = reg;
		}
	}

	sc->sw_dev = sw;
	if (bootverbose)
		device_printf(sc->dev, "%s: gmac0 behind port %d, gmac1 "
		    "behind port %d\n", device_get_nameunit(sw),
		    sc->sw_cpu_port[0], sc->sw_cpu_port[1]);
#endif
}

/*
 * mtge_link_state - what an interface should announce: its trunk's own
 * link, and once the switch has answered, whether any jack behind it has
 * one.
 */
static int
mtge_link_state(struct mtge_softc *sc, if_t ifp)
{
	int g;

	g = (ifp == sc->ifp1) ? 1 : 0;
	if ((MT_READ(sc, MAC_P_MSR(g)) & MSR_LINK) == 0)
		return (LINK_STATE_DOWN);
	if (sc->sw_link[g] == 0)
		return (LINK_STATE_DOWN);
	return (LINK_STATE_UP);
}

/*
 * mtge_switch_poll - derive each GMAC's carrier from the switch.
 *
 * A GMAC's segment is the set of user ports sharing a PVID with the port
 * wired to it: on the BPI-R64 the wan jack with port 5 (VLAN 2), lan0-3
 * with port 6 (VLAN 1).  An unsegmented switch puts everything in VLAN 1
 * and both GMACs then see every jack.  The carrier is up if any jack of
 * the segment has link.  Runs without the softc lock -- every query ends
 * in MDIO traffic, and the switch takes its own lock on the way -- and
 * takes it only to publish the answer.
 */
static void
mtge_switch_poll(struct mtge_softc *sc)
{
	etherswitch_port_t p;
	etherswitch_info_t *info;
	if_t ifp;
	int cpu[MT_SW_MAX_PORTS], pvid[MT_SW_MAX_PORTS], up[MT_SW_MAX_PORTS];
	int g, i, link[2], nports;

	if (sc->sw_dev == NULL) {
		mtge_switch_find(sc);
		if (sc->sw_dev == NULL)
			return;
	}
	info = ETHERSWITCH_GETINFO(sc->sw_dev);
	if (info == NULL)
		return;
	nports = MIN(info->es_nports, MT_SW_MAX_PORTS);

	for (i = 0; i < nports; i++) {
		memset(&p, 0, sizeof(p));
		p.es_port = i;
		if (ETHERSWITCH_GETPORT(sc->sw_dev, &p) != 0) {
			pvid[i] = -1;
			cpu[i] = 0;
			up[i] = 0;
			continue;
		}
		pvid[i] = p.es_pvid;
		cpu[i] = (p.es_flags & ETHERSWITCH_PORT_CPU) != 0;
		up[i] = (p.es_ifmr.ifm_status & (IFM_AVALID | IFM_ACTIVE)) ==
		    (IFM_AVALID | IFM_ACTIVE);
	}

	for (g = 0; g < 2; g++) {
		link[g] = -1;
		if (sc->sw_cpu_port[g] < 0 || sc->sw_cpu_port[g] >= nports ||
		    pvid[sc->sw_cpu_port[g]] < 0)
			continue;
		link[g] = 0;
		for (i = 0; i < nports; i++) {
			if (cpu[i] || i == sc->sw_cpu_port[0] ||
			    i == sc->sw_cpu_port[1])
				continue;
			if (pvid[i] == pvid[sc->sw_cpu_port[g]] && up[i])
				link[g] = 1;
		}
	}

	MTGE_LOCK(sc);
	for (g = 0; g < 2; g++) {
		ifp = (g == 0) ? sc->ifp : sc->ifp1;
		if (ifp == NULL || link[g] < 0 || link[g] == sc->sw_link[g])
			continue;
		sc->sw_link[g] = link[g];
		if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) != 0)
			if_link_state_change(ifp, mtge_link_state(sc, ifp));
	}
	MTGE_UNLOCK(sc);
}

static void
mtge_switch_task(void *context, int pending)
{

	mtge_switch_poll(context);
}

/*
 * mtge_periodic_task - run periodic task
 */
static void
mtge_periodic_task(void *context, int pending)
{
	struct mtge_softc *sc;
	if_t ifp;
	uint32_t tx_work;

	sc = context;
	ifp = sc->ifp;

	if (!(if_getdrvflags(ifp) & IFF_DRV_RUNNING))
		return;

	MTGE_LOCK(sc);
	sc->periodic_round++;
	mtge_update_stats(sc);

	/*
	 * Safety net for the Rx interrupt.  If the engine has advanced past
	 * what the driver has drained and no task is on its way, run one --
	 * a missed or misconfigured Rx interrupt then costs 100ms of latency
	 * instead of the interface, which leaves a way in to read
	 * dev.mtge.<unit>.dump and change the coalescing that caused it.
	 */
	if ((sc->intr_disable_mask & sc->int_rx_mask) == 0 &&
	    sc->rx_ring[0].cur != (int)MT_READ(sc, sc->rx_drx_idx[0])) {
		sc->rx_stall_kicks++;
		taskqueue_enqueue(sc->rx_taskqueue, &sc->rx_done_task);
	}

	/*
	 * The same net for Tx, which needs it more.  Coalescing reports
	 * through the delayed bit alone, and that bit stays masked for as
	 * long as mtge_tx_done_task() runs; a delayed event the engine
	 * raises inside that window is not waiting when the mask comes
	 * back, and the per-queue done bits that are still set cannot
	 * deliver it because they are not among the enabled sources.
	 * Nothing else would reclaim the ring then: the engine has no work
	 * left to raise another interrupt with, so the ring fills up and
	 * the watchdog resets an interface whose hardware is idle and well.
	 *
	 * A completion index the driver has not caught up with says exactly
	 * that has happened, whichever way it came about.
	 */
	if ((sc->intr_disable_mask & sc->int_tx_mask) == 0) {
		tx_work = mtge_tx_unreclaimed(sc);
		if (tx_work != 0) {
			sc->intr_pending_mask |= tx_work;
			sc->tx_stall_kicks++;
			taskqueue_enqueue(sc->tx_taskqueue,
			    &sc->tx_done_task);
		}
	}

	if ((sc->periodic_round % 10) == 0) {
		mtge_update_raw_counters(sc);
		mtge_watchdog(sc);
	}

	/*
	 * Re-arm under the lock, and only while still up.  mtge_stop_locked()
	 * stops this callout under the same lock, so an interface on its way
	 * down cannot have the callout put back behind its back -- which is
	 * what let a firing after detach reach a freed task queue.
	 */
	if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) != 0)
		callout_reset(&sc->periodic_ch, hz / 10, mtge_periodic, sc);
	MTGE_UNLOCK(sc);

	/*
	 * Once a second, and on the system task queue: the poll is MDIO
	 * traffic, and the Tx-done task behind this one should not wait on
	 * it.
	 */
	if ((sc->periodic_round % 10) == 0)
		taskqueue_enqueue(taskqueue_thread, &sc->sw_task);
}

/*
 * mtge_rx_eof - check for frames that done by DMA engine and pass it into
 * network subsystem.
 */
static int
mtge_rx_eof(struct mtge_softc *sc, struct mtge_softc_rx_ring *ring, int limit)
{
	if_t ifp, in_ifp;
	struct mtge_rxdesc *desc;
	struct mtge_softc_rx_data *data;
	struct mbuf *m, *mnew;
	bus_dma_segment_t segs[1];
	bus_dmamap_t dma_map;
	uint32_t index, word3;
	int error, nsegs, len, nframes, lro, rxcsum, sport;

	ifp = sc->ifp;
	nframes = 0;
	lro = sc->lro_ok && (if_getcapenable(ifp) & IFCAP_LRO) != 0;
	rxcsum = (if_getcapenable(ifp) & IFCAP_RXCSUM) != 0;

	/*
	 * DRX_IDX is a register read, and an uncached one costs far more
	 * than the handful of descriptor accesses below.  Take a snapshot
	 * and only go back to the hardware once the batch is drained;
	 * anything that lands after that gets its own interrupt.
	 */
	index = MT_READ(sc, sc->rx_drx_idx[0]);

	while (limit != 0) {
		if (ring->cur == index) {
			index = MT_READ(sc, sc->rx_drx_idx[0]);
			if (ring->cur == index)
				break;
		}

		desc = &ring->desc[ring->cur];
		data = &ring->data[ring->cur];

		bus_dmamap_sync(ring->desc_dma_tag, ring->desc_dma_map,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

		/* XXX Sometime device don`t set DDONE bit */
#ifdef DDONE_FIXED
		if (!(desc->sdl0 & htole16(MT_RXDESC_SDL0_DDONE))) {
			break;
		}
#endif

		len = le16toh(desc->sdl0) & 0x3fff;
		/* Read the result word before the descriptor is recycled. */
		word3 = le32toh(desc->word3);
		sc->rx_last_word3 = word3;

		nframes++;

		mnew = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
		if (mnew == NULL) {
			sc->rx_mbuf_alloc_errors++;
			if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
			goto skip;
		}

		mnew->m_len = mnew->m_pkthdr.len = MCLBYTES;

		error = bus_dmamap_load_mbuf_sg(ring->data_dma_tag,
		    ring->spare_dma_map, mnew, segs, &nsegs, BUS_DMA_NOWAIT);
		if (error != 0) {
			device_printf(sc->dev, "%s could not load Rx mbuf "
			    "DMA map: error=%d, nsegs=%d\n", __func__, error,
			    nsegs);

			m_freem(mnew);

			sc->rx_mbuf_dmamap_errors++;
			if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);

			goto skip;
		}

		KASSERT(nsegs == 1, ("%s: too many DMA segments",
					device_get_nameunit(sc->dev)));

		bus_dmamap_sync(ring->data_dma_tag, data->dma_map,
		    BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(ring->data_dma_tag, data->dma_map);

		dma_map = data->dma_map;
		data->dma_map = ring->spare_dma_map;
		ring->spare_dma_map = dma_map;

		bus_dmamap_sync(ring->data_dma_tag, data->dma_map,
		    BUS_DMASYNC_PREREAD);

		m = data->m;

		data->m = mnew;
		/* Add 2 for proper align of RX IP header */
		desc->sdp0 = htole32(segs[0].ds_addr+2);
		desc->sdl0 = htole16(segs[0].ds_len-2);
		desc->word3 = 0;

		/*
		 * One PDMA serves both MACs, so the ring is shared and rxd4's
		 * source-port field is the only place a frame's origin
		 * survives to.  GDM2 traffic belongs to mtge1; with mtge1
		 * absent or down it has no owner and is dropped, never
		 * delivered to mtge0 -- that would be a frame for a different
		 * station.
		 */
		in_ifp = ifp;
		sport = (word3 >> MT_RXD4_SPORT_SHIFT) & MT_RXD4_SPORT_MASK;
		if (sport == MT_RXD4_SPORT_GDM2) {
			if (sc->ifp1 != NULL &&
			    (if_getdrvflags(sc->ifp1) & IFF_DRV_RUNNING) != 0) {
				in_ifp = sc->ifp1;
				sc->rx_gdm2_packets++;
			} else {
				sc->rx_sport_drops++;
				m_freem(m);
				goto skip;
			}
		}

		m->m_pkthdr.rcvif = in_ifp;
		/* Add 2 to fix data align, after sdp0 = addr + 2 */
		m->m_data += 2;
		m->m_pkthdr.len = m->m_len = len;

		if (__predict_false(sc->ppe_enabled))
			mtge_ppe_rx_note(sc, m, word3);

		/*
		 * Claim only the L4 checksum.  It is the one that covers the
		 * payload and so the one worth not doing twice; the IP header
		 * is twenty bytes and the stack can keep it.  A frame without
		 * the bit is left alone and gets verified upstream as usual.
		 */
		if (rxcsum) {
			if ((word3 & MT_RXDESC_W3_L4_VALID) != 0) {
				m->m_pkthdr.csum_flags |= CSUM_DATA_VALID |
				    CSUM_PSEUDO_HDR;
				m->m_pkthdr.csum_data = 0xffff;
				sc->rx_csum_valid++;
			} else
				sc->rx_csum_none++;
		}

		/*
		 * Give the frame a flow id.  netisr(9) uses it to pick the
		 * queue for NETISR_POLICY_FLOW protocols, so without one every
		 * packet lands on the same netisr thread and deferred dispatch
		 * cannot spread forwarded traffic over both cores.  It also
		 * survives into the Tx side and picks the Tx ring there.
		 */
		m->m_pkthdr.flowid = m_ether_tcpip_hash(MBUF_HASHFLAG_L3 |
		    MBUF_HASHFLAG_L4, m, sc->hash_key);
		M_HASHTYPE_SET(m, M_HASHTYPE_OPAQUE_HASH);

		/*
		 * Hand it to the aggregator first.  A frame it accepts is
		 * merged with the rest of its flow and reaches the stack once
		 * per batch instead of once per segment; anything it will not
		 * take -- the wrong protocol, no room, a gap in the sequence --
		 * comes straight back and goes up on its own.
		 */
		if (!lro || in_ifp != ifp || tcp_lro_rx(&sc->lro, m, 0) != 0)
			if_input(in_ifp, m);
	skip:
		desc->sdl0 &= ~htole16(MT_RXDESC_SDL0_DDONE);

		bus_dmamap_sync(ring->desc_dma_tag, ring->desc_dma_map,
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

		ring->cur = (ring->cur + 1) % sc->rx_ring_size;

		limit--;
	}

	if (ring->cur == 0)
		MT_WRITE(sc, sc->rx_calc_idx[0], sc->rx_ring_size - 1);
	else
		MT_WRITE(sc, sc->rx_calc_idx[0],
		    ring->cur - 1);

	sc->rx_pdma_packets += nframes;

	/*
	 * Flush after the ring has been handed back to the engine: what comes
	 * out here runs the whole stack, and the hardware should not be
	 * waiting on descriptors while that happens.
	 */
	if (lro)
		tcp_lro_flush_all(&sc->lro);

	return (limit == 0);
}

/*
 * mtge_tx_eof - check for successful transmitted frames and mark their
 * descriptor as free.
 */
static void
mtge_tx_eof(struct mtge_softc *sc, struct mtge_softc_tx_ring *ring)
{
	if_t ifp;
	struct mtge_txdesc *desc;
	struct mtge_softc_tx_data *data;
	uint32_t index;
	int ndescs;

	ifp = sc->ifp;

	ndescs = 0;

	MTGE_TX_RING_LOCK(ring);

	bus_dmamap_sync(ring->desc_dma_tag, ring->desc_dma_map,
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	/* One register read per batch, not per descriptor. */
	index = MT_READ(sc, sc->tx_dtx_idx[ring->qid]);

	for (;;) {
		if (ring->desc_next == index) {
			index = MT_READ(sc, sc->tx_dtx_idx[ring->qid]);
			if (ring->desc_next == index)
				break;
		}

		ndescs++;

		desc = &ring->desc[ring->desc_next];

		if (desc->sdl0 & htole16(MT_TXDESC_SDL0_LASTSEG) ||
		    desc->sdl1 & htole16(MT_TXDESC_SDL1_LASTSEG)) {
			data = &ring->data[ring->data_next];

			bus_dmamap_sync(ring->data_dma_tag, data->dma_map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(ring->data_dma_tag, data->dma_map);

			if (data->m != NULL) {
				if_inc_counter((data->m->m_pkthdr.rcvif !=
				    NULL) ? data->m->m_pkthdr.rcvif : ifp,
				    IFCOUNTER_OPACKETS, 1);
				m_freem(data->m);
				data->m = NULL;
			}

			ring->data_queued--;
			ring->data_next = (ring->data_next + 1) %
			    sc->tx_ring_size;
		}

		ring->desc_queued--;
		ring->desc_next = (ring->desc_next + 1) %
		    sc->tx_desc_size;

		desc->sdl0 &= ~htole16(MT_TXDESC_SDL0_DDONE);

	}

	if (ndescs)
		bus_dmamap_sync(ring->desc_dma_tag, ring->desc_dma_map,
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	MTGE_TX_RING_UNLOCK(ring);
}

/*
 * mtge_update_stats - query statistics counters and update related variables.
 */
static void
mtge_update_stats(struct mtge_softc *sc)
{

	/* XXX do update stats here */
}

/*
 * mtge_watchdog - reinit device on watchdog event.
 */
static void
mtge_watchdog(struct mtge_softc *sc)
{
}

/*
 * mtge_update_raw_counters - update counters.
 */
static void
mtge_update_raw_counters(struct mtge_softc *sc)
{
	int gmac = 0;
	uint32_t tmp;

	sc->tx_bytes	+= MT_READ(sc, GDM_TX_GBCNT_LSB(gmac));
	tmp = MT_READ(sc,GDM_TX_GBCNT_MSB(gmac));
	if (tmp)
		sc->tx_bytes    += ((uint64_t) tmp << 32);
	sc->tx_packets	+= MT_READ(sc, GDM_TX_GPCNT(gmac));
	sc->tx_skip	+= MT_READ(sc, GDM_TX_SKIPCNT(gmac));
	sc->tx_collision+= MT_READ(sc, GDM_TX_COLCNT(gmac));

	sc->rx_bytes	+= MT_READ(sc, GDM_RX_GBCNT_LSB(gmac));
	tmp = MT_READ(sc,GDM_RX_GBCNT_MSB(gmac));
	if (tmp)
		sc->rx_bytes    += ((uint64_t)tmp << 32);
	sc->rx_packets	+= MT_READ(sc, GDM_RX_GPCNT(gmac));
	sc->rx_crc_err	+= MT_READ(sc, GDM_RX_CSUM_ERCNT(gmac));
	sc->rx_short_err+= MT_READ(sc, GDM_RX_SHORT_ERCNT(gmac));
	sc->rx_long_err	+= MT_READ(sc, GDM_RX_LONG_ERCNT(gmac));
	sc->rx_phy_err	+= MT_READ(sc, GDM_RX_FERCNT(gmac));
	sc->rx_fifo_overflows+= MT_READ(sc, GDM_RX_OERCNT(gmac));

	/*
	 * gmac1's own counters.  They are the only witness a hardware-bound
	 * flow leaves behind: its frames cross GDM1 to GDM2 inside the chip
	 * and no software counter on either interface ever sees them.
	 */
	sc->rx_packets1	+= MT_READ(sc, GDM_RX_GPCNT(1));
	sc->tx_packets1	+= MT_READ(sc, GDM_TX_GPCNT(1));
}

static void
mtge_intr_enable(struct mtge_softc *sc, uint32_t intr_mask)
{
	uint32_t tmp;

	/*
	 * intr_disable_mask and the hardware enable register are updated
	 * together; the Rx and Tx tasks now run on separate threads and both
	 * get here, so the softc lock is what keeps the pair consistent.
	 */
	MTGE_ASSERT_LOCKED(sc);

	sc->intr_disable_mask &= ~intr_mask;
	tmp = sc->intr_enable_mask & ~sc->intr_disable_mask;
	MT_WRITE(sc, sc->pdma_int_enable, tmp);
}

static void
mtge_intr_disable(struct mtge_softc *sc, uint32_t intr_mask)
{
	uint32_t tmp;

	MTGE_ASSERT_LOCKED(sc);

	sc->intr_disable_mask |= intr_mask;
	tmp = sc->intr_enable_mask & ~sc->intr_disable_mask;
	MT_WRITE(sc, sc->pdma_int_enable, tmp);
}

/*
 * mtge_txrx_enable - enable TX/RX DMA
 */
static int
mtge_txrx_enable(struct mtge_softc *sc)
{
	uint32_t tmp;
	int ntries;

	/* enable Tx/Rx DMA engine */
	for (ntries = 0; ntries < 200; ntries++) {
		tmp = MT_READ(sc, sc->pdma_glo_cfg);
		if (!(tmp & (FE_TX_DMA_BUSY | FE_RX_DMA_BUSY)))
			break;

		DELAY(1000);
	}

	if (ntries == 200) {
		device_printf(sc->dev, "timeout waiting for DMA engine\n");
		return (-1);
	}

	DELAY(50);

	tmp |= FE_TX_WB_DDONE |	FE_RX_DMA_EN | FE_TX_DMA_EN;
	MT_WRITE(sc, sc->pdma_glo_cfg, tmp);

	/* XXX set Rx filter */
	return (0);
}

static int
mtge_alloc_rx_ring(struct mtge_softc *sc, struct mtge_softc_rx_ring *ring,
    int qid)
{
	struct mtge_rxdesc *desc;
	struct mtge_softc_rx_data *data;
	bus_dma_segment_t segs[1];
	int i, nsegs, error;

	error = bus_dma_tag_create(sc->parent_tag,
	    sizeof(struct mtge_rxdesc), 0,
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL,
	    sc->rx_ring_size * sizeof(struct mtge_rxdesc), 1,
	    sc->rx_ring_size * sizeof(struct mtge_rxdesc),
	    0, NULL, NULL, &ring->desc_dma_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create Rx desc DMA tag\n");
		goto fail;
	}

	/*
	 * BUS_DMA_COHERENT here asks bus_dmamem_alloc(9) for an
	 * uncached (VM_MEMATTR_UNCACHEABLE) mapping of the ring.  The
	 * frame engine and the CPU then see the same bytes with no
	 * cache maintenance at all, so the bus_dmamap_sync(9) calls in
	 * the RX path degenerate to a barrier instead of walking every
	 * cache line of the whole ring once per received frame.
	 *
	 * Note this is the *allocation* flag, not the tag flag.  Passing
	 * BUS_DMA_COHERENT to bus_dma_tag_create(9) would only mark the
	 * maps coherent and skip the cache maintenance while the memory
	 * stayed write-back cached -- the CCI-400 that would make that
	 * true is not programmed by FreeBSD, so the engine would read
	 * stale descriptors.
	 */
	error = bus_dmamem_alloc(ring->desc_dma_tag, (void **) &ring->desc,
	    BUS_DMA_NOWAIT | BUS_DMA_ZERO | BUS_DMA_COHERENT,
	    &ring->desc_dma_map);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not allocate Rx desc DMA memory\n");
		goto fail;
	}

	error = bus_dmamap_load(ring->desc_dma_tag, ring->desc_dma_map,
	    ring->desc,
	    sc->rx_ring_size * sizeof(struct mtge_rxdesc),
	    mtge_dma_map_addr, &ring->desc_phys_addr, 0);
	if (error != 0) {
		device_printf(sc->dev, "could not load Rx desc DMA map\n");
		goto fail;
	}

	error = bus_dma_tag_create(sc->parent_tag, 1, 0,
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL,
	    MCLBYTES, MT_SOFTC_MAX_SCATTER, MCLBYTES, 0, NULL, NULL,
	    &ring->data_dma_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create Rx data DMA tag\n");
		goto fail;
	}

	for (i = 0; i < sc->rx_ring_size; i++) {
		desc = &ring->desc[i];
		data = &ring->data[i];

		error = bus_dmamap_create(ring->data_dma_tag, 0,
		    &data->dma_map);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not create Rx data DMA map\n");
			goto fail;
		}

		data->m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
		if (data->m == NULL) {
			device_printf(sc->dev, "could not allocate Rx mbuf\n");
			error = ENOMEM;
			goto fail;
		}

		data->m->m_len = data->m->m_pkthdr.len = MCLBYTES;

		error = bus_dmamap_load_mbuf_sg(ring->data_dma_tag,
		    data->dma_map, data->m, segs, &nsegs, BUS_DMA_WAITOK);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not load Rx mbuf DMA map\n");
			goto fail;
		}

		KASSERT(nsegs == 1, ("%s: too many DMA segments",
					device_get_nameunit(sc->dev)));

		/* Add 2 for proper align of RX IP header */
		desc->sdp0 = htole32(segs[0].ds_addr+2);
		desc->sdl0 = htole16(segs[0].ds_len-2);
	}

	error = bus_dmamap_create(ring->data_dma_tag, 0,
	    &ring->spare_dma_map);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create Rx spare DMA map\n");
		goto fail;
	}

	bus_dmamap_sync(ring->desc_dma_tag, ring->desc_dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	ring->qid = qid;
	return (0);

fail:
	mtge_free_rx_ring(sc, ring);
	return (error);
}

static void
mtge_reset_rx_ring(struct mtge_softc *sc, struct mtge_softc_rx_ring *ring)
{
	struct mtge_rxdesc *desc;
	int i;

	for (i = 0; i < sc->rx_ring_size; i++) {
		desc = &ring->desc[i];
		desc->sdl0 &= ~htole16(MT_RXDESC_SDL0_DDONE);
	}

	bus_dmamap_sync(ring->desc_dma_tag, ring->desc_dma_map,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	ring->cur = 0;
}

static void
mtge_free_rx_ring(struct mtge_softc *sc, struct mtge_softc_rx_ring *ring)
{
	struct mtge_softc_rx_data *data;
	int i;

	if (ring->desc != NULL) {
		bus_dmamap_sync(ring->desc_dma_tag, ring->desc_dma_map,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(ring->desc_dma_tag, ring->desc_dma_map);
		bus_dmamem_free(ring->desc_dma_tag, ring->desc,
		    ring->desc_dma_map);
		ring->desc = NULL;
	}

	if (ring->desc_dma_tag != NULL) {
		bus_dma_tag_destroy(ring->desc_dma_tag);
		ring->desc_dma_tag = NULL;
	}

	for (i = 0; i < sc->rx_ring_size; i++) {
		data = &ring->data[i];

		if (data->m != NULL) {
			bus_dmamap_sync(ring->data_dma_tag, data->dma_map,
			    BUS_DMASYNC_POSTREAD);
			bus_dmamap_unload(ring->data_dma_tag, data->dma_map);
			m_freem(data->m);
		}

		if (data->dma_map != NULL) {
			bus_dmamap_destroy(ring->data_dma_tag, data->dma_map);
			data->dma_map = NULL;
		}
		data->m = NULL;
	}

	if (ring->spare_dma_map != NULL) {
		bus_dmamap_destroy(ring->data_dma_tag, ring->spare_dma_map);
		ring->spare_dma_map = NULL;
	}

	if (ring->data_dma_tag != NULL) {
		bus_dma_tag_destroy(ring->data_dma_tag);
		ring->data_dma_tag = NULL;
	}
}

static int
mtge_alloc_tx_ring(struct mtge_softc *sc, struct mtge_softc_tx_ring *ring,
    int qid)
{
	struct mtge_softc_tx_data *data;
	int error, i;

	error = bus_dma_tag_create(sc->parent_tag,
	    sizeof(struct mtge_txdesc), 0,
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL,
	    sc->tx_desc_size * sizeof(struct mtge_txdesc), 1,
	    sc->tx_desc_size * sizeof(struct mtge_txdesc),
	    0, NULL, NULL, &ring->desc_dma_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create Tx desc DMA tag\n");
		goto fail;
	}

	/* Uncached, for the reasons spelled out in mtge_alloc_rx_ring(). */
	error = bus_dmamem_alloc(ring->desc_dma_tag, (void **) &ring->desc,
	    BUS_DMA_NOWAIT | BUS_DMA_ZERO | BUS_DMA_COHERENT,
	    &ring->desc_dma_map);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not allocate Tx desc DMA memory\n");
		goto fail;
	}

	error = bus_dmamap_load(ring->desc_dma_tag, ring->desc_dma_map,
	    ring->desc,	(sc->tx_desc_size *
			    sizeof(struct mtge_txdesc)), mtge_dma_map_addr,
	    &ring->desc_phys_addr, 0);
	if (error != 0) {
		device_printf(sc->dev, "could not load Tx desc DMA map\n");
		goto fail;
	}

	ring->desc_queued = 0;
	ring->desc_cur = 0;
	ring->desc_next = 0;

	/*
	 * Sized for the largest frame the stack can hand down, which is a
	 * segmentation-offload chain rather than a packet: a whole window in
	 * one mapping.  The per-segment limit is a page to match what
	 * if_sethwtsomaxsegsize() promises tcp_output(9), so the segment
	 * count busdma returns is the one the stack built for.
	 */
	error = bus_dma_tag_create(sc->parent_tag, 1, 0,
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL,
	    MT_SOFTC_TSO_MAX, MT_SOFTC_MAX_SCATTER,
	    PAGE_SIZE, 0, NULL, NULL,
	    &ring->data_dma_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create Tx data DMA tag\n");
		goto fail;
	}

	for (i = 0; i < sc->tx_ring_size; i++) {
		data = &ring->data[i];

		error = bus_dmamap_create(ring->data_dma_tag, 0,
		    &data->dma_map);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not create Tx data DMA map\n");
			goto fail;
		}
	}

	ring->data_queued = 0;
	ring->data_cur = 0;
	ring->data_next = 0;

	ring->qid = qid;
	return (0);

fail:
	mtge_free_tx_ring(sc, ring);
	return (error);
}

static void
mtge_reset_tx_ring(struct mtge_softc *sc, struct mtge_softc_tx_ring *ring)
{
	struct mtge_softc_tx_data *data;
	struct mtge_txdesc *desc;
	int i;

	/*
	 * A transmit that read IFF_DRV_RUNNING just before it was cleared
	 * may still be filling this ring; the ring lock is what it holds.
	 */
	MTGE_TX_RING_LOCK(ring);

	for (i = 0; i < sc->tx_desc_size; i++) {
		desc = &ring->desc[i];

		desc->sdl0 = 0;
		desc->sdl1 = 0;
	}

	ring->desc_queued = 0;
	ring->desc_cur = 0;
	ring->desc_next = 0;

	bus_dmamap_sync(ring->desc_dma_tag, ring->desc_dma_map,
	    BUS_DMASYNC_PREWRITE);

	for (i = 0; i < sc->tx_ring_size; i++) {
		data = &ring->data[i];

		if (data->m != NULL) {
			bus_dmamap_sync(ring->data_dma_tag, data->dma_map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(ring->data_dma_tag, data->dma_map);
			m_freem(data->m);
			data->m = NULL;
		}
	}

	ring->data_queued = 0;
	ring->data_cur = 0;
	ring->data_next = 0;

	MTGE_TX_RING_UNLOCK(ring);
}

/*
 * mtge_free_tx_ring - free RX ring buffer
 */
static void
mtge_free_tx_ring(struct mtge_softc *sc, struct mtge_softc_tx_ring *ring)
{
	struct mtge_softc_tx_data *data;
	int i;

	if (ring->desc != NULL) {
		bus_dmamap_sync(ring->desc_dma_tag, ring->desc_dma_map,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(ring->desc_dma_tag, ring->desc_dma_map);
		bus_dmamem_free(ring->desc_dma_tag, ring->desc,
		    ring->desc_dma_map);
		ring->desc = NULL;
	}

	if (ring->desc_dma_tag != NULL) {
		bus_dma_tag_destroy(ring->desc_dma_tag);
		ring->desc_dma_tag = NULL;
	}

	for (i = 0; i < sc->tx_ring_size; i++) {
		data = &ring->data[i];

		if (data->m != NULL) {
			bus_dmamap_sync(ring->data_dma_tag, data->dma_map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(ring->data_dma_tag, data->dma_map);
			m_freem(data->m);
		}

		if (data->dma_map != NULL) {
			bus_dmamap_destroy(ring->data_dma_tag, data->dma_map);
			data->dma_map = NULL;
		}
		data->m = NULL;
	}

	if (ring->data_dma_tag != NULL) {
		bus_dma_tag_destroy(ring->data_dma_tag);
		ring->data_dma_tag = NULL;
	}
}

/*
 * mtge_dma_map_addr - get address of busdma segment
 */
static void
mtge_dma_map_addr(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	if (error != 0)
		return;

	KASSERT(nseg == 1, ("too many DMA segments, %d should be 1", nseg));

	*(bus_addr_t *) arg = segs[0].ds_addr;
}

/*
 * A packet can need MT_SOFTC_TX_DESC_PER_PKT descriptors and the descriptor
 * count itself goes into TX_MAX_CNT, so round up to the next power of two
 * that covers the worst case.  The CTASSERTs above keep that inside
 * MT_SOFTC_TX_RING_DESC_MAX.
 */
static void
mtge_set_tx_desc_size(struct mtge_softc *sc)
{

	for (sc->tx_desc_size = MT_RING_DATA_MIN;
	    sc->tx_desc_size < sc->tx_ring_size * MT_SOFTC_TX_DESC_PER_PKT;
	    sc->tx_desc_size <<= 1)
		;
}

/*
 * Rebuild the rings at a new size.  The rings are allocated once at attach,
 * so changing a size means tearing them down and building them again; the
 * interface goes down for the duration and comes back if it was up.
 *
 * The softc lock cannot be held across this - draining the taskqueue sleeps
 * - so sc->resizing marks the window instead, and mtge_init_locked() returns
 * early while it is set.
 */
static int
mtge_resize_rings(struct mtge_softc *sc, int rx_size, int tx_size)
{
	int error, i, running;

	MTGE_LOCK(sc);
	if (sc->resizing) {
		MTGE_UNLOCK(sc);
		return (EBUSY);
	}
	sc->resizing = 1;
	running = (if_getdrvflags(sc->ifp) & IFF_DRV_RUNNING) != 0;
	if (running)
		mtge_stop_locked(sc);
	MTGE_UNLOCK(sc);

	/*
	 * mtge_stop_locked() only blocks the task queues.  Drain them here, in
	 * process context, so nothing is still walking the rings we free --
	 * after letting them run again, since a blocked queue leaves a task
	 * that was already pending exactly where it is and the drain would
	 * wait on it forever.  What runs now sees IFF_DRV_RUNNING off.
	 */
	taskqueue_unblock(sc->rx_taskqueue);
	taskqueue_unblock(sc->tx_taskqueue);
	taskqueue_drain(sc->rx_taskqueue, &sc->rx_done_task);
	taskqueue_drain(sc->tx_taskqueue, &sc->tx_done_task);
	taskqueue_drain(sc->tx_taskqueue, &sc->periodic_task);
	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++)
		taskqueue_drain(sc->tx_taskqueue, &sc->tx_ring[i].tx_task);

	/* Free at the old sizes, which is what the free paths walk. */
	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++) {
		MTGE_TX_RING_LOCK(&sc->tx_ring[i]);
		mtge_free_tx_ring(sc, &sc->tx_ring[i]);
		MTGE_TX_RING_UNLOCK(&sc->tx_ring[i]);
	}
	for (i = 0; i < sc->rx_ring_count; i++)
		mtge_free_rx_ring(sc, &sc->rx_ring[i]);

	sc->rx_ring_size = rx_size;
	sc->tx_ring_size = tx_size;
	mtge_set_tx_desc_size(sc);

	error = 0;
	for (i = 0; i < MT_SOFTC_TX_RING_COUNT && error == 0; i++)
		error = mtge_alloc_tx_ring(sc, &sc->tx_ring[i], i);
	for (i = 0; i < sc->rx_ring_count && error == 0; i++)
		error = mtge_alloc_rx_ring(sc, &sc->rx_ring[i], i);

	MTGE_LOCK(sc);
	sc->resizing = 0;
	if (error == 0) {
		if (running)
			mtge_init_locked(sc);
	} else
		device_printf(sc->dev, "could not build rings for rx %d / "
		    "tx %d; the interface stays down\n", rx_size, tx_size);
	MTGE_UNLOCK(sc);

	return (error);
}

/*
 * hw.mtge.rx_ring_size and hw.mtge.tx_ring_size.  These carry the default a
 * new instance attaches with, and a write also applies to every instance
 * that is already attached, so the knob behaves the same whether it is set
 * from loader.conf, from /etc/sysctl.conf or by hand at runtime.
 *
 * The device list is snapshotted under the bus topology lock and the resizes
 * run after it is dropped, because mtge_resize_rings() sleeps.  A device
 * detaching in that window would be missed; this is a root-only tuning knob,
 * and holding the topology lock across a sleeping resize is the worse trade.
 */
static int
mtge_sysctl_default_ring_size(SYSCTL_HANDLER_ARGS)
{
	struct mtge_softc *sc;
	devclass_t dc;
	device_t *devs;
	int error, i, max, ndevs, val;

	max = (arg2 != 0) ? MT_SOFTC_TX_RING_DATA_MAX :
	    MT_SOFTC_RX_RING_DATA_MAX;
	val = *(int *)arg1;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	if (!mtge_ring_size_ok(val, max))
		return (EINVAL);

	*(int *)arg1 = val;

	/*
	 * Registering the OID loads the loader tunable through this handler,
	 * long before any device exists.  Nothing to apply then.
	 */
	bus_topo_lock();
	dc = devclass_find("mtge");
	if (dc == NULL || devclass_get_devices(dc, &devs, &ndevs) != 0) {
		bus_topo_unlock();
		return (0);
	}
	bus_topo_unlock();

	for (i = 0; i < ndevs; i++) {
		if (device_get_state(devs[i]) != DS_ATTACHED)
			continue;
		sc = device_get_softc(devs[i]);
		if (arg2 != 0)
			error = mtge_resize_rings(sc, sc->rx_ring_size, val);
		else
			error = mtge_resize_rings(sc, val, sc->tx_ring_size);
		if (error != 0)
			break;
	}
	free(devs, M_TEMP);

	return (error);
}

/*
 * dev.mtge.<unit>.rx_ring_size and .tx_ring_size.  Writable, so they can be
 * set from /etc/sysctl.conf as well as from the hw.mtge.* loader tunables;
 * a write that changes the size rebuilds the rings.
 */
static int
mtge_sysctl_ring_size(SYSCTL_HANDLER_ARGS)
{
	struct mtge_softc *sc = arg1;
	int error, max, val;

	if (arg2 != 0) {
		val = sc->tx_ring_size;
		max = MT_SOFTC_TX_RING_DATA_MAX;
	} else {
		val = sc->rx_ring_size;
		max = MT_SOFTC_RX_RING_DATA_MAX;
	}

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	if (val < MT_RING_DATA_MIN || val > max || (val & (val - 1)) != 0)
		return (EINVAL);

	if (arg2 != 0) {
		if (val == sc->tx_ring_size)
			return (0);
		return (mtge_resize_rings(sc, sc->rx_ring_size, val));
	}

	if (val == sc->rx_ring_size)
		return (0);

	return (mtge_resize_rings(sc, val, sc->tx_ring_size));
}

/*
 * mtge_sysctl_dump - one-shot dump of the MAC / PDMA state.
 *
 * Reading dev.mtge.<unit>.dump walks the registers a frame meets on its way
 * in, in that order, so one command says which stage drops it:
 *
 *   MCR/MSR   the forced trunk to the switch.  MCR is only what we asked
 *             for; MSR is what the link really does.  mtge_ifmedia_sts()
 *             reports this interface as "active" unconditionally, so
 *             MSR_LINK is the only honest answer to "is the MAC<->MT7531
 *             line up?".
 *   GDM       the frame engine: its forward configuration and packet
 *             counters.  rx_packets rising means frames do arrive from the
 *             switch; tx_packets rising means ours do leave.
 *   PDMA      the descriptor rings.  A DRX index that never moves while the
 *             GDM Rx counter rises puts the loss between the frame engine
 *             and the rings; rx_pdma_packets not following the DRX index
 *             puts it in mtge_rx_eof().
 *
 * Registers are read without the softc lock: this is a read-only debug path,
 * and sbuf(9) drains into user memory, which may sleep.
 */
static int
mtge_sysctl_dump(SYSCTL_HANDLER_ARGS)
{
	struct mtge_softc *sc = arg1;
	struct sbuf *sb;
	uint32_t mcr, msr, glo;
	int error, i;

	sb = sbuf_new_for_sysctl(NULL, NULL, 2048, req);
	if (sb == NULL)
		return (ENOMEM);

	mcr = MT_READ(sc, MAC_P_MCR(0));
	msr = MT_READ(sc, MAC_P_MSR(0));
	glo = MT_READ(sc, sc->pdma_glo_cfg);

	sbuf_printf(sb, "\n");
	sbuf_printf(sb, "MAC0 MCR      0x%08x (tx_en=%d rx_en=%d force=%d)\n",
	    mcr, (mcr & MAC_TX_EN) ? 1 : 0, (mcr & MAC_RX_EN) ? 1 : 0,
	    (mcr & FORCE_MODE) ? 1 : 0);
	sbuf_printf(sb, "MAC0 MSR      0x%08x (link=%s duplex=%s speed=%s)\n",
	    msr, (msr & MSR_LINK) ? "UP" : "DOWN",
	    (msr & MSR_DPX) ? "full" : "half",
	    ((msr & MSR_SPD_MASK) >> MSR_SPD_SHIFT) == MSR_SPD_1000M ? "1000" :
	    ((msr & MSR_SPD_MASK) >> MSR_SPD_SHIFT) == MSR_SPD_100M ? "100" :
	    "10");
	sbuf_printf(sb, "GDM0 IG_CTRL  0x%08x  MAC %04x%08x\n",
	    MT_READ(sc, MT_GDM_IG_CTRL(0)), MT_READ(sc, MT_GDM_MAC_MSB(0)),
	    MT_READ(sc, MT_GDM_MAC_LSB(0)));

	sbuf_printf(sb, "PDMA GLO_CFG  0x%08x (tx_en=%d rx_en=%d "
	    "tx_busy=%d rx_busy=%d)\n", glo,
	    (glo & FE_TX_DMA_EN) ? 1 : 0, (glo & FE_RX_DMA_EN) ? 1 : 0,
	    (glo & FE_TX_DMA_BUSY) ? 1 : 0, (glo & FE_RX_DMA_BUSY) ? 1 : 0);
	sbuf_printf(sb, "PDMA INT      status 0x%08x enable 0x%08x "
	    "delay 0x%08x\n",
	    MT_READ(sc, sc->pdma_int_status), MT_READ(sc, sc->pdma_int_enable),
	    MT_READ(sc, sc->pdma_delay_int_cfg));

	sbuf_printf(sb, "ring  rx %d  tx %d packets / %d descriptors\n",
	    sc->rx_ring_size, sc->tx_ring_size, sc->tx_desc_size);
	sbuf_printf(sb, "coal  rx %d pkts / %d ticks   tx %d pkts / %d ticks\n",
	    sc->rx_coal_pkts, sc->rx_coal_ticks, sc->tx_coal_pkts,
	    sc->tx_coal_ticks);
	sbuf_printf(sb, "intr  enable 0x%08x disable 0x%08x pending 0x%08x  "
	    "rx src %s  tx src %s\n",
	    sc->intr_enable_mask, sc->intr_disable_mask, sc->intr_pending_mask,
	    (sc->int_rx_mask == MT_RX_DLY_INT) ? "delayed" : "per-queue",
	    (sc->int_tx_mask == MT_TX_DLY_INT) ? "delayed" : "per-queue");

	for (i = 0; i < sc->rx_ring_count; i++)
		sbuf_printf(sb, "RX%d  base 0x%08x cnt %u calc_idx %u "
		    "drx_idx %u sw_cur %d\n", i,
		    MT_READ(sc, sc->rx_base_ptr[i]),
		    MT_READ(sc, sc->rx_max_cnt[i]),
		    MT_READ(sc, sc->rx_calc_idx[i]),
		    MT_READ(sc, sc->rx_drx_idx[i]),
		    sc->rx_ring[i].cur);

	for (i = 0; i < MT_SOFTC_TX_RING_COUNT; i++)
		sbuf_printf(sb, "TX%d  base 0x%08x cnt %u ctx_idx %u "
		    "dtx_idx %u sw_cur %d sw_next %d queued %d/%d  "
		    "br %d intr %lu defer %lu full %lu drop %lu\n", i,
		    MT_READ(sc, sc->tx_base_ptr[i]),
		    MT_READ(sc, sc->tx_max_cnt[i]),
		    MT_READ(sc, sc->tx_ctx_idx[i]),
		    MT_READ(sc, sc->tx_dtx_idx[i]),
		    sc->tx_ring[i].desc_cur, sc->tx_ring[i].desc_next,
		    sc->tx_ring[i].desc_queued, sc->tx_ring[i].data_queued,
		    buf_ring_count(sc->tx_ring[i].br), sc->tx_interrupts[i],
		    sc->tx_deferred[i], sc->tx_data_queue_full[i],
		    sc->tx_br_full[i]);

	sbuf_printf(sb, "intr  total %lu rx0 %lu rx1 %lu "
	    "coherent rx %lu tx %lu\n",
	    sc->interrupts, sc->rx_interrupts[0], sc->rx_interrupts[1],
	    sc->rx_coherent_interrupts, sc->tx_coherent_interrupts);
	sbuf_printf(sb, "gdm   rx_packets %lu tx_packets %lu\n",
	    sc->rx_packets, sc->tx_packets);
	sbuf_printf(sb, "pdma  rx_packets %lu (frames given to the stack)\n",
	    sc->rx_pdma_packets);
	sbuf_printf(sb, "gdm2  rx_packets %lu sport_drops %lu (mtge1 %s)\n",
	    sc->rx_gdm2_packets, sc->rx_sport_drops,
	    (sc->ifp1 == NULL) ? "absent" :
	    ((if_getdrvflags(sc->ifp1) & IFF_DRV_RUNNING) ? "up" : "down"));
	sbuf_printf(sb, "csum  rx valid %lu none %lu  last rxd4 0x%08x "
	    "(L4_VALID %s)\n", sc->rx_csum_valid, sc->rx_csum_none,
	    sc->rx_last_word3,
	    (sc->rx_last_word3 & MT_RXDESC_W3_L4_VALID) ? "set" : "clear");
	sbuf_printf(sb, "err   crc %lu phy %lu short %lu long %lu fifo %lu "
	    "tx_skip %lu tx_coll %lu\n",
	    sc->rx_crc_err, sc->rx_phy_err, sc->rx_short_err, sc->rx_long_err,
	    sc->rx_fifo_overflows, sc->tx_skip, sc->tx_collision);

	error = sbuf_finish(sb);
	sbuf_delete(sb);

	return (error);
}

/*
 * mtge_sysctl_attach - attach sysctl nodes for NIC counters.
 */
/*
 * mtge_sysctl_flowctl - turn flow control on the trunks on or off in place.
 *
 * Reprograms MAC_P_MCR of every MAC that has been set up, with the media
 * word it was last given; everything but the two FC bits comes out the same.
 */
static int
mtge_sysctl_flowctl(SYSCTL_HANDLER_ARGS)
{
	struct mtge_softc *sc = arg1;
	int error, g, val;

	val = sc->flowctl;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	val = (val != 0);

	MTGE_LOCK(sc);
	if (val != sc->flowctl) {
		sc->flowctl = val;
		for (g = 0; g < 2; g++) {
			if (sc->trunk_media[g] != 0)
				mtge_mac_change(sc, sc->trunk_media[g], g);
		}
	}
	MTGE_UNLOCK(sc);

	return (0);
}

static void
mtge_sysctl_attach(struct mtge_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	struct sysctl_oid *stats;

	ctx = device_get_sysctl_ctx(sc->dev);
	tree = device_get_sysctl_tree(sc->dev);

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "rx_ring_size",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    mtge_sysctl_ring_size, "I",
	    "Rx ring entries per queue; power of two, rebuilds the rings");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "tx_ring_size",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 1,
	    mtge_sysctl_ring_size, "I",
	    "Tx packets in flight; power of two, rebuilds the rings");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "dump",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    mtge_sysctl_dump, "A",
	    "dump MAC link, frame engine and PDMA ring state");

	/* statistic counters */
	stats = SYSCTL_ADD_NODE(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "stats", CTLFLAG_RD | CTLFLAG_MPSAFE, 0, "statistic");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "interrupts", CTLFLAG_RD, &sc->interrupts,
	    "all interrupts");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "tx_coherent_interrupts", CTLFLAG_RD, &sc->tx_coherent_interrupts,
	    "Tx coherent interrupts");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_coherent_interrupts", CTLFLAG_RD, &sc->rx_coherent_interrupts,
	    "Rx coherent interrupts");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_stall_kicks", CTLFLAG_RD, &sc->rx_stall_kicks,
	    "Rx drains the periodic task had to start itself");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "tx_stall_kicks", CTLFLAG_RD, &sc->tx_stall_kicks,
	    "Tx reclaims the periodic task had to start itself");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "tx_intr_lost", CTLFLAG_RD, &sc->tx_intr_lost,
	    "Tx completions that landed while the Tx sources were masked");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO, "flowctl",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    mtge_sysctl_flowctl, "I",
	    "Honour and send pause frames on the trunks (0 or 1)");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_gdm2_packets", CTLFLAG_RD, &sc->rx_gdm2_packets,
	    "Frames received through GDM2 for mtge1");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_sport_drops", CTLFLAG_RD, &sc->rx_sport_drops,
	    "GDM2 frames dropped because mtge1 is absent or down");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_csum_valid", CTLFLAG_RD, &sc->rx_csum_valid,
	    "Frames the engine reported a good L4 checksum for");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_csum_none", CTLFLAG_RD, &sc->rx_csum_none,
	    "Frames it did not, left for the stack to verify");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_delay_interrupts", CTLFLAG_RD, &sc->rx_delay_interrupts,
	    "Rx delay interrupts");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "tx_delay_interrupts", CTLFLAG_RD, &sc->tx_delay_interrupts,
	    "Tx delay interrupts");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "tx_watchdog_timeouts", CTLFLAG_RD, &sc->tx_watchdog_timeouts,
	    "Tx watchdog timeouts");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "tx_defrag_packets", CTLFLAG_RD, &sc->tx_defrag_packets,
	    "Tx defragmented packets");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "no_tx_desc_avail", CTLFLAG_RD, &sc->no_tx_desc_avail,
	    "no Tx descriptors available");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_mbuf_alloc_errors", CTLFLAG_RD, &sc->rx_mbuf_alloc_errors,
	    "Rx mbuf allocation errors");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_mbuf_dmamap_errors", CTLFLAG_RD, &sc->rx_mbuf_dmamap_errors,
	    "Rx mbuf DMA mapping errors");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_packets", CTLFLAG_RD, &sc->rx_packets,
	    "Rx packets seen by the frame engine (GDMA)");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_pdma_packets", CTLFLAG_RD, &sc->rx_pdma_packets,
	    "Rx packets taken off the PDMA rings and given to the stack");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_crc_errors", CTLFLAG_RD, &sc->rx_crc_err,
	    "Rx CRC errors");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_phy_errors", CTLFLAG_RD, &sc->rx_phy_err,
	    "Rx PHY errors");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_fifo_overflows", CTLFLAG_RD, &sc->rx_fifo_overflows,
	    "Rx FIFO overflows");

	SYSCTL_ADD_QUAD(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_bytes", CTLFLAG_RD, &sc->rx_bytes,
	    "Rx bytes");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_long_err", CTLFLAG_RD, &sc->rx_long_err,
	    "Rx too long frame errors");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "rx_short_err", CTLFLAG_RD, &sc->rx_short_err,
	    "Rx too short frame errors");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "tx_bytes", CTLFLAG_RD, &sc->tx_bytes,
	    "Tx bytes");

	SYSCTL_ADD_QUAD(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "tx_packets", CTLFLAG_RD, &sc->tx_packets,
	    "Tx packets");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "tx_skip", CTLFLAG_RD, &sc->tx_skip,
	    "Tx skip count for GDMA ports");

	SYSCTL_ADD_ULONG(ctx, SYSCTL_CHILDREN(stats), OID_AUTO,
	    "tx_collision", CTLFLAG_RD, &sc->tx_collision,
	    "Tx collision count for GDMA ports");
}

static int
mtge_miibus_wait_idle(struct mtge_softc *sc)
{
	uint32_t dat;
	int retry;

	for (retry = MII_BUSY_RETRY; retry > 0; retry--) {
		dat = MT_READ(sc, MDIO_ACCESS);
		if (!(dat & MDIO_CMD_ONGO))
			break;
		DELAY(10);
	}

	return (retry);
}

/*
 * PSEUDO_PHYAD is a special value for indicate switch attached.
 * No one PHY use PSEUDO_PHYAD (0x1e) address.
 */

static int
mtge_mdio_writereg(device_t dev, int phy, int reg, int val)
{
	struct mtge_softc *sc = device_get_softc(dev);
	int dat;
	int retry;
	int st = MDIO_ST_C22;
	int cmd = MDIO_CMD_WRITE;

	/* Wait prev command done if any */
	retry = mtge_miibus_wait_idle(sc);
	if (!retry) {
		device_printf(dev, "phy write timeout, phy=%d reg=%d\n",
		    phy, reg);
		return (ETIMEDOUT);
	}

	dat = (st << MDIO_ST_SHIFT) |
	    ((cmd << MDIO_CMD_SHIFT) & MDIO_CMD_MASK) |
	    ((phy << MDIO_PHY_ADDR_SHIFT) & MDIO_PHY_ADDR_MASK) |
	    ((reg << MDIO_PHYREG_ADDR_SHIFT) & MDIO_PHYREG_ADDR_MASK) |
	    (val & MDIO_PHY_DATA_MASK);

	MT_WRITE(sc, MDIO_ACCESS, dat);
	MT_WRITE(sc, MDIO_ACCESS, dat | MDIO_CMD_ONGO);

	retry = mtge_miibus_wait_idle(sc);
	if (!retry) {
		device_printf(dev, "phy write timeout, phy=%d reg=%d\n",
		    phy, reg);
		return (ETIMEDOUT);
	}

	return (0);
}

static int
mtge_mdio_readreg(device_t dev, int phy, int reg)
{
	struct mtge_softc *sc = device_get_softc(dev);
	int dat;
	int retry;
	int st = MDIO_ST_C22;
	int cmd = MDIO_CMD_READ;

	/* Wait prev command done if any */
	retry = mtge_miibus_wait_idle(sc);
	if (!retry) {
		device_printf(dev, "phy read timeout, phy=%d reg=%d\n",
		    phy, reg);
		/* MDIO convention: a failed read floats to all-ones. */
		return (0xffff);
	}

	dat = (st << MDIO_ST_SHIFT) |
	    ((cmd << MDIO_CMD_SHIFT) & MDIO_CMD_MASK) |
	    ((phy << MDIO_PHY_ADDR_SHIFT) & MDIO_PHY_ADDR_MASK) |
	    ((reg << MDIO_PHYREG_ADDR_SHIFT) & MDIO_PHYREG_ADDR_MASK);

	MT_WRITE(sc, MDIO_ACCESS, dat);
	MT_WRITE(sc, MDIO_ACCESS, dat | MDIO_CMD_ONGO);

	retry = mtge_miibus_wait_idle(sc);
	if (!retry) {
		device_printf(dev, "phy read timeout, phy=%d reg=%d\n",
		    phy, reg);
		return (0xffff);
	}

	return (MT_READ(sc, MDIO_ACCESS) & MDIO_PHY_DATA_MASK);
}

static int
mtge_attach_mdio_bus(struct mtge_softc *sc)
{
	struct mtge_ofw_devinfo *di;
	device_t cdev;
	phandle_t node, mdio;

	node = ofw_bus_get_node(sc->dev);
	if (node == -1)
		return (0);

	mdio = ofw_bus_find_child(node, "mdio-bus");
	if (mdio == 0)
		mdio = ofw_bus_find_child(node, "mdio");
	if (mdio == 0)
		return (0);

	di = malloc(sizeof(*di), M_DEVBUF, M_WAITOK | M_ZERO);
	if (ofw_bus_gen_setup_devinfo(&di->di_dinfo, mdio) != 0) {
		free(di, M_DEVBUF);
		return (0);
	}
	resource_list_init(&di->di_rl);
	cdev = device_add_child(sc->dev, "mtmdio", DEVICE_UNIT_ANY);
	if (cdev == NULL) {
		device_printf(sc->dev, "could not add mtmdio bus\n");
		resource_list_free(&di->di_rl);
		ofw_bus_gen_destroy_devinfo(&di->di_dinfo);
		free(di, M_DEVBUF);
		return (0);
	}
	device_set_ivars(cdev, di);
	bus_attach_children(sc->dev);

	return (1);
}

static const struct ofw_bus_devinfo *
mtge_ofw_get_devinfo(device_t bus, device_t child)
{
	struct mtge_ofw_devinfo *di = device_get_ivars(child);

	/* Legacy (hinted) children have no OF devinfo. */
	if (di == NULL)
		return (NULL);
	return (&di->di_dinfo);
}

static struct resource_list *
mtge_ofw_get_resource_list(device_t bus, device_t child)
{
	struct mtge_ofw_devinfo *di = device_get_ivars(child);

	if (di == NULL)
		return (NULL);
	return (&di->di_rl);
}

static device_method_t mtge_dev_methods[] =
{
	DEVMETHOD(device_probe, mtge_probe),
	DEVMETHOD(device_attach, mtge_attach),
	DEVMETHOD(device_detach, mtge_detach),

	/* Bus interface for the OF children on our MDIO bus. */
	DEVMETHOD(bus_add_child,		bus_generic_add_child),
	DEVMETHOD(bus_print_child,		bus_generic_print_child),
	DEVMETHOD(bus_alloc_resource,	bus_generic_rl_alloc_resource),
	DEVMETHOD(bus_release_resource,	bus_generic_rl_release_resource),
	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,
	    bus_generic_deactivate_resource),
	DEVMETHOD(bus_get_resource_list,	mtge_ofw_get_resource_list),

	/* OFW bus interface (lets children read their own node/compat). */
	DEVMETHOD(ofw_bus_get_devinfo,	mtge_ofw_get_devinfo),
	DEVMETHOD(ofw_bus_get_compat,	ofw_bus_gen_get_compat),
	DEVMETHOD(ofw_bus_get_model,		ofw_bus_gen_get_model),
	DEVMETHOD(ofw_bus_get_name,		ofw_bus_gen_get_name),
	DEVMETHOD(ofw_bus_get_node,		ofw_bus_gen_get_node),
	DEVMETHOD(ofw_bus_get_type,		ofw_bus_gen_get_type),

	/* MDIO interface */
	DEVMETHOD(mdio_readreg,		mtge_mdio_readreg),
	DEVMETHOD(mdio_writereg,	        mtge_mdio_writereg),

	DEVMETHOD_END
};

static driver_t mtge_driver =
{
	    "mtge",
	    mtge_dev_methods,
	    sizeof(struct mtge_softc)
};

DRIVER_MODULE(miibus, mtge, miibus_driver, 0, 0);
DRIVER_MODULE(mdio, mtge, mdio_driver, 0, 0);

#ifdef FDT
DRIVER_MODULE(mtge, simplebus, mtge_driver, 0, 0);
#endif

MODULE_DEPEND(mtge, ether, 1, 1, 1);
MODULE_DEPEND(mtge, miibus, 1, 1, 1);
MODULE_DEPEND(mtge, mdio, 1, 1, 1);

