/*
 * Copyright (c) 2025, 2026 Martin Filla <freebsd@sysctl.cz>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef _IF_MTGEREG_H_
#define	_IF_MTGEREG_H_

#define	MT_READ(sc, reg)				\
	bus_space_read_4((sc)->bst, (sc)->bsh, reg)

#define	MT_WRITE(sc, reg, val)				\
	bus_space_write_4((sc)->bst, (sc)->bsh, reg, val)

#define	MT_GDMA1_BASE		0x500
#define	MT_GDMA2_BASE		MT_GDMA1_BASE + 0x1000
#define	MT_GDM_BASE(gmac)	((gmac == 0) ? MT_GDMA1_BASE : MT_GDMA2_BASE)

#define	MT_GDM_IG_CTRL(gmac)	(MT_GDM_BASE(gmac) + 0x00)
#define	    INSV_EN		(1<<25)
#define	    STAG_EN		(1<<24)
#define	    GDM_ICS_EN		(1<<22)
#define	    GDM_TCS_EN		(1<<21)
#define	    GDM_UCS_EN		(1<<20)
#define	    GDM_DROP_256B	(1<<19)
#define	    GDM_STRPCRC		(1<<16)
#define	    GDM_UFRC_P_SHIFT	12
#define	    GDM_BFRC_P_SHIFT	8
#define	    GDM_MFRC_P_SHIFT	4
#define	    GDM_OFRC_P_SHIFT	0
#define	    GDM_XFRC_P_MASK	0x07
#define	    GDM_DST_PORT_CPU	0
#define	    GDM_DST_PORT_GDMA1	1
#define	    GDM_DST_PORT_GDMA2	2
#define	    GDM_DST_PORT_PPE	4
#define	    GDM_DST_PORT_QDMA	5
#define	    GDM_DST_PORT_DISCARD 7

#define	MT_GDM_MAC_LSB(gmac)  (MT_GDM_BASE(gmac) + 0x08)
#define	MT_GDM_MAC_MSB(gmac)  (MT_GDM_BASE(gmac) + 0x0c)

#define	MT_PDMA_BASE 0x0800

#define	MT_TX_BASE_PTR(qid)			\
	((qid) * 0x10 + MT_PDMA_BASE + 0x000)
#define	MT_TX_MAX_CNT(qid)			\
	((qid) * 0x10 + MT_PDMA_BASE + 0x004)
#define	MT_TX_CTX_IDX(qid)			\
	((qid) * 0x10 + MT_PDMA_BASE + 0x008)
#define	MT_TX_DTX_IDX(qid)			\
	((qid) * 0x10 + MT_PDMA_BASE + 0x00C)

#define	MT_RX_BASE_PTR(qid)			\
	((qid) * 0x10 + MT_PDMA_BASE + 0x100)
#define	MT_RX_MAX_CNT(qid)			\
	((qid) * 0x10 + MT_PDMA_BASE + 0x104)
#define	MT_RX_CALC_IDX(qid)			\
	((qid) * 0x10 + MT_PDMA_BASE + 0x108)
#define	MT_RX_DRX_IDX(qid)			\
	((qid) * 0x10 + MT_PDMA_BASE + 0x10C)

#define	MT_PDMA_GLO_CFG	(MT_PDMA_BASE + 0x204)
#define	    FE_RX_2B_OFFSET	(1<<31)
#define	    FE_TX_WB_DDONE	(1<<6)
#define	    FE_DMA_BT_SIZE4	(0<<4)
#define	    FE_DMA_BT_SIZE8	(1<<4)
#define	    FE_DMA_BT_SIZE16	(2<<4)
#define	    FE_RX_DMA_BUSY	(1<<3)
#define	    FE_RX_DMA_EN	(1<<2)
#define	    FE_TX_DMA_BUSY	(1<<1)
#define	    FE_TX_DMA_EN	(1<<0)

#define	MT_PDMA_RST_IDX	(MT_PDMA_BASE + 0x208)
#define	    FE_RST_DRX_IDX1	(1<<17)
#define	    FE_RST_DRX_IDX0	(1<<16)
#define	    FE_RST_DTX_IDX3	(1<<3)
#define	    FE_RST_DTX_IDX2	(1<<2)
#define	    FE_RST_DTX_IDX1	(1<<1)
#define	    FE_RST_DTX_IDX0	(1<<0)

#define	MT_DELAY_INT_CFG	(MT_PDMA_BASE + 0x20C)
#define	    TXDLY_INT_EN	(1<<31)
#define	    TXMAX_PINT_SHIFT	24
#define	    TXMAX_PTIME_SHIFT	16
#define	    RXDLY_INT_EN	(1<<15)
#define	    RXMAX_PINT_SHIFT	8
#define	    RXMAX_PTIME_SHIFT	0
/*
 * Both halves hold a packet count in 7 bits and a timer in 8, and the engine
 * raises the delayed interrupt when either is reached.  The timer counts in
 * the frame engine's own ticks, 20us each per the vendor documentation, so
 * the whole 8-bit range covers about 5ms.
 */
#define	    DLY_PINT_MAX	0x7f
#define	    DLY_PTIME_MAX	0xff
#define	    DLY_PTIME_US	20

#define	MT_PDMA_INT_STATUS    (MT_PDMA_BASE + 0x220)
#define	    MT_INT_RX_COHERENT      (1<<31)
#define	    MT_RX_DLY_INT           (1<<30)
#define	    MT_INT_TX_COHERENT      (1<<29)
#define	    MT_TX_DLY_INT           (1<<28)
#define	    MT_INT_RXQ3_DONE	       (1<<19)
#define	    MT_INT_RXQ2_DONE        (1<<18)
#define	    MT_INT_RXQ1_DONE	       (1<<17)
#define	    MT_INT_RXQ0_DONE        (1<<16)
#define	    MT_INT_TXQ3_DONE        (1<<3)
#define	    MT_INT_TXQ2_DONE        (1<<2)
#define	    MT_INT_TXQ1_DONE        (1<<1)
#define	    MT_INT_TXQ0_DONE        (1<<0)
#define	MT_PDMA_INT_ENABLE	(MT_PDMA_BASE + 0x228)
#define	MT_PDMA_SCH_CFG0	(MT_PDMA_BASE + 0x280)

/*
 * Global reset.  Changing where the GDMAs forward their ingress leaves the
 * PSE holding packets destined for the old target, so the vendor driver
 * pulses this after every such change; without it the switchover is not
 * clean.
 */
#define	MT_RST_GL		0x0004
#define	   RST_GL_PSE		(1 << 0)

#define	CNTR_BASE 0x2400
#define	CNTR_GDM(gmac)		(CNTR_BASE + (((gmac) == 0 ) ? 0x00 : 0x40))
#define	    GDM_RX_GBCNT_LSB(gmac)	(CNTR_GDM(gmac) + 0x00)
#define	    GDM_RX_GBCNT_MSB(gmac)	(CNTR_GDM(gmac) + 0x04)
#define	    GDM_RX_GPCNT(gmac)		(CNTR_GDM(gmac) + 0x08)
#define	    GDM_RX_OERCNT(gmac)		(CNTR_GDM(gmac) + 0x10)
#define	    GDM_RX_FERCNT(gmac)		(CNTR_GDM(gmac) + 0x14)
#define	    GDM_RX_SHORT_ERCNT(gmac)	(CNTR_GDM(gmac) + 0x18)
#define	    GDM_RX_LONG_ERCNT(gmac)	(CNTR_GDM(gmac) + 0x1C)
#define	    GDM_RX_CSUM_ERCNT(gmac)	(CNTR_GDM(gmac) + 0x20)
#define	    GDM_RX_FCCNT(gmac)		(CNTR_GDM(gmac) + 0x24)
#define	    GDM_TX_SKIPCNT(gmac)	(CNTR_GDM(gmac) + 0x28)
#define	    GDM_TX_COLCNT(gmac)		(CNTR_GDM(gmac) + 0x2C)
#define	    GDM_TX_GBCNT_LSB(gmac)	(CNTR_GDM(gmac) + 0x30)
#define	    GDM_TX_GBCNT_MSB(gmac)	(CNTR_GDM(gmac) + 0x34)
#define	    GDM_TX_GPCNT(gmac)		(CNTR_GDM(gmac) + 0x38)

#define	GE_PORT_BASE 0x10000
#define	MDIO_ACCESS	GE_PORT_BASE + 0x04
#define	    MDIO_CMD_ONGO		(1<<31)
#define	    MDIO_PHYREG_ADDR_MASK	0x3e000000
#define	    MDIO_PHYREG_ADDR_SHIFT	25
#define	    MDIO_PHY_ADDR_MASK		0x01f00000
#define	    MDIO_PHY_ADDR_SHIFT		20
#define	    MDIO_CMD_MASK		0x000c0000
#define	    MDIO_CMD_SHIFT		18
#define	    MDIO_CMD_WRITE		0x1
#define	    MDIO_CMD_READ		0x2
#define	    MDIO_CMD_READ_C45		0x3
#define	    MDIO_ST_MASK		0x30000
#define	    MDIO_ST_SHIFT		16
#define	    MDIO_ST_C45			0x0
#define	    MDIO_ST_C22			0x1
#define	    MDIO_PHY_DATA_MASK		0x0000ffff
#define	    MDIO_PHY_DATA_SHIFT		0

#define	MAC_P_MCR(gmac)		(GE_PORT_BASE + 0x100 + (gmac) * 0x100)
#define	   MAX_RX_JUMBO_MASK	0xf0000000
#define	   MAX_RX_JUMBO_SHIFT	28
#define	   MAX_RX_JUMBO_2K	0x2	/*2 Kbytes (maxi. length on FE/GDM) */
#define	   MAC_RX_PKT_LEN_MASK	0x03000000
#define	   MAC_RX_PKT_LEN_SHIFT	24
#define	   MAC_RX_PKT_LEN_1518	0x0
#define	   MAC_RX_PKT_LEN_1536	0x1
#define	   MAC_RX_PKT_LEN_1552	0x2
#define	   MAC_RX_PKT_LEN_JUMBO	0x3	/* MAX_RX_JUMBO */
#define	   MTCC_LMT_MASK	0x00F00000
#define	   MTCC_LMT_SHIFT	20
#define	   IPG_CFG_MASK		0x000c0000
#define	   IPG_CFG_SHIFT	18
#define	   IPG_CFG_96BIT	0x0
#define	   IPG_CFG_96BIT_WS_IFG	0x2
/*
 * Both IPG bits set is what the vendor driver programs for gigabit links
 * ("96 bit IPG with short IFG"); it is not a 64 bit IPG despite the name
 * this field carried historically.
 */
#define	   IPG_CFG_96BIT_SHORT	0x3
#define	   MAC_MODE		(1 << 16)
#define	   FORCE_MODE		(1 << 15)
#define	   MAC_TX_EN		(1 << 14)
#define	   MAC_RX_EN		(1 << 13)
#define	   PRMBL_LMT_EN		(1 << 10)
#define	   BKOFF_EN		(1 << 9)
#define	   BACKPR_EN		(1 << 8)
#define	   FORCE_EEE1G		(1 << 7)
#define	   FORCE_EEE100		(1 << 6)
#define	   FORCE_RX_FC		(1 << 5)
#define	   FORCE_TX_FC		(1 << 4)
#define	   FORCE_SPD_MASK	0x0000000c
#define	   FORCE_SPD_SHIFT	2
#define	   FORCE_SPD_10M	0x0
#define	   FORCE_SPD_100M	0x1
#define	   FORCE_SPD_1000M	0x2
#define	   FORCE_DPX		(1 << 1)
#define	   FORCE_LINK		(1 << 0)

/*
 * Per-GMAC status register.  MAC_P_MCR only says what we asked the forced
 * trunk to do; this is the only place that reports what the link actually
 * does.  mtge_ifmedia_sts() reports IFM_ACTIVE from sc->link_up, which
 * mtge_init_locked() sets unconditionally, so ifconfig(8) cannot answer
 * "is the MAC<->MT7531 line up?" - MSR_LINK can.
 */
#define	MAC_P_MSR(gmac)		(GE_PORT_BASE + 0x108 + (gmac) * 0x100)
#define	   MSR_EEE1G		(1 << 7)
#define	   MSR_EEE100M		(1 << 6)
#define	   MSR_RX_FC		(1 << 5)
#define	   MSR_TX_FC		(1 << 4)
#define	   MSR_SPD_MASK		0x0000000c
#define	   MSR_SPD_SHIFT	2
#define	   MSR_SPD_10M		0x0
#define	   MSR_SPD_100M		0x1
#define	   MSR_SPD_1000M	0x2
#define	   MSR_DPX		(1 << 1)
#define	   MSR_LINK		(1 << 0)

/*
 * Packet processing engine (netsys v1), at +0x0c00 inside the frame engine
 * block.  Layout follows the vendor's Linux driver (mtk_ppe_regs.h, v1
 * paths); dev.mtge.<unit>.ppe_dump prints readbacks so every value can be
 * checked against running hardware.
 */
#define	MT_PPE_BASE			0x0c00
#define	MT_PPE(x)			(MT_PPE_BASE + (x))

#define	MT_PPE_GLO_CFG			MT_PPE(0x200)
#define	    PPE_GLO_CFG_EN			(1 << 0)
#define	    PPE_GLO_CFG_IP4_L4_CS_DROP		(1 << 2)
#define	    PPE_GLO_CFG_IP4_CS_DROP		(1 << 3)
#define	    PPE_GLO_CFG_TTL0_DROP		(1 << 4)
#define	    PPE_GLO_CFG_FLOW_DROP_UPDATE	(1 << 9)
#define	    PPE_GLO_CFG_BUSY			(1u << 31)

#define	MT_PPE_FLOW_CFG			MT_PPE(0x204)
#define	    PPE_FLOW_CFG_IP4_TCP_FRAG		(1 << 6)
#define	    PPE_FLOW_CFG_IP4_UDP_FRAG		(1 << 7)
#define	    PPE_FLOW_CFG_IP6_3T_ROUTE		(1 << 8)
#define	    PPE_FLOW_CFG_IP6_5T_ROUTE		(1 << 9)
#define	    PPE_FLOW_CFG_IP6_6RD		(1 << 10)
#define	    PPE_FLOW_CFG_IP4_NAT		(1 << 12)
#define	    PPE_FLOW_CFG_IP4_NAPT		(1 << 13)
#define	    PPE_FLOW_CFG_IP4_DSLITE		(1 << 14)
#define	    PPE_FLOW_CFG_IP4_NAT_FRAG		(1 << 17)

#define	MT_PPE_IP_PROTO_CHK		MT_PPE(0x208)
#define	    PPE_IP_PROTO_CHK_IPV4		0x0000ff00
#define	    PPE_IP_PROTO_CHK_IPV6		0xff000000

#define	MT_PPE_TB_CFG			MT_PPE(0x21c)
#define	    PPE_TB_CFG_ENTRY_80B		(1 << 3)
#define	    PPE_TB_CFG_SEARCH_MISS_SHIFT	4	/* 2 bits */
#define	    PPE_TB_CFG_AGE_PREBIND		(1 << 6)
#define	    PPE_TB_CFG_AGE_NON_L4		(1 << 7)
#define	    PPE_TB_CFG_AGE_UNBIND		(1 << 8)
#define	    PPE_TB_CFG_AGE_TCP			(1 << 9)
#define	    PPE_TB_CFG_AGE_UDP			(1 << 10)
#define	    PPE_TB_CFG_AGE_TCP_FIN		(1 << 11)
#define	    PPE_TB_CFG_KEEPALIVE_SHIFT		12	/* 2 bits */
#define	    PPE_TB_CFG_HASH_MODE_SHIFT		14	/* 2 bits */
#define	    PPE_TB_CFG_SCAN_MODE_SHIFT		16	/* 2 bits */
#define	    PPE_SEARCH_MISS_FORWARD_BUILD	3
#define	    PPE_KEEPALIVE_DISABLE		0
#define	    PPE_SCAN_MODE_KEEPALIVE_AGE		2

#define	MT_PPE_TB_BASE			MT_PPE(0x220)

#define	MT_PPE_BIND_RATE		MT_PPE(0x228)
#define	    PPE_BIND_RATE_BIND_SHIFT		0	/* 16 bits */
#define	    PPE_BIND_RATE_PREBIND_SHIFT		16	/* 16 bits */
#define	MT_PPE_BIND_LIMIT0		MT_PPE(0x22c)
#define	    PPE_BIND_LIMIT0_QUARTER_SHIFT	0	/* 14 bits */
#define	    PPE_BIND_LIMIT0_HALF_SHIFT		16	/* 14 bits */
#define	MT_PPE_BIND_LIMIT1		MT_PPE(0x230)
#define	    PPE_BIND_LIMIT1_FULL_SHIFT		0	/* 14 bits */
#define	    PPE_BIND_LIMIT1_NON_L4_SHIFT	16	/* 8 bits */
#define	MT_PPE_KEEPALIVE		MT_PPE(0x234)
#define	MT_PPE_UNBIND_AGE		MT_PPE(0x238)
#define	    PPE_UNBIND_AGE_DELTA_SHIFT		0	/* 8 bits */
#define	    PPE_UNBIND_AGE_MIN_PKT_SHIFT	16	/* 16 bits */
#define	MT_PPE_BIND_AGE0		MT_PPE(0x23c)
#define	    PPE_BIND_AGE0_DELTA_UDP_SHIFT	0	/* 15 bits */
#define	    PPE_BIND_AGE0_DELTA_NON_L4_SHIFT	16	/* 15 bits */
#define	MT_PPE_BIND_AGE1		MT_PPE(0x240)
#define	    PPE_BIND_AGE1_DELTA_TCP_SHIFT	0	/* 15 bits */
#define	    PPE_BIND_AGE1_DELTA_TCP_FIN_SHIFT	16	/* 15 bits */
#define	MT_PPE_DEFAULT_CPU_PORT		MT_PPE(0x248)
#define	MT_PPE_CACHE_CTL		MT_PPE(0x320)
#define	    PPE_CACHE_CTL_EN			(1 << 0)
#define	    PPE_CACHE_CTL_CLEAR			(1 << 9)

/*
 * FOE table geometry, netsys v1: 8192 entries of 80 bytes; ENTRY_NUM in
 * TB_CFG holds the size as a shift, entries = 1024 << shift.  The state of
 * an entry lives in bits 29:28 of its first word.
 */
#define	MT_PPE_ENTRIES_SHIFT		3
#define	MT_PPE_ENTRIES			(1024 << MT_PPE_ENTRIES_SHIFT)
#define	MT_PPE_ENTRY_SIZE		80
#define	MT_FOE_IB1_STATE_SHIFT		28
#define	MT_FOE_IB1_STATE_MASK		(0x3u << MT_FOE_IB1_STATE_SHIFT)
#define	    MT_FOE_STATE_INVALID		0
#define	    MT_FOE_STATE_UNBIND			1
#define	    MT_FOE_STATE_BIND			2
#define	    MT_FOE_STATE_FIN			3

/* The rest of the first info word, for entries the driver binds itself. */
#define	MT_FOE_IB1_TS_MASK		0x00007fff
#define	MT_FOE_IB1_BIND_CACHE		(1 << 22)
#define	MT_FOE_IB1_BIND_TTL		(1 << 24)
#define	MT_FOE_IB1_PKT_TYPE_SHIFT	25	/* 3 bits */
#define	    MT_FOE_PKT_IPV4_HNAPT		0
#define	MT_FOE_IB1_UDP			(1 << 30)

/* Second info word of a bound entry: where the engine sends the flow. */
#define	MT_FOE_IB2_DEST_PORT_SHIFT	5	/* 3 bits */
#define	    MT_FOE_PSE_PORT_GDM1		1
#define	    MT_FOE_PSE_PORT_GDM2		2
/*
 * PORT_MG must stay zero on this chip.  The field overlaps the WDMA
 * destination bits (16 and 17), and a driver that fills it with ones -- as
 * the older MediaTek parts want -- tells the engine every bound flow is
 * destined for a WiFi DMA device.  The frames then never reach the GDMA in
 * DEST_PORT and the flow black-holes the moment it is bound.
 */
#define	MT_FOE_IB2_PORT_MG_SHIFT	12	/* 6 bits; zero here */
#define	MT_FOE_IB2_PORT_AG_SHIFT	18	/* 6 bits, set them all */

/*
 * Word indexes into an IPv4 HNAPT entry, viewed as 32-bit LE words.  The
 * original tuple is what the engine hashed on ingress -- it fills those
 * words itself while the entry is UNBIND; the new tuple and the MACs are
 * what it rewrites the frame to on egress once the entry is bound.  IP
 * addresses and ports sit in their words in host byte order.
 */
#define	MT_FOE_W_IB1			0
#define	MT_FOE_W_ORIG_SIP		1
#define	MT_FOE_W_ORIG_DIP		2
#define	MT_FOE_W_ORIG_PORTS		3	/* dport | sport << 16 */
#define	MT_FOE_W_IB2			4
#define	MT_FOE_W_NEW_SIP		5
#define	MT_FOE_W_NEW_DIP		6
#define	MT_FOE_W_NEW_PORTS		7	/* dport | sport << 16 */
#define	MT_FOE_W_VLAN_ETYPE		11	/* vlan1 | etype << 16 */
#define	MT_FOE_W_DMAC_HI		12
#define	MT_FOE_W_VLAN2_DMAC_LO		13	/* vlan2 | dmac_lo << 16 */
#define	MT_FOE_W_SMAC_HI		14
#define	MT_FOE_W_PPPOE_SMAC_LO		15	/* pppoe | smac_lo << 16 */

/*
 * rxd4 of a frame that went through the engine: bits 13:0 hold the FOE
 * entry the flow hashed to, bits 18:14 the reason the frame was punted to
 * the CPU anyway.  HIT_UNBIND_RATE is the reason worth acting on -- the
 * flow has crossed the UNBIND_AGE packet threshold and the engine is
 * asking for a binding.
 */
#define	MT_RXD4_FOE_ENTRY_MASK		0x3fff
#define	MT_RXD4_REASON_SHIFT		14
#define	MT_RXD4_REASON_MASK		0x1f
#define	    MT_PPE_REASON_NO_FLOW		0x07
#define	    MT_PPE_REASON_UN_HIT		0x0d
#define	    MT_PPE_REASON_HIT_UNBIND		0x0e
#define	    MT_PPE_REASON_HIT_UNBIND_RATE	0x0f
#define	    MT_PPE_REASON_HIT_BIND_TCP_FIN	0x10
#define	    MT_PPE_REASON_HIT_TTL_1		0x11
#define	    MT_PPE_REASON_HIT_BIND_VLAN_VIOL	0x12
#define	    MT_PPE_REASON_HIT_BIND_FORCE_CPU	0x16
#define	    MT_PPE_REASON_COUNT			32

/* Free-running frame engine clock; its low 15 bits are the FOE timestamp. */
#define	MT_FE_TIMESTAMP			0x0010

#endif /* _IF_MTGEREG_H_ */
