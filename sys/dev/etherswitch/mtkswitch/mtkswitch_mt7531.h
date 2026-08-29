/*
 * Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
 * Copyright (c) 2023 Priit Trees.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef	__MTKSWITCH_MT7531_H__
#define	__MTKSWITCH_MT7531_H__

#define	MT7531_ATC  0x0080
#define		MT7531_ATC_ADDR_INVLD	(1u<<12)
#define		MT7531_ATC_SRCH_HIT	(1u<<13)
#define		MT7531_ATC_SRCH_END	(1u<<14)
#define		MT7531_ATC_BUSY		(1u<<15)
#define		MT7531_ATC_AC_MAT(x)    (((x) & 0xf) << 8)
#define	        MT7531_ATC_AC_CMD_CLEAN (2u<<0)

#define		MT7531_AC_CMD_RD	0x00
#define		MT7531_AC_CMD_RW	0x01
#define		MT7531_AC_CMD_CLN	0x02
#define		MT7531_AC_CMD_SSC	0x04
#define		MT7531_AC_CMD_NSC	0x05

#define	MT7531_VTCR	0x0090
#define		VTCR_BUSY		(1u<<31)
#define		VTCR_FUNC_VID_READ	(0u<<12)
#define		VTCR_FUNC_VID_WRITE	(1u<<12)
#define		VTCR_FUNC_VID_INVALID	(2u<<12)
#define		VTCR_FUNC_VID_VALID	(3u<<12)
#define		VTCR_IDX_INVALID	(1u<<16)
#define		VTCR_VID_MASK		0xfff

#define	MT7531_VAWD1	0x0094
#define		VAWD1_IVL_MAC		(1u<<30)
#define		VAWD1_VTAG_EN		(1u<<28)
#define		VAWD1_MEMBER_OFF	16
#define		VAWD1_MEMBER_MASK	0xff
#define		VAWD1_VALID		(1u<<0)

#define	MT7531_VAWD2	0x0098
#define		VAWD2_PORT_UNTAGGED(p)	(0u<<((p)*2))
#define		VAWD2_PORT_TAGGED(p)	(2u<<((p)*2))
#define		VAWD2_PORT_MASK(p)	(3u<<((p)*2))

/* Register for PHY Indirect Access Control */
#define MT7531_PHY_IAC	0x701C
#define		PHY_ACS_ST		(1u<<31)
#define		PHY_MDIO_REG_ADDR_OFF	25
#define		PHY_MDIO_PHY_ADDR_OFF	20
#define		PHY_MDIO_CMD_WRITE	(1u<<18)
#define		PHY_MDIO_CMD_READ	(2u<<18)
#define		PHY_MDIO_ST		(1u<<16)
#define		PHY_MDIO_RW_DATA_MASK	0xffff

/*
 * Clause 45 flavour of the same indirect access: the ST field (bit 16) is
 * left clear and the "register address" field carries the MMD device address
 * while the data field carries the register number.
 */
#define		PHY_MDIO_CL45_ADDR	(0u<<18)
#define		PHY_MDIO_CL45_WRITE	(1u<<18)
#define		PHY_MDIO_CL45_READ	(3u<<18)

#define	MT7531_CTRL_PHY_ADDR	0
#define	MT7531_MMD_VEND2	31
#define	MT7531_CORE_PLL_GROUP4	0x403
#define		PHY_PLL_OFF		(1u<<5)
#define		PHY_PLL_BYPASS_MODE	(1u<<4)

/*
 * Global switch configuration.
 *
 * MT7531_CFC carries the CPU port map: the set of ports the switch treats as
 * trunks towards the host MAC.  MTKSWITCH_MFC carries the flood maps for
 * broadcast, unknown multicast and unknown unicast frames.  Neither has a
 * useful power-on value for us, so both have to be programmed explicitly or
 * nothing (not even ARP) is forwarded to the CPU.
 *
 * CFC layout, from the MT7531 datasheet:
 *   31:20  reserved
 *   19     MIRROR_EN    enable the mirror port in MIRROR_PORT
 *   18:16  MIRROR_PORT  mirror port number, 0..7
 *   15:8   reserved
 *   7:0    CPU_PMAP     CPU port bit map, bit n = port n
 * CPU_PMAP is a bitmap in the low byte; bits 15:8 are reserved and
 * writing the map there leaves the real field at zero.
 */
#define	MT7531_CFC	0x0004
#define		CFC_CPU_PMAP_OFF	0
#define		CFC_CPU_PMAP_MASK	(0xffu << CFC_CPU_PMAP_OFF)
#define		CFC_CPU_PMAP(x)		(((x) << CFC_CPU_PMAP_OFF) & \
		   CFC_CPU_PMAP_MASK)
#define		CFC_CPU_PMAP_GET(x)	(((x) & CFC_CPU_PMAP_MASK) >> \
		   CFC_CPU_PMAP_OFF)
#define		CFC_MIRROR_EN		(1u << 19)
#define		CFC_MIRROR_PORT_OFF	16
#define		CFC_MIRROR_PORT_MASK	(0x7u << CFC_MIRROR_PORT_OFF)
#define		CFC_MIRROR_PORT(x)	(((x) << CFC_MIRROR_PORT_OFF) & \
		   CFC_MIRROR_PORT_MASK)
#define		CFC_MIRROR_PORT_GET(x)	(((x) & CFC_MIRROR_PORT_MASK) >> \
		   CFC_MIRROR_PORT_OFF)

/*
 *   MFC layout, from the MT7531 datasheet (reset 0xffffffff)
 *   31:24  BC_FFP    broadcast flooding ports
 *   23:16  UNM_FFP   unknown multicast flooding ports
 *   15:8   UNU_FFP   unknown unicast flooding ports
 *   7:0    QRY_FFP   IGMP/MLD query flooding ports
 */
#define	MT7531_MFC	0x0010
#define		MFC_BC_FFP(x)		(((x) & 0xffu) << 24)
#define		MFC_BC_FFP_MASK		MFC_BC_FFP(0xff)
#define		MFC_UNM_FFP(x)		(((x) & 0xffu) << 16)
#define		MFC_UNM_FFP_MASK	MFC_UNM_FFP(0xff)
#define		MFC_UNU_FFP(x)		(((x) & 0xffu) << 8)
#define		MFC_UNU_FFP_MASK	MFC_UNU_FFP(0xff)
#define		MFC_BC_FFP_GET(x)	(((x) >> 24) & 0xff)
#define		MFC_UNM_FFP_GET(x)	(((x) >> 16) & 0xff)
#define		MFC_UNU_FFP_GET(x)	(((x) >> 8) & 0xff)

/*
 * SGMII PCS.
 *
 * Ports 5 and 6 each carry a "LynxI" PCS - the same block the MT7622 SoC has
 * in its sgmiisys - at a per-port window inside the switch register space.
 * The offsets below are relative to that window.  The switch comes out of
 * SYS_CTRL reset with this block unprogrammed, so a trunk running over it
 * (2500base-x to gmac0 on the BananaPi R64) has to be set up explicitly.
 */
#define	MT7531_SGMII_BASE(p)	(0x5000 + (((p) - 5) * 0x1000))

#define	SGMII_PCS_CONTROL_1	0x00
#define		PCS_AN_ENABLE		(1u<<12)	/* BMCR_ANENABLE */
#define		PCS_AN_RESTART		(1u<<9)
#define		PCS_LINK_STATUS		(1u<<18)

#define	SGMII_PCS_ADVERTISE	0x08

/* Programmable link timer, counted in units of 2 * 8 ns. */
#define	SGMII_PCS_LINK_TIMER	0x18
#define		SGMII_LINK_TIMER_MASK	0x000fffff
#define		SGMII_LINK_TIMER_NS(ns)	(((ns) / 16) & SGMII_LINK_TIMER_MASK)
#define		SGMII_LINK_TIMER_BASEX	SGMII_LINK_TIMER_NS(10000000)
#define		SGMII_LINK_TIMER_SGMII	SGMII_LINK_TIMER_NS(1600000)

#define	SGMII_MODE		0x20
#define		SGMII_IF_MODE_SGMII	(1u<<0)
#define		SGMII_SPEED_DUPLEX_AN	(1u<<1)
#define		SGMII_FORCE_SPEED_MASK	(3u<<2)
#define		SGMII_FORCE_SPEED_10	(0u<<2)
#define		SGMII_FORCE_SPEED_100	(1u<<2)
#define		SGMII_FORCE_SPEED_1000	(2u<<2)
#define		SGMII_DUPLEX_HALF	(1u<<4)
#define		SGMII_REMOTE_FAULT_DIS	(1u<<8)

#define	SGMII_RESERVED_0	0x34
#define		SGMII_SW_RESET		(1u<<0)

#define	SGMII_QPHY_PWR_STATE_CTRL	0xe8
#define		SGMII_PHYA_PWD		(1u<<4)

/* ANA RG_ control signals III: the underlying serdes line rate. */
#define	SGMII_PHYA_CTRL_SIGNAL3	0x128
#define		SGMII_PHY_SPEED_MASK	(3u<<2)
#define		SGMII_PHY_SPEED_1_25G	(0u<<2)
#define		SGMII_PHY_SPEED_3_125G	(1u<<2)

#define	MTKSWITCH_SYS_CTRL	0x7000
#define		SYS_CTRL_PHY_RST	(1u<<2)
#define		SYS_CTRL_SW_RST		(1u<<1)
#define		SYS_CTRL_REG_RST	(1u<<0)

#define	MTKSWITCH_PORTREG(r, p)	((r) + ((p) * 0x100))

#define	MTKSWITCH_PCR(x)	MTKSWITCH_PORTREG(0x2004, (x))
#define		PCR_PORT_VLAN_SECURE	(3u<<0)
#define		PCR_MATRIX(x)		(((x) & 0xff) << 16)
#define		PCR_MATRIX_MASK		PCR_MATRIX(0xff)
#define		PCR_MATRIX_GET(x)	(((x) >> 16) & 0xff)

#define	MTKSWITCH_PVC(x)	MTKSWITCH_PORTREG(0x2010, (x))
#define		PVC_VLAN_ATTR_MASK	(3u<<6)

#define	MTKSWITCH_PPBV1(x)	MTKSWITCH_PORTREG(0x2014, (x))
#define	MTKSWITCH_PPBV2(x)	MTKSWITCH_PORTREG(0x2018, (x))
#define		PPBV_VID(v)		(((v)<<16) | (v))
#define		PPBV_VID_FROM_REG(x)	((x) & 0xfff)
#define		PPBV_VID_MASK		0xfff

#define	MTKSWITCH_PMCR(x)	MTKSWITCH_PORTREG(0x3000, (x))
#define		PMCR_FORCE_LINK		(1u<<0)
#define		PMCR_FORCE_DPX		(1u<<1)
#define		PMCR_FORCE_SPD_1000	(2u<<2)
#define		PMCR_FORCE_TX_FC	(1u<<4)
#define		PMCR_FORCE_RX_FC	(1u<<5)
#define		PMCR_BACKPR_EN		(1u<<8)
#define		PMCR_BKOFF_EN		(1u<<9)
#define		PMCR_MAC_RX_EN		(1u<<13)
#define		PMCR_MAC_TX_EN		(1u<<14)
#define		PMCR_MAC_MODE		(1u<<16)
#define		PMCR_IPG_CFG_RND	(1u<<18)
#define         MT7531_PMCR_FORCE_TX_FC	(1u<<27)
#define         MT7531_PMCR_FORCE_RX_FC	(1u<<28)
#define         MT7531_PMCR_FORCE_DPX	(1u<<29)
#define         MT7531_PMCR_FORCE_SPX	(1u<<30)
#define         MT7531_PMCR_FORCE_LINK	(1u<<31)
#define		MT7531_PMCR_FORCE_MODE	(MT7531_PMCR_FORCE_TX_FC | \
		   MT7531_PMCR_FORCE_RX_FC | MT7531_PMCR_FORCE_DPX | \
		   MT7531_PMCR_FORCE_SPX | MT7531_PMCR_FORCE_LINK)
#define		PMCR_CFG_DEFAULT	(PMCR_BACKPR_EN | PMCR_BKOFF_EN | \
		   PMCR_MAC_RX_EN | PMCR_MAC_TX_EN | PMCR_IPG_CFG_RND |  \
		   PMCR_FORCE_RX_FC | PMCR_FORCE_TX_FC)

#define	MTKSWITCH_PMSR(x)	MTKSWITCH_PORTREG(0x3008, (x))
#define		PMSR_MAC_LINK_STS	(1u<<0)
#define		PMSR_MAC_DPX_STS	(1u<<1)
#define		PMSR_MAC_SPD_STS	(3u<<2)
#define		PMSR_MAC_SPD(x)		(((x)>>2) & 0x3)
#define		PMSR_MAC_SPD_10		0
#define		PMSR_MAC_SPD_100	1
#define		PMSR_MAC_SPD_1000	2
#define		PMSR_MAC_SPD_2500	3	/* ports 5 and 6 only */
#define		PMSR_TX_FC_STS		(1u<<4)
#define		PMSR_RX_FC_STS		(1u<<5)
#define	MTKSWITCH_REG_ADDR(r)	(((r) >> 6) & 0x3ff)
#define	MTKSWITCH_REG_LO(r)	(((r) >> 2) & 0xf)
#define	MTKSWITCH_REG_HI(r)	(1 << 4)
#define MTKSWITCH_VAL_LO(v)	((v) & 0xffff)
#define MTKSWITCH_VAL_HI(v)	(((v) >> 16) & 0xffff)
#define MTKSWITCH_GLOBAL_PHY	31
#define	MTKSWITCH_GLOBAL_REG	31

#define	MTKSWITCH_STRAP		0x7800
#define	MTKSWITCH_SWSTRAP	0x7804
#define		STRAP_TM_STRAP		(1u << 0)
#define		STRAP_EEPROM_MODE	(1u << 1)
#define		STRAP_PWR_ON_LIGHT	(1u << 2)
#define		STRAP_PWR_ON_PLL	(1u << 3)
#define		STRAP_EEE		(1u << 4)
#define		STRAP_AUTO_EEPROM	(1u << 5)
#define		STRAP_PHY		(1u << 6)
#define		STRAP_XTAL		(1u << 7)	/* 1 25MHz 0 40MHz */

#define	MT7531_TOP_SIG_SR		0x780c
#define		PAD_DUAL_SGMII			(1u << 1)
#define		PAD_MCM_SMI			(1u << 0)

#define	MTKSWITCH_CREV			0x781C
#define		CREV_CHIP_NAME(x)	(((x) >> 16) & 0xFFFF)
#define		CREV_CHIP_REV(x)	((x) & 0xF)
#define		MT7531_CHIP_ID		0x7531

#define	MT7531_PLLGP			0x7820
#define		PLLGP_COREPLL			(1u << 2)
#define		PLLGP_SW_CLKSW			(1u << 1)
#define		PLLGP_SW_PLLGP			(1u << 0)

#define	MT7531_PLLGP_CR0		0x78a8
#define		PLLGP_RG_COREPLL		(1u << 22)
#define		PLLGP_RG_COREPLL_POSDIV_S	23
#define		PLLGP_RG_COREPLL_POSDIV_M	0x3800000
#define		PLLGP_RG_COREPLL_SDM_PCW_S	1
#define		PLLGP_RG_COREPLL_SDM_PCW_M	0x3ffffe
#define 		MT7531_XTAL_25MHZ	0x140000
#define 		MT7531_XTAL_40MHZ	0x190000
#define		PLLGP_RG_COREPLL_SDM_PCW_CHG	(1u << 0)

#define	MT7531_TSRA1	0x84
#define	MT7531_TSRA2	0x88
#define	MT7531_TSRD	0x8C

/* RGMII and SGMII PLL clock */
#define MT7531_ANA_PLLGP_CR2		0x78b0
#define MT7531_ANA_PLLGP_CR5		0x78bc


#endif	/* __MTKSWITCH_MT7531_H__ */

