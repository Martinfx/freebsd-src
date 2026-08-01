/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Register, descriptor and firmware layout for the MediaTek MT7615E.
 *
 * Derived from the register definitions of the Linux mt76 driver
 * (sys/contrib/dev/mediatek/mt76/mt7615), with the MT7615E register
 * bases already folded in: mt76 keeps a per-chip base table and adds an
 * offset at every access, but this driver only ever drives the PCIe
 * MT7615E, so the sums are constant and are written out here.
 */

#ifndef _IF_MT7615REG_H_
#define	_IF_MT7615REG_H_

#define	MT7615_VENDOR_MEDIATEK		0x14c3
#define	MT7615_DEVICE_MT7615E		0x7615

/*
 * Registers below MT7615_DIRECT_LIMIT are addressed straight in BAR0.
 * Anything above has to be brought into view first: the upper bits of
 * the address are written to MT_MCU_PCIE_REMAP_2 and the access is
 * made at MT_PCIE_REMAP_BASE_2 plus the offset inside the window.
 */
#define	MT7615_DIRECT_LIMIT		0x100000

#define	MT_MCU_PCIE_REMAP_2		0x02504
#define	 MT_MCU_PCIE_REMAP_2_BASE	0xfff80000	/* bits 31:19 */
#define	 MT_MCU_PCIE_REMAP_2_OFFSET	0x0007ffff	/* bits 18:0 */
#define	MT_PCIE_REMAP_BASE_2		0x80000

/*
 * Chip identity and firmware state.  MT_TOP_MISC2 reports how far the
 * MCU has come up; the driver waits on it around the download.
 */
#define	MT_HW_REV			0x1000
#define	MT_HW_CHIPID			0x1008

#define	MT_TOP_MISC2			0x1134
#define	 MT_TOP_MISC2_FW_STATE		0x00000007	/* bits 2:0 */

#define	MT_FW_STATE_FW_DOWNLOAD		1
#define	MT_FW_STATE_NORMAL_OPERATION	5
#define	MT_FW_STATE_RDY			7

/*
 * Host interface (HIF) block: the DMA engine, its rings and the
 * interrupt registers.  MT7615E puts the block at 0x4000.
 */
#define	MT_HIF_BASE			0x04000
#define	MT_HIF(_o)			(MT_HIF_BASE + (_o))

#define	MT_HIF_RST			MT_HIF(0x100)
#define	 MT_HIF_LOGIC_RST_N		(1U << 4)

#define	MT_PDMA_BUSY_STATUS		MT_HIF(0x168)

/* Ownership handshake: the MCU parks the chip in firmware-own on idle. */
#define	MT_CFG_LPCR_HOST		MT_HIF(0x1f0)
#define	 MT_CFG_LPCR_HOST_FW_OWN	(1U << 0)
#define	 MT_CFG_LPCR_HOST_DRV_OWN	(1U << 1)

#define	MT_MCU_INT_EVENT		MT_HIF(0x1f8)
#define	 MT_MCU_INT_EVENT_PDMA_STOPPED	(1U << 0)
#define	 MT_MCU_INT_EVENT_PDMA_INIT	(1U << 1)
#define	 MT_MCU_INT_EVENT_SER_TRIGGER	(1U << 2)
#define	 MT_MCU_INT_EVENT_RESET_DONE	(1U << 3)

#define	MT_INT_SOURCE_CSR		MT_HIF(0x200)
#define	MT_INT_MASK_CSR			MT_HIF(0x204)

#define	 MT_INT_RX_DONE(_n)		(1U << (_n))
#define	 MT_INT_RX_DONE_ALL		0x00000003	/* bits 1:0 */
#define	 MT_INT_TX_DONE(_n)		(1U << ((_n) + 4))
#define	 MT_INT_TX_DONE_ALL		0x000ffff0	/* bits 19:4 */
#define	 MT_INT_MCU_CMD			(1U << 30)

#define	MT_WPDMA_GLO_CFG		MT_HIF(0x208)
#define	 MT_WPDMA_GLO_CFG_TX_DMA_EN	(1U << 0)
#define	 MT_WPDMA_GLO_CFG_TX_DMA_BUSY	(1U << 1)
#define	 MT_WPDMA_GLO_CFG_RX_DMA_EN	(1U << 2)
#define	 MT_WPDMA_GLO_CFG_RX_DMA_BUSY	(1U << 3)
#define	 MT_WPDMA_GLO_CFG_DMA_BURST_SIZE 0x00000030	/* bits 5:4 */
#define	 MT_WPDMA_GLO_CFG_DMA_BURST_SIZE_S 4
#define	 MT_WPDMA_GLO_CFG_TX_WRITEBACK_DONE (1U << 6)
#define	 MT_WPDMA_GLO_CFG_BIG_ENDIAN	(1U << 7)
#define	 MT_WPDMA_GLO_CFG_TX_BT_SIZE_BIT0 (1U << 9)
#define	 MT_WPDMA_GLO_CFG_MULTI_DMA_EN	0x00000c00	/* bits 11:10 */
#define	 MT_WPDMA_GLO_CFG_MULTI_DMA_EN_S 10
#define	 MT_WPDMA_GLO_CFG_FIFO_LITTLE_ENDIAN (1U << 12)
#define	 MT_WPDMA_GLO_CFG_TX_BT_SIZE_BIT21 0x00c00000	/* bits 23:22 */
#define	 MT_WPDMA_GLO_CFG_TX_BT_SIZE_BIT21_S 22
#define	 MT_WPDMA_GLO_CFG_SW_RESET	(1U << 24)
#define	 MT_WPDMA_GLO_CFG_FIRST_TOKEN_ONLY (1U << 26)
#define	 MT_WPDMA_GLO_CFG_OMIT_TX_INFO	(1U << 28)

#define	MT_WPDMA_RST_IDX		MT_HIF(0x20c)
#define	MT_DELAY_INT_CFG		MT_HIF(0x210)

#define	MT_MCU_CMD			MT_HIF(0x234)
#define	 MT_MCU_CMD_STOP_PDMA_FW_RELOAD	(1U << 1)
#define	 MT_MCU_CMD_STOP_PDMA		(1U << 2)
#define	 MT_MCU_CMD_RESET_DONE		(1U << 3)
#define	 MT_MCU_CMD_RECOVERY_DONE	(1U << 4)
#define	 MT_MCU_CMD_NORMAL_STATE	(1U << 5)
#define	 MT_MCU_CMD_ERROR_MASK		(0x0000003e | 0x1f000000)

/*
 * Ring control registers.  Each ring owns four words: descriptor base,
 * ring size, the CPU index the driver advances and the DMA index the
 * engine advances.
 */
#define	MT_TX_RING_BASE			MT_HIF(0x300)
#define	MT_RX_RING_BASE			MT_HIF(0x400)
#define	MT_RING_SIZE			0x10

#define	MT_RING_DESC_BASE		0x00
#define	MT_RING_COUNT			0x04
#define	MT_RING_CPU_IDX			0x08
#define	MT_RING_DMA_IDX			0x0c

#define	MT_WPDMA_GLO_CFG1		MT_HIF(0x500)
#define	MT_WPDMA_TX_PRE_CFG		MT_HIF(0x510)
#define	MT_WPDMA_RX_PRE_CFG		MT_HIF(0x520)
#define	MT_WPDMA_ABT_CFG		MT_HIF(0x530)
#define	MT_WPDMA_ABT_CFG1		MT_HIF(0x534)

/*
 * MAC blocks.  All of these sit below MT7615_DIRECT_LIMIT, so they need
 * no remapping.
 */
#define	MT_WF_CFG_BASE			0x20200
#define	MT_WF_CFG(_o)			(MT_WF_CFG_BASE + (_o))
#define	MT_CFG_CCR			MT_WF_CFG(0x000)
#define	 MT_CFG_CCR_MAC_D1_1X_GC_EN	(1U << 24)
#define	 MT_CFG_CCR_MAC_D0_1X_GC_EN	(1U << 25)
#define	 MT_CFG_CCR_MAC_D1_2X_GC_EN	(1U << 30)
#define	 MT_CFG_CCR_MAC_D0_2X_GC_EN	(1U << 31)

#define	MT_WF_AGG_BASE			0x20a00
#define	MT_WF_AGG(_o)			(MT_WF_AGG_BASE + (_o))
#define	MT_AGG_ARCR			MT_WF_AGG(0x010)
#define	 MT_AGG_ARCR_RTS_RATE_THR	0x00001f00	/* bits 12:8 */
#define	 MT_AGG_ARCR_RTS_RATE_THR_S	8
#define	 MT_AGG_ARCR_RATE_DOWN_RATIO	0x00030000	/* bits 17:16 */
#define	 MT_AGG_ARCR_RATE_DOWN_RATIO_S	16
#define	 MT_AGG_ARCR_RATE_DOWN_RATIO_EN	(1U << 19)
#define	 MT_AGG_ARCR_RATE_UP_EXTRA_TH	0x00700000	/* bits 22:20 */
#define	 MT_AGG_ARCR_RATE_UP_EXTRA_TH_S	20
#define	MT_AGG_ARUCR(_b)		MT_WF_AGG(0x018 + (_b) * 0x100)
#define	MT_AGG_ARDCR(_b)		MT_WF_AGG(0x01c + (_b) * 0x100)
#define	MT_AGG_ARxCR_LIMIT_SHIFT(_n)	(4 * (_n))
#define	MT_AGG_ARxCR_LIMIT(_n, _v)	\
	(((_v) & 0x7) << MT_AGG_ARxCR_LIMIT_SHIFT(_n))
#define	MT_AGG_ACR(_b)			MT_WF_AGG(0x070 + (_b) * 0x100)
#define	 MT_AGG_ACR_NO_BA_AR_RULE	(1U << 1)
#define	 MT_AGG_ACR_PKT_TIME_EN		(1U << 2)
#define	 MT_AGG_ACR_CFEND_RATE		0x0000fff0	/* bits 15:4 */
#define	 MT_AGG_ACR_CFEND_RATE_S	4
#define	 MT_AGG_ACR_BAR_RATE		0xfff00000	/* bits 31:20 */
#define	 MT_AGG_ACR_BAR_RATE_S		20
#define	MT_AGG_SCR			MT_WF_AGG(0x0fc)
#define	 MT_AGG_SCR_NLNAV_MID_PTEC_DIS	(1U << 3)

#define	MT_WF_TMAC_BASE			0x21000
#define	MT_WF_TMAC(_o)			(MT_WF_TMAC_BASE + (_o))
#define	MT_TMAC_TRCR(_b)		MT_WF_TMAC((_b) ? 0x070 : 0x09c)
#define	 MT_TMAC_TRCR_CCA_SEL		0xc0000000	/* bits 31:30 */
#define	 MT_TMAC_TRCR_CCA_SEL_S		30
#define	 MT_TMAC_TRCR_SEC_CCA_SEL	0x30000000	/* bits 29:28 */
#define	 MT_TMAC_TRCR_SEC_CCA_SEL_S	28
/*
 * How long the MAC waits, and for what.  CDTR and ODTR are the CCK and
 * OFDM detection timeouts; ICR holds the interframe spaces and the slot
 * time.  None of this is set up by the firmware to suit the band, and
 * the channel switch command puts it back to defaults, so it has to be
 * written after every move.
 */
#define	MT_TMAC_CDTR			MT_WF_TMAC(0x090)
#define	MT_TMAC_ODTR			MT_WF_TMAC(0x094)
#define	 MT_TIMEOUT_VAL_PLCP		0x0000ffff
#define	 MT_TIMEOUT_VAL_PLCP_S		0
#define	 MT_TIMEOUT_VAL_CCA		0xffff0000
#define	 MT_TIMEOUT_VAL_CCA_S		16
#define	MT_TMAC_ICR(_b)			MT_WF_TMAC((_b) ? 0x074 : 0x0a4)
#define	 MT_IFS_EIFS			0x000001ff	/* bits 8:0 */
#define	 MT_IFS_EIFS_S			0
#define	 MT_IFS_RIFS			0x00007c00	/* bits 14:10 */
#define	 MT_IFS_RIFS_S			10
#define	 MT_IFS_SIFS			0x007f0000	/* bits 22:16 */
#define	 MT_IFS_SIFS_S			16
#define	 MT_IFS_SLOT			0x7f000000	/* bits 30:24 */
#define	 MT_IFS_SLOT_S			24

/* The arbiter, whose transmit and receive halves are stopped while
 * the timing underneath them is rewritten.
 */
#define	MT_WF_ARB_BASE			0x20c00
#define	MT_WF_ARB(_o)			(MT_WF_ARB_BASE + (_o))
#define	MT_ARB_SCR			MT_WF_ARB(0x080)
#define	 MT_ARB_SCR_TX0_DISABLE		(1U << 8)
#define	 MT_ARB_SCR_RX0_DISABLE		(1U << 9)
#define	 MT_ARB_SCR_TX1_DISABLE		(1U << 10)
#define	 MT_ARB_SCR_RX1_DISABLE		(1U << 11)

#define	MT_TMAC_CTCR0			MT_WF_TMAC(0x0f4)
#define	 MT_TMAC_CTCR0_INS_DDLMT_REFTIME 0x0000003f	/* bits 5:0 */
#define	 MT_TMAC_CTCR0_INS_DDLMT_REFTIME_S 0
#define	 MT_TMAC_CTCR0_INS_DDLMT_DENSITY 0x0000f000	/* bits 15:12 */
#define	 MT_TMAC_CTCR0_INS_DDLMT_DENSITY_S 12
#define	 MT_TMAC_CTCR0_INS_DDLMT_EN	(1U << 17)
#define	 MT_TMAC_CTCR0_INS_DDLMT_VHT_SMPDU_EN (1U << 18)

/*
 * Station table.  Three blocks: the entries themselves, and two
 * register windows through which the hardware's rate adaptation is
 * driven and told which entry to apply itself to.
 */
#define	MT_WTBL_ENTRY_BASE		0x30000
#define	MT_WTBL_ENTRY_SIZE		256
#define	MT_WTBL_ENTRY(_wcid)		\
	(MT_WTBL_ENTRY_BASE + (_wcid) * MT_WTBL_ENTRY_SIZE)

#define	MT_WTBL_ON_BASE			0x23000
#define	MT_WTBL_ON(_o)			(MT_WTBL_ON_BASE + (_o))
#define	MT_WTBL_OFF_BASE		0x23400
#define	MT_WTBL_OFF(_o)			(MT_WTBL_OFF_BASE + (_o))

#define	MT_WTBL_UPDATE			MT_WTBL_OFF(0x030)
#define	 MT_WTBL_UPDATE_WLAN_IDX	0x000000ff
#define	 MT_WTBL_UPDATE_RXINFO_UPDATE	(1U << 11)
#define	 MT_WTBL_UPDATE_RATE_UPDATE	(1U << 13)
#define	 MT_WTBL_UPDATE_TX_COUNT_CLEAR	(1U << 14)
#define	 MT_WTBL_UPDATE_BUSY		(1U << 31)

/*
 * The rate ladder.  Eight slots, packed across three registers and
 * split across register boundaries in the middle; the hardware works
 * its way down them as a frame fails.
 */
#define	MT_WTBL_RIUCR0			MT_WTBL_ON(0x020)
#define	MT_WTBL_RIUCR1			MT_WTBL_ON(0x024)
#define	 MT_WTBL_RIUCR1_RATE0		0x00000fff
#define	 MT_WTBL_RIUCR1_RATE0_S		0
#define	 MT_WTBL_RIUCR1_RATE1		0x00fff000
#define	 MT_WTBL_RIUCR1_RATE1_S		12
#define	 MT_WTBL_RIUCR1_RATE2_LO	0xff000000
#define	 MT_WTBL_RIUCR1_RATE2_LO_S	24
#define	MT_WTBL_RIUCR2			MT_WTBL_ON(0x028)
#define	 MT_WTBL_RIUCR2_RATE2_HI	0x0000000f
#define	 MT_WTBL_RIUCR2_RATE2_HI_S	0
#define	 MT_WTBL_RIUCR2_RATE3		0x0000fff0
#define	 MT_WTBL_RIUCR2_RATE3_S		4
#define	 MT_WTBL_RIUCR2_RATE4		0x0fff0000
#define	 MT_WTBL_RIUCR2_RATE4_S		16
#define	 MT_WTBL_RIUCR2_RATE5_LO	0xf0000000
#define	 MT_WTBL_RIUCR2_RATE5_LO_S	28
#define	MT_WTBL_RIUCR3			MT_WTBL_ON(0x02c)
#define	 MT_WTBL_RIUCR3_RATE5_HI	0x000000ff
#define	 MT_WTBL_RIUCR3_RATE5_HI_S	0
#define	 MT_WTBL_RIUCR3_RATE6		0x000fff00
#define	 MT_WTBL_RIUCR3_RATE6_S		8
#define	 MT_WTBL_RIUCR3_RATE7		0xfff00000
#define	 MT_WTBL_RIUCR3_RATE7_S		20

/* Words of an entry the rate ladder is written alongside. */
/*
 * Where a peer's key lives in its table entry, and what says it is
 * there.  The key itself is eight words starting at word thirty; the
 * two words at the front carry the index it was installed under and
 * whether the receiver may use it, and they are written through the
 * RICR pair rather than directly.
 */
#define	MT_WTBL_RICR0			MT_WTBL_ON(0x010)
#define	MT_WTBL_RICR1			MT_WTBL_ON(0x014)

#define	MT_WTBL_W0			(0 * 4)
#define	 MT_WTBL_W0_KEY_IDX		0x01800000	/* bits 24:23 */
#define	 MT_WTBL_W0_KEY_IDX_S		23
#define	 MT_WTBL_W0_RX_KEY_VALID	(1U << 26)
#define	 MT_WTBL_W0_RX_IK_VALID		(1U << 27)
#define	MT_WTBL_W1			(1 * 4)
#define	MT_WTBL_W2			(2 * 4)
#define	 MT_WTBL_W2_KEY_TYPE		0x000000f0	/* bits 7:4 */
#define	 MT_WTBL_W2_KEY_TYPE_S		4
#define	MT_WTBL_W30			(30 * 4)
#define	MT_WTBL_KEY_SIZE		32

/* What the hardware calls each cipher. */
#define	MT_CIPHER_NONE			0
#define	MT_CIPHER_WEP40			1
#define	MT_CIPHER_TKIP			2
#define	MT_CIPHER_TKIP_NO_MIC		3
#define	MT_CIPHER_AES_CCMP		4
#define	MT_CIPHER_WEP104		5
#define	MT_CIPHER_BIP_CMAC_128		6
#define	MT_CIPHER_WEP128		7

#define	MT_WTBL_W5			(5 * 4)
#define	 MT_WTBL_W5_CHANGE_BW_RATE	0x000000e0	/* bits 7:5 */
#define	 MT_WTBL_W5_CHANGE_BW_RATE_S	5
#define	 MT_WTBL_W5_SHORT_GI_20		(1U << 8)
#define	 MT_WTBL_W5_SHORT_GI_40		(1U << 9)
#define	 MT_WTBL_W5_SHORT_GI_80		(1U << 10)
#define	 MT_WTBL_W5_BW_CAP		0x00003000	/* bits 13:12 */
#define	 MT_WTBL_W5_BW_CAP_S		12
#define	 MT_WTBL_W5_MPDU_FAIL_COUNT	0x03800000	/* bits 25:23 */
#define	 MT_WTBL_W5_MPDU_OK_COUNT	0x1c000000	/* bits 28:26 */
#define	 MT_WTBL_W5_RATE_IDX		0xe0000000	/* bits 31:29 */
#define	MT_WTBL_W27			(27 * 4)
#define	 MT_WTBL_W27_CC_BW_SEL		0x00000060	/* bits 6:5 */
#define	 MT_WTBL_W27_CC_BW_SEL_S	5

#define	MT_WF_RMAC_BASE			0x21200
#define	MT_WF_RMAC(_o)			(MT_WF_RMAC_BASE + (_o))
#define	MT_WF_RFCR(_b)			MT_WF_RMAC((_b) ? 0x100 : 0x000)
#define	 MT_WF_RFCR_DROP_STBC_MULTI	(1U << 0)
#define	 MT_WF_RFCR_DROP_FCSFAIL	(1U << 1)
#define	 MT_WF_RFCR_DROP_VERSION	(1U << 3)
#define	 MT_WF_RFCR_DROP_PROBEREQ	(1U << 4)
#define	 MT_WF_RFCR_DROP_MCAST		(1U << 5)
#define	 MT_WF_RFCR_DROP_BCAST		(1U << 6)
#define	 MT_WF_RFCR_DROP_MCAST_FILTERED	(1U << 7)
#define	 MT_WF_RFCR_DROP_A3_MAC		(1U << 8)
#define	 MT_WF_RFCR_DROP_A3_BSSID	(1U << 9)
#define	 MT_WF_RFCR_DROP_A2_BSSID	(1U << 10)
#define	 MT_WF_RFCR_DROP_OTHER_BEACON	(1U << 11)
#define	 MT_WF_RFCR_DROP_FRAME_REPORT	(1U << 12)
#define	 MT_WF_RFCR_DROP_CTL_RSV	(1U << 13)
#define	 MT_WF_RFCR_DROP_CTS		(1U << 14)
#define	 MT_WF_RFCR_DROP_RTS		(1U << 15)
#define	 MT_WF_RFCR_DROP_DUPLICATE	(1U << 16)
#define	 MT_WF_RFCR_DROP_OTHER_BSS	(1U << 17)
#define	 MT_WF_RFCR_DROP_OTHER_UC	(1U << 18)
#define	 MT_WF_RFCR_DROP_OTHER_TIM	(1U << 19)
#define	 MT_WF_RFCR_DROP_NDPA		(1U << 20)
#define	 MT_WF_RFCR_DROP_UNWANTED_CTL	(1U << 21)
#define	MT_WF_RFCR1(_b)			MT_WF_RMAC((_b) ? 0x104 : 0x004)
#define	 MT_WF_RFCR1_DROP_ACK		(1U << 4)
#define	 MT_WF_RFCR1_DROP_BF_POLL	(1U << 5)
#define	 MT_WF_RFCR1_DROP_BA		(1U << 6)
#define	 MT_WF_RFCR1_DROP_CFEND		(1U << 7)
#define	 MT_WF_RFCR1_DROP_CFACK		(1U << 8)
#define	MT_WF_RMAC_MORE(_b)		MT_WF_RMAC((_b) ? 0x124 : 0x024)
#define	 MT_WF_RMAC_MORE_MUAR_MODE	0xc0000000	/* bits 31:30 */
#define	MT_CHFREQ(_b)			MT_WF_RMAC((_b) ? 0x130 : 0x030)
#define	MT_WF_RMAC_MIB_AIRTIME0		MT_WF_RMAC(0x0380)
#define	MT_WF_RMAC_MIB_TIME0		MT_WF_RMAC(0x03c4)
#define	 MT_WF_RMAC_MIB_RXTIME_EN	(1U << 30)
#define	 MT_WF_RMAC_MIB_RXTIME_CLR	(1U << 31)

#define	MT_WF_DMA_BASE			0x21800
#define	MT_WF_DMA(_o)			(MT_WF_DMA_BASE + (_o))
#define	MT_DMA_DCR0			MT_WF_DMA(0x000)
#define	 MT_DMA_DCR0_MAX_RX_LEN		0x0000fffc	/* bits 15:2 */
#define	 MT_DMA_DCR0_MAX_RX_LEN_S	2
#define	 MT_DMA_DCR0_DAMSDU_EN		(1U << 16)
#define	 MT_DMA_DCR0_RX_VEC_DROP	(1U << 17)
#define	 MT_DMA_DCR0_RX_HDR_TRANS_EN	(1U << 19)
#define	MT_DMA_RCFR0(_b)		MT_WF_DMA(0x070 + (_b) * 0x40)
#define	 MT_DMA_RCFR0_MCU_RX_MGMT	(1U << 2)
#define	 MT_DMA_RCFR0_MCU_RX_CTL_NON_BAR (1U << 3)
#define	 MT_DMA_RCFR0_MCU_RX_CTL_BAR	(1U << 4)
#define	 MT_DMA_RCFR0_MCU_RX_TDLS	(1U << 19)
#define	 MT_DMA_RCFR0_MCU_RX_BYPASS	(1U << 21)
#define	 MT_DMA_RCFR0_RX_DROPPED_UCAST	0x03000000	/* bits 25:24 */
#define	 MT_DMA_RCFR0_RX_DROPPED_UCAST_S 24
#define	 MT_DMA_RCFR0_RX_DROPPED_MCAST	0x0c000000	/* bits 27:26 */
#define	 MT_DMA_RCFR0_RX_DROPPED_MCAST_S 26

#define	MT_WF_PF_BASE			0x22000
#define	MT_WF_PFCR			(MT_WF_PF_BASE + 0x000)
#define	 MT_WF_PFCR_TDLS_EN		(1U << 9)

#define	MT_WF_MIB_BASE			0x24800
#define	MT_WF_MIB_SCR0			MT_WF_MIB_BASE

/*
 * Transmit counters.  MT_MIB_SDR14 and SDR15 are how many frames went
 * out inside aggregates and how many of those were acknowledged, which
 * together give the delivery rate.  MT_TX_AGG_CNT is a histogram of how
 * many frames each aggregate held, two sixteen-bit buckets to a
 * register; it only counts once MT_MIB_SCR0_AGG_CNT_RANGE_EN is set.
 */
#define	MT_MIB_SDR14			(MT_WF_MIB_BASE + 0x040)
#define	 MT_MIB_AMPDU_MPDU_COUNT	0x00ffffff
#define	MT_MIB_SDR15			(MT_WF_MIB_BASE + 0x044)
#define	 MT_MIB_AMPDU_ACK_COUNT		0x00ffffff
#define	MT_TX_AGG_CNT(_n)		(MT_WF_MIB_BASE + 0x0a8 + ((_n) << 2))
#define	MT7615_AGG_CNT_REGS		4
#define	MT7615_AGG_BUCKETS		(MT7615_AGG_CNT_REGS * 2)
#define	 MT_MIB_SCR0_AGG_CNT_RANGE_EN	(1U << 21)

/* Behind the remap window. */
#define	MT_WF_PHY_BASE			0x82070000
#define	MT_WF_PHY_WF2_RFCTRL0(_n)	(MT_WF_PHY_BASE + 0x1900 + (_n) * 0x400)
#define	 MT_WF_PHY_WF2_RFCTRL0_LPBCN_EN	(1U << 9)

#define	MT_EFUSE_BASE			0x81070000
#define	MT_EFUSE_BASE_CTRL		0x000
#define	 MT_EFUSE_BASE_CTRL_EMPTY	(1U << 30)
#define	MT_EFUSE_CTRL			0x008
#define	 MT_EFUSE_CTRL_AOUT		0x0000003f	/* bits 5:0 */
#define	 MT_EFUSE_CTRL_MODE		0x000000c0	/* bits 7:6 */
#define	 MT_EFUSE_CTRL_AIN		0x03ff0000	/* bits 25:16 */
#define	 MT_EFUSE_CTRL_AIN_SHIFT	16
#define	 MT_EFUSE_CTRL_VALID		(1U << 29)
#define	 MT_EFUSE_CTRL_KICK		(1U << 30)
#define	MT_EFUSE_RDATA(_i)		(0x030 + ((_i) * 4))

#define	MT7615_EFUSE_BLOCK		16
#define	MT_EE_MAC_ADDR			0x004

/*
 * How many chains the board actually wired up.  A card with one antenna
 * and one with four run the same firmware and answer the same way, so
 * the only place the difference is written down is here.
 */
#define	MT_EE_NIC_CONF_0		0x034
#define	 MT_EE_NIC_CONF_TX_MASK		0xf0
#define	 MT_EE_NIC_CONF_TX_MASK_S	4
#define	 MT_EE_NIC_CONF_RX_MASK		0x0f
#define	MT7615_MAX_CHAINS		4

/*
 * DMA descriptors.  One descriptor addresses up to two buffers; the
 * lengths share the control word and the last one in a frame is marked
 * with LAST_SEC.  The engine sets DMA_DONE when it is finished with a
 * descriptor, which is how both directions detect completion.
 */
struct mt7615_desc {
    uint32_t	buf0;
    uint32_t	ctrl;
    uint32_t	buf1;
    uint32_t	info;
} __packed __aligned(4);

#define	MT_DMA_CTL_SD_LEN1		0x00003fff	/* bits 13:0 */
#define	MT_DMA_CTL_SD_LEN1_S		0
#define	MT_DMA_CTL_LAST_SEC1		(1U << 14)
#define	MT_DMA_CTL_BURST		(1U << 15)
#define	MT_DMA_CTL_SD_LEN0		0x3fff0000	/* bits 29:16 */
#define	MT_DMA_CTL_SD_LEN0_S		16
#define	MT_DMA_CTL_LAST_SEC0		(1U << 30)
#define	MT_DMA_CTL_DMA_DONE		(1U << 31)

/*
 * Tx descriptor (TXD) as the MT7615 LMAC wants it.  Only the first two
 * words matter for MCU traffic; the rest carry the rate and sequence
 * control used for data frames.
 */
#define	MT_TXD_SIZE			(8 * sizeof(uint32_t))

/*
 * DW0 bits 31:26 hold the port and queue index as one value; mt76
 * writes them as two fields, but every constant it uses is already the
 * combined form, so keep them combined here.
 */
#define	MT_TXD0_PQ_IDX			0xfc000000	/* bits 31:26 */
#define	MT_TXD0_PQ_IDX_S		26
#define	MT_TXD0_TX_BYTES		0x0000ffff	/* bits 15:0 */

#define	MT_TX_PQ_MCU_PORT_RX_Q0		0x20
#define	MT_TX_PQ_MCU_PORT_RX_FWDL	0x3e

/*
 * The MCU header keeps the port and queue apart again; the queue is
 * the low five bits of the combined value above.
 */
#define	MT_TX_PORT_IDX_LMAC		0
#define	MT_TX_PORT_IDX_MCU		1

#define	MT_TXD1_OWN_MAC			0xfc000000	/* bits 31:26 */
#define	MT_TXD1_OWN_MAC_S		26
#define	MT_TXD1_PKT_FMT			0x03000000	/* bits 25:24 */
#define	MT_TXD1_PKT_FMT_S		24
#define	MT_TXD1_TID			0x00e00000	/* bits 23:21 */
#define	MT_TXD1_TID_S			21
#define	MT_TXD1_HDR_PAD			0x00060000	/* bits 18:17 */
#define	MT_TXD1_HDR_PAD_S		17
#define	MT_TXD1_LONG_FORMAT		(1U << 15)
#define	MT_TXD1_HDR_FORMAT		0x00006000	/* bits 14:13 */
#define	MT_TXD1_HDR_FORMAT_S		13
#define	MT_TXD1_HDR_INFO		0x00001f00	/* bits 12:8 */
#define	MT_TXD1_HDR_INFO_S		8
#define	MT_TXD1_WLAN_IDX		0x000000ff	/* bits 7:0 */

#define	MT_TXD2_FIX_RATE		(1U << 31)
#define	MT_TXD2_BA_DISABLE		(1U << 29)
#define	MT_TXD2_MULTICAST		(1U << 10)
#define	MT_TXD2_FRAME_TYPE		0x00000030	/* bits 5:4 */
#define	MT_TXD2_FRAME_TYPE_S		4
#define	MT_TXD2_SUB_TYPE		0x0000000f	/* bits 3:0 */

#define	MT_TXD3_SN_VALID		(1U << 31)
#define	MT_TXD3_SEQ			0x0fff0000	/* bits 27:16 */
#define	MT_TXD3_SEQ_S			16
#define	MT_TXD3_REM_TX_COUNT		0x0000f800	/* bits 15:11 */
#define	MT_TXD3_REM_TX_COUNT_S		11
#define	MT_TXD3_TX_COUNT		0x000007c0	/* bits 10:6 */
#define	MT_TXD3_TX_COUNT_S		6
#define	MT_TXD3_PROTECT_FRAME		(1U << 1)
#define	MT_TXD3_NO_ACK			(1U << 0)

#define	MT_TXD5_TX_STATUS_HOST		(1U << 10)
#define	MT_TXD5_PID			0x000000ff	/* bits 7:0 */

#define	MT_TXD6_FIXED_RATE		(1U << 31)
#define	MT_TXD6_SGI			(1U << 30)
#define	MT_TXD6_LDPC			(1U << 29)
#define	MT_TXD6_TX_BF			(1U << 28)
#define	MT_TXD6_TX_RATE			0x0fff0000	/* bits 27:16 */
#define	MT_TXD6_TX_RATE_S		16
#define	MT_TXD6_ANT_ID			0x0000fff0	/* bits 15:4 */
#define	MT_TXD6_ANT_ID_S		4
#define	MT_TXD6_DYN_BW			(1U << 3)
#define	MT_TXD6_FIXED_BW		(1U << 2)
#define	MT_TXD6_BW			0x00000003	/* bits 1:0 */

/*
 * How a rate is named to the hardware: which modulation, how many
 * spatial streams, and the index within that modulation.
 */
#define	MT_TX_RATE_STBC			(1U << 11)
#define	MT_TX_RATE_NSS			0x00000600	/* bits 10:9 */
#define	MT_TX_RATE_NSS_S		9
#define	MT_TX_RATE_MODE			0x000001c0	/* bits 8:6 */
#define	MT_TX_RATE_MODE_S		6
#define	MT_TX_RATE_IDX			0x0000003f	/* bits 5:0 */

#define	MT_PHY_TYPE_CCK			0
#define	MT_PHY_TYPE_OFDM		1
#define	MT_PHY_TYPE_HT			2
#define	MT_PHY_TYPE_HT_GF		3
#define	MT_PHY_TYPE_VHT			4

/*
 * Transmit status.
 *
 * The firmware reports on a frame only when the descriptor asked it to,
 * and the report says which rate the frame finally went out at and how
 * many attempts it took.  That is the only account of what the hardware
 * is doing with the rate ladder: the ladder is programmed once per peer
 * and the hardware walks it on its own, so without these reports the
 * driver cannot tell a peer sitting at the top from one that has fallen
 * to the bottom.
 *
 * Reports arrive on the MCU ring, packed several to an event, seven
 * words apiece behind the single word of receive descriptor.
 */
#define	MT7615_TXS_WORDS		7
#define	MT7615_TXS_SIZE			(MT7615_TXS_WORDS * 4)

#define	MT_TXS0_PID			0xff000000	/* bits 31:24 */
#define	MT_TXS0_PID_S			24
#define	MT_TXS0_BA_ERROR		(1U << 22)
#define	MT_TXS0_PS_FLAG			(1U << 21)
#define	MT_TXS0_TXOP_TIMEOUT		(1U << 20)
#define	MT_TXS0_BIP_ERROR		(1U << 19)
#define	MT_TXS0_QUEUE_TIMEOUT		(1U << 18)
#define	MT_TXS0_RTS_TIMEOUT		(1U << 17)
#define	MT_TXS0_ACK_TIMEOUT		(1U << 16)
#define	MT_TXS0_TX_STATUS_HOST		(1U << 15)
#define	MT_TXS0_TX_STATUS_MCU		(1U << 14)
#define	MT_TXS0_TXS_FORMAT		(1U << 13)
#define	MT_TXS0_FIXED_RATE		(1U << 12)
#define	MT_TXS0_TX_RATE			0x00000fff	/* bits 11:0 */

#define	MT_TXS1_ANT_ID			0xfff00000	/* bits 31:20 */
#define	MT_TXS1_ANT_ID_S		20
#define	MT_TXS1_RESP_RATE		0x000f0000	/* bits 19:16 */
#define	MT_TXS1_RESP_RATE_S		16
#define	MT_TXS1_BW			0x0000c000	/* bits 15:14 */
#define	MT_TXS1_BW_S			14
#define	MT_TXS1_I_TXBF			(1U << 13)
#define	MT_TXS1_E_TXBF			(1U << 12)
#define	MT_TXS1_TID			0x00000e00	/* bits 11:9 */
#define	MT_TXS1_TID_S			9
#define	MT_TXS1_AMPDU			(1U << 8)
#define	MT_TXS1_ACKED_MPDU		(1U << 7)
#define	MT_TXS1_TX_POWER_DBM		0x0000007f	/* bits 6:0 */

#define	MT_TXS2_WCID			0xff000000	/* bits 31:24 */
#define	MT_TXS2_WCID_S			24
#define	MT_TXS2_RXV_SEQNO		0x00ff0000	/* bits 23:16 */
#define	MT_TXS2_RXV_SEQNO_S		16
#define	MT_TXS2_TX_DELAY		0x0000ffff	/* bits 15:0 */

#define	MT_TXS3_LAST_TX_RATE		0xe0000000	/* bits 31:29 */
#define	MT_TXS3_LAST_TX_RATE_S		29
#define	MT_TXS3_TX_COUNT		0x1f000000	/* bits 28:24 */
#define	MT_TXS3_TX_COUNT_S		24
#define	MT_TXS3_F0_SEQNO		0x00000fff	/* bits 11:0 */

/*
 * The identifier the descriptor puts on a frame and the report gives
 * back.  Zero means the firmware need not report on the frame at all,
 * and a report carrying it is discarded, so a frame that wants one has
 * to be marked with something else.  Nothing here matches a report back
 * to the frame it came from, so a single value for all of them is
 * enough to tell the two cases apart.
 */
#define	MT7615_PID_NO_STATUS		0
#define	MT7615_PID_STATUS		1

/*
 * The slowest rate of each modulation, by the hardware's own numbering
 * rather than the standard's: 1 Mbit/s for CCK and 6 Mbit/s for OFDM.
 * Every peer that can join the network understands one of these, which
 * is what makes them the right choice for a frame sent to a peer whose
 * capabilities are not known yet.
 */
#define	MT7615_CCK_RATE_1M		0
#define	MT7615_OFDM_RATE_6M		11

/*
 * DW7 repeats the frame type the LMAC already has in DW2.  It is not
 * redundant to the hardware: the transmit path reads the two at
 * different stages, and a frame whose DW7 disagrees is not sent.
 */
#define	MT_TXD7_TYPE			0x00300000	/* bits 21:20 */
#define	MT_TXD7_TYPE_S			20
#define	MT_TXD7_SUB_TYPE		0x000f0000	/* bits 19:16 */
#define	MT_TXD7_SUB_TYPE_S		16
#define	MT_TXD7_SPE_IDX			0x0000f800	/* bits 15:11 */
#define	MT_TXD7_SPE_IDX_S		11

/* Spatial extension index the reference driver uses throughout. */
#define	MT7615_SPE_IDX_DEFAULT		0x18

/* Header format (TXD DW1). */
#define	MT_HDR_FORMAT_802_3		0
#define	MT_HDR_FORMAT_CMD		1
#define	MT_HDR_FORMAT_802_11		2
#define	MT_HDR_FORMAT_802_11_EXT	3

/* Packet type (TXD DW1). */
#define	MT_TX_TYPE_CT			0
#define	MT_TX_TYPE_SF			1
#define	MT_TX_TYPE_CMD			2
#define	MT_TX_TYPE_FW			3

/*
 * Tx buffer pointer block that follows the TXD on the data queues.
 * MT7615 uses the firmware-managed ("fw txp") flavour: the driver hands
 * the payload over as a list of physical fragments plus a token, and
 * gets the token back through a TXRX_NOTIFY event when the frame is
 * done with.
 */
#define	MT_TXP_MAX_BUF_NUM		6

struct mt7615_fw_txp {
    uint16_t	flags;
    uint16_t	token;
    uint8_t		bss_idx;
    uint16_t	rept_wds_wcid;
    uint8_t		nbuf;
    uint32_t	buf[MT_TXP_MAX_BUF_NUM];
    uint16_t	len[MT_TXP_MAX_BUF_NUM];
} __packed __aligned(4);

/*
 * Flags in the pointer block.  APPLY_TXD tells the firmware to send the
 * descriptor built above it rather than composing one of its own; a
 * frame without it never leaves.  The cipher flag says the frame is
 * already in the clear and needs no key, which is so as long as the
 * driver does no hardware encryption.
 */
#define	MT_CT_INFO_APPLY_TXD		(1U << 0)
#define	MT_CT_INFO_COPY_HOST_TXD_ALL	(1U << 1)
#define	MT_CT_INFO_MGMT_FRAME		(1U << 2)
#define	MT_CT_INFO_NONE_CIPHER_FRAME	(1U << 3)
#define	MT_CT_INFO_HSR2_TX		(1U << 4)

/*
 * How much of the frame the engine reads straight from the ring
 * descriptor.  The firmware parses the header out of it; the rest of
 * the frame it fetches through the pointer block.
 */
#define	MT_CT_PARSE_LEN			72

#define	MT_TXD_TXP_SIZE			(MT_TXD_SIZE + sizeof(struct mt7615_fw_txp))

/*
 * Per-descriptor stride of the Tx scratch area.  Rounded well clear of
 * MT_TXD_TXP_SIZE so each slot starts on its own cache line.
 */
#define	MT7615_TXD_STRIDE		128

/*
 * Rx descriptor.  Words 0-3 are always present; groups 4 and 1 add
 * further words ahead of the 802.11 frame, and RXD0 says which.
 */
#define	MT_RXD0_LENGTH			0x0000ffff	/* bits 15:0 */
#define	MT_RXD0_PKT_FLAG		0x000f0000	/* bits 19:16 */
#define	MT_RXD0_PKT_FLAG_S		16
#define	MT_RXD0_PKT_TYPE		0xe0000000	/* bits 31:29 */
#define	MT_RXD0_PKT_TYPE_S		29
#define	MT_RXD0_NORMAL_GROUP_1		(1U << 25)
#define	MT_RXD0_NORMAL_GROUP_2		(1U << 26)
#define	MT_RXD0_NORMAL_GROUP_3		(1U << 27)
#define	MT_RXD0_NORMAL_GROUP_4		(1U << 28)

#define	MT_RXD1_NORMAL_HDR_TRANS	(1U << 23)
#define	MT_RXD1_NORMAL_HDR_OFFSET	(1U << 22)
#define	MT_RXD1_NORMAL_MAC_HDR_LEN	0x003f0000	/* bits 21:16 */
#define	MT_RXD1_NORMAL_MAC_HDR_LEN_S	16
#define	MT_RXD1_NORMAL_CH_FREQ		0x0000ff00	/* bits 15:8 */
#define	MT_RXD1_NORMAL_CH_FREQ_S	8
#define	MT_RXD1_NORMAL_ADDR_TYPE	0x00000006	/* bits 2:1 */
#define	MT_RXD1_NORMAL_U2M		(1U << 1)

#define	MT_RXD2_NORMAL_NON_AMPDU	(1U << 31)
#define	MT_RXD2_NORMAL_HDR_TRANS_ERROR	(1U << 25)
#define	MT_RXD2_NORMAL_MAX_LEN_ERROR	(1U << 24)
#define	MT_RXD2_NORMAL_AMSDU_ERR	(1U << 23)
#define	MT_RXD2_NORMAL_LEN_MISMATCH	(1U << 22)
#define	MT_RXD2_NORMAL_TKIP_MIC_ERR	(1U << 21)
#define	MT_RXD2_NORMAL_ICV_ERR		(1U << 20)
#define	MT_RXD2_NORMAL_CLM		(1U << 19)
#define	MT_RXD2_NORMAL_CM		(1U << 18)
#define	MT_RXD2_NORMAL_FCS_ERR		(1U << 17)
#define	MT_RXD2_NORMAL_SEC_MODE		0x0000f000	/* bits 15:12 */
#define	MT_RXD2_NORMAL_SEC_MODE_S	12
#define	MT_RXD2_NORMAL_TID		0x00000f00	/* bits 11:8 */
#define	MT_RXD2_NORMAL_TID_S		8
#define	MT_RXD2_NORMAL_WLAN_IDX		0x000000ff	/* bits 7:0 */

#define	MT_RXV1_TX_RATE			0x0000007f	/* bits 6:0 */
#define	MT_RXV1_TX_MODE			0x00007000	/* bits 14:12 */
#define	MT_RXV1_TX_MODE_S		12
#define	MT_RXV1_FRAME_MODE		0x00018000	/* bits 16:15 */
#define	MT_RXV1_FRAME_MODE_S		15
#define	MT_RXV1_HT_SHORT_GI		(1U << 19)

#define	MT_RXV3_IB_RSSI			0x00ff0000	/* bits 23:16 */
#define	MT_RXV3_IB_RSSI_S		16
#define	MT_RXV3_WB_RSSI			0xff000000	/* bits 31:24 */
#define	MT_RXV3_WB_RSSI_S		24

/* Rx packet types carried in RXD0. */
#define	MT_PKT_TYPE_TXS			0
#define	MT_PKT_TYPE_TXRXV		1
#define	MT_PKT_TYPE_NORMAL		2
#define	MT_PKT_TYPE_RX_DUP_RFB		3
#define	MT_PKT_TYPE_RX_TMR		4
#define	MT_PKT_TYPE_RETRIEVE		5
#define	MT_PKT_TYPE_TXRX_NOTIFY		6
#define	MT_PKT_TYPE_RX_EVENT		7
#define	MT_PKT_TYPE_NORMAL_MCU		8

/* Tx-free (TXRX_NOTIFY) payload: tokens the firmware is returning. */
#define	MT_TX_FREE_MSDU_ID_CNT		0x0000007f	/* bits 6:0 */

/*
 * The event by which the firmware hands frames back.
 *
 * A data frame is not copied when it is queued: the descriptor carries
 * only the header, and the firmware fetches the body itself from the
 * addresses in the pointer block, whenever it gets to it.  The ring
 * descriptor is therefore finished with long before the frame is, and
 * freeing the buffer then would pull it out from under the firmware.
 *
 * So each frame goes out carrying a token, and this event is how the
 * firmware says it is done with one.  A run of tokens follows the
 * header, as many as the count says.
 */
struct mt7615_tx_free {
    uint16_t	rx_byte_cnt;
    uint16_t	ctrl;
    uint32_t	txd;
} __packed;

/*
 * MCU transport.
 *
 * A command is a TXD followed by this header and then the command
 * payload.  Replies arrive on the MCU Rx ring with a matching header
 * and carry the sequence number back so the sender can be woken.
 */
struct mt7615_mcu_txd {
    uint32_t	txd[8];

    uint16_t	len;
    uint16_t	pq_id;

    uint8_t		cid;
    uint8_t		pkt_type;
    uint8_t		set_query;
    uint8_t		seq;

    uint8_t		uc_d2b0_rev;
    uint8_t		ext_cid;
    uint8_t		s2d_index;
    uint8_t		ext_cid_ack;

    uint32_t	reserved[5];
} __packed __aligned(4);

struct mt7615_mcu_rxd {
    uint32_t	rxd[4];

    uint16_t	len;
    uint16_t	pkt_type_id;

    uint8_t		eid;
    uint8_t		seq;
    uint16_t	__rsv;

    uint8_t		ext_eid;
    uint8_t		__rsv1[2];
    uint8_t		s2d_index;
} __packed __aligned(4);

#define	MCU_PQ_ID(_p, _q)		(((_p) << 15) | ((_q) << 10))
#define	MCU_PKT_ID			0xa0

#define	MCU_Q_QUERY			0
#define	MCU_Q_SET			1
#define	MCU_Q_RESERVED			2
#define	MCU_Q_NA			3

#define	MCU_S2D_H2N			0
#define	MCU_S2D_C2N			1
#define	MCU_S2D_H2C			2
#define	MCU_S2D_H2CN			3

/* Plain commands. */
#define	MCU_CMD_TARGET_ADDRESS_LEN_REQ	0x01
#define	MCU_CMD_FW_START_REQ		0x02
#define	MCU_CMD_INIT_ACCESS_REG		0x03
#define	MCU_CMD_PATCH_START_REQ		0x05
#define	MCU_CMD_PATCH_FINISH_REQ	0x07
#define	MCU_CMD_PATCH_SEM_CONTROL	0x10
#define	MCU_CMD_EXT_CID			0xed
#define	MCU_CMD_FW_SCATTER		0xee
#define	MCU_CMD_RESTART_DL_REQ		0xef

/* Extended commands, sent with cid MCU_CMD_EXT_CID and ext_cid set. */
#define	MCU_EXT_CMD_RF_TEST		0x04
#define	MCU_EXT_CMD_PM_STATE_CTRL	0x07
#define	MCU_EXT_CMD_CHANNEL_SWITCH	0x08

/*
 * Transmit power, in half decibel-milliwatts, one entry per rate group.
 * The entries below MT_SKU_1SS_DELTA are limits; the four from there on
 * are what a chain may add back when fewer of them are in use.
 */
#define	MT_SKU_1SS_DELTA		49
#define	MT_SKU_COUNT			53

/*
 * What sharing the budget between two chains costs each of them: three
 * decibels, which is six of these halves.
 */
#define	MT7615_TX_POWER_PATH_DELTA	6
#define	MT7615_TX_POWER_MAX		127	/* The field is signed. */

/*
 * The ceiling put on every channel, in whole decibel-milliwatts.  Twenty
 * is what the tightest of the domains this is likely to run in allows,
 * so it holds everywhere without needing a table per band.
 */
#define	MT7615_MAX_REG_POWER		20

/* How wide the channel is, in the firmware's numbering. */
#define	MT_CH_BW_20			0
#define	MT_CH_BW_40			1
#define	MT_CH_BW_80			2
#define	MT_CH_BW_160			3

/*
 * Why the channel is being changed.  The numbering has gaps, so a value
 * outside the list is not a near miss but something the firmware has no
 * meaning for.
 */
#define	MT_CH_SWITCH_NORMAL		0
#define	MT_CH_SWITCH_SCAN		3
#define	MT_CH_SWITCH_MCC		4
#define	MT_CH_SWITCH_DFS		5
#define	MT_CH_SWITCH_SCAN_BYPASS_DPD	9
#define	MCU_EXT_CMD_SET_TX_POWER_CTRL	0x11
#define	MCU_EXT_CMD_EFUSE_BUFFER_MODE	0x21
#define	MCU_EXT_CMD_STA_REC_UPDATE	0x25
#define	MCU_EXT_CMD_BSS_INFO_UPDATE	0x26
#define	MCU_EXT_CMD_EDCA_UPDATE		0x27

/* Which of the channel access parameters a request is setting. */
#define	MT_WMM_AIFS_SET			(1U << 0)
#define	MT_WMM_CW_MIN_SET		(1U << 1)
#define	MT_WMM_CW_MAX_SET		(1U << 2)
#define	MT_WMM_TXOP_SET			(1U << 3)
#define	MT_WMM_PARAM_SET		(MT_WMM_AIFS_SET | MT_WMM_CW_MIN_SET | \
					 MT_WMM_CW_MAX_SET | MT_WMM_TXOP_SET)
#define	MCU_EXT_CMD_DEV_INFO_UPDATE	0x2A
#define	MCU_EXT_CMD_WTBL_UPDATE		0x32
#define	MCU_EXT_CMD_PROTECT_CTRL	0x3e
#define	MCU_EXT_CMD_MAC_INIT_CTRL	0x46
#define	MCU_EXT_CMD_RX_HDR_TRANS	0x47
#define	MCU_EXT_CMD_SET_RX_PATH		0x4e
/*
 * Per-channel calibration.  The receiver's DC offset and the
 * transmitter's pre-distortion both depend on where in the band the
 * radio sits and on how wide it is, and the firmware does not work
 * them out from the channel switch alone.
 */
#define	MCU_EXT_CMD_RXDCOC_CAL		0x59
#define	MCU_EXT_CMD_TXDPD_CAL		0x60

#define	MCU_EXT_CMD_BCN_OFFLOAD		0x49

/*
 * Command payloads that describe a network to the firmware.
 *
 * Each is a small header followed by a run of tag/length blocks.  The
 * firmware walks the blocks it recognises and ignores the rest, so a
 * request carries only what it means to change.
 */
struct mt7615_tlv {
    uint16_t	tag;
    uint16_t	len;
} __packed;

/* DEV_INFO_UPDATE: the hardware MAC slot a network is built on. */
#define	MT_DEV_INFO_ACTIVE		0

struct mt7615_dev_info_req {
    struct {
        uint8_t		omac_idx;
        uint8_t		band_idx;
        uint16_t	tlv_num;
        uint8_t		is_tlv_append;
        uint8_t		rsv[3];
    } __packed hdr;
    struct {
        uint16_t	tag;
        uint16_t	len;
        uint8_t		active;
        uint8_t		band_idx;
        uint8_t		omac_addr[IEEE80211_ADDR_LEN];
    } __packed tlv;
} __packed;

/* BSS_INFO_UPDATE: the network itself. */
#define	MT_BSS_INFO_OMAC		0
#define	MT_BSS_INFO_BASIC		1

struct mt7615_bss_info_omac {
    uint16_t	tag;
    uint16_t	len;
    uint8_t		hw_bss_idx;
    uint8_t		omac_idx;
    uint8_t		band_idx;
    uint8_t		rsv0;
    uint32_t	conn_type;
    uint32_t	rsv1;
} __packed;

struct mt7615_bss_info_basic {
    uint16_t	tag;
    uint16_t	len;
    uint32_t	network_type;
    uint8_t		active;
    uint8_t		rsv0;
    uint16_t	bcn_interval;
    uint8_t		bssid[IEEE80211_ADDR_LEN];
    uint8_t		wmm_idx;
    uint8_t		dtim_period;
    uint8_t		bmc_wcid_lo;
    uint8_t		cipher;
    uint8_t		phy_mode;
    uint8_t		max_bssid;
    uint8_t		non_tx_bssid;
    uint8_t		bmc_wcid_hi;
    uint8_t		rsv[2];
} __packed;

struct mt7615_bss_info_req {
    struct {
        uint8_t		bss_idx;
        uint8_t		wlan_idx_lo;
        uint16_t	tlv_num;
        uint8_t		is_tlv_append;
        uint8_t		muar_idx;
        uint8_t		wlan_idx_hi;
        uint8_t		rsv;
    } __packed hdr;
    struct mt7615_bss_info_omac	omac;
    struct mt7615_bss_info_basic	basic;
} __packed;

/* STA_REC_UPDATE: one associated station. */
#define	MT_STA_REC_BASIC		0
#define	MT_STA_REC_BA			6
#define	MT_STA_REC_HT			9
#define	MT_STA_REC_VHT			10

/*
 * Which end of a block-acknowledgement session this record describes:
 * the one that sends the aggregates or the one that acknowledges them.
 */
#define	MT_BA_TYPE_INVALID		0
#define	MT_BA_TYPE_ORIGINATOR		1
#define	MT_BA_TYPE_RECIPIENT		2

/* How a session being torn down is picked out: by peer address and tid. */
#define	MT_RST_BA_MAC_TID_MATCH		0

#define	MT_CONN_STATE_DISCONNECT	0
#define	MT_CONN_STATE_CONNECT		1
#define	MT_CONN_STATE_PORT_SECURE	2

#define	MT_STA_TYPE_STA			(1U << 0)
#define	MT_STA_TYPE_AP			(1U << 1)
#define	MT_STA_TYPE_BC			(1U << 5)
#define	MT_NETWORK_INFRA		(1U << 16)

#define	MT_CONNECTION_INFRA_STA		(MT_STA_TYPE_STA | MT_NETWORK_INFRA)
#define	MT_CONNECTION_INFRA_AP		(MT_STA_TYPE_AP | MT_NETWORK_INFRA)
#define	MT_CONNECTION_INFRA_BC		(MT_STA_TYPE_BC | MT_NETWORK_INFRA)

#define	MT_EXTRA_INFO_VER		(1U << 0)
#define	MT_EXTRA_INFO_NEW		(1U << 1)

struct mt7615_sta_rec_basic {
    uint16_t	tag;
    uint16_t	len;
    uint32_t	conn_type;
    uint8_t		conn_state;
    uint8_t		qos;
    uint16_t	aid;
    uint8_t		peer_addr[IEEE80211_ADDR_LEN];
    uint16_t	extra_info;
} __packed;

struct mt7615_sta_rec_ht {
    uint16_t	tag;
    uint16_t	len;
    uint16_t	ht_cap;
    uint16_t	rsv;
} __packed;

struct mt7615_sta_rec_vht {
    uint16_t	tag;
    uint16_t	len;
    uint32_t	vht_cap;
    uint16_t	vht_rx_mcs_map;
    uint16_t	vht_tx_mcs_map;
} __packed;

struct mt7615_sta_rec_ba {
    uint16_t	tag;
    uint16_t	len;
    uint8_t		tid;
    uint8_t		ba_type;
    uint8_t		amsdu;
    uint8_t		ba_en;		/* A bit per tid, not a boolean. */
    uint16_t	ssn;
    uint16_t	winsize;
} __packed;

struct mt7615_sta_rec_hdr {
    uint8_t		bss_idx;
    uint8_t		wlan_idx_lo;
    uint16_t	tlv_num;
    uint8_t		is_tlv_append;
    uint8_t		muar_idx;
    uint8_t		wlan_idx_hi;
    uint8_t		rsv;
} __packed;

/*
 * The HT record is only appended for a peer that negotiated it, so a
 * request without one is shorter by exactly that much.
 */
struct mt7615_sta_rec_req {
    struct mt7615_sta_rec_hdr	hdr;
    struct mt7615_sta_rec_basic	basic;
    struct mt7615_sta_rec_ht	ht;
    struct mt7615_sta_rec_vht	vht;
} __packed;

struct mt7615_sta_rec_ba_req {
    struct mt7615_sta_rec_hdr	hdr;
    struct mt7615_sta_rec_ba	ba;
} __packed;

/* WTBL_UPDATE: the station's slot in the hardware table. */
#define	MT_WTBL_GENERIC			0
#define	MT_WTBL_RX			1
#define	MT_WTBL_HT			2
#define	MT_WTBL_VHT			3
#define	MT_WTBL_BA			8
#define	MT_WTBL_SPE			16

#define	MT_WTBL_RESET_AND_SET		1
#define	MT_WTBL_SET			2

/*
 * A pseudo-station stands for whoever is listening rather than for one
 * peer, so it is not tied to a hardware MAC slot.  This value in place
 * of a slot number is how that is said.
 */
#define	MT_WTBL_MUAR_NONE		0xe

struct mt7615_wtbl_generic {
    uint16_t	tag;
    uint16_t	len;
    uint8_t		peer_addr[IEEE80211_ADDR_LEN];
    uint8_t		muar_idx;
    uint8_t		skip_tx;
    uint8_t		cf_ack;
    uint8_t		qos;
    uint8_t		mesh;
    uint8_t		adm;
    uint16_t	partial_aid;
    uint8_t		baf_en;
    uint8_t		aad_om;
} __packed;

struct mt7615_wtbl_rx {
    uint16_t	tag;
    uint16_t	len;
    uint8_t		rcid;
    uint8_t		rca1;
    uint8_t		rca2;
    uint8_t		rv;
    uint8_t		rsv[4];
} __packed;

struct mt7615_wtbl_ht {
    uint16_t	tag;
    uint16_t	len;
    uint8_t		ht;
    uint8_t		ldpc;
    uint8_t		af;		/* A-MPDU length exponent. */
    uint8_t		mm;		/* Minimum MPDU start spacing. */
    uint8_t		rsv[4];
} __packed;

struct mt7615_wtbl_vht {
    uint16_t	tag;
    uint16_t	len;
    uint8_t		ldpc;
    uint8_t		dyn_bw;
    uint8_t		vht;
    uint8_t		txop_ps;
    uint8_t		rsv[4];
} __packed;

struct mt7615_wtbl_spe {
    uint16_t	tag;
    uint16_t	len;
    uint8_t		spe_idx;
    uint8_t		rsv[3];
} __packed;

/*
 * One block-acknowledgement session.  Which half of this is read
 * depends on ba_type: the originator sends the aggregates and needs the
 * starting sequence number and the window, the recipient acknowledges
 * them and is named by address so a session can be torn down again.
 */
struct mt7615_wtbl_ba {
    uint16_t	tag;
    uint16_t	len;
    /* Both ends. */
    uint8_t		tid;
    uint8_t		ba_type;
    uint8_t		rsv0[2];
    /* Originator. */
    uint16_t	sn;
    uint8_t		ba_en;
    uint8_t		ba_winsize_idx;
    /* Both ends. */
    uint16_t	ba_winsize;
    /* Recipient. */
    uint8_t		peer_addr[IEEE80211_ADDR_LEN];
    uint8_t		rst_ba_tid;
    uint8_t		rst_ba_sel;
    uint8_t		rst_ba_sb;
    uint8_t		band_idx;
    uint8_t		rsv1[4];
} __packed;

struct mt7615_wtbl_req_hdr {
    uint8_t		wlan_idx_lo;
    uint8_t		operation;
    uint16_t	tlv_num;
    uint8_t		wlan_idx_hi;
    uint8_t		rsv[3];
} __packed;

struct mt7615_wtbl_req {
    struct mt7615_wtbl_req_hdr	hdr;
    struct mt7615_wtbl_generic	generic;
    struct mt7615_wtbl_rx		rx;
    struct mt7615_wtbl_spe		spe;
    struct mt7615_wtbl_ht		ht;
    struct mt7615_wtbl_vht		vht;
} __packed;

struct mt7615_wtbl_ba_req {
    struct mt7615_wtbl_req_hdr	hdr;
    struct mt7615_wtbl_ba		ba;
} __packed;

/*
 * The window sizes the hardware knows, smallest first.  The session
 * carries the size itself as well, but this chip also wants the index
 * of the largest entry the window covers.
 */
#define	MT7615_BA_RANGE		{ 4, 8, 12, 24, 36, 48, 54, 64 }
#define	MT7615_BA_RANGE_COUNT	8

/*
 * BCN_OFFLOAD: the beacon the firmware repeats on its own.  The frame
 * is handed over whole, descriptor included, and the firmware is told
 * where the TIM sits so it can keep it current.
 */
#define	MT7615_BCN_PKT_MAX		512

struct mt7615_bcn_offload_req {
    uint8_t		omac_idx;
    uint8_t		enable;
    uint8_t		wlan_idx;
    uint8_t		band_idx;
    uint8_t		pkt_type;
    uint8_t		need_pre_tbtt_int;
    uint16_t	csa_ie_pos;
    uint16_t	pkt_len;
    uint16_t	tim_ie_pos;
    uint8_t		pkt[MT7615_BCN_PKT_MAX];
    uint8_t		csa_cnt;
    uint8_t		bcc_cnt;
    uint16_t	bcc_ie_pos;
} __packed;

/* Modulations a network runs, as the firmware counts them. */
#define	MT_PHY_MODE_11B			(1U << 0)
#define	MT_PHY_MODE_11A			(1U << 1)
#define	MT_PHY_MODE_11G			(1U << 2)
#define	MT_PHY_MODE_11GN		(1U << 3)
#define	MT_PHY_MODE_11AN		(1U << 4)
#define	MT_PHY_MODE_11AC		(1U << 5)

/* LMAC queues a descriptor can be aimed at. */
#define	MT_LMAC_ALTX0			0x10
#define	MT_LMAC_BMC0			0x11
#define	MT_LMAC_BCN0			0x12

/*
 * Station slots in the hardware table.
 *
 * Slot 0 is the firmware's own: beacons and management frames are sent
 * from it, and writing a station record there takes the beacon down
 * with it.  Counting from the far end, each network gets a slot for
 * the pseudo-station its group traffic comes from.  Real peers take
 * what is left in the middle.
 */
#define	MT7615_WTBL_GLOBAL		0
#define	MT7615_WTBL_STA_FIRST		1

/* Patch semaphore results. */
#define	MT_PATCH_NOT_DL_SEM_FAIL	0
#define	MT_PATCH_IS_DL			1
#define	MT_PATCH_NOT_DL_SEM_SUCCESS	2
#define	MT_PATCH_REL_SEM_SUCCESS	3

#define	MT_PATCH_SEM_RELEASE		0
#define	MT_PATCH_SEM_GET		1

/* Download mode flags for TARGET_ADDRESS_LEN_REQ. */
#define	MT_DL_MODE_ENCRYPT		(1U << 0)
#define	MT_DL_MODE_KEY_IDX		0x00000006	/* bits 2:1 */
#define	MT_DL_MODE_KEY_IDX_S		1
#define	MT_DL_MODE_RESET_SEC_IV		(1U << 3)
#define	MT_DL_MODE_WORKING_PDA_CR4	(1U << 4)
#define	MT_DL_CONFIG_ENCRY_MODE_SEL	(1U << 6)
#define	MT_DL_MODE_NEED_RSP		(1U << 31)

/* Start options for FW_START_REQ. */
#define	MT_FW_START_OVERRIDE		(1U << 0)
#define	MT_FW_START_DLYCAL		(1U << 1)
#define	MT_FW_START_WORKING_PDA_CR4	(1U << 2)

/*
 * Firmware images.
 *
 * Three are needed.  The ROM patch carries a header at the front; the
 * N9 and CR4 images carry one trailer per region at the back, so the
 * trailers are found by counting back from the end of the file.
 */
#define	MT7615_ROM_PATCH		"mediatek/mt7615_rom_patch.bin"
#define	MT7615_FIRMWARE_N9		"mediatek/mt7615_n9.bin"
#define	MT7615_FIRMWARE_CR4		"mediatek/mt7615_cr4.bin"

#define	MT7615_PATCH_ADDRESS		0x80000

#define	MT7615_N9_REGION_NUM		2
#define	MT7615_CR4_REGION_NUM		1
#define	MT7615_MAX_REGION_NUM		2

/* Each region's payload is followed by a four byte CRC. */
#define	MT7615_IMG_CRC_LEN		4

/*
 * FW_SCATTER commands are capped at this size in total, header
 * included, so the payload that fits is this less the MCU header.
 */
#define	MT7615_FW_CHUNK_SIZE		4096

struct mt7615_patch_hdr {
    char		build_date[16];
    char		platform[4];
    uint32_t	hw_sw_ver;	/* big endian */
    uint32_t	patch_ver;	/* big endian */
    uint16_t	checksum;	/* big endian */
} __packed;

struct mt7615_fw_trailer {
    uint32_t	addr;		/* little endian */
    uint8_t		chip_id;
    uint8_t		feature_set;
    uint8_t		eco_code;
    char		fw_ver[10];
    char		build_date[15];
    uint32_t	len;		/* little endian */
} __packed;

/* feature_set bits, consulted when building the download mode. */
#define	MT7615_FW_FEATURE_SET_ENCRYPT	(1U << 0)
#define	MT7615_FW_FEATURE_SET_KEY_IDX	0x0000000c	/* bits 3:2 */
#define	MT7615_FW_FEATURE_SET_KEY_IDX_S	2

/*
 * Ring geometry.  The hardware ring index is what gets multiplied into
 * MT_TX_RING_BASE / MT_RX_RING_BASE; the driver's own numbering follows
 * in if_mt7615var.h.
 */
#define	MT7615_TXQ_HW_MAIN		0
#define	MT7615_TXQ_HW_EXT		1
#define	MT7615_TXQ_HW_MCU		2
#define	MT7615_TXQ_HW_FWDL		3

#define	MT7615_RXQ_HW_MAIN		0
#define	MT7615_RXQ_HW_MCU		1

#define	MT7615_TX_RING_SIZE		1024

/*
 * How many frames the firmware may hold at once, one token per frame.
 *
 * This is what really bounds how much can be in flight, and it has to
 * be well above the ring: a ring slot is free again as soon as the
 * engine has read the descriptor, while the token stays taken until the
 * firmware has finished with the frame and says so.  Running out means
 * dropping frames that were already accepted, which costs far more
 * than the table does.
 */
#define	MT7615_TOKEN_SIZE		4096
#define	MT7615_TX_MCU_RING_SIZE		128
#define	MT7615_TX_FWDL_RING_SIZE	128
#define	MT7615_RX_RING_SIZE		1024
/* The firmware hands transmit tokens back on this ring, not the main one. */
#define	MT7615_RX_MCU_RING_SIZE		512

#define	MT7615_RX_BUF_SIZE		2048

#define	MT7615_WTBL_SIZE		128
#define	MT7615_MAX_INTERFACES		16

/* Access-category queues the LMAC keeps per network. */
#define	MT7615_MAX_WMM_SETS		4

/* The topmost slot, which the first network's group traffic uses. */
#define	MT7615_WTBL_RESERVED		(MT7615_WTBL_SIZE - 1)

/* One past the last slot a real peer may take. */
#define	MT7615_WTBL_STA_LIMIT		(MT7615_WTBL_RESERVED - \
					 MT7615_MAX_INTERFACES)

#define	MT7615_CFEND_RATE_DEFAULT	0x49	/* OFDM 24M */
#define	MT7615_CFEND_RATE_11B		0x03	/* CCK 11M, long preamble */
#define	MT7615_BAR_RATE_DEFAULT		0x4b	/* OFDM 6M */
#define	MT7615_RATE_RETRY		2

/*
 * How many times the hardware may try one frame.  A beacon gets the
 * largest the field holds, since it is worth repeating; ordinary
 * traffic gets what the reference driver uses, because a frame retried
 * thirty times at one rate holds up everything behind it.
 */
#define	MT7615_TX_RETRY			8

#endif /* _IF_MT7615REG_H_ */
