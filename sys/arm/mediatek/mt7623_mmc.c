/*
 * Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcbrvar.h>
#include <dev/mmc/mmc_fdt_helpers.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/regulator/regulator.h>

#include "opt_mmccam.h"

#ifdef MMCCAM
#include <cam/cam.h>
#include <cam/cam_ccb.h>
#include <cam/cam_debug.h>
#include <cam/cam_sim.h>
#include <cam/cam_xpt_sim.h>
#include <cam/mmc/mmc_sim.h>

#include "mmc_sim_if.h"
#endif

#include "mmc_pwrseq_if.h"

#define	MT_MMC_MEMRES		                0
#define	MT_MMC_IRQRES		                1

#define	MT_MSDC_CFG		                (0x0)
#define  MT_MSDC_CFG_MSDC			(1U << 0)
#define  MT_MSDC_CFG_CCKPD			(1U << 1)
#define  MT_MSDC_CFG_RST			(1U << 2)
#define  MT_MSDC_CFG_PIO			(1U << 3)
#define  MT_MSDC_CFG_CCKDRVE			(1U << 4)
#define  MT_MSDC_CFG_BV18SDT 			(1U << 5)
#define  MT_MSDC_CFG_BV18PSS			(1U << 6)
#define  MT_MSDC_CFG_CCKSB			(1U << 7)
#define  MT_MSDC_CFG_CCKDIV_SHIFT		8
#define	 MT_MSDC_CFG_CCKDIV_MASK		(0xFFF << 8)
#define  MT_MSDC_CFG_CCKMD_SHIFT		20
#define  MT_MSDC_CFG_CCKMD_MASK		(0x03 << 20)
#define  MT_MSDC_CFG_DDR			(1U << 21)
#define	 MT_MSDC_CFG_HS400_CK_MODE_EXTRA	(1U << 22)
#define	MT_MSDC_IOCON		(0x04)
#define  MT_MSDC_IOCON_SDR104CKS		(1U << 0)
#define  MT_MSDC_IOCON_RSPL			(1U << 1)
#define  MT_MSDC_IOCON_RDSPL			(1U << 2)
#define  MT_MSDC_IOCON_DDLSEL			(1U << 3)
#define  MT_MSDC_IOCON_RDSPLSEL		(1U << 5)
#define  MT_MSDC_IOCON_WDSPL			(1U << 8)
#define  MT_MSDC_IOCON_WDSPLSEL		(1U << 9)
#define  MT_MSDC_IOCON_WD0SPL			(1U << 10)
#define  MT_MSDC_IOCON_WD1SPL			(1U << 11)
#define  MT_MSDC_IOCON_WD2SPL			(1U << 12)
#define  MT_MSDC_IOCON_WD3SPL			(1U << 13)
#define  MT_MSDC_IOCON_RD0SPL			(1U << 16)
#define  MT_MSDC_IOCON_RD1SPL			(1U << 17)
#define  MT_MSDC_IOCON_RD2SPL			(1U << 18)
#define  MT_MSDC_IOCON_RD3SPL			(1U << 19)
#define  MT_MSDC_IOCON_RD4SPL			(1U << 20)
#define  MT_MSDC_IOCON_RD5SPL			(1U << 21)
#define  MT_MSDC_IOCON_RD6SPL			(1U << 22)
#define  MT_MSDC_IOCON_RD7SPL			(1U << 23)
#define	MT_MSDC_PS		(0x08)
#define  MT_MSDC_PS_CDEN			(1U << 0)
#define  MT_MSDC_PS_CDSTS			(1U << 1)
#define  MT_MSDC_PS_CDDBCE_SHIFT		12
#define  MT_MSDC_PS_CDDBCE_MASK		(0xF << 12)
#define  MT_MSDC_PS_DAT_SHIFT			16
#define  MT_MSDC_PS_DAT_MASK			(0xFF << 16)
#define  MT_MSDC_PS_CMD			(1U << 24)
#define  MT_MSDC_PS_SDWP			(1U << 31)
#define	MT_MSDC_INT		(0x0c)
#define  MT_MSDC_INT_MMCIRQ			(1U << 0)
#define  MT_MSDC_INT_MSDCCDSC			(1U << 1)
#define  MT_MSDC_INT_SDACDCRDY			(1U << 3)
#define  MT_MSDC_INT_SDACDCTO			(1U << 4)
#define  MT_MSDC_INT_SDACDRCRCER		(1U << 5)
#define  MT_MSDC_INT_DMAQEPTY			(1U << 6)
#define  MT_MSDC_INT_SDIOIRQ			(1U << 7)
#define  MT_MSDC_INT_SDCRDY			(1U << 8)
#define  MT_MSDC_INT_SDCTO			(1U << 9)
#define  MT_MSDC_INT_SDRCRCER			(1U << 10)
#define  MT_MSDC_INT_SDCSTA			(1U << 11)
#define  MT_MSDC_INT_SDXFCPL			(1U << 12)
#define  MT_MSDC_INT_DMAXFDNE			(1U << 13)
#define  MT_MSDC_INT_SDDTO			(1U << 14)
#define  MT_MSDC_INT_SDDCRCERR			(1U << 15)
#define  MT_MSDC_INT_BDCSERR			(1U << 17)
#define  MT_MSDC_INT_GPDCSERR			(1U << 18)
#define  MT_MSDC_INT_DMAPROTECT		(1U << 19)
#define	MT_MSDC_INTEN		(0x10)
#define  MT_MSDC_INTEN_ENMMCIRQ		(1U << 0)
#define  MT_MSDC_INTEN_ENMSDCCDSC		(1U << 1)
#define  MT_MSDC_INTEN_ENSDACDCRDY		(1U << 3)
#define  MT_MSDC_INTEN_ENSDACDCTO		(1U << 4)
#define  MT_MSDC_INTEN_ENSDACDRCRCER		(1U << 5)
#define  MT_MSDC_INTEN_ENDMAQEPTY		(1U << 6)
#define  MT_MSDC_INTEN_ENSDIOIRQ		(1U << 7)
#define  MT_MSDC_INTEN_ENSDCRDY		(1U << 8)
#define  MT_MSDC_INTEN_ENSDCTO			(1U << 9)
#define  MT_MSDC_INTEN_ENSDRCRCER		(1U << 10)
#define  MT_MSDC_INTEN_ENSDCSTA		(1U << 11)
#define  MT_MSDC_INTEN_ENSDXFCPL		(1U << 12)
#define  MT_MSDC_INTEN_ENDMAXFDNE		(1U << 13)
#define  MT_MSDC_INTEN_ENSDDTO			(1U << 14)
#define  MT_MSDC_INTEN_ENSDDCRCERR		(1U << 15)
#define  MT_MSDC_INTEN_ENBDCSERR		(1U << 17)
#define  MT_MSDC_INTEN_ENGPDCSERR		(1U << 18)
#define  MT_MSDC_INTEN_ENDMAPROTECT		(1U << 19)
#define	MT_MSDC_FIFOCS		(0x14)
#define  MT_MSDC_FIFOCS_RXFIFOCNT_SHIFT	0
#define  MT_MSDC_FIFOCS_RXFIFOCNT_MASK		(0xFF << 0)
#define  MT_MSDC_FIFOCS_TXFIFOCNT_SHIFT	16
#define  MT_MSDC_FIFOCS_TXFIFOCNT_MASK		(0xFF << 16)
#define  MT_MSDC_FIFOCS_FIFOCLR		(1U << 31)
#define	MT_MSDC_TXDATA		(0x18)
#define	MT_MSDC_RXDATA		(0x1c)
#define	MT_SDC_CFG		(0x30)
#define  MT_SDC_CFG_ENWKUPSDIOINT		(1U << 0)
#define  MT_SDC_CFG_ENWKUPINS			(1U << 1)
#define  MT_SDC_CFG_BUSWD_1BIT			(0x0 << 16)
#define  MT_SDC_CFG_BUSWD_4BIT			(0x1 << 16)
#define  MT_SDC_CFG_BUSWD_8BIT			(0x2 << 16)
#define  MT_SDC_CFG_BUSWD_MASK			(0x3 << 16)
#define  MT_SDC_CFG_SDIO			(1U << 19)
#define  MT_SDC_CFG_SDIOIDE			(1U << 20)
#define  MT_SDC_CFG_INTBGP			(1U << 21)
#define  MT_SDC_CFG_DTOC_SHIFT			24
#define  MT_SDC_CFG_DTOC_MASK			(0xFF << 24)
#define	MT_SDC_CMD		(0x34)
#define  MT_SDC_CMD_CMD_MASK			(0x1F << 0)
#define  MT_SDC_CMD_BREAK			(1U << 6)
#define  MT_SDC_CMD_RSPTYP_R1			(0x1 << 7)
#define  MT_SDC_CMD_RSPTYP_R2			(0x2 << 7)
#define  MT_SDC_CMD_RSPTYP_R3			(0x3 << 7)
#define  MT_SDC_CMD_RSPTYP_R4			(0x4 << 7)
#define  MT_SDC_CMD_RSPTYP_R1B			(0x7 << 7)
#define  MT_SDC_CMD_DTYPE_SINGLE		(0x1 << 11)
#define  MT_SDC_CMD_DTYPE_MULTI		(0x2 << 11)
#define  MT_SDC_CMD_DTYPE_STREAM		(0x3 << 11)
#define  MT_SDC_CMD_RW				(1U << 13)
#define  MT_SDC_CMD_STOP			(1U << 14)
#define  MT_SDC_CMD_GOIRQ			(1U << 15)
#define  MT_SDC_CMD_LEN_SHIFT			16
#define  MT_SDC_CMD_LEN_MASK			(0xFFF << 16)
#define  MT_SDC_CMD_ACMD			(1U << 28)
#define	MT_SDC_ARG		(0x38)
#define	MT_SDC_STS		(0x3c)
#define  MT_SDC_STS_SDCBSY			(1U << 0)
#define  MT_SDC_STS_CMDBSY			(1U << 1)
#define  MT_SDC_STS_CMD_WR_BUSY		(1U << 16)
#define  MT_SDC_STS_MMCSWRCPL			(1U << 31)
#define	MT_SDC_RESP0		(0x40)
#define	MT_SDC_RESP1		(0x44)
#define	MT_SDC_RESP2		(0x48)
#define	MT_SDC_RESP3		(0x4c)
#define	MT_SDC_BLK_NUM		(0x50)
#define	MT_SDC_CSTS		(0x58)
#define	MT_SDC_CSTS_EN		(0x5c)
#define	MT_SDC_DATCRC_STS	(0x60)
#define  MT_SDC_DATCRC_STS_DCSSP_MASK		(0xFF << 0)
#define	MT_MSDC_DMA_SA		(0x90)
#define	MT_MSDC_DMA_CA		(0x94)
#define	MT_MSDC_DMA_CTRL	(0x98)
#define  MT_MSDC_DMA_CTRL_DMASTART		(1U << 0)
#define  MT_MSDC_DMA_CTRL_DMASTOP		(1U << 1)
#define  MT_MSDC_DMA_CTRL_DMARSM		(1U << 2)
#define  MT_MSDC_DMA_CTRL_DMAMOD		(1U << 8)
#define  MT_MSDC_DMA_CTRL_DMAALIGN		(1U << 9)
#define  MT_MSDC_DMA_CTRL_LASTBF		(1U << 10)
#define  MT_MSDC_DMA_CTRL_SPLIT1K		(1U << 11)
#define  MT_MSDC_DMA_CTRL_BSTSZ_8B		(0x3 << 12)
#define  MT_MSDC_DMA_CTRL_BSTSZ_16B 		(0x4 << 12)
#define  MT_MSDC_DMA_CTRL_BSTSZ_32B 		(0x5 << 12)
#define  MT_MSDC_DMA_CTRL_BSTSZ_64B 		(0x6 << 12)
#define	MT_MSDC_DMA_CFG	(0x9c)
#define  MT_MSDC_DMA_CFG_DMASTS		(1U << 0)
#define  MT_MSDC_DMA_CFG_DSCPCSEN		(1U << 1)
#define  MT_MSDC_DMA_CFG_AHBHPROT2EN_MASK	(0x03 << 8)
#define  MT_MSDC_DMA_CFG_AHBHPROT2EN_NUL	(0x01 << 8)
#define  MT_MSDC_DMA_CFG_AHBHPROT2EN_ONE	(0x02 << 8)
#define  MT_MSDC_DMA_CFG_MSDCACTIVEEN_MASK	(0x03 << 12)
#define  MT_MSDC_DMA_CFG_MSDCACTIVEEN_NUL	(0x01 << 12)
#define  MT_MSDC_DMA_CFG_MSDCACTIVEEN_ONE	(0x02 << 12)
#define  MT_MSDC_DMA_CFG_DMACHKSUM12B		(1U << 16)
#define	MT_MSDC_DBG_SEL	(0xa0)
#define	MT_MSDC_DBG_OUT	(0xa4)
#define	MT_MSDC_DMA_LENGTH	(0xa8)
#define	MT_MSDC_PATCH_BIT0	(0xb0)
#define  MT_MSDC_PATCH_BIT0_PTCH01		(1U << 1)
#define  MT_MSDC_PATCH_BIT0_PTCH02		(1U << 2)
#define  MT_MSDC_PATCH_BIT0_INTCKS_MASK	(0x07 << 7)
#define  MT_MSDC_PATCH_BIT0_PTCH15		(1U << 15)
#define  MT_MSDC_PATCH_BIT0_PTCH17		(1U << 17)
#define  MT_MSDC_PATCH_BIT0_PTCH18_MASK	(0x0F << 18)
#define  MT_MSDC_PATCH_BIT0_PTCH22_MASK	(0x0F << 22)
#define  MT_MSDC_PATCH_BIT0_PTCH26		(1U << 26)
#define  MT_MSDC_PATCH_BIT0_PTCH27		(1U << 27)
#define  MT_MSDC_PATCH_BIT0_PTCH28		(1U << 28)
#define  MT_MSDC_PATCH_BIT0_PTCH29		(1U << 29)
#define  MT_MSDC_PATCH_BIT0_PTCH30		(1U << 30)
#define  MT_MSDC_PATCH_BIT0_PTCH31		(1U << 31)
#define  MT_MSDC_PATCH_BIT0_DESCUP_SEL		(1U << 6)
#define  MT_MSDC_PATCH_BIT0_CKGEN_DLY_SHIFT	10
#define  MT_MSDC_PATCH_BIT0_CKGEN_DLY_MASK	(0x1F << 10)
#define	MT_MSDC_PATCH_BIT1	(0xb4)
#define  MT_MSDC_PATCH_BIT1_WRTA_MASK		(0x07 << 0)
#define  MT_MSDC_PATCH_BIT1_CMDTA_MASK		(0x07 << 3)
#define  MT_MSDC_PATCH_BIT1_GETBUSYMARGIN	(1U << 6)
#define  MT_MSDC_PATCH_BIT1_GETCRCMARGIN	(1U << 7)
#define  MT_MSDC_PATCH_BIT1_BIAS28R2_MASK	(0x0F << 8)
#define  MT_MSDC_PATCH_BIT1_BIAS28R1		(1U << 12)
#define  MT_MSDC_PATCH_BIT1_BIAS28R0		(1U << 13)
#define  MT_MSDC_PATCH_BIT1_HGDMACKEN		(1U << 23)
#define  MT_MSDC_PATCH_BIT1_MSPCCKEN		(1U << 24)
#define  MT_MSDC_PATCH_BIT1_MPSCCKEN		(1U << 25)
#define  MT_MSDC_PATCH_BIT1_MVOLDTCKEN		(1U << 26)
#define  MT_MSDC_PATCH_BIT1_MACMDCKEN		(1U << 27)
#define  MT_MSDC_PATCH_BIT1_MSDCKEN		(1U << 28)
#define  MT_MSDC_PATCH_BIT1_MWCTLCKEN		(1U << 29)
#define  MT_MSDC_PATCH_BIT1_MRCTLCKEN		(1U << 30)
#define  MT_MSDC_PATCH_BIT1_MSHBFCKEN		(1U << 31)
#define  MT_MSDC_PATCH_BIT1_DDR_CMD_FIX_SEL	(1U << 14)
#define  MT_MSDC_PATCH_BIT1_SINGLE_BURST	(1U << 16)
#define  MT_MSDC_PATCH_BIT1_RSVD20_MASK	(0x03 << 17)
#define  MT_MSDC_PATCH_BIT1_AUTO_SYNCST_CLR	(1U << 19)
#define  MT_MSDC_PATCH_BIT1_MARK_POP_WATER	(1U << 20)
#define  MT_MSDC_PATCH_BIT1_LP_DCM_EN		(1U << 21)
#define  MT_MSDC_PATCH_BIT1_RSVD3		(1U << 22)
#define  MT_MSDC_PATCH_BIT1_CLK_ENFEAT_MASK	(0xFFU << 24)
#define	MT_MSDC_PATCH_BIT2	(0xb8)
#define  MT_MSDC_PATCH_BIT2_RESPWAIT_SHIFT	2
#define  MT_MSDC_PATCH_BIT2_RESPWAIT_MASK	(0x03 << 2)
#define  MT_MSDC_PATCH_BIT2_CFGRESP		(1U << 15)
#define  MT_MSDC_PATCH_BIT2_RESPSTSENSEL_SHIFT	16
#define  MT_MSDC_PATCH_BIT2_RESPSTSENSEL_MASK	(0x07 << 16)
#define  MT_MSDC_PATCH_BIT2_CFGCRCSTS		(1U << 28)
#define  MT_MSDC_PATCH_BIT2_CRCSTSENSEL_SHIFT	29
#define  MT_MSDC_PATCH_BIT2_CRCSTSENSEL_MASK	(0x07U << 29)
#define	MTK_MSDC_PAD_TUNE	(0xf0)
#define  MTK_MSDC_PAD_TUNE_DATWRDLY_MASK	(0x1F << 0)
#define	 MTK_MSDC_PAD_TUNE_DELAYEN		(1U << 7)
#define  MTK_MSDC_PAD_TUNE_DATRRDLY_MASK	(0x1F << 8)
#define	 MTK_MSDC_PAD_TUNE_DATRRDLYSEL		(1U << 13)
#define	 MTK_MSDC_PAD_TUNE_RXDLYSEL		(1U << 15)
#define  MTK_MSDC_PAD_TUNE_CMDRDLY_MASK		(0x1F << 16)
#define	 MTK_MSDC_PAD_TUNE_CMDRRDLYSEL		(1U << 21)
#define  MTK_MSDC_PAD_TUNE_CMDRRDLY_MASK	(0x1F << 22)
#define  MTK_MSDC_PAD_TUNE_CLKTDLY_MASK		(0x1F << 27)

#define	MT_EMMC50_CFG0		(0x208)
#define  MT_EMMC50_CFG0_CFCSTS_SEL		(1U << 4)
#define	MT_EMMC50_CFG2		(0x21c)
#define  MT_EMMC50_CFG2_AXI_SET_LEN_MASK	(0x0F << 24)

#define	MT_MMC_DEFAULT_DTOC	40	/* data timeout counter. 65536x40 sclk. */
#define	MT_MSDC_VERSION	        (0x100)
#define	MT_MSDC_ECO_VER	        (0x104)
#define	MT_MMC_MAX_BD		1024
/*
 * The largest transfer we can map.  A buffer of arbitrary user pages needs
 * one descriptor per page plus one for a partial page at each end, so this,
 * and not the per descriptor limit, is what bounds a request.
 */
#define	MT_MMC_MAX_XFER		((MT_MMC_MAX_BD - 1) * PAGE_SIZE)
#define	MT_MSC_DMA_MAX_SIZE	(64 * 1024 - MMC_SECTOR_SIZE)

#define MT_MSC_INT_ERR_BITS	(MT_MSDC_INT_SDRCRCER | MT_MSDC_INT_SDACDRCRCER | \
			       MT_MSDC_INT_SDCTO | MT_MSDC_INT_SDACDCTO | \
			       MT_MSDC_INT_SDDCRCERR | MT_MSDC_INT_SDDTO)

#define	MT_MMC_LOCK(_sc)	mtx_lock(&(_sc)->mt_mtx)
#define	MT_MMC_UNLOCK(_sc)	mtx_unlock(&(_sc)->mt_mtx)
#define	MT_MMC_READ_4(_sc, _reg)					\
       bus_read_4((_sc)->mem_res[MT_MMC_MEMRES], _reg)
#define MT_MMC_WRITE_4(_sc, _reg, _value)				\
       bus_write_4((_sc)->mem_res[MT_MMC_MEMRES], _reg, _value)

static struct ofw_compat_data compat_data[] = {
    { "mediatek,mt7623-mmc",	1 },
    { NULL,				0 }
};

struct mt_mmc_softc {
    device_t		        dev;
    struct resource		*mem_res[2];
    struct mtx		        mt_mtx;
    clk_t 		        mt_clk_source;
    clk_t 		        mt_clk_hclk;
    void *			mt_intrhand;
    int			        mt_bus_busy;
    int			        mt_resid;
    int			        mt_timeout;
    struct callout		mt_timeoutc;
    struct mmc_host		mt_host;
    struct mmc_helper	        mmc_helper;
#ifdef MMCCAM
    union ccb                   *ccb;
    struct mmc_sim		mmc_sim;
#else
    struct mmc_request          *mt_req;
#endif
    uint32_t		        mt_intr_mask;
    uint32_t		        mt_intr_wait;
    uint32_t		        mt_intr_seen;
    uint32_t		        socid;
    int			        mt_clock;
    device_t		        child;

    /* Fields required for DMA access. */
    bus_addr_t		        sc_dma_gpd_addr;
    bus_dmamap_t		sc_dma_gpd_map;
    bus_dma_tag_t		sc_dma_gpd_tag;
    void *			sc_dma_gpd;

    bus_addr_t		        sc_dma_bd_addr;
    bus_dma_tag_t		sc_dma_bd_tag;
    bus_dmamap_t		sc_dma_bd_map;
    void *			sc_dma_bd;
    bus_dma_tag_t		sc_dma_buf_tag;
    bus_dmamap_t		sc_dma_buf_map;
    int			        sc_dma_map_err;
    int			        sc_dma_loaded;

    uint32_t		        sc_dma_ctl;
};

/* DMA Generic Packet Descriptor (GPD) Format */
struct mtk_mmc_dma_gpd {
    uint8_t			gpd_cfg1;
#define	MTK_GPD_HWO		(1U << 0)	/* Hardware Own */
#define	MTK_GPD_BDP		(1U << 1)	/* Buffer Descriptor Present */
    uint8_t			gpd_chksum;	/* GPD Checksum */
    uint16_t		gpd_cfg2;
#define	MTK_GPD_INT		(1U << 1)	/* Interrupt Generation Mask */
    uint32_t		next_gpd;	/* Next DMA GPD Pointer */
    uint32_t		buf_addr;	/* Data Buffer Pointer/
				 DMA BD pointer */
    uint16_t		buf_len;	/* Data Buffer Length */
    uint8_t			desc_ext_len;	/* Descriptor Extension len */
    uint8_t			resv;
    uint32_t		arg;
    uint32_t		block_num;	/* SD BLOCK_NUMBER */
    uint32_t		cmd;
};

/* DMA Buffer Descriptor (BD) Format */
struct mtk_mmc_dma_bd {
    uint8_t			bd_cfg1;
#define	MTK_BD_EOL		(1U << 0)	/* End of List */
    uint8_t			bd_chksum;	/* Buffer Descriptor Checksum */
    uint16_t		bd_cfg2;
#define	MTK_BD_B		(1U << 1)	/* Block Padding */
#define	MTK_BD_D		(1U << 2)
    uint32_t		next_bd;	/* Next BD Pointer */
    uint32_t		buf_addr;	/* Data Buffer Pointer */
    uint16_t		buf_len;	/* Data Buffer Length */
    uint16_t		resv;
};

CTASSERT(sizeof(struct mtk_mmc_dma_gpd) == 28);
CTASSERT(sizeof(struct mtk_mmc_dma_bd) == 16);

static struct resource_spec mt_mmc_res_spec[] = {
    { SYS_RES_MEMORY,	        0,	RF_ACTIVE },
    { SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
    { -1,			0,	0 }
};

static int mt_mmc_probe(device_t);
static int mt_mmc_attach(device_t);
static int mt_mmc_detach(device_t);
static int mt_mmc_setup_dma(struct mt_mmc_softc *);
static void mt_mmc_teardown_dma(struct mt_mmc_softc *sc);
static int mt_mmc_reset(struct mt_mmc_softc *);
static void mt_mmc_req_done(struct mt_mmc_softc *sc);
static void mt_mmc_req_ok(struct mt_mmc_softc *sc);
static void mt_mmc_helper_cd_handler(device_t dev, bool present);

static void
mt_mmc_helper_cd_handler(device_t dev, bool present)
{
	struct mt_mmc_softc *sc;
	sc = device_get_softc(dev);
#ifdef MMCCAM
	mmc_cam_sim_discover(&sc->mmc_sim);
#else
	MT_MMC_LOCK(sc);
	if (present) {
		if (sc->child == NULL) {
			device_printf(sc->dev, "Card inserted\n");

sc->child = device_add_child(sc->dev, "mmc", DEVICE_UNIT_ANY);
			MT_MMC_UNLOCK(sc);
			if (sc->child) {
				device_set_ivars(sc->child, sc);
				(void)device_probe_and_attach(sc->child);
			}
		}
	} else {
		/* Card isn't present, detach if necessary */
		if (sc->child != NULL) {
			MT_MMC_UNLOCK(sc);
			device_printf(sc->dev, "Card removed\n");

			device_delete_child(sc->dev, sc->child);
			sc->child = NULL;
		}
	}
	MT_MMC_UNLOCK(sc);
#endif /* MMCCAM */
}

static void
mt_mmc_start_dma(struct mt_mmc_softc *sc)
{
	/* Set the address of the first descriptor */
	MT_MMC_WRITE_4(sc, MT_MSDC_DMA_SA, sc->sc_dma_gpd_addr);
	/* Enable and start the dma engine */
	MT_MMC_WRITE_4(sc, MT_MSDC_DMA_CTRL, sc->sc_dma_ctl);
}

/*
 * Stop the dma engine and release the transfer buffer.  The engine has to be
 * told to stop before the mapping goes away, it is free to keep writing until
 * it acknowledges that.
 */
static void
mt_mmc_finish_dma(struct mt_mmc_softc *sc, struct mmc_data *data)
{
	bus_dmasync_op_t sync_op;
	uint32_t val;
	int timeout;

	if (!sc->sc_dma_loaded)
		return;

	val = MT_MMC_READ_4(sc, MT_MSDC_DMA_CTRL);
	val |= MT_MSDC_DMA_CTRL_DMASTOP;
	MT_MMC_WRITE_4(sc, MT_MSDC_DMA_CTRL, val);
	for (timeout = 20000; timeout > 0; timeout--) {
		if ((MT_MMC_READ_4(sc, MT_MSDC_DMA_CTRL) &
		    MT_MSDC_DMA_CTRL_DMASTOP) == 0)
			break;
		DELAY(1);
	}
	for (timeout = 20000; timeout > 0; timeout--) {
		if ((MT_MMC_READ_4(sc, MT_MSDC_DMA_CFG) &
		    MT_MSDC_DMA_CFG_DMASTS) == 0)
			break;
		DELAY(1);
	}

	if (data != NULL && (data->flags & MMC_DATA_WRITE) != 0)
		sync_op = BUS_DMASYNC_POSTWRITE;
	else
		sync_op = BUS_DMASYNC_POSTREAD;
	bus_dmamap_sync(sc->sc_dma_buf_tag, sc->sc_dma_buf_map, sync_op);
	bus_dmamap_sync(sc->sc_dma_bd_tag, sc->sc_dma_bd_map,
	    BUS_DMASYNC_POSTWRITE);
	bus_dmamap_unload(sc->sc_dma_buf_tag, sc->sc_dma_buf_map);
	sc->sc_dma_loaded = 0;
}

static void
mt_mmc_req_done(struct mt_mmc_softc *sc)
{
	struct mmc_command *cmd;
	struct mmc_request *req;

	cmd = sc->mt_req->cmd;

	/* Covers the error and the timeout paths as well. */
	mt_mmc_finish_dma(sc, cmd->data);

	/* Reset the in case of errors */
	if (cmd->error != MMC_ERR_NONE)
		mt_mmc_reset(sc);

	callout_stop(&sc->mt_timeoutc);

	sc->mt_resid = 0;
	sc->sc_dma_map_err = 0;
	sc->mt_intr_wait = 0;

	sc->mt_intr_seen = 0;

	req = sc->mt_req;
	sc->mt_req = NULL;
	req->done(req);
}

static int
mt_mmc_reset(struct mt_mmc_softc *sc)
{
	int timeout;
	uint32_t val;

	/* Reset */
	val = MT_MMC_READ_4(sc, MT_MSDC_CFG);
	val |= MT_MSDC_CFG_RST;
	MT_MMC_WRITE_4(sc, MT_MSDC_CFG, val);
	timeout = 1000;
	while (--timeout > 0) {
		if ((MT_MMC_READ_4(sc, MT_MSDC_CFG) & MT_MSDC_CFG_RST) == 0)
			break;
		DELAY(1000);
	}
	if (timeout == 0)
		return (ETIMEDOUT);

	/* Clear FIFO */
	val = MT_MMC_READ_4(sc, MT_MSDC_FIFOCS);
	val |= MT_MSDC_FIFOCS_FIFOCLR;
	MT_MMC_WRITE_4(sc, MT_MSDC_FIFOCS, val);
	timeout = 100;
	while (--timeout > 0) {
		if ((MT_MMC_READ_4(sc, MT_MSDC_FIFOCS) &
		     MT_MSDC_FIFOCS_FIFOCLR) == 0)
			break;
		DELAY(100);
	}
	if (timeout == 0) {
		return (ETIMEDOUT);
	}

	/*
	 * Mask and acknowledge everything the boot loader left behind,
	 * otherwise a pending interrupt fires as soon as the handler is
	 * installed.
	 */
	MT_MMC_WRITE_4(sc, MT_MSDC_INTEN, 0);
	MT_MMC_WRITE_4(sc, MT_MSDC_INT, MT_MMC_READ_4(sc, MT_MSDC_INT));

	/* Remember interrupts we always want */
	sc->mt_intr_mask = MT_MSC_INT_ERR_BITS;

	return (0);
}

static void
mt_mmc_intr(void *arg)
{
	struct mt_mmc_softc *sc;
	struct mmc_data *data;
	uint32_t rint;

	sc = (struct mt_mmc_softc *)arg;
	MT_MMC_LOCK(sc);
	rint  = MT_MMC_READ_4(sc, MT_MSDC_INT);

	if (sc->mt_req == NULL) {
		device_printf(sc->dev,
		    "Spurious interrupt - no active request, rint: 0x%08X\n",
		    rint);
		goto end;
	}
	if (rint & MT_MSC_INT_ERR_BITS) {
		device_printf(sc->dev,
		    "controller error, rint %#x stat %#x\n",
		    rint, MT_MMC_READ_4(sc, MT_MSDC_INTEN));

		if (rint & (MT_MSDC_INT_SDCTO | MT_MSDC_INT_SDACDCTO |
			    MT_MSDC_INT_SDDTO))
			sc->mt_req->cmd->error = MMC_ERR_TIMEOUT;
		else
			sc->mt_req->cmd->error = MMC_ERR_FAILED;
		mt_mmc_req_done(sc);
		goto end;
	}

	data = sc->mt_req->cmd->data;

	/* The card has answered, the data phase can start. */
	if (data != NULL && (rint & MT_MSDC_INT_SDCRDY) != 0)
		mt_mmc_start_dma(sc);

	if (data != NULL && (rint & MT_MSDC_INT_SDXFCPL) != 0) {
		mt_mmc_finish_dma(sc, data);
		sc->mt_resid = data->len >> 2;
	}
	sc->mt_intr_seen |= rint;
	if ((sc->mt_intr_seen & sc->mt_intr_wait) == sc->mt_intr_wait)
		mt_mmc_req_ok(sc);
end:
	MT_MMC_WRITE_4(sc, MT_MSDC_INT, rint);
	MT_MMC_UNLOCK(sc);
}

static void
mt_mmc_req_ok(struct mt_mmc_softc *sc)
{
	struct mmc_command *cmd;
	cmd = sc->mt_req->cmd;

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[0] = MT_MMC_READ_4(sc, MT_SDC_RESP3);
			cmd->resp[1] = MT_MMC_READ_4(sc, MT_SDC_RESP2);
			cmd->resp[2] = MT_MMC_READ_4(sc, MT_SDC_RESP1);
			cmd->resp[3] = MT_MMC_READ_4(sc, MT_SDC_RESP0);
		} else {
			cmd->resp[0] = MT_MMC_READ_4(sc, MT_SDC_RESP0);
		}
	}

	/* All data has been transferred ? */
	if (cmd->data != NULL && (sc->mt_resid << 2) < cmd->data->len)
		cmd->error = MMC_ERR_FAILED;
	mt_mmc_req_done(sc);
}

static int
mt_mmc_read_ivar(device_t bus, device_t child, int which,
		  uintptr_t *result)
{
	struct mt_mmc_softc *sc;

	sc = device_get_softc(bus);
	switch (which) {
		case MMCBR_IVAR_BUS_MODE:
			*(int *)result = sc->mt_host.ios.bus_mode;
			break;
		case MMCBR_IVAR_BUS_WIDTH:
			*(int *)result = sc->mt_host.ios.bus_width;
			break;
		case MMCBR_IVAR_CHIP_SELECT:
			*(int *)result = sc->mt_host.ios.chip_select;
			break;
		case MMCBR_IVAR_CLOCK:
			*(int *)result = sc->mt_host.ios.clock;
			break;
		case MMCBR_IVAR_F_MIN:
			*(int *)result = sc->mt_host.f_min;
			break;
		case MMCBR_IVAR_F_MAX:
			*(int *)result = sc->mt_host.f_max;
			break;
		case MMCBR_IVAR_HOST_OCR:
			*(int *)result = sc->mt_host.host_ocr;
			break;
		case MMCBR_IVAR_MODE:
			*(int *)result = sc->mt_host.mode;
			break;
		case MMCBR_IVAR_OCR:
			*(int *)result = sc->mt_host.ocr;
			break;
		case MMCBR_IVAR_POWER_MODE:
			*(int *)result = sc->mt_host.ios.power_mode;
			break;
		case MMCBR_IVAR_VDD:
			*(int *)result = sc->mt_host.ios.vdd;
			break;
		case MMCBR_IVAR_VCCQ:
			*(int *)result = sc->mt_host.ios.vccq;
			break;
		case MMCBR_IVAR_CAPS:
			*(int *)result = sc->mt_host.caps;
			break;
		case MMCBR_IVAR_TIMING:
			*(int *)result = sc->mt_host.ios.timing;
			break;
		case MMCBR_IVAR_MAX_DATA:
			*(int *)result = MT_MMC_MAX_XFER / MMC_SECTOR_SIZE;
			break;
		default:
			return (EINVAL);
	}

	return (0);
}

static int
mt_mmc_write_ivar(device_t bus, device_t child, int which,
		   uintptr_t value)
{
	struct mt_mmc_softc *sc;

	sc = device_get_softc(bus);
	switch (which) {
		case MMCBR_IVAR_BUS_MODE:
			sc->mt_host.ios.bus_mode = value;
			break;
		case MMCBR_IVAR_BUS_WIDTH:
			sc->mt_host.ios.bus_width = value;
			break;
		case MMCBR_IVAR_CHIP_SELECT:
			sc->mt_host.ios.chip_select = value;
			break;
		case MMCBR_IVAR_CLOCK:
			sc->mt_host.ios.clock = value;
			break;
		case MMCBR_IVAR_MODE:
			sc->mt_host.mode = value;
			break;
		case MMCBR_IVAR_OCR:
			sc->mt_host.ocr = value;
			break;
		case MMCBR_IVAR_POWER_MODE:
			sc->mt_host.ios.power_mode = value;
			break;
		case MMCBR_IVAR_VDD:
			sc->mt_host.ios.vdd = value;
			break;
		case MMCBR_IVAR_VCCQ:
			sc->mt_host.ios.vccq = value;
			break;
		case MMCBR_IVAR_TIMING:
			sc->mt_host.ios.timing = value;
			break;
			/* These are read-only */
		case MMCBR_IVAR_CAPS:
		case MMCBR_IVAR_HOST_OCR:
		case MMCBR_IVAR_F_MIN:
		case MMCBR_IVAR_F_MAX:
		case MMCBR_IVAR_MAX_DATA:
		default:
			return (EINVAL);
	}

	return (0);
}

static uint8_t
mt_mmc_chksum_calcs(uint8_t *buf, uint32_t len)
{
	uint32_t i, sum = 0;

	for (i = 0; i < len; i++)
		sum += buf[i];

	return (0xFF - (uint8_t)sum);
}

static void
mt_mmc_dma_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int err)
{
	struct mt_mmc_softc *sc;
	struct mtk_mmc_dma_gpd *dma_gpd;
	struct mtk_mmc_dma_bd *dma_bd;
	uint32_t dma_bd_addr;
	int i;

	sc = (struct mt_mmc_softc *)arg;
	sc->sc_dma_map_err = err;

	if (err)
		return;

	dma_bd = sc->sc_dma_bd;
	dma_bd_addr = sc->sc_dma_bd_addr;

	for (i = 0; i < nsegs; i++) {
		dma_bd[i].bd_cfg2 = 0;
		dma_bd_addr += sizeof(struct mtk_mmc_dma_bd);
		dma_bd[i].next_bd = dma_bd_addr;
		dma_bd[i].buf_addr = segs[i].ds_addr;
		dma_bd[i].buf_len = segs[i].ds_len;
		if (i == nsegs - 1) {
			dma_bd[i].bd_cfg1 = MTK_BD_EOL;	/* setup the last */
		} else {
			dma_bd[i].bd_cfg1 = 0;
		}
		dma_bd[i].bd_chksum = 0;	/* checksume need to clear first */
		dma_bd[i].bd_chksum = mt_mmc_chksum_calcs(
		    (uint8_t *)(&dma_bd[i]), 16);
	}

	dma_gpd = sc->sc_dma_gpd;
	dma_gpd->gpd_cfg2 = 0;
	dma_gpd->gpd_cfg1 = MTK_GPD_HWO | MTK_GPD_BDP;	/* hw will clear HWO */
	dma_gpd->next_gpd = sc->sc_dma_gpd_addr
			    + sizeof(struct mtk_mmc_dma_gpd);
	dma_gpd->buf_addr = sc->sc_dma_bd_addr;
	dma_gpd->gpd_chksum = 0;	/* checksume need to clear first. */
	dma_gpd->gpd_chksum = mt_mmc_chksum_calcs((uint8_t *)dma_gpd, 16);
}

static int
mt_mmc_prepare_dma(struct mt_mmc_softc *sc)
{
	bus_dmasync_op_t sync_op;
	int error;
	struct mmc_command *cmd;
	uint32_t val;

	cmd = sc->mt_req->cmd;
	if (cmd->data->len > MT_MSC_DMA_MAX_SIZE * MT_MMC_MAX_BD)
		return (EFBIG);
	error = bus_dmamap_load(sc->sc_dma_buf_tag, sc->sc_dma_buf_map,
	    cmd->data->data, cmd->data->len, mt_mmc_dma_cb, sc,
	    BUS_DMA_NOWAIT);
	if (error)
		return (error);
	if (sc->sc_dma_map_err) {
		bus_dmamap_unload(sc->sc_dma_buf_tag, sc->sc_dma_buf_map);
		return (sc->sc_dma_map_err);
	}
	sc->sc_dma_loaded = 1;

	if (cmd->data->flags & MMC_DATA_WRITE)
		sync_op = BUS_DMASYNC_PREWRITE;
	else
		sync_op = BUS_DMASYNC_PREREAD;
	bus_dmamap_sync(sc->sc_dma_buf_tag, sc->sc_dma_buf_map, sync_op);
bus_dmamap_sync(sc->sc_dma_bd_tag, sc->sc_dma_bd_map, BUS_DMASYNC_PREWRITE);

	/* Setup default DMA parameters for Descriptor DMA mode */
	sc->sc_dma_ctl = MT_MSDC_DMA_CTRL_DMAMOD |
			 MT_MSDC_DMA_CTRL_BSTSZ_64B | MT_MSDC_DMA_CTRL_DMASTART;

	/* CLear POI mode */
	val = MT_MMC_READ_4(sc, MT_MSDC_CFG);
	val &= ~MT_MSDC_CFG_PIO;
	MT_MMC_WRITE_4(sc, MT_MSDC_CFG, val);

	/* Enambe Description DMA mode */
	val = MT_MMC_READ_4(sc, MT_MSDC_DMA_CFG);
	val |= MT_MSDC_DMA_CFG_DSCPCSEN;
	MT_MMC_WRITE_4(sc, MT_MSDC_DMA_CFG, val);

	return (0);
}

static void
mt_mmc_timeout(void *arg)
{
	struct mt_mmc_softc *sc;

	sc = (struct mt_mmc_softc *)arg;
	if (sc->mt_req != NULL) {
		device_printf(sc->dev,
		    "controller timeout, msdc_int %#x msdc_inten %#x\n",
		    MT_MMC_READ_4(sc, MT_MSDC_INT),
		    MT_MMC_READ_4(sc, MT_MSDC_INTEN));
		sc->mt_req->cmd->error = MMC_ERR_TIMEOUT;
		mt_mmc_req_done(sc);
	} else
		device_printf(sc->dev,
		    "Spurious timeout - no active request\n");
}

static int
mt_mmc_request(device_t bus, device_t child, struct mmc_request *req)
{
	struct mt_mmc_softc *sc;
	struct mmc_command *cmd;
	uint32_t iwait, cmdr;
	int blksz, error, tout = 1000;
	uint32_t val;

	sc = device_get_softc(bus);

	MT_MMC_LOCK(sc);
	if (sc->mt_req != NULL) {
		MT_MMC_UNLOCK(sc);
		return (EBUSY);
	}

	/* Start with template value */
	sc->mt_req = req;
	cmd = req->cmd;
	cmd->error = MMC_ERR_NONE;
	cmdr = cmd->opcode;
	sc->mt_resid = 0;
	sc->mt_intr_seen = 0;
	iwait = MT_MSDC_INT_SDCRDY;

	/* Configure response format */
	switch (MMC_RSP(cmd->flags)) {
		case MMC_RSP_R1:
			cmdr |= MT_SDC_CMD_RSPTYP_R1;
			break;
		case MMC_RSP_R1B:
			cmdr |= MT_SDC_CMD_RSPTYP_R1B;
			break;
		case MMC_RSP_R2:
			cmdr |= MT_SDC_CMD_RSPTYP_R2;
			break;
		case MMC_RSP_R3:
			cmdr |= MT_SDC_CMD_RSPTYP_R3;
			break;
	};

	if (cmd->data != NULL) {
		if (cmd->data->flags & MMC_DATA_MULTI) {
			cmdr |= MT_SDC_CMD_ACMD;
			cmdr |= MT_SDC_CMD_DTYPE_MULTI;
			iwait |= MT_MSDC_INT_SDACDCRDY;
		}
		else if (cmd->data->flags & MMC_DATA_STREAM)
		{
			cmdr |= MT_SDC_CMD_DTYPE_STREAM;
			device_printf(sc->dev,
			    "MMC_DATA_STREAM\n");
		}
		else
			cmdr |= MT_SDC_CMD_DTYPE_SINGLE;

		if (cmd->data->flags & MMC_DATA_WRITE)
			cmdr |= MT_SDC_CMD_RW;

		blksz = min(cmd->data->len, MMC_SECTOR_SIZE);
		cmdr |= blksz << MT_SDC_CMD_LEN_SHIFT;
		MT_MMC_WRITE_4(sc, MT_SDC_BLK_NUM, (cmd->data->len / blksz));

		error = mt_mmc_prepare_dma(sc);
		if (error != 0) {
			device_printf(sc->dev,
			    "cannot map a %d byte transfer: %d\n",
			    cmd->data->len, error);
			cmd->error = error == EFBIG ? MMC_ERR_INVALID :
			    MMC_ERR_NO_MEMORY;
			sc->mt_req = NULL;
			sc->mt_intr_wait = 0;
			MT_MMC_UNLOCK(sc);
			req->done(req);
			return (0);
		}

		iwait |= MT_MSDC_INT_SDXFCPL;
	}

	if (cmd->opcode == MMC_STOP_TRANSMISSION) {
		cmdr |= (MT_SDC_CMD_STOP);
		cmdr &= ~(0x0FFF << MT_SDC_CMD_LEN_SHIFT);
	}

	/* Is SD Command line or SD controller busy */
	if (cmd->opcode == MMC_SEND_STATUS) {
		while (MT_MMC_READ_4(sc, MT_SDC_STS) & MT_SDC_STS_CMDBSY) {
			if (tout-- < 0) {
				device_printf(sc->dev,
				    "SD command line busy: before CMD<%u>\n",
				    cmd->opcode);
				break;
			}
		}
	} else {
		while (MT_MMC_READ_4(sc, MT_SDC_STS) & MT_SDC_STS_SDCBSY) {
			if (tout-- < 0) {
				device_printf(sc->dev,
				    "SD controller busy: before CMD<%u>\n",
				    cmd->opcode);
				break;
			}
		}
	}

	if (tout < 0) {
		cmd->error = ETIMEDOUT;
		mt_mmc_timeout(sc);
		MT_MMC_UNLOCK(sc);
		return (0);
	}

	sc->mt_intr_wait = iwait;
	val = MT_MMC_READ_4(sc, MT_MSDC_INTEN);
	val |= sc->mt_intr_mask | iwait;
	MT_MMC_WRITE_4(sc, MT_MSDC_INTEN, val);

	MT_MMC_WRITE_4(sc, MT_SDC_ARG, cmd->arg);
	MT_MMC_WRITE_4(sc, MT_SDC_CMD, cmdr);

	callout_reset(&sc->mt_timeoutc, sc->mt_timeout * hz,
	    mt_mmc_timeout, sc);
	MT_MMC_UNLOCK(sc);

	return (0);
}

static int
mt_mmc_config_clock(struct mt_mmc_softc *sc, uint32_t freq)
{
	uint32_t mclk;
	uint64_t hclk;
	uint64_t sclk;
	uint32_t div;
	int mode;
	uint32_t val;
	int timeout;

	if (freq == 0) {
		/* Just stop the card clock, the divider stays as it is. */
		val = MT_MMC_READ_4(sc, MT_MSDC_CFG);
		val &= ~MT_MSDC_CFG_CCKPD;
		MT_MMC_WRITE_4(sc, MT_MSDC_CFG, val);
		return (0);
	}

	clk_get_freq(sc->mt_clk_source, &sclk);
	clk_get_freq(sc->mt_clk_hclk, &hclk);

	if (freq >= sclk) {
		/* Use msdc source clock as bus clock */
		mode = 1;
		div  = 0;
		mclk = sclk;
	} else {
		/* Use clock divider msdc source clock */
		mode = 0;
		if (freq >= (sclk >> 1)) {
			/* divider 1/2 */
			div = 0;
			mclk = sclk >> 1;
		} else {
			/* divider 1/(n * 4) n: 1 - 255 */
			div = (sclk + ((freq << 2) - 1)) / (freq << 2);
			mclk = (sclk >> 2) / div;
		}
	}

	if (bootverbose)
		device_printf(sc->dev,
		    "%s, mclk %u sclk %ju hclk %ju div %u mode %d\n",
		    __func__, mclk, (uintmax_t)sclk, (uintmax_t)hclk, div,
		    mode);

	/*
	 * Stop the card clock while the divider is reprogrammed.  The source
	 * clock has to keep running: MT_MSDC_CFG is only readable and
	 * writable while it does, and gating it here would read the register
	 * back as zero and wipe out the mode bits below.
	 */
	val = MT_MMC_READ_4(sc, MT_MSDC_CFG);
	val &= ~MT_MSDC_CFG_CCKPD;
	MT_MMC_WRITE_4(sc, MT_MSDC_CFG, val);

	val &= ~MT_MSDC_CFG_CCKMD_MASK;
	val |= (mode << MT_MSDC_CFG_CCKMD_SHIFT) & MT_MSDC_CFG_CCKMD_MASK;

	val &= ~MT_MSDC_CFG_CCKDIV_MASK;
	val |= (div << MT_MSDC_CFG_CCKDIV_SHIFT)
	       & MT_MSDC_CFG_CCKDIV_MASK;

	MT_MMC_WRITE_4(sc, MT_MSDC_CFG, val);

	timeout = 100000;
	while ((MT_MMC_READ_4(sc, MT_MSDC_CFG) & MT_MSDC_CFG_CCKSB) == 0) {
		if (--timeout == 0) {
			device_printf(sc->dev, "card clock not stable\n");
			return (ETIMEDOUT);
		}
		DELAY(10);
	}

	val = MT_MMC_READ_4(sc, MT_MSDC_CFG);
	val |= MT_MSDC_CFG_CCKPD;
	MT_MMC_WRITE_4(sc, MT_MSDC_CFG, val);

	return (0);
}

static int
mt_mmc_update_ios(device_t bus, device_t child)
{
	struct mt_mmc_softc *sc;
	struct mmc_ios *ios;
	uint32_t buswd;
	uint32_t val;

	sc = device_get_softc(bus);

	ios = &sc->mt_host.ios;

	device_printf(bus, "%s Setting up clk %u bus_width %d, timing: %d\n",
	    __func__, ios->clock, ios->bus_width, ios->timing);

	/* Set the bus width. */
	switch (ios->bus_width) {
		case bus_width_1:
			buswd = MT_SDC_CFG_BUSWD_1BIT;
			break;
		case bus_width_4:
			buswd = MT_SDC_CFG_BUSWD_4BIT;
			break;
		case bus_width_8:
			buswd = MT_SDC_CFG_BUSWD_8BIT;
			break;
		default:
			return (EINVAL);
	}
	val = MT_MMC_READ_4(sc, MT_SDC_CFG);
	val &= ~MT_SDC_CFG_BUSWD_MASK;
	val |= buswd & MT_SDC_CFG_BUSWD_MASK;
	MT_MMC_WRITE_4(sc, MT_SDC_CFG, val);

	switch (ios->power_mode) {
		case power_on:
			break;
		case power_off:
			if (sc->mmc_helper.vmmc_supply)
				regulator_disable(sc->mmc_helper.vmmc_supply);
			if (sc->mmc_helper.vqmmc_supply)
				regulator_disable(sc->mmc_helper.vqmmc_supply);
			break;
		case power_up:
			if (sc->mmc_helper.vmmc_supply)
				regulator_enable(sc->mmc_helper.vmmc_supply);
			if (sc->mmc_helper.vqmmc_supply)
				regulator_enable(sc->mmc_helper.vqmmc_supply);
			break;
	};

	if (ios->clock != sc->mt_clock) {
		sc->mt_clock = ios->clock;
		mt_mmc_config_clock(sc, ios->clock);
	}

	return (0);
}

static int
mt_mmc_get_ro(device_t bus, device_t child)
{
	struct mt_mmc_softc *sc;

	sc = device_get_softc(bus);

	return (mmc_fdt_gpio_get_readonly(&sc->mmc_helper));
}

static int
mt_mmc_switch_vccq(device_t bus, device_t child)
{
	struct mt_mmc_softc *sc;
	int uvolt, err;

	sc = device_get_softc(bus);

	if (sc->mmc_helper.vqmmc_supply == NULL)
		return (EOPNOTSUPP);

	switch (sc->mt_host.ios.vccq) {
		case vccq_180:
			uvolt = 1800000;
			break;
		case vccq_330:
			uvolt = 3300000;
			break;
		default:
			return (EINVAL);
	}

	err = regulator_set_voltage(sc->mmc_helper.vqmmc_supply, uvolt, uvolt);
	if (err != 0) {
		device_printf(sc->dev,
		    "Cannot set vqmmc to %d<->%d\n",
		    uvolt,
		    uvolt);
		return (err);
	}

	return (0);
}

static void
mt_mmc_dma_desc_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int err)
{
	bus_addr_t *addr;

	addr = arg;
	if (err != 0)
		return;
	*addr = segs[0].ds_addr;
}

/*
 * The controller walks a list of generic packet descriptors, each of which
 * points at a list of buffer descriptors describing the segments of one
 * transfer.  Two GPDs have to be allocated even though only the first one is
 * ever used: its "next" pointer has to reference a second descriptor for the
 * engine to accept the list.
 */
static int
mt_mmc_setup_dma(struct mt_mmc_softc *sc)
{
	bus_addr_t addr;
	int error;

	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), 16, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
	    2 * sizeof(struct mtk_mmc_dma_gpd), 1,
	    2 * sizeof(struct mtk_mmc_dma_gpd), 0, NULL, NULL,
	    &sc->sc_dma_gpd_tag);
	if (error != 0) {
		device_printf(sc->dev, "cannot create the dma gpd tag\n");
		return (error);
	}
	error = bus_dmamem_alloc(sc->sc_dma_gpd_tag, &sc->sc_dma_gpd,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT,
	    &sc->sc_dma_gpd_map);
	if (error != 0) {
		device_printf(sc->dev, "cannot allocate the dma gpd list\n");
		return (error);
	}
	addr = 0;
	error = bus_dmamap_load(sc->sc_dma_gpd_tag, sc->sc_dma_gpd_map,
	    sc->sc_dma_gpd, 2 * sizeof(struct mtk_mmc_dma_gpd),
	    mt_mmc_dma_desc_cb, &addr, BUS_DMA_NOWAIT);
	if (error != 0 || addr == 0) {
		device_printf(sc->dev, "cannot load the dma gpd list\n");
		return (error != 0 ? error : ENOMEM);
	}
	sc->sc_dma_gpd_addr = addr;

	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), 16, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
	    MT_MMC_MAX_BD * sizeof(struct mtk_mmc_dma_bd), 1,
	    MT_MMC_MAX_BD * sizeof(struct mtk_mmc_dma_bd), 0, NULL, NULL,
	    &sc->sc_dma_bd_tag);
	if (error != 0) {
		device_printf(sc->dev, "cannot create the dma bd tag\n");
		return (error);
	}
	error = bus_dmamem_alloc(sc->sc_dma_bd_tag, &sc->sc_dma_bd,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT,
	    &sc->sc_dma_bd_map);
	if (error != 0) {
		device_printf(sc->dev, "cannot allocate the dma bd list\n");
		return (error);
	}
	addr = 0;
	error = bus_dmamap_load(sc->sc_dma_bd_tag, sc->sc_dma_bd_map,
	    sc->sc_dma_bd, MT_MMC_MAX_BD * sizeof(struct mtk_mmc_dma_bd),
	    mt_mmc_dma_desc_cb, &addr, BUS_DMA_NOWAIT);
	if (error != 0 || addr == 0) {
		device_printf(sc->dev, "cannot load the dma bd list\n");
		return (error != 0 ? error : ENOMEM);
	}
	sc->sc_dma_bd_addr = addr;

	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), 4, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
	    MT_MMC_MAX_XFER, MT_MMC_MAX_BD, MT_MSC_DMA_MAX_SIZE, 0,
	    NULL, NULL, &sc->sc_dma_buf_tag);
	if (error != 0) {
		device_printf(sc->dev, "cannot create the dma buffer tag\n");
		return (error);
	}
	error = bus_dmamap_create(sc->sc_dma_buf_tag, 0, &sc->sc_dma_buf_map);
	if (error != 0) {
		device_printf(sc->dev, "cannot create the dma buffer map\n");
		return (error);
	}

	return (0);
}

static void
mt_mmc_teardown_dma(struct mt_mmc_softc *sc)
{
	bus_dmamap_unload(sc->sc_dma_gpd_tag, sc->sc_dma_gpd_map);
	bus_dmamem_free(sc->sc_dma_gpd_tag, sc->sc_dma_gpd, sc->sc_dma_gpd_map);
	if (bus_dma_tag_destroy(sc->sc_dma_gpd_tag) != 0)
		device_printf(sc->dev, "Cannot destroy the dma gpd tag\n");

	bus_dmamap_unload(sc->sc_dma_bd_tag, sc->sc_dma_bd_map);
	bus_dmamem_free(sc->sc_dma_bd_tag, sc->sc_dma_bd, sc->sc_dma_bd_map);
	if (bus_dma_tag_destroy(sc->sc_dma_bd_tag) != 0)
		device_printf(sc->dev, "Cannot destroy the dma bd tag\n");

	bus_dmamap_unload(sc->sc_dma_buf_tag, sc->sc_dma_buf_map);
	bus_dmamap_destroy(sc->sc_dma_buf_tag, sc->sc_dma_buf_map);
	if (bus_dma_tag_destroy(sc->sc_dma_buf_tag) != 0)
		device_printf(sc->dev, "Cannot destroy the dma buf tag\n");
}

static int
mt_mmc_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Mediatek 7623 MMC/SD controller");

	return (BUS_PROBE_DEFAULT);
}

/*
 * Bring the controller into a known state.  This follows msdc_init_hw() of
 * the Linux mtk-sd driver for the mt2701/mt7623 flavour of the block: a
 * 12 bit clock divider, the asynchronous FIFO in the response path and pad
 * based data tuning.  The boot loader leaves the sample edges and the pad
 * delays set up for its own use, so everything the command path depends on
 * is written here rather than inherited.
 */
static int
mt_mmc_init_hw(struct mt_mmc_softc *sc)
{
	uint32_t val;

	/*
	 * Set to MMC/SD mode and let the internal clock run freely.  The
	 * reset below only completes while that clock is running, so
	 * MT_MSDC_CFG_CCKPD has to be set before it is started.
	 */
	val = MT_MMC_READ_4(sc, MT_MSDC_CFG);
	val |= MT_MSDC_CFG_MSDC | MT_MSDC_CFG_CCKPD;
	MT_MMC_WRITE_4(sc, MT_MSDC_CFG, val);

	if (mt_mmc_reset(sc) != 0) {
		device_printf(sc->dev, "cannot reset the controller\n");
		return (ENXIO);
	}

	/* Drop the pad delays and sample edges the boot loader picked. */
	MT_MMC_WRITE_4(sc, MTK_MSDC_PAD_TUNE, 0);
	MT_MMC_WRITE_4(sc, MT_MSDC_IOCON, 0);

	/*
	 * Patch bit 0: odd length transfers on an 8 bit bus, no write
	 * monitor on the command register, transfer done interrupt after the
	 * descriptor update, a long R1b busy detection window, CRC timeout
	 * detection on writes and one stage of clock generator delay.
	 */
	val = MT_MSDC_PATCH_BIT0_PTCH01 | MT_MSDC_PATCH_BIT0_PTCH02 |
	    MT_MSDC_PATCH_BIT0_DESCUP_SEL | MT_MSDC_PATCH_BIT0_PTCH30;
	val |= (15 << 18) & MT_MSDC_PATCH_BIT0_PTCH18_MASK;
	val |= (1 << MT_MSDC_PATCH_BIT0_CKGEN_DLY_SHIFT) &
	    MT_MSDC_PATCH_BIT0_CKGEN_DLY_MASK;
	MT_MMC_WRITE_4(sc, MT_MSDC_PATCH_BIT0, val);

	/*
	 * Patch bit 1: one cycle turnaround for write data, CRC status and
	 * the command response, plus the clock gating defaults.  R1b busy is
	 * checked by software here, so ask the hardware not to wait for it.
	 */
	val = (1 << 0) & MT_MSDC_PATCH_BIT1_WRTA_MASK;
	val |= (1 << 3) & MT_MSDC_PATCH_BIT1_CMDTA_MASK;
	val |= MT_MSDC_PATCH_BIT1_GETCRCMARGIN;
	val |= MT_MSDC_PATCH_BIT1_DDR_CMD_FIX_SEL;
	val |= MT_MSDC_PATCH_BIT1_RSVD20_MASK;
	val |= MT_MSDC_PATCH_BIT1_AUTO_SYNCST_CLR;
	val |= MT_MSDC_PATCH_BIT1_MARK_POP_WATER;
	val |= MT_MSDC_PATCH_BIT1_LP_DCM_EN | MT_MSDC_PATCH_BIT1_RSVD3;
	val |= MT_MSDC_PATCH_BIT1_HGDMACKEN;
	val |= MT_MSDC_PATCH_BIT1_CLK_ENFEAT_MASK;
	if ((MT_MMC_READ_4(sc, MT_EMMC50_CFG2) &
	    MT_EMMC50_CFG2_AXI_SET_LEN_MASK) == 0)
		val |= MT_MSDC_PATCH_BIT1_SINGLE_BURST;
	MT_MMC_WRITE_4(sc, MT_MSDC_PATCH_BIT1, val);

	/*
	 * Patch bit 2: this block has the asynchronous FIFO, so route the
	 * command response and the CRC status through it, delay their enable
	 * signals by two cycles and stretch the response timeout to
	 * 65 + 16 * 3 cycles.  Without this a response can be missed and the
	 * command completes with a timeout instead.
	 */
	val = MT_MMC_READ_4(sc, MT_MSDC_PATCH_BIT2);
	val &= ~MT_MSDC_PATCH_BIT2_RESPWAIT_MASK;
	val |= (3 << MT_MSDC_PATCH_BIT2_RESPWAIT_SHIFT) &
	    MT_MSDC_PATCH_BIT2_RESPWAIT_MASK;
	val &= ~MT_MSDC_PATCH_BIT2_CFGRESP;
	val |= MT_MSDC_PATCH_BIT2_CFGCRCSTS;
	val &= ~(MT_MSDC_PATCH_BIT2_RESPSTSENSEL_MASK |
	    MT_MSDC_PATCH_BIT2_CRCSTSENSEL_MASK);
	val |= (2 << MT_MSDC_PATCH_BIT2_RESPSTSENSEL_SHIFT) &
	    MT_MSDC_PATCH_BIT2_RESPSTSENSEL_MASK;
	val |= (2U << MT_MSDC_PATCH_BIT2_CRCSTSENSEL_SHIFT) &
	    MT_MSDC_PATCH_BIT2_CRCSTSENSEL_MASK;
	MT_MMC_WRITE_4(sc, MT_MSDC_PATCH_BIT2, val);

	val = MT_MMC_READ_4(sc, MT_EMMC50_CFG0);
	val |= MT_EMMC50_CFG0_CFCSTS_SEL;
	MT_MMC_WRITE_4(sc, MT_EMMC50_CFG0, val);

	/* Sample the command response and the read data off the pads. */
	val = MT_MMC_READ_4(sc, MTK_MSDC_PAD_TUNE);
	val |= MTK_MSDC_PAD_TUNE_DATRRDLYSEL | MTK_MSDC_PAD_TUNE_CMDRRDLYSEL;
	MT_MMC_WRITE_4(sc, MTK_MSDC_PAD_TUNE, val);

	val = MT_MMC_READ_4(sc, MT_SDC_CFG);
	/* Enable SDIO mode, CMD5 is not answered without it. */
	val |= MT_SDC_CFG_SDIO;
	val &= ~MT_SDC_CFG_SDIOIDE;
	/* Card detection is done with a gpio, not by the controller. */
	val &= ~MT_SDC_CFG_ENWKUPINS;
	/* Configure to default data timeout */
	val &= ~MT_SDC_CFG_DTOC_MASK;
	val |= (MT_MMC_DEFAULT_DTOC << MT_SDC_CFG_DTOC_SHIFT)
	       & MT_SDC_CFG_DTOC_MASK;
	MT_MMC_WRITE_4(sc, MT_SDC_CFG, val);

	return (0);
}

static int
mt_mmc_attach(device_t dev)
{
	struct mt_mmc_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *tree;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;

#ifndef MMCCAM
	sc->mt_req = NULL;
#endif

	if (bus_alloc_resources(dev, mt_mmc_res_spec, sc->mem_res) != 0) {
		device_printf(dev, "Cannot allocate device resources\n");
		return (ENXIO);
	}

	if (bus_setup_intr(dev, sc->mem_res[MT_MMC_IRQRES],
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, mt_mmc_intr, sc,
	    &sc->mt_intrhand)) {
		if (sc->mem_res != NULL) {
bus_release_resources(dev, mt_mmc_res_spec, sc->mem_res);
			device_printf(dev, "Cannot setup interrupt handler\n");
			return (ENXIO);
		}

		device_printf(dev, "Cannot setup interrupt handler\n");
		return (ENXIO);
	}

	mtx_init(&sc->mt_mtx, device_get_nameunit(sc->dev), "mt_mmc",
	    MTX_DEF);
	callout_init_mtx(&sc->mt_timeoutc, &sc->mt_mtx, 0);

	error = clk_get_by_ofw_name(dev, 0, "source", &sc->mt_clk_source);
	if (error != 0) {
		device_printf(dev, "cannot get source clock\n");
		goto fail;
	}
	error = clk_enable(sc->mt_clk_source);
	if (error != 0) {
		device_printf(dev, "cannot enable source clock\n");
		goto fail;
	}

	error = clk_get_by_ofw_name(dev, 0, "hclk", &sc->mt_clk_hclk);
	if (error != 0) {
		device_printf(dev, "cannot get source clock\n");
		goto fail;
	}
	error = clk_enable(sc->mt_clk_hclk);
	if (error != 0) {
		device_printf(dev, "cannot enable hclk clock\n");
		goto fail;
	}

	sc->mt_timeout = 10;
	ctx = device_get_sysctl_ctx(dev);
	tree = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "req_timeout", CTLFLAG_RW,
	    &sc->mt_timeout, 0, "Request timeout in seconds");

	sc->mt_host.f_max = 25000000;
	sc->mt_host.f_min = 260000;
	sc->mt_host.host_ocr = MMC_OCR_320_330 | MMC_OCR_330_340;
	sc->mt_host.caps = MMC_CAP_HSPEED | MMC_CAP_SIGNALING_330;
	sc->mt_clock = 0;

	if (mt_mmc_init_hw(sc) != 0)
		goto fail;

	if (mt_mmc_setup_dma(sc) != 0)
		goto fail;

	/*
	 * Pick up bus-width, max-frequency, the regulators and the card
	 * detect gpio before the mmc bus is attached, it inspects the
	 * bridge as soon as it is there.
	 */
	mmc_fdt_parse(dev, 0, &sc->mmc_helper, &sc->mt_host);
	mmc_fdt_gpio_setup(dev, 0, &sc->mmc_helper, mt_mmc_helper_cd_handler);

	if (sc->child == NULL) {
		sc->child = device_add_child(sc->dev, "mmc", -1);
		if (sc->child) {
			device_set_ivars(sc->child, sc);
			(void)device_probe_and_attach(sc->child);
		}
	}

#ifdef MMCCAM
	sc->ccb = NULL;

	    if (mmc_cam_sim_alloc(dev, "mt_mmc", &sc->mmc_sim) != 0) {
		    device_printf(dev, "Cannot alloc cam sim\n");
		    goto fail;
	    }
#endif

	return (0);

fail:
	callout_drain(&sc->mt_timeoutc);
	mtx_destroy(&sc->mt_mtx);
	bus_teardown_intr(dev, sc->mem_res[MT_MMC_IRQRES], sc->mt_intrhand);
	if (sc->mem_res != NULL) {
		bus_release_resources(dev, mt_mmc_res_spec, sc->mem_res);
	}
	return (ENXIO);
}


static int
mt_mmc_detach(device_t dev)
{
	struct mt_mmc_softc *sc;
	device_t d;

	sc= device_get_softc(dev);

	mmc_fdt_gpio_teardown(&sc->mmc_helper);

	callout_drain(&sc->mt_timeoutc);

	MT_MMC_LOCK(sc);
	d = sc->child;
	sc->child = NULL;
	MT_MMC_UNLOCK(sc);
	device_delete_child(sc->dev, d);

	mt_mmc_teardown_dma(sc);

	mtx_destroy(&sc->mt_mtx);

	bus_teardown_intr(dev, sc->mem_res[MT_MMC_IRQRES], sc->mt_intrhand);
	bus_release_resources(dev, mt_mmc_res_spec, sc->mem_res);
	return (0);
}

static int
mtk_mmc_get_ro(device_t bus, device_t child)
{
	struct mt_mmc_softc *sc;

	sc = device_get_softc(bus);

	return (mmc_fdt_gpio_get_readonly(&sc->mmc_helper));
}

static int
mt_mmc_acquire_host(device_t bus, device_t child)
{
	struct mt_mmc_softc *sc;
	int error;

	sc = device_get_softc(bus);
	MT_MMC_LOCK(sc);
	while (sc->mt_bus_busy) {
		error = msleep(sc, &sc->mt_mtx, PCATCH, "mmchw", 0);
		if (error != 0) {
			MT_MMC_UNLOCK(sc);
			return (error);
		}
	}
	sc->mt_bus_busy++;
	MT_MMC_UNLOCK(sc);

	return (0);
}

static int
mt_mmc_release_host(device_t bus, device_t child)
{
	struct mt_mmc_softc *sc;

	sc = device_get_softc(bus);
	MT_MMC_LOCK(sc);
	sc->mt_bus_busy--;
	wakeup(sc);
	MT_MMC_UNLOCK(sc);

	return (0);
}

static device_method_t mt_mmc_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,		mt_mmc_probe),
    DEVMETHOD(device_attach,	mt_mmc_attach),
    DEVMETHOD(device_detach,	mt_mmc_detach),

    /* Bus interface */
    DEVMETHOD(bus_read_ivar,	mt_mmc_read_ivar),
    DEVMETHOD(bus_write_ivar,	mt_mmc_write_ivar),
    DEVMETHOD(bus_add_child,	bus_generic_add_child),

#ifndef MMCCAM
    /* MMC bridge interface */
    DEVMETHOD(mmcbr_update_ios,	mt_mmc_update_ios),
    DEVMETHOD(mmcbr_request,	mt_mmc_request),
    DEVMETHOD(mmcbr_get_ro,		mt_mmc_get_ro),
    DEVMETHOD(mmcbr_switch_vccq,	mt_mmc_switch_vccq),
    DEVMETHOD(mmcbr_acquire_host,	mt_mmc_acquire_host),
    DEVMETHOD(mmcbr_release_host,	mt_mmc_release_host),
#endif
    DEVMETHOD_END
};

static DEFINE_CLASS_0(mt_mmc, mt_mmc_driver, mt_mmc_methods,
sizeof(struct mt_mmc_softc));
DRIVER_MODULE(mt_mmc, simplebus, mt_mmc_driver, NULL, NULL);
#ifndef MMCCAM
MMC_DECLARE_BRIDGE(mt_mmc);
#endif
