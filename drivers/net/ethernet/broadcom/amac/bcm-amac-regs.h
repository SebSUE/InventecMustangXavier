/*
 * Copyright (C) 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __BCM_AMAC_REGS_H__
#define __BCM_AMAC_REGS_H__

#include <linux/types.h>


/* Offsets from GMAC_DEVCONTROL */
#define GMAC_INT_STATUS_ADDR        0x020
#define GMAC_INT_MASK_ADDR          0x024

#define GMAC_INTR_RECV_LAZY_ADDR    0x100
#define GMAC_PHY_CTRL_ADDR          0x188

#define GMAC_DMA_TX_CTRL_OFFSET     0x200
#define GMAC_DMA_TX_PTR_OFFSET      0x204
#define GMAC_DMA_TX_ADDR_LO_OFFSET  0x208
#define GMAC_DMA_TX_ADDR_HI_OFFSET  0x20c
#define GMAC_DMA_TX_STATUS0_OFFSET  0x210
#define GMAC_DMA_TX_STATUS1_OFFSET  0x214

#define GMAC_DMA_RX_CTRL_OFFSET     0x220
#define GMAC_DMA_RX_PTR_OFFSET      0x224
#define GMAC_DMA_RX_ADDR_LO_OFFSET  0x228
#define GMAC_DMA_RX_ADDR_HI_OFFSET  0x22c
#define GMAC_DMA_RX_STATUS0_OFFSET  0x230
#define GMAC_DMA_RX_STATUS1_OFFSET  0x234

#define UNIMAC_CMD_CFG_OFFSET       0x808

#define GMAC0_IRL_FRAMECOUNT_SHIFT  24

/* PHY registers */
#define GPHY_MII_CTRL_REG           0x00
#define GPHY_MII_CTRL_REG_PWR_MASK  0x800
#define GPHY_MII_CTRL_REG_RST_MASK  0x8000

#define GPHY_EXP_DATA_REG           0x15
#define GPHY_EXP_SELECT_REG         0x17

/* Switch Global Config registers @0x0301d194 */
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_BYPASS_SWITCH 13
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_BYPASS_SWITCH_WIDTH 1
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_BYPASS_SWITCH_RESETVALUE 0x0
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_MII1_2G_MODE 12
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_MII1_2G_MODE_WIDTH 1
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_MII1_2G_MODE_RESETVALUE 0x0
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_DIRECT_GATE_PORT_L 11
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_DIRECT_GATE_PORT_R 8
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_DIRECT_GATE_PORT_WIDTH 4
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_DIRECT_GATE_PORT_RESETVALUE 0x0
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_REVID_L 7
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_REVID_R 0
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_REVID_WIDTH 8
#define SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_REVID_RESETVALUE 0x01
#define SWITCH_GLOBAL_CONFIG__RESERVED_L 31
#define SWITCH_GLOBAL_CONFIG__RESERVED_R 14
#define SWITCH_GLOBAL_CONFIG_WIDTH 14
#define SWITCH_GLOBAL_CONFIG__WIDTH 14
#define SWITCH_GLOBAL_CONFIG_ALL_L 13
#define SWITCH_GLOBAL_CONFIG_ALL_R 0
#define SWITCH_GLOBAL_CONFIG__ALL_L 13
#define SWITCH_GLOBAL_CONFIG__ALL_R 0
#define SWITCH_GLOBAL_CONFIG_DATAMASK 0x00003fff
#define SWITCH_GLOBAL_CONFIG_RESETVALUE 0x1

/* CRMU Chip IO Pad Control @0x0301d0bc */
#define CRMU_CHIP_IO_PAD_CONTROL__CDRU_IOMUX_FORCE_PAD_IN 0
#define CRMU_CHIP_IO_PAD_CONTROL__CDRU_IOMUX_FORCE_PAD_IN_WIDTH 1
#define CRMU_CHIP_IO_PAD_CONTROL__CDRU_IOMUX_FORCE_PAD_IN_RESETVALUE 0x1
#define CRMU_CHIP_IO_PAD_CONTROL__RESERVED_L 31
#define CRMU_CHIP_IO_PAD_CONTROL__RESERVED_R 1
#define CRMU_CHIP_IO_PAD_CONTROL_WIDTH 1
#define CRMU_CHIP_IO_PAD_CONTROL__WIDTH 1
#define CRMU_CHIP_IO_PAD_CONTROL_ALL_L 0
#define CRMU_CHIP_IO_PAD_CONTROL_ALL_R 0
#define CRMU_CHIP_IO_PAD_CONTROL__ALL_L 0
#define CRMU_CHIP_IO_PAD_CONTROL__ALL_R 0
#define CRMU_CHIP_IO_PAD_CONTROL_DATAMASK 0x00000001
#define CRMU_CHIP_IO_PAD_CONTROL_RESETVALUE 0x1

/* AMAC IDM0 IO CONTROL @0x18110408 */
#define AMAC_IDM0_IO_CONTROL_DIRECT_BASE 0x408
#define AMAC_IDM0_IO_CONTROL_DIRECT__CT 31
#define AMAC_IDM0_IO_CONTROL_DIRECT__CT_WIDTH 1
#define AMAC_IDM0_IO_CONTROL_DIRECT__CT_RESETVALUE 0x0
#define AMAC_IDM0_IO_CONTROL_DIRECT__BYPASS_CT 30
#define AMAC_IDM0_IO_CONTROL_DIRECT__BYPASS_CT_WIDTH 1
#define AMAC_IDM0_IO_CONTROL_DIRECT__BYPASS_CT_RESETVALUE 0x0
#define AMAC_IDM0_IO_CONTROL_DIRECT__ARUSER_L 29
#define AMAC_IDM0_IO_CONTROL_DIRECT__ARUSER_R 25
#define AMAC_IDM0_IO_CONTROL_DIRECT__ARUSER_WIDTH 5
#define AMAC_IDM0_IO_CONTROL_DIRECT__ARUSER_RESETVALUE 0xf
#define AMAC_IDM0_IO_CONTROL_DIRECT__AWUSER_L 24
#define AMAC_IDM0_IO_CONTROL_DIRECT__AWUSER_R 20
#define AMAC_IDM0_IO_CONTROL_DIRECT__AWUSER_WIDTH 5
#define AMAC_IDM0_IO_CONTROL_DIRECT__AWUSER_RESETVALUE 0xf
#define AMAC_IDM0_IO_CONTROL_DIRECT__ARCACHE_L 19
#define AMAC_IDM0_IO_CONTROL_DIRECT__ARCACHE_R 16
#define AMAC_IDM0_IO_CONTROL_DIRECT__ARCACHE_WIDTH 4
#define AMAC_IDM0_IO_CONTROL_DIRECT__ARCACHE_RESETVALUE 0x0
#define AMAC_IDM0_IO_CONTROL_DIRECT__LEDSCAN_FLOP_STAGES_L 13
#define AMAC_IDM0_IO_CONTROL_DIRECT__LEDSCAN_FLOP_STAGES_R 11
#define AMAC_IDM0_IO_CONTROL_DIRECT__LEDSCAN_FLOP_STAGES_WIDTH 3
#define AMAC_IDM0_IO_CONTROL_DIRECT__LEDSCAN_FLOP_STAGES_RESETVALUE 0x0
#define AMAC_IDM0_IO_CONTROL_DIRECT__AWCACHE_L 10
#define AMAC_IDM0_IO_CONTROL_DIRECT__AWCACHE_R 7
#define AMAC_IDM0_IO_CONTROL_DIRECT__AWCACHE_WIDTH 4
#define AMAC_IDM0_IO_CONTROL_DIRECT__AWCACHE_RESETVALUE 0x0
#define AMAC_IDM0_IO_CONTROL_DIRECT__CLK_250_SEL 6
#define AMAC_IDM0_IO_CONTROL_DIRECT__CLK_250_SEL_WIDTH 1
#define AMAC_IDM0_IO_CONTROL_DIRECT__CLK_250_SEL_RESETVALUE 0x1
#define AMAC_IDM0_IO_CONTROL_DIRECT__DIRECT_GMII_MODE 5
#define AMAC_IDM0_IO_CONTROL_DIRECT__DIRECT_GMII_MODE_WIDTH 1
#define AMAC_IDM0_IO_CONTROL_DIRECT__DIRECT_GMII_MODE_RESETVALUE 0x0
#define AMAC_IDM0_IO_CONTROL_DIRECT__TX_CLK_OUT_INVERT_EN 4
#define AMAC_IDM0_IO_CONTROL_DIRECT__TX_CLK_OUT_INVERT_EN_WIDTH 1
#define AMAC_IDM0_IO_CONTROL_DIRECT__TX_CLK_OUT_INVERT_EN_RESETVALUE 0x0
#define AMAC_IDM0_IO_CONTROL_DIRECT__DEST_SYNC_MODE_EN 3
#define AMAC_IDM0_IO_CONTROL_DIRECT__DEST_SYNC_MODE_EN_WIDTH 1
#define AMAC_IDM0_IO_CONTROL_DIRECT__DEST_SYNC_MODE_EN_RESETVALUE 0x0
#define AMAC_IDM0_IO_CONTROL_DIRECT__SOURCE_SYNC_MODE_EN 2
#define AMAC_IDM0_IO_CONTROL_DIRECT__SOURCE_SYNC_MODE_EN_WIDTH 1
#define AMAC_IDM0_IO_CONTROL_DIRECT__SOURCE_SYNC_MODE_EN_RESETVALUE 0x1
#define AMAC_IDM0_IO_CONTROL_DIRECT__CLK_EN 0
#define AMAC_IDM0_IO_CONTROL_DIRECT__CLK_EN_WIDTH 1
#define AMAC_IDM0_IO_CONTROL_DIRECT__CLK_EN_RESETVALUE 0x1
#define AMAC_IDM0_IO_CONTROL_DIRECT__RESERVED_L 15
#define AMAC_IDM0_IO_CONTROL_DIRECT__RESERVED_R 14
#define AMAC_IDM0_IO_CONTROL_DIRECT_WIDTH 32
#define AMAC_IDM0_IO_CONTROL_DIRECT__WIDTH 32
#define AMAC_IDM0_IO_CONTROL_DIRECT_ALL_L 31
#define AMAC_IDM0_IO_CONTROL_DIRECT_ALL_R 0
#define AMAC_IDM0_IO_CONTROL_DIRECT__ALL_L 31
#define AMAC_IDM0_IO_CONTROL_DIRECT__ALL_R 0
#define AMAC_IDM0_IO_CONTROL_DIRECT_DATAMASK 0xffff3ffd
#define AMAC_IDM0_IO_CONTROL_DIRECT_RDWRMASK 0x0000c002
#define AMAC_IDM0_IO_CONTROL_DIRECT_RESETVALUE 0x1ef00045

/* register-specific flag definitions */

/* device control */
#define DC_TSM          0x00000002
#define DC_CFCO         0x00000004
#define DC_RLSS         0x00000008
#define DC_MROR         0x00000010
#define DC_FCM_MASK	    0x00000060
#define DC_FCM_SHIFT    5
#define DC_NAE          0x00000080
#define DC_TF           0x00000100
#define DC_RDS_MASK     0x00030000
#define DC_RDS_SHIFT    16
#define DC_TDS_MASK     0x000c0000
#define DC_TDS_SHIFT    18

/* device status */
#define DS_RBF       0x00000001
#define DS_RDF       0x00000002
#define DS_RIF       0x00000004
#define DS_TBF       0x00000008
#define DS_TDF       0x00000010
#define DS_TIF       0x00000020
#define DS_PO        0x00000040
#define DS_MM_MASK   0x00000300
#define DS_MM_SHIFT  8

/* bist status */
#define BS_MTF   0x00000001
#define BS_MRF   0x00000002
#define BS_TDB   0x00000004
#define BS_TIB   0x00000008
#define BS_TBF   0x00000010
#define BS_RDB   0x00000020
#define BS_RIB   0x00000040
#define BS_RBF   0x00000080
#define BS_URTF  0x00000100
#define BS_UTF   0x00000200
#define BS_URF   0x00000400

/* interrupt status and mask registers */
#define I_MRO      0x00000001
#define I_MTO      0x00000002
#define I_TFD      0x00000004
#define I_LS       0x00000008
#define I_MDIO     0x00000010
#define I_MR       0x00000020
#define I_MT       0x00000040
#define I_TO       0x00000080
#define I_PDEE     0x00000400
#define I_PDE      0x00000800
#define I_DE       0x00001000
#define I_RDU      0x00002000
#define I_RFO      0x00004000
#define I_XFU      0x00008000
#define I_RI       0x00010000
#define I_XI0      0x01000000
#define I_XI1      0x02000000
#define I_XI2      0x04000000
#define I_XI3      0x08000000
#define I_INTMASK  0x0f01fcff
#define I_ERRMASK  0x0000fc00
#define I_XI_ALL   (I_XI0 | I_XI1 | I_XI2 | I_XI3)

/* interrupt receive lazy */
#define IRL_TO_MASK  0x00ffffff
#define IRL_FC_MASK  0xff000000
#define IRL_FC_SHIFT  24

/* flow control thresholds */
#define FCT_TT_MASK  0x00000fff
#define FCT_RT_MASK  0x0fff0000
#define FCT_RT_SHIFT 16

/* txq aribter wrr thresholds */
#define WRRT_Q0T_MASK  0x000000ff
#define WRRT_Q1T_MASK  0x0000ff00
#define WRRT_Q1T_SHIFT 8
#define WRRT_Q2T_MASK  0x00ff0000
#define WRRT_Q2T_SHIFT 16
#define WRRT_Q3T_MASK  0xff000000
#define WRRT_Q3T_SHIFT 24

/* phy access */
#define PA_DATA_MASK  0x0000ffff
#define PA_ADDR_MASK  0x001f0000
#define PA_ADDR_SHIFT 16
#define PA_REG_MASK	  0x1f000000
#define PA_REG_SHIFT  24
#define PA_WRITE      0x20000000
#define PA_START      0x40000000

/* phy control */
#define PC_EPA_MASK   0x0000001f
#define PC_MCT_MASK   0x007f0000
#define PC_MCT_SHIFT  16
#define PC_MTE        0x00800000

/* rxq control */
#define RC_DBT_MASK   0x00000fff
#define RC_DBT_SHIFT  0
#define RC_PTE        0x00001000
#define RC_MDP_MASK   0x3f000000
#define RC_MDP_SHIFT  24

#define RC_MAC_DATA_PERIOD 9

/* txq control */
#define TC_DBT_MASK  0x00000fff
#define TC_DBT_SHIFT 0

/* clk control status */
#define CS_FA  0x00000001
#define CS_FH  0x00000002
#define CS_FI  0x00000004
#define CS_AQ  0x00000008
#define CS_HQ  0x00000010
#define CS_FC  0x00000020
#define CS_ER  0x00000100
#define CS_AA  0x00010000
#define CS_HA  0x00020000
#define CS_BA  0x00040000
#define CS_BH  0x00080000
#define CS_ES  0x01000000

/* command config */
#define CC_TE        0x00000001
#define CC_RE        0x00000002
#define CC_ES_MASK   0x0000000c
#define CC_ES_SHIFT  2
#define CC_PROM      0x00000010
#define CC_PAD_EN    0x00000020
#define CC_CF        0x00000040
#define CC_PF        0x00000080
#define CC_RPI       0x00000100
#define CC_TAI       0x00000200
#define CC_HD        0x00000400
#define CC_HD_SHIFT  10
#define CC_SR        0x00002000
#define CC_ML        0x00008000
#define CC_AE        0x00400000
#define CC_CFE       0x00800000
#define CC_NLC       0x01000000
#define CC_RL        0x02000000
#define CC_RED       0x04000000
#define CC_PE        0x08000000
#define CC_TPI       0x10000000

/* DMA specific bits */

/* transmit channel control */
#define XC_XE  ((unsigned int)1 << 0) /* transmit enable */
#define XC_SE  ((unsigned int)1 << 1) /* transmit suspend request */
#define XC_LE  ((unsigned int)1 << 2) /* loopback enable */
#define XC_FL  ((unsigned int)1 << 4) /* flush request */
#define XC_MR_MASK  0x000000C0 /* Multiple outstanding reads */
#define XC_MR_SHIFT 6
#define XC_PD       ((unsigned int)1 << 11) /* parity check disable */
#define XC_AE       ((unsigned int)3 << 16) /* address extension bits */
#define XC_AE_SHIFT 16
#define XC_BL_MASK  0x001C0000 /* BurstLen bits */
#define XC_BL_SHIFT 18
#define XC_PC_MASK  0x00E00000 /* Prefetch control */
#define XC_PC_SHIFT 21
#define XC_PT_MASK  0x03000000 /* Prefetch threshold */
#define XC_PT_SHIFT 24

/* transmit descriptor table pointer */
#define XP_LD_MASK 0xfff /* last valid descriptor */

/* transmit channel status */
#define XS_CD_MASK      0x0fff /* current descriptor pointer */
#define XS_XS_MASK      0xf000 /* transmit state */
#define XS_XS_SHIFT     12
#define XS_XS_DISABLED  0x0000 /* disabled */
#define XS_XS_ACTIVE    0x1000 /* active */
#define XS_XS_IDLE      0x2000 /* idle wait */
#define XS_XS_STOPPED   0x3000 /* stopped */
#define XS_XS_SUSP      0x4000 /* suspend pending */
#define XS_XE_MASK      0xf0000 /* transmit errors */
#define XS_XE_SHIFT     16
#define XS_XE_NOERR     0x00000 /* no error */
#define XS_XE_DPE       0x10000 /* descriptor protocol error */
#define XS_XE_DFU       0x20000 /* data fifo underrun */
#define XS_XE_BEBR      0x30000 /* bus error on buffer read */
#define XS_XE_BEDA      0x40000 /* bus error on descriptor access */
#define XS_AD_MASK      0xfff00000 /* active descriptor */
#define XS_AD_SHIFT     20

/* transmit channel control */
#define D64_XC_XE       0x00000001 /* transmit enable */
#define D64_XC_SE       0x00000002 /* transmit suspend request */
#define D64_XC_LE       0x00000004 /* loopback enable */
#define D64_XC_FL       0x00000010 /* flush request */
#define D64_XC_MR_MASK  0x000000C0 /* Multiple outstanding reads */
#define D64_XC_MR_SHIFT 6
#define D64_XC_PD       0x00000800 /* parity check disable */
#define D64_XC_AE       0x00030000 /* address extension bits */
#define D64_XC_AE_SHIFT 16
#define D64_XC_BL_MASK  0x001C0000 /* BurstLen bits */
#define D64_XC_BL_SHIFT 18
#define D64_XC_PC_MASK  0x00E00000 /* Prefetch control */
#define D64_XC_PC_SHIFT 21
#define D64_XC_PT_MASK  0x03000000 /* Prefetch threshold */
#define D64_XC_PT_SHIFT 24

/* transmit descriptor table pointer */
#define D64_XP_LD_MASK  0x00001fff /* last valid descriptor */

/* transmit channel status */
#define D64_XS0_CD_MASK     0x00001fff /* current descriptor pointer */
#define D64_XS0_XS_MASK     0xf0000000 /* transmit state */
#define D64_XS0_XS_SHIFT    28
#define D64_XS0_XS_DISABLED 0x00000000 /* disabled */
#define D64_XS0_XS_ACTIVE   0x10000000 /* active */
#define D64_XS0_XS_IDLE     0x20000000 /* idle wait */
#define D64_XS0_XS_STOPPED  0x30000000 /* stopped */
#define D64_XS0_XS_SUSP     0x40000000 /* suspend pending */

#define D64_XS1_AD_MASK     0x00001fff /* active descriptor */
#define D64_XS1_XE_MASK	    0xf0000000 /* transmit errors */
#define D64_XS1_XE_SHIFT    28
#define D64_XS1_XE_NOERR    0x00000000 /* no error */
#define D64_XS1_XE_DPE      0x10000000 /* descriptor protocol error */
#define D64_XS1_XE_DFU      0x20000000 /* data fifo underrun */
#define D64_XS1_XE_DTE      0x30000000 /* data transfer error */
#define D64_XS1_XE_DESRE    0x40000000 /* descriptor read error */
#define D64_XS1_XE_COREE    0x50000000 /* core error */

/* receive channel control */
#define D64_RC_RE       0x00000001 /* receive enable */
#define D64_RC_RO_MASK  0x000000fe /* receive frame offset */
#define D64_RC_RO_SHIFT 1
#define D64_RC_FM 0x00000100 /* direct fifo receive (pio) mode */
#define D64_RC_SH 0x00000200 /* separate rx header descriptor enable */
#define D64_RC_OC 0x00000400 /* overflow continue */
#define D64_RC_PD 0x00000800 /* parity check disable */
#define D64_RC_GE 0x00004000 /* Glom enable */
#define D64_RC_AE 0x00030000 /* address extension bits */
#define D64_RC_AE_SHIFT 16
#define D64_RC_BL_MASK  0x001C0000 /* BurstLen bits */
#define D64_RC_BL_SHIFT 18
#define D64_RC_PC_MASK  0x00E00000 /* Prefetch control */
#define D64_RC_PC_SHIFT 21
#define D64_RC_PT_MASK  0x03000000 /* Prefetch threshold */
#define D64_RC_PT_SHIFT 24

/* flags for dma controller */
#define DMA_CTRL_PEN    (1 << 0) /* partity enable */
#define DMA_CTRL_ROC    (1 << 1) /* rx overflow continue */
#define DMA_CTRL_RXMULTI (1 << 2) /* allow rx scatter to multiple descrip */
#define DMA_CTRL_UNFRAMED (1 << 3) /* Unframed Rx/Tx data */
#define DMA_CTRL_USB_BOUNDRY4KB_WAR (1 << 4)
#define DMA_CTRL_DMA_AVOIDANCE_WAR (1 << 5) /* DMA avoidance WAR for 4331 */

/* receive descriptor table pointer */
#define D64_RP_LD_MASK    0x00001fff /* last valid descriptor */

/* receive channel status */
#define D64_RS0_CD_MASK  0x00001fff /* current descriptor pointer */
#define D64_RS0_RS_MASK  0xf0000000 /* receive state */
#define D64_RS0_RS_SHIFT    28
#define D64_RS0_RS_DISABLED 0x00000000 /* disabled */
#define D64_RS0_RS_ACTIVE   0x10000000 /* active */
#define D64_RS0_RS_IDLE     0x20000000 /* idle wait */
#define D64_RS0_RS_STOPPED  0x30000000 /* stopped */
#define D64_RS0_RS_SUSP     0x40000000 /* suspend pending */

#define D64_RS1_AD_MASK   0x0001ffff /* active descriptor */
#define D64_RS1_RE_MASK   0xf0000000 /* receive errors */
#define D64_RS1_RE_SHIFT  28
#define D64_RS1_RE_NOERR  0x00000000 /* no error */
#define D64_RS1_RE_DPO    0x10000000 /* descriptor protocol error */
#define D64_RS1_RE_DFU    0x20000000 /* data fifo overflow */
#define D64_RS1_RE_DTE    0x30000000 /* data transfer error */
#define D64_RS1_RE_DESRE  0x40000000 /* descriptor read error */
#define D64_RS1_RE_COREE  0x50000000 /* core error */

/* descriptor control flags 1 */
#define D64_CTRL_COREFLAGS 0x0ff00000 /* core specific flags */
#define D64_CTRL1_EOT ((unsigned int)1 << 28) /* end of descriptor table */
#define D64_CTRL1_IOC ((unsigned int)1 << 29) /* interrupt on completion */
#define D64_CTRL1_EOF ((unsigned int)1 << 30) /* end of frame */
#define D64_CTRL1_SOF ((unsigned int)1 << 31) /* start of frame */

/* descriptor control flags 2 */
#define D64_CTRL2_BC_MASK  0x00007fff /* buff byte cnt.real data len <= 16KB */
#define D64_CTRL2_AE       0x00030000 /* address extension bits */
#define D64_CTRL2_AE_SHIFT 16
#define D64_CTRL2_PARITY   0x00040000      /* parity bit */

/* control flags in the range [27:20] are core-specific and not defined here */
#define D64_CTRL_CORE_MASK  0x0ff00000

#define D64_RX_FRM_STS_LEN  0x0000ffff /* frame length mask */
#define D64_RX_FRM_STS_OVFL 0x00800000 /* RxOverFlow */

/* no. of descp used - 1, d11corerev >= 22 */
#define D64_RX_FRM_STS_DSCRCNT 0x0f000000

#define D64_RX_FRM_STS_DATATYPE 0xf0000000 /* core-dependent data type */

#define HWRXOFF    30

#endif /*__BCM_AMAC_REGS_H__ */
