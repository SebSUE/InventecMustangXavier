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

#include <linux/types.h>
#include <linux/mdio.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/net.h>
#include <linux/dma-mapping.h>

#include "bcm-amac-regs.h"
#include "bcm-amac-enet.h"
#include "bcm-amac-core.h"
#include "bcm-robo.h"


#define GMAC_RESET_DELAY 2

#define SPINWAIT(exp, us) { \
	uint countdown = (us) + 9; \
	while ((exp) && (countdown >= 10)) {\
		udelay(10); \
		countdown -= 10; \
	} \
}

static unsigned int tx_done_flag;

static void amac_core_init_reset(struct bcm_amac_priv *privp);
static void amac_core_clear_reset(struct bcm_amac_priv *privp);
static void amac_dma_ctrlflags(struct bcm_amac_priv *privp,
	u32 mask, u32 flags);
static int amac_dma_rx_init(struct bcm_amac_priv *privp);
static int amac_dma_tx_init(struct bcm_amac_priv *privp);
static void amac_set_tx_flag(int val);
static void amac_clear_tx_intr(struct bcm_amac_priv *privp);
static void amac_clear_rx_intr(struct bcm_amac_priv *privp);
static unsigned int amac_dma_check_rx_done(struct bcm_amac_priv *privp);
static void amac_enable_tx_intr(struct bcm_amac_priv *privp, bool enable);
static void *amac_alloc_rx_skb(struct bcm_amac_priv *privp,
	int len, struct skb_list_node *node);


static void amac_dma_ctrlflags(struct bcm_amac_priv *privp,
	u32 mask, u32 flags)
{
	privp->dmactrlflags &= ~mask;
	privp->dmactrlflags |= flags;

	/*If trying to enable parity, check if parity is actually supported */
	if (privp->dmactrlflags & DMA_CTRL_PEN) {
		u32 control;

		control = readl(privp->hw.reg.amac_core +
			GMAC_DMA_TX_CTRL_OFFSET);
		writel(control | D64_XC_PD,
			(privp->hw.reg.amac_core + GMAC_DMA_TX_CTRL_OFFSET));
		if (readl((privp->hw.reg.amac_core +
			GMAC_DMA_TX_CTRL_OFFSET)) & D64_XC_PD) {
			/* We *can* disable it so it is supported,
			 * restore control register
			 */
			writel(control,
			(privp->hw.reg.amac_core + GMAC_DMA_TX_CTRL_OFFSET));
		} else {
			/* Not supported, don't allow it to be enabled */
			privp->dmactrlflags &= ~DMA_CTRL_PEN;
		}
	}
}

/**
 * bcm_amac_dma_start() - Initialize and start the DMA.
 * @privp: driver info pointer
 *
 * Both RX and TX are initialized.
 * Only RX is enabled, TX is enabled as required.
 *
 * Returns: '0' or error
 */
int bcm_amac_dma_start(struct bcm_amac_priv *privp)
{
	int rc;

	if (!privp)
		return -EINVAL;

	amac_dma_ctrlflags(privp, (DMA_CTRL_ROC | DMA_CTRL_PEN), 0);

	rc = amac_dma_rx_init(privp); /* Initialize RX DMA */
	if (rc != 0) {
		netdev_err(privp->ndev, "Failed!! DMA RX Init\n");
		goto dma_start_rx_err;
	}

	rc = amac_dma_tx_init(privp); /* Initialize TX DMA */
	if (rc != 0) {
		netdev_err(privp->ndev, "Failed!! DMA TX Init\n");
		goto dma_start_tx_err;
	}

	amac_enable_tx_intr(privp, true);
	/* Enable RX DMA */
	bcm_amac_enable_rx_dma(privp, true);

	/* enable the overflow continue feature and disable parity */
	amac_dma_ctrlflags(privp, DMA_CTRL_ROC | DMA_CTRL_PEN, DMA_CTRL_ROC);

	amac_set_tx_flag(0);

	return 0;

dma_start_tx_err:
dma_start_rx_err:
	/* Stop DMA */
	bcm_amac_dma_stop(privp);

	return rc;
}

void bcm_amac_dma_stop(struct bcm_amac_priv *privp)
{
	u32 i;

	/* Stop the RX DMA */
	bcm_amac_enable_rx_dma(privp, false);

	/* Free Rx buffers */
	for (i = 0; i < DMA_RX_DESC_NUM; i++) {
		if (privp->dma.rx_skb_list[i].skb)
			dev_kfree_skb_any(privp->dma.rx_skb_list[i].skb);
	}

	kfree(privp->dma.rx_skb_list);
	privp->dma.rx_skb_list = NULL;

	if (privp->dma.rx.descp) {
		dma_free_coherent(NULL, privp->dma.rx.alloc_size,
			privp->dma.rx.descp, privp->dma.rx.addr);
		privp->dma.rx.descp = NULL;
	}

	/* Stop the TX DMA */
	bcm_amac_enable_tx_dma(privp, false);

	/* TODO: Unregister the Interrupt handler */
	/*	free_irq(privp->hw.intr_num, (void *)privp); */

	/* Free Tx buffers */
	bcm_amac_tx_clean(privp);

	if (privp->dma.tx.descp) {
		dma_free_coherent(NULL,
			privp->dma.tx.alloc_size,
			privp->dma.tx.descp,
			privp->dma.tx.addr);

		privp->dma.tx.descp = NULL;
	}

	kfree(privp->dma.tx_skb_list);
	privp->dma.tx_skb_list = NULL;
}

/**
 * amac_alloc_rx_skb() - Allocate RX SKB
 * @privp: driver info pointer
 * @len: length of skb
 * @node: skb node pointer
 *
 * Returns: pointer to skb data.
 */
static void *amac_alloc_rx_skb(struct bcm_amac_priv *privp,
	int len, struct skb_list_node *node)
{
	struct sk_buff *skb;
	int offset;
	struct net_device *ndev = privp->ndev;

	skb = netdev_alloc_skb(ndev, (len + (2*RXALIGN)));
	if (unlikely(skb == NULL))
		return NULL;

	/* Align buffer for DMA requirements */
	/* Desc has to be 16-byte aligned ? */
	offset = (((unsigned long)skb->data + RXALIGN-1) & ~(RXALIGN-1))
			- (unsigned long)skb->data;
	skb_reserve(skb, offset);

	node->skb = skb;
	node->len = len;

	return skb->data;
}

/**
 * amac_enable_tx_intr() - Enable TX interrupt
 * @privp: driver info pointer
 * @enable: enable/disable the interrupt
 *
 * Returns: none
 */
static void amac_enable_tx_intr(struct bcm_amac_priv *privp, bool enable)
{
	u32 intr_mask;

	if (enable) {
		/* Clear TX interrupt */
		amac_clear_tx_intr(privp);

		/* Enable TX interrupt */
		intr_mask = readl(privp->hw.reg.amac_core +
						GMAC_INT_MASK_ADDR);
		intr_mask |= I_XI_ALL;
		writel(intr_mask,
			(privp->hw.reg.amac_core + GMAC_INT_MASK_ADDR));

	} else {
		/* Disable TX interrupt */
		intr_mask = readl(privp->hw.reg.amac_core + GMAC_INT_MASK_ADDR);
		intr_mask &= ~I_XI_ALL;
		writel(intr_mask,
			(privp->hw.reg.amac_core + GMAC_INT_MASK_ADDR));

		/* Clear TX interrupt */
		amac_clear_tx_intr(privp);
	}
}

/**
 * bcm_amac_enable_rx_intr() - Enable RX interrupt
 * @privp: driver info pointer
 * @enable: enable/disable the interrupt
 *
 * Returns: none
 */
void bcm_amac_enable_rx_intr(struct bcm_amac_priv *privp, bool enable)
{
	u32 intr_mask;

	if (enable) {
		/* Clear RX interrupt */
		amac_clear_rx_intr(privp);

		/* Enable RX interrupt */
		intr_mask = readl(privp->hw.reg.amac_core + GMAC_INT_MASK_ADDR);
		intr_mask |= I_RI;
		writel(intr_mask,
			(privp->hw.reg.amac_core + GMAC_INT_MASK_ADDR));

	} else {
		/* Disable RX interrupt */
		intr_mask = readl(privp->hw.reg.amac_core + GMAC_INT_MASK_ADDR);
		intr_mask &= ~I_RI;
		writel(intr_mask,
			(privp->hw.reg.amac_core + GMAC_INT_MASK_ADDR));

		/* Clear RX interrupt */
		amac_clear_rx_intr(privp);
	}
}

/**
 * bcm_amac_isr() - GMAC ISR Routine.
 * @irq: intr number
 * @userdata: driver info data pointer
 *
 * Handles both RX and TX interrupts.
 * In case of RX interrupt the interrupt is disabled, cleared and the
 * NAPI routine is invoked.
 * For TX, interrupt and DMA are both disabled and the tasklet is scheduled
 * in case of further data in the queue.
 *
 * Returns: interrupt handler status
 */
irqreturn_t bcm_amac_isr(int irq, void *userdata)
{
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)userdata;
	u32 intr_status;

	intr_status =  readl(privp->hw.reg.amac_core + GMAC_INT_STATUS_ADDR);

	if (intr_status & I_RI) {
		/* RX DMA complete interrupt */
		if (likely(netif_running(privp->ndev))) {
			/* Disable RX DMA Interrupt*/
			bcm_amac_enable_rx_intr(privp, false);

			napi_schedule(&privp->napi);
		}
	} else if (intr_status & I_XI_ALL) {
		/* TX DMA complete interrupt */
		if (likely(netif_running(privp->ndev))) {
			/* Disable TX DMA */
			bcm_amac_enable_tx_dma(privp, false);

			amac_clear_tx_intr(privp);

			amac_set_tx_flag(0); /* Indicate TX is free */

			/* trigger tx processing in case packets are waiting */
			tasklet_schedule(&privp->tx_tasklet);

			if (unlikely(netif_queue_stopped(privp->ndev)))
				netif_wake_queue(privp->ndev);
		}
	} else
		return IRQ_NONE;

	return IRQ_HANDLED;
}

/**
 * amac_dma_rx_init() - RX DMA initialization routine
 * @privp: driver info pointer
 *
 * Descriptors and skb are allocated and initlialized, the various rx registers
 * are updated with the descriptor information
 *
 * Returns: '0' or error
 */
static int amac_dma_rx_init(struct bcm_amac_priv *privp)
{
	struct net_device *ndev = privp->ndev;
	struct dma_priv *dma_p = &privp->dma;
	struct dma64_desc *descp;
	dma_addr_t dma_buf;
	unsigned int size;
	char *bufp;
	u32 ctrl, i;
	int rc = 0;

	dma_p->rx.ring_len = DMA_RX_DESC_NUM;

	/* Allocate rx descriptors */
	dma_p->rx.index = 0;
	dma_p->rx.alloc_size = DMA_RX_DESC_NUM * sizeof(struct dma64_desc);
	dma_p->rx.descp = dma_alloc_coherent(NULL, dma_p->rx.alloc_size,
			&dma_p->rx.addr, GFP_KERNEL);
	if (dma_p->rx.descp == NULL) {
		netdev_err(ndev, "Failed to alloc rx dma desc\n");
		return -ENOMEM;
	}

	/* Allocate rx skb list */
	size = dma_p->rx.ring_len * sizeof(*dma_p->rx_skb_list);
	dma_p->rx_skb_list = kzalloc(size, GFP_KERNEL);
	if (dma_p->rx_skb_list == NULL) {
		netdev_err(ndev, "Failed to alloc rx skb list size=%u\n", size);
		rc = -ENOMEM;
		goto rx_init_err;
	}

	/* Setup rx descriptor ring */
	for (i = 0; i < DMA_RX_DESC_NUM; i++) {
		descp = (struct dma64_desc *)(dma_p->rx.descp) + i;

		bufp = amac_alloc_rx_skb(privp, DMA_RX_BUF_LEN,
			&dma_p->rx_skb_list[i]);
		if (bufp == NULL) {
			netdev_err(ndev,
				"Failed to alloc rx dma_buf i=%i\n", i);
			rc = -ENOMEM;
			goto rx_init_err;
		}

		ctrl = 0;

		/* if last descr set endOfTable */
		if (i == (DMA_RX_DESC_NUM - 1))
			ctrl = D64_CTRL1_EOT;

		/* Transfer buffer ownership to device */
		dma_buf = dma_map_single(&privp->pdev->dev, bufp,
					 DMA_RX_BUF_LEN, DMA_FROM_DEVICE);
		if (dma_mapping_error(&privp->pdev->dev, dma_buf)) {
			netdev_err(ndev, "DMA mapping Failed !!\n");

			rc = -EFAULT;
			goto rx_init_err;
		}

		descp->ctrl1 = ctrl;
		descp->ctrl2 = DMA_RX_BUF_LEN;
		descp->addrlow = (u32)dma_buf;
		descp->addrhigh = 0;
	}

	descp = (struct dma64_desc *)dma_p->rx.descp;

	/* initailize the DMA channel */
	writel((u32)dma_p->rx.addr,
		(privp->hw.reg.amac_core + GMAC_DMA_RX_ADDR_LO_OFFSET));

	writel(0, (privp->hw.reg.amac_core + GMAC_DMA_RX_ADDR_HI_OFFSET));

	/* now update the dma last descriptor */
	writel(dma_p->rx.addr,
		(privp->hw.reg.amac_core + GMAC_DMA_RX_PTR_OFFSET));

rx_init_err:
	return rc;
}

/**
 * amac_dma_tx_init() - TX DMA initialization routine
 * @privp: driver info pointer
 *
 * Descriptors and skb list are allocated.
 *
 * Returns: 0 or error
 */
static int amac_dma_tx_init(struct bcm_amac_priv *privp)
{
	struct net_device *ndev = privp->ndev;
	struct dma_priv	*dma_p = &privp->dma;
	unsigned int size;
	int	rc = 0;

	dma_p->tx.ring_len = DMA_TX_DESC_NUM;
	dma_p->tx_max_pkts = DMA_TX_MAX_QUEUE_LEN;

	/* Allocate tx descriptors */
	dma_p->tx.index = 0;
	dma_p->tx.alloc_size = dma_p->tx.ring_len * sizeof(struct dma64_desc);
	dma_p->tx.descp = dma_alloc_coherent(NULL, dma_p->tx.alloc_size,
		&dma_p->tx.addr, GFP_KERNEL);
	if (dma_p->tx.descp == NULL) {
		netdev_err(ndev, "Cannot allocate tx dma descriptors.\n");
		rc = -ENOMEM;
		goto tx_init_err;
	}

	/* Allocate tx skb list */
	size = dma_p->tx_max_pkts * sizeof(*dma_p->tx_skb_list);
	dma_p->tx_skb_list = kzalloc(size, GFP_KERNEL);
	if (dma_p->tx_skb_list == NULL) {
		netdev_err(ndev, "Failed to alloc tx skb list size=%u\n", size);
		rc = -ENOMEM;
	}

tx_init_err:
	return rc;

}

static void amac_core_init_reset(struct bcm_amac_priv *privp)
{
	u32 tmp;

	tmp = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_OFFSET);
	tmp |= CC_SR;
	writel(tmp, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_OFFSET));

	udelay(GMAC_RESET_DELAY);
}

static void amac_core_clear_reset(struct bcm_amac_priv *privp)
{
	u32 tmp;

	tmp = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_OFFSET);
	tmp &= ~(CC_SR);
	writel(tmp, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_OFFSET));

	udelay(GMAC_RESET_DELAY);
}

/**
 * bcm_amac_core_init() - Initialize the gmac core
 * @privp: driver info pointer
 *
 * Initialize the gmac core, phy, setup clock, initialize swith mode or
 * switch bypass mode.
 *
 * Returns: '0' for success or '-1'
 */
int bcm_amac_core_init(struct bcm_amac_priv *privp)
{
	u32 tmp;
	u32 cmd;

	/* Reset AMAC core */
	writel(0, privp->hw.reg.amac_idm_reset);

	/* Set clock */
	tmp = readl(privp->hw.reg.amac_io_ctrl);

	tmp &= ~(1 << AMAC_IDM0_IO_CONTROL_DIRECT__CLK_250_SEL);
	tmp |= (1 << AMAC_IDM0_IO_CONTROL_DIRECT__DIRECT_GMII_MODE);
	tmp &= ~(1 << AMAC_IDM0_IO_CONTROL_DIRECT__DEST_SYNC_MODE_EN);
	writel(tmp, privp->hw.reg.amac_io_ctrl);

	/* reset GMAC core */
	cmd = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_OFFSET);
	cmd &= ~(CC_TE | CC_RE | CC_RPI | CC_TAI | CC_HD | CC_ML |
		CC_CFE | CC_RL | CC_RED | CC_PE | CC_TPI |
		CC_PAD_EN | CC_PF);
	cmd |= (CC_NLC | CC_CFE); /* keep promiscuous mode disabled */

	amac_core_init_reset(privp); /* Put GMAC in reset */
	writel(cmd, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_OFFSET));
	amac_core_clear_reset(privp);

	/* Enable clear MIB on read */
	tmp = readl(privp->hw.reg.amac_core);
	tmp |= DC_MROR;
	writel(tmp, privp->hw.reg.amac_core);

	/* PHY set smi_master to driver mdc_clk */
	tmp = readl(privp->hw.reg.amac_core + GMAC_PHY_CTRL_ADDR);
	tmp |= PC_MTE;
	writel(tmp, (privp->hw.reg.amac_core + GMAC_PHY_CTRL_ADDR));

	/* Clear persistant sw intstatus */
	writel(0, (privp->hw.reg.amac_core + GMAC_INT_STATUS_ADDR));

	/* Print CHIP ID */
	tmp = (u32)(readl(privp->hw.reg.icfg_regs) & 0xFFFF);
	dev_info(&privp->pdev->dev, "%s: Chip ID=0x%x ", __func__, tmp);

	/* Configure Switch */
	tmp = readl(privp->hw.reg.switch_global_cfg);

	if (privp->switchmode == 0) {
		/* Switch Bypass mode */
		dev_info(&privp->pdev->dev,
			"%s: Switch bypass mode\n", __func__);
		tmp |= (1 << SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_BYPASS_SWITCH);
	} else {
		/* Switch mode */
		dev_info(&privp->pdev->dev,
			"%s: Switch mode\n", __func__);
		tmp &= ~(1 << SWITCH_GLOBAL_CONFIG__CDRU_SWITCH_BYPASS_SWITCH);
		if (bcm_esw_init(privp))
			return -1;

	}
	writel(tmp, (privp->hw.reg.switch_global_cfg));

	/* Setup IO PAD CTRL */
	tmp = readl(privp->hw.reg.crmu_io_pad_ctrl);
	tmp &= ~(1 << CRMU_CHIP_IO_PAD_CONTROL__CDRU_IOMUX_FORCE_PAD_IN);
	writel(tmp, (privp->hw.reg.crmu_io_pad_ctrl));

	/* Configure GMAC0 */
	/* enable one rx interrupt per received frame */
	writel((1 << GMAC0_IRL_FRAMECOUNT_SHIFT),
		(privp->hw.reg.amac_core + GMAC_INTR_RECV_LAZY_ADDR));

	cmd = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_OFFSET);
	/* enable 802.3x tx flow control (honor received PAUSE frames) */
	cmd &= ~CC_RPI;

	/* keep promiscuous mode disable */

	/* Disbale loopback mode */
	cmd &= ~CC_ML;
	/* set the speed */
	cmd &= ~(CC_ES_MASK | CC_HD);
	/* Set to 1Gbps and full duplex by default */
	cmd |= (2 << CC_ES_SHIFT);

	amac_core_init_reset(privp); /* Put GMAC in reset */
	writel(cmd, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_OFFSET));
	amac_core_clear_reset(privp);

	return 0;
}

/**
 * bcm_amac_enable_tx_dma() - Enable the DMA
 * @privp: driver info pointer
 * @dir: TX or RX direction to enable
 *
 * Returns: 0 or error
 */
int bcm_amac_enable_tx_dma(struct bcm_amac_priv *privp, bool enable)
{
	u32 control;
	int status = 0;

	if (enable) {
		/* Enable TX DMA Interrupts */
		amac_enable_tx_intr(privp, true);

		/* These bits 20:18 (burstLen) of control register can be
		 * written but will take effect only if these bits are
		 * valid. So this will not affect previous versions
		 * of the DMA. They will continue to have those bits set to 0.
		 */
		control = readl(privp->hw.reg.amac_core +
			GMAC_DMA_TX_CTRL_OFFSET);

		control |= D64_XC_XE;
		if ((privp->dmactrlflags & DMA_CTRL_PEN) == 0)
			control |= D64_XC_PD;

		control |= 0x3 << D64_XC_BL_SHIFT; /* Burst length of 3 */

		writel(control,
			(privp->hw.reg.amac_core + GMAC_DMA_TX_CTRL_OFFSET));

		SPINWAIT(((status =
			(readl(privp->hw.reg.amac_core +
				GMAC_DMA_TX_STATUS0_OFFSET)
			& D64_XS0_XS_MASK)) != D64_XS0_XS_IDLE),
			10000);
	} else {
		/* Disable TX DMA */

		/* suspend tx DMA first, if active */
		status = readl(privp->hw.reg.amac_core +
						GMAC_DMA_TX_STATUS0_OFFSET)
				& D64_XS0_XS_MASK;

		if (status == D64_XS0_XS_ACTIVE) {
			status = readl(privp->hw.reg.amac_core +
						GMAC_DMA_TX_CTRL_OFFSET);
			status |= D64_XC_SE;

			writel(status, (privp->hw.reg.amac_core +
				GMAC_DMA_TX_CTRL_OFFSET));

			/* DMA engines are not disabled */
			/* until transfer finishes */
			SPINWAIT(
				((readl(privp->hw.reg.amac_core +
					GMAC_DMA_TX_STATUS0_OFFSET)
				& D64_XS0_XS_MASK) == D64_XS0_XS_ACTIVE),
				10000);
		}

		writel(0, (privp->hw.reg.amac_core + GMAC_DMA_TX_CTRL_OFFSET));
		SPINWAIT(((status =
			(readl(privp->hw.reg.amac_core +
			GMAC_DMA_TX_STATUS0_OFFSET)
			& D64_XS0_XS_MASK)) != D64_XS0_XS_DISABLED),
			10000);

		status &= D64_XS0_XS_MASK;

		/* Disable TX DMA Interrupt */
		amac_enable_tx_intr(privp, false);
	}

	return status;
}

/**
 * bcm_amac_enable_rx_dma() - Enable/Disable the RX DMA
 * @privp: driver data structure pointer
 * @enable: enable/disable rx dma
 *
 * Returns: 0 or error
 */
int bcm_amac_enable_rx_dma(struct bcm_amac_priv *privp, bool enable)
{
	u32 control;
	int status = 0;

	if (enable) {
		bcm_amac_enable_rx_intr(privp, true);

		control = (readl(privp->hw.reg.amac_core +
				GMAC_DMA_RX_CTRL_OFFSET) &
				D64_RC_AE) | D64_RC_RE;

		if ((privp->dmactrlflags & DMA_CTRL_PEN) == 0)
			control |= D64_RC_PD;

		if (privp->dmactrlflags & DMA_CTRL_ROC)
			control |= D64_RC_OC;

		/* These bits 20:18 (burstLen) of control register can be
		 * written but will take effect only if these bits are
		 * valid. So this will not affect previous versions
		 * of the DMA. They will continue to have those bits
		 */

		/* set to 0. */
		control &= ~D64_RC_BL_MASK;
		/* Keep default Rx burstlen */
		control |= readl(privp->hw.reg.amac_core +
					GMAC_DMA_RX_CTRL_OFFSET)
					& D64_RC_BL_MASK;
		control |= HWRXOFF << D64_RC_RO_SHIFT;

		writel(control,
			privp->hw.reg.amac_core + GMAC_DMA_RX_CTRL_OFFSET);

		/* the rx descriptor ring should have
		 * the addresses set properly
		 * set the lastdscr for the rx ring
		 */
		writel(((u32)(privp->dma.rx.descp) +
			(DMA_RX_DESC_NUM - 1) * RX_BUF_SIZE) & D64_XP_LD_MASK,
			(privp->hw.reg.amac_core + GMAC_DMA_RX_PTR_OFFSET));

	} else {
		/* Disable RX DMA */

		/* PR2414 WAR: DMA engines are not disabled until
		 * transfer completes
		 */
		writel(0, (privp->hw.reg.amac_core + GMAC_DMA_RX_CTRL_OFFSET));
		SPINWAIT(((status =
			(readl(privp->hw.reg.amac_core +
				GMAC_DMA_RX_STATUS0_OFFSET)
			& D64_RS0_RS_MASK)) != D64_RS0_RS_DISABLED),
			 10000);

		status &= D64_RS0_RS_MASK;

		/* Disable the DMA interrupt */
		bcm_amac_enable_rx_intr(privp, false);
	}

	return status;
}

/**
 * amac_clear_tx_intr() - Clear Interrupt status
 * @privp: driver info pointer
 * @dir: TX or RX direction to clear
 *
 * Returns: none
 */
static void amac_clear_tx_intr(struct bcm_amac_priv *privp)
{
	u32 intr_status;

	intr_status = readl(privp->hw.reg.amac_core +
		GMAC_INT_STATUS_ADDR);

	intr_status &= I_XI_ALL; /* Clear only TX interrupt(s) */

	/* Clear TX interrupt */
	writel(intr_status,
		(privp->hw.reg.amac_core + GMAC_INT_STATUS_ADDR));
}

/**
 * amac_clear_rx_intr() - Clear Interrupt status
 * @privp: driver info pointer
 * @dir: TX or RX direction to clear
 *
 * Returns: none
 */
static void amac_clear_rx_intr(struct bcm_amac_priv *privp)
{
	u32 intr_status;

	intr_status = readl(privp->hw.reg.amac_core +
		GMAC_INT_STATUS_ADDR);

	intr_status &= I_RI; /* Clear only RX interrupt */

	/* Clear RX interrupt */
	writel(intr_status,
		(privp->hw.reg.amac_core + GMAC_INT_STATUS_ADDR));
}

/**
 * bcm_amac_core_enable() - Enable GMAC core
 * @privp: driver info pointer
 * @enable: Enable or disable
 *
 * Returns: none
 */
void bcm_amac_core_enable(struct bcm_amac_priv *privp, int enable)
{
	u32 cmdcfg;

	cmdcfg = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_OFFSET);

	amac_core_init_reset(privp);

	cmdcfg |= CC_SR;

	/* first deassert rx_ena and tx_ena while in reset */
	cmdcfg &= ~(CC_RE | CC_TE);
	/* write command config reg */
	writel(cmdcfg, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_OFFSET));

	amac_core_clear_reset(privp);

	/* if not enable exit now */
	if (enable == 0)
		return;

	/* enable the mac transmit and receive paths now */
	udelay(2);
	cmdcfg &= ~CC_SR;
	cmdcfg |= (CC_RE | CC_TE);

	/* assert rx_ena and tx_ena when out of reset to enable the mac */
	writel(cmdcfg, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_OFFSET));

	/* Clear Interrupts */
	writel(I_INTMASK, (privp->hw.reg.amac_core + GMAC_INT_STATUS_ADDR));
}

/**
 * bcm_amac_tx_clean() - Prepare for transmission
 * @privp: driver info pointer
 *
 * Returns: none
 */
void bcm_amac_tx_clean(struct bcm_amac_priv *privp)
{
	struct dma_priv *dmap = &privp->dma;
	const struct net_device *ndev = privp->ndev;
	int pktcurr;

	for (pktcurr = 0; pktcurr < dmap->tx_curr; pktcurr++) {
		struct sk_buff *skbp;
		int len;

		/* Payload data */
		skbp = dmap->tx_skb_list[pktcurr].skb;
		dmap->tx_skb_list[pktcurr].skb = NULL;
		len = dmap->tx_skb_list[pktcurr].len;
		if (skbp) {
			dma_map_single(&privp->pdev->dev,
				       skbp->data,
				      len,
				      DMA_TO_DEVICE);

			dev_kfree_skb_any(skbp);
		} else
			netdev_err(ndev, "invalid skb?\n");
	}

	dmap->tx.index = 0;
	dmap->tx_curr = 0;
}

/**
 * bcm_amac_tx_send_packet() - Ethernet TX routine.
 * @privp: driver info pointer
 *
 * Called within the TX tasklet with packets in fifo.
 * Gets data from the queue, formats the descriptor , setup and starts the
 * TX DMA.
 *
 * Returns: none
 */
void bcm_amac_tx_send_packet(struct bcm_amac_priv *privp)
{
	struct dma_priv	*dmap = &privp->dma;
	int				curr = 0;
	int				desc_idx = 0;
	struct dma64_desc	*descp = NULL;
	u32		last_desc;
	struct sk_buff	*skb;
	unsigned int	len;
	char			*bufp;
	dma_addr_t		buf_dma;
	u32		ctrl;
	u32		dma_flags;

	/* Build descriptor chain */
	while ((len = kfifo_out_spinlocked(&dmap->txfifo, (unsigned char *)&skb,
				sizeof(&skb), &privp->lock)) == sizeof(&skb)) {

		/* Indicate we are busy sending a packet */
		if (!bcm_amac_get_tx_flag())
			amac_set_tx_flag(1);

		bufp = skb->data;
		len = skb->len;

		if (unlikely(len < MIN_FRAME_LEN)) {
			/* Clear the padded memory to avoid 'etherleak'
			 * vulnerability
			 */
			memset(skb->data + len, 0, (MIN_FRAME_LEN - len));
			len = MIN_FRAME_LEN;
		}

		/* Timestamp the packet */
		skb_tx_timestamp(skb);

		buf_dma = dma_map_single(&privp->pdev->dev, bufp,
					 len, DMA_TO_DEVICE);
		if (dma_mapping_error(&privp->pdev->dev, buf_dma)) {
			netdev_err(privp->ndev, "DMA mapping Failed !!\n");

			privp->ndev->stats.tx_dropped++;
			privp->eth_stats.tx_dropped_pkts++;
			continue;
		}

		descp = (&((struct dma64_desc *)(dmap->tx.descp))[(desc_idx)]);
		dma_flags = D64_CTRL1_SOF | D64_CTRL1_EOF;
		ctrl = (len & D64_CTRL2_BC_MASK);

		descp->addrhigh = 0;
		descp->addrlow = (u32)buf_dma;
		descp->ctrl1 = (u32)dma_flags;
		descp->ctrl2 = (u32)ctrl;

		/* Add skb to list */
		dmap->tx_skb_list[curr].skb = skb;
		dmap->tx_skb_list[curr].len = len;

		desc_idx++;
		curr++;
	}

	if (descp) {
		dmap->tx_curr = curr;
		dmap->tx.index = desc_idx;

		descp->ctrl1 |=  D64_CTRL1_IOC;/* Interrupt after the last one*/

		descp = (&((struct dma64_desc *)(dmap->tx.descp))[(desc_idx)]);
		descp->ctrl1 = D64_CTRL1_EOT; /* Mark last descriptor as EOT */

		last_desc = ((u32)(&((struct dma64_desc *)
			(dmap->tx.addr))[desc_idx]));
		last_desc &= D64_XP_LD_MASK;

		/* Disable TX DMA */
		bcm_amac_enable_tx_dma(privp, false);

		/* initailize the DMA channel */
		writel((&((struct dma64_desc *)(dmap->tx.addr))[0]),
			(privp->hw.reg.amac_core + GMAC_DMA_TX_ADDR_LO_OFFSET));
		writel(0,
			(privp->hw.reg.amac_core + GMAC_DMA_TX_ADDR_HI_OFFSET));

		wmb();

		/* Enable TX DMA and Interrupt */
		bcm_amac_enable_tx_dma(privp, true);

		/* update the dma last descriptor */
		writel(last_desc,
			(privp->hw.reg.amac_core + GMAC_DMA_TX_PTR_OFFSET));
	}
}

/**
 * amac_dma_check_rx_done() - Check to see if we have a packet to be read.
 * @privp: driver info pointer
 *
 * Returns: '0' if no data or else the offset value
 */
static unsigned int amac_dma_check_rx_done(struct bcm_amac_priv *privp)
{
	struct dma_priv	*dmap = &privp->dma;
	u32 stat0 = 0, stat1 = 0;
	u32 offset;
	u32 control;
	int index, curr, active;

	index = dmap->rx.index;
	offset = (u32)dmap->rx.addr;

	stat0 = readl(
		privp->hw.reg.amac_core + GMAC_DMA_RX_STATUS0_OFFSET)
		& D64_RS0_CD_MASK;

	stat1 = readl(
		privp->hw.reg.amac_core + GMAC_DMA_RX_STATUS1_OFFSET)
		& D64_RS0_CD_MASK;

	curr = ((stat0 - offset) & D64_RS0_CD_MASK) / sizeof(struct dma64_desc);
	active = ((stat1 - offset) & D64_RS0_CD_MASK)
		/ sizeof(struct dma64_desc);

	if (index == curr)
		return 0; /* No Data */

	/* Payload contains hw data hence offset to be used */
	control = readl(
		privp->hw.reg.amac_core + GMAC_DMA_RX_CTRL_OFFSET);
	offset = (control & D64_RC_RO_MASK) >> D64_RC_RO_SHIFT;

	return offset;
}

static void amac_set_tx_flag(int val)
{
	tx_done_flag = 0;
}

int bcm_amac_get_tx_flag(void)
{
	return tx_done_flag;
}

/**
 * amac_inc_rx_desc_index() - Increment the index, with wrap around
 * @idx: index pointer to increment
 *
 * Return: none
 */
static inline void amac_inc_rx_desc_index(int *idx)
{
	(*idx)++;
	if (unlikely(*idx >= DMA_RX_DESC_NUM))
		*idx = 0;
}

/**
 * bcm_amac_dma_get_rx_data() - Retrieves RX data if available.
 * @privp: driver info pointer
 * @skb: skb pointer
 *
 * If data is available, the function updates the rx pointer register.
 * It also strips out the hw specific status from the data.
 *
 * Returns: '0' if no frames, 'length' of packet or error
 */
int bcm_amac_dma_get_rx_data(struct bcm_amac_priv *privp,
	struct sk_buff **skbp)
{
	u32 hw_offset;
	struct dma64_desc *descp = NULL;
	struct dma_priv *dmap = &privp->dma;
	char *new_buff;
	dma_addr_t dma_buff;
	int len;
	char *bufp;
	u32 rx_ptr;

	/* Get next frame from RX DMA */
	descp = (&((struct dma64_desc *)(dmap->rx.descp))[dmap->rx.index]);

	hw_offset = amac_dma_check_rx_done(privp);
	if (hw_offset) {

		struct skb_list_node *const node =
			&dmap->rx_skb_list[dmap->rx.index];

		rx_ptr = ((u32)
		(&((struct dma64_desc *)(dmap->rx.addr))[dmap->rx.index]));

		*skbp = node->skb;

		/* Re-arm descriptor with new buffer */
		new_buff = (char *)amac_alloc_rx_skb(privp,
							DMA_RX_BUF_LEN, node);
		if (!new_buff) {
			/* No skb available.
			 * Leave the existing one in place.
			 * Explicitly discard the frame by marking
			 * the buffer as free, increment the
			 * rx index and update the err counters.
			 * That's the best we can do at this point.
			 */
			descp->ctrl2 = DMA_RX_BUF_LEN;
			amac_inc_rx_desc_index(&dmap->rx.index);

			len = -ENOMEM;
			goto rx_dma_data_done;
		}

		/* Set buffer ownership to device and invalidate cache */
		dma_buff = dma_map_single(&privp->pdev->dev, new_buff,
					  DMA_RX_BUF_LEN, DMA_FROM_DEVICE);
		if (dma_mapping_error(&privp->pdev->dev, dma_buff)) {
			netdev_err(privp->ndev, "DMA mapping error\n");
			descp->ctrl2 = DMA_RX_BUF_LEN;
			amac_inc_rx_desc_index(&dmap->rx.index);
			len = -EFAULT;

			goto rx_dma_data_done;
		}

		/* Update DMA descriptor */
		descp->ctrl2 = DMA_RX_BUF_LEN;
		descp->addrlow = (u32)dma_buff;
		descp->addrhigh = 0;

		/* Increment descp index */
		amac_inc_rx_desc_index(&dmap->rx.index);
	} else {
		/* No Frames */
		return 0;
	}

	bufp = (*skbp)->data;

	dma_buff = dma_map_single(&privp->pdev->dev, bufp,
				  DMA_RX_BUF_LEN, DMA_FROM_DEVICE);
	if (dma_mapping_error(&privp->pdev->dev, dma_buff)) {
		netdev_err(privp->ndev, "DMA mapping error\n");
		len = -EFAULT;

		goto rx_dma_data_done;
	}

	len = *((u16 *)bufp); /* First 2 bytes is data */
	privp->eth_stats.rx_bytes += len;

	/* Received an invalid frame length
	 */
	if (len > DMA_RX_BUF_LEN) {
		len = -EBADMSG;
		netdev_err(privp->ndev, "Invalid frame length, len=%d\n", len);
		goto rx_dma_data_done;
	}

	/* Strip the hw status from the rx data */
	bufp += hw_offset; /* Point to real data */
	memcpy((void *)(*skbp)->data, bufp, len); /* Realign the data in SKB */

rx_dma_data_done:
	/* Update RX pointer */
	writel(rx_ptr, (privp->hw.reg.amac_core + GMAC_DMA_RX_PTR_OFFSET));

	return len;
}

/**
 * amac_set_prom() - Enable / Disable promiscous mode
 * @privp: device data pointer
 * @enable: '0' disables promiscous mode, >0 enables promiscous mode
 */
static void amac_set_prom(struct bcm_amac_priv *privp, int enable)
{
	u32 reg;

	reg = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_OFFSET);

	if ((enable) && (!(reg & CC_PROM)))
		reg |= CC_PROM;
	else if ((!enable) && (reg & CC_PROM))
		reg &= ~CC_PROM;
	else
		return;

	amac_core_init_reset(privp); /* Put GMAC in reset */
	writel(reg, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_OFFSET));
	amac_core_clear_reset(privp);
}

/**
 * bcm_amac_set_rx_mode() - Set the rx mode callback
 * @ndev: net device pointer
 *
 * The API enables multicast or promiscous mode as required. Otherwise
 * it disables multicast and promiscous mode and adds ARL entries.
 */
void bcm_amac_set_rx_mode(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!netif_running(ndev))
		return;

	if (ndev->flags & (IFF_PROMISC | IFF_ALLMULTI)) {
		/* Enable all multicast packets or promiscous mode */
		if (privp->switchmode)
			/* Enable all multicast in switch mode */
			bcm_esw_enable_multicast(privp, 1);
		else
		/* Enable promiscous mode in switch bypass mode */
			amac_set_prom(privp, 1);

		return;
	}

	if (privp->switchmode)
		/* Disable multicast in switch mode */
		bcm_esw_enable_multicast(privp, 0);
	else
		/* Disable promiscous in switch bypass mode */
		amac_set_prom(privp, 0);


	/* Add ARL entries for multicast packet filtering.
	 * The Kernel or n/w stack may add multicast MAC address
	 * filtering.
	 *
	 * In switch by pass mode we can only enable promiscous mode
	 * to allow all packets.
	 */
	if ((ndev->flags & IFF_MULTICAST) && netdev_mc_count(ndev)) {
		if (privp->switchmode) {
			struct netdev_hw_addr *ha;
			/* Do not clear ARL table here as other ARL entry
			 * such as local MAC unicast entry will be
			 * deleted as well.
			 * bcm_esw_set_arl_entry handles possile
			 * duplicated entries
			 */

			netdev_for_each_mc_addr(ha, ndev)
				/* Add new ARL entry and filter based on mac */
				bcm_esw_set_arl_entry(privp,
					ha->addr,
					0,
					0,
					PORT_MASK_ALL,
					true);

		} else
			/* In switch bypass mode there is no mac filtering
			 * so enable promiscous mode to pass all packets up.
			 */
			amac_set_prom(privp, 1);
	}
}

/**
 * bcm_amac_set_mac() - Adds the MAC to the ARL in switch mode
 * @privp: driver data pointer
 * @macp: mac address
 *
 * Returns: '0' or error
 */
int bcm_amac_set_mac(struct bcm_amac_priv *privp, char *macp)
{
	int rc = 0;

	if (privp->switchmode == 0) {
		/* Switch Bypass mode */
		dev_info(&privp->pdev->dev,
			"%s: Not adding MAC to ARL in Switch bypass mode\n",
			__func__);
	} else {
		/* Switch mode */
		dev_info(&privp->pdev->dev,
			"%s: Set MAC to ARL in Switch mode\n", __func__);
		rc =  bcm_esw_set_arl_entry(privp, macp, 0, 0,
			PORT_INTERNAL, true);
	}

	return rc;
}
