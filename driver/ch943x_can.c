#include "ch943x.h"

#define TX_MAILBOX_NR 3

struct rx_mailbox_info {
    u32 rxmdh; /* high byte of the receiving email address */
    u32 rxmdl; /* low byte of the receiving email address */
    u32 rxmdt; /* receiving email data length and timestamp */
    u32 rxmir; /* receiving email identifier */
};

struct tx_mailbox_info {
    u32 txmdh; /* high byte of the send email address */
    u32 txmdl; /* low byte of the send email address */
    u32 txmdt; /* send email data length and timestamp */
    u32 txmir; /* send email identifier */
};

static int ch943x_can_init(struct ch943x *s)
{
    u32 reg_val;
    unsigned long timeout;
    struct can_bittiming *bt = &s->priv->can.bittiming;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_CTLR);
    reg_val &= ~CAN_CTLR_SLEEP;
    ch943x_canreg_write(s, CH9434D_CAN_CTLR, reg_val);

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_CTLR);
    reg_val |= CAN_CTLR_INRQ;
    ch943x_canreg_write(s, CH9434D_CAN_CTLR, reg_val);

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_CTLR);

    timeout = jiffies + HZ;
    while (((ch943x_canreg_read(s, CH9434D_CAN_STATR) & CAN_STATR_INAK) != CAN_STATR_INAK)) {
        usleep_range(CH943X_DELAY_MS * 1000, CH943X_DELAY_MS * 1000 * 2);
        if (time_after(jiffies, timeout)) {
            dev_err(s->dev, "ch9434d didn't enter in conf mode after reset\n");
            return -EBUSY;
        }
    }

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_CTLR);
    reg_val &= ~(CAN_CTLR_TTCM | CAN_CTLR_ABOM | CAN_CTLR_AWUM | CAN_CTLR_RFLM);
    reg_val |= CAN_CTLR_TXFP;
    reg_val |= CAN_CTLR_NART;
    ch943x_canreg_write(s, CH9434D_CAN_CTLR, reg_val);

    DRV_DEBUG(s->dev, "bitrate:%d sample_point:%d tq:%d prop_seg:%d phase_seg1:%d phase_seg2:%d sjw:%d brp:%d\n",
              bt->bitrate, bt->sample_point, bt->tq, bt->prop_seg, bt->phase_seg1, bt->phase_seg2, bt->sjw, bt->brp);

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_BTIMR);
    reg_val = ((bt->sjw - 1) << 24) | ((bt->prop_seg + bt->phase_seg1 - 1) << 16) | ((bt->phase_seg2 - 1) << 20) |
              ((bt->brp - 1) & 0x000001FF);
    ch943x_canreg_write(s, CH9434D_CAN_BTIMR, reg_val);

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_CTLR);
    reg_val &= ~CAN_CTLR_INRQ;
    ch943x_canreg_write(s, CH9434D_CAN_CTLR, reg_val);

    timeout = jiffies + HZ;
    while (((ch943x_canreg_read(s, CH9434D_CAN_STATR) & CAN_STATR_INAK) == CAN_STATR_INAK)) {
        usleep_range(CH943X_DELAY_MS * 1000, CH943X_DELAY_MS * 1000 * 2);
        if (time_after(jiffies, timeout)) {
            dev_err(s->dev, "ch9434d didn't enter in conf mode after reset\n");
            return -EBUSY;
        }
    }

    return 0;
}

static void ch943x_can_filterinit(struct ch943x *s)
{
    u32 filter_num = 0;
    u32 filter_bit = 0;
    u32 reg_val;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    for (filter_num = 0; filter_num < 1; filter_num++) {
        filter_bit = BIT(filter_num);

        reg_val = ch943x_canreg_read(s, CH9434D_CAN_FCTLR);
        reg_val |= CAN_FCTLR_FINIT;
        ch943x_canreg_write(s, CH9434D_CAN_FCTLR, reg_val);

        reg_val = ch943x_canreg_read(s, CH9434D_CAN_FWR);
        reg_val &= ~filter_bit;
        ch943x_canreg_write(s, CH9434D_CAN_FWR, reg_val);

        reg_val = ch943x_canreg_read(s, CH9434D_CAN_FSCFGR);
        reg_val |= filter_bit;
        ch943x_canreg_write(s, CH9434D_CAN_FSCFGR, reg_val);

        reg_val = 0x00000000;
        ch943x_canreg_write(s, CH9434D_CAN_FxR1(filter_num), reg_val);
        reg_val = 0x00000000;
        ch943x_canreg_write(s, CH9434D_CAN_FxR2(filter_num), reg_val);

        reg_val = ch943x_canreg_read(s, CH9434D_CAN_FMCFGR);
        reg_val &= ~filter_bit;
        ch943x_canreg_write(s, CH9434D_CAN_FMCFGR, reg_val);

        reg_val = ch943x_canreg_read(s, CH9434D_CAN_FAFIFOR);
        reg_val &= ~filter_bit;
        ch943x_canreg_write(s, CH9434D_CAN_FAFIFOR, reg_val);

        reg_val = ch943x_canreg_read(s, CH9434D_CAN_FWR);
        reg_val |= filter_bit;
        ch943x_canreg_write(s, CH9434D_CAN_FWR, reg_val);

        reg_val = ch943x_canreg_read(s, CH9434D_CAN_FCTLR);
        reg_val &= ~CAN_FCTLR_FINIT;
        ch943x_canreg_write(s, CH9434D_CAN_FCTLR, reg_val);
    }
}

#ifdef CAN_TX_CONTMODE
static u8 ch943x_handle_can_tx_contmode(struct ch943x *s, struct can_frame *frame, int idx)
{
    u8 tx_mailbox = 0;
    struct tx_mailbox_info txinfo[2];
    u8 txbuf[32] = {0};
    u8 data[16] = {0};
    u32 sid = 0, eid = 0, exide = 0, rtr = 0;
    int retval;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    tx_mailbox = idx;

    exide = (frame->can_id & CAN_EFF_FLAG) ? 1 : 0; /* Extended ID Enable */
    if (exide) {
        eid = frame->can_id & CAN_EFF_MASK;         /* Extended ID */
        sid = (frame->can_id & CAN_EFF_MASK) >> 18; /* Standard ID */
    } else {
        sid = frame->can_id & CAN_SFF_MASK; /* Standard ID */
    }
    rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0; /* Remote transmission */

    if (exide)
        txinfo[0].txmir = ((eid << 3) | CAN_Id_Extended | (rtr << 1)) | CAN_TXMIRx_TXRQ;
    else
        txinfo[0].txmir = ((sid << 21) | (rtr << 1)) | CAN_TXMIRx_TXRQ;

    txinfo[0].txmdt = (frame->can_dlc & 0x0000000F);
    memcpy(data, frame->data, frame->can_dlc);
    txinfo[0].txmdl = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | (data[0]);
    txinfo[0].txmdh = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | (data[4]);

    memcpy(txbuf, &txinfo[0], 16);
    retval = ch943x_txmailbox_write(s, CH9434D_CAN_TX0WRITE_CONT + tx_mailbox, 16, txbuf);

    return tx_mailbox;
}
#else
static u8 ch943x_handle_can_tx(struct ch943x *s, struct can_frame *frame, int idx)
{
    u8 tx_mailbox = 0;
    u32 reg_val;
    u32 sid = 0, eid = 0, exide = 0, rtr = 0;
    u8 data[16] = {0};
    u32 tmp;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    exide = (frame->can_id & CAN_EFF_FLAG) ? 1 : 0; /* Extended ID Enable */
    if (exide) {
        eid = frame->can_id & CAN_EFF_MASK;         /* Extended ID */
        sid = (frame->can_id & CAN_EFF_MASK) >> 18; /* Standard ID */
    } else {
        sid = frame->can_id & CAN_SFF_MASK; /* Standard ID */
    }
    rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0; /* Remote transmission */

    tx_mailbox = idx;
    if (tx_mailbox == 0)
        tmp = CAN_TSTATR_TME0;
    else if (tx_mailbox == 1)
        tmp = CAN_TSTATR_TME1;
    else if (tx_mailbox == 2)
        tmp = CAN_TSTATR_TME2;
    reg_val = ch943x_canreg_read(s, CH9434D_CAN_TSTATR);
    if ((reg_val & tmp) == 0) {
    }

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4);
    reg_val &= CAN_TXMIRx_TXRQ;
    ch943x_canreg_write(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4, reg_val);

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4);
    if (exide)
        reg_val |= ((eid << 3) | CAN_Id_Extended | (rtr << 1));
    else
        reg_val |= ((sid << 21) | (rtr << 1));
    ch943x_canreg_write(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4, reg_val);

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_TXMDTR0 + tx_mailbox * 4);
    reg_val |= (frame->can_dlc & 0x0000000F);
    ch943x_canreg_write(s, CH9434D_CAN_TXMDTR0 + tx_mailbox * 4, reg_val);

    memcpy(data, frame->data, frame->can_dlc);
    reg_val = ((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | (data[0]));
    ch943x_canreg_write(s, CH9434D_CAN_TXMDLR0 + tx_mailbox * 4, reg_val);
    reg_val = ((data[7] << 24) | (data[6] << 16) | (data[5] << 8) | (data[4]));
    ch943x_canreg_write(s, CH9434D_CAN_TXMDHR0 + tx_mailbox * 4, reg_val);

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4);
    reg_val |= CAN_TXMIRx_TXRQ;
    ch943x_canreg_write(s, CH9434D_CAN_TXMIR0 + tx_mailbox * 4, reg_val);

    return 0;
}
#endif

static void ch943x_tx_work_handler(struct work_struct *ws)
{
    struct can_tx_work *tx_work = container_of(ws, struct can_tx_work, work);
    struct ch943x_can_priv *priv = tx_work->priv;
    struct ch943x *s = priv->s;
    struct net_device *ndev = priv->ndev;
    struct can_frame *frame;
    struct sk_buff *skb;
    int ret;

    if (!priv || !s || !ndev) {
        kfree(tx_work);
        return;
    }
    DRV_DEBUG(s->dev, "%s\n", __func__);

    if (priv->can.state == CAN_STATE_BUS_OFF) {
        kfree(tx_work);
        return;
    }
    skb = tx_work->skb;
    frame = (struct can_frame *)tx_work->skb->data;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
    frame->len = min_t(u8, frame->len, 8);
#else
    frame->can_dlc = min_t(u8, frame->can_dlc, 8);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
    can_put_echo_skb(skb, ndev, tx_work->tx_mailbox_id, 0);
#else
    can_put_echo_skb(skb, ndev, tx_work->tx_mailbox_id);
#endif

#ifdef CAN_TX_CONTMODE
    ret = ch943x_handle_can_tx_contmode(s, frame, tx_work->tx_mailbox_id);
#else
    ret = ch943x_handle_can_tx(s, frame, tx_work->tx_mailbox_id);
#endif
    kfree(tx_work);
}

static int ch943x_hw_sleep(struct ch943x *s)
{
    unsigned long timeout;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    ch943x_canreg_write(s, CH9434D_CAN_CTLR, CAN_CTLR_RESET);
    timeout = jiffies + HZ;
    while ((ch943x_canreg_read(s, CH9434D_CAN_STATR) & CAN_STATR_SLAK) != CAN_STATR_SLAK) {
        usleep_range(CH943X_DELAY_MS * 1000, CH943X_DELAY_MS * 1000 * 2);
        if (time_after(jiffies, timeout)) {
            dev_err(s->dev, "ch9434d didn't enter in conf mode after reset\n");
            return -EBUSY;
        }
    }

    return 0;
}

static int ch943x_setup(struct net_device *net, struct ch943x *s)
{
    int ret;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    ret = ch943x_can_init(s);
    ch943x_can_filterinit(s);

    return 0;
}

static void ch943x_restart_work_handler(struct work_struct *ws)
{
    struct ch943x_can_priv *priv = container_of(ws, struct ch943x_can_priv, restart_work);
    struct ch943x *s = priv->s;
    int ret;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    ch943x_hw_sleep(s);
    netif_device_attach(priv->ndev);

    ret = ch943x_setup(priv->ndev, s);

    netif_wake_queue(priv->ndev);
}

static int ch943x_set_mode(struct ch943x *s)
{
    struct ch943x_can_priv *priv = s->priv;
    unsigned long timeout;
    u32 reg_val;

    DRV_DEBUG(s->dev, "%s ctrlmode:0x%x\n", __func__, priv->can.ctrlmode);

    /* Enable interrupts */
    reg_val = ch943x_canreg_read(s, CH9434D_CAN_INTENR);
    reg_val |= (CAN_INTENR_FFIE0 | CAN_INTENR_FOVIE0 | CAN_INTENR_FFIE1 | CAN_INTENR_FOVIE1 | CAN_INTENR_EWGIE |
                CAN_INTENR_EPVIE | CAN_INTENR_BOFIE | CAN_INTENR_LECIE | CAN_INTENR_ERRIE | CAN_INTENR_WKUIE |
                CAN_INTENR_SLKIE | CAN_INTENR_FMPIE0 | CAN_INTENR_FMPIE1);
    reg_val |= CAN_INTENR_TMEIE;
    ch943x_canreg_write(s, CH9434D_CAN_INTENR, reg_val);

    if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
        /* Put device into loopback mode */
        reg_val = ch943x_canreg_read(s, CH9434D_CAN_BTIMR);
        reg_val |= CAN_BTIMR_LBKM;
        ch943x_canreg_write(s, CH9434D_CAN_BTIMR, reg_val);
    } else if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
        /* Put device into listen-only mode */
        reg_val = ch943x_canreg_read(s, CH9434D_CAN_BTIMR);
        reg_val |= CAN_BTIMR_SILM;
        ch943x_canreg_write(s, CH9434D_CAN_BTIMR, reg_val);
    } else {
        /* Put device into normal mode */
        reg_val = ch943x_canreg_read(s, CH9434D_CAN_CTLR);
        reg_val &= ~CAN_CTLR_INRQ;
        ch943x_canreg_write(s, CH9434D_CAN_CTLR, reg_val);

        /* Wait for the device to enter normal mode */
        timeout = jiffies + HZ;
        while ((ch943x_canreg_read(s, CH9434D_CAN_STATR) & CAN_STATR_INAK) == CAN_STATR_INAK) {
            usleep_range(CH943X_DELAY_MS * 1000, CH943X_DELAY_MS * 1000 * 2);
            if (time_after(jiffies, timeout)) {
                dev_err(s->dev, "ch9434d didn't enter in conf mode after reset\n");
                return -EBUSY;
            }
        }
    }

    return 0;
}

static int ch943x_stop(struct net_device *ndev)
{
    struct ch943x_can_priv *priv = netdev_priv(ndev);
    struct ch943x *s = priv->s;
    u32 reg_val;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    close_candev(ndev);

    reg_val = 0;
    ch943x_canreg_write(s, CH9434D_CAN_INTENR, reg_val);
    ch943x_hw_sleep(s);
    priv->can.state = CAN_STATE_STOPPED;
    atomic_set(&s->priv->can_isopen, 0);
    return 0;
}

static netdev_tx_t ch943x_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
    struct ch943x_can_priv *priv = netdev_priv(ndev);
    struct ch943x *s = priv->s;
    unsigned long flags;
    int i = 0;
    int tx_mailbox_id = -1;
    struct can_tx_work *tx_work;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    if (can_dropped_invalid_skb(ndev, skb))
        return NETDEV_TX_OK;

    for (i = 0; i < TX_MAILBOX_NR; i++) {
        if (!priv->tx_busy[i] && ((priv->txm_pendbits & BIT(i)) == 0)) {
            tx_mailbox_id = i;
            spin_lock_irqsave(&priv->tx_lock, flags);
            priv->tx_busy[i] = true;
            priv->txm_pendbits |= BIT(i);
            spin_unlock_irqrestore(&priv->tx_lock, flags);
            break;
        }
    }

    if (tx_mailbox_id < 0) {
        netif_stop_queue(ndev);
        return NETDEV_TX_BUSY;
    }

    tx_work = kmalloc(sizeof(*tx_work), GFP_ATOMIC);
    if (!tx_work) {
        pr_err("kmallor error\n");
        return -ENOMEM;
    }
    INIT_WORK(&tx_work->work, ch943x_tx_work_handler);
    tx_work->tx_mailbox_id = tx_mailbox_id;
    tx_work->skb = skb;
    tx_work->priv = priv;
    queue_work(priv->wq, &tx_work->work);

    return NETDEV_TX_OK;
}

static const struct ethtool_ops ch943x_ethtool_ops = {
    .get_ts_info = ethtool_op_get_ts_info,
};

static const struct can_bittiming_const ch943x_bittiming_const = {
    .name = "ch943x_can",
    .tseg1_min = 2,
    .tseg1_max = 16,
    .tseg2_min = 1,
    .tseg2_max = 8,
    .sjw_max = 4,
    .brp_min = 1,
    .brp_max = 256,
    .brp_inc = 1,
};

static int ch943x_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
    struct ch943x_can_priv *priv = netdev_priv(ndev);
    struct ch943x *s = priv->s;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    switch (mode) {
    case CAN_MODE_START:
        priv->can.state = CAN_STATE_ERROR_ACTIVE;
        queue_work(priv->wq, &priv->restart_work);
        break;
    default:
        return -EOPNOTSUPP;
    }

    return 0;
}

/*
 * Open can device
 */
static int ch943x_open(struct net_device *ndev)
{
    struct ch943x_can_priv *priv = netdev_priv(ndev);
    struct ch943x *s = priv->s;
    int ret;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    ret = open_candev(ndev);
    if (ret) {
        dev_err(s->dev, "unable to set initial baudrate!\n");
        return ret;
    }

    atomic_set(&s->priv->can_isopen, 1);

    priv->force_quit = 0;

    INIT_WORK(&priv->tx_work, ch943x_tx_work_handler);
    INIT_WORK(&priv->restart_work, ch943x_restart_work_handler);

    ret = ch943x_setup(ndev, s);
    if (ret)
        return -1;

    ret = ch943x_set_mode(s);
    if (ret)
        return -1;

    return 0;
}

static const struct net_device_ops ch943x_netdev_ops = {
    .ndo_open = ch943x_open,
    .ndo_stop = ch943x_stop,
    .ndo_start_xmit = ch943x_start_xmit,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
    .ndo_change_mtu = can_change_mtu,
#endif
};

int ch943x_can_register(struct ch943x *s)
{
    struct net_device *ndev;
    int ret = 0;
    int i;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    s->priv = NULL;

    /* Allocate can/net device */
    ndev = alloc_candev(sizeof(struct ch943x_can_priv), 1);
    if (!ndev)
        return -ENOMEM;

    ndev->netdev_ops = &ch943x_netdev_ops;
    ndev->ethtool_ops = &ch943x_ethtool_ops;
    ndev->flags |= IFF_ECHO;

    s->priv = netdev_priv(ndev);
    if (!s->priv) {
        dev_err(s->dev, "Failed to get private data\n");
        return -ENOMEM;
    }

    atomic_set(&s->priv->can_isopen, 0);
    s->priv->s = s;
    s->priv->can.bittiming_const = &ch943x_bittiming_const;
    s->priv->can.do_set_mode = ch943x_do_set_mode;
    s->priv->can.clock.freq = 96000000;
    s->priv->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES | CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY;
    s->priv->ndev = ndev;

    s->priv->wq = alloc_ordered_workqueue("ch943x_wq", WQ_MEM_RECLAIM);
    if (!s->priv->wq) {
        ret = -ENOMEM;
        goto error_probe;
    }

    spin_lock_init(&s->priv->tx_lock);
    s->priv->txm_pendbits = 0;
    s->priv->can.echo_skb_max = 3;

    for (i = 0; i < TX_MAILBOX_NR; i++) {
        s->priv->tx_busy[i] = false;
    }

    ret = ch943x_hw_sleep(s);
    if (ret < 0)
        goto error_probe;

    SET_NETDEV_DEV(ndev, s->dev);

    ret = register_candev(ndev);
    if (ret)
        goto error_probe;

    return 0;

error_probe:
    free_candev(ndev);

    return ret;
}

void ch943x_can_remove(struct ch943x *s)
{
    unregister_candev(s->priv->ndev);
}

#ifdef CAN_RX_CONTMODE
static int ch943x_hw_rx_contmode(struct ch943x *s, int rx_mailbox)
{
    struct ch943x_can_priv *priv = s->priv;
    struct sk_buff *skb;
    struct can_frame *frame;
    u32 exide;
    u8 buf[16] = {0};
    int i;
    int ret;
    struct rx_mailbox_info rxinfo[4];
    u8 rxbuf[64];
    u8 cmpbuf[16] = {0};
    int frame_nums = 0;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    ret = ch943x_rxmailbox_read(s, CH9434D_CAN_RX0READ_CONT + rx_mailbox, rxbuf);
    for (i = 0; i < 4; i++) {
        memcpy(rxinfo + i, rxbuf + (i * 16), 16);
        if (memcmp(rxinfo + i, cmpbuf, 16) == 0) {
            break;
        }
        frame_nums++;

        skb = alloc_can_skb(priv->ndev, &frame);
        if (!skb) {
            dev_err(s->dev, "cannot allocate RX skb\n");
            priv->ndev->stats.rx_dropped++;
            return -1;
        }

        exide = 0x04 & rxinfo[i].rxmir;
        if (exide) {
            /* Extended ID format */
            frame->can_id = CAN_EFF_FLAG;
            frame->can_id |= ((rxinfo[i].rxmir >> 3) & 0x1FFFFFFF);
            /* Remote transmission request */
            if (0x02 & rxinfo[i].rxmir)
                frame->can_id |= CAN_RTR_FLAG;
        } else {
            /* Standard ID format */
            frame->can_id = ((rxinfo[i].rxmir >> 21) & 0x000007FF);
        }

        /* Data length */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
        frame->len = can_cc_dlc2len(rxinfo[i].rxmdt & 0x0F);
#else
        frame->can_dlc = get_can_dlc(rxinfo[i].rxmdt & 0x0F);
#endif
        buf[0] = 0xFF & rxinfo[i].rxmdl;
        buf[1] = 0xFF & (rxinfo[i].rxmdl >> 8);
        buf[2] = 0xFF & (rxinfo[i].rxmdl >> 16);
        buf[3] = 0xFF & (rxinfo[i].rxmdl >> 24);

        buf[4] = 0xFF & rxinfo[i].rxmdh;
        buf[5] = 0xFF & (rxinfo[i].rxmdh >> 8);
        buf[6] = 0xFF & (rxinfo[i].rxmdh >> 16);
        buf[7] = 0xFF & (rxinfo[i].rxmdh >> 24);
        memcpy(frame->data, buf, frame->can_dlc);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
        priv->ndev->stats.rx_bytes += frame->len;
#else
        priv->ndev->stats.rx_bytes += frame->can_dlc;
#endif
        priv->ndev->stats.rx_packets++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
        netif_rx(skb);
#else
        netif_rx_ni(skb);
#endif
    }
    return frame_nums;
}
#else
static void ch943x_hw_rx(struct ch943x *s, int rx_mailbox)
{
    struct ch943x_can_priv *priv = s->priv;
    struct sk_buff *skb;
    struct can_frame *frame;
    u32 reg_val;
    u32 exide;
    u8 buf[16] = {0};

    DRV_DEBUG(s->dev, "%s\n", __func__);

    skb = alloc_can_skb(priv->ndev, &frame);
    if (!skb) {
        dev_err(s->dev, "cannot allocate RX skb\n");
        priv->ndev->stats.rx_dropped++;
        return;
    }

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_RXMIR0 + rx_mailbox * 4);
    exide = 0x04 & reg_val;
    if (exide) {
        /* Extended ID format */
        frame->can_id = CAN_EFF_FLAG;
        frame->can_id |= ((reg_val >> 3) & 0x1FFFFFFF);
        /* Remote transmission request */
        if ((u8)0x02 & reg_val)
            frame->can_id |= CAN_RTR_FLAG;
    } else {
        /* Standard ID format */
        frame->can_id = ((reg_val >> 21) & 0x000007FF);
    }

    /* Data length */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
    reg_val = ch943x_canreg_read(s, CH9434D_CAN_RXMDTR0 + rx_mailbox * 4);
    frame->len = can_cc_dlc2len(reg_val & 0x0F);
#else
    reg_val = ch943x_canreg_read(s, CH9434D_CAN_RXMDTR0 + rx_mailbox * 4);
    frame->can_dlc = get_can_dlc(reg_val & 0x0F);
#endif

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_RXMDLR0 + rx_mailbox * 4);
    buf[0] = 0xFF & reg_val;
    buf[1] = 0xFF & (reg_val >> 8);
    buf[2] = 0xFF & (reg_val >> 16);
    buf[3] = 0xFF & (reg_val >> 24);
    reg_val = ch943x_canreg_read(s, CH9434D_CAN_RXMDHR0 + rx_mailbox * 4);
    buf[4] = 0xFF & reg_val;
    buf[5] = 0xFF & (reg_val >> 8);
    buf[6] = 0xFF & (reg_val >> 16);
    buf[7] = 0xFF & (reg_val >> 24);
    memcpy(frame->data, buf, frame->can_dlc);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
    priv->ndev->stats.rx_bytes += frame->len;
#else
    priv->ndev->stats.rx_bytes += frame->can_dlc;
#endif

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_RFIFO0 + rx_mailbox);
    reg_val |= CAN_RFIFOx_RFOMx;
    ch943x_canreg_write(s, CH9434D_CAN_RFIFO0 + rx_mailbox, reg_val);

    priv->ndev->stats.rx_packets++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
    netif_rx(skb);
#else
    netif_rx_ni(skb);
#endif
}
#endif

static void ch943x_can_err(struct ch943x *s, u32 fifo0_state, u32 fifo1_state, u32 err_state)
{
    struct net_device *ndev = s->priv->ndev;
    struct sk_buff *skb;
    struct can_frame *frame;
    struct net_device_stats *stats = &ndev->stats;
    enum can_state new_state;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    skb = alloc_can_err_skb(ndev, &frame);
    if (!skb)
        return;

    if ((fifo0_state & CAN_RFIFOx_FOVRx) || (fifo1_state & CAN_RFIFOx_FOVRx)) {
        dev_err(s->dev, "CAN data overrun\n");
        frame->can_id |= CAN_ERR_CRTL;
        frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
        stats->rx_over_errors++;
        stats->rx_errors++;
    }

    /* Update can state */
    if (err_state & CAN_ERRSR_EWGF) {
        new_state = CAN_STATE_ERROR_ACTIVE;
    } else if (err_state & (CAN_ERRSR_EPVF | CAN_ERRSR_REC)) {
        new_state = CAN_STATE_ERROR_PASSIVE;
        frame->can_id |= CAN_ERR_CRTL;
        frame->data[1] = CAN_ERR_CRTL_RX_PASSIVE;
    } else if (err_state & (CAN_ERRSR_EPVF | CAN_ERRSR_TEC)) {
        new_state = CAN_STATE_ERROR_PASSIVE;
        frame->can_id |= CAN_ERR_CRTL;
        frame->data[1] = CAN_ERR_CRTL_TX_PASSIVE;
    } else if (err_state & CAN_ERRSR_BOFF) {
        new_state = CAN_STATE_BUS_OFF;
        frame->can_id |= CAN_ERR_BUSOFF;
        s->priv->can.can_stats.bus_off++;
        can_bus_off(ndev);
        ch943x_hw_sleep(s);
    }

    /* Update can state statistics */
    switch (s->priv->can.state) {
    case CAN_STATE_ERROR_ACTIVE:
        if (new_state >= CAN_STATE_ERROR_WARNING && new_state <= CAN_STATE_BUS_OFF)
            s->priv->can.can_stats.error_warning++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
        fallthrough;
#endif
    case CAN_STATE_ERROR_WARNING: /* fallthrough */
        if (new_state >= CAN_STATE_ERROR_PASSIVE && new_state <= CAN_STATE_BUS_OFF)
            s->priv->can.can_stats.error_passive++;
        break;
    default:
        break;
    }
    s->priv->can.state = new_state;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
    netif_rx(skb);
#else
    netif_rx_ni(skb);
#endif
}

void ch943x_can_irq(struct ch943x *s)
{
    u32 reg_val, tmp = 0;
    u32 err_state;
    int i, j;
    int frame_nums;
    u32 fifo0_state, fifo1_state, fifo_state;
    struct net_device *ndev = s->priv->ndev;
    struct ch943x_can_priv *priv = netdev_priv(ndev);
    unsigned long flags;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_STATR);
    tmp = reg_val;
    if (reg_val & CAN_STATR_ERRI)
        tmp |= CAN_STATR_ERRI;
    if (reg_val & CAN_STATR_WKUI)
        tmp |= CAN_STATR_WKUI;
    if (reg_val & CAN_STATR_SLAKI)
        tmp |= CAN_STATR_SLAKI;
    if (reg_val & (CAN_STATR_ERRI | CAN_STATR_WKUI | CAN_STATR_SLAKI))
        ch943x_canreg_write(s, CH9434D_CAN_STATR, tmp);

    reg_val = ch943x_canreg_read(s, CH9434D_CAN_TSTATR);
    tmp = 0;
    if (reg_val & (CAN_TSTATR_RQCP0 | CAN_TSTATR_RQCP1 | CAN_TSTATR_RQCP2)) {
        for (i = 0; i < TX_MAILBOX_NR; i++) {
            if (reg_val & (CAN_TSTATR_RQCP0 << (i * 8))) {
                tmp |= (0x0F << (i * 8));
            }
        }
        ch943x_canreg_write(s, CH9434D_CAN_TSTATR, tmp);
    }

    if (reg_val & (CAN_TSTATR_RQCP0 | CAN_TSTATR_RQCP1 | CAN_TSTATR_RQCP2)) {
        for (i = 0; i < TX_MAILBOX_NR; i++) {
            if (reg_val & (CAN_TSTATR_RQCP0 << (i * 8))) {
                ndev->stats.tx_packets++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
                ndev->stats.tx_bytes += can_get_echo_skb(ndev, i, NULL);
#else
                ndev->stats.tx_bytes += can_get_echo_skb(ndev, i);
#endif
                spin_lock_irqsave(&priv->tx_lock, flags);
                if (priv->tx_busy[i]) {
                    priv->tx_busy[i] = false;
                }
                if (priv->txm_pendbits & BIT(i)) {
                    priv->txm_pendbits &= ~BIT(i);
                }
                spin_unlock_irqrestore(&priv->tx_lock, flags);
            }
        }
        netif_wake_queue(ndev);
    }

    for (i = 0; i < 2; i++) {
        fifo_state = ch943x_canreg_read(s, CH9434D_CAN_RFIFO0 + i);
        if (i == 0)
            fifo0_state = fifo_state;
        else
            fifo1_state = fifo_state;
        if (fifo_state & (CAN_RFIFOx_FMPx | CAN_RFIFOx_FULLx | CAN_RFIFOx_FOVRx)) {
            ch943x_canreg_write(s, CH9434D_CAN_RFIFO0 + i, CAN_RFIFOx_FULLx | CAN_RFIFOx_FOVRx);
#ifdef CAN_RX_CONTMODE
            for (j = 0; j < 4; j++) {
                frame_nums = ch943x_hw_rx_contmode(s, i);
                if (frame_nums < 4) {
                    break;
                }
            }
#else
            frame_nums = fifo_state & 0x000000FF;
            for (j = 0; j < frame_nums; j++)
                ch943x_hw_rx(s, 0);
#endif
        }
    }

    err_state = ch943x_canreg_read(s, CH9434D_CAN_ERRSR);
    if (err_state) {
        if (err_state & (u32)0x70)
            ch943x_canreg_write(s, CH9434D_CAN_ERRSR, 0x70);
        else
            ch943x_canreg_write(s, CH9434D_CAN_ERRSR, 0x00);
    }

    if ((fifo0_state & CAN_RFIFOx_FOVRx) || (fifo1_state & CAN_RFIFOx_FOVRx) || err_state) {
        ch943x_can_err(s, fifo0_state, fifo1_state, err_state);
    }
}
