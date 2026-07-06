#include "ch943x.h"

#define REGS_BUFSIZE 4096

struct ctrl_info {
    uint8_t cmd;    /* optional command identifier */
    uint32_t len;   /* length of data[] in bytes */
    uint8_t data[]; /* variable-length payload */
} __attribute__((packed));

#ifdef USE_SPI_MODE
static inline void spi_delay_set(struct spi_transfer *xfer, unsigned int value)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19)
    xfer->delay.value = value;
#else
    xfer->delay_usecs = value;
#endif
}

static int ch943x_spi_write(struct ch943x *s, u8 cmd, int n_tx, u8 *txbuf, u32 flag)
{
    u8 can_cmd = CH943X_REG_OP_WRITE | CH943X_CANREG_CMD;
    u8 reg_cmd = cmd;
    int retval;
    struct spi_message m;
    struct spi_transfer xfer[3] = {};

    spi_message_init(&m);
    switch (flag) {
    case TRASFER_FLAG_COMM_REG8:
    case TRASFER_FLAG_UART_REG8:
        xfer[0].tx_buf = &reg_cmd;
        xfer[0].len = 1;
        xfer[0].cs_change = 0;
        spi_delay_set(&xfer[0], CH943X_CMD_DELAY);

        xfer[1].tx_buf = txbuf;
        xfer[1].len = n_tx;
        xfer[1].cs_change = 0;
        spi_delay_set(&xfer[1], CH943X_CMD_DELAY);
        break;
    case TRASFER_FLAG_CAN_REG32:
        xfer[0].tx_buf = &can_cmd;
        xfer[0].len = 1;
        xfer[0].cs_change = 0;
        spi_delay_set(&xfer[0], CH943X_CMD_DELAY);

        xfer[1].tx_buf = txbuf;
        xfer[1].len = n_tx;
        xfer[1].cs_change = 0;
        spi_delay_set(&xfer[1], CH943X_CMD_DELAY);
        break;
    case TRASFER_FLAG_CAN_MAILBOX:
        xfer[0].tx_buf = &can_cmd;
        xfer[0].len = 1;
        xfer[0].cs_change = 0;
        spi_delay_set(&xfer[0], 5);

        xfer[1].tx_buf = &reg_cmd;
        xfer[1].len = 1;
        xfer[1].cs_change = 0;
        spi_delay_set(&xfer[1], 5);

        xfer[2].tx_buf = txbuf;
        xfer[2].len = n_tx;
        xfer[2].cs_change = 0;
        spi_delay_set(&xfer[2], 5);
        break;
    default:
        return -EINVAL;
    }

    spi_message_add_tail(&xfer[0], &m);
    spi_message_add_tail(&xfer[1], &m);
    if (flag == TRASFER_FLAG_CAN_MAILBOX)
        spi_message_add_tail(&xfer[2], &m);

    mutex_lock(&s->mutex_bus_access);
    retval = spi_sync(s->spi_dev, &m);

    if (((cmd & 0x0f) == CH943X_FCR_REG) && (flag == TRASFER_FLAG_UART_REG8)) {
        udelay(2000);
    }

    mutex_unlock(&s->mutex_bus_access);
    if (retval < 0) {
        dev_err(s->dev, "%s spi transfer failed\n", __func__);
        return retval;
    }

    return 0;
}

static int ch943x_spi_read(struct ch943x *s, u8 cmd, int n_rx, u8 *rxbuf, u32 flag)
{
    u8 can_cmd = CH943X_CANREG_CMD;
    u8 reg_cmd = cmd;
    int retval;
    struct spi_message m;
    struct spi_transfer xfer[3] = {};

    spi_message_init(&m);
    switch (flag) {
    case TRASFER_FLAG_COMM_REG8:
    case TRASFER_FLAG_UART_REG8:
        xfer[0].tx_buf = &reg_cmd;
        xfer[0].len = 1;
        xfer[0].cs_change = 0;
        spi_delay_set(&xfer[0], CH943X_CMD_DELAY);

        xfer[1].rx_buf = rxbuf;
        xfer[1].len = n_rx;
        xfer[1].cs_change = 0;
        spi_delay_set(&xfer[1], CH943X_CMD_DELAY);
        break;
    case TRASFER_FLAG_CAN_REG32:
    case TRASFER_FLAG_CAN_MAILBOX:
        xfer[0].tx_buf = &can_cmd;
        xfer[0].len = 1;
        xfer[0].cs_change = 0;
        spi_delay_set(&xfer[0], 2);

        xfer[1].tx_buf = &reg_cmd;
        xfer[1].len = 1;
        xfer[1].cs_change = 0;
        spi_delay_set(&xfer[1], 6);

        xfer[2].rx_buf = rxbuf;
        xfer[2].len = n_rx;
        xfer[2].cs_change = 0;
        spi_delay_set(&xfer[2], 3);
        break;
    default:
        return -EINVAL;
    }

    spi_message_add_tail(&xfer[0], &m);
    spi_message_add_tail(&xfer[1], &m);
    if ((flag == TRASFER_FLAG_CAN_REG32) || (flag == TRASFER_FLAG_CAN_MAILBOX))
        spi_message_add_tail(&xfer[2], &m);

    mutex_lock(&s->mutex_bus_access);
    retval = spi_sync(s->spi_dev, &m);
    mutex_unlock(&s->mutex_bus_access);
    if (retval < 0) {
        dev_err(s->dev, "%s spi transfer failed\n", __func__);
        return retval;
    }

    return 0;
}
#elif defined(USE_I2C_MODE)
static int ch943x_i2c_write(struct ch943x *s, u8 cmd, int n_tx, u8 *txbuf, u32 flag)
{
    struct i2c_client *i2c = s->i2c;
    struct i2c_msg xfer[2] = {};
    int index = 0;
    int ret;

    mutex_lock(&s->mutex_bus_access);
    switch (flag) {
    case TRASFER_FLAG_COMM_REG8:
    case TRASFER_FLAG_UART_REG8:
        s->local_buf[index++] = cmd;
        memcpy(s->local_buf + index, txbuf, n_tx);
        index += n_tx;
        break;
    case TRASFER_FLAG_CAN_REG32:
        s->local_buf[index++] = CH943X_REG_OP_WRITE | CH943X_CANREG_CMD;
        memcpy(s->local_buf + index, txbuf, n_tx);
        index += n_tx;
        break;
    default:
        mutex_unlock(&s->mutex_bus_access);
        return -EINVAL;
    }

    xfer[0].addr = i2c->addr;
    xfer[0].flags = 0;
    xfer[0].len = index;
    xfer[0].buf = s->local_buf;

    ret = i2c_transfer(i2c->adapter, xfer, 1);

    if (((cmd & 0x0f) == CH943X_FCR_REG) && (flag == TRASFER_FLAG_UART_REG8)) {
        udelay(2000);
    }
    mutex_unlock(&s->mutex_bus_access);
    if (ret != 1) {
        dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
        return -EIO;
    }

    return 0;
}

static int ch943x_i2c_read(struct ch943x *s, u8 cmd, int n_rx, u8 *rxbuf, u32 flag)
{
    struct i2c_client *i2c = s->i2c;
    struct i2c_msg xfer[2] = {};
    u8 data[2] = {0};
    int ret;

    mutex_lock(&s->mutex_bus_access);
    switch (flag) {
    case TRASFER_FLAG_COMM_REG8:
    case TRASFER_FLAG_UART_REG8:
        data[0] = cmd;
        xfer[0].len = 1;
        break;
    case TRASFER_FLAG_CAN_REG32:
        data[0] = CH943X_REG_OP_READ | CH943X_CANREG_CMD;
        data[1] = cmd;
        xfer[0].len = 2;
        break;
    default:
        mutex_unlock(&s->mutex_bus_access);
        return -EINVAL;
    }

    xfer[0].addr = i2c->addr;
    xfer[0].flags = 0;
    xfer[0].buf = data;

    xfer[1].addr = i2c->addr;
    xfer[1].flags = I2C_M_RD;
    xfer[1].len = n_rx;
    xfer[1].buf = rxbuf;

    ret = i2c_transfer(i2c->adapter, xfer, 2);
    mutex_unlock(&s->mutex_bus_access);
    if (ret != 2) {
        dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
        return -EIO;
    }

    return 0;
}
#elif defined(USE_SERIAL_MODE)
static int ch943x_serial_write(struct ch943x *s, u8 cmd, int n_tx, u8 *txbuf, u32 flag)
{
    int ret;

    mutex_lock(&s->mutex_bus_access);
    s->local_buf[0] = 0x57;
    s->local_buf[1] = 0xab;
    s->local_buf[2] = cmd;
    memcpy(s->local_buf + 3, txbuf, n_tx);

    ret = ch943x_ctrl_tty_write(s, n_tx + 3, s->local_buf);
    mutex_unlock(&s->mutex_bus_access);
    if (ret < 0) {
        dev_err(s->dev, "%s control uart write error. ret:%d\n", __func__, ret);
        return ret;
    }

    return 0;
}

static int ch943x_serial_read(struct ch943x *s, u8 cmd, int n_rx, u8 *rxbuf, u32 flag)
{
    u8 txbuf[3] = {0x57, 0xab, cmd};
    int ret;

    mutex_lock(&s->mutex_bus_access);
    ret = ch943x_ctrl_tty_write(s, 3, txbuf);
    if (ret < 0) {
        dev_err(s->dev, "%s control uart write error. ret:%d\n", __func__, ret);
        mutex_unlock(&s->mutex_bus_access);
        return ret;
    }
    ret = ch943x_ctrl_tty_read(s, n_rx, rxbuf);
    mutex_unlock(&s->mutex_bus_access);
    if (ret < 0) {
        dev_err(s->dev, "%s control uart error. ret:%d\n", __func__, ret);
        return ret;
    }

    return 0;
}
#endif

struct ch943x_bus_ops ch943x_bus_ops = {
#ifdef USE_SPI_MODE
    .write = ch943x_spi_write,
    .read = ch943x_spi_read,
#elif defined(USE_I2C_MODE)
    .write = ch943x_i2c_write,
    .read = ch943x_i2c_read,
#elif defined(USE_SERIAL_MODE)
    .write = ch943x_serial_write,
    .read = ch943x_serial_read,
#endif
};

int ch943x_reg_write(struct ch943x *s, u8 reg, u32 n_tx, u8 *txbuf)
{
    int ret;
    u8 cmd = reg | CH943X_REG_OP_WRITE;

    ret = s->ops->write(s, cmd, n_tx, txbuf, TRASFER_FLAG_COMM_REG8);
    if (ret < 0)
        return ret;

    if (n_tx == 1) {
        DRV_DEBUG(s->dev, "%s %02X %02X\n", __func__, cmd, txbuf[0]);
    } else if (n_tx == 4) {
    } else {
        DRV_DEBUG(s->dev, "%s cmd:%02X n_tx:%d\n", __func__, cmd, n_tx);
        DRV_DEBUG_HEXDUMP("hex: ", DUMP_PREFIX_NONE, 32, 1, txbuf, n_tx, false);
    }

    return 0;
}

int ch943x_reg_read(struct ch943x *s, u8 reg, u32 n_rx, u8 *rxbuf)
{
    int ret;
    u8 cmd = reg;

    ret = s->ops->read(s, cmd, n_rx, rxbuf, TRASFER_FLAG_COMM_REG8);
    if (ret < 0)
        return ret;

    if (n_rx == 1) {
        DRV_DEBUG(s->dev, "%s %02X %02X\n", __func__, cmd, rxbuf[0]);
    } else if (n_rx == 4) {
    } else {
        DRV_DEBUG(s->dev, "%s cmd:%02X n_rx:%d\n", __func__, cmd, n_rx);
        DRV_DEBUG_HEXDUMP("hex: ", DUMP_PREFIX_NONE, 32, 1, rxbuf, n_rx, false);
    }

    return 0;
}

int ch943x_reg_update(struct ch943x *s, u8 reg, u8 mask, u8 val)
{
    u8 tmp;
    int ret;

    ret = ch943x_reg_read(s, reg, 1, &tmp);
    if (ret < 0)
        return ret;

    tmp &= ~mask;
    tmp |= (val & mask);
    return ch943x_reg_write(s, reg, 1, &tmp);
}

u8 ch943x_port_read(struct uart_port *port, u8 reg)
{
    struct ch943x *s = dev_get_drvdata(port->dev);
    u8 cmd = 0, val = 0;
    int ret;

    if ((s->chip.chiptype == CHIP_CH9438F) || (s->chip.chiptype == CHIP_CH9437F))
        cmd = 0x00 | (port->line * 8 + reg);
    else
        cmd = (0x00 | reg) + (port->line * 0x10);

    ret = s->ops->read(s, cmd, 1, &val, TRASFER_FLAG_UART_REG8);
    if (ret < 0)
        return ret;
    DRV_DEBUG(s->dev, "%s %02X[u%d reg:%02X] %02X\n", __func__, cmd, port->line, reg, val);

    return val;
}

int ch943x_port_write(struct uart_port *port, u8 reg, u8 val)
{
    struct ch943x *s = dev_get_drvdata(port->dev);
    u8 cmd;
    u8 tmpval = val;
    int ret;

    if ((s->chip.chiptype == CHIP_CH9438F) || (s->chip.chiptype == CHIP_CH9437F))
        cmd = CH943X_REG_OP_WRITE | (port->line * 8 + reg);
    else
        cmd = (CH943X_REG_OP_WRITE | reg) + (port->line * 0x10);

    ret = s->ops->write(s, cmd, 1, &tmpval, TRASFER_FLAG_UART_REG8);
    if (ret < 0)
        return ret;
    DRV_DEBUG(s->dev, "%s %02X[u%d reg:%02X] %02X\n", __func__, cmd, port->line, reg, tmpval);

    return 0;
}

#ifdef CH9434D_CAN_ON
u32 ch943x_canreg_read(struct ch943x *s, u8 reg)
{
    int ret;
    uint8_t data[5] = {0};
    u32 reg_val = 0;

    ret = s->ops->read(s, reg, 4, data, TRASFER_FLAG_CAN_REG32);
    if (ret < 0)
        return ret;
    reg_val |= data[0];
    reg_val |= data[1] << 8;
    reg_val |= data[2] << 16;
    reg_val |= data[3] << 24;
    DRV_DEBUG(s->dev, "%s %02X %08X\n", __func__, reg, reg_val);

    return reg_val;
}

int ch943x_canreg_write(struct ch943x *s, u8 reg, u32 val)
{
    int ret;
    uint8_t data[5] = {0};

    data[0] = reg;
    data[1] = val & 0xff;
    data[2] = (val >> 8) & 0xff;
    data[3] = (val >> 16) & 0xff;
    data[4] = (val >> 24) & 0xff;

    ret = s->ops->write(s, 0, 5, data, TRASFER_FLAG_CAN_REG32); /* No cmd are used. The default value is 0 */
    if (ret < 0)
        return ret;
    DRV_DEBUG(s->dev, "%s %02X %08X\n", __func__, reg, val);

    return 0;
}

int ch943x_rxmailbox_read(struct ch943x *s, u8 reg, u32 n_rx, u8 *rxbuf)
{
    int ret;

    DRV_DEBUG(s->dev, "%s reg:%02x n_rx:%d\n", __func__, reg, n_rx);

    ret = s->ops->read(s, reg, n_rx, rxbuf, TRASFER_FLAG_CAN_MAILBOX);
    if (ret < 0)
        return ret;
    DRV_DEBUG_HEXDUMP("hex: ", DUMP_PREFIX_NONE, 32, 1, rxbuf, n_rx, false);

    return 0;
}

int ch943x_txmailbox_write(struct ch943x *s, u8 reg, u32 n_tx, u8 *txbuf)
{
    int ret;

    DRV_DEBUG(s->dev, "%s reg:%02x n_tx:%d\n", __func__, reg, n_tx);

    ret = s->ops->write(s, reg, n_tx, txbuf, TRASFER_FLAG_CAN_MAILBOX);
    if (ret < 0)
        return ret;
    DRV_DEBUG_HEXDUMP("hex: ", DUMP_PREFIX_NONE, 32, 1, txbuf, n_tx, false);

    return 0;
}
#endif

#ifdef USE_SERIAL_MODE
int ch943x_ctrl_tty_write(struct ch943x *s, u32 n_tx, const void *txbuf)
{
    int retlen;
    int pos = 0, total = 0, tmp_n_tx;
    loff_t offset = 0;
    unsigned long timeout_jiffies = jiffies + msecs_to_jiffies(500);
    int retry_count = 0;

    tmp_n_tx = n_tx;
    while (!time_after(jiffies, timeout_jiffies)) {
        retlen = kernel_write(s->fp, txbuf + pos, tmp_n_tx, &offset);
        if (retlen < 0) {
            /*
             * The tty layer may return -ERESTARTSYS (-512) if a signal is
             * pending (e.g. during close) or other transient errors.
             * Retry a few times before giving up to avoid interrupt storms
             * caused by failed register writes.
             */
            if (retry_count < 5) {
                retry_count++;
                mdelay(10);
                continue;
            }
            dev_err(s->dev, "kernel_write error. retlen:%d\n", retlen);
            return retlen;
        }

        retry_count = 0;
        if (retlen < tmp_n_tx) {
            tmp_n_tx -= retlen;
            pos += retlen;
        }
        total += retlen;
        if (total == n_tx) {
            return 0;
        }
    }

    dev_err(s->dev, "%s timeout, requested %u bytes, written %d bytes\n", __func__, n_tx, total);
    return -ETIMEDOUT;
}

int ch943x_ctrl_tty_read(struct ch943x *s, u32 n_rx, void *rxbuf)
{
    int retlen;
    int pos = 0, total = 0, tmp_n_rx;
    loff_t offset = 0;
    unsigned long timeout_jiffies = jiffies + msecs_to_jiffies(500);
    int retry_count = 0;

    tmp_n_rx = n_rx;
    while (!time_after(jiffies, timeout_jiffies)) {
        retlen = kernel_read(s->fp, rxbuf + pos, tmp_n_rx, &offset);
        if (retlen < 0) {
            if (retry_count < 5) {
                retry_count++;
                mdelay(10);
                continue;
            }
            pr_err("kernel_read error. retlen:%d\n", retlen);
            return retlen;
        }

        retry_count = 0;
        if (retlen < tmp_n_rx) {
            tmp_n_rx -= retlen;
            pos += retlen;
        }
        total += retlen;
        if (total == n_rx)
            return 0;
    }

    dev_err(s->dev, "%s timeout, requested %u bytes, read %d bytes\n", __func__, n_rx, total);
    return -ETIMEDOUT;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
int ch943x_ctrluart_setopt(struct ch943x *s)
{
    struct termios uart_io;
    struct termios2 tio;
    mm_segment_t fs;
    struct file *fp = s->fp;

    memset(&uart_io, 0, sizeof(uart_io));

    uart_io.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    uart_io.c_oflag &= ~OPOST;
    uart_io.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    uart_io.c_cflag &= ~(CSIZE | PARENB);
    uart_io.c_cflag |= CS8;
    uart_io.c_cflag &= ~CSTOPB;
    uart_io.c_cflag |= CREAD | CLOCAL;
    uart_io.c_cc[VMIN] = 0;
    uart_io.c_cc[VTIME] = 1;

    fs = get_fs();
    set_fs(KERNEL_DS);

    fp->f_op->unlocked_ioctl(fp, TCSETS, (unsigned long)&uart_io);
    fp->f_op->unlocked_ioctl(fp, TCFLSH, 0);

    fp->f_op->unlocked_ioctl(fp, TCGETS2, (unsigned long)&tio);
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
#ifdef MULTI_CHIP_MODE
    tio.c_ispeed = s->ctrluart_baud;
    tio.c_ospeed = s->ctrluart_baud;
#else
    tio.c_ispeed = CTRLUART_BAUD;
    tio.c_ospeed = CTRLUART_BAUD;
#endif

    fp->f_op->unlocked_ioctl(fp, TCSETS2, (unsigned long)&tio);
    fp->f_op->unlocked_ioctl(fp, TCGETS2, (unsigned long)&tio);
    set_fs(fs);

    return 0;
}
#else
int ch943x_ctrluart_setopt(struct ch943x *s)
{
    struct termios uart_io;
    struct termios2 tio;
    struct file *fp = s->fp;

    memset(&uart_io, 0, sizeof(uart_io));

    uart_io.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    uart_io.c_oflag &= ~OPOST;
    uart_io.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    uart_io.c_cflag &= ~(CSIZE | PARENB);
    uart_io.c_cflag |= CS8;
    uart_io.c_cflag &= ~CSTOPB;
    uart_io.c_cflag |= CREAD | CLOCAL;
    uart_io.c_cc[VMIN] = 0;
    uart_io.c_cc[VTIME] = 1;

    fp->f_op->unlocked_ioctl(fp, TCSETS, (unsigned long)&uart_io);
    fp->f_op->unlocked_ioctl(fp, TCFLSH, 0);

    fp->f_op->unlocked_ioctl(fp, TCGETS2, (unsigned long)&tio);
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
#ifdef MULTI_CHIP_MODE
    tio.c_ispeed = s->ctrluart_baud;
    tio.c_ospeed = s->ctrluart_baud;
#else
    tio.c_ispeed = CTRLUART_BAUD;
    tio.c_ospeed = CTRLUART_BAUD;
#endif

    fp->f_op->unlocked_ioctl(fp, TCSETS2, (unsigned long)&tio);
    fp->f_op->unlocked_ioctl(fp, TCGETS2, (unsigned long)&tio);

    return 0;
}
#endif

int ch9437_serialmode_fifo_read(struct ch943x *s, u8 cmd, u32 n_rx, u8 *rxbuf)
{
    u8 txbuf[8] = {0};
    int ret;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    txbuf[0] = 0x57;
    txbuf[1] = 0xab;
    txbuf[2] = cmd;
    txbuf[3] = (n_rx >> 8) & 0xFF;
    txbuf[4] = n_rx & 0xFF;

    mutex_lock(&s->mutex_bus_access);
    ret = ch943x_ctrl_tty_write(s, 5, txbuf);
    if (ret < 0) {
        mutex_unlock(&s->mutex_bus_access);
        return ret;
    }
    ret = ch943x_ctrl_tty_read(s, n_rx, rxbuf);
    mutex_unlock(&s->mutex_bus_access);
    if (ret < 0)
        return ret;

    DRV_DEBUG_HEXDUMP("hex: ", DUMP_PREFIX_NONE, 32, 1, rxbuf, n_rx, false);

    return 0;
}

int ch9437_serialmode_fifo_write(struct ch943x *s, u8 cmd, u32 n_tx, u8 *txbuf)
{
    u8 *buffer;
    int ret;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    buffer = kmalloc(2048, GFP_KERNEL);
    if (!buffer)
        return -ENOMEM;

    buffer[0] = 0x57;
    buffer[1] = 0xab;
    buffer[2] = cmd | CH943X_REG_OP_WRITE;
    buffer[3] = (n_tx >> 8) & 0xFF;
    buffer[4] = n_tx & 0x00FF;
    memcpy(buffer + 5, txbuf, n_tx);

    mutex_lock(&s->mutex_bus_access);
    ret = ch943x_ctrl_tty_write(s, n_tx + 5, buffer);
    mutex_unlock(&s->mutex_bus_access);
    kfree(buffer);

    DRV_DEBUG_HEXDUMP("hex: ", DUMP_PREFIX_NONE, 32, 1, txbuf, n_tx, false);

    return ret;
}
#endif  // USE_SERIAL_MODE

int ch943x_iofunc_get(struct ch943x *s, uint8_t io_cmd, uint8_t io_addr)
{
    u8 txbuf[4] = {0};
    u8 rxbuf[4] = {0};
    int ret = 0;
    int io_stat = -EBUSY;
    unsigned long timeout_jiffies = jiffies + msecs_to_jiffies(3000);

    txbuf[0] = io_cmd;
    txbuf[1] = io_addr;
    txbuf[2] = 0x00;
    txbuf[3] = CH9434X_IO_CMD_ACT;

    ret = ch943x_reg_write(s, CH943X_IO_SEL_FUN_CFG, 4, txbuf);
    if (ret < 0)
        return ret;
    udelay(1000);

    while (!time_after(jiffies, timeout_jiffies)) {
        ret = ch943x_reg_read(s, CH943X_IO_SEL_FUN_CFG, 4, rxbuf);
        if (ret < 0)
            return ret;

        if ((rxbuf[3] == CH9434X_IO_CMD_COMP) && (rxbuf[0] == txbuf[0])) {
            io_stat = rxbuf[2];
            break;
        }
        mdelay(1);
    }
    DRV_DEBUG(s->dev, "%s tx:%02x %02x %02x %02x rx:%02x %02x %02x %02x\n", __func__, txbuf[0], txbuf[1], txbuf[2],
              txbuf[3], rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);

    return io_stat;
}

int ch943x_iofunc_set(struct ch943x *s, u8 io_cmd, u8 io_addr, u8 enable)
{
    u8 txbuf[4] = {0};
    u8 rxbuf[4] = {0};
    int ret;
    int io_stat = -EBUSY;
    unsigned long timeout_jiffies = jiffies + msecs_to_jiffies(3000);

    txbuf[0] = io_cmd;
    txbuf[1] = io_addr;
    txbuf[2] = enable;
    txbuf[3] = CH9434X_IO_CMD_ACT;

    ret = ch943x_reg_write(s, CH943X_IO_SEL_FUN_CFG, 4, txbuf);
    if (ret < 0)
        return ret;
    udelay(1000);

    while (!time_after(jiffies, timeout_jiffies)) {
        ret = ch943x_reg_read(s, CH943X_IO_SEL_FUN_CFG, 4, rxbuf);
        if (ret < 0)
            return ret;

        if ((rxbuf[3] == CH9434X_IO_CMD_COMP) && (rxbuf[0] == txbuf[0]) && (rxbuf[1] == txbuf[1])) {
            io_stat = IOSTATE_ENABLE;
            break;
        }
        mdelay(1);
    }
    DRV_DEBUG(s->dev, "%s tx:%02x %02x %02x %02x rx:%02x %02x %02x %02x\n", __func__, txbuf[0], txbuf[1], txbuf[2],
              txbuf[3], rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);

    return io_stat;
}

int ch943x_get_chip_version(struct ch943x *s)
{
    int ret;
    bool valid_ver = true;

    ret = ch943x_reg_read(s, CH943X_CHIP_VER_REG, VER_LEN, s->chip.ver);
    if (ret < 0)
        return ret;
    DRV_DEBUG(s->dev, "%s reg:%02x data:%02x %02x %02x %02x\n", __func__, CH943X_CHIP_VER_REG, s->chip.ver[0],
              s->chip.ver[1], s->chip.ver[2], s->chip.ver[3]);

    if (s->chip.ver[2] == (s->chip.ver[0] + s->chip.ver[1])) {
        if (s->chip.ver[3] == 0x5A) {
#ifdef USE_SPI_MODE
            s->chip.chiptype = CHIP_CH9434A;
            s->chip.nr_uart = 4;
            s->chip.nr_gpio = 25;
            strcpy(s->chip.chip_name, "CH9434A");

            ch943x_reg_update(s, CH943X_SPI_CONT_MODE_REG, CH943X_SPI_CONTE_BIT, CH943X_SPI_CONTE_BIT);
            if (s->spi_dev->max_speed_hz > 2000000) {
                dev_err(s->dev, "CHIP:%s SPI cont-mode, CLK must not exceed 2MHz.\n", s->chip.chip_name);
                s->spi_dev->max_speed_hz = 2000000;
            }
            s->spi_contmode = true;
#endif
        } else if (s->chip.ver[3] == 0x6B) {
            s->chip.chiptype = CHIP_CH9434D;
            s->chip.nr_uart = 4;
            s->chip.nr_gpio = 4;
            strcpy(s->chip.chip_name, "CH9434D");
        } else if (s->chip.ver[3] == 0x7C) {
            s->chip.chiptype = CHIP_CH9438F;
            s->chip.nr_uart = 8;
            s->chip.nr_gpio = 8;
            strcpy(s->chip.chip_name, "CH9438");
        } else if (s->chip.ver[3] == 0x8D) {
            s->chip.chiptype = CHIP_CH9437F;
            s->chip.nr_uart = 8;
            s->chip.nr_gpio = 11;
            strcpy(s->chip.chip_name, "CH9437");
        } else if (s->chip.ver[3] == 0x9E) {
            s->chip.chiptype = CHIP_CH9432D;
            s->chip.nr_uart = 2;
            s->chip.nr_gpio = 8;
            strcpy(s->chip.chip_name, "CH9432");
        } else {
            dev_err(s->dev, "Unknown chip version byte: 0x%02x\n", s->chip.ver[3]);
            return -ENODEV;
        }
    } else {
#ifdef USE_SPI_MODE
        s->chip.chiptype = CHIP_CH9434M;
        s->chip.nr_uart = 4;
        s->chip.nr_gpio = 25;
        valid_ver = false;
        s->spi_contmode = false;
#endif
    }

    if (valid_ver) {
        dev_info(s->dev, "CHIP TYPE:%s - V%d.%d\n", s->chip.chip_name, s->chip.ver[1], s->chip.ver[0]);
    } else {
        dev_info(s->dev, "No valid version:%02x %02x %02x %02x\n", s->chip.ver[0], s->chip.ver[1], s->chip.ver[2],
                 s->chip.ver[3]);
    }

    return 0;
}

int ch943x_io_enable(struct ch943x *s)
{
    int i;

    s->tnow_enable_bits = 0x00;
    s->extern_clock_on = false;
    s->can_on = false;

#ifdef MULTI_CHIP_MODE
#ifdef USE_SPI_MODE
    /**
     * Situation 1. A single SPI host connects multiple SPI slaves, using hardware CS.
     *
     * Distinguish the chips through the chip_select field
     * and configure the TNOW pin for enabling, the XO/XI
     * pins for enabling, and the CAN pin for enabling.
     *
     * if (s->spi_dev->chip_select == 0) {
     *     s->tnow_enable_bits = 0x00;
     *     s->extern_clock_on = false;
     *     s->can_on = false;
     * } else if (s->spi_dev->chip_select == 1) {
     * }
     *
     * Situation 2. A single SPI host connects multiple SPI slaves, using software CS.
     *
     * Distinguish the chips through the cs_gpio field
     * and configure the TNOW pin for enabling, the XO/XI
     * pins for enabling, and the CAN pin for enabling.
     *
     * if (s->spi_dev->cs_gpio == 0) {
     * } else if (s->spi_dev->cs_gpio == 1) {
     * }
     *
     * Situation 3. Multiple SPI hosts connect to multiple SPI slaves
     *
     * Distinguish chips based on the SPI bus number(bus_num),
     * and configure the TNOW pin for enabling, the XO/XI
     * pins for enabling, and the CAN pin for enabling.
     *
     * if (s->spi_dev->master->bus_num == 0) {
     * } else if (s->spi_dev->master->bus_num == 1) {
     * }
     */
#elif defined(USE_I2C_MODE)
    /**
     * Situation 1. Single host connecting multiple slaves.
     * Identify chips based on I2C slave addresses.
     *
     * if (s->i2c->addr == 0x2a) {
     * } else if (s->i2c->addr == 0x2b) {
     * }
     *
     * Situation 2. Multiple hosts connect to multiple slaves,
     * and the chips are distinguished based on the I2C bus number.
     *
     * if (s->i2c->adapter->nr == 0) {
     * } else if (s->i2c->adapter->nr == 1) {
     * }
     */
#elif defined(USE_SERIAL_MODE)
    /**
     * The SERIAL mode requires initializing the control serial port
     * before enabling the IO.(ctrluart_init function)
     */
#endif
#else
    for (i = 0; i < 7; i++) {
        if (CH943X_TNOW_ENABLE(i)) {
            s->tnow_enable_bits |= BIT(i);
        }
    }
    if (CH943X_EXCLK_ENABLE) {
        s->extern_clock_on = true;
    }
#ifdef CH9434D_CAN_ON
    if (s->chip.chiptype == CHIP_CH9434D)
        s->can_on = true;
#endif
#endif

    /*
     * Only after adding the above code can the following
     * ch943x_iofunc_set function be executed!
     */
    if ((s->chip.chiptype == CHIP_CH9434A) || (s->chip.chiptype == CHIP_CH9434M))
        return 0;

    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_INT_ADD, 1); /* Interrupt Enable */

    for (i = 0; i < s->chip.nr_uart; i++) {
        if ((s->chip.chiptype == CHIP_CH9437F) && IS_USE_SERIAL_MODE && (i == 0))
            continue;
        ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH943X_DEF_U0_ADD + i, 1); /* UART Tx/Rx Enable */
    }

    if (s->chip.chiptype == CHIP_CH9434D) {
        if (s->extern_clock_on)
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9434D_DEF_HSE_ADD, 1); /* External Clock Enable */

        if (s->can_on)
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9434D_DEF_CAN_ADD, 1); /* CAN Tx/Rx Enable */

        if ((s->tnow_enable_bits & BIT(0)) && (!s->can_on))
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW0_ADD, 1); /* TNOW0 Enable */
        if ((s->tnow_enable_bits & BIT(1)) && (!s->can_on))
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW1_ADD, 1); /* TNOW1 Enable */
        if (s->tnow_enable_bits & BIT(2))
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW2_ADD, 1); /* TNOW2 Enable */
        if ((s->tnow_enable_bits & BIT(3)) && (!s->extern_clock_on))
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW3_ADD, 1); /* TNOW3 Enable */

        if (((s->tnow_enable_bits & BIT(0)) == 0) && (!s->can_on))
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9434D_DEF_CTS0_ADD, 1); /* Modem CTS0 Enable */
        if ((s->tnow_enable_bits & BIT(2)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9434D_DEF_CTS3_ADD, 1); /* Modem CTS3 Enable */
        if (((s->tnow_enable_bits & BIT(1)) == 0) && (!s->can_on))
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9434D_MUL_RTS0_ADD, 1); /* Modem RTS0 Enable */
    } else if (s->chip.chiptype == CHIP_CH9438F) {
        if (s->extern_clock_on)
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH943X_DEF_HSE_ADD, 1); /* External Clock Enable */

        for (i = 0; i < s->chip.nr_uart; i++) {
            if (i == 1) {
                continue;
            } else if (i == 7) {
                if ((s->tnow_enable_bits & BIT(7)) && (!s->extern_clock_on))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW7_ADD, 1); /* TNOW7 Enable */
            } else {
                if (s->tnow_enable_bits & BIT(i))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW0_ADD + i, 1); /* TNOWx Enable */
            }
        }

        if ((s->tnow_enable_bits & BIT(2)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS2_ADD, 1); /* RTS2 Enable */
        if ((s->tnow_enable_bits & BIT(4)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS3_ADD, 1); /* RTS3 Enable */
        if ((s->tnow_enable_bits & BIT(6)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS4_ADD, 1); /* RTS4 Enable */
        if ((s->tnow_enable_bits & BIT(0)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS1_ADD, 1); /* CTS1 Enable */
        if ((s->tnow_enable_bits & BIT(3)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS2_ADD, 1); /* CTS2 Enable */
        if ((s->tnow_enable_bits & BIT(5)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS3_ADD, 1); /* CTS3 Enable */
        if (((s->tnow_enable_bits & BIT(7)) == 0) && !s->extern_clock_on)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS4_ADD, 1); /* CTS4 Enable */
    } else if (s->chip.chiptype == CHIP_CH9437F) {
        if (s->extern_clock_on)
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH943X_DEF_HSE_ADD, 1); /* External Clock Enable */

        for (i = 0; i < s->chip.nr_uart; i++) {
            if (IS_USE_SERIAL_MODE && (i == 0))
                continue;
            if (i == 7)
                continue;
            if (s->tnow_enable_bits & BIT(i))
                ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW0_ADD + i, 1); /* TNOWx Enable */
        }

        if ((s->tnow_enable_bits & BIT(1)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS0_ADD, 1); /* RTS0 Enable */
        if ((s->tnow_enable_bits & BIT(2)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS2_ADD, 1); /* RTS2 Enable */
        if ((s->tnow_enable_bits & BIT(4)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS3_ADD, 1); /* RTS3 Enable */
        if ((s->tnow_enable_bits & BIT(6)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS4_ADD, 1); /* RTS4 Enable */
        if ((s->tnow_enable_bits & BIT(0)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS0_ADD, 1); /* CTS0 Enable */
        if ((s->tnow_enable_bits & BIT(3)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS2_ADD, 1); /* CTS2 Enable */
        if ((s->tnow_enable_bits & BIT(5)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS3_ADD, 1); /* CTS3 Enable */
        if (!(s->extern_clock_on))
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS4_ADD, 1); /* CTS4 Enable */
        if (s->chip.interface_mode != I2C_MODE)
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS5_ADD, 1); /* CTS5 Enable */
    } else if (s->chip.chiptype == CHIP_CH9432D) {
        if (s->extern_clock_on)
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9432_DEF_HSE_ADD, 1); /* External Clock Enable */

        for (i = 0; i < s->chip.nr_uart; i++) {
            if (s->tnow_enable_bits & BIT(i))
                ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW0_ADD + i, 1); /* TNOWx Enable */
        }

        ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9432_MUL_RTS0_ADD, 1); /* RTS0 Enable */
        ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9432_MUL_RTS1_ADD, 1); /* RTS1 Enable */
        ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9432_DEF_CTS0_ADD, 1);   /* CTS0 Enable */
        ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9432_DEF_CTS1_ADD, 1);   /* CTS1 Enable */
    }

    return 0;
}

int ch943x_ioctl_do(struct ch943x *s, unsigned int cmd, unsigned long arg)
{
    struct ctrl_info __user *user_info = (struct ctrl_info __user *)arg;
    struct ctrl_info *kern_info = NULL;
    uint32_t info_size, data_len;
    u16 __user *argval = (u16 __user *)arg;
    u32 __user *argval_chiptype = (u32 __user *)arg;
    u16 inarg;
    u8 gpionumber, enable;
    int ret = 0;

    /* Require CAP_SYS_ADMIN for register read/write and FIFO operations */
    if ((cmd == IOCTL_CTRL_WRITE) || (cmd == IOCTL_CTRL_READ) || (cmd == IOCTL_CTRL_SERIALMODE_FIFO_WRITE) ||
        (cmd == IOCTL_CTRL_SERIALMODE_FIFO_READ)) {
        if (!capable(CAP_SYS_ADMIN))
            return -EPERM;
    }

    if ((cmd == IOCTL_CTRL_WRITE) || (cmd == IOCTL_CTRL_READ) || (cmd == IOCTL_CTRL_SERIALMODE_FIFO_WRITE) ||
        (cmd == IOCTL_CTRL_SERIALMODE_FIFO_READ)) {
        if (copy_from_user(&data_len, (void __user *)(&user_info->len), sizeof(user_info->len)))
            return -EFAULT;
        /* Prevent integer overflow and unreasonable allocations */
        if (data_len > 4096)
            return -EINVAL;
        info_size = sizeof(*kern_info) + data_len;

        kern_info = kmalloc(info_size, GFP_KERNEL);
        if (!kern_info)
            return -ENOMEM;

        if (copy_from_user(kern_info, (void __user *)user_info, info_size)) {
            ret = -EFAULT;
            goto err;
        }
    }

    switch (cmd) {
    case IOCTL_CMD_GETCHIPTYPE:
        if (put_user(s->chip.chiptype, argval_chiptype)) {
            ret = -EFAULT;
            goto err;
        }
        break;
    case IOCTL_CMD_CH9434D_GPIOENABLE:
    case IOCTL_CMD_CH9438_GPIOENABLE:
    case IOCTL_CMD_CH9437_GPIOENABLE:
    case IOCTL_CMD_CH9432_GPIOENABLE:
        if (get_user(inarg, argval)) {
            ret = -EFAULT;
            goto err;
        }
        gpionumber = (inarg >> 8) & 0xFF;
        enable = inarg & 0xFF;

        if (gpio_is_available(s, gpionumber) == false) {
            dev_err(s->dev, "%s - GPIO%d is unavailable.\n", __func__, gpionumber);
            ret = -EFAULT;
            goto err;
        }

        ret = gpio_muxfunc_enable(s, gpionumber);
        if (ret < 0)
            goto err;
        ret = gpio_reg_enable(s, gpionumber, enable);
        if (ret < 0)
            goto err;
        break;
    case IOCTL_CTRL_WRITE:
        ret = ch943x_reg_write(s, kern_info->cmd, kern_info->len, kern_info->data);
        if (ret < 0)
            goto err;
        break;
    case IOCTL_CTRL_READ:
        ret = ch943x_reg_read(s, kern_info->cmd, kern_info->len, kern_info->data);
        if (ret < 0)
            goto err;

        if (copy_to_user((void __user *)user_info->data, kern_info->data, kern_info->len)) {
            ret = -EFAULT;
            goto err;
        }
        break;
#ifdef USE_SERIAL_MODE
    case IOCTL_CTRL_SERIALMODE_FIFO_WRITE:
        ret = ch9437_serialmode_fifo_write(s, kern_info->cmd, kern_info->len, kern_info->data);
        if (ret < 0) {
            ret = -EFAULT;
            goto err;
        }
        break;
    case IOCTL_CTRL_SERIALMODE_FIFO_READ:
        ret = ch9437_serialmode_fifo_read(s, kern_info->cmd, kern_info->len, kern_info->data);
        if (ret < 0) {
            ret = -EFAULT;
            goto err;
        }
        if (copy_to_user((void __user *)user_info->data, kern_info->data, kern_info->len)) {
            ret = -EFAULT;
            goto err;
        }
        break;
#endif
    default:
        return -ENOIOCTLCMD;
    }

err:
    if ((cmd == IOCTL_CTRL_WRITE) || (cmd == IOCTL_CTRL_READ) || (cmd == IOCTL_CTRL_SERIALMODE_FIFO_WRITE) ||
        (cmd == IOCTL_CTRL_SERIALMODE_FIFO_READ)) {
        kfree(kern_info);
    }
    return ret;
}

static ssize_t ch943x_proc_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
    /**
     * #if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
     * struct ch943x *s = PDE_DATA(file_inode(file));
     * #else
     * struct ch943x *s = pde_data(file_inode(file));
     * #endif
     */
    char buf[16] = {0};
    ssize_t ret;

    if (!(file->f_mode & FMODE_WRITE)) {
        pr_err("%s can't write\n", __func__);
        return -EACCES;
    }

    if (count > (sizeof(buf) - 1))
        return -EINVAL;
    if (count == 0)
        return 0;

    ret = copy_from_user(buf, user_buf, count);
    if (ret)
        return ret;

    buf[count] = '\0';
    if (buf[count - 1] == '\n') {
        buf[count - 1] = '\0';
    }

    /**
     * It can be used for debug
     */

    return count;
}

static ssize_t ch943x_proc_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
    struct ch943x *s = PDE_DATA(file_inode(file));
#else
    struct ch943x *s = pde_data(file_inode(file));
#endif
    struct ch943x_one *p;
    char *buf;
    u32 len = 0;
    ssize_t ret;
    int i, j;
    u8 lcr, val;
#ifdef CH9434D_CAN_ON
    u32 reg_val;
#endif

    if (!s) {
        return -ENODEV;
    }

    buf = kzalloc(REGS_BUFSIZE, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    len += snprintf(buf + len, REGS_BUFSIZE - len, "============ch943x registers:============\n");
    for (i = 0; i < s->chip.nr_uart; i++) {
        if ((s->chip.chiptype == CHIP_CH9437F) && IS_USE_SERIAL_MODE && (i == 0))
            continue;
        p = s->p + i;

        lcr = ch943x_port_read(&p->port, CH943X_LCR_REG);
        ch943x_port_write(&p->port, CH943X_LCR_REG, CH943X_LCR_CONF_MODE_A);

        len += snprintf(buf + len, REGS_BUFSIZE - len, "============UART%d Dump register at DLAB=1============\n", i);
        for (j = 0; j < 2; j++) {
            val = ch943x_port_read(&p->port, j);
            len += snprintf(buf + len, REGS_BUFSIZE - len, "reg:0x%02x val:0x%02x\n", j, val);
        }

        ch943x_port_write(&p->port, CH943X_LCR_REG, lcr);
        msleep(1);

        len += snprintf(buf + len, REGS_BUFSIZE - len, "============UART%d Dump register at DLAB=0============\n", i);
        for (j = 0; j < 8; j++) {
            if ((s->chip.chiptype == CHIP_CH9437F) && IS_USE_SERIAL_MODE && (j == 0))
                continue;
            val = ch943x_port_read(&p->port, j);
            len += snprintf(buf + len, REGS_BUFSIZE - len, "reg:0x%02x val:0x%02x\n", j, val);
        }
    }

#ifdef CH9434D_CAN_ON
    if (s->can_on && (s->chip.chiptype == CHIP_CH9434D)) {
        for (i = 0x00; i < 0x45; i++) {
            reg_val = ch943x_canreg_read(s, i);
            len += snprintf(buf + len, REGS_BUFSIZE - len, "reg:0x%02x val:0x%08x\n", i, reg_val);
        }
    }
#endif

    ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);

    kfree(buf);
    return ret;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0))
static const struct file_operations ch943x_regs_ops = {
    .owner = THIS_MODULE,
    .read = ch943x_proc_read,
    .write = ch943x_proc_write,
};
#else
static const struct proc_ops ch943x_regs_ops = {
    .proc_read = ch943x_proc_read,
    .proc_write = ch943x_proc_write,
};
#endif

int ch943x_debugfs_init(struct ch943x *s)
{
    struct proc_dir_entry *proc_entry;

    snprintf(s->proc_file_name, sizeof(s->proc_file_name), "ch943x%d_reg", s->minor);
    proc_entry = proc_create_data(s->proc_file_name, S_IRUGO | S_IWUGO, NULL, &ch943x_regs_ops, s);
    if (!proc_entry) {
        dev_err(s->dev, "Failed to create /proc/%s\n", s->proc_file_name);
        return -ENOMEM;
    }

    return 0;
}

void ch943x_debugfs_exit(struct ch943x *s)
{
    remove_proc_entry(s->proc_file_name, NULL);
}
