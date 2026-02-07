#include "ch943x.h"

#define SPI_W_CAN_DELAY1_US 3

#define SPI_W_CAN_ADD_SYNC_CODE  0x22
#define SPI_W_CAN_ADD_INCP_CODE  0x24
#define SPI_W_CAN_DEAL_SYNC_CODE 0x28
#define SPI_W_CAN_DEAL_INCP_CODE 0x30

#define SPI_R_CAN_ADD_SYNC_CODE  0x22
#define SPI_R_CAN_ADD_INCP_CODE  0x24
#define SPI_R_CAN_DATA_SYNC_CODE 0x28
#define SPI_R_CAN_DATA_INCP_CODE 0x30

#define W_WAIT_CAN_ADD_SYNC_CODE 0x42
#define W_RSP_CAN_DATA_INCP_CODE 0x44
#define R_WAIT_CAN_ADD_SYNC_CODE 0x42
#define R_RSP_CAN_DATA_INCP_CODE 0x44

#define REGS_BUFSIZE 4096

static inline void spi_delay_set(struct spi_transfer *xfer, unsigned int value)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19)
    xfer->delay.value = value;
#else
    xfer->delay_usecs = value;
#endif
}

#ifdef CH943X_CANREG_NOTIMEINTER
static int calculate_transfer_size(struct spi_device *spi, unsigned int time_us)
{
    unsigned long spi_freq;
    unsigned long long total_bits;
    int bytes;

    spi_freq = spi->max_speed_hz;

    total_bits = (unsigned long long)time_us * spi_freq;
    bytes = total_bits / (8 * 1000000);

    if (total_bits % (8 * 1000000) != 0)
        bytes++;

    bytes += 3;

    return bytes;
}
#endif

#ifdef USE_SERIAL_MODE
int ch943x_ctrl_tty_write(struct ch943x *s, u32 n_tx, const void *txbuf)
{
    int retlen;
    int pos = 0, total = 0, tmp_n_tx;
    loff_t offset = 0;
    unsigned long timeout_jiffies = jiffies + msecs_to_jiffies(3000);
    bool timeout_occurred = false;

    if (!s || !s->fp || !txbuf || (n_tx <= 0)) {
        pr_err("ch943x_ctrl_tty_write error.\n");
        return -EINVAL;
    }

    tmp_n_tx = n_tx;
    while (1) {
        if (time_after(jiffies, timeout_jiffies)) {
            timeout_occurred = true;
            break;
        }

        retlen = kernel_write(s->fp, txbuf + pos, tmp_n_tx, &offset);
        if (retlen < 0) {
            pr_err("kernel_write error. retlen:%d txbuf:%02x %02x %02x %02x %02x\n", retlen, *((u8 *)(txbuf + 0)),
                   *((u8 *)(txbuf + 1)), *((u8 *)(txbuf + 2)), *((u8 *)(txbuf + 3)), *((u8 *)(txbuf + 4)));
            return retlen;
        }

        if (retlen < tmp_n_tx) {
            tmp_n_tx -= retlen;
            pos += retlen;
        }
        total += retlen;
        if (total == n_tx)
            break;
    }

    if (timeout_occurred) {
        pr_warn("ch943x_ctrl_tty_write timeout, requested %u bytes, read %d bytes\n", n_tx, total);
        return -ETIMEDOUT;
    }

    return 0;
}

int ch943x_ctrl_tty_read(struct ch943x *s, u32 n_rx, void *rxbuf)
{
    int retlen;
    int pos = 0, total = 0, tmp_n_rx;
    loff_t offset = 0;
    unsigned long timeout_jiffies = jiffies + msecs_to_jiffies(3000);
    bool timeout_occurred = false;

    tmp_n_rx = n_rx;
    while (1) {
        if (time_after(jiffies, timeout_jiffies)) {
            timeout_occurred = true;
            break;
        }

        retlen = kernel_read(s->fp, rxbuf + pos, tmp_n_rx, &offset);
        if (retlen < 0) {
            pr_err("kernel_read error. retlen:%d\n", retlen);
            return retlen;
        }

        if (retlen < tmp_n_rx) {
            tmp_n_rx -= retlen;
            pos += retlen;
        }

        total += retlen;
        if (total == n_rx)
            break;
    }

    if (timeout_occurred) {
        pr_warn("ch943x_ctrl_tty_read timeout, requested %u bytes, read %d bytes\n", n_rx, total);
        return -ETIMEDOUT;
    }

    return 0;
}

int ch9437_serialmode_fifo_read(struct ch943x *s, uint8_t cmd, u32 rxlen, uint8_t *rxbuf)
{
    u8 txbuf[8] = {0};
    int retval;

    txbuf[0] = 0x57;
    txbuf[1] = 0xab;
    txbuf[2] = cmd;
    txbuf[3] = (rxlen >> 8) & 0xFF;
    txbuf[4] = rxlen & 0xFF;

    mutex_lock(&s->mutex_bus_access);
    retval = ch943x_ctrl_tty_write(s, 5, txbuf);
    if (retval < 0) {
        dev_err(s->dev, "%s control uart write error. retval:%d\n", __func__, retval);
        mutex_unlock(&s->mutex_bus_access);
        return retval;
    }
    retval = ch943x_ctrl_tty_read(s, rxlen, rxbuf);
    if (retval < 0) {
        dev_err(s->dev, "%s control uart read error. retval:%d\n", __func__, retval);
        mutex_unlock(&s->mutex_bus_access);
        return retval;
    }
    DRV_DEBUG_HEXDUMP("ch9437_serialmode_fifo_read tx: ", DUMP_PREFIX_NONE, 32, 1, txbuf, 5, false);
    DRV_DEBUG_HEXDUMP("ch9437_serialmode_fifo_read rx: ", DUMP_PREFIX_NONE, 32, 1, rxbuf, rxlen, false);

    mutex_unlock(&s->mutex_bus_access);

    return 0;
}

static int ch9437_serialmode_fifo_write(struct ch943x *s, uint8_t cmd, int txlen, uint8_t *txbuf)
{
    int retval = 0;
    u8 *buffer;

    buffer = kmalloc(2048, GFP_KERNEL);
    if (!buffer)
        return -ENOMEM;

    buffer[0] = 0x57;
    buffer[1] = 0xab;
    buffer[2] = cmd;
    buffer[3] = (txlen >> 8) & 0xFF;
    buffer[4] = txlen & 0x00FF;
    memcpy(buffer + 5, txbuf, txlen);
    DRV_DEBUG_HEXDUMP("ch9437_serialmode_fifo_write tx: ", DUMP_PREFIX_NONE, 32, 1, buffer, txlen + 5, false);

    mutex_lock(&s->mutex_bus_access);
    retval = ch943x_ctrl_tty_write(s, txlen + 5, buffer);
    if (retval < 0) {
        dev_err(s->dev, "%s control uart write error. retval:%d\n", __func__, retval);
        mutex_unlock(&s->mutex_bus_access);
        goto tty_write_err;
    }
    mutex_unlock(&s->mutex_bus_access);
tty_write_err:
    kfree(buffer);
    return retval;
}

int pack_ch9437_registers(struct ch943x *s, uint8_t *packet)
{
    int i, j;

    i = 0;
    for (j = 1; j < 8; j++) {
        packet[i++] = 0x57;
        packet[i++] = 0xAB;
        packet[i++] = (j * 8 + CH943X_IIR_REG) | 0x00;

        packet[i++] = 0x57;
        packet[i++] = 0xAB;
        packet[i++] = (j * 8 + CH943X_LSR_REG) | 0x00;

        packet[i++] = 0x57;
        packet[i++] = 0xAB;
        packet[i++] = (j * 8 + CH943X_MSR_REG) | 0x00;

        packet[i++] = 0x57;
        packet[i++] = 0xAB;
        packet[i++] = CH943X_FIFO_REG | CH943X_REG_OP_WRITE;
        packet[i++] = j | CH943X_FIFO_RD_BIT;

        packet[i++] = 0x57;
        packet[i++] = 0xAB;
        packet[i++] = (CH943X_FIFOCL_REG);

        packet[i++] = 0x57;
        packet[i++] = 0xAB;
        packet[i++] = (CH943X_FIFOCH_REG);

        packet[i++] = 0x57;
        packet[i++] = 0xAB;
        packet[i++] = CH943X_FIFO_REG | CH943X_REG_OP_WRITE;
        packet[i++] = j | CH943X_FIFO_WR_BIT;

        packet[i++] = 0x57;
        packet[i++] = 0xAB;
        packet[i++] = (CH943X_FIFOCL_REG);

        packet[i++] = 0x57;
        packet[i++] = 0xAB;
        packet[i++] = (CH943X_FIFOCH_REG);
    }

    return i;
}

int ch9437_reg_bulkread_serialmode(struct ch943x *s, u8 *buf)
{
    u8 txbuf[256] = {0};
    u8 rxbuf[256] = {0};
    int n_tx;
    int n_rx;
    int ret;
    int reg_cnts = 7;
    int openport_cnts = 7;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    n_tx = pack_ch9437_registers(s, txbuf);
    n_rx = reg_cnts * openport_cnts;

    mutex_lock(&s->mutex_bus_access);
    ret = ch943x_ctrl_tty_write(s, n_tx, txbuf);
    if (ret < 0) {
        dev_err(s->dev, "%s - control uart write error. ret:%d\n", __func__, ret);
        mutex_unlock(&s->mutex_bus_access);
        return ret;
    }
    ret = ch943x_ctrl_tty_read(s, n_rx, rxbuf);
    if (ret < 0) {
        dev_err(s->dev, "%s - control uart read error. ret:%d\n", __func__, ret);
        mutex_unlock(&s->mutex_bus_access);
        return ret;
    }
    mutex_unlock(&s->mutex_bus_access);

    memcpy(buf, rxbuf, n_rx);
    DRV_DEBUG_HEXDUMP("ch9437_reg_bulkread_serialmode tx: ", DUMP_PREFIX_NONE, 32, 1, txbuf, n_tx, false);
    DRV_DEBUG_HEXDUMP("ch9437_reg_bulkread_serialmode rx: ", DUMP_PREFIX_NONE, 32, 1, rxbuf, n_rx, false);

    return n_rx;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
int ch943x_ctrluart_setopt(struct ch943x *s)
{
    struct termios uart_io;
    struct termios2 tio;
    mm_segment_t fs;
    int ret;
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

    ret = fp->f_op->unlocked_ioctl(fp, TCSETS, (unsigned long)&uart_io);
    ret = fp->f_op->unlocked_ioctl(fp, TCFLSH, 0);

    ret = fp->f_op->unlocked_ioctl(fp, TCGETS2, (unsigned long)&tio);
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
#ifdef MULTI_CHIP_MODE
    tio.c_ispeed = s->ctrluart_baud;
    tio.c_ospeed = s->ctrluart_baud;
#else
    tio.c_ispeed = CTRLUART_BAUD;
    tio.c_ospeed = CTRLUART_BAUD;
#endif

    ret = fp->f_op->unlocked_ioctl(fp, TCSETS2, (unsigned long)&tio);
    ret = fp->f_op->unlocked_ioctl(fp, TCGETS2, (unsigned long)&tio);

    set_fs(fs);

    return 0;
}
#endif

int ch943x_ioctl_serialmode_fifo_read(struct ch943x *s, u8 cmd, u32 n_rx, void *rxbuf)
{
    u8 *buffer;
    u8 tx_cmd = cmd | CH943X_REG_OP_READ;
    int retval;

    buffer = kmalloc(2048, GFP_KERNEL);
    if (!buffer)
        return -ENOMEM;

    retval = ch9437_serialmode_fifo_read(s, tx_cmd, n_rx, buffer);
    if (retval < 0) {
        dev_err(s->dev, "%s ch943x ioctl read failed.\n", __func__);
        goto out;
    }

    retval = copy_to_user((char __user *)rxbuf, buffer, n_rx);
    if (retval) {
        dev_err(s->dev, "%s copy_to_user error. retval:%d\n", __func__, retval);
        retval = -EFAULT;
    }
out:
    kfree(buffer);
    return retval;
}

int ch943x_ioctl_serialmode_fifo_write(struct ch943x *s, u8 cmd, u32 n_tx, void *txbuf)
{
    u8 *buffer;
    u8 tx_cmd = cmd | CH943X_REG_OP_WRITE;
    int retval;

    buffer = kmalloc(2048, GFP_KERNEL);
    if (!buffer)
        return -ENOMEM;

    retval = copy_from_user(buffer, (char __user *)txbuf, n_tx);
    if (retval) {
        dev_err(s->dev, "%s copy_from_user error. retval:%d\n", __func__, retval);
        goto out;
    }

    retval = ch9437_serialmode_fifo_write(s, tx_cmd, n_tx, buffer);
    if (retval < 0) {
        dev_err(s->dev, "%s ch943x ioctl read failed.\n", __func__);
        goto out;
    }
out:
    kfree(buffer);
    return retval;
}
#endif  // USE_SERIAL_MODE

static int ch943x_transfer_write(struct ch943x *s, u8 _cmd, u32 n_tx, const void *txbuf)
{
    u8 cmd = _cmd;
    int retval;
#ifdef USE_SPI_MODE
    struct spi_message m;
    struct spi_transfer xfer[3] = {};
#elif defined(USE_I2C_MODE)
    struct i2c_client *i2c = s->i2c;
    struct i2c_msg xfer[2] = {};
#elif defined(USE_SERIAL_MODE)
#endif

#ifdef USE_SPI_MODE
    xfer[0].tx_buf = &cmd;
    xfer[0].len = 1;
    xfer[0].cs_change = 0;
    spi_delay_set(&xfer[0], CH943X_CMD_DELAY);

    xfer[1].tx_buf = txbuf;
    xfer[1].len = n_tx;
    xfer[1].cs_change = 0;
    spi_delay_set(&xfer[1], CH943X_CMD_DELAY);

    mutex_lock(&s->mutex_bus_access);
    spi_message_init(&m);
    spi_message_add_tail(&xfer[0], &m);
    spi_message_add_tail(&xfer[1], &m);
    retval = spi_sync(s->spi_dev, &m);
    mutex_unlock(&s->mutex_bus_access);
    if (retval < 0) {
        dev_err(s->dev, "%s spi transfer failed\n", __func__);
        return retval;
    }
#elif defined(USE_I2C_MODE)
    mutex_lock(&s->mutex_bus_access);
    s->local_buf[0] = cmd;
    memcpy(s->local_buf + 1, txbuf, n_tx);

    xfer[0].addr = i2c->addr;
    xfer[0].flags = 0;
    xfer[0].len = n_tx + 1;
    xfer[0].buf = s->local_buf;

    retval = i2c_transfer(i2c->adapter, xfer, 1);
    if (retval != 1) {
        mutex_unlock(&s->mutex_bus_access);
        dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
        return -EIO;
    }
    mutex_unlock(&s->mutex_bus_access);
#elif defined(USE_SERIAL_MODE)
    mutex_lock(&s->mutex_bus_access);
    s->local_buf[0] = 0x57;
    s->local_buf[1] = 0xab;
    s->local_buf[2] = cmd;
    memcpy(s->local_buf + 3, txbuf, n_tx);

    retval = ch943x_ctrl_tty_write(s, n_tx + 3, s->local_buf);
    if (retval < 0) {
        dev_err(s->dev, "%s control uart write error. retval:%d\n", __func__, retval);
        mutex_unlock(&s->mutex_bus_access);
        return retval;
    }
    mutex_unlock(&s->mutex_bus_access);
#endif
    return 0;
}

static int ch943x_transfer_read(struct ch943x *s, u8 _cmd, u32 n_rx, void *rxbuf)
{
    u8 cmd = _cmd;
    int retval;
#ifdef USE_SPI_MODE
    struct spi_message m;
    struct spi_transfer xfer[3] = {};
    int cmd_delay_us = 0;
#elif defined(USE_I2C_MODE)
    struct i2c_client *i2c = s->i2c;
    struct i2c_msg xfer[2] = {};
#elif defined(USE_SERIAL_MODE)
    u8 txbuf[3] = {0x57, 0xab, cmd};
#endif

#ifdef USE_SPI_MODE
    if (cmd == CH943X_PORT_CMD_BULKMODE)
        cmd_delay_us = CH943X_CMD_DELAY * 3;
    else
        cmd_delay_us = CH943X_CMD_DELAY;

    xfer[0].tx_buf = &cmd;
    xfer[0].len = 1;
    xfer[0].cs_change = 0;
    spi_delay_set(&xfer[0], cmd_delay_us);

    xfer[1].rx_buf = rxbuf;
    xfer[1].len = n_rx;
    xfer[1].cs_change = 0;
    spi_delay_set(&xfer[1], cmd_delay_us);

    mutex_lock(&s->mutex_bus_access);
    spi_message_init(&m);
    spi_message_add_tail(&xfer[0], &m);
    spi_message_add_tail(&xfer[1], &m);
    retval = spi_sync(s->spi_dev, &m);
    mutex_unlock(&s->mutex_bus_access);
    if (retval < 0) {
        dev_err(s->dev, "%s spi transfer failed\n", __func__);
        return retval;
    }
#elif defined(USE_I2C_MODE)
    mutex_lock(&s->mutex_bus_access);
    xfer[0].addr = i2c->addr;
    xfer[0].flags = 0;
    xfer[0].len = 1;
    xfer[0].buf = &cmd;

    xfer[1].addr = i2c->addr;
    xfer[1].flags = I2C_M_RD;
    xfer[1].len = n_rx;
    xfer[1].buf = rxbuf;

    retval = i2c_transfer(i2c->adapter, xfer, 2);
    if (retval != 2) {
        mutex_unlock(&s->mutex_bus_access);
        dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
        return -EIO;
    }
    mutex_unlock(&s->mutex_bus_access);
#elif defined(USE_SERIAL_MODE)
    mutex_lock(&s->mutex_bus_access);
    retval = ch943x_ctrl_tty_write(s, 3, txbuf);
    if (retval < 0) {
        dev_err(s->dev, "%s control uart write error. retval:%d\n", __func__, retval);
        mutex_unlock(&s->mutex_bus_access);
        return retval;
    }
    retval = ch943x_ctrl_tty_read(s, n_rx, rxbuf);
    if (retval < 0) {
        dev_err(s->dev, "%s control uart error. retval:%d\n", __func__, retval);
        mutex_unlock(&s->mutex_bus_access);
        return retval;
    }
    mutex_unlock(&s->mutex_bus_access);
#endif
    return 0;
}

int ch943x_reg_write(struct ch943x *s, u8 _cmd, u32 n_tx, const void *txbuf)
{
    int ret;

    ret = ch943x_transfer_write(s, _cmd, n_tx, txbuf);
    if (n_tx == 1)
        DRV_DEBUG(s->dev, "%s - cmd:%02X, val:%02X\n", __func__, _cmd, *((u8 *)txbuf));

    return ret;
}

int ch943x_reg_read(struct ch943x *s, u8 _cmd, u32 n_rx, void *rxbuf)
{
    int ret;

    ret = ch943x_transfer_read(s, _cmd, n_rx, rxbuf);
    if (n_rx == 1)
        DRV_DEBUG(s->dev, "%s - cmd:%02X, val:%02X\n", __func__, _cmd, *((u8 *)rxbuf));

    return ret;
}

#ifdef CH9434D_CAN_ON
#ifndef CH943X_CANREG_NOTIMEINTER
static int ch943x_transfer_read_can(struct ch943x *s, u8 _cmd, u8 _reg, u32 n_rx, void *rxbuf)
{
#ifdef USE_SPI_MODE
    u8 cmd = _cmd;
    u8 reg = _reg;
    ssize_t retval;
    struct spi_message m;
    struct spi_transfer xfer[3] = {};
    int cmd_delay0, cmd_delay1, cmd_delay2;
#elif defined(USE_I2C_MODE)
    u8 cmd = _cmd;
    u8 reg = _reg;
    struct i2c_client *i2c = s->i2c;
    struct i2c_msg xfer[3] = {};
    u8 data[2] = {cmd, reg};
#endif

#ifdef USE_SPI_MODE
    cmd_delay0 = 2;
    cmd_delay1 = (n_rx > 4) ? 6 : 3;
    cmd_delay2 = 3;

    xfer[0].tx_buf = &cmd;
    xfer[0].len = 1;
    xfer[0].cs_change = 0;
    spi_delay_set(&xfer[0], cmd_delay0);

    xfer[1].tx_buf = &reg;
    xfer[1].len = 1;
    xfer[1].cs_change = 0;
    spi_delay_set(&xfer[1], cmd_delay1);

    xfer[2].rx_buf = rxbuf;
    xfer[2].len = n_rx;
    xfer[2].cs_change = 0;
    spi_delay_set(&xfer[2], cmd_delay2);

    mutex_lock(&s->mutex_bus_access);
    spi_message_init(&m);
    spi_message_add_tail(&xfer[0], &m);
    spi_message_add_tail(&xfer[1], &m);
    spi_message_add_tail(&xfer[2], &m);
    retval = spi_sync(s->spi_dev, &m);
    mutex_unlock(&s->mutex_bus_access);
    if (retval < 0) {
        dev_err(s->dev, "%s spi transfer failed\n", __func__);
        return retval;
    }
#elif defined(USE_I2C_MODE)
    mutex_lock(&s->mutex_bus_access);
    xfer[0].addr = i2c->addr;
    xfer[0].flags = 0;
    xfer[0].len = 2;
    xfer[0].buf = data;

    xfer[1].addr = i2c->addr;
    xfer[1].flags = I2C_M_RD;
    xfer[1].len = n_rx;
    xfer[1].buf = rxbuf;

    if (i2c_transfer(i2c->adapter, xfer, 2) != 2) {
        mutex_unlock(&s->mutex_bus_access);
        dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
        return -EIO;
    }
    mutex_unlock(&s->mutex_bus_access);
#endif
    return 0;
}
#endif

u32 ch943x_canreg_read(struct ch943x *s, u8 reg)
{
    u32 reg_val = 0;
    int retval;
#ifdef CH943X_CANREG_NOTIMEINTER
    u8 cmd = CH943X_REG_OP_READ | CH943X_CANREG_CMD_NOTIMEINTER;
    struct spi_message m;
    struct spi_transfer xfer[2] = {};
    uint8_t txbuf[256] = {0};
    uint8_t rxbuf[256] = {0};
    int index = 0, txlen = 0;
    int cmd_delay1_nr, cmd_delay2_nr;
    int i = 0;
    int reg_index = 0;
    int transfer_times = 0, retry_times = 0;
    bool trans_state = false;
#else
    u8 cmd = CH943X_REG_OP_READ | CH943X_CANREG_CMD;
    u8 data[4] = {0};
#endif

#ifdef CH943X_CANREG_NOTIMEINTER
    cmd_delay1_nr = calculate_transfer_size(s->spi_dev, SPI_W_CAN_DELAY1_US);
    cmd_delay2_nr = calculate_transfer_size(s->spi_dev, 4);

    for (;;) {
        index = 0;
        memset(txbuf, 0x00, sizeof(txbuf));
        memset(rxbuf, 0x00, sizeof(rxbuf));

        txbuf[index++] = cmd;
        memset(txbuf + index, SPI_R_CAN_ADD_SYNC_CODE, cmd_delay1_nr);
        index += cmd_delay1_nr;
        txbuf[index++] = SPI_R_CAN_ADD_INCP_CODE;
        reg_index = index;
        txbuf[index++] = reg;
        memset(txbuf + index, SPI_R_CAN_DATA_SYNC_CODE, cmd_delay2_nr);
        index += cmd_delay2_nr;
        memset(txbuf + index, SPI_R_CAN_DATA_SYNC_CODE, 4);
        index += 4;
        txbuf[index++] = SPI_R_CAN_DATA_INCP_CODE;

        txlen = index;

        DRV_DEBUG_HEXDUMP("tx: ", DUMP_PREFIX_NONE, 32, 1, (u8 *)txbuf, txlen, false);

        xfer[0].tx_buf = txbuf;
        xfer[0].rx_buf = rxbuf;
        xfer[0].len = txlen;
        xfer[0].cs_change = 0;

        mutex_lock(&s->mutex_bus_access);
        spi_message_init(&m);
        spi_message_add_tail(&xfer[0], &m);
        retval = spi_sync(s->spi_dev, &m);
        mutex_unlock(&s->mutex_bus_access);
        if (retval < 0) {
            dev_err(s->dev, "%s spi transfer failed\n", __func__);
            return retval;
        }
        DRV_DEBUG_HEXDUMP("rx: ", DUMP_PREFIX_NONE, 32, 1, (u8 *)rxbuf, txlen, false);

        for (i = reg_index + 1; i < reg_index + 1 + cmd_delay2_nr; i++) {
            if (rxbuf[i] == R_RSP_CAN_DATA_INCP_CODE) {
                reg_val |= rxbuf[i + 1];
                reg_val |= rxbuf[i + 2] << 8;
                reg_val |= rxbuf[i + 3] << 16;
                reg_val |= rxbuf[i + 4] << 24;
                DRV_DEBUG(s->dev, "%s - reg:%02x data:%08x\n", __func__, reg, reg_val);
                trans_state = true;
                break;
            }
        }

        if (trans_state) {
            if (retry_times > 0)
                dev_err(s->dev, "%s read CAN reg retry_times:%d\n", __func__, retry_times);
            break;
        } else {
            transfer_times++;
            retry_times++;
            if (transfer_times > 3) {
                dev_err(s->dev, "%s read CAN reg failed\n", __func__);
                return -1;
            }
        }
    }
#else
    retval = ch943x_transfer_read_can(s, cmd, reg, 4, data);
    if (retval < 0)
        return retval;
    reg_val |= data[0];
    reg_val |= data[1] << 8;
    reg_val |= data[2] << 16;
    reg_val |= data[3] << 24;
    DRV_DEBUG(s->dev, "%s - reg:%02x data:%08x\n", __func__, reg, reg_val);
#endif

    return reg_val;
}

int ch943x_canreg_write(struct ch943x *s, u8 reg, u32 val)
{
    int retval;
#ifdef CH943X_CANREG_NOTIMEINTER
    u8 cmd = CH943X_REG_OP_WRITE | CH943X_CANREG_CMD_NOTIMEINTER;
    struct spi_message m;
    struct spi_transfer xfer[2] = {};
    uint8_t txbuf[64] = {0};
    uint8_t rxbuf[64] = {0};
    int index = 0;
    int len;
    int cmd_delay1_nr, cmd_delay2_nr;
#else
    u8 cmd = CH943X_REG_OP_WRITE | CH943X_CANREG_CMD;
    u8 data[5] = {0};
#endif

    DRV_DEBUG(s->dev, "%s - reg:%02x data:%08x\n", __func__, reg, val);

#ifdef CH943X_CANREG_NOTIMEINTER
    cmd_delay1_nr = calculate_transfer_size(s->spi_dev, SPI_W_CAN_DELAY1_US);
    cmd_delay2_nr = calculate_transfer_size(s->spi_dev, 3);

    txbuf[index++] = cmd;
    memset(txbuf + index, SPI_W_CAN_ADD_SYNC_CODE, cmd_delay1_nr);
    index += cmd_delay1_nr;
    txbuf[index++] = SPI_W_CAN_ADD_INCP_CODE;
    txbuf[index++] = reg;
    txbuf[index++] = val & 0xff;
    txbuf[index++] = (val >> 8) & 0xff;
    txbuf[index++] = (val >> 16) & 0xff;
    txbuf[index++] = (val >> 24) & 0xff;
    memset(txbuf + index, SPI_W_CAN_DEAL_SYNC_CODE, cmd_delay2_nr);
    index += cmd_delay2_nr;
    txbuf[index++] = SPI_W_CAN_DEAL_INCP_CODE;
    len = index;

    DRV_DEBUG_HEXDUMP("tx: ", DUMP_PREFIX_NONE, 32, 1, (u8 *)txbuf, len, false);

    xfer[0].tx_buf = txbuf;
    xfer[0].rx_buf = rxbuf;
    xfer[0].len = len;
    xfer[0].cs_change = 0;

    mutex_lock(&s->mutex_bus_access);
    spi_message_init(&m);
    spi_message_add_tail(&xfer[0], &m);
    retval = spi_sync(s->spi_dev, &m);
    mutex_unlock(&s->mutex_bus_access);
    if (retval < 0) {
        dev_err(s->dev, "%s spi transfer failed\n", __func__);
        return retval;
    }
    DRV_DEBUG_HEXDUMP("rx: ", DUMP_PREFIX_NONE, 32, 1, (u8 *)rxbuf, len, false);
#else
    data[0] = reg;
    data[1] = val & 0xff;
    data[2] = (val >> 8) & 0xff;
    data[3] = (val >> 16) & 0xff;
    data[4] = (val >> 24) & 0xff;
    retval = ch943x_transfer_write(s, cmd, 5, data);
    if (retval < 0) {
        dev_err(s->dev, "%s ch943x canreg write failed.\n", __func__);
        return retval;
    }
#endif

    return 0;
}

int ch943x_rxmailbox_read(struct ch943x *s, u8 reg, u8 *rxbuf)
{
#ifdef CH943X_CANREG_NOTIMEINTER
    u8 cmd = CH943X_REG_OP_READ | CH943X_CANREG_CMD_NOTIMEINTER;
    int retval;
    struct spi_message m;
    struct spi_transfer xfer[2] = {};
    uint8_t txbuf[256] = {0};
    uint8_t rxbuf_tmp[256] = {0};
    int index = 0;
    int txlen;
    int reg_index;
    int i;
    bool trans_state = false;
    int cmd_delay1_nr, cmd_delay2_nr;
    int transfer_times = 0, retry_times = 0;
#else
    u8 cmd = CH943X_REG_OP_READ | CH943X_CANREG_CMD;
    int retval;
#endif

    DRV_DEBUG(s->dev, "%s\n", __func__);

#ifdef CH943X_CANREG_NOTIMEINTER
    cmd_delay1_nr = calculate_transfer_size(s->spi_dev, SPI_W_CAN_DELAY1_US);
    cmd_delay2_nr = calculate_transfer_size(s->spi_dev, 6);

    for (;;) {
        index = 0;
        memset(txbuf, 0x00, sizeof(txbuf));
        memset(rxbuf_tmp, 0x00, sizeof(rxbuf_tmp));

        txbuf[index++] = cmd;
        memset(txbuf + index, SPI_R_CAN_ADD_SYNC_CODE, cmd_delay1_nr);
        index += cmd_delay1_nr;
        txbuf[index++] = SPI_R_CAN_ADD_INCP_CODE;
        reg_index = index;
        txbuf[index++] = reg;
        memset(txbuf + index, SPI_R_CAN_DATA_SYNC_CODE, cmd_delay2_nr);
        index += cmd_delay2_nr;
        memset(txbuf + index, SPI_R_CAN_DATA_SYNC_CODE, 64);
        index += 64;
        txbuf[index++] = SPI_R_CAN_DATA_INCP_CODE;

        txlen = index;

        xfer[0].tx_buf = txbuf;
        xfer[0].rx_buf = rxbuf_tmp;
        xfer[0].len = txlen;
        xfer[0].cs_change = 0;

        mutex_lock(&s->mutex_bus_access);
        spi_message_init(&m);
        spi_message_add_tail(&xfer[0], &m);
        retval = spi_sync(s->spi_dev, &m);
        mutex_unlock(&s->mutex_bus_access);
        if (retval < 0) {
            dev_err(s->dev, "%s spi transfer failed\n", __func__);
            return retval;
        }

        DRV_DEBUG_HEXDUMP("tx: ", DUMP_PREFIX_NONE, 32, 1, (u8 *)txbuf, txlen, false);
        DRV_DEBUG_HEXDUMP("rx: ", DUMP_PREFIX_NONE, 32, 1, (u8 *)rxbuf_tmp, txlen, false);

        for (i = reg_index + 1; i < reg_index + 1 + cmd_delay2_nr; i++) {
            if (rxbuf_tmp[i] == R_RSP_CAN_DATA_INCP_CODE) {
                memcpy(rxbuf, rxbuf_tmp + i + 1, 64);
                trans_state = true;
                break;
            }
        }

        if (trans_state) {
            if (retry_times > 0)
                dev_err(s->dev, "%s read CAN reg retry_times:%d\n", __func__, retry_times);
            break;
        } else {
            transfer_times++;
            retry_times++;
            if (transfer_times > 3) {
                dev_err(s->dev, "%s read CAN reg failed\n", __func__);
                return -1;
            }
        }
    }
#else
    retval = ch943x_transfer_read_can(s, cmd, reg, 64, rxbuf);
    if (retval < 0) {
        return retval;
    }
    DRV_DEBUG_HEXDUMP("cont_read canreg: ", DUMP_PREFIX_NONE, 32, 1, rxbuf, 64, false);
#endif
    return 0;
}

int ch943x_txmailbox_write(struct ch943x *s, u8 reg, u32 n_tx, const void *txbuf)
{
#ifdef CH943X_CANREG_NOTIMEINTER
    u8 cmd = CH943X_REG_OP_WRITE | CH943X_CANREG_CMD_NOTIMEINTER;
    int retval;
    struct spi_message m;
    struct spi_transfer xfer[2] = {};
    uint8_t txbuf_tmp[256] = {0};
    uint8_t rxbuf[256] = {0};
    int index = 0;
    int txlen;
    int cmd_delay1_nr, cmd_delay2_nr;
#else
    u8 cmd = CH943X_REG_OP_WRITE | CH943X_CANREG_CMD;

#ifdef USE_SPI_MODE
    ssize_t retval;
    struct spi_message m;
    struct spi_transfer xfer[3] = {};
    int cmd_delay_us;
#elif defined(USE_I2C_MODE)
    u8 buf[64] = {0};
    struct i2c_client *i2c = s->i2c;
    struct i2c_msg xfer[2] = {};
#endif
#endif

    DRV_DEBUG(s->dev, "%s\n", __func__);

#ifdef CH943X_CANREG_NOTIMEINTER
    cmd_delay1_nr = calculate_transfer_size(s->spi_dev, SPI_W_CAN_DELAY1_US);
    cmd_delay2_nr = calculate_transfer_size(s->spi_dev, 3);

    txbuf_tmp[index++] = cmd;
    memset(txbuf_tmp + index, SPI_W_CAN_ADD_SYNC_CODE, cmd_delay1_nr);
    index += cmd_delay1_nr;
    txbuf_tmp[index++] = SPI_W_CAN_ADD_INCP_CODE;
    txbuf_tmp[index++] = reg;
    memcpy(txbuf_tmp + index, (uint8_t *)txbuf, n_tx);
    index += n_tx;
    memset(txbuf_tmp + index, SPI_W_CAN_DEAL_SYNC_CODE, cmd_delay2_nr);
    index += cmd_delay2_nr;
    txbuf_tmp[index++] = SPI_W_CAN_DEAL_INCP_CODE;
    txlen = index;

    xfer[0].tx_buf = txbuf_tmp;
    xfer[0].rx_buf = rxbuf;
    xfer[0].len = txlen;
    xfer[0].cs_change = 0;

    mutex_lock(&s->mutex_bus_access);
    spi_message_init(&m);
    spi_message_add_tail(&xfer[0], &m);
    retval = spi_sync(s->spi_dev, &m);
    mutex_unlock(&s->mutex_bus_access);
    if (retval < 0) {
        dev_err(s->dev, "%s spi transfer failed\n", __func__);
        return retval;
    }
    DRV_DEBUG_HEXDUMP("tx: ", DUMP_PREFIX_NONE, 32, 1, (u8 *)txbuf_tmp, txlen, false);
    DRV_DEBUG_HEXDUMP("rx: ", DUMP_PREFIX_NONE, 32, 1, (u8 *)rxbuf, txlen, false);
#else
#ifdef USE_SPI_MODE
    cmd_delay_us = 5;

    xfer[0].tx_buf = &cmd;
    xfer[0].len = 1;
    xfer[0].cs_change = 0;
    spi_delay_set(&xfer[0], cmd_delay_us);

    xfer[1].tx_buf = &reg;
    xfer[1].len = 1;
    xfer[1].cs_change = 0;
    spi_delay_set(&xfer[1], cmd_delay_us);

    xfer[2].tx_buf = txbuf;
    xfer[2].len = n_tx;
    xfer[2].cs_change = 0;
    spi_delay_set(&xfer[2], cmd_delay_us);

    mutex_lock(&s->mutex_bus_access);
    spi_message_init(&m);
    spi_message_add_tail(&xfer[0], &m);
    spi_message_add_tail(&xfer[1], &m);
    spi_message_add_tail(&xfer[2], &m);
    retval = spi_sync(s->spi_dev, &m);
    mutex_unlock(&s->mutex_bus_access);
    if (retval < 0) {
        dev_err(s->dev, "%s spi transfer failed\n", __func__);
        return retval;
    }
#elif defined(USE_I2C_MODE)
    mutex_lock(&s->mutex_bus_access);
    buf[0] = cmd;
    buf[1] = reg;
    memcpy(buf + 2, txbuf, n_tx);

    xfer[0].addr = i2c->addr;
    xfer[0].flags = 0;
    xfer[0].len = n_tx + 2;
    xfer[0].buf = buf;

    if (i2c_transfer(i2c->adapter, xfer, 1) != 1) {
        mutex_unlock(&s->mutex_bus_access);
        dev_err(&i2c->dev, "%s i2c transfer failed\n", __func__);
        return -EIO;
    }
#endif
    DRV_DEBUG_HEXDUMP("cont_write canreg: ", DUMP_PREFIX_NONE, 32, 1, (u8 *)txbuf, n_tx, false);
#endif
    return 0;
}
#endif

u8 ch943x_port_read(struct uart_port *port, u8 reg)
{
    struct ch943x *s = dev_get_drvdata(port->dev);
    u8 cmd = 0x00;
    u8 val = 0x00;
    int retval;

    if ((s->chip.chiptype == CHIP_CH9438) || (s->chip.chiptype == CHIP_CH9437))
        cmd = 0x00 | (port->line * 8 + reg);
    else
        cmd = (0x00 | reg) + (port->line * 0x10);

    retval = ch943x_transfer_read(s, cmd, 1, &val);
    if (retval < 0) {
        dev_err(s->dev, "%s ch943x uart reg read failed.\n", __func__);
        return retval;
    }
    DRV_DEBUG(s->dev, "%s - cmd:%02X[u%d reg:%02X], val:%02X\n", __func__, cmd, port->line, reg, val);

    return val;
}

int ch943x_port_write(struct uart_port *port, u8 reg, u8 val)
{
    struct ch943x *s = dev_get_drvdata(port->dev);
    u8 cmd;
    int retval;

    if ((s->chip.chiptype == CHIP_CH9438) || (s->chip.chiptype == CHIP_CH9437))
        cmd = 0x80 | (port->line * 8 + reg);
    else
        cmd = (0x80 | reg) + (port->line * 0x10);

    retval = ch943x_transfer_write(s, cmd, 1, &val);
    if (retval < 0) {
        dev_err(s->dev, "%s ch943x uart reg write failed.\n", __func__);
        return retval;
    }
    DRV_DEBUG(s->dev, "%s - cmd:%02X[u%d reg:%02X], val:%02X\n", __func__, cmd, port->line, reg, val);

    return 0;
}

int ch943x_port_bulkread(struct ch943x *s, u8 reg, u8 *buf, int len)
{
    u8 cmd = reg;
    int retval;

    retval = ch943x_reg_read(s, cmd, len, buf);
    if (retval < 0) {
        dev_err(s->dev, "%s ch9438 read uart regs failed.\n", __func__);
        return retval;
    }

    DRV_DEBUG(s->dev, "%s - cmd:%02x, len:%d\n", __func__, cmd, len);
    DRV_DEBUG_HEXDUMP("data: ", DUMP_PREFIX_NONE, 16, 1, buf, len, false);

    return 0;
}

int ch943x_port_update(struct uart_port *port, u8 reg, u8 mask, u8 val)
{
    unsigned int tmp;

    tmp = ch943x_port_read(port, reg);
    tmp &= ~mask;
    tmp |= val & mask;
    ch943x_port_write(port, reg, tmp);
    return 0;
}

int ch943x_iofunc_get(struct ch943x *s, uint8_t io_cmd, uint8_t io_addr)
{
    u8 cmd_buf[4] = {0};
    u8 rx_buf[4] = {0};
    u8 cmd_w = CH943X_IO_SEL_FUN_CFG | 0x80;
    u8 cmd_r = CH943X_IO_SEL_FUN_CFG;
    int ret = 0;
    int retval;
    unsigned long timeout_jiffies = jiffies + msecs_to_jiffies(3000);

    cmd_buf[0] = io_cmd;
    cmd_buf[1] = io_addr;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = CH9434X_IO_CMD_ACT;

    retval = ch943x_reg_write(s, cmd_w, 4, cmd_buf);
    if (retval < 0) {
        dev_err(s->dev, "%s ch943x iofunc write failed.\n", __func__);
        return retval;
    }
    udelay(1000);
    DRV_DEBUG(s->dev, "%s - cmd:%02x, data:%02x %02x %02x %02x\n", __func__, cmd_w, cmd_buf[0], cmd_buf[1], cmd_buf[2],
              cmd_buf[3]);

    while (1) {
        if (time_after(jiffies, timeout_jiffies)) {
            dev_err(s->dev, "%s timeout.\n", __func__);
            break;
        }

        retval = ch943x_reg_read(s, cmd_r, 4, rx_buf);
        if (retval < 0) {
            dev_err(s->dev, "%s ch943x iofunc read failed.\n", __func__);
            return retval;
        }
        DRV_DEBUG(s->dev, "%s - cmd:%02x, recv:%02x %02x %02x %02x\n", __func__, cmd_r, rx_buf[0], rx_buf[1], rx_buf[2],
                  rx_buf[3]);

        if ((rx_buf[3] == CH9434X_IO_CMD_COMP) && (rx_buf[0] == cmd_buf[0])) {
            ret = rx_buf[2];
            break;
        } else {
            mdelay(1);
            continue;
        }
    }

    return ret;
}

int ch943x_iofunc_set(struct ch943x *s, u8 io_cmd, u8 io_addr, u8 enable)
{
    u8 cmd_buf[4] = {0};
    u8 rx_buf[4] = {0};
    u8 cmd_w = CH943X_IO_SEL_FUN_CFG | 0x80;
    u8 cmd_r = CH943X_IO_SEL_FUN_CFG;
    int ret;
    int retval;
    unsigned long timeout_jiffies = jiffies + msecs_to_jiffies(3000);

    cmd_buf[0] = io_cmd;
    cmd_buf[1] = io_addr;
    cmd_buf[2] = enable;
    cmd_buf[3] = CH9434X_IO_CMD_ACT;

    retval = ch943x_reg_write(s, cmd_w, 4, cmd_buf);
    if (retval < 0) {
        dev_err(s->dev, "%s ch943x iofunc write failed.\n", __func__);
        return retval;
    }
    udelay(1000);
    DRV_DEBUG(s->dev, "%s - cmd:%02x, data:%02x %02x %02x %02x\n", __func__, cmd_w, cmd_buf[0], cmd_buf[1], cmd_buf[2],
              cmd_buf[3]);

    while (1) {
        if (time_after(jiffies, timeout_jiffies)) {
            dev_err(s->dev, "%s timeout.\n", __func__);
            break;
        }

        retval = ch943x_reg_read(s, cmd_r, 4, rx_buf);
        if (retval < 0) {
            dev_err(s->dev, "%s ch943x iofunc read failed.\n", __func__);
            return retval;
        }
        DRV_DEBUG(s->dev, "%s - cmd:%02x, recv:%02x %02x %02x %02x\n", __func__, cmd_r, rx_buf[0], rx_buf[1], rx_buf[2],
                  rx_buf[3]);

        if ((rx_buf[3] == CH9434X_IO_CMD_COMP) && (rx_buf[0] == cmd_buf[0]) && (rx_buf[1] == cmd_buf[1])) {
            ret = 0;
            break;
        } else {
            mdelay(1);
            continue;
        }
    }

    return ret;
}

int ch943x_port_read_version(struct ch943x *s, u8 reg, u8 *buf, u8 count)
{
    u8 cmd = reg;
    int retval;

    retval = ch943x_reg_read(s, cmd, 4, buf);
    if (retval < 0) {
        dev_err(s->dev, "%s ch943x read version failed.\n", __func__);
        return retval;
    }
    DRV_DEBUG(s->dev, "%s - cmd:%02x buf:%02x %02x %02x %02x\n", __func__, cmd, buf[0], buf[1], buf[2], buf[3]);

    return 0;
}

int ch943x_raw_write(struct uart_port *port, u8 cmd, u8 *buf, u32 len)
{
    struct ch943x *s = dev_get_drvdata(port->dev);
    int retval;
#ifdef USE_SERIAL_MODE
#elif defined(USE_SPI_MODE)
    int i;
    struct spi_message m;
    struct spi_transfer xfer[2] = {};
#endif

    if ((s->chip.chiptype == CHIP_CH9434D) || (s->chip.chiptype == CHIP_CH9432) || (s->chip.chiptype == CHIP_CH9438) ||
        ((s->chip.chiptype == CHIP_CH9437) && (s->chip.interface_mode == I2C_MODE))) {
        retval = ch943x_reg_write(s, cmd, len, buf);
        if (retval < 0) {
            dev_err(s->dev, "%s ch943x write txfifo failed.\n", __func__);
            return retval;
        }
        DRV_DEBUG(s->dev, "%s - cmd:%02x[u%d], len:%d\n", __func__, cmd, port->line, len);
        DRV_DEBUG_HEXDUMP("ch943x_raw_write tx: ", DUMP_PREFIX_NONE, 32, 1, buf, len, false);
    } else if ((s->chip.chiptype == CHIP_CH9437) && (s->chip.interface_mode == SERIAL_MODE)) {
#ifdef USE_SERIAL_MODE
        retval = ch9437_serialmode_fifo_write(s, cmd, len, buf);
        if (retval < 0) {
            dev_err(s->dev, "%s ch943x write txfifo failed.\n", __func__);
            return retval;
        }
#endif
    } else if ((s->chip.chiptype == CHIP_CH9434A) || (s->chip.chiptype == CHIP_CH9434M)) {
#ifdef USE_SPI_MODE
        if (s->spi_contmode) {
            s->txfifo_buf[0] = cmd;
            memcpy(s->txfifo_buf + 1, buf, len);

            xfer[0].tx_buf = s->txfifo_buf;
            xfer[0].len = len + 1;

            mutex_lock(&s->mutex_bus_access);
            spi_message_init(&m);
            spi_message_add_tail(&xfer[0], &m);
            retval = spi_sync(s->spi_dev, &m);
            mutex_unlock(&s->mutex_bus_access);
            if (retval < 0) {
                dev_err(s->dev, "%s spi transfer failed\n", __func__);
                return retval;
            }
            DRV_DEBUG_HEXDUMP("ch943x_raw_write tx: ", DUMP_PREFIX_NONE, 32, 1, buf, len, false);
        } else {
            for (i = 0; i < len; i++) {
                retval = ch943x_reg_write(s, cmd, 1, buf + i);
                if (retval < 0) {
                    dev_err(s->dev, "%s ch943x write txfifo failed.\n", __func__);
                    return retval;
                }
                DRV_DEBUG(s->dev, "%s - cmd:%02X, buf[%d]:%02X\n", __func__, cmd, i, buf[i]);
            }
        }
#endif
    }
    return 0;
}

int ch943x_raw_read(struct uart_port *port, u8 reg, u8 *buf, u32 len)
{
    struct ch943x *s = dev_get_drvdata(port->dev);
    u8 cmd;
    int retval;
#ifdef USE_SPI_MODE
    struct spi_message m;
    struct spi_transfer xfer[2] = {};
#endif

    DRV_DEBUG(s->dev, "%s\n", __func__);

    if ((s->chip.chiptype == CHIP_CH9438) || (s->chip.chiptype == CHIP_CH9437)) {
        cmd = 0x00 | (port->line * 8 + reg);
    } else {
        cmd = (0x00 | reg) + (port->line * 0x10);
    }

    if ((s->chip.chiptype == CHIP_CH9434D) || (s->chip.chiptype == CHIP_CH9432) || (s->chip.chiptype == CHIP_CH9438) ||
        ((s->chip.chiptype == CHIP_CH9437) && (s->chip.interface_mode == I2C_MODE))) {
        retval = ch943x_reg_read(s, cmd, len, s->rxfifo_buf);
        if (retval < 0) {
            dev_err(s->dev, "%s ch943x read rxfifo failed.\n", __func__);
            return retval;
        }
        DRV_DEBUG(s->dev, "%s - cmd:%02x[u%d], len:%d\n", __func__, cmd, port->line, len);
        DRV_DEBUG_HEXDUMP("ch943x_raw_read rx: ", DUMP_PREFIX_NONE, 32, 1, s->rxfifo_buf, len, false);
        memcpy(buf, s->rxfifo_buf, len);
    } else if (s->chip.chiptype == CHIP_CH9434A) {
#ifdef USE_SPI_MODE
        s->rxfifo_buf[0] = cmd;

        xfer[0].tx_buf = s->rxfifo_buf;
        xfer[0].rx_buf = s->rxfifo_buf;
        xfer[0].len = len + 2;

        mutex_lock(&s->mutex_bus_access);
        spi_message_init(&m);
        spi_message_add_tail(&xfer[0], &m);
        retval = spi_sync(s->spi_dev, &m);
        mutex_unlock(&s->mutex_bus_access);
        if (retval < 0) {
            dev_err(s->dev, "%s spi transfer failed\n", __func__);
            return retval;
        }
        memcpy(buf, s->rxfifo_buf + 2, len);
#endif
    } else if ((s->chip.chiptype == CHIP_CH9437) && (s->chip.interface_mode == SERIAL_MODE)) {
#ifdef USE_SERIAL_MODE
        retval = ch9437_serialmode_fifo_read(s, cmd, len, s->rxfifo_buf);
        if (retval < 0) {
            dev_err(s->dev, "%s ch943x read rxfifo failed.\n", __func__);
            return retval;
        }
        memcpy(buf, s->rxfifo_buf, len);
#endif
    }

    return 0;
}

int ch943x_get_chip_version(struct ch943x *s)
{
#ifdef USE_SPI_MODE
    u8 spi_contmode_reg;
#endif
    int ret;

    /* get chip type and version */
    ret = ch943x_port_read_version(s, CH943X_CHIP_VER_REG, s->chip.ver, VER_LEN);
    if (ret) {
        dev_err(s->dev, "Get CH943X chip version failed\n");
        return -1;
    }

    if ((s->chip.ver[3] == 0x5A) && (s->chip.ver[2] == (s->chip.ver[0] + s->chip.ver[1]))) {
#ifdef USE_SPI_MODE
        s->chip.chiptype = CHIP_CH9434A;
        strcpy(s->chip.chip_name, "CH9434A");
        ret = ch943x_reg_read(s, CH943X_SPI_CONT_MODE_REG, 1, &spi_contmode_reg);
        spi_contmode_reg |= CH943X_SPI_CONTE_BIT;
        ch943x_reg_write(s, CH943X_SPI_CONT_MODE_REG | CH943X_REG_OP_WRITE, 1, &spi_contmode_reg);
        if (s->spi_dev->max_speed_hz > 2000000) {
            dev_err(s->dev, "CHIP:%s SPI cont-mode, CLK must not exceed 2MHz.\n", s->chip.chip_name);
            s->spi_dev->max_speed_hz = 2000000;
        }
        s->spi_contmode = true;
#endif
    } else if ((s->chip.ver[3] == 0x6B) && (s->chip.ver[2] == (s->chip.ver[0] + s->chip.ver[1]))) {
        s->chip.chiptype = CHIP_CH9434D;
        strcpy(s->chip.chip_name, "CH9434D");
    } else if ((s->chip.ver[3] == 0x7C) && (s->chip.ver[2] == (s->chip.ver[0] + s->chip.ver[1]))) {
        s->chip.chiptype = CHIP_CH9438;
        strcpy(s->chip.chip_name, "CH9438");
    } else if ((s->chip.ver[3] == 0x8D) && (s->chip.ver[2] == (s->chip.ver[0] + s->chip.ver[1]))) {
        s->chip.chiptype = CHIP_CH9437;
        strcpy(s->chip.chip_name, "CH9437");
    } else if ((s->chip.ver[3] == 0x9E) && (s->chip.ver[2] == (s->chip.ver[0] + s->chip.ver[1]))) {
        s->chip.chiptype = CHIP_CH9432;
        strcpy(s->chip.chip_name, "CH9432");
    } else {
        s->chip.chiptype = CHIP_CH9434M;
#ifdef USE_SPI_MODE
        s->chip.ver[1] = 1;
        s->chip.ver[0] = 0;
        s->spi_contmode = false;
        strcpy(s->chip.chip_name, "CH9434M");
#endif
    }
    dev_info(s->dev, "CHIP TYPE:%s - V%d.%d\n", s->chip.chip_name, s->chip.ver[1], s->chip.ver[0]);

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
     * Situation 1. A single SPI host connects multiple SPI slaves, using software CS.
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
     *
     * Situation 4. ...
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
     *
     * Situation 2. ...
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
    if (CH9434D_CAN_ENABLE) {
        s->can_on = true;
    }
#endif

    /*
     * Only after adding the above code can the following
     * ch943x_iofunc_set function be executed!
     */
    DRV_DEBUG(s->dev, "%s nr_uart:%d\n", __func__, s->chip.nr_uart);

    if ((s->chip.chiptype == CHIP_CH9434A) || (s->chip.chiptype == CHIP_CH9434M))
        return 0;

    DRV_DEBUG(s->dev, "%s chiptype:%d INT pin enable.\n", __func__, s->chip.chiptype);
    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_INT_ADD, 1); /* Interrupt Enable */

    DRV_DEBUG(s->dev, "%s chiptype:%d TX/RX pin enable.\n", __func__, s->chip.chiptype);

    for (i = 0; i < s->chip.nr_uart; i++) {
        if ((s->chip.chiptype == CHIP_CH9437) && IS_USE_SERIAL_MODE && (i == 0))
            continue;
        ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH943X_DEF_U0_ADD + i, 1); /* UART Tx/Rx Enable */
    }

    if (s->chip.chiptype == CHIP_CH9434D) {
        DRV_DEBUG(s->dev, "%s chiptype:%d XI/XO pin enable.\n", __func__, s->chip.chiptype);
        if (s->extern_clock_on)
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9434D_DEF_HSE_ADD, 1); /* External Clock Enable */

        DRV_DEBUG(s->dev, "%s chiptype:%d CAN pin enable.\n", __func__, s->chip.chiptype);
        if (s->can_on)
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9434D_DEF_CAN_ADD, 1); /* CAN Tx/Rx Enable */

        DRV_DEBUG(s->dev, "%s chiptype:%d TNOW pin enable.\n", __func__, s->chip.chiptype);
        if ((s->tnow_enable_bits & BIT(0)) && (!s->can_on))
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW0_ADD, 1); /* TNOW0 Enable */
        if ((s->tnow_enable_bits & BIT(1)) && (!s->can_on))
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW1_ADD, 1); /* TNOW1 Enable */
        if (s->tnow_enable_bits & BIT(2))
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW2_ADD, 1); /* TNOW2 Enable */
        if ((s->tnow_enable_bits & BIT(3)) && (!s->extern_clock_on))
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW3_ADD, 1); /* TNOW3 Enable */

        DRV_DEBUG(s->dev, "%s chiptype:%d Modem pin enable.\n", __func__, s->chip.chiptype);
        if (((s->tnow_enable_bits & BIT(0)) == 0) && (!s->can_on))
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9434D_DEF_CTS0_ADD, 1); /* Modem CTS0 Enable */
        if ((s->tnow_enable_bits & BIT(2)) == 0)
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9434D_DEF_CTS3_ADD, 1); /* Modem CTS3 Enable */
        if (((s->tnow_enable_bits & BIT(1)) == 0) && (!s->can_on))
            ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9434D_MUL_RTS0_ADD, 1); /* Modem RTS0 Enable */
    } else if (s->chip.chiptype == CHIP_CH9438) {
        DRV_DEBUG(s->dev, "%s chiptype:%d XI/XO pin enable.\n", __func__, s->chip.chiptype);
        if (s->extern_clock_on)
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH943X_DEF_HSE_ADD, 1); /* External Clock Enable */

        DRV_DEBUG(s->dev, "%s chiptype:%d TNOW pin enable.\n", __func__, s->chip.chiptype);
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

        DRV_DEBUG(s->dev, "%s chiptype:%d Modem pin enable.\n", __func__, s->chip.chiptype);
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
    } else if (s->chip.chiptype == CHIP_CH9437) {
        DRV_DEBUG(s->dev, "%s chiptype:%d XI/XO pin enable.\n", __func__, s->chip.chiptype);
        if (s->extern_clock_on)
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH943X_DEF_HSE_ADD, 1); /* External Clock Enable */

        DRV_DEBUG(s->dev, "%s chiptype:%d TNOW pin enable.\n", __func__, s->chip.chiptype);
        for (i = 0; i < s->chip.nr_uart; i++) {
            if (IS_USE_SERIAL_MODE && (i == 0))
                continue;
            if (i == 7)
                continue;
            if (s->tnow_enable_bits & BIT(i))
                ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW0_ADD + i, 1); /* TNOWx Enable */
        }

        DRV_DEBUG(s->dev, "%s chiptype:%d Modem pin enable.\n", __func__, s->chip.chiptype);
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
    } else if (s->chip.chiptype == CHIP_CH9432) {
        DRV_DEBUG(s->dev, "%s chiptype:%d XI/XO pin enable.\n", __func__, s->chip.chiptype);
        if (s->extern_clock_on)
            ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9432_DEF_HSE_ADD, 1); /* External Clock Enable */

        DRV_DEBUG(s->dev, "%s chiptype:%d TNOW pin enable.\n", __func__, s->chip.chiptype);
        for (i = 0; i < s->chip.nr_uart; i++) {
            if (s->tnow_enable_bits & BIT(i))
                ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_TNOW0_ADD + i, 1); /* TNOWx Enable */
        }

        DRV_DEBUG(s->dev, "%s chiptype:%d Modem pin enable.\n", __func__, s->chip.chiptype);
        ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9432_MUL_RTS0_ADD, 1); /* RTS0 Enable */
        ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9432_MUL_RTS1_ADD, 1); /* RTS1 Enable */
        ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9432_DEF_CTS0_ADD, 1);   /* CTS0 Enable */
        ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9432_DEF_CTS1_ADD, 1);   /* CTS1 Enable */
    }

    return 0;
}

static int ch943x_ioctl_write(struct ch943x *s, u8 cmd, u32 n_tx, void *txbuf)
{
    u8 *buffer;
    int retval = 0;

    buffer = kmalloc(2048, GFP_KERNEL);
    if (!buffer)
        return -ENOMEM;
    buffer[0] = cmd | CH943X_REG_OP_WRITE;

    retval = copy_from_user(buffer + 1, (char __user *)txbuf, n_tx);
    if (retval) {
        dev_err(s->dev, "%s copy_from_user error. retval:%d\n", __func__, retval);
        goto out;
    }

    retval = ch943x_reg_write(s, buffer[0], n_tx, buffer + 1);
    if (retval < 0) {
        dev_err(s->dev, "%s ch943x ioctl write failed.\n", __func__);
        goto out;
    }
out:
    kfree(buffer);
    return retval;
}

static int ch943x_ioctl_read(struct ch943x *s, u8 cmd, u32 n_rx, void *rxbuf)
{
    u8 *buffer;
    u8 tx_cmd = cmd | CH943X_REG_OP_READ;
    int retval;

    buffer = kmalloc(2048, GFP_KERNEL);
    if (!buffer)
        return -ENOMEM;

    retval = ch943x_reg_read(s, tx_cmd, n_rx, buffer);
    if (retval < 0) {
        dev_err(s->dev, "%s ch943x ioctl read failed.\n", __func__);
        goto out;
    }

    retval = copy_to_user((char __user *)rxbuf, buffer, n_rx);
    if (retval) {
        dev_err(s->dev, "%s copy_to_user error. retval:%d\n", __func__, retval);
        retval = -EFAULT;
    }
out:
    kfree(buffer);
    return retval;
}

int __ch943x_io_ioctl(struct ch943x *s, unsigned int cmd, unsigned long arg)
{
    u16 __user *argval = (u16 __user *)arg;
    u32 __user *argval1 = (u32 __user *)arg;
    u16 inarg;
    int rv, ret;
    u8 val, gpionumber, enable;
    unsigned long arg1, arg2, arg3;
    int i;

    switch (cmd) {
    case IOCTL_CMD_GETCHIPTYPE: {
        if (put_user(s->chip.chiptype, argval1)) {
            rv = -EFAULT;
            goto out;
        } else
            rv = 0;
        break;
    }
    case IOCTL_CMD_CH9434D_GPIOENABLE: {
        if (get_user(inarg, argval))
            return -EFAULT;
        gpionumber = (inarg >> 8) & 0xFF;
        enable = inarg & 0xFF;

        if ((gpionumber == 3) && CH943X_EXCLK_ENABLE) {
            dev_err(s->dev, "%s - CH9434D GPIO3 is unavailable.\n", __func__);
            return -1;
        } else if (((gpionumber == 0) || (gpionumber == 1)) && CH9434D_CAN_ENABLE) {
            dev_err(s->dev, "%s - CH9434D GPIO0/1 is unavailable.\n", __func__);
            return -1;
        }

        for (i = 0; i < s->chip.nr_uart; i++) {
            if ((i == gpionumber) && CH943X_TNOW_ENABLE(i)) {
                dev_err(s->dev, "%s - CH9434D GPIO%d is unavailable.\n", __func__, i);
                return -1;
            }
        }

        if (enable) {
            if (gpionumber == 0)
                ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9434D_DEF_CTS0_ADD, DISABLE);
            else if (gpionumber == 1)
                ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9434D_MUL_RTS0_ADD, DISABLE);
            else if (gpionumber == 2)
                ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9434D_DEF_CTS3_ADD, DISABLE);

            for (i = 0; i < s->chip.nr_gpio; i++) {
                if (i == gpionumber) {
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9434D_MUL_GPIO0_ADD + i, ENABLE);
                }
            }
        }

        ret = ch943x_reg_read(s, CH9434_GPIO_FUNC_EN_0 + (gpionumber / 8), 1, &val);
        if (enable)
            val |= BIT(gpionumber % 8);
        else
            val &= ~BIT(gpionumber % 8);
        rv = ch943x_reg_write(s, (CH9434_GPIO_FUNC_EN_0 + (gpionumber / 8)) | CH943X_REG_OP_WRITE, 1, &val);
        break;
    }
    case IOCTL_CMD_CH9438_GPIOENABLE: {
        if (get_user(inarg, argval))
            return -EFAULT;
        gpionumber = (inarg >> 8) & 0xFF;
        enable = inarg & 0xFF;

        if (gpionumber == 1) {
            dev_err(s->dev, "%s - CH9438 GPIO1 is unavailable.\n", __func__);
            return -1;
        } else if ((gpionumber == 7) && CH943X_EXCLK_ENABLE) {
            dev_err(s->dev, "%s - CH9438 GPIO7 is unavailable.\n", __func__);
            return -1;
        }

        for (i = 0; i < s->chip.nr_uart; i++) {
            if (i == gpionumber && CH943X_TNOW_ENABLE(i)) {
                dev_err(s->dev, "%s - CH9438 GPIO%d is unavailable.\n", __func__, i);
                return -1;
            }
        }

        if (enable) {
            switch (gpionumber) {
            case 0:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS1_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS1_ADD, DISABLE);
                break;
            case 2:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS2_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS2_ADD, DISABLE);
                break;
            case 3:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS2_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS2_ADD, DISABLE);
                break;
            case 4:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS3_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS3_ADD, DISABLE);
                break;
            case 5:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS3_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS3_ADD, DISABLE);
                break;
            case 6:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS4_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS4_ADD, DISABLE);
                break;
            case 7:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS4_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS4_ADD, DISABLE);
                break;
            default:
                break;
            }

            for (i = 0; i < s->chip.nr_gpio; i++) {
                if (i == gpionumber) {
                    if (!ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_GPIO0_ADD + i))
                        ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_GPIO0_ADD + i, ENABLE);
                }
            }
        }

        ret = ch943x_reg_read(s, CH9434_GPIO_FUNC_EN_0 + (gpionumber / 8), 1, &val);
        if (enable)
            val |= BIT(gpionumber % 8);
        else
            val &= ~BIT(gpionumber % 8);
        rv = ch943x_reg_write(s, (CH9434_GPIO_FUNC_EN_0 + (gpionumber / 8)) | CH943X_REG_OP_WRITE, 1, &val);
        break;
    }
    case IOCTL_CMD_CH9437_GPIOENABLE: {
        if (get_user(inarg, argval))
            return -EFAULT;
        gpionumber = (inarg >> 8) & 0xFF;
        enable = inarg & 0xFF;

        if (gpionumber == 0 && s->chip.interface_mode == I2C_MODE) {
            dev_err(s->dev, "%s - CH9437 GPIO%d is unavailable.\n", __func__, gpionumber);
            return -1;
        } else if (gpionumber == 8 && CH943X_EXCLK_ENABLE) {
            dev_err(s->dev, "%s - CH9437 GPIO%d is unavailable.\n", __func__, gpionumber);
            return -1;
        } else if (gpionumber == 9 && s->chip.interface_mode == I2C_MODE) {
            dev_err(s->dev, "%s - CH9437 GPIO%d is unavailable.\n", __func__, gpionumber);
            return -1;
        } else if (gpionumber == 10) {
            dev_err(s->dev, "%s - CH9437 GPIO%d is unavailable.\n", __func__, gpionumber);
            return -1;
        }

        for (i = 1; i < s->chip.nr_uart; i++) {
            if ((gpionumber == i) && CH943X_TNOW_ENABLE(i - 1)) {
                dev_err(s->dev, "%s - CH9437 GPIO%d is unavailable.\n", __func__, gpionumber);
                return -1;
            }
        }

        if (enable) {
            switch (gpionumber) {
            case 1:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS0_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS0_ADD, DISABLE);
                break;
            case 2:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS0_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS0_ADD, DISABLE);
                break;
            case 3:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS2_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS2_ADD, DISABLE);
                break;
            case 4:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS2_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS2_ADD, DISABLE);
                break;
            case 5:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS3_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS3_ADD, DISABLE);
                break;
            case 6:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS3_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS3_ADD, DISABLE);
                break;
            case 7:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS4_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS4_ADD, DISABLE);
                break;
            case 8:
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS4_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS4_ADD, DISABLE);
                break;
            default:
                break;
            }

            for (i = 0; i < s->chip.nr_gpio; i++) {
                if (i == gpionumber) {
                    if (!ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_GPIO0_ADD + i)) {
                        ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_GPIO0_ADD + i, ENABLE);
                    }
                }
            }
        }

        ret = ch943x_reg_read(s, CH9434_GPIO_FUNC_EN_0 + (gpionumber / 8), 1, &val);
        if (enable)
            val |= BIT(gpionumber % 8);
        else
            val &= ~BIT(gpionumber % 8);
        rv = ch943x_reg_write(s, (CH9434_GPIO_FUNC_EN_0 + (gpionumber / 8)) | CH943X_REG_OP_WRITE, 1, &val);
        break;
    }
    case IOCTL_CMD_CH9432_GPIOENABLE: {
        if (get_user(inarg, argval))
            return -EFAULT;
        gpionumber = (inarg >> 8) & 0xFF;
        enable = inarg & 0xFF;

        if (gpionumber == 5 && CH943X_TNOW_ENABLE(0)) {
            dev_err(s->dev, "%s - CH9432 GPIO%d is unavailable.\n", __func__, gpionumber);
            return -1;
        } else if (gpionumber == 6 && CH943X_TNOW_ENABLE(1)) {
            dev_err(s->dev, "%s - CH9432 GPIO%d is unavailable.\n", __func__, gpionumber);
            return -1;
        } else if (gpionumber == 7 && CH943X_EXCLK_ENABLE) {
            dev_err(s->dev, "%s - CH9432 GPIO%d is unavailable.\n", __func__, gpionumber);
            return -1;
        }

        if (enable) {
            if (gpionumber == 0) {
                if (ch943x_iofunc_get(s, CH943X_IO_DEF_R_EN, CH9432_DEF_CTS0_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9432_DEF_CTS0_ADD, DISABLE);
            }
            if (gpionumber == 1) {
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH9432_MUL_RTS0_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9432_MUL_RTS0_ADD, DISABLE);
            }
            if (gpionumber == 2) {
                if (ch943x_iofunc_get(s, CH943X_IO_DEF_R_EN, CH9432_DEF_CTS1_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9432_DEF_CTS1_ADD, DISABLE);
            }
            if (gpionumber == 3) {
                if (ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH9432_MUL_RTS1_ADD))
                    ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9432_MUL_RTS1_ADD, DISABLE);
            }

            for (i = 0; i < s->chip.nr_gpio; i++) {
                if (i == gpionumber) {
                    if (!ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH9432_MUL_GPIO0_ADD + i)) {
                        ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9432_MUL_GPIO0_ADD + i, ENABLE);
                    }
                }
            }
        }

        ret = ch943x_reg_read(s, CH9434_GPIO_FUNC_EN_0 + (gpionumber / 8), 1, &val);
        if (enable)
            val |= BIT(gpionumber % 8);
        else
            val &= ~BIT(gpionumber % 8);
        rv = ch943x_reg_write(s, (CH9434_GPIO_FUNC_EN_0 + (gpionumber / 8)) | CH943X_REG_OP_WRITE, 1, &val);
        break;
    }
    case IOCTL_CTRL_WRITE: {
        get_user(arg1, (u8 __user *)arg);
        get_user(arg2, (u32 __user *)((u8 *)arg + 1));
        arg3 = (unsigned long)((u8 __user *)arg + 5);
        rv = ch943x_ioctl_write(s, arg1, arg2, (u8 __user *)arg3);
        break;
    }
    case IOCTL_CTRL_READ: {
        get_user(arg1, (u8 __user *)arg);
        get_user(arg2, (u32 __user *)((u8 *)arg + 1));
        arg3 = (unsigned long)((u8 __user *)arg + 5);
        rv = ch943x_ioctl_read(s, arg1, arg2, (u8 __user *)arg3);
        break;
    }
#ifdef USE_SERIAL_MODE
    case IOCTL_CTRL_SERIALMODE_FIFO_READ: {
        get_user(arg1, (u8 __user *)arg);
        get_user(arg2, (u32 __user *)((u8 *)arg + 1));
        arg3 = (unsigned long)((u8 __user *)arg + 5);
        rv = ch943x_ioctl_serialmode_fifo_read(s, arg1, arg2, (u8 __user *)arg3);
        break;
    }
    case IOCTL_CTRL_SERIALMODE_FIFO_WRITE: {
        get_user(arg1, (u8 __user *)arg);
        get_user(arg2, (u32 __user *)((u8 *)arg + 1));
        arg3 = (unsigned long)((u8 __user *)arg + 5);
        rv = ch943x_ioctl_serialmode_fifo_write(s, arg1, arg2, (u8 __user *)arg3);
        break;
        break;
    }
#endif
    default:
        rv = -ENOIOCTLCMD;
        break;
    }

out:
    return rv;
}

static ssize_t ch943x_proc_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
    struct ch943x *s = PDE_DATA(file_inode(file));
#else
    struct ch943x *s = pde_data(file_inode(file));
#endif
    char buf[16] = {0};
    ssize_t ret;
    uint8_t val;
    int retval;

    if (!(file->f_mode & FMODE_WRITE)) {
        pr_err("%s:can't write\n", __func__);
        return -EACCES;
    }

    if (count > (sizeof(buf) - 1)) {
        return -EINVAL;
    }

    ret = copy_from_user(buf, user_buf, count);
    if (ret) {
        return ret;
    }

    buf[count] = '\0';
    if (buf[count - 1] == '\n') {
        buf[count - 1] = '\0';
    }

    if (strcmp(buf, "idle") == 0) {
        val = 1;
        retval = ch943x_reg_write(s, CH943X_SLEEP_MODE_REG | CH943X_REG_OP_WRITE, 1, &val);
    } else if (strcmp(buf, "sleep") == 0) {
        val = 2;
        retval = ch943x_reg_write(s, CH943X_SLEEP_MODE_REG | CH943X_REG_OP_WRITE, 1, &val);
    } else if (strcmp(buf, "shut") == 0) {
        val = 3;
        retval = ch943x_reg_write(s, CH943X_SLEEP_MODE_REG | CH943X_REG_OP_WRITE, 1, &val);
    } else if (strcmp(buf, "resume") == 0) {
        val = 0x55;
        retval = ch943x_reg_write(s, (CH943X_SPR_REG + (0 * 0x10)) | CH943X_REG_OP_WRITE, 1, &val);
        mdelay(10);
    } else if (strcmp(buf, "test") == 0) { /* It can be used for debug */
        val = 0x5a;
        retval = ch943x_reg_write(s, (CH943X_SPR_REG + (0 * 0x10)) | CH943X_REG_OP_WRITE, 1, &val);
        val = 0xa5;
        retval = ch943x_reg_write(s, (CH943X_SPR_REG + (1 * 0x10)) | CH943X_REG_OP_WRITE, 1, &val);
    } else {
        pr_err("%s:invalid command\n", __func__);
    }

    if (retval < 0) {
        pr_err("%s: register write failed: %d\n", __func__, retval);
        return retval;
    }

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
        return -1;
    }

    buf = kzalloc(REGS_BUFSIZE, GFP_KERNEL);
    if (!buf)
        return 0;

    len += snprintf(buf + len, REGS_BUFSIZE - len, "============ch943x registers:============\n");
    for (i = 0; i < s->chip.nr_uart; i++) {
        if ((s->chip.chiptype == CHIP_CH9437) && IS_USE_SERIAL_MODE && (i == 0))
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
            if ((s->chip.chiptype == CHIP_CH9437) && IS_USE_SERIAL_MODE && (j == 0))
                continue;
            val = ch943x_port_read(&p->port, j);
            len += snprintf(buf + len, REGS_BUFSIZE - len, "reg:0x%02x val:0x%02x\n", j, val);
        }
    }

#ifdef CH9434D_CAN_ON
    if (s->chip.chiptype == CHIP_CH9434D) {
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
