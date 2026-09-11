#include "ch943x.h"

struct ch943x_reg {
    u8 iir;
    u8 lsr;
    u8 msr;
    u8 rfifo_cnt_l;
    u8 rfifo_cnt_h;
    u8 tfifo_cnt_l;
    u8 tfifo_cnt_h;
};

#ifdef USE_SERIAL_MODE
static int pack_ch9437_registers(struct ch943x *s, uint8_t *packet)
{
    int i = 0, j;

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
#endif

static int ch943x_port_update(struct uart_port *port, u8 reg, u8 mask, u8 val)
{
    u8 tmp;

    tmp = ch943x_port_read(port, reg);
    tmp &= ~mask;
    tmp |= val & mask;
    ch943x_port_write(port, reg, tmp);

    return 0;
}

static int ch943x_set_baud(struct uart_port *port, int baud)
{
    struct ch943x *s = dev_get_drvdata(port->dev);
    uint32_t clk = port->uartclk;
    u8 lcr, dll = 0, dlh = 0;
    u32 divider_value = 0, fractional_divider = 0;
    u64 integer_divider = 0;

    DRV_DEBUG(s->dev, "%s u%d %d\n", __func__, port->line, baud);

    if ((s->chip.chiptype == CHIP_CH9434A) || (s->chip.chiptype == CHIP_CH9434M)) {
        divider_value = clk * 10 / 8 / baud;
        divider_value = (divider_value + 5) / 10;
        dll = (u8)(divider_value & 0xFF);
        dlh = (u8)((divider_value >> 8) & 0xFF);
    } else {
        integer_divider = ((u64)25 * clk);            /* Compute: (clk * 25) / (4 * baud_rate) */
        do_div(integer_divider, 4 * baud);            /* (clk * 25) / (4 * baud_rate) == (clk/(16*baud)) * 100 */
        divider_value = (integer_divider / 100) << 4; /* Extract integer part: upper 12 bits for integer, lower 4 bits for fraction */
        fractional_divider = integer_divider - (100 * (divider_value >> 4));  /* Compute fractional part */
        fractional_divider = DIV_ROUND_CLOSEST(fractional_divider * 16, 100); /* Compute fractional divider and round */
        divider_value |= (fractional_divider & 0x0F);                         /* Merge integer and fractional parts */
        /* Carry adjustment */
        if ((s->chip.chiptype == CHIP_CH9438F) || (s->chip.chiptype == CHIP_CH9437F)) {
            if (fractional_divider & 0x10) {
                if ((divider_value >> 4) == 0xFFF) {
                    divider_value |= 0x0F; /* Saturation at max value */
                } else {
                    divider_value += (1 << 4); /* Integer part carry */
                }
            }
        }
        dll = (u8)(divider_value & 0xFF);
        dlh = (u8)((divider_value >> 8) & 0xFF);
    }

    lcr = ch943x_port_read(port, CH943X_LCR_REG);
    /* Open the LCR divisors for configuration */
    ch943x_port_write(port, CH943X_LCR_REG, CH943X_LCR_CONF_MODE_A);
    /* Write the new divisor */
    ch943x_port_write(port, CH943X_DLL_REG, dll);
    ch943x_port_write(port, CH943X_DLH_REG, dlh);
    /* Put LCR back to the normal mode */
    ch943x_port_write(port, CH943X_LCR_REG, lcr);

    return 0;
}

static void ch943x_handle_rx(struct uart_port *port, u32 rxlen, u8 iir, u8 lsr)
{
    struct ch943x *s = dev_get_drvdata(port->dev);
    struct ch943x_one *one = to_ch943x_one(port, port);
    u8 cmd, ch;
    u32 flag, bytes_read = 0, i;
    int ret;

    if ((s->chip.chiptype == CHIP_CH9438F) || (s->chip.chiptype == CHIP_CH9437F))
        cmd = 0x00 | (port->line * 8 + CH943X_RHR_REG);
    else
        cmd = (0x00 | CH943X_RHR_REG) + (port->line * 0x10);

    DRV_DEBUG(s->dev, "%s u%d cmd:%02x rxlen:%d\n", __func__, port->line, cmd, rxlen);

    switch (s->chip.chiptype) {
    case CHIP_CH9434D:
    case CHIP_CH9432D:
    case CHIP_CH9438F:
        ret = ch943x_reg_read(s, cmd, rxlen, one->rxbuf);
        if (ret < 0)
            return;
        break;
    case CHIP_CH9437F:
        if (s->chip.interface_mode == I2C_MODE) {
            ret = ch943x_reg_read(s, cmd, rxlen, one->rxbuf);
            if (ret < 0)
                return;
        } else {
#ifdef USE_SERIAL_MODE
            ret = ch9437_serialmode_fifo_read(s, cmd, rxlen, one->rxbuf);
            if (ret < 0)
                return;
#endif
        }
        break;
    case CHIP_CH9434A:
    case CHIP_CH9434M:
#ifdef USE_SPI_MODE
        if (s->spi_contmode) {
            ret = ch943x_reg_read(s, cmd, rxlen + 1, one->rxbuf);
            if (ret < 0)
                return;
            memcpy(one->rxbuf, one->rxbuf + 1, rxlen);
        } else {
            for (i = 0; i < rxlen; i++)
                one->rxbuf[i] = ch943x_port_read(port, CH943X_RHR_REG);
        }
#endif
        break;
    default:
        break;
    }

    bytes_read += rxlen;
    port->icount.rx += rxlen;

    if (atomic_read(&one->isopen) == 0) {
        return;
    }

    flag = TTY_NORMAL;
    if (unlikely(lsr & CH943X_LSR_BRK_ERROR_MASK)) {
        dev_err(s->dev, "%s u%d lsr:%02x error detect\n", __func__, port->line, lsr);
        if (lsr & CH943X_LSR_BI_BIT) {
            lsr &= ~(CH943X_LSR_FE_BIT | CH943X_LSR_PE_BIT);
            port->icount.brk++;
            if (uart_handle_break(port))
                goto ignore_char;
        } else if (lsr & CH943X_LSR_PE_BIT) {
            port->icount.parity++;
        } else if (lsr & CH943X_LSR_FE_BIT) {
            port->icount.frame++;
        } else if (lsr & CH943X_LSR_OE_BIT) {
            port->icount.overrun++;
            dev_err(s->dev, "%s u%d overrun detect\n", __func__, port->line);
        }

        lsr &= port->read_status_mask;
        if (lsr & CH943X_LSR_BI_BIT)
            flag = TTY_BREAK;
        else if (lsr & CH943X_LSR_PE_BIT)
            flag = TTY_PARITY;
        else if (lsr & CH943X_LSR_FE_BIT)
            flag = TTY_FRAME;
    }

    for (i = 0; i < bytes_read; i++) {
        ch = one->rxbuf[i];
        if (uart_handle_sysrq_char(port, ch))
            continue;
        if (lsr & port->ignore_status_mask)
            continue;
        uart_insert_char(port, lsr, CH943X_LSR_OE_BIT, ch, flag);
    }

ignore_char:
    tty_flip_buffer_push(&port->state->port);
}

static void ch943x_handle_tx(struct uart_port *port)
{
    struct ch943x *s = dev_get_drvdata(port->dev);
    struct ch943x_one *one = to_ch943x_one(port, port);
    struct circ_buf *xmit;
    u32 txlen, to_send, i;
    u8 cmd;
    int ret;

    DRV_DEBUG(s->dev, "%s u%d\n", __func__, port->line);

    if (!port->state)
        return;
    xmit = &port->state->xmit;

    /* xon/xoff char */
    if (unlikely(port->x_char)) {
        ch943x_port_write(port, CH943X_THR_REG, port->x_char);
        port->icount.tx++;
        port->x_char = 0;
        return;
    }

    if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
        DRV_DEBUG(s->dev, "u%d has no data or has stop send\n", port->line);
        ch943x_port_update(port, CH943X_IER_REG, CH943X_IER_THRI_BIT, 0);
        return;
    }

    /* Get length of data pending in circular buffer */
    to_send = uart_circ_chars_pending(xmit);
    if (likely(to_send)) {
        /* Limit to size of TX FIFO */
        txlen = CH943X_TXFIFO_SIZE;

        to_send = (to_send > txlen) ? txlen : to_send;
        /* Add data to send */
        port->icount.tx += to_send;

        /* Convert to linear buffer */
        for (i = 0; i < to_send; ++i) {
            one->txbuf[i] = xmit->buf[xmit->tail];
            xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
        }

        if ((s->chip.chiptype == CHIP_CH9438F) || (s->chip.chiptype == CHIP_CH9437F))
            cmd = (0x80 | (CH943X_THR_REG + port->line * 8));
        else
            cmd = (0x80 | CH943X_THR_REG) + (port->line * 0x10);

        switch (s->chip.chiptype) {
        case CHIP_CH9434D:
        case CHIP_CH9432D:
        case CHIP_CH9438F:
            ret = ch943x_reg_write(s, cmd, to_send, one->txbuf);
            if (ret < 0)
                return;
            break;
        case CHIP_CH9437F:
            if (s->chip.interface_mode == I2C_MODE) {
                ret = ch943x_reg_write(s, cmd, to_send, one->txbuf);
                if (ret < 0)
                    return;
            } else {
#ifdef USE_SERIAL_MODE
                ch9437_serialmode_fifo_write(s, cmd, to_send, one->txbuf);
                if (ret < 0) {
                    return;
                }
#endif
            }
            break;
        case CHIP_CH9434A:
        case CHIP_CH9434M:
#ifdef USE_SPI_MODE
            if (s->spi_contmode) {
                ret = ch943x_reg_write(s, cmd, to_send, one->txbuf);
                if (ret < 0)
                    return;
            } else {
                for (i = 0; i < to_send; i++) {
                    ret = ch943x_reg_write(s, cmd, 1, one->txbuf + i);
                    if (ret < 0)
                        return;
                }
            }
#endif
            break;
        default:
            break;
        }
    }

    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);
}

void ch943x_port_irq_bulkmode(struct ch943x *s)
{
    struct ch943x_reg reg[8] = {0};
    int ret, i;
    int nr_regs = 7;
    int packet_cnts, left;
    u32 rxlen;
#ifdef USE_SERIAL_MODE
    int n_tx;
    u8 txbuf[256] = {0};
#endif

    DRV_DEBUG(s->dev, "%s\n", __func__);

    if ((s->chip.chiptype == CHIP_CH9438F) || (s->chip.chiptype == CHIP_CH9434D) ||
        (s->chip.chiptype == CHIP_CH9432D)) {
        ret = ch943x_reg_read(s, CH943X_PORT_CMD_BULKMODE, nr_regs * s->chip.nr_uart, (u8 *)reg);
        if (ret < 0)
            return;
    } else if (s->chip.chiptype == CHIP_CH9437F) {
#ifdef USE_SERIAL_MODE
        n_tx = pack_ch9437_registers(s, txbuf);
        mutex_lock(&s->mutex_bus_access);
        ret = ch943x_ctrl_tty_write(s, n_tx, txbuf);
        if (ret < 0) {
            mutex_unlock(&s->mutex_bus_access);
            return;
        }
        ret = ch943x_ctrl_tty_read(s, nr_regs * 7, (u8 *)(&reg[1]));
        mutex_unlock(&s->mutex_bus_access);
        if (ret < 0) {
            return;
        }
#endif
    }

    for (i = 0; i < s->chip.nr_uart; i++) {
        if ((s->chip.chiptype == CHIP_CH9437F) && (i == 0))
            continue;
        if (reg[i].iir & CH943X_IIR_NO_INT_BIT)
            continue;
        DRV_DEBUG(s->dev, "u%d iir:%02x lsr:%02x msr:%02x rxl:%02x%02x txl:%02x%02x\n", i, reg[i].iir, reg[i].lsr,
                  reg[i].msr, reg[i].rfifo_cnt_h, reg[i].rfifo_cnt_l, reg[i].tfifo_cnt_h, reg[i].tfifo_cnt_l);

        switch (reg[i].iir & CH943X_IIR_ID_MASK) {
        case CH943X_IIR_RDI_SRC:
        case CH943X_IIR_RLSE_SRC:
        case CH943X_IIR_RTOI_SRC:
            rxlen = reg[i].rfifo_cnt_l | (reg[i].rfifo_cnt_h << 8);
            DRV_DEBUG(s->dev, "u%d rxlen:%d\n", i, rxlen);

            if (unlikely(rxlen == 0xFFFF)) {
                dev_err(s->dev, "u%d incorrect rxfifo len:%d", i, rxlen);
                break;
            } else if (unlikely(rxlen == 0)) {
                dev_info(s->dev, "%s u%d rxlen:%d iir:%02x lsr:%02x", __func__, i, rxlen, reg[i].iir, reg[i].lsr);
                break;
            }

            if (s->chip.chiptype == CHIP_CH9437F) {
                packet_cnts = rxlen / (CH943X_TXFIFO_SIZE - 128);
                left = rxlen % (CH943X_TXFIFO_SIZE - 128);
                while (packet_cnts--)
                    ch943x_handle_rx(&s->p[i].port, (CH943X_TXFIFO_SIZE - 128), reg[i].iir, reg[i].lsr);

                if (left > 0)
                    ch943x_handle_rx(&s->p[i].port, left, reg[i].iir, reg[i].lsr);
            } else {
                if (rxlen > CH943X_RXFIFO_SIZE)
                    rxlen = CH943X_RXFIFO_SIZE;
                ch943x_handle_rx(&s->p[i].port, rxlen, reg[i].iir, reg[i].lsr);
            }
            break;
        case CH943X_IIR_MSI_SRC:
            s->p[i].msr_reg = reg[i].msr;
            uart_handle_cts_change(&s->p[i].port, !!(reg[i].msr & CH943X_MSR_CTS_BIT));
            DRV_DEBUG(s->dev, "uart modem change. msr:%02x\n", reg[i].msr);
            break;
        case CH943X_IIR_THRI_SRC:
            mutex_lock(&s->mutex);
            ch943x_handle_tx(&s->p[i].port);
            mutex_unlock(&s->mutex);
            break;
        default:
            dev_err(s->dev, "port:%d unexpected interrupt. iir:%02x", i, reg[i].iir);
            break;
        }
    }
}

void ch943x_port_irq(struct ch943x *s, int portno)
{
    struct ch943x_one *p = s->p + portno;
    struct uart_port *port = &p->port;
    u8 iir, lsr, msr;
    u16 rxlen = 0;
    u8 rxlen_l, rxlen_h, data;

    iir = ch943x_port_read(port, CH943X_IIR_REG);
    if (iir & CH943X_IIR_NO_INT_BIT) {
        return;
    }
    lsr = ch943x_port_read(port, CH943X_LSR_REG);

    switch (iir & CH943X_IIR_ID_MASK) {
    case CH943X_IIR_RDI_SRC:
    case CH943X_IIR_RLSE_SRC:
    case CH943X_IIR_RTOI_SRC:
        data = port->line | CH943X_FIFO_RD_BIT;
        ch943x_reg_write(s, CH943X_FIFO_REG, 1, &data);
        ch943x_reg_read(s, CH943X_FIFOCL_REG, 1, &rxlen_l);
        ch943x_reg_read(s, CH943X_FIFOCH_REG, 1, &rxlen_h);
        rxlen = rxlen_l | (rxlen_h << 8);
        DRV_DEBUG(s->dev, "%s u%d rxlen:%d\n", __func__, portno, rxlen);

        if (unlikely(rxlen == 0xFFFF)) {
            dev_err(port->dev, "u%d incorrect rxfifo len:%d", portno, rxlen);
            break;
        } else if (unlikely(rxlen == 0)) {
            dev_info(port->dev, "%s u%d rxlen:%d iir:%02x lsr:%02x", __func__, port->line, rxlen, iir, lsr);
            if (s->chip.chiptype == CHIP_CH9434A) {
                /**
                 * Sometimes interrupt but no data in fifo, 
                 * cause by wrong fifo cnt after reset, need to reset fifo to clear wrong cnt
                 */
                p->err_times++;
                if (p->err_times > 10) {
                    p->err_times = 0;
                    ch943x_port_write(port, CH943X_FCR_REG, CH943X_FCR_RXRESET_BIT | CH943X_FCR_FIFO_BIT);
                    mdelay(2);
                }
            }
            break;
        }
        if (rxlen > CH943X_RXFIFO_SIZE)
            rxlen = CH943X_RXFIFO_SIZE;

        ch943x_handle_rx(port, rxlen, iir, lsr);
        break;
    case CH943X_IIR_MSI_SRC:
        msr = ch943x_port_read(port, CH943X_MSR_REG);
        p->msr_reg = msr;
        uart_handle_cts_change(port, !!(msr & CH943X_MSR_CTS_BIT));
        DRV_DEBUG(s->dev, "uart modem change. msr:%02x\n", msr);
        break;
    case CH943X_IIR_THRI_SRC:
        mutex_lock(&s->mutex);
        ch943x_handle_tx(port);
        mutex_unlock(&s->mutex);
        break;
    default:
        dev_err(port->dev, "port:%d unexpected interrupt. iir:%02x", port->line, iir);
        break;
    }
}

static void ch943x_wq_proc(struct work_struct *ws)
{
    struct ch943x_one *one = to_ch943x_one(ws, tx_work);
    struct ch943x *s = dev_get_drvdata(one->port.dev);
    uint8_t val;

    DRV_DEBUG(s->dev, "%s u%d\n", __func__, one->port.line);

    mutex_lock(&s->mutex);
    val = ch943x_port_read(&one->port, CH943X_IER_REG);
    if (val & CH943X_IER_THRI_BIT) {
        val &= ~CH943X_IER_THRI_BIT;
        ch943x_port_write(&one->port, CH943X_IER_REG, val);
    }
    val |= CH943X_IER_THRI_BIT;
    ch943x_port_write(&one->port, CH943X_IER_REG, val);
    mutex_unlock(&s->mutex);
}

static void ch943x_stop_tx(struct uart_port *port)
{
    struct ch943x_one *one = to_ch943x_one(port, port);
    struct ch943x *s = dev_get_drvdata(one->port.dev);

    DRV_DEBUG(s->dev, "%s\n", __func__);
    schedule_work(&one->stop_tx_work);
}

static void ch943x_stop_rx(struct uart_port *port)
{
    struct ch943x_one *one = to_ch943x_one(port, port);
    struct ch943x *s = dev_get_drvdata(one->port.dev);

    DRV_DEBUG(s->dev, "%s u%d\n", __func__, one->port.line);

    schedule_work(&one->stop_rx_work);
}

static void ch943x_start_tx(struct uart_port *port)
{
    struct ch943x_one *one = to_ch943x_one(port, port);
    struct ch943x *s = dev_get_drvdata(one->port.dev);

    /* handle rs485 */
    if ((one->rs485.flags & SER_RS485_ENABLED) && (one->rs485.delay_rts_before_send > 0)) {
        mdelay(one->rs485.delay_rts_before_send);
    }
    if (!work_pending(&one->tx_work)) {
        DRV_DEBUG(s->dev, "%s u%d\n", __func__, port->line);
        schedule_work(&one->tx_work);
    }
}

static void ch943x_stop_rx_work_proc(struct work_struct *ws)
{
    struct ch943x_one *one = to_ch943x_one(ws, stop_rx_work);
    struct ch943x *s = dev_get_drvdata(one->port.dev);

    DRV_DEBUG(s->dev, "%s u%d\n", __func__, one->port.line);

    mutex_lock(&s->mutex);
    one->port.read_status_mask &= ~CH943X_LSR_DR_BIT;
    ch943x_port_update(&one->port, CH943X_IER_REG, CH943X_IER_RDI_BIT, 0);
    ch943x_port_update(&one->port, CH943X_IER_REG, CH943X_IER_RLSI_BIT, 0);
    mutex_unlock(&s->mutex);
}

static void ch943x_stop_tx_work_proc(struct work_struct *ws)
{
    struct ch943x_one *one = to_ch943x_one(ws, stop_tx_work);
    struct ch943x *s = dev_get_drvdata(one->port.dev);

    DRV_DEBUG(s->dev, "%s\n", __func__);

    mutex_lock(&s->mutex);
    ch943x_port_update(&one->port, CH943X_IER_REG, CH943X_IER_THRI_BIT, 0);
    mutex_unlock(&s->mutex);
}

static u32 ch943x_tx_empty(struct uart_port *port)
{
    struct ch943x *s = dev_get_drvdata(port->dev);
    u32 lsr;
    u32 result;

    DRV_DEBUG(s->dev, "%s u%d\n", __func__, port->line);

    lsr = ch943x_port_read(port, CH943X_LSR_REG);
    result = (lsr & CH943X_LSR_THRE_BIT) ? TIOCSER_TEMT : 0;

    return result;
}

static u32 ch943x_get_mctrl(struct uart_port *port)
{
    u32 status, ret;
    struct ch943x *s = dev_get_drvdata(port->dev);
    struct ch943x_one *p = s->p + port->line;

    DRV_DEBUG(s->dev, "%s u%d msr_reg:%02x\n", __func__, port->line, p->msr_reg);
    status = p->msr_reg;
    ret = 0;
    if (status & UART_MSR_DCD)
        ret |= TIOCM_CAR;
    if (status & UART_MSR_RI)
        ret |= TIOCM_RNG;
    if (status & UART_MSR_DSR)
        ret |= TIOCM_DSR;
    if (status & UART_MSR_CTS)
        ret |= TIOCM_CTS;

    return ret;
}

static void ch943x_md_proc(struct work_struct *ws)
{
    struct ch943x_one *one = to_ch943x_one(ws, md_work);
    struct ch943x *s = dev_get_drvdata(one->port.dev);
    u32 mctrl = one->port.mctrl;
    u8 mcr = 0;

    if (mctrl & TIOCM_RTS) {
        mcr |= UART_MCR_RTS;
    }
    if (mctrl & TIOCM_DTR) {
        mcr |= UART_MCR_DTR;
    }
    if (mctrl & TIOCM_OUT1) {
        mcr |= UART_MCR_OUT1;
    }
    if (mctrl & TIOCM_OUT2) {
        mcr |= UART_MCR_OUT2;
    }
    if (mctrl & TIOCM_LOOP) {
        mcr |= UART_MCR_LOOP;
    }

    mcr |= one->mcr_force;

    DRV_DEBUG(s->dev, "%s - mcr:%02x, force:%02x\n", __func__, mcr, one->mcr_force);

    ch943x_port_write(&one->port, CH943X_MCR_REG, mcr);
}

static void ch943x_set_mctrl(struct uart_port *port, u32 mctrl)
{
    struct ch943x_one *one = to_ch943x_one(port, port);
    struct ch943x *s = dev_get_drvdata(one->port.dev);

    DRV_DEBUG(s->dev, "%s u%d mctrl:%08x\n", __func__, port->line, mctrl);

    schedule_work(&one->md_work);
}

static void ch943x_break_ctl(struct uart_port *port, int break_state)
{
    struct ch943x *s = dev_get_drvdata(port->dev);

    DRV_DEBUG(s->dev, "%s\n", __func__);
    if ((s->chip.chiptype == CHIP_CH9434D) || (s->chip.chiptype == CHIP_CH9432D))
        return;
    else
        ch943x_port_update(port, CH943X_LCR_REG, CH943X_LCR_TXBREAK_BIT, break_state ? CH943X_LCR_TXBREAK_BIT : 0);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void ch943x_set_termios(struct uart_port *port, struct ktermios *termios, const struct ktermios *old)
#else
static void ch943x_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
#endif
{
    struct ch943x *s = dev_get_drvdata(port->dev);
    struct ch943x_one *one = to_ch943x_one(port, port);
    u32 lcr;
    int baud;
    u8 bParityType;

    DRV_DEBUG(s->dev, "%s u%d\n", __func__, port->line);

    /* Word size */
    switch (termios->c_cflag & CSIZE) {
    case CS5:
        lcr = CH943X_LCR_WORD_LEN_5;
        break;
    case CS6:
        lcr = CH943X_LCR_WORD_LEN_6;
        break;
    case CS7:
        lcr = CH943X_LCR_WORD_LEN_7;
        break;
    case CS8:
        lcr = CH943X_LCR_WORD_LEN_8;
        break;
    default:
        lcr = CH943X_LCR_WORD_LEN_8;
        termios->c_cflag &= ~CSIZE;
        termios->c_cflag |= CS8;
        break;
    }

    bParityType =
        termios->c_cflag & PARENB ? (termios->c_cflag & PARODD ? 1 : 2) + (termios->c_cflag & CMSPAR ? 2 : 0) : 0;
    lcr |= CH943X_LCR_PARITY_BIT;

    switch (bParityType) {
    case 0x01:
        lcr |= CH943X_LCR_ODDPARITY_BIT;
        DRV_DEBUG(s->dev, "%s u%d parity:odd\n", __func__, port->line);
        break;
    case 0x02:
        lcr |= CH943X_LCR_EVENPARITY_BIT;
        DRV_DEBUG(s->dev, "%s u%d parity:even\n", __func__, port->line);
        break;
    case 0x03:
        lcr |= CH943X_LCR_MARKPARITY_BIT;
        DRV_DEBUG(s->dev, "%s u%d parity:mark\n", __func__, port->line);
        break;
    case 0x04:
        lcr |= CH943X_LCR_SPACEPARITY_BIT;
        DRV_DEBUG(s->dev, "%s u%d parity:space\n", __func__, port->line);
        break;
    default:
        lcr &= ~CH943X_LCR_PARITY_BIT;
        DRV_DEBUG(s->dev, "%s u%d parity:none\n", __func__, port->line);
        break;
    }

    /* Stop bits */
    if (termios->c_cflag & CSTOPB)
        lcr |= CH943X_LCR_STOPLEN_BIT; /* 2 stops */

    /* Set read status mask */
    port->read_status_mask = CH943X_LSR_OE_BIT;
    if (termios->c_iflag & INPCK)
        port->read_status_mask |= CH943X_LSR_PE_BIT | CH943X_LSR_FE_BIT;
    if (termios->c_iflag & (BRKINT | PARMRK))
        port->read_status_mask |= CH943X_LSR_BI_BIT;

    /* Set status ignore mask */
    port->ignore_status_mask = 0;
    if (termios->c_iflag & IGNBRK)
        port->ignore_status_mask |= CH943X_LSR_BI_BIT;
    if (!(termios->c_cflag & CREAD))
        port->ignore_status_mask |= CH943X_LSR_BRK_ERROR_MASK;

    /* Update LCR register */
    if ((s->chip.chiptype == CHIP_CH9434D) || (s->chip.chiptype == CHIP_CH9432D) ||
        (s->chip.chiptype == CHIP_CH9438F) || (s->chip.chiptype == CHIP_CH9437F)) {
        if ((bParityType == 0x01) || (bParityType == 0x02)) {
            lcr &= ~(BIT(0) | BIT(1));
        }
    }
    ch943x_port_write(port, CH943X_LCR_REG, lcr);

    /* Configure flow control */
    if (termios->c_cflag & CRTSCTS) {
        DRV_DEBUG(s->dev, "%s u%d enable rts/cts flow control\n", __func__, port->line);
        ch943x_port_update(port, CH943X_MCR_REG, CH943X_MCR_AFE | CH943X_MCR_RTS_BIT,
                           CH943X_MCR_AFE | CH943X_MCR_RTS_BIT);
        one->mcr_force |= CH943X_MCR_AFE | CH943X_MCR_RTS_BIT;

        // add on 20200608 suppose cts status is always valid here
        uart_handle_cts_change(port, 1);
    } else {
        DRV_DEBUG(s->dev, "%s u%d disable rts/cts flow control\n", __func__, port->line);
        ch943x_port_update(port, CH943X_MCR_REG, CH943X_MCR_AFE, 0);
        one->mcr_force &= ~(CH943X_MCR_AFE | CH943X_MCR_RTS_BIT);
    }

    /* Get baud rate generator configuration */
    baud = uart_get_baud_rate(port, termios, old, port->uartclk / 16 / 0xffff, port->uartclk / 16 * 24);
    /* Setup baudrate generator */
    ch943x_set_baud(port, baud);
    uart_update_timeout(port, termios->c_cflag, baud);
}

static int ch943x_startup(struct uart_port *port)
{
    struct ch943x *s = dev_get_drvdata(port->dev);
    struct ch943x_one *one = to_ch943x_one(port, port);

    DRV_DEBUG(s->dev, "%s u%d\n", __func__, port->line);

    if ((s->chip.chiptype == CHIP_CH9434D) || (s->chip.chiptype == CHIP_CH9432D) ||
        (s->chip.chiptype == CHIP_CH9437F) || (s->chip.chiptype == CHIP_CH9438F)) {
        ch943x_port_write(port, CH943X_FCR_REG, CH943X_FCR_RXRESET_BIT | CH943X_FCR_TXRESET_BIT);
        ch943x_port_write(port, CH943X_FCR_REG, CH943X_FCR_FIFO_BIT);
    } else {
        ch943x_port_write(port, CH943X_FCR_REG, CH943X_FCR_RXRESET_BIT | CH943X_FCR_TXRESET_BIT | CH943X_FCR_FIFO_BIT);
    }

    /* Enable RX, TX, CTS change interrupts */
    ch943x_port_write(port, CH943X_IER_REG, CH943X_IER_RDI_BIT | CH943X_IER_RLSI_BIT | CH943X_IER_MSI_BIT);

    msleep(20);
    atomic_set(&one->isopen, 1);
    one->mcr_force = CH943X_MCR_OUT2;

    return 0;
}

static void ch943x_shutdown(struct uart_port *port)
{
    struct ch943x *s = dev_get_drvdata(port->dev);
    struct ch943x_one *one = to_ch943x_one(port, port);

    DRV_DEBUG(s->dev, "%s u%d\n", __func__, port->line);
#ifdef USE_SERIAL_MODE
    /*
     * Flush any pending signals for the current process.  When close()
     * is called after a signal (e.g. SIGINT), the tty layer's
     * n_tty_write() will keep returning -ERESTARTSYS (-512) for every
     * kernel_write() because signal_pending(current) stays true.
     * Clearing signals here allows the shutdown register writes to
     * succeed.
     */
    flush_signals(current);
#endif

    /* Disable all interrupts */
    ch943x_port_write(port, CH943X_IER_REG, 0);
    if ((s->chip.chiptype == CHIP_CH9434D) || (s->chip.chiptype == CHIP_CH9432D) ||
        (s->chip.chiptype == CHIP_CH9438F) || (s->chip.chiptype == CHIP_CH9437F)) {
        ch943x_port_write(port, CH943X_FCR_REG, 0);
    }
    ch943x_port_write(port, CH943X_MCR_REG, 0);

    one->mcr_force = 0;

    atomic_set(&one->isopen, 0);

    /*
     * Wait for any in-flight interrupt handler to complete.
     * This ensures that no IRQ thread is still accessing port->state
     * when the serial core frees it after this function returns.
     */
    synchronize_irq(s->irq);
}

static const char *ch943x_type(struct uart_port *port)
{
    return NULL;
}

static int ch943x_request_port(struct uart_port *port)
{
    return 0;
}

static void ch943x_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE)
        port->type = PORT_SC16IS7XX;
}

static int ch943x_verify_port(struct uart_port *port, struct serial_struct *s)
{
    if ((s->type != PORT_UNKNOWN) && (s->type != PORT_SC16IS7XX))
        return -EINVAL;
    if (s->irq != port->irq)
        return -EINVAL;

    return 0;
}

static void ch943x_pm(struct uart_port *port, u32 state, u32 oldstate)
{
    struct ch943x *s = dev_get_drvdata(port->dev);

    DRV_DEBUG(s->dev, "%s u%d\n", __func__, port->line);
}

static void ch943x_null_void(struct uart_port *port)
{
}

static void ch943x_enable_ms(struct uart_port *port)
{
}

const struct uart_ops ch943x_ops = {
    .tx_empty = ch943x_tx_empty,
    .set_mctrl = ch943x_set_mctrl,
    .get_mctrl = ch943x_get_mctrl,
    .stop_tx = ch943x_stop_tx,
    .start_tx = ch943x_start_tx,
    .stop_rx = ch943x_stop_rx,
    .break_ctl = ch943x_break_ctl,
    .startup = ch943x_startup,
    .shutdown = ch943x_shutdown,
    .set_termios = ch943x_set_termios,
    .type = ch943x_type,
    .request_port = ch943x_request_port,
    .release_port = ch943x_null_void,
    .config_port = ch943x_config_port,
    .verify_port = ch943x_verify_port,
    .enable_ms = ch943x_enable_ms,
    .pm = ch943x_pm,
};

int ch943x_register_uart_driver(struct ch943x *s)
{
    int ret;
#ifdef MULTI_CHIP_MODE
    const char *ch943x_uart_name[] = {"ttyCH943XA", "ttyCH943XB", "ttyCH943XC", "ttyCH943XD",
                                      "ttyCH943XE", "ttyCH943XF", "ttyCH943XG", "ttyCH943XH"};
    const char *ch943x_driver_name[] = {"ch943x_uartA", "ch943x_uartB", "ch943x_uartC", "ch943x_uartD",
                                        "ch943x_uartE", "ch943x_uartF", "ch943x_uartG", "ch943x_uartH"};
#endif
    s->uart.owner = THIS_MODULE;
#ifdef MULTI_CHIP_MODE
    s->uart.dev_name = ch943x_uart_name[s->minor];
    s->uart.driver_name = ch943x_driver_name[s->minor];
#else
    s->uart.dev_name = "ttyCH943X";
    s->uart.driver_name = "ch943x_uart";
#endif
    s->uart.nr = s->chip.nr_uart;
    ret = uart_register_driver(&s->uart);
    if (ret) {
        dev_err(s->dev, "Registering UART driver failed\n");
        return -1;
    }

    return 0;
}

int ch943x_register_uart_port(struct ch943x *s)
{
    int i, ret;
    u8 ctrl1_data = 0, ctrl2_data = 0;
    struct ch943x_one *p;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    s->p = devm_kzalloc(s->dev, sizeof(struct ch943x_one) * s->chip.nr_uart, GFP_KERNEL);
    if (!s->p) {
        dev_err(s->dev, "Error allocating port structure\n");
        return -ENOMEM;
    }

    for (i = 0; i < s->chip.nr_uart; i++) {
        if ((s->chip.chiptype == CHIP_CH9437F) && IS_USE_SERIAL_MODE && (i == 0))
            continue;
        p = s->p + i;
        /* Initialize port data */
        p->port.line = i;
        p->port.dev = s->dev;
        p->port.irq = s->irq;
        p->port.type = PORT_SC16IS7XX;
        p->port.fifosize = CH943X_TXFIFO_SIZE;
        p->port.flags = UPF_FIXED_TYPE | UPF_LOW_LATENCY;
        p->port.iotype = UPIO_PORT;
        p->port.ops = &ch943x_ops;

        p->rxbuf = kmalloc(CH943X_RXFIFO_SIZE * 2, GFP_KERNEL);
        if (!p->rxbuf) {
            dev_err(s->dev, "kmalloc rxbuf failed\n");
            return -ENOMEM;
        }

        p->txbuf = kmalloc(CH943X_TXFIFO_SIZE * 2, GFP_KERNEL);
        if (!p->txbuf) {
            dev_err(s->dev, "kmalloc txbuf failed\n");
            return -ENOMEM;
        }

        atomic_set(&p->isopen, 0);
        /* Disable all interrupts */
        ch943x_port_write(&p->port, CH943X_IER_REG, 0);
        /* Disable uart interrupts */
        ch943x_port_write(&p->port, CH943X_MCR_REG, 0);
        if ((s->chip.chiptype != CHIP_CH9434A) && (s->chip.chiptype != CHIP_CH9434M)) {
            ch943x_port_write(&p->port, CH943X_FCR_REG, 0);
        }

        p->msr_reg = ch943x_port_read(&p->port, CH943X_MSR_REG);

        /* Initialize queue for start TX */
        INIT_WORK(&p->tx_work, ch943x_wq_proc);
        /* Initialize queue for changing mode */
        INIT_WORK(&p->md_work, ch943x_md_proc);

        INIT_WORK(&p->stop_rx_work, ch943x_stop_rx_work_proc);
        INIT_WORK(&p->stop_tx_work, ch943x_stop_tx_work_proc);

        /* Register port */
        ret = uart_add_one_port(&s->uart, &p->port);
        if (ret < 0) {
            dev_err(s->dev, "Failed to add UART: %d\n", ret);
            return ret;
        }
        dev_info(s->dev, "%s%d: uart device\n", s->uart.dev_name, i);
    }

    for (i = 0; i < s->chip.nr_uart; i++) {
        if ((s->chip.chiptype == CHIP_CH9437F) && IS_USE_SERIAL_MODE && (i == 0))
            continue;

        p = s->p + i;
        if ((s->chip.chiptype == CHIP_CH9434D) || (s->chip.chiptype == CHIP_CH9432D))
            p->port.uartclk = 96000000;
        else if ((s->chip.chiptype == CHIP_CH9438F) || (s->chip.chiptype == CHIP_CH9437F))
            p->port.uartclk = 192000000;
        else
            p->port.uartclk = 32000000 * 15 / 13;
    }

    /* TNOW function enable */
    if ((s->chip.chiptype == CHIP_CH9434A) || (s->chip.chiptype == CHIP_CH9434M) ||
        (s->chip.chiptype == CHIP_CH9434D) || (s->chip.chiptype == CHIP_CH9432D)) {
        for (i = 0; i < s->chip.nr_uart; i++) {
            if (s->chip.chiptype == CHIP_CH9434D) {
                if ((i == 3) && CH943X_EXCLK_ENABLE)
                    continue;
                if ((i == 0 || i == 1) && s->can_on)
                    continue;
            }

            if (s->tnow_enable_bits & BIT(i)) {
                p = s->p + i;
                s->reg485 |= BIT(i);
            }
        }
        ch943x_reg_write(s, CH943X_RS485_CTRL1_REG, 1, &s->reg485);
    } else if (s->chip.chiptype == CHIP_CH9438F || s->chip.chiptype == CHIP_CH9437F) {
        for (i = 0; i < s->chip.nr_uart; i++) {
            if (s->chip.chiptype == CHIP_CH9438F) {
                if (i == 1)
                    continue;
                if (i == 7 && s->extern_clock_on) {
                    continue;
                }
            } else if (s->chip.chiptype == CHIP_CH9437F) {
                if ((i == 0) && (s->chip.interface_mode == SERIAL_MODE))
                    continue;
                if (i == 7)
                    continue;
            }

            if (s->tnow_enable_bits & BIT(i)) {
                p = s->p + i;
                s->reg485 |= BIT(i);
            }
        }
        ctrl1_data = s->reg485 & 0x0f;
        ctrl2_data = (s->reg485 >> 4) & 0x0f;
        ch943x_reg_write(s, CH943X_RS485_CTRL1_REG, 1, &ctrl1_data);
        ch943x_reg_write(s, CH943X_RS485_CTRL2_REG, 1, &ctrl2_data);
    }

    /**
     * Used to increase the flipping speed of TNOW
     *
     * if ((s->chip.chiptype != CHIP_CH9434A) && (s->chip.chiptype != CHIP_CH9434M)) {
     *     ch943x_reg_read(s, 0x40 | CH943X_REG_OP_READ, 1, &val);
     *     val |= (1 << 0);
     *     ch943x_reg_write(s, 0x40, 1, &val);
     * }
     */

    return 0;
}

void ch943x_uart_remove(struct ch943x *s)
{
    int i;
    struct ch943x_one *p;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    for (i = 0; i < s->uart.nr; i++) {
        if (s->chip.chiptype == CHIP_CH9437F && IS_USE_SERIAL_MODE && i == 0)
            continue;

        p = s->p + i;
        cancel_work_sync(&p->tx_work);
        cancel_work_sync(&p->md_work);
        cancel_work_sync(&p->stop_rx_work);
        cancel_work_sync(&p->stop_tx_work);
        uart_remove_one_port(&s->uart, &p->port);

        kfree(p->txbuf);
        kfree(p->rxbuf);
    }
    uart_unregister_driver(&s->uart);
}
