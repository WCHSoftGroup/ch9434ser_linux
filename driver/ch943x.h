#define DEBUG
#define VERBOSE_DEBUG

#include "linux/version.h"
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/spi/spi.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/gpio/driver.h>
#include <linux/netdevice.h>
#include <linux/uaccess.h>
#include <linux/can/core.h>
#include <linux/list.h>
#include <linux/can/dev.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0))
#include <linux/can/led.h>
#endif
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/fs.h>
#include <linux/fs_struct.h>
#include <linux/proc_fs.h>

#include "ch943x_cfg.h"

#define DRIVER_AUTHOR   "WCH"
#define DRIVER_DESC     "SPI/I2C/UART to SERIAL/CAN/GPIO driver for CH9434/CH9438/CH9437/CH9432, etc"
#define VERSION_DESC    "V1.5 On 2026.02"
#define CH943X_NAME_SPI "ch943x_spi"
#define CH943X_NAME_I2C "ch943x_i2c"

#if ENABLE_DRIVER_DEBUG
#define DRV_DEBUG(dev, format, ...)          \
    do {                                     \
        dev_dbg(dev, format, ##__VA_ARGS__); \
    } while (0)
#else
#define DRV_DEBUG(dev, format, ...)          \
    if (0) {                                 \
        dev_dbg(dev, format, ##__VA_ARGS__); \
    }
#endif

#if ENABLE_DRIVER_VERBOSEDEGUG
#define DRV_DEBUG_HEXDUMP(prefix_str, prefix_type, rowsize, groupsize, buf, len, ascii)           \
    do {                                                                                          \
        print_hex_dump(KERN_DEBUG, prefix_str, prefix_type, rowsize, groupsize, buf, len, ascii); \
    } while (0)
#else
#define DRV_DEBUG_HEXDUMP(prefix_str, prefix_type, rowsize, groupsize, buf, len, ascii) no_printk(KERN_DEBUG prefix_str)
#endif

#define ENABLE  1
#define DISABLE 0

#define IOCTL_MAGIC                  'W'
#define IOCTL_CMD_CH9434D_GPIOENABLE _IOWR(IOCTL_MAGIC, 0x86, uint16_t)
#define IOCTL_CMD_CH9438_GPIOENABLE  _IOWR(IOCTL_MAGIC, 0x90, uint16_t)
#define IOCTL_CMD_CH9437_GPIOENABLE  _IOWR(IOCTL_MAGIC, 0x91, uint16_t)
#define IOCTL_CMD_CH9432_GPIOENABLE  _IOWR(IOCTL_MAGIC, 0x92, uint16_t)
#define IOCTL_CTRL_WRITE             _IOWR(IOCTL_MAGIC, 0x87, uint16_t)
#define IOCTL_CTRL_READ              _IOWR(IOCTL_MAGIC, 0x88, uint16_t)
#define IOCTL_CMD_GETCHIPTYPE        _IOWR(IOCTL_MAGIC, 0x89, uint16_t)

#define IOCTL_CTRL_SERIALMODE_FIFO_READ  _IOWR(IOCTL_MAGIC, 0x93, uint16_t)
#define IOCTL_CTRL_SERIALMODE_FIFO_WRITE _IOWR(IOCTL_MAGIC, 0x94, uint16_t)

#define VER_LEN             4
#define CH943X_REG_OP_WRITE 0x80
#define CH943X_REG_OP_READ  0x00

/***********************************************************************/
#define CH943X_IO_MULTI_W_EN  0x01
#define CH943X_IO_MULTI_R_EN  0x81
#define CH943X_IO_DEF_W_EN    0x03
#define CH943X_IO_DEF_R_EN    0x83
#define CH9434X_IO_CMD_ACT    0xA5
#define CH9434X_IO_CMD_COMP   0x5A
#define CH943X_IO_SEL_FUN_CFG 0x45

/**
 * CH9434D/CH9437/CH9438/CH9432 INT#
 */
#define CH943X_MUL_INT_ADD 1
/**
 * CH9437/CH9438 UART0~6/7
 * CH9434D UART0~3
 * CH9432 UART0~1
 */
#define CH943X_DEF_U0_ADD 1
#define CH943X_DEF_U1_ADD 2
#define CH943X_DEF_U2_ADD 3
#define CH943X_DEF_U3_ADD 4
#define CH943X_DEF_U4_ADD 5
#define CH943X_DEF_U5_ADD 6
#define CH943X_DEF_U6_ADD 7
#define CH943X_DEF_U7_ADD 8

/**
 * CH9437/CH9438 TNOW0~7
 * CH9434D TNOW0~3
 */
#define CH943X_MUL_TNOW0_ADD 2
#define CH943X_MUL_TNOW1_ADD 3
#define CH943X_MUL_TNOW2_ADD 4
#define CH943X_MUL_TNOW3_ADD 5
#define CH943X_MUL_TNOW4_ADD 6
#define CH943X_MUL_TNOW5_ADD 7
#define CH943X_MUL_TNOW6_ADD 8
#define CH943X_MUL_TNOW7_ADD 9
/**
 * CH9437/CH9438 RTS0~7
 */
#define CH943X_MUL_RTS0_ADD 18
#define CH943X_MUL_RTS1_ADD 19
#define CH943X_MUL_RTS2_ADD 20
#define CH943X_MUL_RTS3_ADD 21
#define CH943X_MUL_RTS4_ADD 22
#define CH943X_MUL_RTS5_ADD 23
#define CH943X_MUL_RTS6_ADD 24
#define CH943X_MUL_RTS7_ADD 25
/**
 * CH9434D RTS0/3
 */
#define CH9434D_MUL_RTS0_ADD 8
#define CH9434D_MUL_RTS3_ADD 11
/**
 * CH9432 RTS0/1
 */
#define CH9432_MUL_RTS0_ADD 4
#define CH9432_MUL_RTS1_ADD 5

/**
 * CH9437/CH9438 CTS0~7
 */
#define CH943X_MUL_CTS0_ADD 10
#define CH943X_MUL_CTS1_ADD 11
#define CH943X_MUL_CTS2_ADD 12
#define CH943X_MUL_CTS3_ADD 13
#define CH943X_MUL_CTS4_ADD 14
#define CH943X_MUL_CTS5_ADD 15
#define CH943X_MUL_CTS6_ADD 16
#define CH943X_MUL_CTS7_ADD 17
/**
 * CH9434D CTS0/3
 */
#define CH9434D_DEF_CTS0_ADD 8
#define CH9434D_DEF_CTS3_ADD 9
/**
 * CH9432 CTS0/3
 */
#define CH9432_DEF_CTS0_ADD 4
#define CH9432_DEF_CTS1_ADD 5
/**
 * CH9437/CH9438 GPIO0~7
 */
#define CH943X_MUL_GPIO0_ADD 26
#define CH943X_MUL_GPIO1_ADD 27
#define CH943X_MUL_GPIO2_ADD 28
#define CH943X_MUL_GPIO3_ADD 29
#define CH943X_MUL_GPIO4_ADD 30
#define CH943X_MUL_GPIO5_ADD 31
#define CH943X_MUL_GPIO6_ADD 32
#define CH943X_MUL_GPIO7_ADD 33
/**
 * CH9434D GPIO0~3
 */
#define CH9434D_MUL_GPIO0_ADD 12
#define CH9434D_MUL_GPIO1_ADD 13
#define CH9434D_MUL_GPIO2_ADD 14
#define CH9434D_MUL_GPIO3_ADD 15
/**
 * CH9432 GPIO0~7
 */
#define CH9432_MUL_GPIO0_ADD 6
#define CH9432_MUL_GPIO1_ADD 7
#define CH9432_MUL_GPIO2_ADD 8
#define CH9432_MUL_GPIO3_ADD 9
#define CH9432_MUL_GPIO4_ADD 10
#define CH9432_MUL_GPIO5_ADD 11
#define CH9432_MUL_GPIO6_ADD 12
#define CH9432_MUL_GPIO7_ADD 13
/**
 * CH9437/CH9438 XI/XO
 */
#define CH943X_DEF_HSE_ADD 9
/**
 * CH9434D XI/XO
 */
#define CH9434D_DEF_HSE_ADD 5
/**
 * CH9432 XI/XO
 */
#define CH9432_DEF_HSE_ADD 3
/**
 * CH9434D CAN TX/RX
 */
#define CH9434D_DEF_CAN_ADD 6
/*************************************************************************/

static const int ch943x_tnow_enable[] = {
#ifdef CH943X_TNOW0_ON
    1,
#else
    0,
#endif
#ifdef CH943X_TNOW1_ON
    1,
#else
    0,
#endif
#ifdef CH943X_TNOW2_ON
    1,
#else
    0,
#endif
#ifdef CH943X_TNOW3_ON
    1,
#else
    0,
#endif
#ifdef CH943X_TNOW4_ON
    1,
#else
    0,
#endif
#ifdef CH943X_TNOW5_ON
    1,
#else
    0,
#endif
#ifdef CH943X_TNOW6_ON
    1,
#else
    0,
#endif
#ifdef CH943X_TNOW7_ON
    1,
#else
    0,
#endif
};

#define CH943X_TNOW_ENABLE(n) \
    ((n) >= 0 && (n) < (int)(sizeof(ch943x_tnow_enable) / sizeof(ch943x_tnow_enable[0])) ? ch943x_tnow_enable[n] : 0)

#if defined(USE_SERIAL_MODE)
#define IS_USE_SERIAL_MODE 1
#else
#define IS_USE_SERIAL_MODE 0
#endif

#if defined(USE_I2C_MODE)
#define IS_USE_I2C_MODE 1
#else
#define IS_USE_I2C_MODE 0
#endif

#if defined(USE_SPI_MODE)
#define IS_USE_SPI_MODE 1
#else
#define IS_USE_SPI_MODE 0
#endif

#if defined(CH9434D_CAN_ON)
#define CH9434D_CAN_ENABLE 1
#else
#define CH9434D_CAN_ENABLE 0
#endif

#if defined(EXTERN_CLOCK)
#define CH943X_EXCLK_ENABLE 1
#else
#define CH943X_EXCLK_ENABLE 0
#endif

/* -----------------------------------------------------------------------------
 *                         CH943X UART register definitions
 * -----------------------------------------------------------------------------
 */
#define CH943X_PORT_CMD_BULKMODE 0x4C

#define CH943X_RHR_REG (0x00) /* RX FIFO */
#define CH943X_THR_REG (0x00) /* TX FIFO */
#define CH943X_IER_REG (0x01) /* Interrupt enable */
#define CH943X_IIR_REG (0x02) /* Interrupt Identification */
#define CH943X_FCR_REG (0x02) /* FIFO control */
#define CH943X_LCR_REG (0x03) /* Line Control */
#define CH943X_MCR_REG (0x04) /* Modem Control */
#define CH943X_LSR_REG (0x05) /* Line Status */
#define CH943X_MSR_REG (0x06) /* Modem Status */
#define CH943X_SPR_REG (0x07) /* Scratch Pad */

#define CH943X_CLK_REG         (0x48) /* Clock Set */
#define CH943X_RS485_CTRL1_REG (0x41) /* UART0~3 RS485 Control */
#define CH943X_RS485_CTRL2_REG (0x47) /* UART4~7 RS485 Control */
#define CH943X_FIFO_REG        (0x42) /* FIFO Control */
#define CH943X_FIFOCL_REG      (0x43) /* FIFO Count Low */
#define CH943X_FIFOCH_REG      (0x44) /* FIFO Count High */
#define CH943X_SLEEP_MODE_REG  (0x4a) /* SLEEP Mode */

#define CH943X_GPIOEN_REG  (0x50) /* GPIO Enable Set */
#define CH943X_GPIODIR_REG (0x54) /* GPIO Direction Set */
#define CH943X_GPIOPU_REG  (0x58) /* GPIO PullUp Set */
#define CH943X_GPIOPD_REG  (0x5C) /* GPIO PullDown Set */
#define CH943X_GPIOVAL_REG (0x60) /* GPIO Value Set */

#define CH943X_SPI_CONT_MODE_REG (0x64) /* SPI transfer mode Set */
#define CH943X_CHIP_VER_REG      (0x65) /* Firmware Version */

/* Special Register set: Only if (LCR[7] == 1) */
#define CH943X_DLL_REG (0x00) /* Divisor Latch Low */
#define CH943X_DLH_REG (0x01) /* Divisor Latch High */

/* IER register bits */
#define CH943X_IER_RDI_BIT  (1 << 0) /* Enable RX data interrupt */
#define CH943X_IER_THRI_BIT (1 << 1) /* Enable TX holding register interrupt */
#define CH943X_IER_RLSI_BIT (1 << 2) /* Enable RX line status interrupt */
#define CH943X_IER_MSI_BIT  (1 << 3) /* Enable Modem status interrupt */

/* IER enhanced register bits */
#define CH943X_IER_RESET_BIT    (1 << 7) /* Enable Soft reset */
#define CH943X_IER_LOWPOWER_BIT (1 << 6) /* Enable low power mode */
#define CH943X_IER_SLEEP_BIT    (1 << 5) /* Enable sleep mode */

/* FCR register bits */
#define CH943X_FCR_FIFO_BIT    (1 << 0) /* Enable FIFO */
#define CH943X_FCR_RXRESET_BIT (1 << 1) /* Reset RX FIFO */
#define CH943X_FCR_TXRESET_BIT (1 << 2) /* Reset TX FIFO */
#define CH943X_FCR_RXLVLL_BIT  (1 << 6) /* RX Trigger level LSB */
#define CH943X_FCR_RXLVLH_BIT  (1 << 7) /* RX Trigger level MSB */

/* IIR register bits */
#define CH943X_IIR_NO_INT_BIT (1 << 0) /* No interrupts pending */
#define CH943X_IIR_ID_MASK    0x0e     /* Mask for the interrupt ID */
#define CH943X_IIR_THRI_SRC   0x02     /* TX holding register empty */
#define CH943X_IIR_RDI_SRC    0x04     /* RX data interrupt */
#define CH943X_IIR_RLSE_SRC   0x06     /* RX line status error */
#define CH943X_IIR_RTOI_SRC   0x0c     /* RX time-out interrupt */
#define CH943X_IIR_MSI_SRC    0x00     /* Modem status interrupt */

/* LCR register bits */
#define CH943X_LCR_STOPLEN_BIT     (1 << 2)
#define CH943X_LCR_PARITY_BIT      (1 << 3) /* Parity bit enable */
#define CH943X_LCR_ODDPARITY_BIT   (0)      /* Odd parity bit enable */
#define CH943X_LCR_EVENPARITY_BIT  (1 << 4) /* Even parity bit enable */
#define CH943X_LCR_MARKPARITY_BIT  (1 << 5) /* Mark parity bit enable */
#define CH943X_LCR_SPACEPARITY_BIT (3 << 4) /* Space parity bit enable */

#define CH943X_LCR_TXBREAK_BIT (1 << 6) /* TX break enable */
#define CH943X_LCR_DLAB_BIT    (1 << 7) /* Divisor Latch enable */
#define CH943X_LCR_WORD_LEN_5  (0x00)
#define CH943X_LCR_WORD_LEN_6  (0x01)
#define CH943X_LCR_WORD_LEN_7  (0x02)
#define CH943X_LCR_WORD_LEN_8  (0x03)
#define CH943X_LCR_CONF_MODE_A CH943X_LCR_DLAB_BIT /* Special reg set */

/* MCR register bits */
#define CH943X_MCR_DTR_BIT  (1 << 0) /* DTR complement */
#define CH943X_MCR_RTS_BIT  (1 << 1) /* RTS complement */
#define CH943X_MCR_OUT1     (1 << 2) /* OUT1 */
#define CH943X_MCR_OUT2     (1 << 3) /* OUT2 */
#define CH943X_MCR_LOOP_BIT (1 << 4) /* Enable loopback test mode */
#define CH943X_MCR_AFE      (1 << 5) /* Enable Hardware Flow control */

/* LSR register bits */
#define CH943X_LSR_DR_BIT         (1 << 0) /* Receiver data ready */
#define CH943X_LSR_OE_BIT         (1 << 1) /* Overrun Error */
#define CH943X_LSR_PE_BIT         (1 << 2) /* Parity Error */
#define CH943X_LSR_FE_BIT         (1 << 3) /* Frame Error */
#define CH943X_LSR_BI_BIT         (1 << 4) /* Break Interrupt */
#define CH943X_LSR_BRK_ERROR_MASK 0x1E     /* BI, FE, PE, OE bits */
#define CH943X_LSR_THRE_BIT       (1 << 5) /* TX holding register empty */
#define CH943X_LSR_TEMT_BIT       (1 << 6) /* Transmitter empty */
#define CH943X_LSR_FIFOE_BIT      (1 << 7) /* Fifo Error */

/* MSR register bits */
#define CH943X_MSR_DCTS_BIT   (1 << 0) /* Delta CTS Clear To Send */
#define CH943X_MSR_DDSR_BIT   (1 << 1) /* Delta DSR Data Set Ready */
#define CH943X_MSR_DRI_BIT    (1 << 2) /* Delta RI Ring Indicator */
#define CH943X_MSR_DCD_BIT    (1 << 3) /* Delta CD Carrier Detect */
#define CH943X_MSR_CTS_BIT    (1 << 4) /* CTS */
#define CH943X_MSR_DSR_BIT    (1 << 5) /* DSR */
#define CH943X_MSR_RI_BIT     (1 << 6) /* RI */
#define CH943X_MSR_CD_BIT     (1 << 7) /* CD */
#define CH943X_MSR_DELTA_MASK 0x0F     /* Any of the delta bits! */

/* Clock Set */
#define CH943X_CLK_PLL_BIT (1 << 7) /* PLL Enable */
#define CH943X_CLK_EXT_BIT (1 << 6) /* Extenal Clock Enable */

/* FIFO */
#define CH943X_FIFO_RD_BIT (0 << 4) /* Receive FIFO */
#define CH943X_FIFO_WR_BIT (1 << 4) /* Receive FIFO */

/* SPI Cont Mode Set */
#define CH943X_SPI_CONTE_BIT (1 << 0) /* SPI Cont Enable */

/* Misc definitions */
#define CH943X_FIFO_SIZE (1536)
#define CH943X_CMD_DELAY 3

/* -----------------------------------------------------------------------------
 *                         CH9434D CAN Related
 * -----------------------------------------------------------------------------
 */
#define CAN_TX_CONTMODE
#define CAN_RX_CONTMODE
// #define CH943X_CANREG_NOTIMEINTER

#define CH943X_CANREG_CMD             0x46
#define CH943X_CANREG_CMD_NOTIMEINTER 0x47

#define CH9434D_CAN_CTLR 0x00
/*******************  Bit definition for CAN_CTLR register  ********************/
#define CAN_CTLR_INRQ  BIT(0)  /* Initialization Request */
#define CAN_CTLR_SLEEP BIT(1)  /* Sleep Mode Request */
#define CAN_CTLR_TXFP  BIT(2)  /* Transmit FIFO Priority */
#define CAN_CTLR_RFLM  BIT(3)  /* Receive FIFO Locked Mode */
#define CAN_CTLR_NART  BIT(4)  /* No Automatic Retransmission */
#define CAN_CTLR_AWUM  BIT(5)  /* Automatic Wakeup Mode */
#define CAN_CTLR_ABOM  BIT(6)  /* Automatic Bus-Off Management */
#define CAN_CTLR_TTCM  BIT(7)  /* Time Triggered Communication Mode */
#define CAN_CTLR_RESET BIT(15) /* CAN software master reset */
#define CAN_CTLR_DBF   BIT(16) /* CAN controller operating state selection during debugging */

#define CH9434D_CAN_STATR 0x01
/*******************  Bit definition for CAN_STATR register  ********************/
#define CAN_STATR_INAK  BIT(0)  /* Initialization Acknowledge */
#define CAN_STATR_SLAK  BIT(1)  /* Sleep Acknowledge */
#define CAN_STATR_ERRI  BIT(2)  /* Error Interrupt */
#define CAN_STATR_WKUI  BIT(3)  /* Wakeup Interrupt */
#define CAN_STATR_SLAKI BIT(4)  /* Sleep Acknowledge Interrupt */
#define CAN_STATR_TXM   BIT(8)  /* Transmit Mode */
#define CAN_STATR_RXM   BIT(9)  /* Receive Mode */
#define CAN_STATR_SAMP  BIT(10) /* Last Sample Point */
#define CAN_STATR_RX    BIT(11) /* CAN Rx Signal */

#define CH9434D_CAN_TSTATR 0x02
/*******************  Bit definition for CAN_TSTATR register  ********************/
#define CAN_TSTATR_RQCP0 BIT(0)  /* Request Completed Mailbox0 */
#define CAN_TSTATR_TXOK0 BIT(1)  /* Transmission OK of Mailbox0 */
#define CAN_TSTATR_ALST0 BIT(2)  /* Arbitration Lost for Mailbox0 */
#define CAN_TSTATR_TERR0 BIT(3)  /* Transmission Error of Mailbox0 */
#define CAN_TSTATR_ABRQ0 BIT(7)  /* Abort Request for Mailbox0 */
#define CAN_TSTATR_RQCP1 BIT(8)  /* Request Completed Mailbox1 */
#define CAN_TSTATR_TXOK1 BIT(9)  /* Transmission OK of Mailbox1 */
#define CAN_TSTATR_ALST1 BIT(10) /* Arbitration Lost for Mailbox1 */
#define CAN_TSTATR_TERR1 BIT(11) /* Transmission Error of Mailbox1 */
#define CAN_TSTATR_ABRQ1 BIT(15) /* Abort Request for Mailbox 1 */
#define CAN_TSTATR_RQCP2 BIT(16) /* Request Completed Mailbox2 */
#define CAN_TSTATR_TXOK2 BIT(17) /* Transmission OK of Mailbox 2 */
#define CAN_TSTATR_ALST2 BIT(18) /* Arbitration Lost for mailbox 2 */
#define CAN_TSTATR_TERR2 BIT(19) /* Transmission Error of Mailbox 2 */
#define CAN_TSTATR_ABRQ2 BIT(23) /* Abort Request for Mailbox 2 */

#define CAN_TSTATR_TME0 BIT(26) /* Transmit Mailbox 0 Empty */
#define CAN_TSTATR_TME1 BIT(27) /* Transmit Mailbox 1 Empty */
#define CAN_TSTATR_TME2 BIT(28) /* Transmit Mailbox 2 Empty */

#define CH9434D_CAN_RFIFO0 0x03
#define CH9434D_CAN_RFIFO1 0x04
/*******************  Bit definition for CAN_RFIFOx register  *******************/
#define CAN_RFIFOx_FMPx  ((u32)0x000000FF) /* FIFO x Message Pending */
#define CAN_RFIFOx_FULLx BIT(16)           /* FIFO x Full */
#define CAN_RFIFOx_FOVRx BIT(17)           /* FIFO x Overrun */
#define CAN_RFIFOx_RFOMx BIT(18)           /* Release FIFO x Output Mailbox */

#define CH9434D_CAN_INTENR 0x05
/********************  Bit definition for CAN_INTENR register  *******************/
#define CAN_INTENR_TMEIE  BIT(0)  /* Transmit Mailbox Empty Interrupt Enable */
#define CAN_INTENR_FMPIE0 BIT(1)  /* FIFO Message Pending Interrupt Enable */
#define CAN_INTENR_FFIE0  BIT(2)  /* FIFO Full Interrupt Enable */
#define CAN_INTENR_FOVIE0 BIT(3)  /* FIFO Overrun Interrupt Enable */
#define CAN_INTENR_FMPIE1 BIT(4)  /* FIFO Message Pending Interrupt Enable */
#define CAN_INTENR_FFIE1  BIT(5)  /* FIFO Full Interrupt Enable */
#define CAN_INTENR_FOVIE1 BIT(6)  /* FIFO Overrun Interrupt Enable */
#define CAN_INTENR_EWGIE  BIT(8)  /* Error Warning Interrupt Enable */
#define CAN_INTENR_EPVIE  BIT(9)  /* Error Passive Interrupt Enable */
#define CAN_INTENR_BOFIE  BIT(10) /* Bus-Off Interrupt Enable */
#define CAN_INTENR_LECIE  BIT(11) /* Last Error Code Interrupt Enable */
#define CAN_INTENR_ERRIE  BIT(15) /* Error Interrupt Enable */
#define CAN_INTENR_WKUIE  BIT(16) /* Wakeup Interrupt Enable */
#define CAN_INTENR_SLKIE  BIT(17) /* Sleep Interrupt Enable */

#define CH9434D_CAN_ERRSR 0x06
/********************  Bit definition for CAN_ERRSR register  *******************/
#define CAN_ERRSR_EWGF BIT(0) /* Error Warning Flag */
#define CAN_ERRSR_EPVF BIT(1) /* Error Passive Flag */
#define CAN_ERRSR_BOFF BIT(2) /* Bus-Off Flag */

#define CAN_ERRSR_TEC ((u32)0x00FF0000) /* Least significant byte of the 9-bit Transmit Error Counter */
#define CAN_ERRSR_REC ((u32)0xFF000000) /* Receive Error Counter */

#define CH9434D_CAN_BTIMR 0x07
/*******************  Bit definition for CAN_BTIMR register  ********************/
#define CAN_BTIMR_BRP  ((u32)0x000003FF) /* Baud Rate Prescaler */
#define CAN_BTIMR_TS1  ((u32)0x000F0000) /* Time Segment 1 */
#define CAN_BTIMR_TS2  ((u32)0x00700000) /* Time Segment 2 */
#define CAN_BTIMR_SJW  ((u32)0x03000000) /* Resynchronization Jump Width */
#define CAN_BTIMR_LBKM BIT(30)           /* Loop Back Mode (Debug) */
#define CAN_BTIMR_SILM BIT(31)           /* Silent Mode */

#define CH9434D_CAN_TTCTLR 0x08
/********************  Bit definition for CAN_TTCTLR register  *******************/
#define CAN_TTCTLR_TIMCMV ((u32)0x0000FFFF)
#define CAN_TTCTLR_TIMRST ((u32)0x00010000)
#define CAN_TTCTLR_MODE   ((u32)0x00020000)

#define CH9434D_CAN_TTCNT 0x09
/********************  Bit definition for CAN_TTCNT register  *******************/
#define CAN_TTCNT ((u32)0x0000FFFF)

#define CH9434D_CAN_TERR_CNT 0x0A
/********************  Bit definition for CAN_TERR_CNT register  *******************/
#define CAN_TERR_CNT ((u32)0x000001FF)

#define CH9434D_CAN_TXMIR0 0x0B
/******************  Bit definition for CAN_TXMI0R register  ********************/
#define CAN_TXMIRx_TXRQ ((u32)0x00000001) /* Transmit Mailbox Request */
#define CAN_TXMIRx_RTR  ((u32)0x00000002) /* Remote Transmission Request */
#define CAN_TXMIRx_IDE  ((u32)0x00000004) /* Identifier Extension */
#define CAN_TXMIRx_EXID ((u32)0x001FFFF8) /* Extended Identifier */
#define CAN_TXMIRx_STID ((u32)0xFFE00000) /* Standard Identifier or Extended Identifier */

#define CH9434D_CAN_TXMDTR0 0x0C
/******************  Bit definition for CAN_TXMDT0R register  *******************/
#define CAN_TXMDTRx_DLC  ((u32)0x0000000F) /* Data Length Code */
#define CAN_TXMDTRx_TGT  ((u32)0x00000100) /* Transmit Global Time */
#define CAN_TXMDTRx_TIME ((u32)0xFFFF0000) /* Message Time Stamp */

#define CH9434D_CAN_TXMDLR0 0x0D
/******************  Bit definition for CAN_TXMDL0R register  *******************/
#define CAN_TXMDLRx_DATA0 ((u32)0x000000FF) /* Data byte 0 */
#define CAN_TXMDLRx_DATA1 ((u32)0x0000FF00) /* Data byte 1 */
#define CAN_TXMDLRx_DATA2 ((u32)0x00FF0000) /* Data byte 2 */
#define CAN_TXMDLRx_DATA3 ((u32)0xFF000000) /* Data byte 3 */

#define CH9434D_CAN_TXMDHR0 0x0E
/******************  Bit definition for CAN_TXMDH0R register  *******************/
#define CAN_TXMDHRx_DATA4 ((u32)0x000000FF) /* Data byte 4 */
#define CAN_TXMDHRx_DATA5 ((u32)0x0000FF00) /* Data byte 5 */
#define CAN_TXMDHRx_DATA6 ((u32)0x00FF0000) /* Data byte 6 */
#define CAN_TXMDHRx_DATA7 ((u32)0xFF000000) /* Data byte 7 */

#define CH9434D_CAN_TXMIR1  0x0F
#define CH9434D_CAN_TXMDTR1 0x10
#define CH9434D_CAN_TXMDLR1 0x11
#define CH9434D_CAN_TXMDHR1 0x12

#define CH9434D_CAN_TXMIR2  0x13
#define CH9434D_CAN_TXMDTR2 0x14
#define CH9434D_CAN_TXMDLR2 0x15
#define CH9434D_CAN_TXMDHR2 0x16

#define CH9434D_CAN_RXMIR0 0x17
/*******************  Bit definition for CAN_RXMI0R register  *******************/
#define CAN_RXMIRx_RTR  ((u32)0x00000002) /* Remote Transmission Request */
#define CAN_RXMIRx_IDE  ((u32)0x00000004) /* Identifier Extension */
#define CAN_RXMIRx_EXID ((u32)0x001FFFF8) /* Extended Identifier */
#define CAN_RXMIRx_STID ((u32)0xFFE00000) /* Standard Identifier or Extended Identifier */

#define CH9434D_CAN_RXMDTR0 0x18
/*******************  Bit definition for CAN_RXMDT0R register  ******************/
#define CAN_RXMDTRx_DLC  ((u32)0x0000000F) /* Data Length Code */
#define CAN_RXMDTRx_BRS  ((u32)0x00000010)
#define CAN_RXMDTRx_ESI  ((u32)0x00000020)
#define CAN_RXMDHRx_RES  ((u32)0x00000100)
#define CAN_RXMDTRx_FMI  ((u32)0x0000FF00) /* Filter Match Index */
#define CAN_RXMDTRx_TIME ((u32)0xFFFF0000) /* Message Time Stamp */

#define CH9434D_CAN_RXMDLR0 0x19
/*******************  Bit definition for CAN_RXMDL0R register  ******************/
#define CAN_RXMDLRx_DATA0 ((u32)0x000000FF) /* Data byte 0 */
#define CAN_RXMDLRx_DATA1 ((u32)0x0000FF00) /* Data byte 1 */
#define CAN_RXMDLRx_DATA2 ((u32)0x00FF0000) /* Data byte 2 */
#define CAN_RXMDLRx_DATA3 ((u32)0xFF000000) /* Data byte 3 */

#define CH9434D_CAN_RXMDHR0 0x1A
/*******************  Bit definition for CAN_RXMDH0R register  ******************/
#define CAN_RXMDHRx_DATA4 ((u32)0x000000FF) /* Data byte 4 */
#define CAN_RXMDHRx_DATA5 ((u32)0x0000FF00) /* Data byte 5 */
#define CAN_RXMDHRx_DATA6 ((u32)0x00FF0000) /* Data byte 6 */
#define CAN_RXMDHRx_DATA7 ((u32)0xFF000000) /* Data byte 7 */

#define CH9434D_CAN_RX0READ_CONT  0x40
#define CH9434D_CAN_RX1READ_CONT  0x41
#define CH9434D_CAN_TX0WRITE_CONT 0x42
#define CH9434D_CAN_TX1WRITE_CONT 0x43
#define CH9434D_CAN_TX2WRITE_CONT 0x44

#define CH9434D_CAN_RXMIR1  0x1B
#define CH9434D_CAN_RXMDTR1 0x1C
#define CH9434D_CAN_RXMDLR1 0x1D
#define CH9434D_CAN_RXMDHR1 0x1E

#define CH9434D_CAN_FCTLR 0x1F
/*******************  Bit definition for CAN_FCTLR register  ********************/
#define CAN_FCTLR_FINIT ((uint8_t)BIT(0)) /* Filter Init Mode */

#define CH9434D_CAN_FMCFGR 0x20
/*******************  Bit definition for CAN_FMCFGR register  *******************/
#define CAN_FMCFGR_FBM_MASK ((u32)0x00003FFF) /* Filter Mode */
#define CAN_FMCFGR_FBM(x)   ((u32)(1 << x))   /* Filter Init Mode bit x */

#define CH9434D_CAN_FSCFGR 0x21
/*******************  Bit definition for CAN_FSCFGR register  *******************/
#define CAN_FSCFGR_FSC_MASK ((u32)0x00003FFF) /* Filter Scale Configuration */
#define CAN_FSCFGR_FSC(x)   ((u32)(1 << x))   /* Filter Scale Configuration bit x */

#define CH9434D_CAN_FAFIFOR 0x22
/******************  Bit definition for CAN_FAFIFOR register  *******************/
#define CAN_FAFIFOR_FFA_MASK ((u32)0x00003FFF) /* Filter FIFO Assignment */
#define CAN_FAFIFOR_FFA(x)   ((u32)(1 << x))   /* Filter FIFO Assignment for Filter x */

#define CH9434D_CAN_FWR 0x23
/*******************  Bit definition for CAN_FWR register  *******************/
#define CAN_FWR_FACT_MASK ((u32)0x00003FFF) /* Filter Active */
#define CAN_FWR_FACT(x)   ((u32)(1 << x))   /* Filter x Active */

/*******************  Bit definition for CAN_FnRm register  *******************/
#define CAN_FnRm_FB(x)      ((u32)(1 << x)) /* Filter bit x */
#define CH9434D_CAN_FxR1(x) (0x24 + x * 2)
#define CH9434D_CAN_FxR2(x) (0x25 + x * 2)

#define CH943X_DELAY_MS (5)

/* CAN_identifier_type */
#define CAN_Id_Extended BIT(2) /* Extended Id */

/* CAN_transmit_constants */
#define CAN_TxStatus_Failed    ((u8)0x00) /* CAN transmission failed */
#define CAN_TxStatus_Ok        ((u8)0x01) /* CAN transmission succeeded */
#define CAN_TxStatus_Pending   ((u8)0x02) /* CAN transmission pending */
#define CAN_TxStatus_NoMailBox ((u8)0x04) /* CAN cell did not provide an empty mailbox */

/* -----------------------------------------------------------------------------
 *                         CH943X GPIO register definitions
 * -----------------------------------------------------------------------------
 */
#define CH9434_GPIO_FUNC_EN_0 0x50
#define CH9434_GPIO_FUNC_EN_1 0x51
#define CH9434_GPIO_FUNC_EN_2 0x52
#define CH9434_GPIO_FUNC_EN_3 0x53

#define CH9434_GPIO_DIR_MOD_0 0x54
#define CH9434_GPIO_DIR_MOD_1 0x55
#define CH9434_GPIO_DIR_MOD_2 0x56
#define CH9434_GPIO_DIR_MOD_3 0x57

#define CH9434_GPIO_PU_MOD_0  0x58
#define CH9434_GPIO_DIR_MOD_4 0x58
#define CH9434_GPIO_PU_MOD_1  0x59
#define CH9434_GPIO_DIR_MOD_5 0x59
#define CH9434_GPIO_PU_MOD_2  0x5A
#define CH9434_GPIO_DIR_MOD_6 0x5A
#define CH9434_GPIO_PU_MOD_3  0x5B
#define CH9434_GPIO_DIR_MOD_7 0x5B

#define CH9434_GPIO_PD_MOD_0 0x5C
#define CH9434_GPIO_SET_0    0x5C
#define CH9434_GPIO_PD_MOD_1 0x5D
#define CH9434_GPIO_SET_1    0x5D
#define CH9434_GPIO_PD_MOD_2 0x5E
#define CH9434_GPIO_RESET_0  0x5E
#define CH9434_GPIO_PD_MOD_3 0x5F
#define CH9434_GPIO_RESET_1  0x5F

#define CH9434_GPIO_PIN_VAL_0 0x60
#define CH9434_GPIO_PIN_VAL_1 0x61
#define CH9434_GPIO_PIN_VAL_2 0x62
#define CH9434_GPIO_PIN_VAL_3 0x63

#define to_ch943x_one(p, e) ((container_of((p), struct ch943x_one, e)))

enum CHIPTYPE {
    CHIP_CH9434A = 0,
    CHIP_CH9434D,
    CHIP_CH9434M,
    CHIP_CH9438,
    CHIP_CH9437,
    CHIP_CH9432,
};

enum INTERFACE_MODE {
    I2C_MODE = 0,
    SPI_MODE,
    SERIAL_MODE,
};

struct ch943x;

struct ch943x_chip_info {
    enum CHIPTYPE chiptype;
    enum INTERFACE_MODE interface_mode;
    int nr_uart;
    int nr_gpio;
    uint8_t ver[VER_LEN];
    char chip_name[16];
};

struct ch943x_can_priv {
    struct can_priv can;
    struct net_device *ndev;
    struct spi_device *spi;
    struct ch943x *s;

    struct mutex ops_lock;
    struct mutex can_lock;
    struct workqueue_struct *wq;
    struct work_struct tx_work;
    struct work_struct restart_work;

    spinlock_t tx_lock;
    bool tx_busy[3];
    u8 txm_pendbits;

    int force_quit;
    int after_suspend;
#define AFTER_SUSPEND_UP      1
#define AFTER_SUSPEND_DOWN    2
#define AFTER_SUSPEND_POWER   4
#define AFTER_SUSPEND_RESTART 8
    int restart_tx;
    atomic_t can_isopen;
    struct regulator *power;
    struct regulator *transceiver;
};

struct can_tx_work {
    struct ch943x_can_priv *priv;
    struct work_struct work;
    struct sk_buff *skb;
    int tx_mailbox_id;
};

struct ch943x_one {
    struct uart_port port;
    struct work_struct tx_work;
    struct work_struct md_work;
    struct work_struct stop_rx_work;
    struct work_struct stop_tx_work;
    struct serial_rs485 rs485;
    u8 msr_reg;
    u8 ier;
    u8 mcr_force;
    atomic_t isopen;
    volatile bool txfifo_empty_flag;
    bool tnow_on;
    u8 *txbuf;
    u8 *rxbuf;
};

struct ch943x {
    struct ch943x_chip_info chip;
    struct device *dev;
    struct list_head ch943x_list;
    struct uart_driver uart;
    struct ch943x_can_priv *priv;
    struct mutex mutex;
    struct mutex mutex_bus_access;
#ifdef USE_SPI_MODE
    struct spi_device *spi_dev;
    bool spi_contmode;
#elif defined(USE_I2C_MODE)
    struct i2c_client *i2c;
#elif defined(USE_SERIAL_MODE)
    struct platform_device *pdev;
    struct file *fp;
    char ctrluart_path[32];
    int ctrluart_baud;
#endif
    char proc_file_name[16];
    uint8_t reg485;
#ifdef MULTI_CHIP_MODE
#endif
    uint8_t tnow_enable_bits;
    bool extern_clock_on;
    bool can_on;

    int minor;
    int irq;
    struct class *ch943x_io_class;
    struct cdev cdev;
    struct ch943x_one *p;
    u8 *local_buf;
    u8 rxfifo_buf[4096];
    u8 txfifo_buf[4096];
};

extern int ch943x_iofunc_set(struct ch943x *s, uint8_t reg, uint8_t io_cmd, uint8_t io_addr);
extern int ch943x_iofunc_get(struct ch943x *s, uint8_t io_cmd, uint8_t io_addr);
extern uint8_t ch943x_port_read(struct uart_port *port, uint8_t reg);
extern int ch943x_port_write(struct uart_port *port, uint8_t reg, uint8_t val);
extern int ch943x_reg_read(struct ch943x *s, u8 _cmd, u32 n_rx, void *rxbuf);
extern int ch943x_reg_write(struct ch943x *s, u8 _cmd, u32 n_tx, const void *txbuf);
extern int ch943x_port_read_version(struct ch943x *s, uint8_t reg, uint8_t *buf, uint8_t count);
extern int ch943x_port_update(struct uart_port *port, uint8_t reg, uint8_t mask, uint8_t val);
extern int ch943x_raw_write(struct uart_port *port, u8 reg, u8 *buf, u32 len);
extern int ch943x_raw_read(struct uart_port *port, uint8_t reg, u8 *buf, u32 len);
extern int ch943x_get_chip_version(struct ch943x *s);
extern int ch943x_scr_test(struct uart_port *port);

extern int ch943x_port_bulkread(struct ch943x *s, u8 reg, uint8_t *buf, int len);

extern int ch943x_register_uart_driver(struct ch943x *s);
extern int ch943x_register_uart_port(struct ch943x *s);
extern void ch943x_uart_remove(struct ch943x *s);

extern irqreturn_t ch943x_ist_top(int irq, void *dev_id);
extern irqreturn_t ch943x_ist(int irq, void *dev_id);

extern void ch943x_port_irq_bulkmode(struct ch943x *s);
extern void ch943x_port_irq(struct ch943x *s, int portno);

#ifdef USE_SERIAL_MODE
extern int ch943x_ctrl_tty_write(struct ch943x *s, u32 n_tx, const void *txbuf);
extern int ch943x_ctrl_tty_read(struct ch943x *s, u32 n_rx, void *rxbuf);
extern struct file *ch943x_ctrluart_open_as_root(const char *filename, int flags, umode_t mode);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
extern int ch943x_ctrluart_setopt(struct ch943x *s);
#endif
extern int ch9437_reg_bulkread_serialmode(struct ch943x *s, u8 *buf);
#endif

extern int ch943x_debugfs_init(struct ch943x *s);
extern void ch943x_debugfs_exit(struct ch943x *s);

extern int ch943x_can_register(struct ch943x *s);
extern void ch943x_can_remove(struct ch943x *s);
extern void ch943x_can_irq(struct ch943x *s);
extern int ch943x_canreg_write(struct ch943x *s, u8 reg, u32 val);
extern u32 ch943x_canreg_read(struct ch943x *s, u8 reg);

extern int ch943x_rxmailbox_read(struct ch943x *s, u8 reg, u8 *rxbuf);
extern int ch943x_txmailbox_write(struct ch943x *s, u8 reg, u32 n_tx, const void *txbuf);

extern int ch943x_io_enable(struct ch943x *s);
extern int __ch943x_io_ioctl(struct ch943x *s, unsigned int cmd, unsigned long arg);

#ifdef USE_CHIP_CH432
/* -----------------------------------------------------------------------------
 *                         CH432 UART driver definitions
 * -----------------------------------------------------------------------------
 */

/* external crystal freq */
#define CRYSTAL_FREQ 22118400

// #define USE_IRQ_FROM_DTS
// #define GPIO_NUMBER 0
// #define USE_SPI_MODE
#define CH43X_NAME     "ch43x"
#define CH43X_NAME_SPI "ch43x_spi"

/* CH43X register definitions */
#define CH43X_RHR_REG (0x00) /* RX FIFO */
#define CH43X_THR_REG (0x00) /* TX FIFO */
#define CH43X_IER_REG (0x01) /* Interrupt enable */
#define CH43X_IIR_REG (0x02) /* Interrupt Identification */
#define CH43X_FCR_REG (0x02) /* FIFO control */
#define CH43X_LCR_REG (0x03) /* Line Control */
#define CH43X_MCR_REG (0x04) /* Modem Control */
#define CH43X_LSR_REG (0x05) /* Line Status */
#define CH43X_MSR_REG (0x06) /* Modem Status */
#define CH43X_SPR_REG (0x07) /* Scratch Pad */

/* Special Register set: Only if (LCR[7] == 1) */
#define CH43X_DLL_REG (0x00) /* Divisor Latch Low */
#define CH43X_DLH_REG (0x01) /* Divisor Latch High */

/* IER register bits */
#define CH43X_IER_RDI_BIT  (1 << 0) /* Enable RX data interrupt */
#define CH43X_IER_THRI_BIT (1 << 1) /* Enable TX holding register interrupt */
#define CH43X_IER_RLSI_BIT (1 << 2) /* Enable RX line status interrupt */
#define CH43X_IER_MSI_BIT  (1 << 3) /* Enable Modem status interrupt */

/* IER enhanced register bits */
#define CH43X_IER_RESET_BIT    (1 << 7) /* Enable Soft reset */
#define CH43X_IER_LOWPOWER_BIT (1 << 6) /* Enable low power mode */
#define CH43X_IER_SLEEP_BIT    (1 << 5) /* Enable sleep mode */
#define CH43X_IER_CK2X_BIT     (1 << 5) /* Enable clk * 2 */

/* FCR register bits */
#define CH43X_FCR_FIFO_BIT    (1 << 0) /* Enable FIFO */
#define CH43X_FCR_RXRESET_BIT (1 << 1) /* Reset RX FIFO */
#define CH43X_FCR_TXRESET_BIT (1 << 2) /* Reset TX FIFO */
#define CH43X_FCR_RXLVLL_BIT  (1 << 6) /* RX Trigger level LSB */
#define CH43X_FCR_RXLVLH_BIT  (1 << 7) /* RX Trigger level MSB */

/* IIR register bits */
#define CH43X_IIR_NO_INT_BIT (1 << 0) /* No interrupts pending */
#define CH43X_IIR_ID_MASK    0x0e     /* Mask for the interrupt ID */
#define CH43X_IIR_THRI_SRC   0x02     /* TX holding register empty */
#define CH43X_IIR_RDI_SRC    0x04     /* RX data interrupt */
#define CH43X_IIR_RLSE_SRC   0x06     /* RX line status error */
#define CH43X_IIR_RTOI_SRC   0x0c     /* RX time-out interrupt */
#define CH43X_IIR_MSI_SRC    0x00     /* Modem status interrupt */

/* LCR register bits */
#define CH43X_LCR_LENGTH0_BIT     (1 << 0) /* Word length bit 0 */
#define CH43X_LCR_LENGTH1_BIT     (1 << 1)
#define CH43X_LCR_STOPLEN_BIT     (1 << 2)
#define CH43X_LCR_PARITY_BIT      (1 << 3) /* Parity bit enable */
#define CH43X_LCR_ODDPARITY_BIT   (0)      /* Odd parity bit enable */
#define CH43X_LCR_EVENPARITY_BIT  (1 << 4) /* Even parity bit enable */
#define CH43X_LCR_MARKPARITY_BIT  (1 << 5) /* Mark parity bit enable */
#define CH43X_LCR_SPACEPARITY_BIT (3 << 4) /* Space parity bit enable */

#define CH43X_LCR_TXBREAK_BIT (1 << 6) /* TX break enable */
#define CH43X_LCR_DLAB_BIT    (1 << 7) /* Divisor Latch enable */
#define CH43X_LCR_WORD_LEN_5  (0x00)
#define CH43X_LCR_WORD_LEN_6  (0x01)
#define CH43X_LCR_WORD_LEN_7  (0x02)
#define CH43X_LCR_WORD_LEN_8  (0x03)
#define CH43X_LCR_CONF_MODE_A CH43X_LCR_DLAB_BIT /* Special reg set */

/* MCR register bits */
#define CH43X_MCR_DTR_BIT  (1 << 0) /* DTR complement */
#define CH43X_MCR_RTS_BIT  (1 << 1) /* RTS complement */
#define CH43X_MCR_OUT1     (1 << 2) /* OUT1 */
#define CH43X_MCR_OUT2     (1 << 3) /* OUT2 */
#define CH43X_MCR_LOOP_BIT (1 << 4) /* Enable loopback test mode */
#define CH43X_MCR_AFE      (1 << 5) /* Enable Hardware Flow control */

/* LSR register bits */
#define CH43X_LSR_DR_BIT         (1 << 0) /* Receiver data ready */
#define CH43X_LSR_OE_BIT         (1 << 1) /* Overrun Error */
#define CH43X_LSR_PE_BIT         (1 << 2) /* Parity Error */
#define CH43X_LSR_FE_BIT         (1 << 3) /* Frame Error */
#define CH43X_LSR_BI_BIT         (1 << 4) /* Break Interrupt */
#define CH43X_LSR_BRK_ERROR_MASK 0x1E     /* BI, FE, PE, OE bits */
#define CH43X_LSR_THRE_BIT       (1 << 5) /* TX holding register empty */
#define CH43X_LSR_TEMT_BIT       (1 << 6) /* Transmitter empty */
#define CH43X_LSR_FIFOE_BIT      (1 << 7) /* Fifo Error */

/* MSR register bits */
#define CH43X_MSR_DCTS_BIT   (1 << 0) /* Delta CTS Clear To Send */
#define CH43X_MSR_DDSR_BIT   (1 << 1) /* Delta DSR Data Set Ready */
#define CH43X_MSR_DRI_BIT    (1 << 2) /* Delta RI Ring Indicator */
#define CH43X_MSR_DCD_BIT    (1 << 3) /* Delta CD Carrier Detect */
#define CH43X_MSR_CTS_BIT    (1 << 4) /* CTS */
#define CH43X_MSR_DSR_BIT    (1 << 5) /* DSR */
#define CH43X_MSR_RI_BIT     (1 << 6) /* RI */
#define CH43X_MSR_CD_BIT     (1 << 7) /* CD */
#define CH43X_MSR_DELTA_MASK 0x0F     /* Any of the delta bits! */

/* Misc definitions */
#define CH43X_FIFO_SIZE (16)
#define CH43X_REG_SHIFT 2

struct ch43x_devtype {
    char name[10];
    int nr_uart;
};

struct ch43x_one {
    struct uart_port port;
    struct work_struct tx_work;
    struct work_struct md_work;
    struct work_struct stop_rx_work;
    struct work_struct stop_tx_work;
    struct serial_rs485 rs485;
    unsigned char msr_reg;
    unsigned char ier;
    unsigned char mcr_force;
};

struct ch43x_port {
    struct uart_driver uart;
    struct ch43x_devtype *devtype;
    struct mutex mutex;
    struct mutex mutex_bus_access;
    struct clk *clk;
    struct spi_device *spi_dev;
    unsigned char buf[65536];
    struct ch43x_one p[0];
};

#define to_ch43x_one(p, e) ((container_of((p), struct ch43x_one, e)))

extern const struct of_device_id __maybe_unused ch43x_dt_ids[];

extern int ch43x_spi_probe(struct spi_device *spi);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
extern void ch43x_spi_remove(struct spi_device *spi);
#else
extern int ch43x_spi_remove(struct spi_device *spi);
#endif

#endif
