// SPDX-License-Identifier: GPL-2.0+
/*
 * SPI/I2C/UART to SERIAL/CAN/GPIO multifunction driver for chip CH9434/CH9438/CH9437/CH9432, etc.
 *
 * Copyright (C) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web:      http://wch.cn
 * Author:   WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Update Log:
 * V1.0 - initial version
 * V1.1 - change RS485 setting and getting method, add procfs for dump registers
 *      - add support for CH9434A with continuous spi transmission
 * V1.2 - add support for kernel version beyond 5.12.18
 * V1.3 - improve spi transfer compatibility on some platforms
 * V1.4 - add support for CH9434D
 * V1.5 - add support for CH9438F/CH9437F/CH9432F
 * V1.6 - modify uart driver's log output format
 *        modify the names of some files
 * V1.7 - abstract SPI/I2C/serial bus operations
 *        add GPIO sysfs interface support
 *        modify driver debug/log output mechanism
 *        improve resource management and error handling paths
 */

#include "ch943x.h"

#ifndef PORT_SC16IS7XX
#define PORT_SC16IS7XX 128
#endif

#define CH943X_MAX_CNT      8
#define CH943X_GPIODRV_NAME "ch943x_io"

static struct ch943x *ch943x_table[CH943X_MAX_CNT];
static struct cdev ch943x_cdev;
static struct class *ch943x_io_class;
static dev_t devt;
static int ch943x_major = 0x00;

static DEFINE_MUTEX(ch943x_minors_lock);

int drv_debug_enable = 0;
module_param(drv_debug_enable, int, 0644);
MODULE_PARM_DESC(drv_debug_enable, "Enable driver debug output (0=off, 1=on)");

const int ch943x_tnow_enable[8] = {
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

static struct ch943x *ch943x_get_by_index(unsigned int index)
{
    struct ch943x *ch943x;

    mutex_lock(&ch943x_minors_lock);
    ch943x = ch943x_table[index];
    mutex_unlock(&ch943x_minors_lock);

    return ch943x;
}

static int ch943x_alloc_minor(struct ch943x *ch943x)
{
    int minor;

    mutex_lock(&ch943x_minors_lock);
    for (minor = 0; minor < CH943X_MAX_CNT; minor++) {
        if (!ch943x_table[minor]) {
            ch943x_table[minor] = ch943x;
            break;
        }
    }
    mutex_unlock(&ch943x_minors_lock);

    if (minor >= CH943X_MAX_CNT)
        return -ENOSR;

    return minor;
}

static void ch943x_release_minor(struct ch943x *ch943x)
{
    mutex_lock(&ch943x_minors_lock);
    ch943x_table[ch943x->minor] = NULL;
    mutex_unlock(&ch943x_minors_lock);
}

static int ch943x_io_open(struct inode *inode, struct file *fp)
{
    unsigned int minor = iminor(inode);
    struct ch943x *s = ch943x_get_by_index(minor);

    if (!s)
        return -ENODEV;

    fp->private_data = s;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    return 0;
}

static int ch943x_io_release(struct inode *inode, struct file *fp)
{
    return 0;
}

static long ch943x_io_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct ch943x *s = file->private_data;
    int ret;

    ret = ch943x_ioctl_do(s, cmd, arg);
    if (ret < 0) {
        dev_err(s->dev, "ch943x_ioctl_do failed. cmd:%u arg:%lu ret:%d\n", cmd, arg, ret);
        return ret;
    }

    return 0;
}

static irqreturn_t ch943x_ist_top(int irq, void *dev_id)
{
    return IRQ_WAKE_THREAD;
}

static irqreturn_t ch943x_ist(int irq, void *dev_id)
{
    struct ch943x *s = (struct ch943x *)dev_id;
    int i;

    DRV_DEBUG(s->dev, "%s interrupt enter...\n", __func__);

    if ((s->chip.chiptype == CHIP_CH9438F) || ((s->chip.chiptype == CHIP_CH9437F) && IS_USE_SERIAL_MODE) ||
        ((s->chip.chiptype == CHIP_CH9434D) && IS_USE_SPI_MODE && ((s->chip.ver[1] == 1) && (s->chip.ver[0] == 1))) ||
        ((s->chip.chiptype == CHIP_CH9432D) && IS_USE_SPI_MODE)) {
        ch943x_port_irq_bulkmode(s);
    } else {
        for (i = 0; i < s->uart.nr; ++i)
            ch943x_port_irq(s, i);
    }

#ifdef CH9434D_CAN_ON
    if ((s->chip.chiptype == CHIP_CH9434D) && s->can_on) {
        if (atomic_read(&s->priv->can_isopen) == 1) {
            ch943x_can_irq(s);
        }
    }
#endif

    DRV_DEBUG(s->dev, "%s end\n", __func__);
    return IRQ_HANDLED;
}

static int ch943x_clock_init(struct ch943x *s)
{
    u8 data;
    int ret;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    if (s->chip.chiptype == CHIP_CH9434D || s->chip.chiptype == CHIP_CH9432D || s->chip.chiptype == CHIP_CH9438F ||
        s->chip.chiptype == CHIP_CH9437F) {
        if (s->extern_clock_on) {
            data = 0x03 << 6;
            ret = ch943x_reg_write(s, CH943X_CLK_REG, 1, &data);
            if (ret < 0)
                return ret;
        } else {
            data = 0;
            ret = ch943x_reg_write(s, CH943X_CLK_REG, 1, &data);
            if (ret < 0)
                return ret;
        }
    } else {
        data = CH943X_CLK_EXT_BIT | CH943X_CLK_PLL_BIT | 13;
        ret = ch943x_reg_write(s, CH943X_CLK_REG, 1, &data);
        if (ret < 0)
            return ret;
    }
    mdelay(200);

    return 0;
}

static int ch943x_hw_test(struct ch943x *s)
{
    u8 val, data;
    int i, ret;

    DRV_DEBUG(s->dev, "%s\n", __func__);

    for (i = 0; i < 2; i++) {
        data = 0x55;
        ret = ch943x_reg_write(s, (CH943X_SPR_REG + (i * 0x10)), 1, &data);
        if (ret < 0)
            return ret;
        ret = ch943x_reg_read(s, CH943X_SPR_REG + (i * 0x10), 1, &val);
        if (ret < 0)
            return ret;
        if (val != data)
            return -EIO;

        data = 0xAA;
        ret = ch943x_reg_write(s, (CH943X_SPR_REG + (i * 0x10)), 1, &data);
        if (ret < 0)
            return ret;
        ret = ch943x_reg_read(s, CH943X_SPR_REG + (i * 0x10), 1, &val);
        if (ret < 0)
            return ret;
        if (val != data)
            return -EIO;
    }
    return 0;
}

#if defined(USE_SERIAL_MODE)
static int ctrluart_init(struct ch943x *s)
{
    int ret;
    u8 data;

#ifdef MULTI_CHIP_MODE
    /**
     * Distinguish the control serial ports based on the id field of the platform_device.
     * SERIAL mode example for adapting to multi-chip(CH9437):
     *
     * if (s->pdev->id == 0) {
     *     snprintf(s->ctrluart_path, sizeof(s->ctrluart_path), "ttyS0");
     *     s->ctrluart_baud = 4000000;
     * } else if (s->pdev->id == 1) {
     *     snprintf(s->ctrluart_path, sizeof(s->ctrluart_path), "ttyS1");
     *     s->ctrluart_baud = 2000000;
     * } else if (s->pdev->id == 2) {
     *      ...
     * }
     *
     * Only after adding the above code can the following filp_open function be executed.
     */
    s->fp = filp_open(s->ctrluart_path, O_RDWR, 0);
    if (IS_ERR(s->fp)) {
        dev_err(s->dev, "uart %s open failed!\n", s->ctrluart_path);
        ret = PTR_ERR(s->fp);
        return ret;
    }
#else
    s->fp = filp_open(CTRLUART_PATH, O_RDWR, 0);
    if (IS_ERR(s->fp)) {
        ret = PTR_ERR(s->fp);
        return ret;
    }
#endif /* MULTI_CHIP_MODE */

    /**
     * If the Linux kernel version is greater than 5.17,
     * then the serial port application must be used to configure
     * parameters such as the baud rate of the serial port.
     */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
    ret = ch943x_ctrluart_setopt(s);
    if (ret < 0) {
#ifdef MULTI_CHIP_MODE
        dev_err(s->dev, "set uart(%s) Failed.\n", s->ctrluart_path);
#else
        dev_err(s->dev, "set uart(%s) Failed.\n", CTRLUART_PATH);
#endif
        goto out;
    }
#endif

    data = 0x55;
    ret = ch943x_ctrl_tty_write(s, 1, &data);
    if (ret < 0)
        goto out;

    mdelay(100);

    return 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
out:
#endif
    filp_close(s->fp, NULL);
    return ret;
}
#endif

static int ch943x_probe(struct device *dev, int irq)
{
    int ret, i;
    struct ch943x *s;
    struct ch943x_one *p;
    struct device *class_dev = NULL;

    DRV_DEBUG(dev, "%s irq:%d\n", __func__, irq);

    /* Alloc port structure */
    s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
    if (!s) {
        dev_err(dev, "Error allocating port structure\n");
        return -ENOMEM;
    }
    dev_set_drvdata(dev, s);

    s->dev = dev;
    s->minor = ch943x_alloc_minor(s);
    if (s->minor < 0) {
        dev_err(dev, "Failed to allocate minor: %d\n", s->minor);
        return s->minor;
    }

    s->irq = irq;
    s->reg485 = 0x00;
    s->local_buf = devm_kmalloc(dev, 4096, GFP_KERNEL);
    if (!s->local_buf) {
        ret = -ENOMEM;
        goto release_minor;
    }
    s->ops = &ch943x_bus_ops;

#ifdef USE_SPI_MODE
    s->spi_dev = to_spi_device(dev);
    s->chip.interface_mode = SPI_MODE;
#elif defined(USE_I2C_MODE)
    s->i2c = to_i2c_client(dev);
    s->chip.interface_mode = I2C_MODE;
#elif defined(USE_SERIAL_MODE)
    s->pdev = to_platform_device(dev);
    s->chip.interface_mode = SERIAL_MODE;
    ret = ctrluart_init(s);
    if (ret < 0) {
        dev_err(s->dev, "ctrluart init Failed.\n");
        goto out1;
    }
#endif
    mutex_init(&s->mutex);
    mutex_init(&s->mutex_bus_access);

    ret = ch943x_hw_test(s);
    if (ret < 0) {
        dev_err(s->dev, "Hardware transfer test Failed.\n");
        goto out1;
    }

    ret = ch943x_get_chip_version(s);
    if (ret < 0) {
        dev_err(dev, "ch943x get chip version Failed.\n");
        goto out1;
    }

    /* CH9434D/CH9438/CH9437/CH9432 IO enable */
    ret = ch943x_io_enable(s);
    if (ret < 0) {
        dev_err(s->dev, "ch943x enable io Failed.\n");
        goto out1;
    }

    /* Init Clock */
    ret = ch943x_clock_init(s);
    if (ret < 0) {
        dev_err(s->dev, "ch943x init clock Failed.\n");
        goto out1;
    }

    /* Register UART driver */
    ret = ch943x_register_uart_driver(s);
    if (ret < 0) {
        dev_err(dev, "ch943x register uart_driver Failed.\n");
        goto out1;
    }

    /* Register UART port */
    ret = ch943x_register_uart_port(s);
    if (ret < 0) {
        dev_err(dev, "ch943x register uart_port Failed.\n");
        goto out2;
    }

#ifdef CH9434D_CAN_ON
    if ((s->chip.chiptype == CHIP_CH9434D) && s->can_on) {
        ret = ch943x_can_register(s);
        if (ret < 0) {
            dev_err(dev, "ch943x register can Failed.\n");
            goto out3;
        }
    }
#endif

    ret = ch943x_gpio_register(s);
    if (ret < 0) {
        dev_err(dev, "ch943x gpio probe Failed.\n");
        goto out4;
    }

    ret = devm_request_threaded_irq(dev, irq, ch943x_ist_top, ch943x_ist, IRQF_ONESHOT | IRQF_TRIGGER_LOW,
                                    dev_name(dev), s);
    if (ret != 0) {
        dev_err(dev, "irq %d request failed, error %d\n", irq, ret);
        goto out5;
    }

    class_dev = device_create(ch943x_io_class, s->dev, MKDEV(MAJOR(devt), s->minor), s, "ch943x_iodev%d", s->minor);
    if (IS_ERR(class_dev)) {
        ret = PTR_ERR(class_dev);
        dev_err(s->dev, "Could not create device node: %d\n", ret);
        goto out5;
    }
    dev_info(s->dev, "ch943x_iodev%d: character device\n", s->minor);

    ch943x_debugfs_init(s);

    return 0;

out5:
    ch943x_gpio_remove(s);
out4:
#ifdef CH9434D_CAN_ON
    if ((s->chip.chiptype == CHIP_CH9434D) && s->can_on) {
        ch943x_can_remove(s);
    }
out3:
#endif
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
out2:
    uart_unregister_driver(&s->uart);
out1:
    mutex_destroy(&s->mutex);
    mutex_destroy(&s->mutex_bus_access);
release_minor:
    ch943x_release_minor(s);

    return ret;
}

static int ch943x_remove(struct device *dev)
{
    struct ch943x *s = dev_get_drvdata(dev);

    DRV_DEBUG(dev, "%s\n", __func__);

    /*
     * Release interrupt first to ensure no IRQ thread is running
     * while we tear down UART ports and other resources below.
     * This MUST come before any other cleanup.
     */
    devm_free_irq(dev, s->irq, s);

    ch943x_uart_remove(s);
#ifdef CH9434D_CAN_ON
    if ((s->chip.chiptype == CHIP_CH9434D) && s->can_on) {
        ch943x_can_remove(s);
    }
#endif

    ch943x_gpio_remove(s);

#ifdef USE_SERIAL_MODE
    if (s->chip.chiptype == CHIP_CH9437F && IS_USE_SERIAL_MODE)
        filp_close(s->fp, NULL);
#endif
    mutex_destroy(&s->mutex);
    mutex_destroy(&s->mutex_bus_access);
    ch943x_debugfs_exit(s);

    device_destroy(ch943x_io_class, MKDEV(MAJOR(devt), s->minor));
    ch943x_release_minor(s);

    return 0;
}

static const struct of_device_id __maybe_unused ch943x_dt_ids[] = {
    {
     .compatible = "wch,ch943x",
     },
    {},
};
MODULE_DEVICE_TABLE(of, ch943x_dt_ids);

#ifdef USE_SPI_MODE
static int ch943x_spi_probe(struct spi_device *spi)
{
    struct device *dev = &spi->dev;
    int ret;

    spi->mode |= SPI_MODE_3;
    ret = spi_setup(spi);
    if (ret < 0) {
        dev_err(dev, "spi_setup failed, err=%d\n", ret);
        return ret;
    }

#ifdef USE_IRQ_FROM_DTS
    /* if your platform supports acquire irq number from dts */
    ret = ch943x_probe(dev, spi->irq);
    if (ret) {
        dev_err(dev, "ch943x_probe error\n");
        goto out;
    }
#else
    DRV_DEBUG(dev, "gpio_to_irq:%d\n", gpio_to_irq(GPIO_NUMBER));
    ret = devm_gpio_request(dev, GPIO_NUMBER, "gpioint");
    if (ret) {
        dev_err(dev, "Failed request gpio:%d\n", GPIO_NUMBER);
        goto out;
    }
    ret = gpio_direction_input(GPIO_NUMBER);
    if (ret) {
        dev_err(dev, "Failed set gpio direction\n");
        goto out;
    }

    ret = ch943x_probe(dev, gpio_to_irq(GPIO_NUMBER));
    if (ret) {
        dev_err(dev, "ch943x_probe error\n");
        goto out;
    }
#endif
out:
    return ret;
}
#endif

#ifdef USE_I2C_MODE
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
static int ch943x_i2c_probe(struct i2c_client *i2c)
#else
static int ch943x_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
#endif
{
    struct device *dev = &i2c->dev;
    int ret;

#ifdef USE_IRQ_FROM_DTS
    ret = ch943x_probe(dev, i2c->irq);
    if (ret) {
        dev_err(dev, "ch943x_probe error\n");
        goto out;
    }
#else
    ret = devm_gpio_request(dev, GPIO_NUMBER, "gpioint");
    if (ret) {
        dev_err(dev, "Failed request gpio:%d\n", GPIO_NUMBER);
        goto out;
    }
    ret = gpio_direction_input(GPIO_NUMBER);
    if (ret) {
        dev_err(dev, "Failed set gpio direction\n");
        goto out;
    }

    ret = ch943x_probe(dev, gpio_to_irq(GPIO_NUMBER));
    if (ret) {
        dev_err(dev, "ch943x_probe error\n");
        goto out;
    }
#endif
out:
    return ret;
}
#endif

#ifdef USE_SERIAL_MODE
static int ch943x_platform_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    int ret;
#ifdef MULTI_CHIP_MODE
    int gpio_number;
#endif

#ifdef USE_IRQ_FROM_DTS
    int irq;

    irq = platform_get_irq(pdev, 0);
    if (irq < 0) {
        dev_err(dev, "Failed to get IRQ: %d\n", irq);
        return irq;
    }
    DRV_DEBUG(dev, "irq:%d\n", irq);
    ret = ch943x_probe(dev, irq);
    if (ret) {
        dev_err(dev, "ch943x_probe error\n");
        goto out;
    }
#else
#ifdef MULTI_CHIP_MODE
    /**
     * Distinguish the control serial ports based on the id field of the platform_device.
     *
     * Example of specifying the GPIO_NUMBER corresponding to the designated interrupt IO:
     * if (pdev->id == 0) {
     *     gpio_number = 100;
     * } else if (pdev->id == 1) {
     *     gpio_number = 101;
     * } else if (pdev->id == 2) {
     *     ...
     * }
     *
     * Only after adding the above code can the following
     * devm_gpio_request function be executed.
     */
    ret = devm_gpio_request(dev, gpio_number, "gpioint");
    if (ret) {
        dev_err(dev, "Failed request gpio:%d\n", GPIO_NUMBER);
        goto out;
    }
    ret = gpio_direction_input(gpio_number);
    if (ret) {
        dev_err(dev, "Failed set gpio direction\n");
        goto out;
    }

    ret = ch943x_probe(dev, gpio_to_irq(gpio_number));
    if (ret) {
        dev_err(dev, "ch943x_probe error\n");
        goto out;
    }
#else
    DRV_DEBUG(dev, "gpio_to_irq:%d\n", gpio_to_irq(GPIO_NUMBER));
    ret = devm_gpio_request(dev, GPIO_NUMBER, "gpioint");
    if (ret) {
        dev_err(dev, "gpio request\n");
        goto out;
    }
    ret = gpio_direction_input(GPIO_NUMBER);
    if (ret) {
        dev_err(dev, "Failed set gpio direction\n");
        goto out;
    }

    ret = ch943x_probe(dev, gpio_to_irq(GPIO_NUMBER));
    if (ret) {
        dev_err(dev, "ch943x_probe error\n");
        goto out;
    }
#endif /* MULTI_CHIP_MODE */
#endif /* USE_IRQ_FROM_DTS */
out:
    return ret;
}
#endif

#ifdef USE_SPI_MODE
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
static void ch943x_spi_remove(struct spi_device *spi)
#else
static int ch943x_spi_remove(struct spi_device *spi)
#endif
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
    ch943x_remove(&spi->dev);
#else
    return ch943x_remove(&spi->dev);
#endif
}
#elif defined(USE_I2C_MODE)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void ch943x_i2c_remove(struct i2c_client *client)
#else
static int ch943x_i2c_remove(struct i2c_client *client)
#endif
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
    ch943x_remove(&client->dev);
#else
    return ch943x_remove(&client->dev);
#endif
}
#elif defined(USE_SERIAL_MODE)
static int ch943x_platform_remove(struct platform_device *pdev)
{
    ch943x_remove(&pdev->dev);
    return 0;
}
#endif

static int ch943x_suspend(struct device *dev)
{
    dev_info(dev, "%s\n", __func__);
    return 0;
}

static int ch943x_resume(struct device *dev)
{
    dev_info(dev, "%s\n", __func__);
    return 0;
}

static struct dev_pm_ops ch943x_pm_ops = {
    .suspend = ch943x_suspend,
    .resume = ch943x_resume,
};

#ifdef USE_SPI_MODE
static struct spi_driver ch943x_spi_driver = {
    .driver = {
        .name = "ch943x",
        .bus = &spi_bus_type,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(ch943x_dt_ids),
        .pm = &ch943x_pm_ops,
    },
    .probe = ch943x_spi_probe,
    .remove = ch943x_spi_remove,
};
MODULE_ALIAS("spi:ch943x");
#elif defined(USE_I2C_MODE)
static struct i2c_driver ch943x_i2c_driver = {
    .driver = {
       .name = "ch943x",
       .owner = THIS_MODULE,
       .of_match_table = of_match_ptr(ch943x_dt_ids),
       .pm = &ch943x_pm_ops,
    },
    .probe = ch943x_i2c_probe,
    .remove = ch943x_i2c_remove,
};
MODULE_ALIAS("i2c:ch943x");
#elif defined(USE_SERIAL_MODE)
static struct platform_driver ch943x_platform_driver = {
    .driver = {
        .name = "ch943x",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(ch943x_dt_ids),
        .pm = &ch943x_pm_ops,
    },
    .probe = ch943x_platform_probe,
    .remove = ch943x_platform_remove,
};
#endif

static const struct file_operations ch943x_io_fops = {
    .owner = THIS_MODULE,
    .open = ch943x_io_open,
    .release = ch943x_io_release,
    .unlocked_ioctl = ch943x_io_ioctl,
};

static int __init ch943x_init(void)
{
    int ret = 0;
    pr_info(KBUILD_MODNAME ": " DRIVER_DESC "\n");
    pr_info(KBUILD_MODNAME ": " VERSION_DESC "\n");

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0))
    ch943x_io_class = class_create(THIS_MODULE, "ch943x_io_class");
#else
    ch943x_io_class = class_create("ch943x_io_class");
#endif
    if (IS_ERR(ch943x_io_class)) {
        return PTR_ERR(ch943x_io_class);
    }

    ret = alloc_chrdev_region(&devt, 0, CH943X_MAX_CNT, CH943X_GPIODRV_NAME);
    if (ret)
        goto destroy_class;

    ch943x_major = MAJOR(devt);
    cdev_init(&ch943x_cdev, &ch943x_io_fops);
    ret = cdev_add(&ch943x_cdev, MKDEV(ch943x_major, 0), CH943X_MAX_CNT);
    if (ret)
        goto unregister_chrdev;

#ifdef USE_SPI_MODE
    ret = spi_register_driver(&ch943x_spi_driver);
#elif defined(USE_I2C_MODE)
    ret = i2c_add_driver(&ch943x_i2c_driver);
#elif defined(USE_SERIAL_MODE)
    ret = platform_driver_register(&ch943x_platform_driver);
#endif
    if (ret)
        goto cdev_del;

    return 0;

cdev_del:
    cdev_del(&ch943x_cdev);
unregister_chrdev:
    unregister_chrdev_region(MKDEV(ch943x_major, 0), CH943X_MAX_CNT);
destroy_class:
    class_destroy(ch943x_io_class);

    return ret;
}

static void __exit ch943x_exit(void)
{
    pr_info(KBUILD_MODNAME ": ch943x driver exit.\n");

#ifdef USE_SPI_MODE
    spi_unregister_driver(&ch943x_spi_driver);
#elif defined(USE_I2C_MODE)
    i2c_del_driver(&ch943x_i2c_driver);
#elif defined(USE_SERIAL_MODE)
    platform_driver_unregister(&ch943x_platform_driver);
#endif

    unregister_chrdev_region(MKDEV(ch943x_major, 0), CH943X_MAX_CNT);
    class_destroy(ch943x_io_class);
    cdev_del(&ch943x_cdev);
}

module_init(ch943x_init);
module_exit(ch943x_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
