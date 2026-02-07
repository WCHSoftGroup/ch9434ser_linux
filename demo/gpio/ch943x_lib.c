/*
 * GPIO application library of CH9434A/M/D/CH9438/CH9437/CH9432 chip, etc.
 *
 * Copyright (C) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web: http://wch.cn
 * Author: WCH <tech@wch.cn>
 *
 * V1.0 - initial version
 * V1.1 - add support CH9434D/CH9438/CH9437/CH9432
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "ch943x_lib.h"

#define BIT(i) (1 << i)

#define IOCTL_MAGIC            'W'
#define IOCTL_CMD_GPIOENABLE   _IOW(IOCTL_MAGIC, 0x80, uint16_t)
#define IOCTL_CMD_GPIODIR      _IOW(IOCTL_MAGIC, 0x81, uint16_t)
#define IOCTL_CMD_GPIOPULLUP   _IOW(IOCTL_MAGIC, 0x82, uint16_t)
#define IOCTL_CMD_GPIOPULLDOWN _IOW(IOCTL_MAGIC, 0x83, uint16_t)
#define IOCTL_CMD_GPIOSET      _IOW(IOCTL_MAGIC, 0x84, uint16_t)
#define IOCTL_CMD_GPIOGET      _IOWR(IOCTL_MAGIC, 0x85, uint16_t)

#define IOCTL_CMD_CH9434D_GPIOENABLE _IOWR(IOCTL_MAGIC, 0x86, uint16_t)
#define IOCTL_CMD_CH9438_GPIOENABLE  _IOWR(IOCTL_MAGIC, 0x90, uint16_t)
#define IOCTL_CMD_CH9437_GPIOENABLE  _IOWR(IOCTL_MAGIC, 0x91, uint16_t)
#define IOCTL_CMD_CH9432_GPIOENABLE  _IOWR(IOCTL_MAGIC, 0x92, uint16_t)

#define IOCTL_CTRL_WRITE      _IOWR(IOCTL_MAGIC, 0x87, uint16_t)
#define IOCTL_CTRL_READ       _IOWR(IOCTL_MAGIC, 0x88, uint16_t)
#define IOCTL_CMD_GETCHIPTYPE _IOWR(IOCTL_MAGIC, 0x89, uint16_t)

#define CH943D_GPIO_NUMS   4
#define CH943D_GPIO_GROUPS 1

#define CH9434_GPIO_FUNC_EN_0 (0x50)
#define CH9434_GPIO_DIR_MOD_0 (0x54)
#define CH9434_GPIO_SET_0     (0x5C)
#define CH9434_GPIO_RESET_0   (0x5E)
#define CH9434_GPIO_PIN_VAL_0 (0x60)

#define CH943X_GPIOEN_REG  (0x50) /* GPIO Enable Set */
#define CH943X_GPIODIR_REG (0x54) /* GPIO Direction Set */
#define CH943X_GPIOPU_REG  (0x58) /* GPIO PullUp Set */
#define CH943X_GPIOPD_REG  (0x5C) /* GPIO PullDown Set */
#define CH943X_GPIOVAL_REG (0x60) /* GPIO Value Set */

struct _ctrl_info {
    uint8_t cmd;
    uint32_t datalen;
    uint8_t data[0];
} __attribute__((packed));

static int ch943x_ctrl_write(int fd, uint8_t cmd, uint32_t len, uint8_t *data)
{
    struct _ctrl_info *ctrlinfo;
    int ret;

    ctrlinfo = malloc(sizeof(struct _ctrl_info) + len);
    if (!ctrlinfo) {
        printf("malloc error\n");
        return -ENOMEM;
    }

    ctrlinfo->cmd = cmd;
    ctrlinfo->datalen = len;
    memcpy(ctrlinfo->data, data, len);

    ret = ioctl(fd, IOCTL_CTRL_WRITE, (unsigned long)ctrlinfo);

    free(ctrlinfo);
    return ret;
}

static int ch943x_ctrl_read(int fd, uint8_t cmd, uint32_t len, uint8_t *data)
{
    struct _ctrl_info *ctrlinfo;
    int ret;

    ctrlinfo = malloc(sizeof(struct _ctrl_info) + len);
    if (!ctrlinfo) {
        printf("malloc error\n");
        return -ENOMEM;
    }

    ctrlinfo->cmd = cmd;
    ctrlinfo->datalen = len;

    ret = ioctl(fd, IOCTL_CTRL_READ, (unsigned long)ctrlinfo);

    memcpy(data, ctrlinfo->data, len);

    free(ctrlinfo);
    return ret;
}

static int is_gpio_number_valid(CHIPTYPE chiptype, uint8_t gpionumber)
{
    if ((chiptype == CHIP_CH9434D) && (gpionumber > 3))
        return -ENXIO;
    else if ((chiptype == CHIP_CH9432) && (gpionumber > 7))
        return -ENXIO;
    else if ((chiptype == CHIP_CH9438) && (gpionumber > 7))
        return -ENXIO;
    else if ((chiptype == CHIP_CH9437) && (gpionumber > 10))
        return -ENXIO;
    else if (((chiptype == CHIP_CH9434A) || (chiptype == CHIP_CH9434M)) && (gpionumber > 24))
        return -ENXIO;

    return 0;
}

/**
 * ch943x_get_chiptype - get chip type
 * @fd: file descriptor of ch943x GPIO device
 * @type: pointer to chip type
 *
 * The function return 0 if success, others if fail.
 */
int ch943x_get_chiptype(int fd, CHIPTYPE *type)
{
    int ret;
    ret = ioctl(fd, IOCTL_CMD_GETCHIPTYPE, type);
    return ret;
}

/**
 * ch943x_single_gpioenable - enable or disable the GPIO function
 * @fd: file descriptor of ch943x gpio device
 * @gpionumber: gpio number
 * @enable: 1 on enable, 0 on disable
 *
 * The function return 0 if success, others if fail.
 */
int ch943x_single_gpioenable(int fd, uint8_t gpionumber, uint8_t enable)
{
    uint32_t val1;
    uint8_t val2;
    int ret;
    CHIPTYPE chiptype;

    if (ch943x_get_chiptype(fd, &chiptype) < 0)
        return -ERROR_CODE3;

    ret = is_gpio_number_valid(chiptype, gpionumber);
    if (ret < 0)
        return ret;

    if (chiptype == CHIP_CH9434D) {
        val1 = (gpionumber << 8) | enable;
        ret = ioctl(fd, IOCTL_CMD_CH9434D_GPIOENABLE, &val1);
        if (ret < 0)
            return ret;
    } else if (chiptype == CHIP_CH9432) {
        val1 = (gpionumber << 8) | enable;
        ret = ioctl(fd, IOCTL_CMD_CH9432_GPIOENABLE, &val1);
        if (ret < 0)
            return ret;
    } else if (chiptype == CHIP_CH9438) {
        val1 = (gpionumber << 8) | enable;
        ret = ioctl(fd, IOCTL_CMD_CH9438_GPIOENABLE, &val1);
        if (ret < 0)
            return ret;
    } else if (chiptype == CHIP_CH9437) {
        val1 = (gpionumber << 8) | enable;
        ret = ioctl(fd, IOCTL_CMD_CH9437_GPIOENABLE, &val1);
        if (ret < 0)
            return ret;
    } else {
        if (ch943x_ctrl_read(fd, CH9434_GPIO_FUNC_EN_0 + (gpionumber / 8), 1, &val2))
            return -ERROR_CODE1;

        if (enable)
            val2 |= BIT(gpionumber % 8);
        else
            val2 &= ~BIT(gpionumber % 8);

        if (ch943x_ctrl_write(fd, CH9434_GPIO_FUNC_EN_0 + (gpionumber / 8), 1, &val2))
            return -ERROR_CODE1;
    }

    /* default high level */
    if ((chiptype == CHIP_CH9434D) || (chiptype == CHIP_CH9432) || (chiptype == CHIP_CH9438) ||
        (chiptype == CHIP_CH9437)) {
        if (ch943x_ctrl_read(fd, CH9434_GPIO_SET_0 + (gpionumber / 8), 1, &val2))
            return -ERROR_CODE1;
        val2 |= BIT(gpionumber % 8);
        if (ch943x_ctrl_write(fd, CH9434_GPIO_SET_0 + (gpionumber / 8), 1, &val2))
            return -ERROR_CODE1;
    }

    return ret;
}

/**
 * ch943x_single_gpioconfig - configure the direction and electrical properties of the GPIO.
 * @fd: file descriptor of ch943x gpio device
 * @gpionumber: gpio number
 * @dir: gpio direction, 1 on output, 0 on input
 * @mode: The electrical properties of the GPIO
 *       CH943X_IOMODE_PUSH_PULL:      Push–pull output
 *       CH943X_IOMODE_OPEN_DRAIN:     Floating open output
 *       CH943X_IOMODE_FLOAT_INPUT:    Floating input
 *       CH943X_IOMODE_PULLUP_INPUT:   Pull-up input
 *       CH943X_IOMODE_PULLDOWN_INPUT: Pull-down input
 *
 * The function return 0 if success, others if fail.
 */
int ch943x_single_gpioconfig(int fd, uint8_t gpionumber, IODir dir, IOMode mode)
{
    uint8_t val1 = 0, val2 = 0, val3 = 0;
    uint8_t val;
    int ret;
    int retval;
    CHIPTYPE chiptype;

    if (ch943x_get_chiptype(fd, &chiptype) < 0)
        return -ERROR_CODE3;

    ret = is_gpio_number_valid(chiptype, gpionumber);
    if (ret < 0)
        return ret;

    /* configure the direction and electrical properties */
    if ((chiptype == CHIP_CH9434D) || (chiptype == CHIP_CH9432) || (chiptype == CHIP_CH9438) ||
        (chiptype == CHIP_CH9437)) {
        if (ch943x_ctrl_read(fd, CH9434_GPIO_DIR_MOD_0 + (gpionumber / 2), 1, &val1))
            return -ERROR_CODE1;

        if ((gpionumber % 2) == 0) { /* GPIO0/2/4/6... */
            if (dir == OUTPUT) {
                val1 |= (BIT(0) | BIT(1));
                if (mode == CH943X_IOMODE_PUSH_PULL)
                    val1 &= ~(BIT(2) | BIT(3));
                else if (mode == CH943X_IOMODE_OPEN_DRAIN) {
                    val1 |= BIT(2);
                    val1 &= ~BIT(3);
                } else
                    return -ERROR_CODE5;
            } else if (dir == INPUT) {
                val1 &= ~(BIT(0) | BIT(1));
                if (mode == CH943X_IOMODE_FLOAT_INPUT) {
                    val1 |= BIT(2);
                    val1 &= ~BIT(3);
                } else if ((mode == CH943X_IOMODE_PULLUP_INPUT) || (mode == CH943X_IOMODE_PULLDOWN_INPUT)) {
                    val1 |= BIT(3);
                    val1 &= ~BIT(2);
                } else
                    return -ERROR_CODE5;
            } else
                return -ERROR_CODE4;
        } else { /* GPIO1/3/5/7... */
            if (dir == OUTPUT) {
                val1 |= (BIT(4) | BIT(5));
                if (mode == CH943X_IOMODE_PUSH_PULL)
                    val1 &= ~(BIT(6) | BIT(7));
                else if (mode == CH943X_IOMODE_OPEN_DRAIN) {
                    val1 |= BIT(6);
                    val1 &= ~BIT(7);
                } else
                    return -ERROR_CODE5;
            } else if (dir == INPUT) {
                val1 &= ~(BIT(4) | BIT(5));
                if (mode == CH943X_IOMODE_FLOAT_INPUT) {
                    val1 |= BIT(6);
                    val1 &= ~BIT(7);
                } else if ((mode == CH943X_IOMODE_PULLUP_INPUT) || (mode == CH943X_IOMODE_PULLDOWN_INPUT)) {
                    val1 |= BIT(7);
                    val1 &= ~BIT(6);
                } else
                    return -ERROR_CODE5;
            } else
                return -ERROR_CODE4;
        }

        if (dir == INPUT) {
            if (mode == CH943X_IOMODE_PULLUP_INPUT) {
                if (ch943x_ctrl_read(fd, CH9434_GPIO_SET_0 + (gpionumber / 8), 1, &val2))
                    return -ERROR_CODE1;

                val2 |= BIT(gpionumber % 8);
                if (ch943x_ctrl_write(fd, CH9434_GPIO_SET_0 + (gpionumber / 8), 1, &val2))
                    return -ERROR_CODE1;
            } else if (mode == CH943X_IOMODE_PULLDOWN_INPUT) {
                if (ch943x_ctrl_read(fd, CH9434_GPIO_RESET_0 + (gpionumber / 8), 1, &val2))
                    return -ERROR_CODE1;

                val2 |= BIT(gpionumber % 8);
                if (ch943x_ctrl_write(fd, CH9434_GPIO_RESET_0 + (gpionumber / 8), 1, &val2))
                    return -ERROR_CODE1;
            } else if (mode == CH943X_IOMODE_FLOAT_INPUT) {
            } else {
                return -ERROR_CODE5;
            }
        }

        if (ch943x_ctrl_write(fd, CH9434_GPIO_DIR_MOD_0 + (gpionumber / 2), 1, &val1))
            return -ERROR_CODE1;
    } else {
        if (ch943x_ctrl_read(fd, CH9434_GPIO_DIR_MOD_0 + (gpionumber / 8), 1, &val3))
            return -ERROR_CODE1;

        if (dir == INPUT)
            val3 &= ~BIT(gpionumber % 8);
        else if (dir == OUTPUT)
            val3 |= BIT(gpionumber % 8);
        else
            return -ERROR_CODE4;

        if (ch943x_ctrl_write(fd, CH9434_GPIO_DIR_MOD_0 + (gpionumber / 8), 1, &val3))
            return -ERROR_CODE1;

        if (dir == INPUT) {
            if (mode == CH943X_IOMODE_PULLUP_INPUT) {
                if (ch943x_ctrl_read(fd, CH943X_GPIOPD_REG + (gpionumber / 8), 1, &val))
                    return -ERROR_CODE1;
                val &= ~BIT(gpionumber % 8);
                if (ch943x_ctrl_write(fd, CH943X_GPIOPD_REG + (gpionumber / 8), 1, &val))
                    return -ERROR_CODE1;

                if (ch943x_ctrl_read(fd, CH943X_GPIOPU_REG + (gpionumber / 8), 1, &val))
                    return -ERROR_CODE1;
                val |= BIT(gpionumber % 8);
                if (ch943x_ctrl_write(fd, CH943X_GPIOPU_REG + (gpionumber / 8), 1, &val))
                    return -ERROR_CODE1;
            } else if (mode == CH943X_IOMODE_PULLDOWN_INPUT) {
                if (ch943x_ctrl_read(fd, CH943X_GPIOPU_REG + (gpionumber / 8), 1, &val))
                    return -ERROR_CODE1;
                val &= ~BIT(gpionumber % 8);
                if (ch943x_ctrl_write(fd, CH943X_GPIOPU_REG + (gpionumber / 8), 1, &val))
                    return -ERROR_CODE1;

                if (ch943x_ctrl_read(fd, CH943X_GPIOPD_REG + (gpionumber / 8), 1, &val))
                    return -ERROR_CODE1;
                val |= BIT(gpionumber % 8);
                if (ch943x_ctrl_write(fd, CH943X_GPIOPD_REG + (gpionumber / 8), 1, &val))
                    return -ERROR_CODE1;
            } else if (mode == CH943X_IOMODE_FLOAT_INPUT) {
                if (ch943x_ctrl_read(fd, CH943X_GPIOPU_REG + (gpionumber / 8), 1, &val))
                    return -ERROR_CODE1;
                val &= ~BIT(gpionumber % 8);
                if (ch943x_ctrl_write(fd, CH943X_GPIOPU_REG + (gpionumber / 8), 1, &val))
                    return -ERROR_CODE1;

                if (ch943x_ctrl_read(fd, CH943X_GPIOPD_REG + (gpionumber / 8), 1, &val))
                    return -ERROR_CODE1;
                val &= ~BIT(gpionumber % 8);
                if (ch943x_ctrl_write(fd, CH943X_GPIOPD_REG + (gpionumber / 8), 1, &val))
                    return -ERROR_CODE1;
            } else {
                return -ERROR_CODE5;
            }
        }
    }

    return 0;
}

/**
 * ch943x_single_gpioset - Configure the output level of the GPIO
 * @fd: file descriptor of ch943x gpio device
 * @gpionumber: gpio number
 * @value: 1 on high, 0 on low
 *
 * The function return 0 if success, others if fail.
 */
int ch943x_single_gpioset(int fd, uint8_t gpionumber, uint8_t value)
{
    uint8_t val = 0;
    int retval;
    int ret;
    CHIPTYPE chiptype;

    if (ch943x_get_chiptype(fd, &chiptype) < 0)
        return -ERROR_CODE3;

    ret = is_gpio_number_valid(chiptype, gpionumber);
    if (ret < 0)
        return ret;

    if ((chiptype == CHIP_CH9434D) || (chiptype == CHIP_CH9432) || (chiptype == CHIP_CH9438) ||
        (chiptype == CHIP_CH9437)) {
        if (value) {
            val = BIT(gpionumber % 8);
            if (ch943x_ctrl_write(fd, CH9434_GPIO_SET_0 + (gpionumber / 8), 1, &val))
                return -ERROR_CODE1;
        } else {
            val = BIT(gpionumber % 8);
            if (ch943x_ctrl_write(fd, CH9434_GPIO_RESET_0 + (gpionumber / 8), 1, &val) < 0)
                return -ERROR_CODE1;
        }
    } else {
        if (ch943x_ctrl_read(fd, CH943X_GPIOVAL_REG + (gpionumber / 8), 1, &val))
            return -ERROR_CODE1;

        if (value)
            val |= BIT(gpionumber % 8);
        else
            val &= ~BIT(gpionumber % 8);

        if (ch943x_ctrl_write(fd, CH943X_GPIOVAL_REG + (gpionumber / 8), 1, &val))
            return -ERROR_CODE1;
    }

    return 0;
}

/**
 * ch943x_single_gpioget - Get the input level value of the GPIO
 * @fd: file descriptor of ch943x gpio device
 * @gpionumber: gpio number
 *
 * The function return the level value of the GPIO, 1 on high, 0 on low, others if fail.
 */
int ch943x_single_gpioget(int fd, uint8_t gpionumber)
{
    uint8_t val = 0;
    int retval;
    int ret;
    CHIPTYPE chiptype;

    if (ch943x_get_chiptype(fd, &chiptype) < 0)
        return -ERROR_CODE3;

    ret = is_gpio_number_valid(chiptype, gpionumber);
    if (ret < 0)
        return ret;

    if ((chiptype == CHIP_CH9434D) || (chiptype == CHIP_CH9432) || (chiptype == CHIP_CH9438) ||
        (chiptype == CHIP_CH9437)) {
        if (ch943x_ctrl_read(fd, CH9434_GPIO_PIN_VAL_0 + (gpionumber / 8), 1, &val) < 0)
            return -ERROR_CODE1;
        retval = (val & BIT(gpionumber % 8)) > 0 ? 1 : 0;
    } else {
        if (ch943x_ctrl_read(fd, CH943X_GPIOVAL_REG + (gpionumber / 8), 1, &val) < 0)
            return -ERROR_CODE1;
        retval = (val & BIT(gpionumber % 8)) > 0 ? 1 : 0;
    }

    return retval;
}
