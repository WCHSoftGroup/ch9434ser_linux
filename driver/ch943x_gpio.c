#include "ch943x.h"

bool gpio_is_available(struct ch943x *s, unsigned int offset)
{
    int i;

    if (offset >= s->chip.nr_gpio) {
        dev_err(s->dev, "%s - GPIO%d is out of range.\n", __func__, offset);
        return false;
    }

    switch (s->chip.chiptype) {
    case CHIP_CH9434A:
    case CHIP_CH9434M:
        break;
    case CHIP_CH9434D:
        if ((((offset == 0) || (offset == 1)) && s->can_on) || (offset == 3 && CH943X_EXCLK_ENABLE) ||
            (CH943X_TNOW_ENABLE(offset))) {
            return false;
        }
        break;
    case CHIP_CH9432D:
        if (((offset == 5) && CH943X_TNOW_ENABLE(0)) || ((offset == 6) && CH943X_TNOW_ENABLE(1)) ||
            ((offset == 7) && CH943X_EXCLK_ENABLE)) {
            return false;
        }
        break;
    case CHIP_CH9438F:
        if ((offset == 1) || ((offset == 7) && CH943X_EXCLK_ENABLE) || CH943X_TNOW_ENABLE(offset)) {
            return false;
        }
        break;
    case CHIP_CH9437F:
        if (((offset == 0) && (s->chip.interface_mode == I2C_MODE)) || ((offset == 8) && CH943X_EXCLK_ENABLE) ||
            ((offset == 9) && (s->chip.interface_mode == I2C_MODE)) || (offset == 10)) {
            return false;
        }
        for (i = 1; i < s->chip.nr_uart; i++) {
            if ((offset == i) && CH943X_TNOW_ENABLE(offset - 1))
                return false;
        }
        break;
    default:
        break;
    }

    return true;
}

int gpio_muxfunc_enable(struct ch943x *s, unsigned int offset)
{
    int ret;

    switch (s->chip.chiptype) {
    case CHIP_CH9434A:
    case CHIP_CH9434M:
        break;
    case CHIP_CH9434D:
        switch (offset) {
        case 0:
            ret = ch943x_iofunc_get(s, CH943X_IO_DEF_R_EN, CH9434D_DEF_CTS0_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9434D_DEF_CTS0_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 1:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH9434D_MUL_RTS0_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9434D_MUL_RTS0_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 2:
            ret = ch943x_iofunc_get(s, CH943X_IO_DEF_R_EN, CH9434D_DEF_CTS3_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9434D_DEF_CTS3_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        default:
            break;
        }

        ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH9434D_MUL_GPIO0_ADD + offset);
        if (ret < 0)
            return ret;
        if (ret == DISABLE) {
            ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9434D_MUL_GPIO0_ADD + offset, ENABLE);
            if (ret < 0)
                return ret;
        }
        break;
    case CHIP_CH9432D:
        switch (offset) {
        case 0:
            ret = ch943x_iofunc_get(s, CH943X_IO_DEF_R_EN, CH9432_DEF_CTS0_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9432_DEF_CTS0_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 1:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH9432_MUL_RTS0_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9432_MUL_RTS0_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 2:
            ret = ch943x_iofunc_get(s, CH943X_IO_DEF_R_EN, CH9432_DEF_CTS1_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_DEF_W_EN, CH9432_DEF_CTS1_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 3:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH9432_MUL_RTS1_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9432_MUL_RTS1_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        default:
            break;
        }

        ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH9432_MUL_GPIO0_ADD + offset);
        if (ret < 0)
            return ret;
        if (ret == DISABLE) {
            ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH9432_MUL_GPIO0_ADD + offset, ENABLE);
            if (ret < 0)
                return ret;
        }
        break;
    case CHIP_CH9438F:
        switch (offset) {
        case 0:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS1_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS1_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 2:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS2_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS2_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 3:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS2_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS2_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 4:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS3_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS3_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 5:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS3_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS3_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 6:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS4_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS4_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 7:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS4_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS4_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        default:
            break;
        }

        ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_GPIO0_ADD + offset);
        if (ret < 0)
            return ret;
        if (ret == DISABLE) {
            ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_GPIO0_ADD + offset, ENABLE);
            if (ret < 0)
                return ret;
        }
        break;
    case CHIP_CH9437F:
        switch (offset) {
        case 1:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS0_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS0_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 2:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS0_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS0_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 3:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS2_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS2_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 4:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS2_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS2_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 5:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS3_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS3_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 6:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS3_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS3_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 7:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_RTS4_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_RTS4_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        case 8:
            ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_CTS4_ADD);
            if (ret < 0)
                return ret;
            if (ret == ENABLE) {
                ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_CTS4_ADD, DISABLE);
                if (ret < 0)
                    return ret;
            }
            break;
        default:
            break;
        }

        ret = ch943x_iofunc_get(s, CH943X_IO_MULTI_R_EN, CH943X_MUL_GPIO0_ADD + offset);
        if (ret < 0)
            return ret;
        if (ret == DISABLE) {
            ret = ch943x_iofunc_set(s, CH943X_IO_MULTI_W_EN, CH943X_MUL_GPIO0_ADD + offset, ENABLE);
            if (ret < 0)
                return ret;
        }
        break;
    default:
        break;
    }

    return 0;
}

int gpio_reg_enable(struct ch943x *s, unsigned int offset, bool enable)
{
    u8 val = 0;
    u8 bit = BIT(offset % 8);

    ch943x_reg_read(s, CH9434_GPIO_FUNC_EN_0 + (offset / 8), 1, &val);
    if (enable)
        val |= bit;
    else
        val &= ~bit;
    return ch943x_reg_write(s, CH9434_GPIO_FUNC_EN_0 + (offset / 8), 1, &val);
}

static int ch943x_gpio_request(struct gpio_chip *gc, unsigned offset)
{
    struct ch943x *s = gpiochip_get_data(gc);
    bool enable = true;
    int ret = 0;

    DRV_DEBUG(s->dev, "%s offset:%u\n", __func__, offset);

    switch (s->chip.chiptype) {
    case CHIP_CH9434A:
    case CHIP_CH9434M:
        ret = gpio_reg_enable(s, offset, enable);
        if (ret < 0)
            goto out;
        break;
    case CHIP_CH9434D:
    case CHIP_CH9432D:
    case CHIP_CH9438F:
    case CHIP_CH9437F:
        if (gpio_is_available(s, offset) == false) {
            dev_err(s->dev, "%s - GPIO%d is unavailable.\n", __func__, offset);
            ret = -EBUSY;
            goto out;
        }

        ret = gpio_muxfunc_enable(s, offset);
        if (ret < 0)
            goto out;
        ret = gpio_reg_enable(s, offset, enable);
        if (ret < 0)
            goto out;
        break;
    }
out:
    return ret;
}

static void ch943x_gpio_free(struct gpio_chip *gc, unsigned int offset)
{
    struct ch943x *s = gpiochip_get_data(gc);
    bool disable = false;

    DRV_DEBUG(s->dev, "%s offset:%u\n", __func__, offset);

    gpio_reg_enable(s, offset, disable);
}

static int ch943x_gpio_dir_input(struct gpio_chip *gc, unsigned int offset)
{
    struct ch943x *s = gpiochip_get_data(gc);
    u8 val;
    u8 bit = BIT(offset % 8);
    int ret;

    DRV_DEBUG(s->dev, "%s offset:%u\n", __func__, offset);

    switch (s->chip.chiptype) {
    case CHIP_CH9434A:
    case CHIP_CH9434M:
        ret = ch943x_reg_update(s, CH9434_GPIO_DIR_MOD_0 + (offset / 8), bit, 0);
        if (ret < 0)
            return ret;
        /* disable pull-down */
        ret = ch943x_reg_update(s, CH943X_GPIOPD_REG + (offset / 8), bit, 0);
        if (ret < 0)
            return ret;
        /* enable pull-up */
        ret = ch943x_reg_update(s, CH943X_GPIOPU_REG + (offset / 8), bit, bit);
        if (ret < 0)
            return ret;
        break;
    case CHIP_CH9434D:
    case CHIP_CH9432D:
    case CHIP_CH9437F:
    case CHIP_CH9438F:
        ret = ch943x_reg_update(s, CH9434_GPIO_SET_0 + (offset / 8), bit, bit); /* Enable pull-up resistor */
        if (ret < 0)
            return ret;

        ret = ch943x_reg_read(s, CH9434_GPIO_DIR_MOD_0 + (offset / 2), 1, &val);
        if (ret < 0)
            return ret;

        if ((offset % 2) == 0) {
            val &= ~GENMASK(1, 0); /* Input mode */
            val &= ~BIT(2);        /* Input with pull-up/down resistor */
            val |= BIT(3);
        } else {
            val &= ~GENMASK(5, 4);
            val |= BIT(7);
            val &= ~BIT(6);
        }
        ret = ch943x_reg_write(s, CH9434_GPIO_DIR_MOD_0 + (offset / 2), 1, &val);
        if (ret < 0)
            return ret;
        break;
    default:
        break;
    }

    return 0;
}

static int ch943x_gpio_dir_output(struct gpio_chip *gc, unsigned int offset, int value)
{
    struct ch943x *s = gpiochip_get_data(gc);
    u8 set_mask = 0, clear_mask = 0;
    u8 val;
    u8 bit = BIT(offset % 8);
    int ret;

    DRV_DEBUG(s->dev, "%s offset:%u\n", __func__, offset);

    switch (s->chip.chiptype) {
    case CHIP_CH9434A:
    case CHIP_CH9434M:
        ret = ch943x_reg_update(s, CH9434_GPIO_DIR_MOD_0 + (offset / 8), bit, bit);
        if (ret < 0)
            return ret;
        break;
    case CHIP_CH9434D:
    case CHIP_CH9432D:
    case CHIP_CH9438F:
    case CHIP_CH9437F:
        /* push-pull output */
        ret = ch943x_reg_read(s, CH9434_GPIO_DIR_MOD_0 + (offset / 2), 1, &val);
        if (ret < 0)
            return ret;
        if ((offset % 2) == 0) {        /* GPIO0/2/4/6... */
            set_mask = GENMASK(1, 0);   /* Output mode */
            clear_mask = GENMASK(3, 2); /* Push-pull output */
        } else {                        /* GPIO1/3/5/7... */
            set_mask = GENMASK(5, 4);
            clear_mask = GENMASK(7, 6);
        }
        val = (val & ~clear_mask) | set_mask;
        ret = ch943x_reg_write(s, CH9434_GPIO_DIR_MOD_0 + (offset / 2), 1, &val);
        if (ret < 0)
            return ret;
        break;
    default:
        break;
    }

    return 0;
}

#define GPIO_DIR_IN  1
#define GPIO_DIR_OUT 0

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
// returns direction for signal "offset", 0=out, 1=in
static int ch943x_gpio_get_direction(struct gpio_chip *gc, unsigned offset)
{
    struct ch943x *s = gpiochip_get_data(gc);
    u8 val;
    int ret = -EINVAL;

    DRV_DEBUG(s->dev, "%s offset:%u\n", __func__, offset);

    switch (s->chip.chiptype) {
    case CHIP_CH9434A:
    case CHIP_CH9434M:
        ret = ch943x_reg_read(s, CH9434_GPIO_DIR_MOD_0 + (offset / 8), 1, &val);
        if (ret < 0)
            return ret;
        
        if (val & BIT(offset % 8))
            ret = GPIO_DIR_OUT;
        else 
            ret = GPIO_DIR_IN;
        break;
    case CHIP_CH9434D:
    case CHIP_CH9432D:
    case CHIP_CH9438F:
    case CHIP_CH9437F:
        ret = ch943x_reg_read(s, CH9434_GPIO_DIR_MOD_0 + (offset / 2), 1, &val);
        if (ret < 0)
            return ret;

        if ((offset % 2) == 0) {
            if ((val & GENMASK(1, 0)) == 0)
                ret = GPIO_DIR_IN;
            else if ((val & GENMASK(1, 0)) == GENMASK(1, 0))
                ret = GPIO_DIR_OUT;
        } else {
            if ((val & GENMASK(5, 4)) == 0)
                ret = GPIO_DIR_IN;
            else if ((val & GENMASK(5, 4)) == GENMASK(5, 4))
                ret = GPIO_DIR_OUT;
        }
        break;
    default:
        break;
    }

    return ret;
}
#endif

static int ch943x_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
    struct ch943x *s = gpiochip_get_data(gc);
    u8 val;
    int ret;

    DRV_DEBUG(s->dev, "%s offset:%u\n", __func__, offset);

    switch (s->chip.chiptype) {
    case CHIP_CH9434A:
    case CHIP_CH9434M:
        ret = ch943x_reg_read(s, CH943X_GPIOVAL_REG + (offset / 8), 1, &val);
        if (ret < 0)
            return ret;
        break;
    case CHIP_CH9434D:
    case CHIP_CH9432D:
    case CHIP_CH9438F:
    case CHIP_CH9437F:
        ret = ch943x_reg_read(s, CH9434_GPIO_PIN_VAL_0 + (offset / 8), 1, &val);
        if (ret < 0)
            return ret;
        break;
    default:
        return 0;
    }

    ret = (val & BIT(offset % 8)) ? 1 : 0;
    return ret;
}

static void ch943x_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
    struct ch943x *s = gpiochip_get_data(gc);
    u8 val;
    u8 bit = BIT(offset % 8);
    int ret;

    DRV_DEBUG(s->dev, "%s offset:%u\n", __func__, offset);

    switch (s->chip.chiptype) {
    case CHIP_CH9434A:
    case CHIP_CH9434M:
        if (value) {
            ret = ch943x_reg_update(s, CH943X_GPIOVAL_REG + (offset / 8), bit, bit);
            if (ret < 0)
                return;
        } else {
            ret = ch943x_reg_update(s, CH943X_GPIOVAL_REG + (offset / 8), bit, 0);
            if (ret < 0)
                return;
        }
        break;
    case CHIP_CH9434D:
    case CHIP_CH9432D:
    case CHIP_CH9438F:
    case CHIP_CH9437F:
        val = bit;
        if (value) {
            ret = ch943x_reg_write(s, CH9434_GPIO_SET_0 + (offset / 8), 1, &val);
            if (ret < 0)
                return;
        } else {
            ret = ch943x_reg_write(s, CH9434_GPIO_RESET_0 + (offset / 8), 1, &val);
            if (ret < 0)
                return;
        }
        break;
    default:
        break;
    }
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
static int ch943x_gpio_set_config(struct gpio_chip *gc, unsigned int offset, unsigned long config)
{
    struct ch943x *s = gpiochip_get_data(gc);
    u8 bit = BIT(offset % 8);
    u8 val;
    u8 reg;
    u8 set_mask = 0;
    u8 clear_mask = 0;
    int ret;

    DRV_DEBUG(s->dev, "%s offset:%u config:%d\n", __func__, offset, pinconf_to_config_param(config));

    switch (s->chip.chiptype) {
    case CHIP_CH9434A:
    case CHIP_CH9434M:
        switch (pinconf_to_config_param(config)) {
        /* To control the bias resistor, upgrade to libgpiod 2.x and Linux 5.10+. */
        case PIN_CONFIG_BIAS_DISABLE:
            ret = ch943x_reg_update(s, CH943X_GPIOPU_REG + (offset / 8), bit, 0);
            if (ret < 0)
                return ret;
            ret = ch943x_reg_update(s, CH943X_GPIOPD_REG + (offset / 8), bit, 0);
            if (ret < 0)
                return ret;
            break;
        case PIN_CONFIG_BIAS_PULL_DOWN:
            ret = ch943x_reg_update(s, CH943X_GPIOPU_REG + (offset / 8), bit, 0);
            if (ret < 0)
                return ret;
            ret = ch943x_reg_update(s, CH943X_GPIOPD_REG + (offset / 8), bit, bit);
            if (ret < 0)
                return ret;
            break;
        case PIN_CONFIG_BIAS_PULL_UP:
            ret = ch943x_reg_update(s, CH943X_GPIOPD_REG + (offset / 8), bit, 0);
            if (ret < 0)
                return ret;
            ret = ch943x_reg_update(s, CH943X_GPIOPU_REG + (offset / 8), bit, bit);
            if (ret < 0)
                return ret;
            break;
        default:
            break;
        }
        break;
    case CHIP_CH9434D:
    case CHIP_CH9432D:
    case CHIP_CH9438F:
    case CHIP_CH9437F:
        switch (pinconf_to_config_param(config)) {
        case PIN_CONFIG_DRIVE_PUSH_PULL:
            ret = ch943x_reg_read(s, CH9434_GPIO_DIR_MOD_0 + (offset / 2), 1, &val);
            if (ret < 0)
                return ret;
            if ((offset % 2) == 0) {        /* GPIO0/2/4/6... */
                set_mask = GENMASK(1, 0);   /* Output mode */
                clear_mask = GENMASK(3, 2); /* Push-pull output */
            } else {                        /* GPIO1/3/5/7... */
                set_mask = GENMASK(5, 4);
                clear_mask = GENMASK(7, 6);
            }
            val = (val & ~clear_mask) | set_mask;
            ret = ch943x_reg_write(s, CH9434_GPIO_DIR_MOD_0 + (offset / 2), 1, &val);
            if (ret < 0)
                return ret;
            break;
        case PIN_CONFIG_DRIVE_OPEN_DRAIN:
            ret = ch943x_reg_read(s, CH9434_GPIO_DIR_MOD_0 + (offset / 2), 1, &val);
            if (ret < 0)
                return ret;
            if ((offset % 2) == 0) {      /* GPIO0/2/4/6... */
                val |= (BIT(0) | BIT(1)); /* Output mode */
                val |= BIT(2);            /* Open-drain output */
                val &= ~BIT(3);
            } else {                      /* GPIO1/3/5/7... */
                val |= (BIT(4) | BIT(5)); /* Output mode */
                val |= BIT(6);            /* Open-drain output */
                val &= ~BIT(7);
            }
            ret = ch943x_reg_write(s, CH9434_GPIO_DIR_MOD_0 + (offset / 2), 1, &val);
            if (ret < 0)
                return ret;
            break;
        case PIN_CONFIG_BIAS_DISABLE:
            ret = ch943x_reg_read(s, CH9434_GPIO_DIR_MOD_0 + (offset / 2), 1, &val);
            if (ret < 0)
                return ret;
            if ((offset % 2) == 0) {
                val &= ~GENMASK(1, 0); /* Input mode */
                val |= BIT(2);         /* Floating input */
                val &= ~BIT(3);
            } else {
                val &= ~GENMASK(5, 4);
                val |= BIT(6);
                val &= ~BIT(7);
            }
            ret = ch943x_reg_write(s, CH9434_GPIO_DIR_MOD_0 + (offset / 2), 1, &val);
            if (ret < 0)
                return ret;
            break;
        case PIN_CONFIG_BIAS_PULL_DOWN:
        case PIN_CONFIG_BIAS_PULL_UP:
            if (pinconf_to_config_param(config) == PIN_CONFIG_BIAS_PULL_UP)
                reg = CH9434_GPIO_SET_0;
            else
                reg = CH9434_GPIO_RESET_0;

            ret = ch943x_reg_update(s, reg + (offset / 8), bit, bit); /* Enable pull-up/down resistor */
            if (ret < 0)
                return ret;

            ret = ch943x_reg_read(s, CH9434_GPIO_DIR_MOD_0 + (offset / 2), 1, &val);
            if (ret < 0)
                return ret;
            if ((offset % 2) == 0) {
                val &= ~GENMASK(1, 0); /* Input mode */
                val &= ~BIT(2);        /* Input with pull-up/down resistor */
                val |= BIT(3);
            } else {
                val &= ~GENMASK(5, 4);
                val |= BIT(7);
                val &= ~BIT(6);
            }
            ret = ch943x_reg_write(s, CH9434_GPIO_DIR_MOD_0 + (offset / 2), 1, &val);
            if (ret < 0)
                return ret;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

    return 0;
}
#endif

int ch943x_gpio_register(struct ch943x *s)
{
    int ret;

    s->gpio_chip.label = "ch943x_gpio";
    s->gpio_chip.parent = s->dev;
    s->gpio_chip.request = ch943x_gpio_request;
    s->gpio_chip.free = ch943x_gpio_free;
    s->gpio_chip.direction_input = ch943x_gpio_dir_input;
    s->gpio_chip.direction_output = ch943x_gpio_dir_output;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
    s->gpio_chip.get_direction = ch943x_gpio_get_direction;
#endif
    s->gpio_chip.get = ch943x_gpio_get;
    s->gpio_chip.set = ch943x_gpio_set;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
    s->gpio_chip.set_config = ch943x_gpio_set_config;
#endif
    s->gpio_chip.ngpio = s->chip.nr_gpio;
    s->gpio_chip.can_sleep = 1;
    s->gpio_chip.base = -1;

    ret = devm_gpiochip_add_data(s->dev, &s->gpio_chip, s);
    if (ret < 0) {
        dev_err(s->dev, "gpio_init: Failed to add ch943x_gpio\n");
        return ret;
    }
    dev_info(s->dev, "Registered GPIOs from %d to %d", s->gpio_chip.base, s->gpio_chip.base + s->gpio_chip.ngpio - 1);

    return 0;
}

int ch943x_gpio_remove(struct ch943x *s)
{
    return 0;
}
