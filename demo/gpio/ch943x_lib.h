#ifndef _LIB_CH943X_H
#define _LIB_CH943X_H

/* error code */
#define ERROR_CODE1 1 /* Control interface transfer error */
#define ERROR_CODE2 2 /* Illegal GPIO number */
#define ERROR_CODE3 3 /* Failed to get the chip type */
#define ERROR_CODE4 4 /* Illegal direction parameters */
#define ERROR_CODE5 5 /* Illegal electrical attribute parameters */

typedef enum {
    CHIP_CH9434A = 0,
    CHIP_CH9434D,
    CHIP_CH9434M,
    CHIP_CH9438,
    CHIP_CH9437,
    CHIP_CH9432,
} CHIPTYPE;

typedef enum {
    OUTPUT,
    INPUT,
} IODir;

typedef enum {
    CH943X_IOMODE_PUSH_PULL = 0,  /* Push–pull output */
    CH943X_IOMODE_OPEN_DRAIN,     /* Floating open output */
    CH943X_IOMODE_FLOAT_INPUT,    /* Floating input */
    CH943X_IOMODE_PULLUP_INPUT,   /* Pull-up input */
    CH943X_IOMODE_PULLDOWN_INPUT, /* Pull-down input */
} IOMode;

/**
 * ch943x_get_chiptype - get chip type
 * @fd: file descriptor of ch943x GPIO device
 * @type: pointer to chip type
 *
 * The function return 0 if success, others if fail.
 */
extern int ch943x_get_chiptype(int fd, CHIPTYPE *type);

/**
 * ch943x_single_gpioenable - enable or disable the GPIO function
 * @fd: file descriptor of ch943x gpio device
 * @gpionumber: gpio number
 * @enable: 1 on enable, 0 on disable
 *
 * The function return 0 if success, others if fail.
 */
extern int ch943x_single_gpioenable(int fd, uint8_t gpionumber, uint8_t enable);

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
extern int ch943x_single_gpioconfig(int fd, uint8_t gpionumber, IODir dir, IOMode mode);

/**
 * ch943x_single_gpioset - Configure the output level of the GPIO
 * @fd: file descriptor of ch943x gpio device
 * @gpionumber: gpio number
 * @value: 1 on high, 0 on low
 *
 * The function return 0 if success, others if fail.
 */
extern int ch943x_single_gpioset(int fd, uint8_t gpionumber, uint8_t value);

/**
 * ch943x_single_gpioget - Get the input level value of the GPIO
 * @fd: file descriptor of ch943x gpio device
 * @gpionumber: gpio number
 *
 * The function return the level value of the GPIO, 1 on high, 0 on low, others if fail.
 */
extern int ch943x_single_gpioget(int fd, uint8_t gpionumber);

#endif
