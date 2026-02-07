
/* enable or disable the output of debug information (read/write registers) */
#define ENABLE_DRIVER_DEBUG 0
/* enable or disable the output of more detailed debugging information */
#define ENABLE_DRIVER_VERBOSEDEGUG 0

/**
 * If this macro is defined, it indicates that the interrupt is obtained from the dts.
 * The macro "USE_IRQ_FROM_DTS" cannot be defined simultaneously with the macro "GPIO_NUMBER".
 */
#define USE_IRQ_FROM_DTS

/**
 * If the DTS file does not define the interrupt IO,
 * then define the GPIO_NUMBER macro and specify the
 * GPIO_NUMBER corresponding to the interrupt IO.
 * Usually, you can view it in the /sys/class/gpio/ directory.
 */
#ifndef USE_IRQ_FROM_DTS
#define GPIO_NUMBER (0)
#endif

/**
 * When choosing the communication interface of the chip, you can define
 * USE_SPI_MODE(CH9434A/M/D CH9438 CH9432), USE_I2C_MODE(CH9434D CH9437 CH9432)
 * or USE_SERIAL_MODE(CH9437). Only one of these three options can be selected.
 */
#define USE_SPI_MODE /* USE_SPI_MODE/USE_I2C_MODE/USE_SERIAL_MODE */

/**
 * If multiple CH943X chips are used,
 * the MULTI_CHIP_MODE macro needs to be defined.
 * However, if using a single chip, there is no need to define it.
 */

#ifndef MULTI_CHIP_MODE

#ifdef USE_SERIAL_MODE
/**
 * If using the serial port mode (only supported by CH9437),
 * then the CTRLUART_PATH macro and CTRLUART_BAUD macro need to be defined
 * to declare the absolute path and baud rate of the control serial port.
 */
#define CTRLUART_PATH "/dev/ttyS0"
#define CTRLUART_BAUD 4000000
#endif

/**
 * Define the EXTERN_CLOCK or INTERNAL_CLOCK
 * macro to indicate whether the chip uses an external clock or an internal clock.
 * Only one of these options can be selected.
 */
#define INTERNAL_CLOCK /* INTERNAL_CLOCK/EXTERN_CLOCK */

/**
 * When using RS485 serial port communication,
 * define "CH943X_TNOWX_ON" to indicate that the "TNOW" pin will be utilized.
 */
// #define CH943X_TNOW0_ON
// #define CH943X_TNOW1_ON
// #define CH943X_TNOW2_ON
// #define CH943X_TNOW3_ON
// #define CH943X_TNOW4_ON
// #define CH943X_TNOW5_ON
// #define CH943X_TNOW6_ON
// #define CH943X_TNOW7_ON

/**
 * Define the CH9434D_CAN_ON macro and declare whether to use the CAN interface
 */
#define CH9434D_CAN_ON

#endif /* MULTI_CHIP_MODE */
