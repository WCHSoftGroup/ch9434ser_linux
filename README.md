## CH943X Linux驱动程序和GPIO接口库使用说明

此资料包包含以下部分：Linux系统SPI/I2C/UART控制接口转SERIAL/CAN/GPIO设备驱动程序、GPIO应用库与演示程序。
适用于如下芯片的接口扩展功能：

|   型号    | 控制接口 |        转换接口        |
| :-------: | :------: | :--------------------: |
|  CH9434D  | SPI/I2C  | 4路UART/4路GPIO/1路CAN |
| CH9434A/M |   SPI    |    4路UART/25路GPIO    |
|  CH9438F  |   SPI    |    8路UART/8路GPIO     |
|  CH9437F  | I2C/UART | 8路或7路UART/11路GPIO  |
|  CH9432D  | SPI/I2C  |    2路UART/8路GPIO     |
| CH432T/Q  |   SPI    |        2路UART         |

**注意：CH432芯片FIFO缓冲区较小，且不支持SPI连续传输，对于Linux系统平台19200bps及以上的应用，建议选择CH943X系列芯片。**

## 1 硬件准备

1、SPI/I2C/UART接线方式、电源、时钟、主控接口配置等，可参考芯片对应的EVT原理图；
2、**中断功能(INT#引脚)为必要功能**，须和SOC主机的中断IO连接

## 2 驱动移植

### 2.1 相关功能配置

对于**CH9434D/CH9438F/CH9437F/CH9432D**芯片，引脚之间存在功能复用，须根据实际使用需求在`ch943x_cfg.h`文件中修改适配，同时也可配置是否输出调试信息、中断IO的获取方式等，具体修改方式如下：

**注：当前为单芯片适配方式，即：安卓/Linux主机当前仅连接1颗扩展芯片，多芯片适配方式参考第3章**

#### 2.1.1 驱动调试输出开关

```
/* 启用或禁用调试信息的输出（1:启用 0:禁用） */
#define ENABLE_DRIVER_DEBUG 0
/* 启用或禁用更详细的调试信息的输出（1:启用 0:禁用） */
#define ENABLE_DRIVER_VERBOSEDEGUG 0
```

#### 2.1.2 配置中断IO

声明如下宏之一，用来指定中断IO的获取方式：
`USE_IRQ_FROM_DTS`：表示在DTS文件中指定中断IO
`GPIO_NUMBER`：表示指定中断IO对应的GPIO引脚号

示例：

```
/* 在DTS文件中指定中断IO */
#define USE_IRQ_FROM_DTS

/* 如果DTS文件未定义中断I/O, 则需定义GPIO_NUMBER宏, 并指定与中断I/O相对应的GPIO引脚号, 通常可以在 /sys/class/gpio/ 目录中查看该信息 */
#ifndef USE_IRQ_FROM_DTS
#define GPIO_NUMBER (0)
#endif
```

**注意：在Linux/安卓系统应用芯片时，中断IO为必要配置**

#### 2.1.3 配置控制接口
在选择芯片的控制接口时，可以声明如下宏之一，用来指定芯片的控制接口：
`USE_SPI_MODE`：SPI接口
`USE_I2C_MODE`：I2C接口
`USE_SERIAL_MODE`：串口接口

示例：

```
#define USE_SPI_MODE /* 使用SPI接口 */
```

如果使用串口作为控制接口(仅CH9437F芯片支持)，则须指定主控串口设备路径和波特率，示例：

```
#define USE_SERIAL_MODE

#ifdef USE_SERIAL_MODE
#define CTRLUART_PATH "/dev/ttyS3" /* 表示使用SOC的/dev/ttyS3串口作为主控接口连接CH9437扩展7路UART */
#define CTRLUART_BAUD 4000000      /* 表示SOC的/dev/ttyS3串口波特率配置为4Mbps */
#endif
```

如果系统的Linux内核版本大于5.17，在加载ch943x驱动之前，须使用stty命令或通用串口应用程序配置一次主控串口的参数，示例：

```
stty -F /dev/ttyS3 1000000 raw -echo -echoe -echok -echoctl -echoke
```

#### 2.1.4 配置时钟

对于**CH9434D/CH9438F/CH9437F/CH9432D**芯片，支持内部时钟或外部时钟模式，声明如下宏之一，用来指定芯片时钟：
`INTERNAL_CLOCK`：内部时钟
`EXTERN_CLOCK`：外部时钟

示例：

```
#define INTERNAL_CLOCK
```

#### 2.1.5 配置TNOW使能

使用RS485串行端口通信时，声明`CH943X_TNOWX_ON`宏，表示启用TNOWx引脚，示例：

```
#define CH943X_TNOW0_ON /* 启用TNOW0 */
#define CH943X_TNOW1_ON /* 启用TNOW1 */
#define CH943X_TNOW2_ON /* 启用TNOW2 */
#define CH943X_TNOW3_ON /* 启用TNOW3 */
...
```

#### 2.1.6 配置CAN使能

对于CH9434D芯片，支持CAN接口，声明`CH9434D_CAN_ON`宏表示启用CAN接口

```
#define CH9434D_CAN_ON
```

### 2.2 编译驱动程序

Makefile示例

```makefile
KERNEL_DIR=xxx # Linux内核绝对路径
ARCH=xxx
CROSS_COMPILE=xxx # 指定交叉编译工具链
export  ARCH  CROSS_COMPILE

DRIVERNAME := ch943x-module
obj-m := $(DRIVERNAME).o
$(DRIVERNAME)-y := ch943x.o ch943x_ctrl.o ch943x_uart.o ch943x_can.o

all:
	$(MAKE) EXTRA_CFLAGS=-fno-pic -C $(KERNEL_DIR) M=$(CURDIR) modules
.PHONY:clean
clean:
	$(MAKE)  -C $(KERNEL_DIR) M=$(CURDIR) clean
```

### 2.3 配置DTS

SPI接口dts示例：

```
&spiX{
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <xxx>;
    pinctrl-1 = <xxx>;
    cs-gpios = <xxx>;

    ch943x@0 {
        compatible = "wch,ch943x";
        reg = <0>;
        spi-max-frequency = <2000000>; 
        interrupt-parent = <&gpiox>; /* 指定中断IO */
        interrupts = <xxx IRQ_TYPE_LEVEL_LOW>;
    };
};
```

I2C接口dts示例：

```
&i2c0{
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <xxx>;
    #address-cells = <1>;
    #size-cells = <0>;
    clock-frequency = <1000000>;

    ch943x@xx {
        compatible = "wch,ch943x";
        reg = <xx>;
        status = "okay";
    };
};
```

**注意**：需要在dts文件中指定CH943X芯片所在的SPI总线、片选IO、SPI时钟频率、中断IO等，不同SOC平台的dts语法、节点命名方式等可能有差异，应以实际为准，或参考SOC平台的SDK资料。

### 2.4 加载驱动

执行`insmod ch943x_module.ko`即可加载驱动程序，此处仅展示驱动程序动态编译方式，静态编译可参考各SoC厂商SDK相关说明。

```
加载驱动模块
root@user:~# insmod ch943x-module.ko

以CH9434A为例,驱动正常加载后,系统日志中会输出如下信息

[20399.782346] ch943x_module: SPI/I2C/SERIAL to SERIAL/CAN/GPIO driver for CH9434/CH9438/CH9437/CH9432.
[20399.782358] ch943x_module: V1.5 On 2026.02
[20399.783682] ch943x_spi spi3.0: CHIP TYPE:CH9434A - V1.3
[20399.984664] ch943x_spi spi3.0: ttyCH943X0: uart device
[20399.985088] ch943x_spi spi3.0: ttyCH943X1: uart device
[20399.985739] ch943x_spi spi3.0: ttyCH943X2: uart device
[20399.987878] ch943x_spi spi3.0: ttyCH943X3: uart device
[20399.993565] ch943x_spi spi3.0: ch943x_iodev0: character device
```

## 3 多芯片应用适配方法

**注意：单芯片应用场景，可忽略本章。**
多芯片应用场景，包括单SPI/I2C主机级联多个芯片、多SPI/I2C主机连接多芯片、多个控制串口连接多个芯片等，具体适配方式如下：
在`ch943x_cfg.h`文件中，增加`MULTI_CHIP_MODE`宏声明，然后根据具体的项目需求，配置要使用的IO引脚使能，然后修改驱动程序源码。

```
#define MULTI_CHIP_MODE
```

### 3.1 SPI接口模式

单SPI主机应用：
修改`ch943x_ctrl.c`中`ch943x_io_enable`函数，根据SPI从机或CS索引号区分芯片，并根据实际应用需求配置引脚功能复用，示例：

```c
int ch943x_io_enable(struct ch943x *s)
{
    int i;
#ifdef USE_SPI_MODE
    if (s->spi_dev->chip_select == 0) { /* CH943X 芯片#0 */
        s->tnow_enable_bits = 0x0f; /* BIT0-7表示TNOW0-7启用或禁用(1:启用 0:禁用) */
        s->extern_clock_on = false; /* 使用内部或外部时钟(true:外部时钟 false:内部时钟) */
        s->can_on = false;          /* 启用或禁用CAN功能(true:启用 false:禁用) */
    } else if (s->spi_dev->chip_select == 1) { /* CH943X 芯片#1 */
        s->tnow_enable_bits = 0x00;
        s->extern_clock_on = false;
        s->can_on = false;
    }
#elif defined(USE_I2C_MODE)
    ...
#elif defined(USE_SERIAL_MODE)
	...
#endif
	...
}
```

多SPI主机应用：
修改`ch943x_ctrl.c`中`ch943x_io_enable`函数，可根据SPI总线所以区分芯片，并根据实际应用需求配置引脚功能复用，示例：

```c
int ch943x_io_enable(struct ch943x *s)
{
    int i;
#ifdef USE_SPI_MODE
    if (s->spi_dev->master->bus_num == 0) {
        /* 同上示例程序 */
    } else if (s->spi_dev->master->bus_num == 1) {
    }
#elif defined(USE_I2C_MODE)
    ...
#elif defined(USE_SERIAL_MODE)
	...
#endif
	...
}
```

### 3.2 I2C接口模式

单I2C主机应用
修改`ch943x_ctrl.c`中`ch943x_io_enable`函数，可根据I2C从机地址区分芯片，并根据实际应用需求配置引脚功能复用，示例：

```c
int ch943x_io_enable(struct ch943x *s)
{
    int i;
#ifdef USE_SPI_MODE
	...
#elif defined(USE_I2C_MODE)
    if (s->i2c->addr == 0x2a) {
        /* 同3.1示例程序 */
    } else if (s->i2c->addr == 0x2b) {
    }
#elif defined(USE_SERIAL_MODE)
	...
#endif
	...
}
```

多I2C主机应用
修改`ch943x_ctrl.c`中`ch943x_io_enable`函数，可根据I2C主机总线索引区分芯片，并根据实际应用需求配置引脚功能复用，示例：

```c
int ch943x_io_enable(struct ch943x *s)
{
    int i;
#ifdef USE_SPI_MODE
	...
#elif defined(USE_I2C_MODE)
    if (s->i2c->adapter->nr == 0) {
        /* 同3.1示例程序 */
    } else if (s->i2c->adapter->nr == 1) {
    }
#elif defined(USE_SERIAL_MODE)
	...
#endif
	...
}
```

### 3.3 串口接口模式

多串口分别连接多个CH9437芯片应用时，修改`ch943x.c`文件的`ctrluart_init`函数，可根据`platform_device`的`id`字段区分芯片，然后分别指定要使用的控制接口，示例：

```c
static int ctrluart_init(struct ch943x *s)
{
    int ret;
    u8 data;

#ifdef MULTI_CHIP_MODE
    /**
     * Distinguish the control serial ports based on the id field of the platform_device.
     * SERIAL mode example for adapting to multi-chip(CH9437):
     *
     * Only after adding the above code can the following filp_open function be executed.
     */
    if (s->pdev->id == 0) {
        snprintf(s->ctrluart_path, sizeof(s->ctrluart_path), "ttyS3");
        s->ctrluart_baud = 4000000;
    } else if (s->pdev->id == 1) {
    	snprintf(s->ctrluart_path, sizeof(s->ctrluart_path), "ttyS4");
        s->ctrluart_baud = 4000000;
    } else if (s->pdev->id == 2) {
        ...
    }
    ...
#else
	...
#endif /* MULTI_CHIP_MODE */
	...
}
```

然后修改`ch943x_ctrl.c`中`ch943x_io_enable`函数，可根据`platform_device`的`id`字段区分芯片，并根据实际应用需求配置引脚功能复用，示例：

```c
int ch943x_io_enable(struct ch943x *s)
{
    int i;
#ifdef USE_SPI_MODE
	...
#elif defined(USE_I2C_MODE)
	...
#elif defined(USE_SERIAL_MODE)
	if (s->pdev->id == 0) {
        /* 同3.1示例程序 */
    } else if (s->pdev->id == 1) {
    }
#endif
	...
}
```

## 4 功能验证

### 4.1 GPIO接口库使用方法

####  4.1.1 demo程序演示
编译：
`gcc ch943x_lib.c ch943x_demo_gpio.c -o demo`

执行：

```
root@lubancat:~/ch943x_gpio_lib# ./demo /dev/ch943x_iodev0 0
***************************************************
Enter the following command to control the GPIO0:
e - enable GPIO
d - disable GPIO
o - set direction output
i - set direction input
h - set gpio high
l - set gpio low
g - get gpio value
q - exit app
***************************************************

输入参数说明：
e:使能GPIO
d:禁用GPIO
o:设置方向为输出
i:设置方向为输入
h:输出高电平
l:输出低电平
g:获取输入电平
q:退出程序
```

#### 4.1.2 接口调用流程

step1: 打开设备节点文件, 调用`open("/dev/ch943x_iodevX", O_RDWR)`;
step2: 启用GPIO, 调用`ch943x_single_gpioenable`;
step3: 配置GPIO方向和电气属性, 调用`ch943x_single_gpioconfig`;
step4: 输出高/低电平或读取输入状态, 调用`ch943x_single_gpioset`或`ch943x_single_gpioget`;
step5: 禁用GPIO, 调用`ch943x_single_gpioenable`;
step6: 关闭设备节点文件, 调用`close`

### 4.2 UART功能

驱动加载成功后，在/dev/目录会出现多个串口设备节点：/dev/ttyCH943X，然后即可调用系统API来操作串口。
Linux系统串口应用程序开发参考：
Linux串口应用开源项目tty_uart链接：https://github.com/WCHSoftGroup/tty_uart

### 4.3 RS485功能

CH943X芯片的串口为TTL电平，在实现半双工 RS485 串口时需要外接RS485电平转换芯片，设计中需要有信号来控制 RS485 转接芯片的发送和接收使能端，即TNOW引脚（TNOW功能配置方式参考2.1.5节）。
开启TNOW功能后，该引脚**默认为低电平**，当串口处于**发送状态**时会**自动拉高**处于有效状态，发送完成再恢复低电平。

### 4.4 CAN功能

加载驱动后，执行ifconfig -a命令可查看新创建的CAN设备

```
root@lubancat:~# ifconfig -a
can0: flags=128<NOARP>  mtu 16
        unspec 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  txqueuelen 10  (未指定)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

基本通信功能验证

```
初始化CAN接口
root@user:~# ip link set can0 type can bitrate 1000000
root@user:~# ifconfig can0 up

发送数据
root@user:~# cansend can0 5A1#11.22.33.44.55.66.77.88

接收数据
root@user:~# candump can0
can0  06A5EE64   [8]  71 5B 50 76 55 C2 22 63
```

## 5 CH432驱动适配方法

该驱动程序兼容CH432芯片（SPI转2路串口），使用前须在ch943x_cfg.h文件中声明USE_CHIP_CH432宏，示例：

```c
/* enable or disable the output of debug information (read/write registers) */
#define ENABLE_DRIVER_DEBUG 0
/* enable or disable the output of more detailed debugging information */
#define ENAVLE_DRIVER_VERBOSEDEGUG 0

#define USE_CHIP_CH432
...
```

Makefile示例：

```makefile
KERNEL_DIR=xxx # Linux内核绝对路径
ARCH=xxx
CROSS_COMPILE=xxx # 指定交叉编译工具链
export  ARCH  CROSS_COMPILE

DRIVERNAME := ch943x-module
obj-m := $(DRIVERNAME).o
$(DRIVERNAME)-y := ch943x.o ch43x_uart.o # 仅编译ch943x.c和ch43x_uart.c

all:
	$(MAKE) EXTRA_CFLAGS=-fno-pic -C $(KERNEL_DIR) M=$(CURDIR) modules
.PHONY:clean
clean:
	$(MAKE)  -C $(KERNEL_DIR) M=$(CURDIR) clean
```

同第1章介绍的驱动加载流程，加载驱动后可创建出2个串口设备。

## 6 常见问题解法方法

**1、主控接口通信测试失败**
加载驱动时，如果系统日志输出如下报错信息，表示**主控接口通信测试失败**。

```
[  148.356879] ch943x_module: SPI/I2C/UART to SERIAL/CAN/GPIO driver for CH9434/CH9438/CH9437/CH9432, etc
[  148.356883] ch943x_module: V1.5 On 2026.02
[  148.358435] ch943x_spi spi3.0: ch943x_hw_test Failed. tx:87 55, tx:07 rx:00
[  148.358497] ch943x_spi spi3.0: Hardware transfer test Failed.
[  148.358511] ch943x_spi spi3.0: ch943x_probe error
[  148.358607] ch943x_spi: probe of spi3.0 failed with error -1

```

**排查方法：**

- **检查SPI/I2C的接线方式**
  SPI：MOSI连接SDI，MISO连接SDO，CLK和CS对应连接
  I2C：SDA和SCL对应连接

- **检查芯片工作状态**
  芯片正常供电后，如果使用外部时钟，则检查晶振是否启振；
  CH9434A/M：VDD=VCC=3.3V，Vcore=1.05V左右，XI=XO=300mV左右，串口IO电压=VCC=3.3V
  CH9434D/CH9432D：串口IO电压=VCC(1.8V~3.6V)
  CH9437F/CH9438F：VCC(1.8V~3.6V)，串口IO电压=VIO
- **检查原理图设计**
  可参考评估板原理图
- **检查硬件时序**
  抓SPI/I2C时序，检查实际的时序格式是否符合芯片要求。

**2、/dev中未出现串口节点**
串口设备节点由该驱动程序主动创建，如果未出现，则是初始化期间有出现错误，**应查看系统日志查看返回错误位置**。

**3、中断处理函数(ch943x_ist)没有执行**
执行串口发送或芯片串口接收时，芯片的INT#脚会自动发出(低电平)中断信号；

可以用示波器或逻辑分析仪验证，应用软件执行串口发送后，如果芯片的INT#脚已经发出中断信号，但驱动程序的中断处理函 ch943x_ist 没有执行，则可能的原因：

- 中断IO配置有误，如：dts里配置的中断IO和实际用的不匹配。可手动拉低CPU端的中断IO验证，看 ch943x_ist 函数是否执行；
- 如果中断IO经过了电平转换，如3.3V转1.8V，建议用示波器分别测量电平转换前和转换后的INT#引脚时序状态，确认中断信号经过电平转换后是否正常。

**4、串口波特率和传输速率的区别**

- 串口波特率
  也称比特率，常用表示单位为bps（比特每秒，1秒钟可传输的bit总数）。以9600波特率为例：1秒钟串口信号线可以传输9600个bit位，1个位所需时间就是1/9600秒约104us，芯片支持多少波特率由芯片本身和串口基准时钟决定。
  
- 传输速率
  传输速率是指串口在连续传输时的平均速度，一般等于字节数/单位时间。
  **传输速率大小和多个因素有关，如SPI/I2C的传输效率、主机端驱动程序的处理速度、CPU处理性能等**，驱动程序是通过中断处理串口发送和接收的。因存在以上限制，实际应用阶段可能存在性能不及理论预期的情况，如需要支持连续的高速率传输，建议优先选择FIFO容量更大，控制接口更高速的转接型号。
  
  推荐的产品优先级选型：
  
  SPI接口扩展：CH9432D/CH9434D/CH9438F
  
  I2C接口扩展：CH9432D/CH9434D/CH9437F

如有任何问题，您可以将反馈信息发送至邮箱: tech@wch.cn