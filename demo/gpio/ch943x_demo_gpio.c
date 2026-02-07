/*
 * CH9434A/M/D,CH9438/CH9437/CH9432 gpio application example
 *
 * Copyright (C) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web: http://wch.cn
 * Author: WCH <tech@wch.cn>
 * 
 * V1.0 - initial version
 * 
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/serial.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include "ch943x_lib.h"

int main(int argc, char *argv[])
{
    int fd, ret;
    char c;
    int gpio_number;
    int ioval;

    if (argc != 3) {
        fprintf(stdout, "Usage: ./demo /dev/ch943x_iodevX [gpio number]\n");
        fprintf(stdout, "For example, Operate GPIO0: ./demo /dev/ch943x_iodev0 0\n");
        return -1;
    }

    gpio_number = atoi(argv[2]);
    if (gpio_number < 0)
        return -1;

    fd = open(argv[1], O_RDWR);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    while (1) {
        if (c != '\n') {
            fprintf(stdout, "***************************************************\n");
            fprintf(stdout, "Enter the following command to control the GPIO%d:\n", gpio_number);
            fprintf(stdout, "e - enable GPIO\n");
            fprintf(stdout, "d - disable GPIO\n");
            fprintf(stdout, "o - set direction output\n");
            fprintf(stdout, "i - set direction input\n");
            fprintf(stdout, "h - set gpio high\n");
            fprintf(stdout, "l - set gpio low\n");
            fprintf(stdout, "g - get gpio value\n");
            fprintf(stdout, "q - exit app\n");
            fprintf(stdout, "***************************************************\n");
        }
        scanf("%c", &c);
        if (c == 'q')
            break;

        switch (c) {
        case 'e':
            ret = ch943x_single_gpioenable(fd, gpio_number, 1);
            if (ret < 0) {
                printf("GPIO%d enable/disable failed. ret:%d\n", gpio_number, ret);
                break;
            } else
                printf("GPIO%d enable/disable Success.\n", gpio_number);
            break;
        case 'd':
            ret = ch943x_single_gpioenable(fd, gpio_number, 0);
            if (ret < 0) {
                printf("GPIO%d disable failed.\n", gpio_number);
                break;
            } else {
                printf("GPIO%d disable Success.\n", gpio_number);
            }
            break;
        case 'o':
            ret = ch943x_single_gpioconfig(fd, gpio_number, OUTPUT, CH943X_IOMODE_OPEN_DRAIN);
            if (ret < 0) {
                printf("GPIO%d config failed.\n", gpio_number);
                break;
            }
            break;
        case 'i':
            ret = ch943x_single_gpioconfig(fd, gpio_number, INPUT, CH943X_IOMODE_PULLUP_INPUT);
            if (ret < 0) {
                printf("GPIO%d config failed.\n", gpio_number);
                break;
            }
            break;
        case 'h':
            ret = ch943x_single_gpioset(fd, gpio_number, 1);
            if (ret < 0) {
                printf("GPIO%d set failed.\n", gpio_number);
                break;
            }
            break;
        case 'l':
            ret = ch943x_single_gpioset(fd, gpio_number, 0);
            if (ret < 0) {
                printf("GPIO%d set failed.\n", gpio_number);
                break;
            }
            break;
        case 'g':
            ioval = ch943x_single_gpioget(fd, gpio_number);
            if (ioval < 0) {
                printf("GPIO%d get failed.\n", gpio_number);
                break;
            } else
                printf("gpio input value: %d\n", ioval);
            break;
        default:
            break;
        }
    }

    close(fd);
    return 0;
}
