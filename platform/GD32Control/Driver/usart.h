/*
 * File      : usart.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2017-08-20     Jone         change this file to gd32f10x
 */
#ifndef __USART_H__
#define __USART_H__

#include <rthw.h>
#include <rtthread.h>

/* USART1 */
#define UART1_GPIO_TX        GPIO_PIN_9
#define UART1_GPIO_RX        GPIO_PIN_10
#define UART1_GPIO           GPIOA

/* USART2 */
#define UART2_GPIO_TX        GPIO_PIN_2
#define UART2_GPIO_RX        GPIO_PIN_3
#define UART2_GPIO           GPIOA

/* USART3_REMAP[1:0] = 00 */
#define UART3_GPIO_TX        GPIO_PIN_10
#define UART3_GPIO_RX        GPIO_PIN_11
#define UART3_GPIO           GPIOB

/* USART4 */
#define UART4_GPIO_TX        GPIO_PIN_10
#define UART4_GPIO_RX        GPIO_PIN_11
#define UART4_GPIO           GPIOC

#define UART_ENABLE_IRQ(n)   NVIC_EnableIRQ((n))
#define UART_DISABLE_IRQ(n)  NVIC_DisableIRQ((n))

void rt_hw_usart_init(void);


#endif

