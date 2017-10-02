#ifndef __LED_H__
#define __LED_H__

#include <rtthread.h>
#include "gd32f10x.h"

// LED define
#define LED1_RCC                    RCC_APB2PERIPH_GPIOA
#define LED1_GPIO                   GPIOA
#define LED1_PIN                    (GPIO_PIN_15)

#define LED2_RCC                    RCC_APB2PERIPH_GPIOB
#define LED2_GPIO                   GPIOB
#define LED2_PIN                    (GPIO_PIN_3)

#define LED3_RCC                    RCC_APB2PERIPH_GPIOB
#define LED3_GPIO                   GPIOB
#define LED3_PIN                    (GPIO_PIN_4)

#define LEDR_RCC                    RCC_APB2PERIPH_GPIOA
#define LEDR_GPIO                   GPIOA
#define LEDR_PIN                    (GPIO_PIN_8)

#define LEDG_RCC                    RCC_APB2PERIPH_GPIOB
#define LEDG_GPIO                   GPIOB
#define LEDG_PIN                    (GPIO_PIN_6)

#define LEDB_RCC                    RCC_APB2PERIPH_GPIOB
#define LEDB_GPIO                   GPIOB
#define LEDB_PIN                    (GPIO_PIN_7)

void led_init(void);
void led_on(rt_uint32_t n);
void led_off(rt_uint32_t n);

#endif
