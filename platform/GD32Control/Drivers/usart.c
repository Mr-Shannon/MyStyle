/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 * 2013-05-13     aozima       update for kehong-lingtai.
 * 2015-01-31     armink       make sure the serial transmit complete in putc()
 * 2016-05-13     armink       add DMA Rx mode
 * 2017-08-20     Jone         change this file to gd32f10x
 */
 
#include "gd32f10x.h"
#include "usart.h"
#include "board.h"
#include <rtdevice.h>

/* GD32 uart driver */
struct gd32_uart
{
    USART_TypeDef *uart_device;
    IRQn_Type irq;
    struct gd32_uart_dma {
        /* dma channel */
        DMA_Channel_TypeDef *rx_ch;
        /* dma global flag */
        uint32_t rx_gl_flag;
        /* dma irq channel */
        uint8_t rx_irq_ch;
        /* last receive index */
        rt_size_t last_recv_len;
    } dma;
};

static void DMA_Configuration(struct rt_serial_device *serial);

static rt_err_t gd32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct gd32_uart* uart;
    USART_InitPara USART_InitStruct;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct gd32_uart *)serial->parent.user_data;

    USART_InitStruct.USART_BRR = cfg->baud_rate;

    if (cfg->data_bits == DATA_BITS_8){
        USART_InitStruct.USART_WL = USART_WL_8B;
    } else if (cfg->data_bits == DATA_BITS_9) {
        USART_InitStruct.USART_WL = USART_WL_9B;
    }

    if (cfg->stop_bits == STOP_BITS_1){
        USART_InitStruct.USART_STBits = USART_STBITS_1;
    } else if (cfg->stop_bits == STOP_BITS_2){
        USART_InitStruct.USART_STBits = USART_STBITS_2;
    }

    if (cfg->parity == PARITY_NONE){
        USART_InitStruct.USART_Parity = USART_PARITY_RESET;
    } else if (cfg->parity == PARITY_ODD) {
        USART_InitStruct.USART_Parity = USART_PARITY_SETODD;
    } else if (cfg->parity == PARITY_EVEN) {
        USART_InitStruct.USART_Parity = USART_PARITY_SETEVEN;
    }

    USART_InitStruct.USART_HardwareFlowControl = USART_HARDWAREFLOWCONTROL_NONE;
    USART_InitStruct.USART_RxorTx = USART_RXORTX_RX | USART_RXORTX_TX;
    USART_Init(uart->uart_device, &USART_InitStruct);

    /* Enable USART */
    USART_Enable(uart->uart_device, ENABLE);

    return RT_EOK;
}

static rt_err_t gd32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct gd32_uart* uart;
    rt_uint32_t ctrl_arg = (rt_uint32_t)(arg);

    RT_ASSERT(serial != RT_NULL);
    uart = (struct gd32_uart *)serial->parent.user_data;

    switch (cmd)
    {
        /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        UART_DISABLE_IRQ(uart->irq);
        /* disable interrupt */
        USART_INT_Set(uart->uart_device, USART_INT_RBNE, DISABLE);
        break;
        /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        UART_ENABLE_IRQ(uart->irq);
        /* enable interrupt */
        USART_INT_Set(uart->uart_device, USART_INT_RBNE, ENABLE);
        break;
        /* USART config */
    case RT_DEVICE_CTRL_CONFIG :
        if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX) {
            DMA_Configuration(serial);
        }
        break;
    }
    return RT_EOK;
}

static int gd32_putc(struct rt_serial_device *serial, char c)
{
    struct gd32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct gd32_uart *)serial->parent.user_data;

    uart->uart_device->DR = c;
    while (!(uart->uart_device->STR & USART_FLAG_TC));

    return 1;
}

static int gd32_getc(struct rt_serial_device *serial)
{
    int ch;
    struct gd32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct gd32_uart *)serial->parent.user_data;

    ch = -1;
    if (uart->uart_device->STR & USART_FLAG_RBNE)
    {
        ch = uart->uart_device->DR & 0xff;
    }

    return ch;
}

/**
 * Serial port receive idle process. This need add to uart idle ISR.
 *
 * @param serial serial device
 */
static void dma_uart_rx_idle_isr(struct rt_serial_device *serial) {
    struct gd32_uart *uart = (struct gd32_uart *) serial->parent.user_data;
    rt_size_t recv_total_len, recv_len;
    /* disable dma, stop receive data */
    DMA_Enable(uart->dma.rx_ch, DISABLE);

    recv_total_len = serial->config.bufsz - DMA_GetCurrDataCounter(uart->dma.rx_ch);
    if (recv_total_len > uart->dma.last_recv_len) {
        recv_len = recv_total_len - uart->dma.last_recv_len;
    } else {
        recv_len = recv_total_len;
    }
    uart->dma.last_recv_len = recv_total_len;

    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

    /* read a data for clear receive idle interrupt flag */
    USART_DataReceive(uart->uart_device);
    DMA_ClearBitState(uart->dma.rx_gl_flag);
    DMA_Enable(uart->dma.rx_ch, ENABLE);
}

/**
 * DMA receive done process. This need add to DMA receive done ISR.
 *
 * @param serial serial device
 */
static void dma_rx_done_isr(struct rt_serial_device *serial) {
    struct gd32_uart *uart = (struct gd32_uart *) serial->parent.user_data;
    rt_size_t recv_total_len, recv_len;
    /* disable dma, stop receive data */
    DMA_Enable(uart->dma.rx_ch, DISABLE);

    recv_total_len = serial->config.bufsz - DMA_GetCurrDataCounter(uart->dma.rx_ch);
    if (recv_total_len > uart->dma.last_recv_len) {
        recv_len = recv_total_len - uart->dma.last_recv_len;
    } else {
        recv_len = recv_total_len;
    }
    uart->dma.last_recv_len = recv_total_len;

    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

    DMA_ClearBitState(uart->dma.rx_gl_flag);
    /* reload */
    DMA_SetCurrDataCounter(uart->dma.rx_ch, serial->config.bufsz);
    DMA_Enable(uart->dma.rx_ch, ENABLE);
}

/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
static void uart_isr(struct rt_serial_device *serial) {
    struct gd32_uart *uart = (struct gd32_uart *) serial->parent.user_data;

    RT_ASSERT(uart != RT_NULL);

    if(USART_GetIntBitState(uart->uart_device, USART_INT_RBNE) != RESET)
    {
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
        /* clear interrupt */
        USART_ClearIntBitState(uart->uart_device, USART_INT_RBNE);
    }
    if(USART_GetIntBitState(uart->uart_device, USART_INT_IDLEF) != RESET)
    {
        dma_uart_rx_idle_isr(serial);
    }
    if (USART_GetIntBitState(uart->uart_device, USART_INT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearIntBitState(uart->uart_device, USART_INT_TC);
    }
    if (USART_GetBitState(uart->uart_device, USART_FLAG_ORE) == SET)
    {
        gd32_getc(serial);
    }
}

static const struct rt_uart_ops gd32_uart_ops =
{
    gd32_configure,
    gd32_control,
    gd32_putc,
    gd32_getc,
};

#if defined(RT_USING_UART1)
/* UART1 device driver Struct */
struct gd32_uart uart1 =
{
    USART1,
    USART1_IRQn,
    {
        DMA1_CHANNEL5,
        DMA1_FLAG_GL5,
        DMA1_Channel5_IRQn,
        0,
    },
};
struct rt_serial_device serial1;

void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel5_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
/* UART2 device driver Struct */
struct gd32_uart uart2 =
{
    USART2,
    USART2_IRQn,
    {
        DMA1_CHANNEL6,
        DMA1_FLAG_GL6,
        DMA1_Channel6_IRQn,
        0,
    },
};
struct rt_serial_device serial2;

void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel6_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
/* UART3 device driver Struct */
struct gd32_uart uart3 =
{
    USART3,
    USART3_IRQn,
    {
        DMA1_CHANNEL3,
        DMA1_FLAG_GL3,
        DMA1_Channel3_IRQn,
        0,
    },
};
struct rt_serial_device serial3;

void USART3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel3_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
/* UART4 device driver Struct */
struct gd32_uart uart4 =
{
    UART4,
    UART4_IRQn,
    {
        DMA2_CHANNEL3,
        DMA2_FLAG_GL3,
        DMA2_Channel3_IRQn,
        0,
    },
};
struct rt_serial_device serial4;

void UART4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial4);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Channel3_IRQHandler(void) {
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial4);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART4 */

static void RCC_Configuration(void)
{
#if defined(RT_USING_UART1)
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_GPIOA | RCC_APB2PERIPH_AF, ENABLE);
    /* Enable UART clock */
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_USART1, ENABLE);
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_GPIOA | RCC_APB2PERIPH_AF, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_USART2, ENABLE);
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_GPIOB | RCC_APB2PERIPH_AF, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_USART3, ENABLE);
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_GPIOC | RCC_APB2PERIPH_AF, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_UART4, ENABLE);
#endif /* RT_USING_UART4 */
}

static void GPIO_Configuration(void)
{
    GPIO_InitPara GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Speed = GPIO_SPEED_50MHZ;

#if defined(RT_USING_UART1)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_IN_FLOATING;
    GPIO_InitStruct.GPIO_Pin = UART1_GPIO_RX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.GPIO_Pin = UART1_GPIO_TX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStruct);
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_IN_FLOATING;
    GPIO_InitStruct.GPIO_Pin = UART2_GPIO_RX;
    GPIO_Init(UART2_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.GPIO_Pin = UART2_GPIO_TX;
    GPIO_Init(UART2_GPIO, &GPIO_InitStruct);
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_IN_FLOATING;
    GPIO_InitStruct.GPIO_Pin = UART3_GPIO_RX;
    GPIO_Init(UART3_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.GPIO_Pin = UART3_GPIO_TX;
    GPIO_Init(UART3_GPIO, &GPIO_InitStruct);
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_IN_FLOATING;
    GPIO_InitStruct.GPIO_Pin = UART4_GPIO_RX;
    GPIO_Init(UART4_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.GPIO_Pin = UART4_GPIO_TX;
    GPIO_Init(UART4_GPIO, &GPIO_InitStruct);
#endif /* RT_USING_UART4 */
}

static void NVIC_Configuration(struct gd32_uart* uart)
{
    NVIC_InitPara NVIC_InitStruct;

    /* Enable the USART1 Interrupt */
    NVIC_InitStruct.NVIC_IRQ = uart->irq;
    NVIC_InitStruct.NVIC_IRQPreemptPriority = 0;
    NVIC_InitStruct.NVIC_IRQSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQEnable = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

static void DMA_Configuration(struct rt_serial_device *serial) {
    struct gd32_uart *uart = (struct gd32_uart *) serial->parent.user_data;
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
    DMA_InitPara DMA_InitStruct;
    NVIC_InitPara NVIC_InitStruct;

    /* enable transmit idle interrupt */
    USART_INT_Set(uart->uart_device, USART_INT_IDLEF , ENABLE);

    /* DMA clock enable */
    RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_DMA1, ENABLE);
    RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_DMA2, ENABLE);

    /* rx dma config */
    DMA_DeInit(uart->dma.rx_ch);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(uart->uart_device->DR);
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t) rx_fifo->buffer;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PERIPHERALSRC;
    DMA_InitStruct.DMA_BufferSize = serial->config.bufsz;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PERIPHERALDATASIZE_BYTE;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MEMORYDATASIZE_BYTE;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_HIGH;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_Init(uart->dma.rx_ch, &DMA_InitStruct);
    DMA_ClearBitState(uart->dma.rx_gl_flag);
    DMA_INTConfig(uart->dma.rx_ch, DMA_INT_TC, ENABLE);
    USART_DMA_Enable(uart->uart_device, USART_DMAREQ_RX, ENABLE);
    DMA_Enable(uart->dma.rx_ch, ENABLE);

    /* rx dma interrupt config */
    NVIC_InitStruct.NVIC_IRQ = uart->dma.rx_irq_ch;
    NVIC_InitStruct.NVIC_IRQPreemptPriority = 0;
    NVIC_InitStruct.NVIC_IRQSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQEnable = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

void rt_hw_usart_init(void)
{
    struct gd32_uart* uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    RCC_Configuration();
    GPIO_Configuration();

#if defined(RT_USING_UART1)
    uart = &uart1;
    config.baud_rate = BAUD_RATE_115200;

    serial1.ops    = &gd32_uart_ops;
    serial1.config = config;

    NVIC_Configuration(uart);

    /* register UART1 device */
    rt_hw_serial_register(&serial1, "uart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX,
                          uart);
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
    uart = &uart2;

    config.baud_rate = BAUD_RATE_115200;
    serial2.ops    = &gd32_uart_ops;
    serial2.config = config;

    NVIC_Configuration(uart);

    /* register UART2 device */
    rt_hw_serial_register(&serial2, "uart2",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX,
                          uart);
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
    uart = &uart3;

    config.baud_rate = BAUD_RATE_115200;

    serial3.ops    = &gd32_uart_ops;
    serial3.config = config;

    NVIC_Configuration(uart);

    /* register UART3 device */
    rt_hw_serial_register(&serial3, "uart3",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX,
                          uart);
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
    uart = &uart4;

    config.baud_rate = BAUD_RATE_115200;

    serial4.ops    = &gd32_uart_ops;
    serial4.config = config;

    NVIC_Configuration(uart);

    /* register UART4 device */
    rt_hw_serial_register(&serial4, "uart4",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX,
                          uart);
#endif /* RT_USING_UART4 */
}


