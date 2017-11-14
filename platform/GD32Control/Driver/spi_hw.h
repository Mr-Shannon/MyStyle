#ifndef __SPI_HW_H__
#define __SPI_HW_H__

#include <rtdevice.h>

#include "gd32f10x.h"
#include "gd32f10x_spi.h"

#include "board.h"

//#define SPI_USE_DMA

struct gd32_spi_bus
{
    struct rt_spi_bus parent;
    SPI_TypeDef * SPI;
#ifdef SPI_USE_DMA
    DMA_Channel_TypeDef * DMA_Channel_TX;
    DMA_Channel_TypeDef * DMA_Channel_RX;
    uint32_t DMA_Channel_TX_FLAG_TC;
    uint32_t DMA_Channel_TX_FLAG_TE;
    uint32_t DMA_Channel_RX_FLAG_TC;
    uint32_t DMA_Channel_RX_FLAG_TE;
#endif /* SPI_USE_DMA */
};

struct gd32_spi_cs
{
    GPIO_TypeDef * GPIOx;
    uint16_t GPIO_Pin;
};


/* public function list */
rt_err_t gd32_spi_register(SPI_TypeDef * SPI,
                            struct gd32_spi_bus * gd32_spi,
                            const char * spi_bus_name);
                            
void rt_hw_spi_init(void);

#endif // gd32_SPI_H_INCLUDED
