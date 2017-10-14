#include "spi_hard.h"

static rt_err_t configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration);
static rt_uint32_t xfer(struct rt_spi_device* device, struct rt_spi_message* message);

static struct rt_spi_ops gd32_spi_ops =
{
    configure,
    xfer
};

#ifdef USING_SPI1
static struct gd32_spi_bus gd32_spi_bus_1;
#endif /* #ifdef USING_SPI1 */

#ifdef USING_SPI2
static struct gd32_spi_bus gd32_spi_bus_2;
#endif /* #ifdef USING_SPI2 */

#ifdef USING_SPI3
static struct gd32_spi_bus gd32_spi_bus_3;
#endif /* #ifdef USING_SPI3 */

//------------------ DMA ------------------
#ifdef SPI_USE_DMA
static uint8_t dummy = 0xFF;
#endif

#ifdef SPI_USE_DMA
static void DMA_Configuration(struct gd32_spi_bus * gd32_spi_bus, const void * send_addr, void * recv_addr, rt_size_t size)
{
    DMA_InitPara DMA_InitStruct;

    DMA_ClearBitState(gd32_spi_bus->DMA_Channel_RX_FLAG_TC
                  | gd32_spi_bus->DMA_Channel_RX_FLAG_TE
                  | gd32_spi_bus->DMA_Channel_TX_FLAG_TC
                  | gd32_spi_bus->DMA_Channel_TX_FLAG_TE);

    /* RX channel configuration */
    DMA_Enable(gd32_spi_bus->DMA_Channel_RX, DISABLE);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(gd32_spi_bus->SPI->DTR));
    DMA_InitStruct.DMA_DIR = DMA_DIR_PERIPHERALSRC;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PERIPHERALDATASIZE_BYTE;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MEMORYDATASIZE_BYTE;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;

    DMA_InitStruct.DMA_BufferSize = size;

    if(recv_addr != RT_NULL)
    {
        DMA_InitStruct.DMA_MemoryBaseAddr = (u32) recv_addr;
        DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    }
    else
    {
        DMA_InitStruct.DMA_MemoryBaseAddr = (u32) (&dummy);
        DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_DISABLE;
    }

    DMA_Init(gd32_spi_bus->DMA_Channel_RX, &DMA_InitStruct);

    DMA_Enable(gd32_spi_bus->DMA_Channel_RX, ENABLE);

    /* TX channel configuration */
    DMA_Enable(gd32_spi_bus->DMA_Channel_TX, DISABLE);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)(&(gd32_spi_bus->SPI->DTR));
    DMA_InitStruct.DMA_DIR = DMA_DIR_PERIPHERALDST;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PERIPHERALDATASIZE_BYTE;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MEMORYDATASIZE_BYTE;
    DMA_InitStruct.DMA_Priority = DMA_PRIORITY_MEDIUM;
    DMA_InitStruct.DMA_Mode = DMA_MODE_NORMAL;
    DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;

    DMA_InitStruct.DMA_BufferSize = size;

    if(send_addr != RT_NULL)
    {
        DMA_InitStruct.DMA_MemoryBaseAddr = (u32)send_addr;
        DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    }
    else
    {
        DMA_InitStruct.DMA_MemoryBaseAddr = (u32)(&dummy);;
        DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_DISABLE;
    }

    DMA_Init(gd32_spi_bus->DMA_Channel_TX, &DMA_InitStruct);

    DMA_Enable(gd32_spi_bus->DMA_Channel_TX, ENABLE);
}
#endif

rt_inline uint16_t get_spi_BaudRatePrescaler(rt_uint32_t max_hz)
{
    uint16_t SPI_BaudRatePrescaler;

    /* gd32F10x SPI MAX 18Mhz */
    if(max_hz >= SystemCoreClock/2 && SystemCoreClock/2 <= 18000000)
    {
        SPI_BaudRatePrescaler = SPI_PSC_2;
    }
    else if(max_hz >= SystemCoreClock/4)
    {
        SPI_BaudRatePrescaler = SPI_PSC_4;
    }
    else if(max_hz >= SystemCoreClock/8)
    {
        SPI_BaudRatePrescaler = SPI_PSC_8;
    }
    else if(max_hz >= SystemCoreClock/16)
    {
        SPI_BaudRatePrescaler = SPI_PSC_16;
    }
    else if(max_hz >= SystemCoreClock/32)
    {
        SPI_BaudRatePrescaler = SPI_PSC_32;
    }
    else if(max_hz >= SystemCoreClock/64)
    {
        SPI_BaudRatePrescaler = SPI_PSC_64;
    }
    else if(max_hz >= SystemCoreClock/128)
    {
        SPI_BaudRatePrescaler = SPI_PSC_128;
    }
    else
    {
        /* min prescaler 256 */
        SPI_BaudRatePrescaler = SPI_PSC_256;
    }

    return SPI_BaudRatePrescaler;
}

static rt_err_t configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration)
{
    struct gd32_spi_bus * gd32_spi_bus = (struct gd32_spi_bus *)device->bus;
    SPI_InitPara SPI_InitStruct;

    SPI_ParaInit(&SPI_InitStruct);

    /* data_width */
    if(configuration->data_width <= 8)
    {
        SPI_InitStruct.SPI_FrameFormat = SPI_FRAMEFORMAT_8BIT;
    }
    else if(configuration->data_width <= 16)
    {
        SPI_InitStruct.SPI_FrameFormat = SPI_FRAMEFORMAT_16BIT;
    }
    else
    {
        return RT_EIO;
    }
    /* baudrate */
    SPI_InitStruct.SPI_PSC = get_spi_BaudRatePrescaler(configuration->max_hz);
    /* CPOL */
    if(configuration->mode & RT_SPI_CPOL)
    {
        SPI_InitStruct.SPI_SCKPL = SPI_SCKPL_HIGH;
    }
    else
    {
        SPI_InitStruct.SPI_SCKPL = SPI_SCKPL_LOW;
    }
    /* CPHA */
    if(configuration->mode & RT_SPI_CPHA)
    {
        SPI_InitStruct.SPI_SCKPH = SPI_SCKPH_2EDGE;
    }
    else
    {
        SPI_InitStruct.SPI_SCKPH = SPI_SCKPH_1EDGE;
    }
    /* MSB or LSB */
    if(configuration->mode & RT_SPI_MSB)
    {
        SPI_InitStruct.SPI_FirstBit = SPI_FIRSTBIT_MSB;
    }
    else
    {
        SPI_InitStruct.SPI_FirstBit = SPI_FIRSTBIT_LSB;
    }
    SPI_InitStruct.SPI_TransType = SPI_TRANSTYPE_FULLDUPLEX;
    SPI_InitStruct.SPI_Mode = SPI_MODE_MASTER;
    SPI_InitStruct.SPI_SWNSSEN  = SPI_SWNSS_SOFT;

    /* init SPI */
    SPI_I2S_DeInit(gd32_spi_bus->SPI);
    SPI_Init(gd32_spi_bus->SPI, &SPI_InitStruct);
    /* Enable SPI_MASTER */
    SPI_Enable(gd32_spi_bus->SPI, ENABLE);
    SPI_Enable(gd32_spi_bus->SPI, DISABLE);

    return RT_EOK;
};

static rt_uint32_t xfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
    struct gd32_spi_bus * gd32_spi_bus = (struct gd32_spi_bus *)device->bus;
    struct rt_spi_configuration * config = &device->config;
    SPI_TypeDef * SPI = gd32_spi_bus->SPI;
    struct gd32_spi_cs * gd32_spi_cs = device->parent.user_data;
    rt_uint32_t size = message->length;

    /* take CS */
    if(message->cs_take)
    {
        GPIO_ResetBits(gd32_spi_cs->GPIOx, gd32_spi_cs->GPIO_Pin);
    }

#ifdef SPI_USE_DMA
    if(message->length > 32)
    {
        if(config->data_width <= 8)
        {
            DMA_Configuration(gd32_spi_bus, message->send_buf, message->recv_buf, message->length);
            SPI_I2S_DMA_Enable(SPI, SPI_I2S_DMA_TX | SPI_I2S_DMA_RX, ENABLE);
            while (DMA_GetBitState(gd32_spi_bus->DMA_Channel_RX_FLAG_TC) == RESET
                    || DMA_GetBitState(gd32_spi_bus->DMA_Channel_TX_FLAG_TC) == RESET);
            SPI_I2S_DMA_Enable(SPI, SPI_I2S_DMA_TX | SPI_I2S_DMA_RX, DISABLE);
        }
//        rt_memcpy(buffer,_spi_flash_buffer,DMA_BUFFER_SIZE);
//        buffer += DMA_BUFFER_SIZE;
    }
    else
#endif
    {
        if(config->data_width <= 8)
        {
            const rt_uint8_t * send_ptr = message->send_buf;
            rt_uint8_t * recv_ptr = message->recv_buf;

            while(size--)
            {
                rt_uint8_t data = 0xFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while (SPI_I2S_GetBitState(SPI, SPI_FLAG_TBE) == RESET);
                // Send the byte
                SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetBitState(SPI, SPI_FLAG_RBNE) == RESET);
                // Get the received data
                data = SPI_I2S_ReceiveData(SPI);

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
        else if(config->data_width <= 16)
        {
            const rt_uint16_t * send_ptr = message->send_buf;
            rt_uint16_t * recv_ptr = message->recv_buf;

            while(size--)
            {
                rt_uint16_t data = 0xFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }

                //Wait until the transmit buffer is empty
                while (SPI_I2S_GetBitState(SPI, SPI_FLAG_TBE) == RESET);
                // Send the byte
                SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
                while (SPI_I2S_GetBitState(SPI, SPI_FLAG_RBNE) == RESET);
                // Get the received data
                data = SPI_I2S_ReceiveData(SPI);

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
    }

    /* release CS */
    if(message->cs_release)
    {
        GPIO_SetBits(gd32_spi_cs->GPIOx, gd32_spi_cs->GPIO_Pin);
    }

    return message->length;
};

/** \brief init and register gd32 spi bus.
 *
 * \param SPI: gd32 SPI, e.g: SPI1,SPI2,SPI3.
 * \param gd32_spi: gd32 spi bus struct.
 * \param spi_bus_name: spi bus name, e.g: "spi1"
 * \return
 *
 */
rt_err_t gd32_spi_register(SPI_TypeDef * SPI,
                            struct gd32_spi_bus * gd32_spi,
                            const char * spi_bus_name)
{
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_AF, ENABLE);

    if(SPI == SPI1)
    {
    	gd32_spi->SPI = SPI1;
#ifdef SPI_USE_DMA
        /* Enable the DMA1 Clock */
        RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_DMA1, ENABLE);

        gd32_spi->DMA_Channel_RX = DMA1_CHANNEL2;
        gd32_spi->DMA_Channel_TX = DMA1_CHANNEL3;
        gd32_spi->DMA_Channel_RX_FLAG_TC = DMA1_FLAG_TC2;
        gd32_spi->DMA_Channel_RX_FLAG_TE = DMA1_FLAG_ERR2;
        gd32_spi->DMA_Channel_TX_FLAG_TC = DMA1_FLAG_TC3;
        gd32_spi->DMA_Channel_TX_FLAG_TE = DMA1_FLAG_ERR3;
#endif
        RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_SPI1, ENABLE);
    }
    else if(SPI == SPI2)
    {
        gd32_spi->SPI = SPI2;
#ifdef SPI_USE_DMA
        /* Enable the DMA1 Clock */
        RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_DMA1, ENABLE);

        gd32_spi->DMA_Channel_RX = DMA1_CHANNEL4;
        gd32_spi->DMA_Channel_TX = DMA1_CHANNEL5;
        gd32_spi->DMA_Channel_RX_FLAG_TC = DMA1_FLAG_TC4;
        gd32_spi->DMA_Channel_RX_FLAG_TE = DMA1_FLAG_ERR4;
        gd32_spi->DMA_Channel_TX_FLAG_TC = DMA1_FLAG_TC5;
        gd32_spi->DMA_Channel_TX_FLAG_TE = DMA1_FLAG_ERR5;
#endif
        RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_SPI2, ENABLE);
    }
    else if(SPI == SPI3)
    {
    	gd32_spi->SPI = SPI3;
#ifdef SPI_USE_DMA
        /* Enable the DMA2 Clock */
        RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_DMA2, ENABLE);

        gd32_spi->DMA_Channel_RX = DMA2_CHANNEL1;
        gd32_spi->DMA_Channel_TX = DMA2_CHANNEL2;
        gd32_spi->DMA_Channel_RX_FLAG_TC = DMA2_FLAG_TC1;
        gd32_spi->DMA_Channel_RX_FLAG_TE = DMA2_FLAG_TE1;
        gd32_spi->DMA_Channel_TX_FLAG_TC = DMA2_FLAG_TC2;
        gd32_spi->DMA_Channel_TX_FLAG_TE = DMA2_FLAG_TE2;
#endif
        RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_SPI3, ENABLE);
    }
    else
    {
        return RT_ENOSYS;
    }

    return rt_spi_bus_register(&gd32_spi->parent, spi_bus_name, &gd32_spi_ops);
}
