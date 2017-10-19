#include <board.h>
#include <rtthread.h>
#include "led.h"
#include "enc28j60.h"

ALIGN(4)
static rt_uint8_t led1_stack[512];
static struct rt_thread led1_thread;
static void led1_thread_entry(void* parameter)
{
  led_init();
  
  while (1)
  {
    /* led1 on */
    led_on(0);
    rt_thread_delay( RT_TICK_PER_SECOND/2 ); /* sleep 0.5 second and switch to other thread */
    
    /* led1 off */
    led_off(0);
    rt_thread_delay( RT_TICK_PER_SECOND/2 );
  }
}

ALIGN(4)
static rt_uint8_t led2_stack[256];
static struct rt_thread led2_thread;
static void led2_thread_entry(void* parameter)
{
  led_init();

  while (1)
  {
    /* led1 on */
    led_on(1);
    rt_thread_delay( RT_TICK_PER_SECOND/4 ); /* sleep 0.25 second and switch to other thread */

    /* led1 off */
    led_off(1);
    rt_thread_delay( RT_TICK_PER_SECOND/4 );
  }
}

static rt_thread_t led3_thread = RT_NULL;
static void led3_thread_entry(void* parameter)
{
  led_init();

  while (1)
  {
    /* led2 on */
    led_on(2);
    rt_thread_delay( RT_TICK_PER_SECOND); /* sleep 1 second and switch to other thread */

    /* led2 off */
    led_off(2);
    rt_thread_delay( RT_TICK_PER_SECOND);
  }
}

void rt_init_thread_entry(void* parameter)
{
    /* initialize enc28j60 */
    {
        NVIC_InitPara NVIC_InitStruct;
    
        // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
        /* Enable the EXTI1 Interrupt */
        NVIC_InitStruct.NVIC_IRQ = EXTI1_IRQn;
        NVIC_InitStruct.NVIC_IRQPreemptPriority = 0;
        NVIC_InitStruct.NVIC_IRQSubPriority = 1;
        NVIC_InitStruct.NVIC_IRQEnable = ENABLE;	
        NVIC_Init(&NVIC_InitStruct);
        
        GPIO_InitPara GPIO_InitStruct;
        EXTI_InitPara EXTI_InitStruct;
        
        RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_GPIOA | RCC_APB2PERIPH_AF, 
                               ENABLE);
        GPIO_EXTILineConfig(GPIO_PORT_SOURCE_GPIOA, GPIO_PINSOURCE1);
        
        /* PA1 IPU */
        GPIO_InitStruct.GPIO_Pin = GPIO_PIN_1;	            
        GPIO_InitStruct.GPIO_Speed = GPIO_SPEED_10MHZ;
        GPIO_InitStruct.GPIO_Mode = GPIO_MODE_IPU;
        GPIO_Init(GPIOA, &GPIO_InitStruct);	
        GPIO_EXTILineConfig(GPIO_PORT_SOURCE_GPIOA, GPIO_PINSOURCE1);
    
        /* EXTI1 */
        EXTI_InitStruct.EXTI_LINE = EXTI_LINE1;
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_InitStruct.EXTI_LINEEnable = ENABLE;
        EXTI_Init(&EXTI_InitStruct);
    
        EXTI_ClearIntBitState(EXTI_LINE1);
    }
    enc28j60_attach("spi10");
    rt_kprintf("stm32 register spi bus device spi10\n");
}

/**
* This is RT-Thread  application initialize.
*
*/
int rt_application_init(void)
{
  rt_err_t result;
  rt_thread_t init_thread;

  /* init led1 thread */
  result = rt_thread_init(&led1_thread, 
                          "led1", 
                          led1_thread_entry,  
                          RT_NULL,
                         (rt_uint8_t*)&led1_stack[0],
                          sizeof(led1_stack), 
                          20, 
                          5);
  if (result == RT_EOK)
    rt_thread_startup(&led1_thread);
  else 
    rt_thread_detach (&led1_thread);

  /* init led2 thread */
  result = rt_thread_init(&led2_thread, 
                          "led2", 
                          led2_thread_entry,
                          RT_NULL,
                         (rt_uint8_t*)&led2_stack[0], 
                          sizeof(led2_stack), 
                          10, 
                          5);
  if (result == RT_EOK)
    rt_thread_startup(&led2_thread);  
  else 
    rt_thread_detach (&led2_thread);

  /* init led3 thread */
  led3_thread = rt_thread_create("led3", 
                                  led3_thread_entry,
                                  (void*)3, 
                                  256, 
                                  25, 
                                  5); 
                
  if(led3_thread != RT_NULL)
    rt_thread_startup(led3_thread);
  else 
    rt_thread_delete(led3_thread);
  
  init_thread = rt_thread_create("init",
                                  rt_init_thread_entry, 
                                  RT_NULL,
                                  2048, 
                                  8, 
                                  20);
  if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

  return 0;
}
