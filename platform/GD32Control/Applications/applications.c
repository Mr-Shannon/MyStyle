#include <board.h>
#include <rtthread.h>
#include "led.h"

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

/**
* This is RT-Thread  application initialize.
*
*/
int rt_application_init(void)
{
  rt_err_t result;

  /* init led1 thread */
  result = rt_thread_init(&led1_thread, "led1", led1_thread_entry,  RT_NULL,
                         (rt_uint8_t*)&led1_stack[0],sizeof(led1_stack), 20, 5);
  if (result == RT_EOK)
    rt_thread_startup(&led1_thread);
  else 
    rt_thread_detach (&led1_thread);

  /* init led2 thread */
  result = rt_thread_init(&led2_thread, "led2", led2_thread_entry,RT_NULL,
                         (rt_uint8_t*)&led2_stack[0], sizeof(led2_stack), 10, 5);
  if (result == RT_EOK)
    rt_thread_startup(&led2_thread);  
  else 
    rt_thread_detach (&led2_thread);

  /* init led3 thread */
  led3_thread = rt_thread_create("led3", led3_thread_entry,(void*)3, 256, 25, 5); 
                
  if(led3_thread != RT_NULL)
    rt_thread_startup(led3_thread);
  else 
    rt_thread_delete(led3_thread);

  return 0;
}
