#include "delay.h"
#include "sys.h"

#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h" //FreeRTOSÊ¹ÓÃ
#include "task.h"
#endif

static u8 fac_us = 0;  //usÑÓÊ±±¶³ËÊý
static u16 fac_ms = 0; //msÑÓÊ±±¶³ËÊý,ÔÚosÏÂ,Žú±íÃ¿žöœÚÅÄµÄmsÊý

extern void xPortSysTickHandler(void);

void SysTick_Handler(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) //ÏµÍ³ÒÑŸ­ÔËÐÐ
    {
        xPortSysTickHandler();
    }
}

void delay_init(u8 SYSCLK)
{
    u32 reload;
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    fac_us = SYSCLK;                           //²»ÂÛÊÇ·ñÊ¹ÓÃOS,fac_us¶ŒÐèÒªÊ¹ÓÃ
    reload = SYSCLK;                           //Ã¿ÃëÖÓµÄŒÆÊýŽÎÊý µ¥Î»ÎªM
    reload *= 1000000 / configTICK_RATE_HZ;    //žùŸÝconfigTICK_RATE_HZÉè¶šÒç³öÊ±Œä
                                               //reloadÎª24Î»ŒÄŽæÆ÷,×îŽóÖµ:16777216,ÔÚ168MÏÂ,ÔŒºÏ0.0998s×óÓÒ
    fac_ms = 1000 / configTICK_RATE_HZ;        //Žú±íOS¿ÉÒÔÑÓÊ±µÄ×îÉÙµ¥Î»
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; //¿ªÆôSYSTICKÖÐ¶Ï
    SysTick->LOAD = reload;                    //Ã¿1/configTICK_RATE_HZ¶ÏÒ»ŽÎ
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  //¿ªÆôSYSTICK
}

void delay_us(u32 nus)
{
    u32 ticks;
    u32 told, tnow, tcnt = 0;
    u32 reload = SysTick->LOAD; //LOADµÄÖµ
    ticks = nus * fac_us;       //ÐèÒªµÄœÚÅÄÊý
    told = SysTick->VAL;        //žÕœøÈëÊ±µÄŒÆÊýÆ÷Öµ
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow; //ÕâÀï×¢ÒâÒ»ÏÂSYSTICKÊÇÒ»žöµÝŒõµÄŒÆÊýÆ÷ŸÍ¿ÉÒÔÁË.
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks)
                break; //Ê±Œä³¬¹ý/µÈÓÚÒªÑÓ³ÙµÄÊ±Œä,ÔòÍË³ö.
        }
    };
}

void delay_ms(u32 nms)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) //ÏµÍ³ÒÑŸ­ÔËÐÐ
    {
        if (nms >= fac_ms) //ÑÓÊ±µÄÊ±ŒäŽóÓÚOSµÄ×îÉÙÊ±ŒäÖÜÆÚ
        {
            vTaskDelay(nms / fac_ms); //FreeRTOSÑÓÊ±
        }
        nms %= fac_ms; //OSÒÑŸ­ÎÞ·šÌá¹©ÕâÃŽÐ¡µÄÑÓÊ±ÁË,²ÉÓÃÆÕÍš·œÊœÑÓÊ±
    }
    delay_us((u32)(nms * 1000)); //ÆÕÍš·œÊœÑÓÊ±
}

void delay_xms(u32 nms)
{
    u32 i;
    for (i = 0; i < nms; i++)
        delay_us(1000);
}
