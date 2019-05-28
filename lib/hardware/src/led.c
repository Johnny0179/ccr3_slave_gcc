#include "led.h"

//LED对应IO初始化
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); //使能GPIOE时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //LED0、LED1和LED2对应IO口
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;          //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;     //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);                 //初始化GPIO

    GPIO_SetBits(GPIOE, GPIO_Pin_2 | GPIO_Pin_3);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); //使能GPIOF时钟
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;             //LED0、LED1和LED2对应IO口
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;         //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          //上拉
    GPIO_Init(GPIOF, &GPIO_InitStructure);                //初始化GPIO

    GPIO_SetBits(GPIOF, GPIO_Pin_9);
}