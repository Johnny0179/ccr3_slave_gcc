#ifndef __LED_H
#define __LED_H

// support cpp
#ifdef __cplusplus
extern "C"
{
#endif

#include "sys.h"
//LED端口定义
#define LED0 PEout(2)
#define LED1 PEout(3)
#define LED2 PFout(9)

//初始化
void LED_Init(void);

#ifdef __cplusplus
}
#endif

#endif