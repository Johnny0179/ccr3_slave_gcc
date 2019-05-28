#ifndef __DELAY_H
#define __DELAY_H

// support cpp
#ifdef __cplusplus
extern "C"
{
#endif

#include <sys.h>

    void delay_init(u8 SYSCLK);
    void delay_us(u32 nus);
    void delay_ms(u32 nms);
    void delay_xms(u32 nms);

#ifdef __cplusplus
}
#endif

#endif