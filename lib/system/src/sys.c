#include "sys.h"  

 void WFI_SET(void)
{
    __asm__(
	"WFI"
    );		  
}

 void INTX_DISABLE(void)
{
     __asm__(
	"CPSID   I \n"
	"BX      LR \n"
    );	  
}

void INTX_ENABLE(void)
{
     __asm__(
	"CPSIE   I \n"
	"BX      LR \n"
    );  
}

void MSR_MSP(u32 addr) 
{
     __asm__(
	"MSR MSP, r0 \n" 			//set Main Stack value
	"BX r14 \n"
    );
}















