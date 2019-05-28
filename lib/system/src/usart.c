#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//Èç¹ûÊ¹ÓÃucos,Ôò°üÀšÏÂÃæµÄÍ·ÎÄŒþŒŽ¿É.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOSÊ¹ÓÃ	  
#endif

//////////////////////////////////////////////////////////////////
//ŒÓÈëÒÔÏÂŽúÂë,Ö§³Öprintfº¯Êý,¶ø²»ÐèÒªÑ¡Ôñuse MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//±ê×Œ¿âÐèÒªµÄÖ§³Öº¯Êý                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//¶šÒå_sys_exit()ÒÔ±ÜÃâÊ¹ÓÃ°ëÖ÷»úÄ£Êœ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//ÖØ¶šÒåfputcº¯Êý 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//Ñ­»··¢ËÍ,Ö±µœ·¢ËÍÍê±Ï   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //Èç¹ûÊ¹ÄÜÁËœÓÊÕ
//Ž®¿Ú1ÖÐ¶Ï·þÎñ³ÌÐò
//×¢Òâ,¶ÁÈ¡USARTx->SRÄÜ±ÜÃâÄªÃûÆäÃîµÄŽíÎó   	
u8 USART_RX_BUF[USART_REC_LEN];     //œÓÊÕ»º³å,×îŽóUSART_REC_LENžö×ÖœÚ.
//œÓÊÕ×ŽÌ¬
//bit15£¬	œÓÊÕÍê³É±êÖŸ
//bit14£¬	œÓÊÕµœ0x0d
//bit13~0£¬	œÓÊÕµœµÄÓÐÐ§×ÖœÚÊýÄ¿
u16 USART_RX_STA=0;       //œÓÊÕ×ŽÌ¬±êŒÇ	

//³õÊŒ»¯IO Ž®¿Ú1 
//bound:²šÌØÂÊ
void uart_init(u32 bound){
	//GPIO¶Ë¿ÚÉèÖÃ
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //Ê¹ÄÜGPIOAÊ±ÖÓ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//Ê¹ÄÜUSART1Ê±ÖÓ
 
	//Ž®¿Ú1¶ÔÓŠÒýœÅžŽÓÃÓ³Éä
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9žŽÓÃÎªUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10žŽÓÃÎªUSART1
	
	//USART1¶Ë¿ÚÅäÖÃ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9ÓëGPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//žŽÓÃ¹ŠÄÜ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//ËÙ¶È50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //ÍÆÍìžŽÓÃÊä³ö
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //ÉÏÀ­
	GPIO_Init(GPIOA,&GPIO_InitStructure); //³õÊŒ»¯PA9£¬PA10

   //USART1 ³õÊŒ»¯ÉèÖÃ
	USART_InitStructure.USART_BaudRate = bound;//²šÌØÂÊÉèÖÃ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//×Ö³€Îª8Î»ÊýŸÝžñÊœ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//Ò»žöÍ£Ö¹Î»
	USART_InitStructure.USART_Parity = USART_Parity_No;//ÎÞÆæÅŒÐ£ÑéÎ»
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ÎÞÓ²ŒþÊýŸÝÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//ÊÕ·¢Ä£Êœ
	USART_Init(USART1, &USART_InitStructure); //³õÊŒ»¯Ž®¿Ú1
	
	USART_Cmd(USART1, ENABLE);  //Ê¹ÄÜŽ®¿Ú1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//¿ªÆôÏà¹ØÖÐ¶Ï

	//Usart1 NVIC ÅäÖÃ
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//Ž®¿Ú1ÖÐ¶ÏÍšµÀ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//ÇÀÕŒÓÅÏÈŒ¶3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//×ÓÓÅÏÈŒ¶3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÍšµÀÊ¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);	//žùŸÝÖž¶šµÄ²ÎÊý³õÊŒ»¯VICŒÄŽæÆ÷¡¢

#endif
	
}


void USART1_IRQHandler(void)                	//Ž®¿Ú1ÖÐ¶Ï·þÎñ³ÌÐò
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //œÓÊÕÖÐ¶Ï(œÓÊÕµœµÄÊýŸÝ±ØÐëÊÇ0x0d 0x0aœáÎ²)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//¶ÁÈ¡œÓÊÕµœµÄÊýŸÝ
		
		if((USART_RX_STA&0x8000)==0)//œÓÊÕÎŽÍê³É
		{
			if(USART_RX_STA&0x4000)//œÓÊÕµœÁË0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//œÓÊÕŽíÎó,ÖØÐÂ¿ªÊŒ
				else USART_RX_STA|=0x8000;	//œÓÊÕÍê³ÉÁË 
			}
			else //»¹Ã»ÊÕµœ0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//œÓÊÕÊýŸÝŽíÎó,ÖØÐÂ¿ªÊŒœÓÊÕ	  
				}		 
			}
		}   		 
	} 
} 
#endif	

 


