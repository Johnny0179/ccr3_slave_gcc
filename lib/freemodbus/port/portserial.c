/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

//STM32�������ͷ�ļ�
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
/**
  * @brief  ���ƽ��պͷ���״̬
  * @param  xRxEnable ����ʹ�ܡ�
  *         xTxEnable ����ʹ��
  * @retval None
  */
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
	
	  if(xRxEnable)
  {
    //ʹ�ܽ��պͽ����ж�
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //MAX485���� �͵�ƽΪ����ģʽ
    //GPIO_ResetBits(GPIOG,GPIO_Pin_8);
  }
  else
  {
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); 
    //MAX485���� �ߵ�ƽΪ����ģʽ
    //GPIO_SetBits(GPIOG,GPIO_Pin_8);
  }

  if(xTxEnable)
  {
    //ʹ�ܷ�������ж�
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
  }
  else
  {
    //��ֹ��������ж�
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
  }
}

/**
  * @brief  ���ڳ�ʼ��
  * @param  ucPORT      ���ں�
  *         ulBaudRate  ������
  *         ucDataBits  ����λ
  *         eParity     У��λ 
  * @retval None
  */
BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	(void)ucPORT;     //���޸Ĵ���
  (void)ucDataBits; //���޸�����λ����
  (void)eParity;    //���޸�У���ʽ


   //GPIO�˿�����
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = ulBaudRate;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1
  

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  //�趨USART1 �ж����ȼ�
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //�������485���ͺͽ���ģʽ
/*	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //ʹ��GPIODʱ��
	//GPIOG8��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOG, &GPIO_InitStructure); */

  return TRUE; 
}

/**
  * @brief  ͨ�����ڷ�������
  * @param  None
  * @retval None
  */
BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
	  //��������
  USART_SendData(USART1, ucByte);
    return TRUE;
}

/**
  * @brief  �Ӵ��ڻ������
  * @param  None
  * @retval None
  */
BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	  //��������
  *pucByte = USART_ReceiveData(USART1);
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR( void )
{
	//mb.c eMBInit������
  //pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM 
  //����״̬��
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR( void )
{
	//mb.c eMBInit������
  //pxMBFrameCBByteReceived = xMBRTUReceiveFSM
  //����״̬��
    pxMBFrameCBByteReceived(  );
}


u8 flag_rx = 0;

/**
  * @brief  USART1�жϷ�����
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
  //���������ж�
  if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
  {
    prvvUARTRxISR(); 
    //����жϱ�־λ    
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);   
  }
  
  //��������ж�
  if(USART_GetITStatus(USART1, USART_IT_TC) == SET)
  {
    prvvUARTTxReadyISR();
    //����жϱ�־
    USART_ClearITPendingBit(USART1, USART_IT_TC);
  }
  
  //���Կ��Ƿ����ȥ�� 2012-07-23
  //���-������������Ҫ�ȶ�SR,�ٶ�DR�Ĵ��� �������������жϵ�����
  /*
  if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET)
  {
    USART_ClearFlag(USART1,USART_FLAG_ORE); //��SR
    USART_ReceiveData(USART1);              //��DR
  }
  */
}
