#ifndef _CAN_H_
#define _CAN_H_

// support cpp
#ifdef __cplusplus
extern "C"
{
#endif

#include "sys.h"

//CAN1æ¥æ¶RX0äž?æ?äœ¿èœ
#define CAN1_RX0_INT_ENABLE 0  //0,äžäœ¿è?;1,äœ¿èœ.
#define CAN_BAUD_500Kbps_brp 6 //æ³¢ç¹ç?=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
#define CAN_BAUD_1Mbps_brp 3

/*  Function Codes 
   ---------------
  defined in the canopen DS301 
*/
#define NMT ((u16)0x0 << 7)
#define SYNC ((u16)0x1 << 7)
#define TIME_STAMP ((u16)0x2 << 7)
#define PDO1tx ((u16)0x3 << 7)
#define PDO1rx ((u16)0x4 << 7)
#define PDO2tx ((u16)0x5 << 7)
#define PDO2rx ((u16)0x6 << 7)
#define PDO3tx ((u16)0x7 << 7)
#define PDO3rx ((u16)0x8 << 7)
#define PDO4tx ((u16)0x9 << 7)
#define PDO4rx ((u16)0xA << 7)
#define SDOtx ((u16)0xB << 7)
#define SDOrx ((u16)0xC << 7)
#define NODE_GUARD ((u16)(0xE) << 7)
#define LSS ((u16)0xF << 7)

/* NMT Command Specifier, sent by master to change a slave state */
/* ------------------------------------------------------------- */
/* Should not be modified */
#define NMT_Start_Node 0x01
#define NMT_Stop_Node 0x02
#define NMT_Enter_PreOperational 0x80
#define NMT_Reset_Node 0x81
#define NMT_Reset_Comunication 0x82

#define Rd_STA_TRQ_POS RX_PDO1
#define Rd_STA_TRQ_SPD RX_PDO2
#define Rd_SPD_POS RX_PDO3
#define Rd_STA_ERR_TRQ_MODE RX_PDO4

#define NULL ((void *)0)
    //CANåå?å
    u8 CAN1_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode);
    //åéæ°æ?
    u8 CAN1_Send_Frame(u16 CobId, u8 Rtr, u8 Len, u8 *msg);
    //æ¥æ¶æ°æ®
    u8 CAN1_Receive_Msg(u8 *buf);
    void CanInit(void);

    u8 Sdo_WrU32(u8 SlaveID, u16 index, u8 subindex, u32 data);
    u8 Sdo_WrU16(u8 SlaveID, u16 index, u8 subindex, u32 data);
    u8 Sdo_WrU8(u8 SlaveID, u16 index, u8 subindex, u32 data);

    u8 NMT_Start(u8 SlaveID);
    u8 NMT_Stop(u8 SlaveID);
    u8 NMT_PreSTA(u8 SlaveID);
    u8 NMT_RstNode(u8 SlaveID);
    u8 NMT_RstComm(u8 SlaveID);

    u8 SetMotorCtrlword(u8 SlaveID, u16 Ctrlword);
    u8 TX_PDO1(u8 SlaveID, u16 CtrlWord);
    u8 TX_PDO2(u8 SlaveID, u16 CtrlWord, u32 pos);
    u8 TX_PDO3(u8 SlaveID, u32 speed);
    // u8 TX_PDO4(u8 SlaveID, u16 max_current_limit);

    u8 TX_PDO4(u8 SlaveID, u8 mode);

    u8 RX_PDO1(u8 SlaveID);
    void StartMotor(u8 SlaveID);

// support cpp
#ifdef __cplusplus
}
#endif

#endif