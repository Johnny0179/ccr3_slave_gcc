#include "can.h"
#include "delay.h"
#include "sys.h"

// CAN初始化
// tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
// tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
// tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
// brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
// mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
// Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//则波特率为:42M/((6+7+1)*6)=500Kbps
//返回值:0,初始化OK;
//    其他,初始化失败;

u8 CAN1_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //使能相关时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能PORTA时钟

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); //使能CAN1时钟

    //初始化GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       //复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);             //初始化PA11,PA12

    //引脚复用映射配置
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1); // GPIOA11复用为CAN1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1); // GPIOA12复用为CAN1

    // CAN单元设置
    CAN_InitStructure.CAN_TTCM = DISABLE; //非时间触发通信模式
    CAN_InitStructure.CAN_ABOM = DISABLE; //软件自动离线管理
    CAN_InitStructure.CAN_AWUM =
        DISABLE;                          //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_InitStructure.CAN_NART = ENABLE;  //禁止报文自动传送
    CAN_InitStructure.CAN_RFLM = DISABLE; //报文不锁定,新的覆盖旧的
    CAN_InitStructure.CAN_TXFP = DISABLE; //优先级由报文标识符决定
    CAN_InitStructure.CAN_Mode = mode;    //模式设置
    CAN_InitStructure.CAN_SJW =
        tsjw;                              //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1 = tbs1;      // Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2 = tbs2;      // Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler = brp; //分频系数(Fdiv)为brp+1
    CAN_Init(CAN1, &CAN_InitStructure);    // 初始化CAN1

    //配置过滤器
    CAN_FilterInitStructure.CAN_FilterNumber = 0; //过滤器0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; // 32位
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;               ////32位ID
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000; // 32位MASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment =
        CAN_Filter_FIFO0;                                  //过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //激活过滤器0
    CAN_FilterInit(&CAN_FilterInitStructure);              //滤波器初始化

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); // FIFO0消息挂号中断允许.
    // 后面有空配置一下发送邮箱为空的中断。
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 主优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

#if 1
    CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE); // FIFO0消息挂号中断允许.
    // 后面有空配置一下发送邮箱为空的中断。
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 主优先级为2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

    return 0;
}

//中断服务函数
void CAN1_RX0_IRQHandler(void)
{

    CanRxMsg RxMessage;
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_Receive(CAN1, 0, &RxMessage);
        // canDispatch(&RxMessage);
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

#if 1
void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
    }
}
#endif

// printf("StdId:%x\r\n",RxMessage.StdId);
//	for(i=0;i<8;i++)
//	printf("rxbuf[%d]:%x\r\n",i,RxMessage.Data[i]);
// can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)
// len:数据长度(最大为8)
// msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
//死等发完
u8 CAN1_Send_Frame(u16 CobId, u8 Rtr, u8 Len, u8 *msg)
{
    u8 mbox;
    // u8 err;
    u16 i;
    CanTxMsg TxMessage;

    TxMessage.StdId = CobId;    // 标准标识符,0x601,0x581
    TxMessage.ExtId = 0;        // 0x12;	 // 设置扩展标示符（29位）,暂时没用
    TxMessage.IDE = CAN_ID_STD; // 标准帧，不使用扩展标识符

    if (Rtr == 0)
        TxMessage.RTR = CAN_RTR_DATA; ////;		  //
                                      ///消息类型为数据帧，或
    else
        TxMessage.RTR = CAN_RTR_Remote; // RTR 请求帧

    TxMessage.DLC = Len; // 发送两帧信息
    for (i = 0; i < Len; i++)
        TxMessage.Data[i] = msg[i]; // 第一帧信息

    mbox = CAN_Transmit(CAN1, &TxMessage);

#if 1
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //等待发送结束
    if (i >= 0XFFF)
        return 1;
    return 0;
#else
    if (mbox == CAN_TxStatus_NoMailBox)
        return 1; // err //还可以设置自动重传
    else
        return 0; // OK

#endif
}

u8 CAN1_Receive_Msg(u8 *buf)
{
    u32 i;
    CanRxMsg RxMessage;
    if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0)
        return 0; //没有接收到数据,直接退出

    // printf("rx data!");
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage); //读取数据
    for (i = 0; i < RxMessage.DLC; i++)
        buf[i] = RxMessage.Data[i];
    return RxMessage.DLC;
}

void CanInit(void)
{
    CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_6tq, CAN_BS1_7tq, CAN_BAUD_1Mbps_brp,
                   CAN_Mode_Normal); // brp 需设置
}

//写4字节
u8 Sdo_WrU32(u8 SlaveID, u16 index, u8 subindex, u32 data)
{
    u8 msg[8];
    msg[0] = 0x23; //区别在这里
    msg[1] = index & 0xff;
    msg[2] = (index >> 8) & 0xff;
    msg[3] = subindex;
    msg[4] = data & 0xff;
    msg[5] = (data >> 8) & 0xff;
    msg[6] = (data >> 16) & 0xff;
    msg[7] = (data >> 24) & 0xff;
    return CAN1_Send_Frame((SDOrx + SlaveID), 0, 8,
                           msg); // SDOrx为0x600,这个主要是对主站来说,所以用0x600
}

//写2字节
u8 Sdo_WrU16(u8 SlaveID, u16 index, u8 subindex, u32 data)
{
    u8 msg[8];
    msg[0] = 0x2B; //区别在这里
    msg[1] = index & 0xff;
    msg[2] = (index >> 8) & 0xff;
    msg[3] = subindex;
    msg[4] = data & 0xff;
    msg[5] = (data >> 8) & 0xff;
    msg[6] = msg[7] = 0;
    return CAN1_Send_Frame((SDOrx + SlaveID), 0, 8,
                           msg); // SDOrx为0x600,这个主要是对主站来说,所以用0x600
}

//写1字节
u8 Sdo_WrU8(u8 SlaveID, u16 index, u8 subindex, u32 data)
{
    u8 msg[8];
    msg[0] = 0x2F; //区别在这里
    msg[1] = index & 0xff;
    msg[2] = (index >> 8) & 0xff;
    msg[3] = subindex;
    msg[4] = data & 0xff;
    msg[5] = msg[6] = msg[7] = 0;
    return CAN1_Send_Frame(
        (SDOrx + SlaveID), 0, 8,
        msg); // SDOrx为0x600,这个主要是对主站来说,所以用0x600 ,从站应答
}

u8 NMT_Start(u8 SlaveID)
{
    u8 msg[8];
    msg[0] = NMT_Start_Node;
    msg[1] = SlaveID;
    return CAN1_Send_Frame(NMT, 0, 2, msg);
}

u8 NMT_Stop(u8 SlaveID)
{
    u8 msg[8];
    msg[0] = NMT_Stop_Node;
    msg[1] = SlaveID;
    return CAN1_Send_Frame(NMT, 0, 2, msg);
}

u8 NMT_PreSTA(u8 SlaveID)
{
    u8 msg[8];
    msg[0] = NMT_Enter_PreOperational;
    msg[1] = SlaveID;
    return CAN1_Send_Frame(NMT, 0, 2, msg);
}

u8 NMT_RstNode(u8 SlaveID)
{
    u8 msg[8];
    msg[0] = NMT_Reset_Node;
    msg[1] = SlaveID;
    return CAN1_Send_Frame(NMT, 0, 2, msg);
}

u8 NMT_RstComm(u8 SlaveID)
{
    u8 msg[8];
    msg[0] = NMT_Reset_Comunication;
    msg[1] = SlaveID;
    return CAN1_Send_Frame(NMT, 0, 2, msg);
}

u8 SetMotorCtrlword(u8 SlaveID, u16 Ctrlword)
{
    return TX_PDO1(SlaveID, Ctrlword);
}

u8 TX_PDO1(u8 SlaveID, u16 CtrlWord)
{
    //控制字CtrlWord
    u8 msg[8];
    msg[0] = CtrlWord & 0xff;
    msg[1] = (CtrlWord >> 8) & 0xff;
    return CAN1_Send_Frame((PDO1rx + SlaveID), 0, 2, msg); //	PDO1Rx 为 0x200,
}

u8 TX_PDO2(u8 SlaveID, u16 CtrlWord, u32 pos)
{
    //控制字CtrlWord与位置POS_Set
    u8 msg[8];
    msg[0] = (CtrlWord)&0xff;
    msg[1] = (CtrlWord >> 8) & 0xff;

    msg[2] = (pos)&0xff;
    msg[3] = (pos >> 8) & 0xff;
    msg[4] = (pos >> 16) & 0xff;
    msg[5] = (pos >> 24) & 0xff;

    return CAN1_Send_Frame((PDO2rx + SlaveID), 0, 6, msg);
}

u8 TX_PDO3(u8 SlaveID, u32 speed)
{
    //速度
    u8 msg[8];
    msg[0] = (speed)&0xff;
    msg[1] = (speed >> 8) & 0xff;
    msg[2] = (speed >> 16) & 0xff;
    msg[3] = (speed >> 24) & 0xff;

    return CAN1_Send_Frame((PDO3rx + SlaveID), 0, 4, msg);
}

// u8 TX_PDO4(u8 SlaveID, u16 max_current_limit)
// {
//     //最大电流限制
//     u8 msg[8];

//     msg[0] = (max_current_limit)&0xff;
//     msg[1] = (max_current_limit >> 8) & 0xff;
//     msg[2] = 0;
//     msg[3] = 0;

//     return CAN1_Send_Frame((PDO4rx + SlaveID), 0, 4, msg);
// }

u8 TX_PDO4(u8 SlaveID, u8 mode)
{

    u8 msg[8];

    msg[0] = mode;

    return CAN1_Send_Frame((PDO4rx + SlaveID), 0, 1, msg);
}

void StartMotor(u8 SlaveID)
{

    SetMotorCtrlword(SlaveID, 0x0006);
    delay_ms(200);
    SetMotorCtrlword(SlaveID, 0x000F);
}

// 4.RPDO,主站读从站数据，用RTR方式，不用主动上报方式;
//主要原因，总线上挂7个电机比较多，若主动上报，不成功导致重复发送的概率大,而且不是每个时刻，都需要知道每个电机的状态;
u8 RX_PDO1(u8 SlaveID)
{ //还需要等待接收,接收放在中断里
    return CAN1_Send_Frame(
        (PDO1tx + SlaveID), 1, 0,
        (void *)NULL); //注意,RTR 为 1，请求帧 。PDO1tx 为 0x180,
}