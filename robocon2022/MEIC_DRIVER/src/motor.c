#include "motor.h"
#include "string.h"

uint8_t Send_Date[5];
Motor_Structure Motor[4]={'\0'};
Endercode_Structure Motor_Endercode[3]={'\0'};
uint8_t zero[3];

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs1,u8 tbs2,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp CAN_Init里有-1操作
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
    
		//配置过滤器
   	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
#if CAN1_RX0_INT_ENABLE
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs1,u8 tbs2,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN2_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN1 
    
		//配置过滤器
   	CAN_FilterInitStructure.CAN_FilterNumber=14;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
#if CAN2_RX0_INT_ENABLE
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
} 
#if CAN2_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN2_RX0_IRQHandler(void)
{	/* enter interrupt */
	rt_interrupt_enter();          //在中断中一定要调用这对函数，进入中断

  	CanRxMsg RxMessage;
    CAN_Receive(CAN2, 0, &RxMessage);
    
		COMM_CAN_RE_VESC_decoding(&RxMessage);
		
	/* leave interrupt */
  rt_interrupt_leave();    //在中断中一定要调用这对函数，离开中断 

}
#endif
u8 encoder_ok=0;
#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)
{	  /* enter interrupt */
	  rt_interrupt_enter();          //在中断中一定要调用这对函数，进入中断
  	CanRxMsg RxMessage;
    uint16_t enconder;
    CAN_Receive(CAN1, 0, &RxMessage);
			switch(RxMessage.StdId)
			{	
				   case 0x07:
											Motor_Endercode[0].NowEnconder = (((uint32_t)RxMessage.Data[6] << 24) |
																								  ((uint32_t)RxMessage.Data[5] << 16) |
																								  ((uint32_t)RxMessage.Data[4] << 8)  |
																								  ((uint32_t)RxMessage.Data[3]));
											if(!zero[0])
											{
												Motor_Endercode[0].LastEnconder = Motor_Endercode[0].NowEnconder;
												zero[0]=1;
											}
											OAiDi_pass_zero(0,32768);
											break;
				   case 0x08:
										  Motor_Endercode[1].NowEnconder = (((uint32_t)RxMessage.Data[6] << 24) |
																								  ((uint32_t)RxMessage.Data[5] << 16) |
																								  ((uint32_t)RxMessage.Data[4] << 8)  |
																								  ((uint32_t)RxMessage.Data[3]));
											if(!zero[1])
											{
												Motor_Endercode[1].LastEnconder = Motor_Endercode[1].NowEnconder;
												zero[1]=1;
											}
	                    OAiDi_pass_zero(1,32768);				
											break;
			    case 0x09:
										 Motor_Endercode[2].NowEnconder =  (((uint32_t)RxMessage.Data[6] << 24) |
																								  ((uint32_t)RxMessage.Data[5] << 16) |
																		              ((uint32_t)RxMessage.Data[4] << 8)  |
																		              ((uint32_t)RxMessage.Data[3]));
											if(!zero[2])
											{
												Motor_Endercode[2].LastEnconder = Motor_Endercode[2].NowEnconder;
												zero[2]=1;
											}
					            OAiDi_pass_zero(2,32768);
											break;
				  case 0x201: 
											Motor[0].NowSpeed = (short)(RxMessage.Data[2]<<8|(u16)RxMessage.Data[3]);//获取速度
											enconder = (u16)(RxMessage.Data[0]<<8|(u16)RxMessage.Data[1]);//获取机械角度即编码器的值及路程
											Motor[0].NowEnconder = enconder;
											pass_zero(0,8192);
											break;
					case 0x202:
											Motor[1].NowSpeed = (short)(RxMessage.Data[2]<<8|(u16)RxMessage.Data[3]);//获取速度
											enconder = (u16)(RxMessage.Data[0]<<8|(u16)RxMessage.Data[1]);//获取机械角度即编码器的值及路程
											Motor[1].NowEnconder = enconder;
											pass_zero(1,8192);
											break;
					case 0x203:
											Motor[2].NowSpeed = (short)(RxMessage.Data[2]<<8|(u16)RxMessage.Data[3]);//获取速度
											enconder = (u16)(RxMessage.Data[0]<<8|(u16)RxMessage.Data[1]);//获取机械角度即编码器的值及路程
											Motor[2].NowEnconder = enconder;
											pass_zero(2,8192);
											break;
					case 0x204:
											Motor[3].NowSpeed = (short)(RxMessage.Data[2]<<8|(u16)RxMessage.Data[3]);//获取速度
											enconder = (u16)(RxMessage.Data[0]<<8|(u16)RxMessage.Data[1]);//获取机械角度即编码器的值及路程
											Motor[3].NowEnconder = enconder;
											pass_zero(3,8192);
											break;
				
				  
		      
					default:    
									 printf(" 未知ID:%x ",RxMessage.StdId);
									break;
			}
	/* leave interrupt */

  rt_interrupt_leave();    //在中断中一定要调用这对函数，离开中断 

}
#endif
static u8 get_H_8(int16_t date)
{

    return (u8)((date&0xff00)>>8);
}

static u8 get_L_8(int16_t date)
{

    return (u8)(date&0x00ff);
}

//给大疆电调发送数据
uint8_t Send_Motor_current(uint32_t Send_CAN_ID,uint32_t IDE_mode,int16_t elecurrent[4])
{

	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=Send_CAN_ID;			// 标准标识符 
	TxMessage.ExtId=Send_CAN_ID;			// 设置扩展标示符  
	TxMessage.IDE=IDE_mode; 	    // ID帧类型
	TxMessage.RTR=CAN_RTR_Data;		// 数据帧
	TxMessage.DLC=8;				// 要发送的数据长度
    
    TxMessage.Data[0] = get_H_8(elecurrent[0]);
    TxMessage.Data[1] = get_L_8(elecurrent[0]); //控制电机1

    TxMessage.Data[2] = get_H_8(elecurrent[1]);
    TxMessage.Data[3] = get_L_8(elecurrent[1]);//控制电机2

    TxMessage.Data[4] = get_H_8(elecurrent[2]);//控制电机3
    TxMessage.Data[5] = get_L_8(elecurrent[2]);

    TxMessage.Data[6] = get_H_8(elecurrent[3]);//控制电机4
    TxMessage.Data[7] = get_L_8(elecurrent[3]);
    
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)return 1;
	return 0;	
}

//给欧艾迪编码器发送指令
uint8_t Send_Encoder_Value(uint32_t Send_CAN_ID,uint32_t IDE_mode,uint8_t SendDate[5]) 
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=Send_CAN_ID;			 // 标准标识符 
	TxMessage.ExtId=Send_CAN_ID;			 // 设置扩展标示符  
	TxMessage.IDE=IDE_mode; 	         // ID帧类型
	TxMessage.RTR=CAN_RTR_Data;		     // 数据帧
	TxMessage.DLC=8;				           // 要发送的数据长度
	
	TxMessage.Data[0] = SendDate[0];   //发送指令
  TxMessage.Data[1] = SendDate[1];
  TxMessage.Data[2] = SendDate[2];
  TxMessage.Data[3] = SendDate[3];
  TxMessage.Data[4] = SendDate[4];
	
	mbox= CAN_Transmit(CAN1, &TxMessage);
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)return 1;
	return 0;	
}

//can发送一组数据(固定格式:ID,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x12;	 // 标准标识符为0
  TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		

}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}

//大疆电调 多圈数设置
void pass_zero(u8 number,float T)
{
     static float pos_errtal=0;
     static uint8_t i=0; 
     if(Motor[number].pos_err<1&&Motor[number].NowEnconder>1&&Motor[number].LastEnconder<1&&i<20)	
     {
          i++;
          pos_errtal +=  Motor[number].NowEnconder;
          Motor[number].pos_err = pos_errtal/i;        
     }
     else 
		     (Motor[number].LastEnconder - Motor[number].NowEnconder >= T/2 )? Motor[number].Laps++ :( Motor[number].LastEnconder - Motor[number].NowEnconder <= -T/2? Motor[number].Laps--:NULL);
    (Motor[number].SerialEnconder) = Motor[number].Laps*T + Motor[number].NowEnconder - Motor[number].pos_err;
    (Motor[number].LastEnconder)  = Motor[number].NowEnconder ; 
}

//欧艾迪编码器 多圈数设置
void OAiDi_pass_zero(u8 number,float T)
{
	Motor_Endercode[number].NowSpeed = Motor_Endercode[number].SerialEnconder - Motor_Endercode[number].LastSerialEnconder;
	Motor_Endercode[number].LastSerialEnconder = Motor_Endercode[number].SerialEnconder;
	
	(Motor_Endercode[number].LastEnconder - Motor_Endercode[number].NowEnconder >= T/2)? Motor_Endercode[number].Laps++ :(Motor_Endercode[number].LastEnconder - Motor_Endercode[number].NowEnconder <= -T/2 ? Motor_Endercode[number].Laps--:NULL);
	Motor_Endercode[number].SerialEnconder = Motor_Endercode[number].Laps*T + Motor_Endercode[number].NowEnconder;
	
	Motor_Endercode[number].LastEnconder = Motor_Endercode[number].NowEnconder;
}

void Motor_State_Init()
{
	u8 i;
	for(i=0;i<4;i++){
	Motor[i].NowSpeed       = 0;
	Motor[i].ExpectSpeed    = 0;
  Motor[i].NowEnconder    = 0;
  Motor[i].SerialEnconder = 0;
	Motor[i].LastEnconder   = 0;
	Motor[i].Laps           = 0;
	Motor[i].ExpectEnconder = 0;}
}		


void GM6020_PWM_Init()
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTA时钟	
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1); //复用GPIOA_Pin8为TIM1_Ch1, 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);//复用GPIOA_Pin11为TIM1_Ch4,注意没有CH4N 
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化P
	
	TIM_TimeBaseStructure.TIM_Prescaler=83;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=39999;   //自动重装载值 TIM1
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;//默认就为0
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器1
 
	//初始化TIM1  PWM模式	 
	//PWM 模式 1–– 在递增计数模式下，只要 TIMx_CNT<TIMx_CCR1，通道 1 便为有效状态，否则为无效状态。在递减计数模式下，
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM1为正常占空比模式，PWM2为反极性模式
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低,有效电平为低电平
 
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//在空闲时输出低,这里的设置可以改变TIM_OCPolarity 如果没这句，第1通道有问题
	
	TIM_OCInitStructure.TIM_Pulse = 2000; //输入通道1 CCR1（占空比数值）
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); //Ch1初始化
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器,CCR自动装载默认也是打开的
	TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
	TIM_CtrlPWMOutputs(TIM1, ENABLE);//使能TIM1的PWM输出，TIM1与TIM8有效,如果没有这行会问题
}

//给欧艾迪编码器发送指令 指令详情请看欧艾迪编码器说明书
uint8_t Set_OAiDi_Command(uint8_t command,uint8_t SendDate[5]) 
{
	switch(command){
		case 0x01:
			SendDate[0]=0x04;
		  SendDate[1]=0x08;
		  SendDate[2]=0x01;
		  SendDate[3]=0x00;
		break;
		
		case 0x02:
			SendDate[0]=0x04;
		  SendDate[1]=0x01;
		  SendDate[2]=0x02;
		  SendDate[3]=0x08;
		break;
		
		case 0x03:
			SendDate[0]=0x04;
		  SendDate[1]=0x08;
		  SendDate[2]=0x03;
		  SendDate[3]=0x01;
		break;
		
		case 0x04:
			SendDate[0]=0x04;
		  SendDate[1]=0x08;
		  SendDate[2]=0x04;
		  SendDate[3]=0xAA;
		break;
		
		case 0x05:
			SendDate[0]=0x05;
		  SendDate[1]=0x09;
		  SendDate[2]=0x05;
		  SendDate[3]=0xE8;
			SendDate[4]=0x03;
		break;
	
		case 0x06:
			SendDate[0]=0x04;
		  SendDate[1]=0x07;
		  SendDate[2]=0x06;
		  SendDate[3]=0x00;
		break;
		
		case 0x07:
			SendDate[0]=0x04;
		  SendDate[1]=0x01;
		  SendDate[2]=0x07;
		  SendDate[3]=0x01;
		break;
		
		case 0x08:
			SendDate[0]=0x04;
		  SendDate[1]=0x01;
		  SendDate[2]=0x08;
		  SendDate[3]=0x00;
		break;
		
		case 0x09:
			SendDate[0]=0x04;
		  SendDate[1]=0x01;
		  SendDate[2]=0x09;
		  SendDate[3]=0x00;
		break;
		
		case 0x0A:
			SendDate[0]=0x04;
		  SendDate[1]=0x01;
		  SendDate[2]=0x0A;
		  SendDate[3]=0x00;
		break;
	
		case 0x0B:
			SendDate[0]=0x05;
		  SendDate[1]=0x01;
		  SendDate[2]=0x0B;
		  SendDate[3]=0xE8;
			SendDate[4]=0x03;
		break;
		
		default:
			printf("输入指令错误\n");
}
	return command;
}

void Motor_Endercode_XianFu(u8 number)
{
	if(Motor_Endercode[number].setspeed < 100 && Motor_Endercode[number].setspeed> -100) Motor_Endercode[number].setspeed=0;
//	if(Motor_Endercode[number].setspeed > 100) Motor_Endercode[number].setspeed = 100;
//	if(Motor_Endercode[number].setspeed < -100) Motor_Endercode[number].setspeed = -100;
}

u8 Motor_XianFu()
{
	u8 i;
	for(i=0;i<3;i++)
	{
		if(Motor[i].SerialEnconder>1228800 || Motor[i].SerialEnconder<-1228800) return 1;
	}
	return 0;
}
