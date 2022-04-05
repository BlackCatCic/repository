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
    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp CAN_Init����-1����
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
    
		//���ù�����
   	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN1_RX0_INT_ENABLE
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
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
    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN2, &CAN_InitStructure);   // ��ʼ��CAN1 
    
		//���ù�����
   	CAN_FilterInitStructure.CAN_FilterNumber=14;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN2_RX0_INT_ENABLE
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
} 
#if CAN2_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void CAN2_RX0_IRQHandler(void)
{	/* enter interrupt */
	rt_interrupt_enter();          //���ж���һ��Ҫ������Ժ����������ж�

  	CanRxMsg RxMessage;
    CAN_Receive(CAN2, 0, &RxMessage);
    
		COMM_CAN_RE_VESC_decoding(&RxMessage);
		
	/* leave interrupt */
  rt_interrupt_leave();    //���ж���һ��Ҫ������Ժ������뿪�ж� 

}
#endif
u8 encoder_ok=0;
#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void CAN1_RX0_IRQHandler(void)
{	  /* enter interrupt */
	  rt_interrupt_enter();          //���ж���һ��Ҫ������Ժ����������ж�
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
											Motor[0].NowSpeed = (short)(RxMessage.Data[2]<<8|(u16)RxMessage.Data[3]);//��ȡ�ٶ�
											enconder = (u16)(RxMessage.Data[0]<<8|(u16)RxMessage.Data[1]);//��ȡ��е�Ƕȼ���������ֵ��·��
											Motor[0].NowEnconder = enconder;
											pass_zero(0,8192);
											break;
					case 0x202:
											Motor[1].NowSpeed = (short)(RxMessage.Data[2]<<8|(u16)RxMessage.Data[3]);//��ȡ�ٶ�
											enconder = (u16)(RxMessage.Data[0]<<8|(u16)RxMessage.Data[1]);//��ȡ��е�Ƕȼ���������ֵ��·��
											Motor[1].NowEnconder = enconder;
											pass_zero(1,8192);
											break;
					case 0x203:
											Motor[2].NowSpeed = (short)(RxMessage.Data[2]<<8|(u16)RxMessage.Data[3]);//��ȡ�ٶ�
											enconder = (u16)(RxMessage.Data[0]<<8|(u16)RxMessage.Data[1]);//��ȡ��е�Ƕȼ���������ֵ��·��
											Motor[2].NowEnconder = enconder;
											pass_zero(2,8192);
											break;
					case 0x204:
											Motor[3].NowSpeed = (short)(RxMessage.Data[2]<<8|(u16)RxMessage.Data[3]);//��ȡ�ٶ�
											enconder = (u16)(RxMessage.Data[0]<<8|(u16)RxMessage.Data[1]);//��ȡ��е�Ƕȼ���������ֵ��·��
											Motor[3].NowEnconder = enconder;
											pass_zero(3,8192);
											break;
				
				  
		      
					default:    
									 printf(" δ֪ID:%x ",RxMessage.StdId);
									break;
			}
	/* leave interrupt */

  rt_interrupt_leave();    //���ж���һ��Ҫ������Ժ������뿪�ж� 

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

//���󽮵����������
uint8_t Send_Motor_current(uint32_t Send_CAN_ID,uint32_t IDE_mode,int16_t elecurrent[4])
{

	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=Send_CAN_ID;			// ��׼��ʶ�� 
	TxMessage.ExtId=Send_CAN_ID;			// ������չ��ʾ��  
	TxMessage.IDE=IDE_mode; 	    // ID֡����
	TxMessage.RTR=CAN_RTR_Data;		// ����֡
	TxMessage.DLC=8;				// Ҫ���͵����ݳ���
    
    TxMessage.Data[0] = get_H_8(elecurrent[0]);
    TxMessage.Data[1] = get_L_8(elecurrent[0]); //���Ƶ��1

    TxMessage.Data[2] = get_H_8(elecurrent[1]);
    TxMessage.Data[3] = get_L_8(elecurrent[1]);//���Ƶ��2

    TxMessage.Data[4] = get_H_8(elecurrent[2]);//���Ƶ��3
    TxMessage.Data[5] = get_L_8(elecurrent[2]);

    TxMessage.Data[6] = get_H_8(elecurrent[3]);//���Ƶ��4
    TxMessage.Data[7] = get_L_8(elecurrent[3]);
    
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;	
}

//��ŷ���ϱ���������ָ��
uint8_t Send_Encoder_Value(uint32_t Send_CAN_ID,uint32_t IDE_mode,uint8_t SendDate[5]) 
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=Send_CAN_ID;			 // ��׼��ʶ�� 
	TxMessage.ExtId=Send_CAN_ID;			 // ������չ��ʾ��  
	TxMessage.IDE=IDE_mode; 	         // ID֡����
	TxMessage.RTR=CAN_RTR_Data;		     // ����֡
	TxMessage.DLC=8;				           // Ҫ���͵����ݳ���
	
	TxMessage.Data[0] = SendDate[0];   //����ָ��
  TxMessage.Data[1] = SendDate[1];
  TxMessage.Data[2] = SendDate[2];
  TxMessage.Data[3] = SendDate[3];
  TxMessage.Data[4] = SendDate[4];
	
	mbox= CAN_Transmit(CAN1, &TxMessage);
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;	
}

//can����һ������(�̶���ʽ:ID,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x12;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;
  return 0;		

}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}

//�󽮵�� ��Ȧ������
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

//ŷ���ϱ����� ��Ȧ������
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
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTAʱ��	
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1); //����GPIOA_Pin8ΪTIM1_Ch1, 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);//����GPIOA_Pin11ΪTIM1_Ch4,ע��û��CH4N 
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��P
	
	TIM_TimeBaseStructure.TIM_Prescaler=83;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=39999;   //�Զ���װ��ֵ TIM1
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;//Ĭ�Ͼ�Ϊ0
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//��ʼ����ʱ��1
 
	//��ʼ��TIM1  PWMģʽ	 
	//PWM ģʽ 1�C�C �ڵ�������ģʽ�£�ֻҪ TIMx_CNT<TIMx_CCR1��ͨ�� 1 ��Ϊ��Ч״̬������Ϊ��Ч״̬���ڵݼ�����ģʽ�£�
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM1Ϊ����ռ�ձ�ģʽ��PWM2Ϊ������ģʽ
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�,��Ч��ƽΪ�͵�ƽ
 
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//�ڿ���ʱ�����,��������ÿ��Ըı�TIM_OCPolarity ���û��䣬��1ͨ��������
	
	TIM_OCInitStructure.TIM_Pulse = 2000; //����ͨ��1 CCR1��ռ�ձ���ֵ��
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); //Ch1��ʼ��
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���,CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�
	TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
	TIM_CtrlPWMOutputs(TIM1, ENABLE);//ʹ��TIM1��PWM�����TIM1��TIM8��Ч,���û�����л�����
}

//��ŷ���ϱ���������ָ�� ָ�������뿴ŷ���ϱ�����˵����
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
			printf("����ָ�����\n");
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
