#include "BSP_All.h"

void BSP_ALL(void)
{
  LED_Init();
	jy61_init();
	remote_control_init();//ң������ʼ��
	Motor_State_Init();
	PID_Init();
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS1_9tq,CAN_BS2_4tq,3,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������1Mbps  3->1M 6->500kb-
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS1_9tq,CAN_BS2_4tq,3,CAN_Mode_Normal);
}

