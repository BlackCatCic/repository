#ifndef __MOTOR_H
#define __MOTOR_H
#include "Include.h"

typedef struct{
        int NowSpeed;              //�����ٶ�
        int ExpectSpeed;  //�����ٶ�
        int NowEnconder;           //���ڱ�������ֵ
        int LastEnconder;          //�ϴα�������ֵ        
        int SerialEnconder;        //������������ֵ��������·��ֵ
        int Laps;							     //������ת����Ȧ��
        int ExpectEnconder;        //���õ�·��ֵ
        _pid pid_speed;            //�ٶ�PID
        _pid pid_loc;              //λ��PID
        int pos_err;               //λ�����
}Motor_Structure;                  //���ڽ��ܴ󽮵�����͵�����

typedef struct{
        int NowSpeed;              //�����ٶ�
        int ExpectSpeed;           //�����ٶ�
        int NowEnconder;           //���ڱ�������ֵ
        int LastEnconder;          //�ϴα�������ֵ        
        int SerialEnconder;        //������������ֵ��������·��ֵ
				int LastSerialEnconder;      //�ϴ�������������ֵ
        int Laps;							     //������ת����Ȧ��
        int ExpectEnconder;        //���õ�·��ֵ
				float ExpectAngle;
        _pid pid_speed;            //�ٶ�PID
				_pid pid_loc;              //λ��PID
        int pos_err;               //λ�����
	      float speed_pid;
				int setspeed;
}Endercode_Structure;              //����ŷ���ϱ��������͵�����

//CAN1����RX0�ж�ʹ��
#define CAN1_RX0_INT_ENABLE	    	1	//0,��ʹ��;1,ʹ��.		
#define CAN2_RX0_INT_ENABLE	    	1	//0,��ʹ��;1,ʹ��.
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs1,u8 tbs2,u16 brp,u8 mode);
 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//��������

u8 CAN1_Receive_Msg(u8 *buf);							//��������

extern Motor_Structure Motor[4];
extern Endercode_Structure Motor_Endercode[3];
extern uint8_t Send_Date[5];

uint8_t Send_Motor_current(uint32_t Send_CAN_ID,uint32_t IDE_mode,int16_t elecurrent[4]);
uint8_t Send_Encoder_Value(uint32_t Send_CAN_ID,uint32_t IDE_mode,uint8_t SendDate[5]);

uint8_t Set_OAiDi_Command(uint8_t command,uint8_t SendDate[5]);

void Motor_State_Init(void);

void pass_zero(u8 number,float T);
void OAiDi_pass_zero(u8 number,float T);

void GM6020_PWM_Init(void);

void Motor_Endercode_XianFu(u8 number);
u8 Motor_XianFu(void);

#endif

