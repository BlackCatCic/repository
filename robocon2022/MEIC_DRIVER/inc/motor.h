#ifndef __MOTOR_H
#define __MOTOR_H
#include "Include.h"

typedef struct{
        int NowSpeed;              //现在速度
        int ExpectSpeed;  //期望速度
        int NowEnconder;           //现在编码器的值
        int LastEnconder;          //上次编码器的值        
        int SerialEnconder;        //连续编码器的值即正比于路程值
        int Laps;							     //编码器转过的圈数
        int ExpectEnconder;        //设置的路程值
        _pid pid_speed;            //速度PID
        _pid pid_loc;              //位置PID
        int pos_err;               //位置误差
}Motor_Structure;                  //用于接受大疆电机发送的数据

typedef struct{
        int NowSpeed;              //现在速度
        int ExpectSpeed;           //期望速度
        int NowEnconder;           //现在编码器的值
        int LastEnconder;          //上次编码器的值        
        int SerialEnconder;        //连续编码器的值即正比于路程值
				int LastSerialEnconder;      //上次连续编码器的值
        int Laps;							     //编码器转过的圈数
        int ExpectEnconder;        //设置的路程值
				float ExpectAngle;
        _pid pid_speed;            //速度PID
				_pid pid_loc;              //位置PID
        int pos_err;               //位置误差
	      float speed_pid;
				int setspeed;
}Endercode_Structure;              //用于欧艾迪编码器发送的数据

//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	    	1	//0,不使能;1,使能.		
#define CAN2_RX0_INT_ENABLE	    	1	//0,不使能;1,使能.
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs1,u8 tbs2,u16 brp,u8 mode);
 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据

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

