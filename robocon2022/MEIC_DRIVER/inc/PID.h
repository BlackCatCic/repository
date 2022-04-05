#ifndef __PID_H
#define __PID_H

#define PID_ANGLE_IMAX		1500  //积分上限
#define PID_ANGLE_VMAX		16000  //输出上限

typedef struct _PID{
	float Kp;                 //比例
	float Ki;                 //积分
	float Kd;                 //微分
	float SetSpeed;						//设定值
	float AcutualSpeed;				//实际值
	float err;								//偏差值
	float err_last;						//上一个偏差值
	float I_Max;							//积分上限
	float V_Max;							//输出上限
	float voltage;						//定义电压值（控制执行器的变量）
	float interal;						//定义积分值
}_pid;

extern _pid stPID_Angle;
extern _pid stPID_ASpeed;
extern _pid stPID_Speed;

void PID_Init(void);																												//PID初始化
void PID_Data_Init(_pid *pid);																							//PID数据初值化
void PID_Param_Init(_pid *p,float IM,float VM);	//PID参数嵌入
float PID_Calc(_pid *pid,float kp,float ki,float kd,float speed,float feedback_speed);									//PID主运算
float PID_Speed(_pid *pid,float kp,float ki,float kd,float speed,float feedback_speed);

#endif

