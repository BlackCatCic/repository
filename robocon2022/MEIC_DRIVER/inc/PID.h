#ifndef __PID_H
#define __PID_H

#define PID_ANGLE_IMAX		1500  //��������
#define PID_ANGLE_VMAX		16000  //�������

typedef struct _PID{
	float Kp;                 //����
	float Ki;                 //����
	float Kd;                 //΢��
	float SetSpeed;						//�趨ֵ
	float AcutualSpeed;				//ʵ��ֵ
	float err;								//ƫ��ֵ
	float err_last;						//��һ��ƫ��ֵ
	float I_Max;							//��������
	float V_Max;							//�������
	float voltage;						//�����ѹֵ������ִ�����ı�����
	float interal;						//�������ֵ
}_pid;

extern _pid stPID_Angle;
extern _pid stPID_ASpeed;
extern _pid stPID_Speed;

void PID_Init(void);																												//PID��ʼ��
void PID_Data_Init(_pid *pid);																							//PID���ݳ�ֵ��
void PID_Param_Init(_pid *p,float IM,float VM);	//PID����Ƕ��
float PID_Calc(_pid *pid,float kp,float ki,float kd,float speed,float feedback_speed);									//PID������
float PID_Speed(_pid *pid,float kp,float ki,float kd,float speed,float feedback_speed);

#endif

