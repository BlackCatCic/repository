#include "PID.h"
#include "motor.h"

_pid stPID_Angle;
_pid stPID_ASpeed;
_pid stPID_Speed;
void PID_Init(void)//PID 始值化
{
	PID_Param_Init(&(Motor_Endercode[0].pid_speed),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor_Endercode[0].pid_speed));
	
	PID_Param_Init(&(Motor_Endercode[0].pid_loc),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor_Endercode[0].pid_loc));
	
	PID_Param_Init(&(Motor_Endercode[1].pid_speed),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor_Endercode[1].pid_speed));
	
	PID_Param_Init(&(Motor_Endercode[1].pid_loc),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor_Endercode[1].pid_loc));
	
	PID_Param_Init(&(Motor_Endercode[2].pid_speed),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor_Endercode[2].pid_speed));
	
	PID_Param_Init(&(Motor_Endercode[2].pid_loc),PID_ANGLE_IMAX,PID_ANGLE_VMAX);
	PID_Data_Init(&(Motor_Endercode[2].pid_loc));

}

void PID_Data_Init(_pid *pid)	//PID 数据始值化
{	
	pid->SetSpeed			=		0.0;		
	pid->AcutualSpeed	=		0.0;
	pid->err					=		0.0;
	pid->err_last			=		0.0;
	pid->interal			=		0.0;
	pid->voltage			=		0.0;
}


void PID_Param_Init(_pid *p,float IM,float VM)  //PID参数嵌入
{
	p->I_Max	=		IM;
	p->V_Max	=		VM;
}

float PID_Calc(_pid *pid,float kp,float ki,float kd,float speed,float feedback_speed)								
{
	pid->Kp                  =     kp;
	pid->Ki                  =     ki;
	pid->Kd                  =     kd;
	pid->AcutualSpeed		=			feedback_speed;   //反馈实际速度
	pid->SetSpeed				=			speed;            //设定速度
	pid->err						=			pid->SetSpeed		-		pid->AcutualSpeed;  //差值
	if(feedback_speed==0)pid->interal=0;
		if(pid->interal > pid->I_Max)                          //定义积分值大于积分上限
						pid->interal	=		pid->I_Max;                  //积分上限值赋予定义积分值
		else if(pid->interal < -pid->I_Max)                    //定义积分值小于负的积分上限
						pid->interal	=		-pid->I_Max;                 //负的积分上限值赋予定义积分值
		
	pid->interal	+=	pid->err;                              //定义积分值累加差值
	pid->voltage	=		pid->Kp * pid->err + pid->Ki * pid->interal + pid->Kd * (pid->err - pid->err_last);//计算定义电压值（控制执行器的变量）
	pid->err_last	=		pid->err;                              //上一个偏差值
		
			 if(pid->voltage > pid->V_Max) 		pid->voltage 	= pid->V_Max;//定义电压值大于输出上限
	else if(pid->voltage < -pid->V_Max) 	pid->voltage	=	-pid->V_Max;	
		
	return pid->voltage;                                          //返回定义电压值
}

			
