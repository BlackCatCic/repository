#include "motor_turn.h"
#include <math.h>
#include "arm_math.h"

void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes<100)
	{
		if(*angle>180.0f)
		{
			*angle-=360.0f;
			AngleLimit(angle);
		}
		else if(*angle<-180.0f)
		{
			*angle+=360.0f;
			AngleLimit(angle);
		}
	}
	
	recursiveTimes--;
}

wheel_t d_wheel={0};

/**
* @brief  CalcWheelSpeed计算轮子的和速度大小和方向
  * @note
* @param  sumVel:轮子选择
			vel:平移速度大小（mm/s）
		  direction:平移速度方向（-180°到180°）
		  omega:绕底盘中心旋转角速度(度/s),顺时针为负逆时针为正
		  angleN:在机器人坐标系下旋转线速度的正方向(单位：度)
* @retval 
*/
float last_sumangle;
void CalcWheelSpeed(wheelVel_t*sumVel, float vel , float direction , float omega , float angleN , float postureAngle)
{
	float velX , velY = 0.0f;
	float velN ,velNDirection= 0.0f;
	float sumVelX , sumVelY = 0.0f;
	
	//计算平移速度的X，Y分量
	velX = vel*arm_cos_f32(ANGLE2RAD(direction));
	velY = vel*arm_sin_f32(ANGLE2RAD(direction));
	
	//计算旋转的线速度
	velN = ANGLE2RAD(omega)*MOVEBASE_RADIUS;
	
	//计算舵轮角度
	velNDirection = angleN + postureAngle;
	
	//角度限幅(限制在-180到180度)
	AngleLimit(&velNDirection);	
	
	//计算和速度大小和方向
	sumVelX = velX + velN * arm_cos_f32(ANGLE2RAD(velNDirection));
	sumVelY = velY + velN * arm_sin_f32(ANGLE2RAD(velNDirection));
	
	arm_sqrt_f32(sumVelX * sumVelX + sumVelY * sumVelY,&sumVel->vel);
	
	//计算合成速度方向时将0向量单独处理
	if(sumVel->vel>0.01f)
	{
		sumVel->direction = RAD2ANGLE(atan2f(sumVelY , sumVelX))-postureAngle;
	}
	else
	{
		sumVel->direction = direction-postureAngle;
	}
	
	if(sumVel->direction>90){sumVel->direction-=180.0f;sumVel->vel=-sumVel->vel;}
	if(sumVel->direction<-90){sumVel->direction+=180.0f;sumVel->vel=-sumVel->vel;}
	
//	if(sumVel->direction-last_sumangle>90||sumVel->direction-last_sumangle<-90) {sumVel->direction=last_sumangle;sumVel->vel=-sumVel->vel;}
//	last_sumangle=sumVel->direction;
}

void  CalcWheelSpeed_XY(wheelVel_t*sumVel, float vel_X , float vel_Y ,float relative_angle ,float omega , float angleN , float postureAngle)
{
	float velX , velY = 0.0f;
	float velN ,velNDirection= 0.0f;
	float sumVelX , sumVelY = 0.0f;
	
	//计算平移速度的X，Y分量
	velX = vel_X*arm_cos_f32(ANGLE2RAD(relative_angle))    + vel_Y*arm_sin_f32(ANGLE2RAD(relative_angle));
	velY = -1*vel_X*arm_sin_f32(ANGLE2RAD(relative_angle)) + vel_Y*arm_cos_f32(ANGLE2RAD(relative_angle));
	
	//计算旋转的线速度
	velN = ANGLE2RAD(omega)*MOVEBASE_RADIUS;
	
	//计算舵轮角度
	velNDirection = angleN + postureAngle;
	
	//角度限幅(限制在-180到180度)
	AngleLimit(&velNDirection);	
	
	//计算和速度大小和方向
	sumVelX = velX + velN * arm_cos_f32(ANGLE2RAD(velNDirection));
	sumVelY = velY + velN * arm_sin_f32(ANGLE2RAD(velNDirection));
	
	arm_sqrt_f32(sumVelX * sumVelX + sumVelY * sumVelY,&sumVel->vel);
	
	//计算合成速度方向时将0向量单独处理
	if(sumVel->vel>0.01f)
	{
		sumVel->direction = RAD2ANGLE(atan2f(sumVelY , sumVelX))-postureAngle;
	}
	else
	{
		//sumVel->direction = direction-postureAngle;
	}
	
	if(sumVel->direction>90){sumVel->direction-=180.0f;sumVel->vel=-sumVel->vel;}
	if(sumVel->direction<-90){sumVel->direction+=180.0f;sumVel->vel=-sumVel->vel;}
}

float xianfu_Dbus(int16_t value)
{
	float value_copy;
	if(value>0) value_copy = -1*rc_ctrl.rc.ch[0]/7.33;
	else value_copy = 1*rc_ctrl.rc.ch[0]/7.33;
	if(value_copy>90)  value_copy  =  90;
	if(value_copy<-90) value_copy  = -90;
	return value_copy;
}

