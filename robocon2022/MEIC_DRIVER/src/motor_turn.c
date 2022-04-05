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
* @brief  CalcWheelSpeed�������ӵĺ��ٶȴ�С�ͷ���
  * @note
* @param  sumVel:����ѡ��
			vel:ƽ���ٶȴ�С��mm/s��
		  direction:ƽ���ٶȷ���-180�㵽180�㣩
		  omega:�Ƶ���������ת���ٶ�(��/s),˳ʱ��Ϊ����ʱ��Ϊ��
		  angleN:�ڻ���������ϵ����ת���ٶȵ�������(��λ����)
* @retval 
*/
float last_sumangle;
void CalcWheelSpeed(wheelVel_t*sumVel, float vel , float direction , float omega , float angleN , float postureAngle)
{
	float velX , velY = 0.0f;
	float velN ,velNDirection= 0.0f;
	float sumVelX , sumVelY = 0.0f;
	
	//����ƽ���ٶȵ�X��Y����
	velX = vel*arm_cos_f32(ANGLE2RAD(direction));
	velY = vel*arm_sin_f32(ANGLE2RAD(direction));
	
	//������ת�����ٶ�
	velN = ANGLE2RAD(omega)*MOVEBASE_RADIUS;
	
	//������ֽǶ�
	velNDirection = angleN + postureAngle;
	
	//�Ƕ��޷�(������-180��180��)
	AngleLimit(&velNDirection);	
	
	//������ٶȴ�С�ͷ���
	sumVelX = velX + velN * arm_cos_f32(ANGLE2RAD(velNDirection));
	sumVelY = velY + velN * arm_sin_f32(ANGLE2RAD(velNDirection));
	
	arm_sqrt_f32(sumVelX * sumVelX + sumVelY * sumVelY,&sumVel->vel);
	
	//����ϳ��ٶȷ���ʱ��0������������
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
	
	//����ƽ���ٶȵ�X��Y����
	velX = vel_X*arm_cos_f32(ANGLE2RAD(relative_angle))    + vel_Y*arm_sin_f32(ANGLE2RAD(relative_angle));
	velY = -1*vel_X*arm_sin_f32(ANGLE2RAD(relative_angle)) + vel_Y*arm_cos_f32(ANGLE2RAD(relative_angle));
	
	//������ת�����ٶ�
	velN = ANGLE2RAD(omega)*MOVEBASE_RADIUS;
	
	//������ֽǶ�
	velNDirection = angleN + postureAngle;
	
	//�Ƕ��޷�(������-180��180��)
	AngleLimit(&velNDirection);	
	
	//������ٶȴ�С�ͷ���
	sumVelX = velX + velN * arm_cos_f32(ANGLE2RAD(velNDirection));
	sumVelY = velY + velN * arm_sin_f32(ANGLE2RAD(velNDirection));
	
	arm_sqrt_f32(sumVelX * sumVelX + sumVelY * sumVelY,&sumVel->vel);
	
	//����ϳ��ٶȷ���ʱ��0������������
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

