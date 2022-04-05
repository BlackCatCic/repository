#ifndef __MOTORTURN_H
#define __MOTORTURN_H	 

#include "Include.h"

//�Ƕ���ת��Ϊ������
#define ANGLE2RAD(x) (x/180.0f*PI)
//������ת��Ϊ�Ƕ���
#define RAD2ANGLE(x) (x/PI*180.0f)

//������ת�뾶
#define MOVEBASE_RADIUS (390.088f)

//ǰ���������������߷���(ȫ�ֶ�λ�Եģ�
#define FRONT_VERTICAL_ANG       (90.0f)
//�Һ����������������߷���(��ͣ�����Ե�)
#define RIGHT_REAR_VERTICAL_ANG  (-30.0f)
//��������������ߵ����߷���
#define LEFT_REAR_VERTICAL_ANG   (-150.0f)

//�����ٶȺͷ���ṹ��
typedef struct
{
	//�����ٶȴ�С
	float vel;
	//�����ٶȷ���
	float direction;
//	//����ʵ���ٶȷ���
//	angle_conversion angle;
}wheelVel_t;

//���ӽṹ��
typedef struct
{
	//ǰ��
	wheelVel_t Front;
	//�Һ���
	wheelVel_t RightRear;
	//�����
	wheelVel_t LeftRear;
}wheel_t;

extern wheel_t d_wheel;

void  CalcWheelSpeed(wheelVel_t*sumVel, float vel , float direction , float omega , float angleN , float postureAngle);
void  CalcWheelSpeed_XY(wheelVel_t*sumVel, float vel_X , float vel_Y ,float relative_angle ,float omega , float angleN , float postureAngle);
float xianfu_Dbus(int16_t value);

#endif
