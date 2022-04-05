#ifndef __MOTORTURN_H
#define __MOTORTURN_H	 

#include "Include.h"

//角度制转化为弧度制
#define ANGLE2RAD(x) (x/180.0f*PI)
//弧度制转换为角度制
#define RAD2ANGLE(x) (x/PI*180.0f)

//底盘旋转半径
#define MOVEBASE_RADIUS (390.088f)

//前轮与中心连线切线方向(全局定位旁的）
#define FRONT_VERTICAL_ANG       (90.0f)
//右后轮与中心连线切线方向(急停开关旁的)
#define RIGHT_REAR_VERTICAL_ANG  (-30.0f)
//左后轮与中心连线的切线方向
#define LEFT_REAR_VERTICAL_ANG   (-150.0f)

//轮子速度和方向结构体
typedef struct
{
	//轮子速度大小
	float vel;
	//轮子速度方向
	float direction;
//	//轮子实际速度方向
//	angle_conversion angle;
}wheelVel_t;

//轮子结构体
typedef struct
{
	//前轮
	wheelVel_t Front;
	//右后轮
	wheelVel_t RightRear;
	//左后轮
	wheelVel_t LeftRear;
}wheel_t;

extern wheel_t d_wheel;

void  CalcWheelSpeed(wheelVel_t*sumVel, float vel , float direction , float omega , float angleN , float postureAngle);
void  CalcWheelSpeed_XY(wheelVel_t*sumVel, float vel_X , float vel_Y ,float relative_angle ,float omega , float angleN , float postureAngle);
float xianfu_Dbus(int16_t value);

#endif
