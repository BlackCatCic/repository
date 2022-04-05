#ifndef __JY61_H
#define __JY61_H
#include "stm32f4xx.h"
#include <rtthread.h>

extern char cmdopen[5];
extern char cmdzero[5];
extern float yaw_error;

void CopeSerial2Data(unsigned char ucData);				
void Get_acc(float *accx,float *accy,float *accz);
void Get_gyro(float *gyrox,float *gyroy,float *gyroz);
void Get_angle(float *roll,float *pitch,float *yaw);
void jy61_init(void);
void Get_pitch(float *pitch);
void Get_yaw(float *yaw);
void sendcmd(char cmd[]);
void Get_yaw_error(float *yaw);
void Dispose_JY61_yaw(float *yaw,float yaw_error);
#endif
