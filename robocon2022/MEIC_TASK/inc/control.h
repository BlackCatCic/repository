#ifndef __CONTROL_H
#define __CONTROL_H
#include "Include.h"

extern float Pitch;
extern float Angle_Speed_Pwm;
extern float Angle_Pwm;
extern int Moto1,Moto2;
extern float Turn_Pwm;
extern int16_t elecurrent[4];

void Xianfu_Pwm(void);

#endif

