#include "control.h"

float Pitch;
float Angle_Speed_Pwm;
float Angle_Pwm;
int Moto1,Moto2;
float Turn_Pwm;
int16_t elecurrent[4];

void Xianfu_Pwm()
{
	if(Moto1>4000)Moto1=4000;if(Moto1<-4000)Moto1=-4000;
	if(Moto2>4000)Moto2=4000;if(Moto2<-4000)Moto2=-4000;
}
