#include "app_thread.h"
extern struct rt_timer timer1,timer2;
float YAW,yaw_error;
/* 线程入口1 */
void thread_entry1(void* parameter)
{
	Get_yaw(&yaw_error);
	while(1)
	{
		Get_yaw(&YAW);
		Dispose_JY61_yaw(&YAW,yaw_error);
		LED_1=0;
		rt_thread_mdelay(50);
		LED_1=1;
		//printf("%f\r\n",Motor_Endercode[0].ExpectAngle);
		//printf("%d %d %d\r\n",Motor[0].SerialEnconder,Motor[1].SerialEnconder,Motor[2].SerialEnconder);
		printf("%f %f \r\n",YAW,yaw_error);
		rt_thread_mdelay(50);
	}
}	
/* 线程入口2 */
void thread_entry2(void* parameter)
{
	while(1)
	{
	if(rc_ctrl.rc.s[0]==2&&rc_ctrl.rc.s[1]==2)
	{
		CalcWheelSpeed_XY(&d_wheel.Front,     -1*rc_ctrl.rc.ch[1]*5 , 1*rc_ctrl.rc.ch[0]*5 ,YAW ,1*rc_ctrl.rc.ch[2]*2 ,FRONT_VERTICAL_ANG , 0.0);
		CalcWheelSpeed_XY(&d_wheel.RightRear, -1*rc_ctrl.rc.ch[1]*5 , 1*rc_ctrl.rc.ch[0]*5 ,YAW ,1*rc_ctrl.rc.ch[2]*2 ,RIGHT_REAR_VERTICAL_ANG , 0.0);
		CalcWheelSpeed_XY(&d_wheel.LeftRear,  -1*rc_ctrl.rc.ch[1]*5 , 1*rc_ctrl.rc.ch[0]*5 ,YAW ,1*rc_ctrl.rc.ch[2]*2 ,LEFT_REAR_VERTICAL_ANG , 0.0);
//	  CalcWheelSpeed(&d_wheel.Front,    -1*rc_ctrl.rc.ch[1]*5,xianfu_Dbus(rc_ctrl.rc.ch[1]),1*rc_ctrl.rc.ch[2]/5,FRONT_VERTICAL_ANG,0.0);
//		CalcWheelSpeed(&d_wheel.RightRear,-1*rc_ctrl.rc.ch[1]*5,xianfu_Dbus(rc_ctrl.rc.ch[1]),1*rc_ctrl.rc.ch[2]/5,RIGHT_REAR_VERTICAL_ANG,0.0);
//		CalcWheelSpeed(&d_wheel.LeftRear ,-1*rc_ctrl.rc.ch[1]*5,xianfu_Dbus(rc_ctrl.rc.ch[1]),1*rc_ctrl.rc.ch[2]/5,LEFT_REAR_VERTICAL_ANG, 0.0);
	  Motor_Endercode[0].ExpectAngle = d_wheel.Front.direction;
    Motor_Endercode[0].ExpectSpeed = d_wheel.Front.vel;
	  Motor_Endercode[1].ExpectAngle = d_wheel.RightRear.direction;
	  Motor_Endercode[1].ExpectSpeed = d_wheel.RightRear.vel;
	  Motor_Endercode[2].ExpectAngle = d_wheel.LeftRear.direction;
	  Motor_Endercode[2].ExpectSpeed = d_wheel.LeftRear.vel;
	  comm_can_set_rpm(90,Motor_Endercode[0].ExpectSpeed);
	  comm_can_set_rpm(4, Motor_Endercode[1].ExpectSpeed);
    comm_can_set_rpm(89,Motor_Endercode[2].ExpectSpeed);
	}	
			rt_thread_mdelay(50);
	}
}	
/* 线程入口2 */
void thread_entry3(void* parameter)
{
	int16_t ERROR_ZERO[4]={0};
	while(1)
	{
   if(rc_ctrl.rc.s[0]==1&&rc_ctrl.rc.s[1]==1)
	 { 
		 rt_timer_stop(&timer1);
		 while(1){
		 Send_Motor_current(0x200,CAN_Id_Standard,ERROR_ZERO);
		 comm_can_set_rpm(89 ,0);	
	   comm_can_set_rpm(4 ,0);	
	   comm_can_set_rpm(90 ,0);	
	  }
		}
	 if(Motor_XianFu())
	 {
		 while(1)
		 {
			 elecurrent[0]=0;
		   elecurrent[1]=0;
		   elecurrent[2]=0;
		   elecurrent[3]=0;
		   comm_can_set_rpm(89 ,0);	
	     comm_can_set_rpm(4 ,0);	
	     comm_can_set_rpm(90 ,0);
			 printf("%d %d %d\r\n",Motor[0].SerialEnconder,Motor[1].SerialEnconder,Motor[2].SerialEnconder);
		 }	
	 }
		rt_thread_mdelay(10);
	}
}	
/*舵轮编码器环*/
void timeout1(void* parameter)
{
	
	Motor_Endercode[0].setspeed = PID_Calc(&(Motor_Endercode[0].pid_loc),1.5,0.001,1.25,(332.83f*Motor_Endercode[0].ExpectAngle+16384),Motor_Endercode[0].SerialEnconder);
	Motor_Endercode_XianFu(0);
	Motor_Endercode[0].setspeed = PID_Calc(&(Motor_Endercode[0].pid_speed),5.0,0.3,0.15,Motor_Endercode[0].setspeed,Motor[0].NowSpeed);
	
	Motor_Endercode[1].speed_pid = PID_Calc(&(Motor_Endercode[1].pid_loc),1.5,0.001,1.25,(332.83f*Motor_Endercode[1].ExpectAngle+16384),Motor_Endercode[1].SerialEnconder);
	Motor_Endercode_XianFu(1);
	Motor_Endercode[1].setspeed = PID_Calc(&(Motor_Endercode[1].pid_speed),5.0,0.3,0.15,Motor_Endercode[1].speed_pid,Motor[1].NowSpeed);
	
	Motor_Endercode[2].setspeed = PID_Calc(&(Motor_Endercode[2].pid_loc),1.5,0.001,1.25,(332.83f*Motor_Endercode[2].ExpectAngle+16384),Motor_Endercode[2].SerialEnconder);
	Motor_Endercode_XianFu(2);
	Motor_Endercode[2].setspeed = PID_Calc(&(Motor_Endercode[2].pid_speed),5.0,0.3,0.15,Motor_Endercode[2].setspeed,Motor[2].NowSpeed);
	
	elecurrent[0]=Motor_Endercode[0].setspeed;
	elecurrent[1]=Motor_Endercode[1].setspeed;
	elecurrent[2]=Motor_Endercode[2].setspeed;
	elecurrent[3]=0;
	
	Send_Motor_current(0x200,CAN_Id_Standard,elecurrent);

}


