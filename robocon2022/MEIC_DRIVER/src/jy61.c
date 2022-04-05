#include "jy61.h"
#include <string.h>
#include <stdio.h>
struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle; 
//发送指令集
char YAWCMD[3] = {0XFF,0XAA,0X52}; //Z轴角度清零
char ACCCMD[3] = {0XFF,0XAA,0X67}; //加速度校准
char SLEEPCMD[3] = {0XFF,0XAA,0X60};
char UARTMODECMD[3] = {0XFF,0XAA,0X61};
char IICMODECMD[3] = {0XFF,0XAA,0X62};

char cmdopen[5]={0xFF,0xAA,0x69,0x88,0xB5};
char cmdzero[5]={0xFF,0xAA,0x76,0x00,0x00};

//用串口2给JY模块发送指令
void sendcmd(char cmd[])
{
	char i;
	for(i=0;i<5;i++)
		USART_SendData(USART2,cmd[i]);
}

//CopeSerialData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;

		}
		ucRxCnt=0;//清空缓存区
	}
}

void CopeSerial1Data(unsigned char ucData)
{	
	//UART2_Put_Char(ucData);//转发串口1收到的数据给串口2（JY模块）
}

void Get_acc(float *accx,float *accy,float *accz)
{
  *accx=(float)stcAcc.a[0]/32768*16;
	*accy=(float)stcAcc.a[1]/32768*16;
	*accz=(float)stcAcc.a[2]/32768*16;
}
void Get_gyro(float *gyrox,float *gyroy,float *gyroz)
{
  *gyrox=(float)stcGyro.w[0]/32768*2000;
	*gyroy=(float)stcGyro.w[1]/32768*2000;
	*gyroz=(float)stcGyro.w[2]/32768*2000;
}
void Get_angle(float *roll,float *pitch,float *yaw)
{
  *roll=(float)stcAngle.Angle[0]/32768*180;
	*pitch=(float)stcAngle.Angle[1]/32768*180;
	*yaw=(float)stcAngle.Angle[2]/32768*180;
}
void Get_pitch(float *pitch)
{
	*pitch=(float)stcAngle.Angle[1]/32768*180;
}
void Get_yaw(float *yaw)
{
	*yaw=((float)stcAngle.Angle[2]/32768*180);
}
void Get_yaw_error(float *yaw)
{
	*yaw=(float)stcAngle.Angle[2]/32768*180;
}
void Dispose_JY61_yaw(float *yaw,float yaw_error)
{
	*yaw = *yaw-yaw_error;
	if(*yaw>=180) *yaw  = *yaw - 360;
	if(*yaw<=-180) *yaw = *yaw + 360;
}

void jy61_init(void)
{
	//GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
 //串口2对应引脚复用映射
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
 //USART2端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  GPIO_Init(GPIOA,&GPIO_InitStructure);
	
  USART_InitStructure.USART_BaudRate = 115200;//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART2, ENABLE);
	
}

void USART2_IRQHandler(void)                	//串口3中断服务程序
{
	/* enter interrupt */
	rt_interrupt_enter();          //在中断中一定要调用这对函数，进入中断

	if(USART_GetITStatus(USART2, USART_IT_RXNE)==SET)
	 {
		CopeSerial2Data(USART_ReceiveData(USART2));

		}
	/* leave interrupt */
  rt_interrupt_leave();    //在中断中一定要调用这对函数，离开中断  
} 

