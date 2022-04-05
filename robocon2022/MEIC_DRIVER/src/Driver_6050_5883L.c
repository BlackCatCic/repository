
#include "Driver_6050_5883L.h"
#include "Driver_USART.h"
#include "BSP_IIC.h"

u8 isDRY = 0;              //isDataReady
int GyroOffset[3] = {0};   //陀螺仪零点
float MagOffset[6] = {0};  //磁力计零点

/*  @brief 初始化陀螺仪和磁力计
 *  @return 0  初始化成功
 *          !0 初始化失败
 */
u16 MPU6050_HML5883L_Init(void)
{
	u16 res = 0x00;
	
	/*一定记得在外部开一个延时*/
	//delay_ms(100);
	
	/*开陀螺仪*/
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,PWR_MGMT_1,0x01)   << 1;         //解除休眠状态
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,CONFIG,0x03)       << 2;         //设置低通滤波器
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,GYRO_CONFIG,0x10)  << 3;         //陀螺仪量程  ±1000dps
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,ACCEL_CONFIG,0x00) << 4;         //加速度计量程  ±2g
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_PIN_CFG,0x02)  << 5;  
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x00)   << 6;         //关闭所有中断
  res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,MPU6050_RA_USER_CTRL,0x00) << 7; //关闭Master模式
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,SMPLRT_DIV,0x01)   << 8;         //陀螺仪采样率	
	res |= IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x01)   << 9;         //开INT
	
	/*开磁力计*/
  res |= IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFA,0x70) << 10;
	res |= IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFB,0xA0) << 11;
	res |= IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_MODE,0x00)  << 12;
	res |= IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFA,0x18) << 13; 
	
	delay_ms(500); /*此延时不可关，零点确定全靠它了*/

  /*自检模式*/
//  res |= IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFA,0x71) << 9;
//	delay_ms(5);
//	res |= IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFB,0xA0) << 10;
//	delay_ms(5);
//	res |= IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_MODE,0x00)  << 11;
//	delay_ms(500);
	
	/*是否开启失败*/
	if (res)
	{
		printf("MPU Error Code:%d\r\n",res);
		return res;
	}
	
	/*开中断*/
	EXTIX_Init();

	/*陀螺仪和加速度计确定零点*/
	Gyro_Acc_Cali();

	/*磁力计计算零点*/
	Mag_Calc();
	
	/*初始化四元数*/
  InitQ();
	
	return 0;
}

/*  @brief 确定磁力计零点
 *  @return 无
 */
void Mag_Cali(void)
{
	static s16 MagMaxMinX[2] = {0};
	static s16 MagMaxMinY[2] = {0};
	static s16 MagMaxMinZ[2] = {0};	

  short mx,my,mz;
	
  Get_HML5883L_Data(&mx,&my,&mz);
	Get_HML5883L_Data(&mx,&my,&mz);

  /*加滤波*/
	if (mx >= MagMaxMinX[0]) MagMaxMinX[0] = 0.2*MagMaxMinX[0] + 0.8*mx;
	if (mx <= MagMaxMinX[1]) MagMaxMinX[1] = 0.2*MagMaxMinX[1] + 0.8*mx;
	if (my >= MagMaxMinY[0]) MagMaxMinY[0] = 0.2*MagMaxMinY[0] + 0.8*my;
	if (my <= MagMaxMinY[1]) MagMaxMinY[1] = 0.2*MagMaxMinY[1] + 0.8*my;
	if (mz >= MagMaxMinZ[0]) MagMaxMinZ[0] = 0.2*MagMaxMinZ[0] + 0.8*mz;
	if (mz <= MagMaxMinZ[1]) MagMaxMinZ[1] = 0.2*MagMaxMinZ[1] + 0.8*mz;

  /*不加滤波*/
//	if (mx >= MagMaxMinX[0]) MagMaxMinX[0] = mx;
//	if (mx <= MagMaxMinX[1]) MagMaxMinX[1] = mx;
//	if (my >= MagMaxMinY[0]) MagMaxMinY[0] = my;
//	if (my <= MagMaxMinY[1]) MagMaxMinY[1] = my;
//	if (mz >= MagMaxMinZ[0]) MagMaxMinZ[0] = mz;
//	if (mz <= MagMaxMinZ[1]) MagMaxMinZ[1] = mz;

	printf("MaxX = %d MinX = %d MaxY = %d MinY = %d MaxZ = %d MinZ = %d\r\n",
	MagMaxMinX[0],MagMaxMinX[1],MagMaxMinY[0],MagMaxMinY[1],MagMaxMinZ[0],MagMaxMinZ[1]);

}

/*  @brief 计算磁力计零点
 *  @return 无
 */
void Mag_Calc(void)
{
  MagOffset[0] = (MagMaxX + MagMinX) / 2.0f;
	MagOffset[1] = (MagMaxY + MagMinY) / 2.0f;
	MagOffset[2] = (MagMaxZ + MagMinZ) / 2.0f;	
	MagOffset[3] = 1.0f;
	MagOffset[4] = (MagMaxX - MagMinX) / (MagMaxY - MagMinY);
	MagOffset[5] = (MagMaxX - MagMinX) / (MagMaxZ - MagMinZ);
}


/*  @brief 确定陀螺仪和加速度计零点
 *  @return 无
 */
void Gyro_Acc_Cali(void)
{
	short ax,ay,az,gx,gy,gz;
	short cnt = 300;    //采cnt次求均值
	short tmp_cnt = 0;
	
	while (cnt--)
	{
		while (1)
		{
			if (isDRY)
			{
				isDRY = 0;
				if (!Get_MPU6050_Data(&ax,&ay,&az,&gx,&gy,&gz))
				{
					tmp_cnt++;
					GyroOffset[0] += gx;
					GyroOffset[1] += gy;
					GyroOffset[2] += gz;			
				}
				break;
			}
		}
	}
	
	GyroOffset[0] /= tmp_cnt;
	GyroOffset[1] /= tmp_cnt;
	GyroOffset[2] /= tmp_cnt;
	
}

void EXTIX_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);
	
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	   
}


void EXTI4_IRQHandler(void)
{
	 isDRY = 1;
	 EXTI_ClearITPendingBit(EXTI_Line4);  
}

/*  @brief 获取6轴陀螺仪数据
 *  @return 0  获取成功
 *          1  获取失败
 */
u8 Get_MPU6050_Data(short *ax,short *ay,short *az,short *gx,short *gy,short *gz)
{
	u8 buf[20];
	if (!IIC_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14))
	{
		*ax = ((u16)buf[0] << 8) | buf[1];
		*ay = ((u16)buf[2] << 8) | buf[3];
		*az = ((u16)buf[4] << 8) | buf[5];
		//temp = ((u16)buf[6] << 8) | buf[7]; /*温度*/
		*gx = ((u16)buf[8]  << 8) | buf[9];
		*gy = ((u16)buf[10] << 8) | buf[11];
		*gz = ((u16)buf[12] << 8) | buf[13];	
    
    return 0;		
	}else
	{
		return 1;
	}
}

/*  @brief 获取三轴磁力计数据
 *  @return 0  获取成功
 *          1  获取失败
 */
u8 Get_HML5883L_Data(short *mx,short *my,short *mz)
{
	u8 buf[10];
	if (!IIC_ReadData(HMC5883_ADDRESS,HMC58X3_R_XM,buf,6))
	{
		*mx = ((u16)buf[4] << 8) | buf[5];
		*my = ((u16)buf[0] << 8) | buf[1];
		*mz = ((u16)buf[2] << 8) | buf[3];
		
		return 0;
	}else
	{
		return 1;
	}
}

void Get_9Motion_Data(s16 *fifo)
{
		short mx,my,mz;
		short ax,ay,az,gx,gy,gz;
		Get_MPU6050_Data(&ax,&ay,&az,&gx,&gy,&gz);
		Get_HML5883L_Data(&mx,&my,&mz);
	  Get_HML5883L_Data(&mx,&my,&mz);  /*非得读两次才能读出来，我不懂你啊*/
	  
		fifo[0] = (s16)ax;
		fifo[1] = (s16)ay;
		fifo[2] = (s16)az;
		fifo[3] = (s16)gx - GyroOffset[0];
		fifo[4] = (s16)gy - GyroOffset[1];
		fifo[5] = (s16)gz - GyroOffset[2];
		fifo[6] = ((s16)mx - MagOffset[0]) * MagOffset[3];
		fifo[7] = ((s16)my - MagOffset[1]) * MagOffset[4];
		fifo[8] = ((s16)mz - MagOffset[2]) * MagOffset[5];
}


