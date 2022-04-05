#include "icm20602.h"

int16_t icm_gyro_x,icm_gyro_y,icm_gyro_z;
int16_t icm_acc_x,icm_acc_y,icm_acc_z;


//-------------------------------------------------------------------------------------------------------------------
// @brief		SPI���ͽ��պ���
// @param		spi_n			ѡ��SPIģ�� (SPI_1-SPI_2)
// @param		modata			���͵����ݻ�������ַ
// @param		midata			��������ʱ���յ������ݵĴ洢��ַ(����Ҫ������ NULL)
// @param		len				���͵��ֽ���
// @param		continuous		����ͨ����CS�Ƿ����������Ч״̬ 1:�������� 0:ÿ������һ���ֽڹر�CS(һ������Ϊ1 ����)
// @return		void				
// @since		v2.0
// Sample usage:				spi_mosi(SPI_1,buf,buf,1);										//����buff�����ݣ������յ�buf�����Ϊ1�ֽ� ͨ���ڼ�CS��������
//-------------------------------------------------------------------------------------------------------------------
void spi_mosi (u8 *modata, u8 *midata, u32 len)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}//�ȴ��������� 
	while(len--)																				// �жϳ���
	{
		SPI2->DR = *modata++;
		//SPI_I2S_SendData(SPI2, *modata++);								// ��������
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){} 
		if(midata != NULL)																		// ������Ч
		{
			*midata++ = SPI2->DR;												// ��ȡ����
		}
	}
}

//SPI2 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI1_ReadWriteByte(u8 TxData)
{		 			 
 
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}//�ȴ���������  
	
	SPI_I2S_SendData(SPI2, TxData); //ͨ������SPIx����һ��byte  ����
		
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){} //�ȴ�������һ��byte  
 
	return SPI_I2S_ReceiveData(SPI2); //����ͨ��SPIx������յ�����	
 		    
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPIд�Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      val     ��Ҫд�������
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm_spi_w_reg_byte(uint8_t cmd, uint8_t val)
{
	  uint8_t dat[2];
  	ICM_CS=0;
    dat[0] = cmd | ICM20602_SPI_W;
    dat[1] = val;	
	 
	  spi_mosi(dat, dat, 2);
	
//		SPI1_ReadWriteByte(dat[0]);   //����дȡ״̬�Ĵ�������    
//	  SPI1_ReadWriteByte(dat[1]);               //д��һ���ֽ�    
    ICM_CS=1;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPI���Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      *val    �������ݵĵ�ַ
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------

void icm_spi_r_reg_byte(uint8_t cmd, uint8_t *val)
{
    uint8_t dat[2];

    ICM_CS=0;
    dat[0] = cmd | ICM20602_SPI_R;
	  dat[1] = *val;
	  spi_mosi(dat, dat, 2);
//    dat[0] = SPI1_ReadWriteByte(dat[0]);
//	  dat[1] = SPI1_ReadWriteByte(dat[1]);
    *val=dat[1];
	  ICM_CS=1;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602 SPI���ֽڶ��Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      *val    �������ݵĵ�ַ
//  @param      num     ��ȡ����
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm_spi_r_reg_bytes(uint8_t * val, uint8_t num)
{
//	  u8 i;
    ICM_CS=0;
	  spi_mosi(val, val, num);
//	  for(i=0;i<num;i++)
//	{
//	   *(val+i)=SPI1_ReadWriteByte(*val);
//	}
    ICM_CS=1;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM20602�Լ캯��
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm20602_self3_check(void)
{
    uint8_t dat=0;
    while(0x12 != dat)   //��ȡICM20602 ID
    {
        icm_spi_r_reg_byte(ICM20602_WHO_AM_I,&dat);
			  rt_kprintf("error:icm2006:%d\r\n",dat);
        delay_ms(10);
        //��������ԭ�������¼���
        //1 ���ˣ�������µ������ĸ��ʼ���
        //2 ���ߴ������û�нӺ�
        //3 ��������Ҫ����������裬������3.3V
    }

}
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
	SPI2->CR1&=0XFFC7;//λ3-5���㣬�������ò�����
	SPI2->CR1|=SPI_BaudRatePrescaler;	//����SPI1�ٶ� 
	SPI_Cmd(SPI2,ENABLE); //ʹ��SPI1
} 
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʼ��ICM20602
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_spi(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
 
  //GPIOFB3,4,5��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
	ICM_CS=1;
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2); 
 
	//����ֻ���SPI�ڳ�ʼ��
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//��λSPI1
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//ֹͣ��λSPI1

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 10;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����

  SPI2_SetSpeed(SPI_BaudRatePrescaler_8);		
  uint8_t val = 0x0;
  icm20602_self3_check();//���

  icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80);//��λ�豸
  delay_ms(20);
  do
  {   //�ȴ���λ�ɹ�
      icm_spi_r_reg_byte(ICM20602_PWR_MGMT_1,&val);
  } while(0x41 != val);

  icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,     0x01);            //ʱ������
  icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2,     0x00);            //���������Ǻͼ��ٶȼ�
  icm_spi_w_reg_byte(ICM20602_CONFIG,         0x01);            //176HZ 1KHZ
  icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV,     0x07);            //�������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
  icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG,    0x18);            //��2000 dps
  icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG,   0x10);            //��8g
  icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2, 0x03);            //Average 4 samples   44.8HZ   //0x23 Average 16 samples
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata_spi(void)
{
    struct
    {
        uint8_t reg;
        uint8_t dat[6];
    } buf;

    buf.reg = ICM20602_ACCEL_XOUT_H | ICM20602_SPI_R;

    icm_spi_r_reg_bytes(&buf.reg, 7);
    icm_acc_x = (int16_t)(((uint16_t)buf.dat[0]<<8 | buf.dat[1]));
    icm_acc_y = (int16_t)(((uint16_t)buf.dat[2]<<8 | buf.dat[3]));
    icm_acc_z = (int16_t)(((uint16_t)buf.dat[4]<<8 | buf.dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM20602����������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro_spi(void)
{
    struct
    {
        uint8_t reg;
        uint8_t dat[6];
    } buf;

    buf.reg = ICM20602_GYRO_XOUT_H | ICM20602_SPI_R;

    icm_spi_r_reg_bytes(&buf.reg, 7);
    icm_gyro_x = (int16_t)(((uint16_t)buf.dat[0]<<8 | buf.dat[1]));
    icm_gyro_y = (int16_t)(((uint16_t)buf.dat[2]<<8 | buf.dat[3]));
    icm_gyro_z = (int16_t)(((uint16_t)buf.dat[4]<<8 | buf.dat[5]));
}

