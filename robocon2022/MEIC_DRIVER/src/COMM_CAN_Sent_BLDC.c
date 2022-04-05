/**
	*	本杰明方案无刷电机驱动板之CAN传输协议(主机发送)
	*	
	*	注意事项(仅对配套驱动板):
	*		1.波特率1M
	*		2.CAN传输函数预先自己填写!!
	*		3.使用拓展ID
	*		4.ID的低8位为 对象;其余位为 命令;
	*		5.全体广播ID为 0XFF 驱动板独立ID为APP内设
	*		6.CAN控制电机须重复发送 指令最大间隔为2秒 超2秒会导致超时启动保护程序(电机会停,但如果重复发送间隔超过2秒会跑得不流畅)
	*		7.添加支持匿名联合(#pragma anon_unions)
	*
	*	更新日志:
	*		2020.10.10:
	*			*更新成VSEC 5.01驱动方案
	*			-删除了真浮点传输
	*			+增加了官方格式的接码
	*
	*	版本:V1.1.0
	* 
	*	更新时间:2020.10.10
	*
	* 作者:MEIC战队
	*/


#include "COMM_CAN_Sent_BLDC.h"
#include "string.h"


/**************************自定义一个CAN传输函数********************************/
#define CANTRANSMIT 0 //0:使用默认程序 1:自定义
#ifdef CANTRANSMIT
#if !CANTRANSMIT
// 默认程序为STMF103正点原子的CAN例程测试-已测试 
#include "stdio.h"
static int canTransmit(const CANTxFrame *ctfp)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
//	TxMessage.StdId		=	ctfp->SID;																									// 标准标识符 
	TxMessage.ExtId		=	ctfp->EID;																									// 设置扩展标示符 
	TxMessage.IDE			=	CAN_Id_Extended; 																						// 扩展帧
	TxMessage.RTR			=	CAN_RTR_Data;																								// 数据帧
	TxMessage.DLC			=	ctfp->DLC;																									// 要发送的数据长度
	
	for(i=0;i<ctfp->DLC;i++)
	TxMessage.Data[i]=ctfp->data8[i];			          
	mbox= CAN_Transmit(CAN2, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;			// 等待发送结束
	if(i>=0XFFF)return 1;
	return 0;	 
}
#else
static void canTransmit(const CANTxFrame *ctfp)
{
	//填在这
}
#endif
#else
#error  undeclared identifier CANTRANSMIT //没有自己创建一个CAN传输函数
#endif

union _unfloat_int{
	uint32_t _32uint;
	float   _32float;
}unfloat_int;




int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}


float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)buffer_get_int32(buffer, index) / scale;
}


int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
	int16_t res =	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}



void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

void comm_can_transmit_eid(uint32_t id, uint8_t *data, uint8_t len) {
	CANTxFrame txmsg;

	if (len > 8) {	len = 8;	}
	
	txmsg.IDE = CAN_IDE_EXT;
	txmsg.EID = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.data8, data, len);

	canTransmit(&txmsg);

}

void comm_can_transmit_sid(uint32_t id, uint8_t *data, uint8_t len) {
	
	CANTxFrame txmsg;
	
	if (len > 8) {	len = 8;	}
	
	txmsg.IDE = CAN_IDE_STD;
	txmsg.SID = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.data8, data, len);

	canTransmit(&txmsg);
}

void comm_can_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

void comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void comm_can_set_current_brake(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0; 
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

void comm_can_set_pos(uint8_t controller_id, float pos) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer,(int32_t)(pos * 1000000.0), &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

/** 设置电流相对于最小和最大电流限制。
 * Set current relative to the minimum and maximum current limits.
 *
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [-1.0f 1.0f]
 */
void comm_can_set_current_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5f, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
}

/** 设置制动电流相对于最小电流限制。
 * Set brake current relative to the minimum current limit.
 *
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [0.0f 1.0f]
 */
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5f, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index);
}

/** 设置手刹电流
 * Set handbrake current.
 *
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The handbrake current value
 */
void comm_can_set_handbrake(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current, 1e3f, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, send_index);
}

/**
 * Set handbrake current relative to the minimum current limit.
 *
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The relative handbrake current value, range [0.0f 1.0f]
 */
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5f, &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, send_index);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
// 以下是接收部分

CAN_VESC_RE vesc_re[4]={'\0'};


void COMM_CAN_RE_VESC_decoding(CanRxMsg *RxMessage)
{
	int ind=0;	
	uint8_t	cmd=	(RxMessage->ExtId)>>8;
	uint8_t *data8 = RxMessage->Data;
    CAN_VESC_RE *re;
	
    switch((RxMessage->ExtId)&0xff)
    {
        case 90:      //这里判断ID 1号电机
            re = &vesc_re[0];
        break;
        case 4:      //这里判断ID  2号电机
            re = &vesc_re[1];
        break;
				case 89:      //这里判断ID  2号电机
            re = &vesc_re[2];
        break;
				case 20:      //这里判断ID  2号电机
            re = &vesc_re[3];
        break;

    default:   //printf("%d\r\n",(RxMessage->ExtId)&0xff);
               return;     //对不上直接返回防止调用野指针
    }

	
	switch(cmd) {
	case CAN_PACKET_STATUS:
				re->mgs1.rpm = (float)buffer_get_int32(data8, &ind);
				re->mgs1.current = (float)buffer_get_int16(data8, &ind) / 10.0;
				re->mgs1.duty = (float)buffer_get_int16(data8, &ind) / 1000.0;
				break;

	case CAN_PACKET_STATUS_2:
				re->mgs2.amp_hours = (float)buffer_get_int32(data8, &ind) / 1e4;
				re->mgs2.amp_hours_charged = (float)buffer_get_int32(data8, &ind) / 1e4;
				break;

	case CAN_PACKET_STATUS_3:
				re->mgs3.watt_hours = (float)buffer_get_int32(data8, &ind) / 1e4;
				re->mgs3.watt_hours_charged = (float)buffer_get_int32(data8, &ind) / 1e4;
				break;

	case CAN_PACKET_STATUS_4:
				re->mgs4.temp_fet = (float)buffer_get_int16(data8, &ind) / 10.0;
				re->mgs4.temp_motor = (float)buffer_get_int16(data8, &ind) / 10.0;
				re->mgs4.current_in = (float)buffer_get_int16(data8, &ind) / 10.0;
				re->mgs4.pid_pos_now = (float)buffer_get_int16(data8, &ind) / 50.0;
				break;

	case CAN_PACKET_STATUS_5:
				re->mgs5.tacho_value = buffer_get_int32(data8, &ind);
				re->mgs5.v_in = (float)buffer_get_int16(data8, &ind) / 1e1;
				break;
	
	default:
		break;
	}
}



void printf_CAN_test(void)
{
//  printf("TAC:%d\r\n",-(int)vesc_re[1].mgs5.tacho_value);
//  printf("RPM:%d\t%d\r\n",(int)vesc_re[0].mgs1.rpm,(int)-vesc_re[1].mgs1.rpm); 
  	printf("rpm:%d\t i:%d\t D:%d\r\n",(int)vesc_re[0].mgs1.rpm,(int)(vesc_re[0].mgs1.current*100),(int)(vesc_re[0].mgs1.duty*100));
//	printf("pos:%d\t%d\r\n",(int)(vesc_re[1].mgs1.rpm),(int)(vesc_re[1].mgs4.serial_pos));
//  printf("motor_right:rpm:%d\t i:%d\t D:%d\t",(int)vesc_re[1].mgs1.rpm,(int)(vesc_re[1].mgs1.current*100),(int)(vesc_re[1].mgs1.duty*100));
//  printf("mptor_right:pos:%d.%02d\r\n",(int)(vesc_re[0].mgs4.pid_pos_now),(int)(vesc_re[1].mgs4.pid_pos_now*100)%100);
}





