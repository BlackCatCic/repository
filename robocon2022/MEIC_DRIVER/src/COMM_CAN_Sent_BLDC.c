/**
	*	������������ˢ���������֮CAN����Э��(��������)
	*	
	*	ע������(��������������):
	*		1.������1M
	*		2.CAN���亯��Ԥ���Լ���д!!
	*		3.ʹ����չID
	*		4.ID�ĵ�8λΪ ����;����λΪ ����;
	*		5.ȫ��㲥IDΪ 0XFF ���������IDΪAPP����
	*		6.CAN���Ƶ�����ظ����� ָ�������Ϊ2�� ��2��ᵼ�³�ʱ������������(�����ͣ,������ظ����ͼ������2����ܵò�����)
	*		7.���֧����������(#pragma anon_unions)
	*
	*	������־:
	*		2020.10.10:
	*			*���³�VSEC 5.01��������
	*			-ɾ�����渡�㴫��
	*			+�����˹ٷ���ʽ�Ľ���
	*
	*	�汾:V1.1.0
	* 
	*	����ʱ��:2020.10.10
	*
	* ����:MEICս��
	*/


#include "COMM_CAN_Sent_BLDC.h"
#include "string.h"


/**************************�Զ���һ��CAN���亯��********************************/
#define CANTRANSMIT 0 //0:ʹ��Ĭ�ϳ��� 1:�Զ���
#ifdef CANTRANSMIT
#if !CANTRANSMIT
// Ĭ�ϳ���ΪSTMF103����ԭ�ӵ�CAN���̲���-�Ѳ��� 
#include "stdio.h"
static int canTransmit(const CANTxFrame *ctfp)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
//	TxMessage.StdId		=	ctfp->SID;																									// ��׼��ʶ�� 
	TxMessage.ExtId		=	ctfp->EID;																									// ������չ��ʾ�� 
	TxMessage.IDE			=	CAN_Id_Extended; 																						// ��չ֡
	TxMessage.RTR			=	CAN_RTR_Data;																								// ����֡
	TxMessage.DLC			=	ctfp->DLC;																									// Ҫ���͵����ݳ���
	
	for(i=0;i<ctfp->DLC;i++)
	TxMessage.Data[i]=ctfp->data8[i];			          
	mbox= CAN_Transmit(CAN2, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;			// �ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;	 
}
#else
static void canTransmit(const CANTxFrame *ctfp)
{
	//������
}
#endif
#else
#error  undeclared identifier CANTRANSMIT //û���Լ�����һ��CAN���亯��
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

/** ���õ����������С�����������ơ�
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

/** �����ƶ������������С�������ơ�
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

/** ������ɲ����
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
// �����ǽ��ղ���

CAN_VESC_RE vesc_re[4]={'\0'};


void COMM_CAN_RE_VESC_decoding(CanRxMsg *RxMessage)
{
	int ind=0;	
	uint8_t	cmd=	(RxMessage->ExtId)>>8;
	uint8_t *data8 = RxMessage->Data;
    CAN_VESC_RE *re;
	
    switch((RxMessage->ExtId)&0xff)
    {
        case 90:      //�����ж�ID 1�ŵ��
            re = &vesc_re[0];
        break;
        case 4:      //�����ж�ID  2�ŵ��
            re = &vesc_re[1];
        break;
				case 89:      //�����ж�ID  2�ŵ��
            re = &vesc_re[2];
        break;
				case 20:      //�����ж�ID  2�ŵ��
            re = &vesc_re[3];
        break;

    default:   //printf("%d\r\n",(RxMessage->ExtId)&0xff);
               return;     //�Բ���ֱ�ӷ��ط�ֹ����Ұָ��
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





