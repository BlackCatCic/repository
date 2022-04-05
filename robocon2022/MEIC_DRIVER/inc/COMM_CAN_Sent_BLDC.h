#ifndef __CAN_SENT_DRV8301_H
#define __CAN_SENT_DRV8301_H

#include "sys.h"


#define CAN_IDE_STD                 0           /**< @brief Standard id.    */
#define CAN_IDE_EXT                 1           /**< @brief Extended id.    */


// CAN commands CAN命令
typedef enum {
	CAN_PACKET_SET_DUTY = 0,  							//CAN设置	占空比
	CAN_PACKET_SET_CURRENT,									//CAN设置	电流
	CAN_PACKET_SET_CURRENT_BRAKE,						//CAN设置	刹车电流		
	CAN_PACKET_SET_RPM,											//CAN设置	转速
	CAN_PACKET_SET_POS,											//CAN设置	环置环(无感)
	CAN_PACKET_FILL_RX_BUFFER,							//CAN设置 填充RX缓冲区
	CAN_PACKET_FILL_RX_BUFFER_LONG,					//CAN设置 填充RX缓冲区LONG
	CAN_PACKET_PROCESS_RX_BUFFER,						//CAN设置 处理RX缓冲区	
	CAN_PACKET_PROCESS_SHORT_BUFFER,				//CAN设置 处理RX缓冲区SHORT
	CAN_PACKET_STATUS,				
	CAN_PACKET_SET_CURRENT_REL,							//CAN设置 电流上限
	CAN_PACKET_SET_CURRENT_BRAKE_REL,				//CAN设置 电流下限 0.0~1.0
	CAN_PACKET_SET_CURRENT_HANDBRAKE,				//CAN设置 手刹电流
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,		//CAN设置 手刹电流上限
	CAN_PACKET_STATUS_2,
	CAN_PACKET_STATUS_3,
	CAN_PACKET_STATUS_4,
	CAN_PACKET_PING,
	CAN_PACKET_PONG,
	CAN_PACKET_DETECT_APPLY_ALL_FOC,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
	CAN_PACKET_CONF_CURRENT_LIMITS,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_FOC_ERPMS,
	CAN_PACKET_CONF_STORE_FOC_ERPMS,
	CAN_PACKET_STATUS_5,
	CAN_PACKET_POLL_TS5700N8501_STATUS,
	CAN_PACKET_CONF_BATTERY_CUT,
	CAN_PACKET_CONF_STORE_BATTERY_CUT,
	CAN_PACKET_SHUTDOWN,
	//以下为添加补充
	CAN_PACKET_MAGN_POS,										//位置
	CAN_PACKET_SET_MAGN_POS,								//位置环(磁编码器)
	CAN_PACKET_SET_FEEDBACK_MOD,						//CAN返回数据模式 占空比 电流 速度 位置 关
	CAN_PACKET_SET_,
	//状态保持模式
	//

	
} CAN_PACKET_ID;




#pragma anon_unions //添加支持匿名联合
/**
 * @brief   CAN transmission frame.
 * @note    Accessing the frame data as word16 or word32 is not portable because
 *          machine data endianness, it can be still useful for a quick filling.
 */
typedef struct {  //CAN 传输帧框架
  struct {
    uint8_t                 DLC:4;          /**< @brief Data length.        */
    uint8_t                 RTR:1;          /**< @brief Frame type.         */
    uint8_t                 IDE:1;          /**< @brief Identifier type.    */
  };
  union {
    struct {
      uint32_t              SID:11;         /**< @brief Standard identifier.*/
    };
    struct {
      uint32_t              EID:29;         /**< @brief Extended identifier.*/
    };
  };
  union {
    uint8_t                 data8[8];       /**< @brief Frame data.         */
    uint16_t                data16[4];      /**< @brief Frame data.         */
    uint32_t                data32[2];      /**< @brief Frame data.         */
  };
} CANTxFrame;


typedef struct {
	float rpm;
	float current;
	float duty;
} can_status_msg;

typedef struct {
	float amp_hours;
	float amp_hours_charged;
} can_status_msg_2;

typedef struct {
	float watt_hours;
	float watt_hours_charged;
} can_status_msg_3;

typedef struct {
	float temp_fet;
	float temp_motor;
	float current_in;
	float pid_pos_now;
    float serial_pos;
    float pos_last;
    float  cur_count;
    float pos_err;
} can_status_msg_4;
typedef struct {
	float v_in;
	int32_t tacho_value;
} can_status_msg_5;


typedef struct {
    can_status_msg mgs1;
    can_status_msg_2 mgs2;
    can_status_msg_3 mgs3;
    can_status_msg_4 mgs4;
    can_status_msg_5 mgs5;    
} CAN_VESC_RE;


void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index);
void comm_can_transmit_eid(uint32_t id, uint8_t *data, uint8_t len);
void comm_can_transmit_sid(uint32_t id, uint8_t *data, uint8_t len);
void comm_can_set_duty(uint8_t controller_id, float duty);
void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_current_brake(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);//电角度 = 转子转速*极对数
void comm_can_set_pos(uint8_t controller_id, float pos);
void comm_can_set_current_rel(uint8_t controller_id, float current_rel);
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel);
void comm_can_set_handbrake(uint8_t controller_id, float current);
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel);

void COMM_CAN_RE_VESC_decoding(CanRxMsg *RxMessage);

void printf_CAN_test(void);
 
extern CAN_VESC_RE vesc_re[4];
#endif
