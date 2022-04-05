#ifndef _DRIVER_DBUS_H
#define _DRIVER_DBUS_H
#include "Include.h"


void DBUS_Init(uint8_t *rx1_buf, uint16_t dma_buf_num);
 
#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)


typedef   signed          char int8_t;
typedef   signed short     int int16_t;
/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
  __packed struct
  {
          int16_t ch[5];
          char s[2];
  } rc;
  __packed struct
  {
          int16_t x;
          int16_t y;
          int16_t z;
          uint8_t press_l;
          uint8_t press_r;
  } mouse;
  __packed struct
  {
          uint16_t v;
  } key;

} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern RC_ctrl_t rc_ctrl;

#define  RC_right_x get_remote_control_point()->rc.ch[0]
#define  RC_right_y get_remote_control_point()->rc.ch[1]
#define  RC_left_x get_remote_control_point()->rc.ch[2]
#define  RC_left_y get_remote_control_point()->rc.ch[3]
#define  RC_left_mode get_remote_control_point()->rc.s[0]
#define  RC_right_mode get_remote_control_point()->rc.s[1]


#endif

