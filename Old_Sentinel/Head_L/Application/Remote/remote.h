#ifndef __REMOTE_H
#define __REMOTE_H

#include "stm32f4xx_hal.h"

/*****************************************************************/
#define REMOTE_OFFLINE_CNT_MAX  30          //ң����ʧ����������
/* ��ⰴ������ʱ�� */
#define MOUSE_BTN_L_CNT_MAX     300         //ms ������
#define MOUSE_BTN_R_CNT_MAX     500         //ms ����Ҽ�
#define KEY_Q_CNT_MAX           500         //ms Q��
#define KEY_W_CNT_MAX           2000         //ms W��
#define KEY_E_CNT_MAX           500         //ms E��
#define KEY_R_CNT_MAX           500         //ms R��
#define KEY_A_CNT_MAX           2000         //ms A��
#define KEY_S_CNT_MAX           2000         //ms S��
#define KEY_D_CNT_MAX           2000         //ms D��
#define KEY_F_CNT_MAX           500         //ms F��
#define KEY_G_CNT_MAX           500         //ms G��
#define KEY_Z_CNT_MAX           500         //ms Z��
#define KEY_X_CNT_MAX           500         //ms X��
#define KEY_C_CNT_MAX           500         //ms C��
#define KEY_V_CNT_MAX           500         //ms V��
#define KEY_B_CNT_MAX           500         //ms B��
#define KEY_SHIFT_CNT_MAX       500         //ms SHIFT��
#define KEY_CTRL_CNT_MAX        500         //ms CTRL��
/* ƽ���˲����� */
#define REMOTE_SMOOTH_TIMES     10          //���ƽ���˲�����
/* ��ť�ٽ�ֵ */
#define WHEEL_JUMP_VALUE        550         //��ť�����ж�ֵ

/*****************************************************************/

/* ����״̬ö�� */
typedef enum
{
  relax_K,        //����
  down_K,         //����
  up_K,           //̧��
  short_press_K,  //�̰�
  long_press_K,   //����
}key_board_status_e;

/* ң����������ť״̬ö�� */
typedef enum 
{
  keep_R,         //����
  up_R,           //���ϲ�
  mid_R,          //���в�
  down_R,         //���²�
}remote_status_e;

/* ������Ϣ */
typedef struct
{
  uint8_t value;    //ֵ
  uint8_t status;   //״̬
  int16_t cnt;      //��ǰ����
  int16_t cnt_max;  //��������
}key_board_info_t;

/* ������Ϣ */
typedef struct
{
  uint8_t value_last;  //��һ��ֵ
  uint8_t value;       //��ֵ
  uint8_t status;      //״̬
}remote_switch_info_t;

/* ��ť��Ϣ */
typedef struct
{
  int16_t value_last;  //��һ��ֵ
  int16_t value;       //��ֵ
  uint8_t status;      //״̬
	uint8_t status_last;
}remote_wheel_info_t;

/* ң��ԭʼ��Ϣ */
typedef struct 
{
  /* ң���� */
  int16_t                 ch0;                  //�ҵ�����
  int16_t                 ch1;                  //�ҵ�ǰ��
  int16_t                 ch2;                  //�������
  int16_t                 ch3;                  //���ǰ��
  remote_switch_info_t    s1;                   //�󲦸�
  remote_switch_info_t    s2;                   //�Ҳ���
  remote_wheel_info_t     thumbwheel;           //����Ť
  /* ���� */
  int16_t                 mouse_vx;             //���x���ٶ�
  int16_t                 mouse_vy;             //���y���ٶ�
  int16_t                 mouse_vz;             //���z���ٶ�
  key_board_info_t        mouse_btn_l;          //������
  key_board_info_t        mouse_btn_r;          //����Ҽ�
  key_board_info_t        Q;                    //����Q
  key_board_info_t        W;                    //����W
  key_board_info_t        E;                    //����E
  key_board_info_t        R;                    //����R
  key_board_info_t        A;                    //����A
  key_board_info_t        S;                    //����S
  key_board_info_t        D;                    //����D
  key_board_info_t        F;                    //����F
  key_board_info_t        G;                    //����G
  key_board_info_t        Z;                    //����Z
  key_board_info_t        X;                    //����X
  key_board_info_t        C;                    //����C
  key_board_info_t        V;                    //����V
  key_board_info_t        B;                    //����B
  key_board_info_t        Shift;                //����Shift
  key_board_info_t        Ctrl;                 //����Ctrl
}rc_base_info_t;

/* ң����Ϣ */
typedef struct 
{
	int16_t             offline_cnt;  //ʧ������
	uint8_t             status;       //״̬
	float				mouse_x;      //���x���ٶ�
	float  				mouse_y;      //���y���ٶ�
	float               mouse_x_K;    //���x���˲����ٶ�
	float  				mouse_y_K;    //���y���˲����ٶ�
}rc_info_t;

/* ң�� */
typedef struct
{
  rc_base_info_t     *base_info;
  rc_info_t          *info;
}rc_t;

/* �ⲿ���� */
extern rc_t rc;

/* ��ʼ�� */

void rc_init(rc_t *rc, rc_info_t *info, rc_base_info_t *base_info);

/* �ж� */
void rc_interrupt_update(rc_t *rc);

/* �δ����� */
void rc_tick_task(rc_t *rc);

/* �������� */
void rc_ctrl(rc_t *rc);

/* ״̬���� */
void key_board_status_update(key_board_info_t *key);
void all_key_board_status_update(rc_base_info_t *info);
/* �ж�״̬���� */
void rc_switch_status_interrupt_update(rc_base_info_t *info);
void rc_wheel_status_interrupt_update(rc_base_info_t *info);
void key_board_status_interrupt_update(key_board_info_t *key);

void remote_soft_reset_check(rc_t *rc);
void all_key_board_status_interrupt_update(rc_base_info_t *info);
void key_board_status_update(key_board_info_t *key);
void key_board_cnt_max_set(rc_base_info_t *info);

void rc_base_info_update(rc_base_info_t *info, uint8_t *rxBuf);
void rc_base_info_check(rc_base_info_t *info);


/*****************************************************************/

#endif
