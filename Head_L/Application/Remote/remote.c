

#include "remote.h"

/**
  * @brief  按键长按时间设置
  */
void key_board_cnt_max_set(rc_base_info_t *info)
{
  info->mouse_btn_l.cnt_max = MOUSE_BTN_L_CNT_MAX;
  info->mouse_btn_r.cnt_max = MOUSE_BTN_R_CNT_MAX;
  info->Q.cnt_max = KEY_Q_CNT_MAX;
  info->W.cnt_max = KEY_W_CNT_MAX;
  info->E.cnt_max = KEY_E_CNT_MAX;
  info->R.cnt_max = KEY_R_CNT_MAX;
  info->A.cnt_max = KEY_A_CNT_MAX;
  info->S.cnt_max = KEY_S_CNT_MAX;
  info->D.cnt_max = KEY_D_CNT_MAX;
  info->F.cnt_max = KEY_F_CNT_MAX;
  info->G.cnt_max = KEY_G_CNT_MAX;
  info->Z.cnt_max = KEY_Z_CNT_MAX;
  info->X.cnt_max = KEY_X_CNT_MAX;
  info->C.cnt_max = KEY_C_CNT_MAX;
  info->V.cnt_max = KEY_V_CNT_MAX;
  info->B.cnt_max = KEY_B_CNT_MAX;
  info->Shift.cnt_max = KEY_SHIFT_CNT_MAX;
  info->Ctrl.cnt_max = KEY_CTRL_CNT_MAX;
}


/**
  * @brief  遥控信息更新
  */
void rc_base_info_update(rc_base_info_t *info, uint8_t *rxBuf)
{
  info->ch0 = (rxBuf[0]      | rxBuf[1] << 8                 ) & 0x07FF;
  info->ch0 -= 1024;
  info->ch1 = (rxBuf[1] >> 3 | rxBuf[2] << 5                 ) & 0x07FF;
  info->ch1 -= 1024;
  info->ch2 = (rxBuf[2] >> 6 | rxBuf[3] << 2 | rxBuf[4] << 10) & 0x07FF;
  info->ch2 -= 1024;
  info->ch3 = (rxBuf[4] >> 1 | rxBuf[5] << 7                 ) & 0x07FF;
  info->ch3 -= 1024;
  info->s1.value = ((rxBuf[5] >> 4) & 0x000C) >> 2;
  info->s2.value = ( rxBuf[5] >> 4) & 0x0003;
	
	
  info->mouse_vx = rxBuf[6]  | (rxBuf[7 ] << 8);
  info->mouse_vy = rxBuf[8]  | (rxBuf[9 ] << 8);
  info->mouse_vz = rxBuf[10] | (rxBuf[11] << 8);
  info->mouse_btn_l.value = rxBuf[12] & 0x01;
  info->mouse_btn_r.value = rxBuf[13] & 0x01;
  info->W.value =   rxBuf[14]        & 0x01;
  info->S.value = ( rxBuf[14] >> 1 ) & 0x01;
  info->A.value = ( rxBuf[14] >> 2 ) & 0x01;
  info->D.value = ( rxBuf[14] >> 3 ) & 0x01;
  info->Shift.value = ( rxBuf[14] >> 4 ) & 0x01;
  info->Ctrl.value = ( rxBuf[14] >> 5 ) & 0x01;
  info->Q.value = ( rxBuf[14] >> 6 ) & 0x01 ;
  info->E.value = ( rxBuf[14] >> 7 ) & 0x01 ;
  info->R.value = ( rxBuf[15] >> 0 ) & 0x01 ;
  info->F.value = ( rxBuf[15] >> 1 ) & 0x01 ;
  info->G.value = ( rxBuf[15] >> 2 ) & 0x01 ;
  info->Z.value = ( rxBuf[15] >> 3 ) & 0x01 ;
  info->X.value = ( rxBuf[15] >> 4 ) & 0x01 ;
  info->C.value = ( rxBuf[15] >> 5 ) & 0x01 ;
  info->V.value = ( rxBuf[15] >> 6 ) & 0x01 ;
  info->B.value = ( rxBuf[15] >> 7 ) & 0x01 ;

  info->thumbwheel.value = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff;
  info->thumbwheel.value -= 1024;
	
	
	
}


void rc_init(rc_t *rc, rc_info_t *info, rc_base_info_t *base_info)
{
	rc->base_info = base_info;
	rc->info      = info;
	
	
	/* 基本信息置零 */
	memset(base_info,0,sizeof(rc_base_info_t));
	/* 拨杆旋钮状态初始化 */
	base_info->s1.status = keep_R;
	base_info->s2.status = keep_R;
	base_info->thumbwheel.status = keep_R;
	
	
	/* 设置失联计数 */
	info->offline_cnt = REMOTE_OFFLINE_CNT_MAX;
	
	key_board_cnt_max_set(base_info);
	
	/* 设置状态 */
	// info->status = DEV_OFFLINE;
	
}




void rc_switch_status_interrupt_update(rc_base_info_t *info)
{
  /* 左拨杆判断 */
  if(info->s1.value != info->s1.value_last)
  {
    switch(info->s1.value)
    {
      case 1:
        info->s1.status = up_R;
        break;
      case 3:
        info->s1.status = mid_R;
        break;
      case 2:
        info->s1.status = down_R;
        break;
      default:
        break;
    }
    info->s1.value_last = info->s1.value;
  }
  else 
  {
    info->s1.status = keep_R;
  }
  /* 右拨杆判断 */
  if(info->s2.value != info->s2.value_last)
  {
    switch(info->s2.value)
    {
      case 1:
        info->s2.status = up_R;
        break;
      case 3:
        info->s2.status = mid_R;
        break;
      case 2:
        info->s2.status = down_R;
        break;
      default:
        break;
    }
    info->s2.value_last = info->s2.value;
  }
  else 
  {
    info->s2.status = keep_R;
  }
}

/**
  * @brief  遥控器旋钮状态跳变判断并更新
  */
void rc_wheel_status_interrupt_update(rc_base_info_t *info)
{
  if(abs(info->thumbwheel.value_last) < WHEEL_JUMP_VALUE)
  {
    if(info->thumbwheel.value > WHEEL_JUMP_VALUE)
    {
      info->thumbwheel.status = up_R;
    }
    else if(info->thumbwheel.value < -WHEEL_JUMP_VALUE)
    {
      info->thumbwheel.status = down_R;
    }
    else 
    {
      info->thumbwheel.status = keep_R;
    }
  }
  else 
  {
    info->thumbwheel.status = keep_R;
  }
  info->thumbwheel.value_last = info->thumbwheel.value;
	
}

//键盘控制
/**
  * @brief  键盘按键状态判断并更新
  */
void key_board_status_update(key_board_info_t *key)
{
  switch(key->status)
  {
    case down_K:
      key->status = short_press_K;
      key->cnt++;
      break;
    case up_K:
      key->status = relax_K;
			key->cnt = 0;
      break;
    case short_press_K:
      key->cnt++;
      if(key->cnt >= key->cnt_max)
      {
        key->value = long_press_K;
				key->cnt = key->cnt_max;
      }
      break;
    default:
      break;
  }
}



/**
  * @brief  遥控器接收产生中断时键盘按键状态判断并更新
  */
void key_board_status_interrupt_update(key_board_info_t *key)
{
  switch(key->status)
  {
    case relax_K:
      if(key->value == 1)
      {
        key->status = down_K;
        key->cnt = 0;
      }
      break;
    case short_press_K:
      if(key->value == 0)
      {
        key->status = up_K;
				key->cnt = 0;
      }
      else if(key->value == 1)
      {
        key->cnt++;
        if(key->cnt >= key->cnt_max)
        {
          key->status = long_press_K;
					key->cnt = key->cnt_max;
        }
      }
      break;
    case long_press_K:
      if(key->value == 0)
      {
        key->status = up_K;
				key->cnt = 0;
      }
      break;
    default:
      break;
  }
}
/**
  * @brief  遥控器接收产生中断时所有键盘按键状态判断并更新
  */
void all_key_board_status_interrupt_update(rc_base_info_t *info)
{
  key_board_status_interrupt_update(&info->mouse_btn_l);
  key_board_status_interrupt_update(&info->mouse_btn_r);
  key_board_status_interrupt_update(&info->Q);
  key_board_status_interrupt_update(&info->W);
  key_board_status_interrupt_update(&info->E);
  key_board_status_interrupt_update(&info->R);
  key_board_status_interrupt_update(&info->A);
  key_board_status_interrupt_update(&info->S);
  key_board_status_interrupt_update(&info->D);
  key_board_status_interrupt_update(&info->F);
  key_board_status_interrupt_update(&info->G);
  key_board_status_interrupt_update(&info->Z);
  key_board_status_interrupt_update(&info->X);
  key_board_status_interrupt_update(&info->C);
  key_board_status_interrupt_update(&info->V);
  key_board_status_interrupt_update(&info->B);
  key_board_status_interrupt_update(&info->Shift);
  key_board_status_interrupt_update(&info->Ctrl);
}

/**
  * @brief  所有键盘按键状态判断并更新
  */
void all_key_board_status_update(rc_base_info_t *info)
{
  key_board_status_update(&info->mouse_btn_l);
  key_board_status_update(&info->mouse_btn_r);
  key_board_status_update(&info->Q);
  key_board_status_update(&info->W);
  key_board_status_update(&info->E);
  key_board_status_update(&info->R);
  key_board_status_update(&info->A);
  key_board_status_update(&info->S);
  key_board_status_update(&info->D);
  key_board_status_update(&info->F);
  key_board_status_update(&info->G);
  key_board_status_update(&info->Z);
  key_board_status_update(&info->X);
  key_board_status_update(&info->C);
  key_board_status_update(&info->V);
  key_board_status_update(&info->B);
  key_board_status_update(&info->Shift);
  key_board_status_update(&info->Ctrl);
	
	

}



