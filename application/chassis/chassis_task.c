/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "dbus.h"
#include "chassis_task.h"
#include "chassis_cmd.h"
#include "os_timer.h"
#include "infantry_cmd.h"
#include "board.h"
#include "event_mgr.h"
#include "event.h"
#include "chassis.h"
#include "offline_service.h"
#include "tim.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
typedef float fp32;


struct pid_param chassis_motor_param =
{
    .p = 6.5f,
    .i = 0.1f,
    .max_out = 15000,
    .integral_limit = 500,
};

static void chassis_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp);
static int32_t chassis_angle_broadcast(void *argv);

struct chassis chassis;
struct rc_device chassis_rc;
struct ahrs_sensor chassis_gyro;

/* chassis speed */
static float vx, vy, wz;

/* fllow control */
struct pid pid_follow = {0};
static uint8_t              chassis_can_send_data[8];
static uint8_t              gimbal_can_send_data[8];
float follow_relative_angle;
static CAN_TxHeaderTypeDef  chassis_tx_message;
static CAN_TxHeaderTypeDef  gimbal_tx_message;
void fric_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, cmd);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, cmd);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, cmd);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, cmd);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, cmd);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, cmd);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, cmd);
}

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void chassis_task(void const *argument)
{
    
	rc_info_t p_rc_info;

    subscriber_t listSubs;
    subscriber_t nolistSubs;

    EventSubscribeInit(&listSubs, SUBS_MODE_NORMAL);
    EventSubscribe(&listSubs, DBUS_MSG, DBUS_MSG_LEN, 3, chassis_dr16_data_update);

    EventSubscribeInit(&nolistSubs, SUBS_MODE_NOLIST);
    EventSubscribe(&nolistSubs, AHRS_MSG, AHRS_MSG_LEN, 0, NULL);

    rc_device_register(&chassis_rc, "Chassis RC");
    p_rc_info = rc_device_get_info(&chassis_rc);

    chassis_pid_init(&chassis, "Chassis", chassis_motor_param, DEVICE_CAN2);

    soft_timer_register((soft_timer_callback)chassis_pid_calculate, (void *)&chassis, 5);
    soft_timer_register((soft_timer_callback)chassis_angle_broadcast, (void *)NULL, 10);

    pid_struct_init(&pid_follow, MAX_CHASSIS_VW_SPEED, 50, 8.0f, 0.0f, 2.0f);
	HAL_TIM_Base_Start(&htim1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
   
   
    HAL_Delay(100);
    
		while (1)
    {int a;
			int b;
			fp32 pwm;
			//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);
			 
        /* dr16 data update */
        EventMsgProcess(&listSubs, 0);
        /* gyro data update */
        EventMsgGetLast(&nolistSubs, AHRS_MSG, &chassis_gyro, NULL);

        chassis_gyro_updata(&chassis, chassis_gyro.yaw * RAD_TO_DEG, chassis_gyro.gz * RAD_TO_DEG);

        if (rc_device_get_state(&chassis_rc, RC_S2_UP) == E_OK)
        {  //__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);
            //vx = (float)p_rc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED;
					
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1800);
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1800);
					
				
           vx = (float)p_rc_info->ch2*2 / 5280* MAX_CHASSIS_VX_SPEED;
            vy = -(float)p_rc_info->ch1*2/ 5280 * MAX_CHASSIS_VY_SPEED;
            wz = -(float)p_rc_info->ch3 / 5280 * MAX_CHASSIS_VW_SPEED;
            chassis_set_offset(&chassis, 0, 0);
            chassis_set_acc(&chassis, 0, 0, 0);
            chassis_set_speed(&chassis, vx, vy, wz);
        a = (float)p_rc_info->ch4*10;
					
					CAN_cmd_gimbal(0, 0, a, 0);
					}
        

        if (rc_device_get_state(&chassis_rc, RC_S2_MID) == E_OK)
					
        {__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1750);
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1750);
           b = (float)p_rc_info->ch4*10;
					CAN_cmd_gimbal(b, 0, 0, 0);
					
					
           if(p_rc_info->ch2>500){
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1700);}
					 else if (p_rc_info->ch2<-500){
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,2500 );}
					 else {
						 __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,2050 );}
         
					
        }

        if (rc_device_get_state(&chassis_rc, RC_S2_MID2DOWN) == E_OK)
        {
            chassis_set_speed(&chassis, 0, 0, 0);
            chassis_set_acc(&chassis, 0, 0, 0);
        }

        if (rc_device_get_state(&chassis_rc, RC_S2_MID2UP) == E_OK)
        {
            chassis_set_speed(&chassis, 0, 0, 0);
            chassis_set_acc(&chassis, 0, 0, 0);
        }

        if (rc_device_get_state(&chassis_rc, RC_S2_DOWN) == E_OK)
        {//pwm = 1000.0f + (p_rc_info->ch3 > 0? p_rc_info->ch3  : 0);
			if(p_rc_info->ch2>500){
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1400);}
			else if (p_rc_info->ch2<-500){
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1800);}
			
			if(p_rc_info->ch4>500){
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1400);}
 else if(p_rc_info->ch4<-500){
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2100);}
	a = (float)p_rc_info->ch1*10;
					
					CAN_cmd_gimbal(0, 0, a, 0);
					
					 b = (float)p_rc_info->ch3*10;
					CAN_cmd_gimbal(b, 0, 0, 0);
				//else{}
					
            //offline_event_enable(OFFLINE_MANIFOLD2_HEART);
           // offline_event_enable(OFFLINE_CONTROL_CMD);

            //if ((p_rc_info->ch1 < -400) && (p_rc_info->ch2 < -400) && (p_rc_info->ch3 > 400) && (p_rc_info->ch4 < -400))
            //{
               // static int cnt = 0;
                //cnt++;
                /* 2 second */
               // if (cnt > 400)
                //{
                  //  motor_auto_set_id(DEVICE_CAN2);
               // }
           // }
       // }
        //else
        //{
            /* disable sdk */
            //set_chassis_sdk_mode(CHASSIS_SDK_OFF);
            //offline_event_disable(OFFLINE_MANIFOLD2_HEART);
            //offline_event_disable(OFFLINE_CONTROL_CMD);
        }

        osDelay(5);
    }
	}

/**
  * @brief  send chassis angle to gimbal
  * @param
  * @retval void
  */
int32_t chassis_angle_broadcast(void *argv)
{
    int32_t s_yaw, s_yaw_rate;

    s_yaw = chassis.mecanum.gyro.yaw_gyro_angle * 1000;
    s_yaw_rate = chassis.mecanum.gyro.yaw_gyro_rate * 1000;

    uint8_t data[8];
    data[0] = s_yaw >> 24;
    data[1] = s_yaw >> 16;
    data[2] = s_yaw >> 8;
    data[3] = s_yaw;
    data[4] = s_yaw_rate >> 24;
    data[5] = s_yaw_rate >> 16;
    data[6] = s_yaw_rate >> 8;
    data[7] = s_yaw_rate;

    can1_std_transmit(0x401, data, 8);
    return 0;
}

struct chassis *get_chassis(void)
{
    return &chassis;
}

/**
  * @brief  subscrib dr16 event, update
  * @param
  * @retval void
  */
static void chassis_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp)
{
    rc_device_date_update(&chassis_rc, pMsgData);
}

/**
  * @brief  follow mode angle update
  * @param
  * @retval void
  */
int32_t follow_angle_info_rcv(uint8_t *buff, uint16_t len)
{
    struct cmd_gimbal_info *info;
    info = (struct cmd_gimbal_info *)buff;
    follow_relative_angle = info->yaw_ecd_angle / 10.0f;
    offline_event_time_update(OFFLINE_GIMBAL_INFO);
    return 0;
}

void set_follow_relative(float val)
{
    follow_relative_angle = val;
}
