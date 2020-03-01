/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能，其中射击的初始化，以及循环都是在云台任务中调用，故而此文件
  *             不是freeRTOS任务，射击分关闭状态，准备状态，射击状态，以及完成状态
  *             关闭状态是关闭摩擦轮以及激光，准备状态是将子弹拨到微型开关处，射击状
  *             态是将子弹射出，判断微型开关值，进入完成状态，完成状态通过判断一定时间
  *             微型开关无子弹认为已经将子弹射出。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "main.h"
#include "CAN_Receive.h"
#include "remote_control.h"
#include "user_lib.h"
#include "Gimbal_Task.h"
//射击发射开关通道数据
#define Shoot_RC_Channel    1
//云台模式使用的开关通道
#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    500.0f

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E

//大小发射机构切换
#define SHOOT_BIG_ON_KEYBOARD KEY_PRESSED_OFFSET_R
#define SHOOT_SMALL_ON_KEYBOARD KEY_PRESSED_OFFSET_F

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME 10
//鼠标长按判断
#define PRESS_LONG_TIME 400
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME 2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME 80
//电机反馈码盘值范围
#define Half_ecd_range 4096
#define ecd_range 8191
//电机rmp 变化成 旋转速度的比例
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18
//拨弹速度
#define TRIGGER_SPEED 10.0f
#define Ready_Trigger_Speed 5.0f
#define SWITCH_TRIGGER_OFF 1.0f

#define KEY_OFF_JUGUE_TIME 500
//#define SWITCH_TRIGGER_ON 0
//#define SWITCH_TRIGGER_OFF 1		//微动开关电控设置，当前英雄微动开关仅为机械限位，未使用电控控制

//卡单时间 以及反转时间
#define BLOCK_TIME 700
#define REVERSE_TIME 500
#define REVERSE_SPEED_LIMIT 13.0f

#define PI_Four 0.78539816339744830961566084581988f
#define PI_Ten 0.314f

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP 800.0f
#define TRIGGER_ANGLE_PID_KI 0.5f
#define TRIGGER_ANGLE_PID_KD 0.0f

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define TRIGGER_READY_PID_MAX_OUT 5000.0f
#define TRIGGER_READY_PID_MAX_IOUT 2500.0f

//大拨弹轮电机PID
#define SHOOT_BIG_TRIGGER_SPEED_PID_KP 10.0f
#define SHOOT_BIG_TRIGGER_SPEED_PID_KI 0.0f
#define SHOOT_BIG_TRIGGER_SPEED_PID_KD 0.0f
#define SHOOT_BIG_TRIGGER_SPEED_PID_MAX_OUT 16000.0f
#define SHOOT_BIG_TRIGGER_SPEED_PID_MAX_IOUT 0.0f

//大发射机构电机速度
#define SHOOT_BIG_FRIC_STD_SPEED	1300
#define SHOOT_BIG_FRIC_OFF	0
#define SHOOT_BIG_TRIGGER_ON	200
#define SHOOT_BIG_TRIGGER_OFF	0

//大发射机构限位电机
#define LIMIT_ANGLE_PID_KP 100.0f
#define LIMIT_ANGLE_PID_KI 0.0f
#define LIMIT_ANGLE_PID_KD 0.0f

#define LIMIT_PID_MAX_OUT 1000.0f
#define LIMIT_PID_MAX_IOUT 0.0f

#define LIMIT_READY_PID_MAX_OUT 1000.0f
#define LIMIT_READY_PID_MAX_IOUT 0.0f	//line 102-106当前值时，ready和发射的pid不同

//大发射机构限位电机拨弹速度
#define LIMIT_SPEED 1000.0f
#define Ready_LIMIT_Speed 1000.0f
#define LIMIT_SPEED_OFF 0.0f

#define KEY_OFF_JUGUE_TIME 500


typedef struct
{
		const motor_measure_t *shoot_big_motor_measure;
		PidTypeDef motor_shoot_big_speed_pid;
		fp32 speed_set;
		int16_t give_current;
} Shoot_Big_Motor_t;

typedef struct
{
		const motor_measure_t *shoot_big_limit_motor_measure;
		PidTypeDef motor_shoot_big_limit_speed_pid;
		fp32 speed_set;
		int16_t give_current;
} Shoot_Big_Limit_Motor_t;

typedef struct
{
		Shoot_Big_Motor_t motor_shoot_big[3];          			 //大发射机构电机数据
		Shoot_Big_Limit_Motor_t motor_shoot_limit;
} Shoot_Big_Control_t;

typedef struct
{
    ramp_function_source_t fric1_ramp;
    ramp_function_source_t fric2_ramp;
    const motor_measure_t *shoot_motor_measure;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    bool_t move_flag;
    uint32_t cmd_time;
    uint32_t run_time;
    bool_t key;
    uint16_t key_time;
    bool_t shoot_done;
    uint8_t shoot_done_time;
    int16_t BulletShootCnt;
    int16_t last_butter_count;
} Shoot_Motor_t;

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_DONE,
		SHOOT_REVERSE,
} shoot_mode_e;

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

#endif
