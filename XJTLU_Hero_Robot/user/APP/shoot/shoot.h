/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܣ���������ĳ�ʼ�����Լ�ѭ����������̨�����е��ã��ʶ����ļ�
  *             ����freeRTOS��������ֹر�״̬��׼��״̬�����״̬���Լ����״̬
  *             �ر�״̬�ǹر�Ħ�����Լ����⣬׼��״̬�ǽ��ӵ�����΢�Ϳ��ش������״
  *             ̬�ǽ��ӵ�������ж�΢�Ϳ���ֵ���������״̬�����״̬ͨ���ж�һ��ʱ��
  *             ΢�Ϳ������ӵ���Ϊ�Ѿ����ӵ������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
//������俪��ͨ������
#define Shoot_RC_Channel    1
//��̨ģʽʹ�õĿ���ͨ��
#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    500.0f

//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E

//��С��������л�
#define SHOOT_BIG_ON_KEYBOARD KEY_PRESSED_OFFSET_R
#define SHOOT_SMALL_ON_KEYBOARD KEY_PRESSED_OFFSET_F

//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME 10
//��곤���ж�
#define PRESS_LONG_TIME 400
//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME 2000
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME 80
//�����������ֵ��Χ
#define Half_ecd_range 4096
#define ecd_range 8191
//���rmp �仯�� ��ת�ٶȵı���
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18
//�����ٶ�
#define TRIGGER_SPEED 10.0f
#define Ready_Trigger_Speed 5.0f
#define SWITCH_TRIGGER_OFF 1.0f

#define KEY_OFF_JUGUE_TIME 500
//#define SWITCH_TRIGGER_ON 0
//#define SWITCH_TRIGGER_OFF 1		//΢�����ص�����ã���ǰӢ��΢�����ؽ�Ϊ��е��λ��δʹ�õ�ؿ���

//����ʱ�� �Լ���תʱ��
#define BLOCK_TIME 700
#define REVERSE_TIME 500
#define REVERSE_SPEED_LIMIT 13.0f

#define PI_Four 0.78539816339744830961566084581988f
#define PI_Ten 0.314f

//�����ֵ��PID
#define TRIGGER_ANGLE_PID_KP 800.0f
#define TRIGGER_ANGLE_PID_KI 0.5f
#define TRIGGER_ANGLE_PID_KD 0.0f

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define TRIGGER_READY_PID_MAX_OUT 5000.0f
#define TRIGGER_READY_PID_MAX_IOUT 2500.0f

//�󲦵��ֵ��PID
#define SHOOT_BIG_TRIGGER_SPEED_PID_KP 10.0f
#define SHOOT_BIG_TRIGGER_SPEED_PID_KI 0.0f
#define SHOOT_BIG_TRIGGER_SPEED_PID_KD 0.0f
#define SHOOT_BIG_TRIGGER_SPEED_PID_MAX_OUT 16000.0f
#define SHOOT_BIG_TRIGGER_SPEED_PID_MAX_IOUT 0.0f

//�����������ٶ�
#define SHOOT_BIG_FRIC_STD_SPEED	1300
#define SHOOT_BIG_FRIC_OFF	0
#define SHOOT_BIG_TRIGGER_ON	200
#define SHOOT_BIG_TRIGGER_OFF	0

//���������λ���
#define LIMIT_ANGLE_PID_KP 100.0f
#define LIMIT_ANGLE_PID_KI 0.0f
#define LIMIT_ANGLE_PID_KD 0.0f

#define LIMIT_PID_MAX_OUT 1000.0f
#define LIMIT_PID_MAX_IOUT 0.0f

#define LIMIT_READY_PID_MAX_OUT 1000.0f
#define LIMIT_READY_PID_MAX_IOUT 0.0f	//line 102-106��ǰֵʱ��ready�ͷ����pid��ͬ

//���������λ��������ٶ�
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
		Shoot_Big_Motor_t motor_shoot_big[3];          			 //��������������
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

//�����������̨ʹ��ͬһ��can��id��Ҳ�����������̨������ִ��
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

#endif
