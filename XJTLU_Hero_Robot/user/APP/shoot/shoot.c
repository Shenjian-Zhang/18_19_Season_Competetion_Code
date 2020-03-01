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

#include "Shoot.h"
#include "CAN_Receive.h"
#include "gimbal_behaviour.h"
#include "Detect_Task.h"
#include "pid.h"
#include "laser.h"
#include "fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "CAN_receive.h"

#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //Ħ����1pwm�궨��
#define shoot_fric2_on(pwm) fric2_on((pwm)) //Ħ����2pwm�궨��
#define shoot_fric_off() fric_off()         //�ر�����Ħ����

#define shoot_big_fric1_on(pwm) fric1_big_on((pwm))	//��Ħ����1pwm�궨��
#define shoot_big_fric2_on(pwm) fric2_big_on((pwm)) //��Ħ����2pwm�궨��
#define shoot_big_fric_off() fric_big_off()					//�ر�������Ħ����

#define shoot_laser_on() laser_on()   //���⿪���궨��
#define shoot_laser_off() laser_off() //����رպ궨��

static const RC_ctrl_t *shoot_rc; //ң����ָ��

static PidTypeDef trigger_motor_pid;         //���PID
Shoot_Motor_t trigger_motor;          //�������

shoot_mode_e shoot_mode = SHOOT_STOP; //���״̬����ȥ����static��

Shoot_Big_Control_t shoot_big;

//΢������IO
//#define Butten_Trig_Pin GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_10)

//extern void getTriggerMotorMeasure(motor_measure_t *motor); δ���壬������CAN_receive.c�й�



/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void);
/**
  * @brief          ������ݸ���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void);
/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);
/**
  * @brief          �����ɿ��ƣ��ж�΢������һ��ʱ�����ӵ����ж�һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_done_control(void);
/**
  * @brief          ���׼�����ƣ����ӵ��͵�΢�����ش���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_ready_control(void);


/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @author         RM
  * @param[in]      void
  * @retval         ���ؿ�
  */
ramp_function_source_t big_ramp1, big_ramp2;
void shoot_init(void)
{
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 Shoot_Big_trigger_speed_pid[3] = {SHOOT_BIG_TRIGGER_SPEED_PID_KP, SHOOT_BIG_TRIGGER_SPEED_PID_KI, SHOOT_BIG_TRIGGER_SPEED_PID_KD};
		static const fp32 Limit_speed_pid[3] = {LIMIT_ANGLE_PID_KP, LIMIT_ANGLE_PID_KI, LIMIT_ANGLE_PID_KD};
		
		//ң����ָ��
    shoot_rc = get_remote_control_point();
    //���ָ��
    trigger_motor.shoot_motor_measure = get_Trigger_Motor_Measure_Point();
		shoot_big.motor_shoot_limit.shoot_big_limit_motor_measure = get_Limit_Motor_Measure_Point();

		shoot_big.motor_shoot_big[2].shoot_big_motor_measure = get_Shoot_Big_Motor_Measure_Point(2);

		//��ʼ��PID
    PID_Init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_Init(&shoot_big.motor_shoot_limit.motor_shoot_big_limit_speed_pid, PID_POSITION, Limit_speed_pid, LIMIT_READY_PID_MAX_OUT, LIMIT_READY_PID_MAX_IOUT);
		PID_Init(&shoot_big.motor_shoot_big[2].motor_shoot_big_speed_pid, PID_POSITION, Shoot_Big_trigger_speed_pid, SHOOT_BIG_TRIGGER_SPEED_PID_MAX_OUT, SHOOT_BIG_TRIGGER_SPEED_PID_MAX_IOUT);
		
		//��������
    Shoot_Feedback_Update();
    ramp_init(&trigger_motor.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_DOWN, Fric_OFF);
    ramp_init(&trigger_motor.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_DOWN, Fric_OFF);
		
    ramp_init(&big_ramp1, SHOOT_CONTROL_TIME * 0.001f, Fric_Big_DOWN, Fric_OFF);
    ramp_init(&big_ramp2, SHOOT_CONTROL_TIME * 0.001f, Fric_Big_DOWN, Fric_OFF);


    trigger_motor.ecd_count = 0;
    trigger_motor.angle = trigger_motor.shoot_motor_measure->ecd * Motor_ECD_TO_ANGLE;
    trigger_motor.given_current = 0;
    trigger_motor.move_flag = 0;
    trigger_motor.set_angle = trigger_motor.angle;
    trigger_motor.speed = 0.0f;
    trigger_motor.speed_set = 0.0f;
    trigger_motor.BulletShootCnt = 0;
}
/**
  * @brief          ���ѭ��
  * @author         RM
  * @param[in]      void
  * @retval         ����can����ֵ
  */
int16_t shoot_control_loop(void)
{
    int16_t shoot_CAN_Set_Current; //���ص�canֵ

    Shoot_Set_Mode();        //����״̬��
    Shoot_Feedback_Update(); //��������

    //����׼��״̬����
    if (shoot_mode == SHOOT_READY)
    {
        trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
        shoot_ready_control();	
    }

    if (shoot_mode == SHOOT_STOP)
    {
        trigger_motor.fric1_ramp.out = Fric_OFF;
        trigger_motor.fric2_ramp.out = Fric_OFF;
			
				big_ramp1.out = Fric_Big_OFF;
				big_ramp2.out = Fric_Big_OFF;
			
        shoot_fric_off();
        shoot_laser_off();
        shoot_CAN_Set_Current = 0;
				
				shoot_big_fric_off();
				shoot_big.motor_shoot_big[2].speed_set = SHOOT_BIG_TRIGGER_OFF;
			  shoot_big.motor_shoot_limit.speed_set = LIMIT_SPEED_OFF;
			
				shoot_big.motor_shoot_big[2].give_current = PID_Calc(&shoot_big.motor_shoot_big[2].motor_shoot_big_speed_pid, shoot_big.motor_shoot_big[2].shoot_big_motor_measure->speed_rpm, shoot_big.motor_shoot_big[2].speed_set);
				shoot_big.motor_shoot_limit.give_current = PID_Calc(&shoot_big.motor_shoot_limit.motor_shoot_big_limit_speed_pid, shoot_big.motor_shoot_limit.shoot_big_limit_motor_measure->speed_rpm, shoot_big.motor_shoot_limit.speed_set);
				CAN_CMD_SHOOT_BIG(0,0,0,0);
    }
    else
    {
        //Ħ����pwm
        static uint16_t fric_pwm1 = Fric_OFF;
        static uint16_t fric_pwm2 = Fric_OFF;
				//��Ħ����pwm
				static uint16_t fric_big_pwm1 = Fric_Big_OFF;
				static uint16_t fric_big_pwm2 = Fric_Big_OFF;


        shoot_laser_on();       //���⿪��


        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        ramp_calc(&trigger_motor.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

        if(trigger_motor.fric1_ramp.out == trigger_motor.fric1_ramp.max_value)
        {
            ramp_calc(&trigger_motor.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        }
				
        ramp_calc(&big_ramp1, SHOOT_FRIC_PWM_ADD_VALUE);

        if(big_ramp1.out == big_ramp1.max_value)
        {
            ramp_calc(&big_ramp2, SHOOT_FRIC_PWM_ADD_VALUE);
        }


//����Ҽ����¼���Ħ���֣�ʹ�������������� �Ҽ��������
//        static uint16_t up_time = 0;
//        if (trigger_motor.press_r)
//        {
//            up_time = UP_ADD_TIME;
//        }

//        if (up_time > 0)
//        {
//            trigger_motor.fric1_ramp.max_value = Fric_UP;
//            trigger_motor.fric2_ramp.max_value = Fric_UP;
//					
//						big_ramp1.max_value = Fric_Big_UP;
//						big_ramp2.max_value = Fric_Big_UP;
//					
//            up_time--;
//        }
//        else
//        {
            trigger_motor.fric1_ramp.max_value = Fric_DOWN;
            trigger_motor.fric2_ramp.max_value = Fric_DOWN;
					
						big_ramp1.max_value = Fric_Big_DOWN;
						big_ramp2.max_value = Fric_Big_DOWN;
//        }

        fric_pwm1 = (uint16_t)(trigger_motor.fric1_ramp.out);
        fric_pwm2 = (uint16_t)(trigger_motor.fric2_ramp.out);
				
        fric_big_pwm1 = (uint16_t)(big_ramp1.out);
        fric_big_pwm2 = (uint16_t)(big_ramp2.out);

        shoot_fric1_on(fric_pwm1);
        shoot_fric2_on(fric_pwm2);
				
				fric1_big_on(fric_big_pwm1);
				fric2_big_on(fric_big_pwm2);
				
				shoot_big.motor_shoot_big[2].give_current = PID_Calc(&shoot_big.motor_shoot_big[2].motor_shoot_big_speed_pid, shoot_big.motor_shoot_big[2].shoot_big_motor_measure->speed_rpm, shoot_big.motor_shoot_big[2].speed_set);
				shoot_big.motor_shoot_limit.give_current = PID_Calc(&shoot_big.motor_shoot_limit.motor_shoot_big_limit_speed_pid, shoot_big.motor_shoot_limit.shoot_big_limit_motor_measure->speed_rpm, shoot_big.motor_shoot_limit.speed_set);
				
				PID_Calc(&trigger_motor_pid, trigger_motor.speed, trigger_motor.speed_set);
				trigger_motor.given_current = (int16_t)(trigger_motor_pid.out);
				if(trigger_motor.speed_set == 0) trigger_motor.given_current = (int16_t)0;
		
				CAN_CMD_SHOOT_BIG(trigger_motor.given_current, 0, shoot_big.motor_shoot_big[2].give_current,shoot_big.motor_shoot_limit.give_current);
		}

    return shoot_CAN_Set_Current;
}

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //�ϲ��жϣ� һ�ο������ٴιر�
    if ((switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_up(last_s) && shoot_mode == SHOOT_STOP))
    {
        shoot_mode = SHOOT_READY;
    }
    else if ((switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_up(last_s) && shoot_mode != SHOOT_STOP))
    {
        shoot_mode = SHOOT_STOP;
    }

    //�����е��� ����ʹ�ü��̿���Ħ����
    if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && (shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_mode == SHOOT_STOP)
    {
        shoot_mode = SHOOT_READY;
    }
    //�����е��� ����ʹ�ü��̹ر�Ħ����
    else if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && (shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_mode == SHOOT_READY)
    {
        shoot_mode = SHOOT_STOP;
    }
		if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && switch_is_down(last_s))
    {
							trigger_motor.speed_set = 0;
							shoot_big.motor_shoot_limit.speed_set = LIMIT_SPEED_OFF;
							shoot_big.motor_shoot_big[2].speed_set = SHOOT_BIG_TRIGGER_OFF;
		}
		
    //�����̨״̬�� ����״̬���͹ر����
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_mode = SHOOT_STOP;
    }
    last_s = shoot_rc->rc.s[Shoot_RC_Channel];
}
/**
  * @brief          ������ݸ���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    trigger_motor.speed = speed_fliter_3;

    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
    if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd > Half_ecd_range)
    {
        trigger_motor.ecd_count--;
    }
    else if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd < -Half_ecd_range)
    {
        trigger_motor.ecd_count++;
    }

    if (trigger_motor.ecd_count == FULL_COUNT)
    {
        trigger_motor.ecd_count = -(FULL_COUNT - 1);
    }
    else if (trigger_motor.ecd_count == -FULL_COUNT)
    {
        trigger_motor.ecd_count = FULL_COUNT - 1;
    }

    //���������Ƕ�
    trigger_motor.angle = (trigger_motor.ecd_count * ecd_range + trigger_motor.shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE;
    //��갴��
    trigger_motor.last_press_l = trigger_motor.press_l;
    trigger_motor.last_press_r = trigger_motor.press_r;
    trigger_motor.press_l = shoot_rc->mouse.press_l;
    trigger_motor.press_r = shoot_rc->mouse.press_r;
    //������ʱ
    if (trigger_motor.press_l)
    {
        if (trigger_motor.press_l_time < PRESS_LONG_TIME)
        {
            trigger_motor.press_l_time++;
        }
    }
    else
    {
        trigger_motor.press_l_time = 0;
    }

    if (trigger_motor.press_r)
    {
        if (trigger_motor.press_r_time < PRESS_LONG_TIME)
        {
            trigger_motor.press_r_time++;
        }
    }
    else
    {
        trigger_motor.press_r_time = 0;
    }

    //��������µ�ʱ���ʱ
    if (shoot_mode != SHOOT_STOP && switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]))
    {

        if (trigger_motor.rc_s_time < RC_S_LONG_TIME)
        {
            trigger_motor.rc_s_time++;
        }
    }
    else
    {
        trigger_motor.rc_s_time = 0;
    }
}
/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
    //ÿ�β��� 1/4PI�ĽǶ�
    if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_BULLET)
    {
        trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
        trigger_motor.cmd_time = xTaskGetTickCount();
        trigger_motor.move_flag = 1;
    }

    //����Ƕ��ж�
    if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
    {
        //û����һֱ������ת�ٶ�
        trigger_motor.speed_set = TRIGGER_SPEED;
        trigger_motor.run_time = xTaskGetTickCount();

        //��ת�ж�
        if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME && trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
        {
            trigger_motor.speed_set = -TRIGGER_SPEED;
        }
        else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)
        {
            trigger_motor.cmd_time = xTaskGetTickCount();
        }
    }
    else
    {
        trigger_motor.move_flag = 0;
    }
}
/**
  * @brief          �����ɿ��ƣ��ж�΢������һ��ʱ�����ӵ����ж�һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_done_control(void)
{
		trigger_motor.speed_set = 0.0f;
		if(trigger_motor.key == SWITCH_TRIGGER_OFF)
		{
				shoot_mode = SHOOT_READY;
		}
		else
		{
				shoot_mode = SHOOT_BULLET;
		}
}
/**
  * @brief          ���׼�����ƣ����ӵ��͵�΢�����ش���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */

int16_t small_shoot_mode = 0;
static void shoot_ready_control(void)
{
		//��С���跢���������
		if(switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]))
		{
				if (((shoot_rc->key.v & SHOOT_BIG_ON_KEYBOARD) || trigger_motor.press_r))
				{
						small_shoot_mode = 0;
					
						shoot_big.motor_shoot_limit.speed_set = LIMIT_SPEED;						//modify to ׼��״̬��Ħ���֣����״̬ʱ�ٴ򿪴�����λ�Ͳ��֣�С���貦��
						shoot_big.motor_shoot_big[2].speed_set = SHOOT_BIG_TRIGGER_ON;
					
						trigger_motor.speed_set = 0;
				}
				else if (((shoot_rc->key.v & SHOOT_SMALL_ON_KEYBOARD) || trigger_motor.press_l))
				{
						small_shoot_mode = 1;
					
						shoot_big.motor_shoot_limit.speed_set = LIMIT_SPEED_OFF;
						shoot_big.motor_shoot_big[2].speed_set = SHOOT_BIG_TRIGGER_OFF;					
				}
				else
				{
						small_shoot_mode = 0;
					
						shoot_big.motor_shoot_limit.speed_set = LIMIT_SPEED_OFF;
						shoot_big.motor_shoot_big[2].speed_set = SHOOT_BIG_TRIGGER_OFF;
						trigger_motor.speed_set = 0;
				}
		}
		
		if(small_shoot_mode == 1)
		{
				//ÿ�β��� 1/4PI�ĽǶ�
				if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_READY)
				{
						trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
						trigger_motor.cmd_time = xTaskGetTickCount();
						trigger_motor.move_flag = 1;
				}
				
				//����Ƕ��ж�
				if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
				{
						//�Ƕȴﵽ�ж�
						trigger_motor.speed_set = TRIGGER_SPEED;
						trigger_motor.run_time = xTaskGetTickCount();
						//��ת�ж�
						if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME && trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
						{
								trigger_motor.speed_set = -TRIGGER_SPEED;
						}
						else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)
						{
								trigger_motor.cmd_time = xTaskGetTickCount();
						}
				}
				else
				{
						trigger_motor.move_flag = 0;
				}
		}
			
			
//		�²�һ�λ�����갴��һ�Σ��������״̬
//    if ((trigger_motor.press_l && trigger_motor.last_press_l == 0) || (trigger_motor.press_r && trigger_motor.last_press_r == 0))
//    {
//        
//    }

		
		
		//���������£�С���跢��
		//if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && shoot_rc->mouse.press_l )
//		if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && trigger_motor.press_l)
//		{
//			
//				//ÿ�β��� 1/4PI�ĽǶ�
//				if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_READY)
//				{
//						trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
//						trigger_motor.cmd_time = xTaskGetTickCount();
//						trigger_motor.move_flag = 1;
//				}

//				//����Ƕ��ж�
//				if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
//				{
//					//�Ƕȴﵽ�ж�
//					trigger_motor.speed_set = TRIGGER_SPEED;
//					trigger_motor.run_time = xTaskGetTickCount();
//					//��ת�ж�
//					if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME && trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
//					{
//							trigger_motor.speed_set = -TRIGGER_SPEED;
//					}
//					else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)
//					{
//							trigger_motor.cmd_time = xTaskGetTickCount();
//					}
//				}
//				else
//				{
//						trigger_motor.move_flag = 0;
//				}
//							
//				shoot_big.motor_shoot_limit.speed_set = LIMIT_SPEED_OFF;
//				shoot_big.motor_shoot_big[2].speed_set = SHOOT_BIG_TRIGGER_OFF;
//		}
		
		
		
		//����Ҽ����£����跢��
		//if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && shoot_rc->mouse.press_r )
//		if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && trigger_motor.press_r)
//		{
//							shoot_big.motor_shoot_limit.speed_set = LIMIT_SPEED;
//							shoot_big.motor_shoot_big[2].speed_set = SHOOT_BIG_TRIGGER_ON;
//							trigger_motor.speed_set = 0;
//		}
		
		
		
		
		//ң������Readyģʽ�²����µ�����С����ͬʱ����
    //if (switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_down(last_s) && shoot_mode == SHOOT_READY)
    if (switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]))
    {
				shoot_big.motor_shoot_limit.speed_set = LIMIT_SPEED;
				shoot_big.motor_shoot_big[2].speed_set = SHOOT_BIG_TRIGGER_ON;
			
				//ÿ�β��� 1/4PI�ĽǶ�
				if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_READY)
				{
						trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
						trigger_motor.cmd_time = xTaskGetTickCount();
						trigger_motor.move_flag = 1;
				}

				//����Ƕ��ж�
				if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
				{
					//�Ƕȴﵽ�ж�
					trigger_motor.speed_set = TRIGGER_SPEED;
					trigger_motor.run_time = xTaskGetTickCount();
					//��ת�ж�
					if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME && trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
					{
							trigger_motor.speed_set = -TRIGGER_SPEED;
					}
					else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)
					{
							trigger_motor.cmd_time = xTaskGetTickCount();
					}
				}
				else
				{
						trigger_motor.move_flag = 0;
				}

		}
		
//		//ÿ�β��� 1/4PI�ĽǶ�
//		if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_READY)
//		{
//				trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
//				trigger_motor.cmd_time = xTaskGetTickCount();
//				trigger_motor.move_flag = 1;
//		}

//		//����Ƕ��ж�
//		if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
//		{
//			//�Ƕȴﵽ�ж�
//			trigger_motor.speed_set = TRIGGER_SPEED;
//			trigger_motor.run_time = xTaskGetTickCount();
//			//��ת�ж�
//			if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME && trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
//			{
//					trigger_motor.speed_set = -TRIGGER_SPEED;
//			}
//			else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)
//			{
//					trigger_motor.cmd_time = xTaskGetTickCount();
//			}
//		}
//		else
//		{
//				trigger_motor.move_flag = 0;
//		}
}
