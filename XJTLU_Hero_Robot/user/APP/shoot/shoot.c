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

#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#define shoot_fric_off() fric_off()         //关闭两个摩擦轮

#define shoot_big_fric1_on(pwm) fric1_big_on((pwm))	//大摩擦轮1pwm宏定义
#define shoot_big_fric2_on(pwm) fric2_big_on((pwm)) //大摩擦轮2pwm宏定义
#define shoot_big_fric_off() fric_big_off()					//关闭两个大摩擦轮

#define shoot_laser_on() laser_on()   //激光开启宏定义
#define shoot_laser_off() laser_off() //激光关闭宏定义

static const RC_ctrl_t *shoot_rc; //遥控器指针

static PidTypeDef trigger_motor_pid;         //电机PID
Shoot_Motor_t trigger_motor;          //射击数据

shoot_mode_e shoot_mode = SHOOT_STOP; //射击状态机（去掉了static）

Shoot_Big_Control_t shoot_big;

//微动开关IO
//#define Butten_Trig_Pin GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_10)

//extern void getTriggerMotorMeasure(motor_measure_t *motor); 未定义，可能与CAN_receive.c有关



/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void);
/**
  * @brief          射击数据更新
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void);
/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);
/**
  * @brief          射击完成控制，判断微动开关一段时间无子弹来判断一次发射
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_done_control(void);
/**
  * @brief          射击准备控制，将子弹送到微动开关处，
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_ready_control(void);


/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @author         RM
  * @param[in]      void
  * @retval         返回空
  */
ramp_function_source_t big_ramp1, big_ramp2;
void shoot_init(void)
{
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 Shoot_Big_trigger_speed_pid[3] = {SHOOT_BIG_TRIGGER_SPEED_PID_KP, SHOOT_BIG_TRIGGER_SPEED_PID_KI, SHOOT_BIG_TRIGGER_SPEED_PID_KD};
		static const fp32 Limit_speed_pid[3] = {LIMIT_ANGLE_PID_KP, LIMIT_ANGLE_PID_KI, LIMIT_ANGLE_PID_KD};
		
		//遥控器指针
    shoot_rc = get_remote_control_point();
    //电机指针
    trigger_motor.shoot_motor_measure = get_Trigger_Motor_Measure_Point();
		shoot_big.motor_shoot_limit.shoot_big_limit_motor_measure = get_Limit_Motor_Measure_Point();

		shoot_big.motor_shoot_big[2].shoot_big_motor_measure = get_Shoot_Big_Motor_Measure_Point(2);

		//初始化PID
    PID_Init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_Init(&shoot_big.motor_shoot_limit.motor_shoot_big_limit_speed_pid, PID_POSITION, Limit_speed_pid, LIMIT_READY_PID_MAX_OUT, LIMIT_READY_PID_MAX_IOUT);
		PID_Init(&shoot_big.motor_shoot_big[2].motor_shoot_big_speed_pid, PID_POSITION, Shoot_Big_trigger_speed_pid, SHOOT_BIG_TRIGGER_SPEED_PID_MAX_OUT, SHOOT_BIG_TRIGGER_SPEED_PID_MAX_IOUT);
		
		//更新数据
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
  * @brief          射击循环
  * @author         RM
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{
    int16_t shoot_CAN_Set_Current; //返回的can值

    Shoot_Set_Mode();        //设置状态机
    Shoot_Feedback_Update(); //更新数据

    //发射准备状态控制
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
        //摩擦轮pwm
        static uint16_t fric_pwm1 = Fric_OFF;
        static uint16_t fric_pwm2 = Fric_OFF;
				//大摩擦轮pwm
				static uint16_t fric_big_pwm1 = Fric_Big_OFF;
				static uint16_t fric_big_pwm2 = Fric_Big_OFF;


        shoot_laser_on();       //激光开启


        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
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


//鼠标右键按下加速摩擦轮，使得左键低速射击， 右键高速射击
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
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_up(last_s) && shoot_mode == SHOOT_STOP))
    {
        shoot_mode = SHOOT_READY;
    }
    else if ((switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_up(last_s) && shoot_mode != SHOOT_STOP))
    {
        shoot_mode = SHOOT_STOP;
    }

    //处于中档， 可以使用键盘开启摩擦轮
    if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && (shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_mode == SHOOT_STOP)
    {
        shoot_mode = SHOOT_READY;
    }
    //处于中档， 可以使用键盘关闭摩擦轮
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
		
    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_mode = SHOOT_STOP;
    }
    last_s = shoot_rc->rc.s[Shoot_RC_Channel];
}
/**
  * @brief          射击数据更新
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    trigger_motor.speed = speed_fliter_3;

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
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

    //计算输出轴角度
    trigger_motor.angle = (trigger_motor.ecd_count * ecd_range + trigger_motor.shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE;
    //鼠标按键
    trigger_motor.last_press_l = trigger_motor.press_l;
    trigger_motor.last_press_r = trigger_motor.press_r;
    trigger_motor.press_l = shoot_rc->mouse.press_l;
    trigger_motor.press_r = shoot_rc->mouse.press_r;
    //长按计时
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

    //射击开关下档时间计时
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
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
    //每次拨动 1/4PI的角度
    if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_BULLET)
    {
        trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
        trigger_motor.cmd_time = xTaskGetTickCount();
        trigger_motor.move_flag = 1;
    }

    //到达角度判断
    if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
    {
        //没到达一直设置旋转速度
        trigger_motor.speed_set = TRIGGER_SPEED;
        trigger_motor.run_time = xTaskGetTickCount();

        //堵转判断
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
  * @brief          射击完成控制，判断微动开关一段时间无子弹来判断一次发射
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
  * @brief          射击准备控制，将子弹送到微动开关处，
  * @author         RM
  * @param[in]      void
  * @retval         void
  */

int16_t small_shoot_mode = 0;
static void shoot_ready_control(void)
{
		//大小弹丸发射机构控制
		if(switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]))
		{
				if (((shoot_rc->key.v & SHOOT_BIG_ON_KEYBOARD) || trigger_motor.press_r))
				{
						small_shoot_mode = 0;
					
						shoot_big.motor_shoot_limit.speed_set = LIMIT_SPEED;						//modify to 准备状态打开摩擦轮，射击状态时再打开大弹丸限位和拨轮；小弹丸拨轮
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
				//每次拨动 1/4PI的角度
				if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_READY)
				{
						trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
						trigger_motor.cmd_time = xTaskGetTickCount();
						trigger_motor.move_flag = 1;
				}
				
				//到达角度判断
				if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
				{
						//角度达到判断
						trigger_motor.speed_set = TRIGGER_SPEED;
						trigger_motor.run_time = xTaskGetTickCount();
						//堵转判断
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
			
			
//		下拨一次或者鼠标按下一次，进入射击状态
//    if ((trigger_motor.press_l && trigger_motor.last_press_l == 0) || (trigger_motor.press_r && trigger_motor.last_press_r == 0))
//    {
//        
//    }

		
		
		//鼠标左键按下，小弹丸发射
		//if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && shoot_rc->mouse.press_l )
//		if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && trigger_motor.press_l)
//		{
//			
//				//每次拨动 1/4PI的角度
//				if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_READY)
//				{
//						trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
//						trigger_motor.cmd_time = xTaskGetTickCount();
//						trigger_motor.move_flag = 1;
//				}

//				//到达角度判断
//				if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
//				{
//					//角度达到判断
//					trigger_motor.speed_set = TRIGGER_SPEED;
//					trigger_motor.run_time = xTaskGetTickCount();
//					//堵转判断
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
		
		
		
		//鼠标右键按下，大弹丸发射
		//if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && shoot_rc->mouse.press_r )
//		if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && trigger_motor.press_r)
//		{
//							shoot_big.motor_shoot_limit.speed_set = LIMIT_SPEED;
//							shoot_big.motor_shoot_big[2].speed_set = SHOOT_BIG_TRIGGER_ON;
//							trigger_motor.speed_set = 0;
//		}
		
		
		
		
		//遥控器在Ready模式下拨到下档，大小弹丸同时发射
    //if (switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_down(last_s) && shoot_mode == SHOOT_READY)
    if (switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]))
    {
				shoot_big.motor_shoot_limit.speed_set = LIMIT_SPEED;
				shoot_big.motor_shoot_big[2].speed_set = SHOOT_BIG_TRIGGER_ON;
			
				//每次拨动 1/4PI的角度
				if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_READY)
				{
						trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
						trigger_motor.cmd_time = xTaskGetTickCount();
						trigger_motor.move_flag = 1;
				}

				//到达角度判断
				if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
				{
					//角度达到判断
					trigger_motor.speed_set = TRIGGER_SPEED;
					trigger_motor.run_time = xTaskGetTickCount();
					//堵转判断
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
		
//		//每次拨动 1/4PI的角度
//		if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_READY)
//		{
//				trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
//				trigger_motor.cmd_time = xTaskGetTickCount();
//				trigger_motor.move_flag = 1;
//		}

//		//到达角度判断
//		if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
//		{
//			//角度达到判断
//			trigger_motor.speed_set = TRIGGER_SPEED;
//			trigger_motor.run_time = xTaskGetTickCount();
//			//堵转判断
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
