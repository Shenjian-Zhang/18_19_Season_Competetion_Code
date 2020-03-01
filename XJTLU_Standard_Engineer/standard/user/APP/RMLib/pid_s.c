/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       pid.c
	* @brief      pid parameter initialization, position and delta pid calculate
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017   Richard.luo        
  * @verbatim
	********************************(C) COPYRIGHT 2017 DJI************************
	*/
	
#include "pid_s.h"

#define ABS(x) ((x > 0) ? x : -x)
void abs_limit_s(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

static void pid_param_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float    kp,
    float    ki,
    float    kd)
{

  pid->integral_limit = intergral_limit;
  pid->max_out     = maxout;
  pid->pid_mode      = mode;

  pid->p = kp;
  pid->i = ki;
  pid->d = kd;

}
/**
  * @brief     modify pid parameter when code running
  * @param[in] pid: control pid struct
  * @param[in] p/i/d: pid parameter
  * @retval    none
  */
static void pid_reset(pid_t *pid, float kp, float ki, float kd)
{
  pid->p = kp;
  pid->i = ki;
  pid->d = kd;
  
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out  = 0;
  
}

/**
  * @brief     calculate delta PID and position PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output 
  */
float pid_calc(pid_t *pid, float get, float set)
{
	pid->get = get;
	pid->set = set;
	pid->err[NOW] = set - get;
	
	if ((pid->input_max_err != 0) && (ABS(pid->err[NOW]) > pid->input_max_err))
			return 0;

	if (pid->pid_mode == POSITION_PID) //position PID
	{
			pid->pout = pid->p * pid->err[NOW];
			pid->iout += pid->i * pid->err[NOW];
			pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
		
			abs_limit_s(&(pid->iout), pid->integral_limit);
			pid->out = pid->pout + pid->iout + pid->dout;
			abs_limit_s(&(pid->out), pid->max_out);
	}
	else if (pid->pid_mode == DELTA_PID) //delta PID
	{
			pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
			pid->iout = pid->i * pid->err[NOW];
			pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

			pid->out += pid->pout + pid->iout + pid->dout;
			abs_limit_s(&(pid->out), pid->max_out);
	}

	pid->err[LLAST] = pid->err[LAST];
	pid->err[LAST]  = pid->err[NOW];
  
  
  if ((pid->output_deadband != 0) && (ABS(pid->out) < pid->output_deadband))
    return 0;
  else
    return pid->out;

}


float pid_calc_grade(grade_pid_t *pid, float get, float set)
{
	pid->get = get;
	pid->set = set;
	pid->err[NOW] = set - get; 
	
	if ((pid->input_max_err != 0) && (ABS(pid->err[NOW]) > pid->input_max_err))
			return 0;

	if(ABS(pid->err[NOW]) > pid->grade_range)
		pid->p = pid->p_far;
	else
		pid->p = pid->p_near;
	
	pid->pout = pid->p * pid->err[NOW];
	pid->iout += pid->i * pid->err[NOW];
	pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
		
	abs_limit_s(&(pid->iout), pid->integral_limit);
	pid->out = pid->pout + pid->iout + pid->dout;
	abs_limit_s(&(pid->out), pid->max_out);

	pid->err[LLAST] = pid->err[LAST];
	pid->err[LAST]  = pid->err[NOW];

	if ((pid->output_deadband != 0) && (ABS(pid->out) < pid->output_deadband))
		return 0;
	else
		return pid->out;
}
/**
  * @brief     initialize pid parameter
  * @retval    none
  */
void PID_struct_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd)
{
  pid->f_param_init = pid_param_init;
  pid->f_pid_reset  = pid_reset;

  pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
  pid->f_pid_reset(pid, kp, ki, kd);
  
}

//pid_t pid_yaw           = PID_PARAM_DEFAULT;
//pid_t pid_pit           = PID_PARAM_DEFAULT;
//pid_t pid_yaw_speed     = PID_PARAM_DEFAULT; //yaw palstance loop
//pid_t pid_pit_speed     = PID_PARAM_DEFAULT; //pitch palstance loop
pid_t pid_chassis_speed[4]        = { 0 };
pid_t pid_3508_speed		= { 0 };
pid_t pid_3508					=	{ 0 };
//pid_t pid_lift_l				= { 0 };
//pid_t pid_lift_r				= { 0 };
//pid_t pid_chassis_angle = { 0 };
//pid_t pid_golf					=	{ 0 };
//pid_t pid_golf_speed		= { 0 };

//pid_t pid_fric_l					= { 0 };
//pid_t pid_fric_r					= { 0 };
//pid_t pid_trigger       	= { 0 };
//pid_t pid_trigger_speed 	= { 0 };
//pid_t pid_imu_tmp       = { 0 };
//pid_t pid_rfid_arm      = { 0 };
//pid_t pid_test_speed    = { 0 };


