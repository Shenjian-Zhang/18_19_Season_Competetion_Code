/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       INSTask.c/h
  * @brief      ��Ҫ����������mpu6500��������ist8310�������̬���㣬�ó�ŷ���ǣ�
  *             �ṩͨ��mpu6500��data ready �ж�����ⲿ�������������ݵȴ��ӳ�
  *             ͨ��DMA��SPI�����ԼCPUʱ�䣬�ṩע�Ͷ�Ӧ�ĺ궨�壬�ر�DMA��
  *             DR���ⲿ�жϵķ�ʽ.
  * @note       SPI �������ǳ�ʼ����ʱ����Ҫ����2MHz��֮���ȡ���������20MHz
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

#include "main.h"
#include "INS_Task.h"

#include "stm32f4xx.h"

#include "buzzer.h"
#include "timer.h"
#include "spi.h"
#include "exit_init.h"
#include "IST8310driver.h"
#include "mpu6500driver.h"
#include "mpu6500reg.h"
#include "mpu6500driver_middleware.h"

#include "AHRS.h"

#include "calibrate_Task.h"
#include "pid.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "math.h"

#define IMUWarnBuzzerOn() buzzer_on(95, 10000) //����������У׼������

#define IMUWarnBuzzerOFF() buzzer_off() //����������У׼�������ر�

#define MPU6500_TEMPERATURE_PWM_INIT() TIM3_Init(MPU6500_TEMP_PWM_MAX, 1) //�������¶ȿ���PWM��ʼ��
#define IMUTempPWM(pwm) TIM_SetCompare2(TIM3, (pwm))                      //pwm����
#define INS_GET_CONTROL_TEMPERATURE() get_control_temperate()             //��ȡ�����¶ȵ�Ŀ��ֵ

#if defined(MPU6500_USE_DATA_READY_EXIT)

#define MPU6500_DATA_READY_EXIT_INIT() GPIOB_Exti8_GPIO_Init() //��ʼ��mpu6500�� �ⲿ�ж� ʹ��PB8 �ⲿ�ж��� 8

#define MPU6500_DATA_READY_EXIT_IRQHandler EXTI9_5_IRQHandler //�궨���ⲿ�жϺ�����ʹ����line8�ⲿ�ж�

#define MPU6500_DATA_READY_EXIT_Line EXTI_Line8 //�궨���ⲿ�ж���
#endif

#if defined(MPU6500_USE_SPI) && defined(MPU6500_USE_SPI_DMA)

//�궨���ʼ��SPI��DMA��ͬʱ����SPIΪ8λ��4��Ƶ
#define MPU6500_SPI_DMA_Init(txbuf, rxbuf)                                 \
    {                                                                      \
        SPI5_DMA_Init((uint32_t)txbuf, (uint32_t)rxbuf, DMA_RX_NUM);       \
        SPI_I2S_DMACmd(SPI5, SPI_I2S_DMAReq_Rx, ENABLE);                   \
        SPI_I2S_DMACmd(SPI5, SPI_I2S_DMAReq_Tx, ENABLE);                   \
        SPI5SetSpeedAndDataSize(SPI_BaudRatePrescaler_8, SPI_DataSize_8b); \
    }

#define MPU6500_SPI_DMA_Enable() SPI5_DMA_Enable(DMA_RX_NUM) // ��ʼһ��SPI��DMA����
//�궨��SPI��DMA�����жϺ����Լ������жϱ�־λ
#define MPU6500_DMA_IRQHandler DMA2_Stream5_IRQHandler
#define MPU6500_DMA_Stream DMA2_Stream5
#define MPU6500_DMA_FLAG DMA_FLAG_TCIF5
#elif defined(MPU6500_USE_SPI_DMA)
#error "the communication of mpu6500 is not SPI, can't use the DMA"
#endif

//���ʹ��mpu6500������׼���ⲿ�жϣ�����ʹ������֪ͨ������������
#if defined(MPU6500_USE_DATA_READY_EXIT) || defined(MPU6500_USE_SPI_DMA)
static TaskHandle_t INSTask_Local_Handler;
#endif

//DMA��SPI ���͵�buf����INT_STATUS��ʼ������ȡ DMA_RX_NUM��С��ַ��ֵ
#if defined(MPU6500_USE_SPI_DMA)
static const uint8_t mpu6500_spi_DMA_txbuf[DMA_RX_NUM] =
    {
        MPU_INT_STATUS | MPU_SPI_READ_MSB};
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t INSTaskStack;
#endif

#define IMU_BOARD_INSTALL_SPIN_MATRIX                           \
                                        { 0.0f, 1.0f, 0.0f},    \
                                        {-1.0f, 0.0f, 0.0f},    \
                                        { 0.0f, 0.0f, 1.0f}    \
																				
#define Kp_q 5.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki_q 0.01f   // integral gain governs rate of convergence of gyroscope biases
																				
//���������ǣ����ٶȼƣ����������ݵ����Զȣ���Ư
static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310);
static void IMU_temp_Control(fp32 temp);

static uint8_t mpu6500_spi_rxbuf[DMA_RX_NUM]; //������յ�ԭʼ����
static mpu6500_real_data_t mpu6500_real_data; //ת���ɹ��ʵ�λ��MPU6500����
static fp32 Gyro_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //������У׼���Զ�
static fp32 gyro_cali_offset[3] ={0.0f, 0.0f, 0.0f};
static fp32 Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};            //��������Ư
static fp32 Accel_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //���ٶ�У׼���Զ�
static fp32 Accel_Offset[3] = {0.0f, 0.0f, 0.0f};            //���ٶ���Ư
static ist8310_real_data_t ist8310_real_data;                //ת���ɹ��ʵ�λ��IST8310����
static fp32 Mag_Scale_Factor[3][3] = {{1.0f, 0.0f, 0.0f},
                                      {0.0f, 1.0f, 0.0f},
                                      {0.0f, 0.0f, 1.0f}}; //������У׼���Զ�
static fp32 Mag_Offset[3] = {0.0f, 0.0f, 0.0f};            //��������Ư
static const float TimingTime = INS_DELTA_TICK * 0.001f;   //�������е�ʱ�� ��λ s

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};

fp32 INS_Angle[3] = {0.0f, 0.0f, 0.0f};      //ŷ���� ��λ rad
static volatile fp32 INS_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; //��Ԫ��

static const fp32 imuTempPID[3] = {MPU6500_TEMPERATURE_PID_KP, MPU6500_TEMPERATURE_PID_KI, MPU6500_TEMPERATURE_PID_KD};
static PidTypeDef imuTempPid;

static uint8_t first_temperate = 0;

volatile float exInt, eyInt, ezInt;  // ������
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;
volatile uint32_t lastUpdate, now; // �������ڼ��� ��λ us

void INSTask(void *pvParameters)
{

    vTaskDelay(INS_TASK_INIT_TIME);
    //��ʼ��mpu6500��ʧ�ܽ�����ѭ��
    while (mpu6500_init() != MPU6500_NO_ERROR)
    {
        ;
    }

//��ʼ��ist8310��ʧ�ܽ�����ѭ��
#if defined(USE_IST8310)
    while (ist8310_init() != IST8310_NO_ERROR)
    {
        ;
    }
#endif

#if defined(MPU6500_USE_DATA_READY_EXIT) || defined(MPU6500_USE_SPI_DMA)
    //��ȡ��ǰ���������������������֪ͨ
    INSTask_Local_Handler = xTaskGetHandle(pcTaskGetName(NULL));
#endif

#if defined(MPU6500_USE_DATA_READY_EXIT)
    //��ʼ��mpu6500������׼�����ⲿ�ж�
    MPU6500_DATA_READY_EXIT_INIT();
#else

    //�����ʹ���ⲿ�жϻ�������ķ�������ʹ�ô�ͳ�������л��ķ���
    TickType_t INS_LastWakeTime;
    INS_LastWakeTime = xTaskGetTickCount();

#endif

//��ʼ��SPI��DMA����ķ���
#if defined(MPU6500_USE_SPI_DMA) && defined(MPU6500_USE_SPI)
    MPU6500_SPI_DMA_Init(mpu6500_spi_DMA_txbuf, mpu6500_spi_rxbuf);

#endif

    while (1)
    {

#if defined(MPU6500_USE_DATA_READY_EXIT)
        //�ȴ��ⲿ�ж��жϻ�������
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }
#else
        //������ʱ�л�����
        vTaskDelayUntil(&INS_LastWakeTime, INS_DELTA_TICK);
//����ʱ�����л������������DMA����
#ifdef MPU6500_USE_SPI_DMA

        MPU6500_SPI_NS_L();
        MPU6500_SPI_DMA_Enable();
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }
#endif

#endif

//�����ʹ��SPI�ķ�������ʹ����ͨSPIͨ�ŵķ���
#ifndef MPU6500_USE_SPI_DMA
        mpu6500_read_muli_reg(MPU_INT_STATUS, mpu6500_spi_rxbuf, DMA_RX_NUM);
#endif

        //����ȡ����mpu6500ԭʼ���ݴ���ɹ��ʵ�λ������
        mpu6500_read_over((mpu6500_spi_rxbuf + MPU6500_RX_BUF_DATA_OFFSET), &mpu6500_real_data);

//����ȡ����ist8310ԭʼ���ݴ���ɹ��ʵ�λ������
#if defined(USE_IST8310)
        ist8310_read_over((mpu6500_spi_rxbuf + IST8310_RX_BUF_DATA_OFFSET), &ist8310_real_data);
#endif
        //��ȥ��Ư�Լ���ת����ϵ
        //IMU_Cali_Slove(INS_gyro, INS_accel, INS_mag, &mpu6500_real_data, &ist8310_real_data);
        //�ж��Ƿ��һ�ν��룬�����һ�����ʼ����Ԫ����֮�������Ԫ������Ƕȵ�λrad
				IMU_AHRSupdate();
				INS_Angle[0] = -atan2(2 * INS_quat[1] * INS_quat[2] + 2 * INS_quat[0] * INS_quat[3], -2 * INS_quat[2]*INS_quat[2] - 2 * INS_quat[3] * INS_quat[3] + 1)* 180/PI; // yaw        -pi----pi
				INS_Angle[1] = -asin(-2 * INS_quat[1] * INS_quat[3] + 2 * INS_quat[0] * INS_quat[2])* 180/PI; // pitch    -pi/2    --- pi/2 
				INS_Angle[2] = atan2(2 * INS_quat[2] * INS_quat[3] + 2 * INS_quat[0] * INS_quat[1], -2 * INS_quat[1] * INS_quat[1] - 2 * INS_quat[2] * INS_quat[2] + 1)* 180/PI; // roll       -pi-----pi  
				
				
//        static uint8_t updata_count = 0;

//        if( mpu6500_real_data.status & 1 << MPU_DATA_READY_BIT)
//        {
//            if (updata_count == 0)
//            {
//                MPU6500_TEMPERATURE_PWM_INIT();
//                PID_Init(&imuTempPid, PID_DELTA, imuTempPID, MPU6500_TEMPERATURE_PID_MAX_OUT, MPU6500_TEMPERATURE_PID_MAX_IOUT);

//                updata_count++;
//            }
//            else
//            {
//                //�����ǿ���У׼
//                {
//                    static uint16_t start_gyro_cali_time = 0;
//                    if(start_gyro_cali_time == 0)
//                    {
//                        Gyro_Offset[0] = gyro_cali_offset[0];
//                        Gyro_Offset[1] = gyro_cali_offset[1];
//                        Gyro_Offset[2] = gyro_cali_offset[2];
//                        start_gyro_cali_time++;
//                    }
//                    else if (start_gyro_cali_time < GYRO_OFFSET_START_TIME)
//                    {
//                        //IMUWarnBuzzerOn();
//                        if( first_temperate)
//                        {
//                            //������gyro_offset������������˶�start_gyro_cali_time++��������˶� start_gyro_cali_time = 0
//                            gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, &start_gyro_cali_time);
//                        }
//                    }
//                    else if (start_gyro_cali_time == GYRO_OFFSET_START_TIME)
//                    {

//                        IMUWarnBuzzerOFF();
//                        start_gyro_cali_time++;
//                    }
//                }       //�����ǿ���У׼   code end

//            }           //update count if   code end
//        }               //mpu6500 status  if end
        //����������������¶ȿ��ƴ���

        //IMU_temp_Control(mpu6500_real_data.temp);

#if INCLUDE_uxTaskGetStackHighWaterMark
        INSTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif

        //while(1) end
    }
    //task function end
}

/**
  * @brief          У׼������
  * @author         RM
  * @param[in]      �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[in]      �����ǵ���Ư���ɼ������ǵľ�ֹ�������Ϊoffset
  * @param[in]      �����ǵ�ʱ�̣�ÿ����gyro_offset���û��1,
  * @retval         ���ؿ�
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
    if (first_temperate)
    {
        if( *time_count == 0)
        {
            Gyro_Offset[0] = gyro_cali_offset[0];
            Gyro_Offset[1] = gyro_cali_offset[1];
            Gyro_Offset[2] = gyro_cali_offset[2];
        }
        gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, time_count);

        cali_offset[0] = Gyro_Offset[0];
        cali_offset[1] = Gyro_Offset[1];
        cali_offset[2] = Gyro_Offset[2];
    }
}

/**
  * @brief          У׼���������ã�����flash���������ط�����У׼ֵ
  * @author         RM
  * @param[in]      �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[in]      �����ǵ���Ư
  * @retval         ���ؿ�
  */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
}

const fp32 *get_INS_angle_point(void)
{
    return INS_Angle;
}
const fp32 *get_MPU6500_Gyro_Data_Point(void)
{
    return INS_gyro;
}

const fp32 *get_MPU6500_Accel_Data_Point(void)
{
    return INS_accel;
}

static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = mpu6500->gyro[0] * Gyro_Scale_Factor[i][0] + mpu6500->gyro[1] * Gyro_Scale_Factor[i][1] + mpu6500->gyro[2] * Gyro_Scale_Factor[i][2] + Gyro_Offset[i];
        accel[i] = mpu6500->accel[0] * Accel_Scale_Factor[i][0] + mpu6500->accel[1] * Accel_Scale_Factor[i][1] + mpu6500->accel[2] * Accel_Scale_Factor[i][2] + Accel_Offset[i];
        mag[i] = ist8310->mag[0] * Mag_Scale_Factor[i][0] + ist8310->mag[1] * Mag_Scale_Factor[i][1] + ist8310->mag[2] * Mag_Scale_Factor[i][2] + Mag_Offset[i];
    }
}
static void IMU_temp_Control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0 ;
    if (first_temperate)
    {
        PID_Calc(&imuTempPid, temp, INS_GET_CONTROL_TEMPERATURE());
        if (imuTempPid.out < 0.0f)
        {
            imuTempPid.out = 0.0f;
        }
        tempPWM = (uint16_t)imuTempPid.out;
        IMUTempPWM(tempPWM);
    }
    else
    {
        //��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
        if (temp > INS_GET_CONTROL_TEMPERATURE())
        {
            temp_constant_time ++;
            if(temp_constant_time > 200)
            {
                //�ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
                first_temperate = 1;
                imuTempPid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;

            }
        }

        IMUTempPWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

#if defined(MPU6500_USE_DATA_READY_EXIT)

void MPU6500_DATA_READY_EXIT_IRQHandler(void)
{
    if (EXTI_GetITStatus(MPU6500_DATA_READY_EXIT_Line) != RESET)
    {

        EXTI_ClearITPendingBit(MPU6500_DATA_READY_EXIT_Line);

//�������DMA���� ����������DMA�ж����
#if defined(MPU6500_USE_SPI_DMA)
        mpu6500_SPI_NS_L();
        MPU6500_SPI_DMA_Enable();
#else

        //��������
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR((INSTask_Local_Handler), &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

#endif
    }
}

#endif

#if defined(MPU6500_USE_SPI) && defined(MPU6500_USE_SPI_DMA)

void MPU6500_DMA_IRQHandler(void)
{
    if (DMA_GetFlagStatus(MPU6500_DMA_Stream, MPU6500_DMA_FLAG))
    {
        DMA_ClearFlag(MPU6500_DMA_Stream, MPU6500_DMA_FLAG);
        mpu6500_SPI_NS_H();

        //��������
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INSTask_Local_Handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

//������Ԫ��
void IMU_AHRSupdate(void) {
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez,halfT;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = INS_quat[0]*INS_quat[0];
    float q0q1 = INS_quat[0]*INS_quat[1];
    float q0q2 = INS_quat[0]*INS_quat[2];
    float q0q3 = INS_quat[0]*INS_quat[3];
    float q1q1 = INS_quat[1]*INS_quat[1];
    float q1q2 = INS_quat[1]*INS_quat[2];
    float q1q3 = INS_quat[1]*INS_quat[3];
    float q2q2 = INS_quat[2]*INS_quat[2];   
    float q2q3 = INS_quat[2]*INS_quat[3];
    float q3q3 = INS_quat[3]*INS_quat[3];   

    gx = mpu6500_real_data.gyro[0];
    gy = mpu6500_real_data.gyro[1];
    gz = mpu6500_real_data.gyro[2];
    ax = mpu6500_real_data.accel[0];
    ay = mpu6500_real_data.accel[1];
    az = mpu6500_real_data.accel[2];
    mx = ist8310_real_data.mag[0];
    my = ist8310_real_data.mag[1];
    mz = ist8310_real_data.mag[2];		

    now = Get_Time_Micros();  //��ȡʱ�� ��λ��us   
    if(now<lastUpdate)
    {
    }
    else	
    {
        halfT =  ((float)(now - lastUpdate) / 2000000.0f);
    }
    lastUpdate = now;	//����ʱ��
    //������ƽ�����㷨
    norm = AHRS_invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //�ѼӼƵ���ά����ת�ɵ�λ������
    norm = AHRS_invSqrt(mx*mx + my*my + mz*mz);          
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm; 
    // compute reference direction of flux
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki_q * halfT;
        eyInt = eyInt + ey * Ki_q * halfT;	
        ezInt = ezInt + ez * Ki_q * halfT;
        // �ò���������PI����������ƫ
        gx = gx + Kp_q*ex + exInt;
        gy = gy + Kp_q*ey + eyInt;
        gz = gz + Kp_q*ez + ezInt;
    }
    // ��Ԫ��΢�ַ���
    tempq0 = INS_quat[0] + (-INS_quat[1]*gx - INS_quat[2]*gy - INS_quat[3]*gz)*halfT;
    tempq1 = INS_quat[1] + (INS_quat[0]*gx + INS_quat[2]*gz - INS_quat[3]*gy)*halfT;
    tempq2 = INS_quat[2] + (INS_quat[0]*gy - INS_quat[1]*gz + INS_quat[3]*gx)*halfT;
    tempq3 = INS_quat[3] + (INS_quat[0]*gz + INS_quat[1]*gy - INS_quat[2]*gx)*halfT;  

    // ��Ԫ���淶��
    norm = AHRS_invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    INS_quat[0] = tempq0 * norm;
    INS_quat[1] = tempq1 * norm;
    INS_quat[2] = tempq2 * norm;
    INS_quat[3] = tempq3 * norm;
}

//��ȡϵͳʱ��
uint32_t Get_Time_Micros(void)
{
	return TIM2->CNT;
}
#endif
