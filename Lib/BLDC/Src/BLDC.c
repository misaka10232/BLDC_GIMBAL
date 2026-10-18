#include "BLDC.h"
#include "AS5600.h"
#include "tim.h"
#include "usart.h"
#include <stdint.h>
#include <math.h>

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))//限制函数，限制amt在low和high之间
#define BLDC_DIRECT_LOW -1                                    //电机旋转方向，1为逆时针正方向，
                                                          // -1为顺时针，根据实际情况修改
#define BLDC_DIRECT_HIGH -1                                    //电机旋转方向，1为逆时针正方向，
                                                          // -1为顺时针，根据实际情况修改
#define LOW_MOTOR                 1     
#define HIGH_MOTOR                2
uint8_t BLDC_DIRECT[2] = {BLDC_DIRECT_LOW, BLDC_DIRECT_HIGH}; //电机旋转方向，1为逆时针正方向，-1为顺时针，根据实际情况修改


#define POLE_PAIRS                7                       //极对数，根据实际电机调整
#define PWM_TIM_LOW                   &htim1                  //PWM定时器，根据实际情况修改
#define PWM_TIM_HIGH                  &htim2                  //PWM定时器，根据实际情况修改

#define _3PI_2                    4.71238898038f          //3pi/2，电角度270度，帮助电机启动
#define PERIODCALLBACK_TIM        &htim8                  //定时器中断回调函数使用的定时器，根据实际情况修改
#define SQRT3                     1.73205080757f          //根号3，电压变换中使用

float voltage_limit =             10.0;                   //BLDC电压限制
float voltage_power_supply =      12.0;                   //电源电压
float shaft_angle =               0;                      //轴角度
uint32_t open_loop_timestamp[2] = {0, 0};                      //开环时间戳
float zero_electrical_angle[2] = {0, 0};                      //零电角度
float Ualpha = 0, Ubeta =         0;                      //两相电压
float ua = 0, ub = 0, uc =        0;                      //三相电压
float da_a = 0, db_b = 0, dc_c =  0;                      //三相占空比

float Target_Velocity = 0.0f; // 目标速度，单位为rad/s

extern volatile float Angle_WithTrack[2];                    // 带累计的角度值
extern volatile float Angle_WithoutTrack[2];                 // 不带累计的角度值


/**
  * 函    数：电机初始化函数,定时器1用于PWM输出，定时器4用于开环速度生成器的时间基准(单位：ms)
  * 功    能：此函数内部只启动PWM输出，定时器需要cubemx，此处使用二分频，
                自动重装载值为1800-1，中心对齐模式3，PWM频率约为20kHz，分辨率为1/1800
  *             用户需要根据实际情况调整定时器参数以满足电机控制需求
    *           其他功能如传感器校准、零电角度测量等需要用户根据实际情况添加（以后再写）
  * 输    入：无
  * 参    数：无
  * 返 回 值：无
  */
void BLDC_Init(void)
{
    // 启动PWM
    HAL_TIM_PWM_Start(PWM_TIM_LOW, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(PWM_TIM_LOW, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(PWM_TIM_LOW, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(PWM_TIM_HIGH, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(PWM_TIM_HIGH, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(PWM_TIM_HIGH, TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(PERIODCALLBACK_TIM);    // 启动周期回调定时器
    char buffer[30] = {"完成电机初始化"};
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer,
                         sizeof(buffer), HAL_MAX_DELAY);

    HAL_Delay(1000);                             // 等待电机稳定
    set_phase_voltage(3, 0, _3PI_2, LOW_MOTOR);      // 以电角度270度（即物理角度270/极对数）施加电压，帮助电机启动
    set_phase_voltage(3, 0, _3PI_2, HIGH_MOTOR);      // 以电角度270度（即物理角度270/极对数）施加电压，帮助电机启动
    HAL_Delay(3000);
    zero_electrical_angle[0] = _electricalangle(LOW_MOTOR);         // 读取当前电角度作为零电角度
    zero_electrical_angle[1] = _electricalangle(HIGH_MOTOR);         // 读取当前电角度作为零电角度
    set_phase_voltage(0, 0, _3PI_2, LOW_MOTOR);      // 施加0电压，停止电机
    set_phase_voltage(0, 0, _3PI_2, HIGH_MOTOR);      // 施加0电压，停止电机
}


void BLDC_Enable(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); //使能低处电机
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); //使能高处电机
}

void BLDC_Disable(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); 
}

/**
  * 函    数：计算电角度、设置三相电压和设置相电压的函数
  * 功    能：计算电角度
  * 输    入：物理角度与极对数
  * 返 回 值：电角度
  */

float electrical_angle(float shaft_Angle, uint8_t pole_pairs)// 计算电角度
{
    return (shaft_Angle * pole_pairs);
}


float _electricalangle(uint8_t motor_number)
{
    if (motor_number < 1 || motor_number > 2)
    {
        return 0.0f; // 无效的电机编号，返回0
    }
    return angle_nomalize((float)(POLE_PAIRS) * 
        Angle_WithoutTrack[motor_number - 1] - 
        zero_electrical_angle[motor_number - 1]); // 计算电角度并归一化
}
/**
  * 函    数：电角度归一化函数
  * 功    能：归一化电角度
  * 输    入：电角度
  * 返 回 值：归一化后的电角度（0-360度）
  */

float angle_nomalize(float angle)// 角度归一化到0-2pi范围内
{
    float a = fmodf(angle, 2.0f * (float)M_PI);
    return a >= 0.0f ? a : (a + 2.0f * (float)M_PI);
}

/**
  * 函    数：计算三项占空比和设置三相电压的函数
  * 功    能：计算三项占空比并设置三相电压
  * 输    入：三相电压
  * 返 回 值：无
  */

void set_PWM(float Ua, float Ub, float Uc, uint8_t motor_number) // 设置三相电压
{

    // 电压限制
    Ua = _constrain(Ua, 0, voltage_limit);
    Ub = _constrain(Ub, 0, voltage_limit);
    Uc = _constrain(Uc, 0, voltage_limit);

    // 电压转占空比,限制占空比在0-1之间
    float inv_vcc = 1.0f / voltage_power_supply;
    da_a = _constrain(Ua * inv_vcc, 0.0f, 1.0f);
    db_b = _constrain(Ub * inv_vcc, 0.0f, 1.0f);
    dc_c = _constrain(Uc * inv_vcc, 0.0f, 1.0f);

    // 占空比转PWM值
    if (motor_number == LOW_MOTOR)
    {
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(PWM_TIM_LOW) + 1;
        __HAL_TIM_SET_COMPARE(PWM_TIM_LOW, TIM_CHANNEL_1, (uint32_t)(da_a * arr));
        __HAL_TIM_SET_COMPARE(PWM_TIM_LOW, TIM_CHANNEL_2, (uint32_t)(db_b * arr));
        __HAL_TIM_SET_COMPARE(PWM_TIM_LOW, TIM_CHANNEL_3, (uint32_t)(dc_c * arr));
    }
    else if (motor_number == HIGH_MOTOR)
    {
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(PWM_TIM_HIGH) + 1;
        __HAL_TIM_SET_COMPARE(PWM_TIM_HIGH, TIM_CHANNEL_1, (uint32_t)(da_a * arr));
        __HAL_TIM_SET_COMPARE(PWM_TIM_HIGH, TIM_CHANNEL_2, (uint32_t)(db_b * arr));
        __HAL_TIM_SET_COMPARE(PWM_TIM_HIGH, TIM_CHANNEL_3, (uint32_t)(dc_c * arr));
    }
    else
    {
        // 无效的电机编号，直接返回
        return;
    }
}

/**
  * 函    数：根据输入电压，电角度进行克拉克逆变换和帕克逆变换，并设置PWM输出的函数
  * 功    能：根据输入电压进行克拉克逆变换和帕克逆变换，并设置PWM输出
  * 输    入：Uq：q轴电压，Ud：d轴电压，Angle：电角度（rad）
  * 参    数：无
  * 返 回 值：无
  */

void set_phase_voltage(float Uq, float Ud, float Angle, uint8_t motor) // 设置相电压
{
    if (motor < 1 || motor > 2)
    {
        return; // 无效的电机编号，直接返回
    }
    Angle = angle_nomalize(Angle + zero_electrical_angle[motor - 1]); // 计算电角度并归一化
    
    float sin_a = sinf(Angle);
    float cos_a = cosf(Angle);

    //帕克逆变换，将d轴和q轴电压转换为两相静止坐标系下的电压，注意这里的正负号和坐标系定义有关，可能需要根据实际情况调整
    Ualpha = -Uq * sin_a + Ud * cos_a;
    Ubeta = Uq * cos_a + Ud * sin_a;

    //克拉克逆变换，并加上电源电压的一半以实现正负电压的输出，保证占空比在0-1之间
    float half_vcc = voltage_power_supply * 0.5f;
    float Ualpha_half = Ualpha * 0.5f;
    float Ubeta_sqrt3_half = Ubeta * (SQRT3 * 0.5f);
    
    ua = Ualpha + half_vcc;
    ub = Ubeta_sqrt3_half - Ualpha_half + half_vcc;
    uc = -Ubeta_sqrt3_half - Ualpha_half + half_vcc;

    // 设置PWM
    set_PWM(ua, ub, uc, motor); 
}

 /**
   * 函    数：开环速度生成器
   * 功    能：请注意开环调控不要间隔太久，可能会导致tim4溢出超过两次，大概会出问题？
   * 输    入：角速度(单位：rad / s)
   * 参    数：极对数
   * 返 回 值：当前电压，主要用于调试
   */
 float velocity_open_loop(float target_velocity, uint8_t motor_number)
 {
     if (motor_number < 1 || motor_number > 2)
     {
         return 0.0f; // 无效的电机编号，返回0
     }
     uint32_t current_time = HAL_GetTick(); // 获取当前时间（单位：ms）
     float dt = (current_time - open_loop_timestamp[motor_number - 1]) * 0.001f; // 计算时间差（单位：s）
 
     // 仅当时间有变化时才更新角度和时间戳
     if (dt > 0.0f)
     {
        shaft_angle = angle_nomalize(shaft_angle + target_velocity * dt); // 更新轴角度，target_velocity单位为rad/s
        open_loop_timestamp[motor_number - 1] = current_time; // 更新时间戳
     }
 
     float Uq = voltage_limit; // 开环时直接使用最大电压
     float Ud = 0; // d轴电压为0
     set_phase_voltage(Uq, Ud, electrical_angle(shaft_angle, POLE_PAIRS), motor_number); // 设置相电压
     // 返回当前电压，主要用于调试
     return Uq;
}


 /**
   * 函    数：根据输入目标角度和当前角度进行位置控制的函数，使用简单的P控制器
   * 功    能：根据输入目标角度和当前角度进行位置控制，
   * 输    入：目标角度（单位：rad）
   * 参    数：无
   * 返 回 值：无
   */
float Kd_Last_Angle1[2] = {0.0f, 0.0f}; // 上一次的角度值，用于计算角速度
float Error0[2] = {0.0f, 0.0f};
float Error1[2] = {0.0f, 0.0f};
float DifOut[2] = {0.0f, 0.0f};
float a = 0.5f;
// 使用注意事项：定位控制的结构是主循环不断调用AS5600_Measure()函数更新角度值，
// 定时器3周期性调用Set_TarAngle()函数进行位置控制，目标角度可以根据需要修改，
// 建议不要设置过大或过小的目标角度，以免电机过载或无法达到目标位置。
// 控制参数kp和kd需要根据实际情况调整，以获得更好的控制效果。
// AS5600采用DMA方式读取数据，确保在使用过程中DMA传输正常工作，以避免数据丢失或更新不及时的问题。
// 代码结构大概是：主循环不断调用AS5600_Measure()函数更新角度值，回调函数内部处理数据。
// 定时器3周期性调用Set_TarAngle()函数进行位置控制
void Set_TarAngle(float Target_Angle, uint8_t motor_number)//角度输入范围：0-2pi，单位为rad
{
  if (motor_number < 1 || motor_number > 2)
  {
      return; // 无效的电机编号，直接返回
  }
  float Sensor_Angle = Angle_WithTrack[motor_number - 1]; // 获取当前角度值
  const float kp = 0.133f;
  const float kd = 0.05f;
  const float rad_to_deg = 180.0f / (float)M_PI;
  Error1[motor_number - 1] = Error0[motor_number - 1];
  Error0[motor_number - 1] = Target_Angle - Sensor_Angle;
  DifOut[motor_number - 1] = kd * a * (Error0[motor_number - 1] - Error1[motor_number - 1]) + 
                                  (1.0f - a) * DifOut[motor_number - 1];
  float Uq1 = BLDC_DIRECT[motor_number - 1] * _constrain((kp * Error0[motor_number - 1] + DifOut[motor_number - 1]) * rad_to_deg, -6.0f, 6.0f);
  set_phase_voltage(Uq1, 0, _electricalangle(motor_number), motor_number);
  Kd_Last_Angle1[motor_number - 1] = Sensor_Angle;

}
  
void Set_TarVelocity(float Target_Velocity, uint8_t motor_number)
{
  if (motor_number < 1 || motor_number > 2)
  {
      return; // 无效的电机编号，直接返回
  }
  static float interror[2] = {0.0f, 0.0f};
  static float Last_Error[2] = {0.0f, 0.0f};
  static uint32_t time_last[2] = {0, 0};
  const float kp = 0.2f;
  const float ki = 0.15f;

  uint32_t current_time = HAL_GetTick();
  if (time_last[motor_number - 1] == 0) {
      time_last[motor_number - 1] = current_time;
  }
  float dt = (current_time - time_last[motor_number - 1]) * 0.001f; // 计算时间差（单位：s）
  time_last[motor_number - 1] = current_time; // 更新时间戳

  float Error = Target_Velocity - AS5600_Get_FliteredVelocity(motor_number); // 计算速度误差
  
  if (dt > 0.0f) {
      interror[motor_number - 1] += 0.5f * (Error + Last_Error[motor_number - 1]) * dt;
      interror[motor_number - 1] = _constrain(interror[motor_number - 1], -100.0f, 100.0f); // 积分限幅，防止积分过大导致控制不稳定
  }
  
  Last_Error[motor_number - 1] = Error;
  float Uq = BLDC_DIRECT[motor_number - 1] * _constrain(kp * Error + ki * interror[motor_number - 1], -voltage_limit / 2.0f, voltage_limit / 2.0f);
  set_phase_voltage(Uq, 0, _electricalangle(motor_number), motor_number);
}






uint8_t i = 0; // 用于调试的变量



// 定时器中断回调函数，周期性调用位置控制函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim8)                   // 判断是否为定时器8的中断
  {
    i++; // 用于调试，记录定时器中断的次数
    // AS5600_Measure();                   // 定时器8溢出时调用测量函数更新角度值，确保位置控制使用的是最新的角度数据
    Set_TarVelocity(50, 1);      // 定时器8溢出时调用速度控制函数，目标速度为10 rad/s，根据需要修改
    // Set_TarAngle(1, 1);      // 定时器8溢出时调用位置控制函数，目标角度为0
    
  }
}

