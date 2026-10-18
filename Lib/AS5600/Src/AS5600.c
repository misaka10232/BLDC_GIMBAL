#include "AS5600.h"
#include <stdint.h>
#include "tim.h"
#define AS5600_ADDRESS (0x36 << 1)          // AS5600 I2C地址
#define AS5600_RAW_ANGLE_H    0x0C          // AS5600原始角度寄存器高字节地址
#define AS5600_RAW_ANGLE_L    0x0D          // AS5600原始角度寄存器低字节地址
#define AS5600_IIC_LOW      &hi2c1              // I2C句柄，根据实际情况修改
#define AS5600_IIC_HIGH     &hi2c2              // I2C句柄，根据实际情况修改
#define _2PI 6.28318530718f
#define AS5600_DIRECT_LOW 1                    // 1为顺时针增大，-1为逆时针增大，根据实际情况修改
#define AS5600_DIRECT_HIGH 1                   // 1为顺时针增大，-1为逆时针增大，根据实际情况修改
uint8_t AS5600_DIRECT[2] = {AS5600_DIRECT_LOW, AS5600_DIRECT_HIGH}; // 1为顺时针增大，-1为逆时针增大，根据实际情况修改
uint8_t data_low[2] = {0};                      // 用于存储从AS5600读取的数据
uint8_t data_high[2] = {0};                     // 用于存储从AS5600读取的数据


//这两个变量extern出去，在电机操控函数里面要用
volatile float Angle_WithTrack[2] = {0.0f, 0.0f};             // 带累计的角度值
volatile float Angle_WithoutTrack[2] = {0.0f, 0.0f};          // 不带累计的角度值

volatile uint8_t j = 0; // 用于调试的变量


 /**
   * 函    数：AS5600_Init
   * 功    能： 初始化AS5600，先读取一次角度值初始化last_angle，然后启动DMA模式开始测量，测量完成后会触发I2C传输完成中断，
   * 输    入：无
   * 参    数：无
   * 返 回 值：无
   */
void AS5600_Init(void)
{
    // 先读取一次初始化 last_angle
    uint8_t reg = AS5600_RAW_ANGLE_H;
    HAL_I2C_Master_Transmit(AS5600_IIC_LOW, AS5600_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(AS5600_IIC_LOW, AS5600_ADDRESS, data_low, 2, HAL_MAX_DELAY);

    HAL_I2C_Master_Transmit(AS5600_IIC_HIGH, AS5600_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(AS5600_IIC_HIGH, AS5600_ADDRESS, data_high, 2, HAL_MAX_DELAY);
    
    Angle_WithoutTrack[0] = AS5600_DIRECT[0] * AS5600_Analyse(0);
    Angle_WithTrack[0] = Angle_WithoutTrack[0];

    Angle_WithoutTrack[1] = AS5600_DIRECT[1] * AS5600_Analyse(1);
    Angle_WithTrack[1] = Angle_WithoutTrack[1];
}




void AS5600_WriteReg(uint8_t RegAddress, uint8_t Data) // 写寄存器函数
{
    HAL_I2C_Master_Transmit(AS5600_IIC_LOW, AS5600_ADDRESS,
             &RegAddress, 1, HAL_MAX_DELAY); // 发送寄存器地址
    HAL_I2C_Master_Transmit(AS5600_IIC_LOW, AS5600_ADDRESS,
             &Data, 1, HAL_MAX_DELAY); // 发送数据
    HAL_I2C_Master_Transmit(AS5600_IIC_HIGH, AS5600_ADDRESS,
             &RegAddress, 1, HAL_MAX_DELAY); // 发送寄存器地址
    HAL_I2C_Master_Transmit(AS5600_IIC_HIGH, AS5600_ADDRESS,
             &Data, 1, HAL_MAX_DELAY); // 发送数据
}

uint8_t AS5600_ReadReg(uint8_t RegAddress, uint8_t motor) // 读寄存器函数
{
    uint8_t Data;
    if (motor == 1)
    {    
        HAL_I2C_Master_Transmit(AS5600_IIC_LOW, AS5600_ADDRESS,
             &RegAddress, 1, HAL_MAX_DELAY); // 发送寄存器地址
        HAL_I2C_Master_Receive(AS5600_IIC_LOW, AS5600_ADDRESS,
             &Data, 1, HAL_MAX_DELAY); // 接收数据
    }
    else if (motor == 2)
    {    
        HAL_I2C_Master_Transmit(AS5600_IIC_HIGH, AS5600_ADDRESS,
             &RegAddress, 1, HAL_MAX_DELAY); // 发送寄存器地址
        HAL_I2C_Master_Receive(AS5600_IIC_HIGH, AS5600_ADDRESS,
             &Data, 1, HAL_MAX_DELAY); // 接收数据
    }    
    return Data;
}


 /**
   * 函    数：AS5600_Measure
   * 功    能：发送测量命令，开始测量角度值，测量完成后会触发I2C传输完成中断，
                在中断回调函数中读取数据并更新角度值
   * 输    入：无
   * 参    数：无
   * 返 回 值：无
   */
volatile uint8_t AS5600_GetState[2] = {0, 0}; // 0表示空闲，1表示正在接收数据
static uint32_t measure_timeout[2] = {0, 0};  // DMA传输超时时间戳

void AS5600_Measure(void)
{
    /* 超时保护：如果DMA传输超过50ms仍未完成，强制重置状态 */
    if (AS5600_GetState[0])
    {
        if (HAL_GetTick() - measure_timeout[0] > 50)
        {
            HAL_I2C_Master_Abort_IT(AS5600_IIC_LOW, AS5600_ADDRESS);
            AS5600_GetState[0] = 0;
        }
        return;
    }
    if (AS5600_GetState[1])
    {
        if (HAL_GetTick() - measure_timeout[1] > 50)
        {
            HAL_I2C_Master_Abort_IT(AS5600_IIC_HIGH, AS5600_ADDRESS);
            AS5600_GetState[1] = 0;
        }
        return;
    }

    AS5600_GetState[0] = 1;
    AS5600_GetState[1] = 1;
    measure_timeout[0] = HAL_GetTick();
    measure_timeout[1] = HAL_GetTick();

    if(HAL_I2C_Mem_Read_DMA(
        AS5600_IIC_LOW,
        AS5600_ADDRESS,
        AS5600_RAW_ANGLE_H,
        I2C_MEMADD_SIZE_8BIT,
        data_low,
        2
    ) != HAL_OK)
    {
        AS5600_GetState[0] = 0;
    }

    if(HAL_I2C_Mem_Read_DMA(
        AS5600_IIC_HIGH,
        AS5600_ADDRESS,
        AS5600_RAW_ANGLE_H,
        I2C_MEMADD_SIZE_8BIT,
        data_high,
        2
    ) != HAL_OK)
    {
        AS5600_GetState[1] = 0; 
    }
}


 /**
   * 函    数：AS5600_Analyse
   * 功    能：解析从AS5600读取的原始数据，计算出当前的角度值，单位为弧度
   * 输    入：无
   * 参    数：无
   * 返 回 值：当前角度值
   */
float AS5600_Analyse(uint8_t motor_number)
{
    if (motor_number == 1)
    {
        uint16_t raw = (((uint16_t)data_low[0] << 8) | data_low[1]) & 0x0FFF; // 12位原始角度值
        return (float)raw * _2PI / 4096.0f;
    }
    else if (motor_number == 2)
    {
        uint16_t raw = (((uint16_t)data_high[0] << 8) | data_high[1]) & 0x0FFF; // 12位原始角度值
        return (float)raw * _2PI / 4096.0f;
    }
    else
    {
        return 0.0f; // 无效的电机编号，返回0
    }
}



volatile float full_rotations[2] = {0.0f, 0.0f};//圈数
volatile float Last_Angle[2] = {0.0f, 0.0f}; // 上一次的角度值，用于计算角度变化
//磁编码器弧度制角度累计计算:(0-∞)
 /**
   * 函    数：获取累计角度值的函数
   * 功    能：获取累计角度值
   * 输    入：无
   * 参    数：无
   * 返 回 值：累计角度值
   */
void GetAngle(void)
{
    float angle_low = AS5600_Analyse(1); 
    float angle_high = AS5600_Analyse(2); 
    float d_angle_low = angle_low - Last_Angle[0];
    float d_angle_high = angle_high - Last_Angle[1];

    if (d_angle_low > M_PI) 
    {
        full_rotations[0] -= 1;
    } 
    else if (d_angle_low < -M_PI) 
    {
        full_rotations[0] += 1;
    }

    if (d_angle_high > M_PI) 
    {
        full_rotations[1] -= 1;
    } 
    else if (d_angle_high < -M_PI) 
    {
        full_rotations[1] += 1;
    }

    
    Last_Angle[0] = angle_low;
    Last_Angle[1] = angle_high;

    Angle_WithTrack[0] = AS5600_DIRECT[0] * (full_rotations[0] * _2PI + angle_low);
    Angle_WithTrack[1] = AS5600_DIRECT[1] * (full_rotations[1] * _2PI + angle_high);
}

 /**
   * 函    数：获取不带累计的角度值的函数
   * 功    能：获取不带累计的角度值
   * 输    入：无
   * 参    数：无
   * 返 回 值：角度值
   */
void GetAngle_Without_Track(void)
{
    float Angle_low = AS5600_DIRECT[0] * AS5600_Analyse(1);
    float Angle_high = AS5600_DIRECT[1] * AS5600_Analyse(2);
    while (Angle_low < 0.0f)
    {
        Angle_low += _2PI;
    }
    while (Angle_low >= _2PI)
    {
        Angle_low -= _2PI;
    }
    Angle_WithoutTrack[0] = Angle_low;

    while (Angle_high < 0.0f)
    {
        Angle_high += _2PI;
    }
    while (Angle_high >= _2PI)
    {
        Angle_high -= _2PI;
    }
    Angle_WithoutTrack[1] = Angle_high;

}


float AS5600_GetVelocity(uint8_t motor_number)
{
    if (motor_number < 1 || motor_number > 2)
    {
        return 0.0f; // 无效的电机编号，返回0
    }
    static float Last_Angle_WithTrack[2] = {0.0f, 0.0f};
    static uint32_t Last_Time[2] = {0, 0};
    static uint32_t Current_Time;
    Current_Time = HAL_GetTick();                       // 获取当前时间戳
    uint32_t d_time = Current_Time - Last_Time[motor_number - 1];         // 计算时间差，单位为ms
    float dt = d_time / 1000.0f;                        // 将时间差转换为秒
    if (dt == 0) dt = 1e-3;                             // 避免除以0
    float d_angle = Angle_WithTrack[motor_number - 1] - Last_Angle_WithTrack[motor_number - 1]; // 计算角度差
    float velocity = d_angle / dt;                      // 计算角速度，单位为rad/s
    Last_Angle_WithTrack[motor_number - 1] = Angle_WithTrack[motor_number - 1];             // 更新上次的角度值
    Last_Time[motor_number - 1] = Current_Time;                           // 更新上次的时间戳
    return velocity;
}



 /**
   * 函    数：获取滤波速度
   * 功    能： 获取滤波后的速度值，使用一阶低通滤波器对速度进行滤波
   *           滤波器公式：Flitered_Velocity = a * Current
   * 输    入：电机编号
   * 参    数：无
   * 返 回 值：滤波后的速度值
   */
float AS5600_Get_FliteredVelocity(uint8_t motor_number)
{
    if (motor_number < 1 || motor_number > 2)
    {
        return 0.0f; // 无效的电机编号，返回0
    }
    static float Last_Velocity[2] = {0.0f, 0.0f};
    float Flitered_Velocity = 0.0f;
    const float a = 0.3f; // 滤波系数，范围为0-1，值越小滤波效果越强
    float velocity = AS5600_GetVelocity(motor_number); // 获取当前角速度
    Flitered_Velocity = a * velocity + (1.0f - a) * Last_Velocity[motor_number - 1]; // 一阶低通滤波
    Last_Velocity[motor_number - 1] = Flitered_Velocity; // 更新上次的角速度值
    return Flitered_Velocity;
}  



uint8_t k;
// I2C接收完成回调函数,在此函数中读取数据并更新角度值
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == AS5600_IIC_LOW || hi2c == AS5600_IIC_HIGH) // 判断是否为AS5600的I2C传输完成
    {
        j ++;
        GetAngle(); // 更新角度值，带累计和不带累计都会更新
        GetAngle_Without_Track();
        AS5600_GetState[0] = 0; // 设置状态为接收完成
    }
}


// I2C错误回调函数：传输失败时重置状态，防止状态锁死
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == AS5600_IIC_LOW || hi2c == AS5600_IIC_HIGH)
    {   k++;
        AS5600_GetState[0] = 0; // 发生错误时重置状态，确保下次可以重新发起传输
    }
}