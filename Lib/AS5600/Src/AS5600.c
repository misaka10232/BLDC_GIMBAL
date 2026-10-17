#include "AS5600.h"
#include <stdint.h>
#include "tim.h"
#define AS5600_ADDRESS (0x36 << 1)          // AS5600 I2C地址
#define AS5600_RAW_ANGLE_H    0x0C          // AS5600原始角度寄存器高字节地址
#define AS5600_RAW_ANGLE_L    0x0D          // AS5600原始角度寄存器低字节地址
#define AS5600_IIC      &hi2c1              // I2C句柄，根据实际情况修改
#define _2PI 6.28318530718f
#define AS5600_DIRECT 1                    // 1为顺时针增大，-1为逆时针增大，根据实际情况修改
uint8_t data[2] = {0};                      // 用于存储从AS5600读取的数据


//这两个变量extern出去，在电机操控函数里面要用
volatile float Angle_WithTrack;             // 带累计的角度值
volatile float Angle_WithoutTrack;          // 不带累计的角度值

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
    HAL_I2C_Master_Transmit(AS5600_IIC, AS5600_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(AS5600_IIC, AS5600_ADDRESS, data, 2, HAL_MAX_DELAY);
    Angle_WithoutTrack = AS5600_DIRECT * AS5600_Analyse();
    Angle_WithTrack = Angle_WithoutTrack;
}




void AS5600_WriteReg(uint8_t RegAddress, uint8_t Data) // 写寄存器函数
{
    HAL_I2C_Master_Transmit(AS5600_IIC, AS5600_ADDRESS,
             &RegAddress, 1, HAL_MAX_DELAY); // 发送寄存器地址
    HAL_I2C_Master_Transmit(AS5600_IIC, AS5600_ADDRESS,
             &Data, 1, HAL_MAX_DELAY); // 发送数据
}

uint8_t AS5600_ReadReg(uint8_t RegAddress) // 读寄存器函数
{
    uint8_t Data;
    HAL_I2C_Master_Transmit(AS5600_IIC, AS5600_ADDRESS,
             &RegAddress, 1, HAL_MAX_DELAY); // 发送寄存器地址
    HAL_I2C_Master_Receive(AS5600_IIC, AS5600_ADDRESS,
             &Data, 1, HAL_MAX_DELAY); // 接收数据
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
volatile uint8_t AS5600_GetState = 0; // 0表示空闲，1表示正在接收数据
static uint32_t measure_timeout = 0;  // DMA传输超时时间戳

void AS5600_Measure(void)
{
    /* 超时保护：如果DMA传输超过50ms仍未完成，强制重置状态 */
    if (AS5600_GetState)
    {
        if (HAL_GetTick() - measure_timeout > 50)
        {
            HAL_I2C_Master_Abort_IT(AS5600_IIC, AS5600_ADDRESS);
            AS5600_GetState = 0;
        }
        return;
    }

    AS5600_GetState = 1;
    measure_timeout = HAL_GetTick();

    if(HAL_I2C_Mem_Read_DMA(
        AS5600_IIC,
        AS5600_ADDRESS,
        AS5600_RAW_ANGLE_H,
        I2C_MEMADD_SIZE_8BIT,
        data,
        2
    ) != HAL_OK)
    {
        AS5600_GetState = 0;
    }
}


 /**
   * 函    数：AS5600_Analyse
   * 功    能：解析从AS5600读取的原始数据，计算出当前的角度值，单位为弧度
   * 输    入：无
   * 参    数：无
   * 返 回 值：当前角度值
   */
float AS5600_Analyse(void)
{
    uint16_t raw = (((uint16_t)data[0] << 8) | data[1]) & 0x0FFF; // 12位原始角度值
    return (float)raw * _2PI / 4096.0f;
}



volatile float full_rotations = 0.0f;//圈数
volatile float Last_Angle = 0.0f;
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
    float angle = AS5600_Analyse(); 
    float d_angle = angle - Last_Angle;

    if (d_angle > M_PI) 
    {
        full_rotations -= 1;
    } 
    else if (d_angle < -M_PI) 
    {
        full_rotations += 1;
    }

    Last_Angle = angle;

    Angle_WithTrack = AS5600_DIRECT * (full_rotations * _2PI + angle);
}

 /**
   * 函    数：获取不带累计的角度值的函数
   * 功    能：获取不带累计的角度值
   * 输    入：无
   * 参    数：无
   * 返 回 值：角度值
   */
void GetAngle_Without_Track()
{
    float Angle = AS5600_DIRECT * AS5600_Analyse();
    while (Angle < 0.0f)
    {
        Angle += _2PI;
    }
    while (Angle >= _2PI)
    {
        Angle -= _2PI;
    }
    Angle_WithoutTrack = Angle;

}


float AS5600_GetVelocity()
{
    static float Last_Angle_WithTrack;
    static uint32_t Last_Time;
    static uint32_t Current_Time;
    Current_Time = HAL_GetTick();                       // 获取当前时间戳
    uint32_t d_time = Current_Time - Last_Time;         // 计算时间差，单位为ms
    float dt = d_time / 1000.0f;                        // 将时间差转换为秒
    if (dt == 0) dt = 1e-3;                             // 避免除以0
    float d_angle = Angle_WithTrack - Last_Angle_WithTrack; // 计算角度差
    float velocity = d_angle / dt;                      // 计算角速度，单位为rad/s
    Last_Angle_WithTrack = Angle_WithTrack;             // 更新上次的角度值
    Last_Time = Current_Time;                           // 更新上次的时间戳
    return velocity;
}


float AS5600_Get_FliteredVelocity()
{
    static float Last_Velocity = 0.0f;
    static float Flitered_Velocity = 0.0f;
    const float a = 0.1f; // 滤波系数，范围为0-1，值越小滤波效果越强
    float velocity = AS5600_GetVelocity(); // 获取当前角速度
    Flitered_Velocity = a * velocity + (1.0f - a) * Last_Velocity; // 一阶低通滤波
    Last_Velocity = Flitered_Velocity; // 更新上次的角速度值
    return Flitered_Velocity;
}



uint8_t k;
// I2C接收完成回调函数,在此函数中读取数据并更新角度值
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == AS5600_IIC) // 判断是否为AS5600的I2C传输完成
    {
        j ++;
        GetAngle(); // 更新角度值，带累计和不带累计都会更新
        GetAngle_Without_Track();
        AS5600_GetState = 0; // 设置状态为接收完成
    }
}

// I2C错误回调函数：传输失败时重置状态，防止状态锁死
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == AS5600_IIC)
    {   k++;
        AS5600_GetState = 0; // 发生错误时重置状态，确保下次可以重新发起传输
    }
}