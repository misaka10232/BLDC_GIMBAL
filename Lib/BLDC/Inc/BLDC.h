#ifndef __BLDC_H
#define __BLDC_H

#include "tim.h"
#include "gpio.h"
#include "usart.h"
void set_phase_voltage(float Uq, float Ud, float Angle);
void BLDC_Init(void);
float velocity_open_loop(float target_velocity);
float _electricalangle(void);
void Set_TarAngle(float Target_Angle);
void Set_TarVelocity(float Target_Velocity);
float angle_nomalize(float angle);
void IHM16M1_Enable(void);
void IHM16M1_Disable(void);


#endif /* __BLDC_H */
