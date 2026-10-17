#ifndef __AS5600_H
#define __AS5600_H
#include "i2c.h"
#include "math.h"
#include <stdint.h>
#include "BLDC.h"

void GetAngle(void);
void GetAngle_Without_Track();
void AS5600_Measure(void);
float AS5600_Analyse(void);
void AS5600_Init(void);
float AS5600_GetVelocity(void);
float AS5600_Get_FliteredVelocity();

#endif /* __AS5600_H */
