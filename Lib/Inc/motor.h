#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f1xx_hal.h"

void load(int motor1,int motor2);  //motor取值范围-7200~7200
void limit(int *motor1,int *motor2);  //限制电机PWM输出范围
#endif