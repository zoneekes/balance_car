#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f1xx_hal.h"

void load(int motor1,int motor2);  //motor取值范围-7200~7200
void limit(int *motor1,int *motor2);  //限制电机PWM输出范围

// Convenience motor control helpers
void motor_stop(void);
void motor_forward(int pwm);
void motor_backward(int pwm);
void motor_turn_left(int pwm);
void motor_turn_right(int pwm);
void motor_set_pwms(int left_pwm, int right_pwm);

#endif