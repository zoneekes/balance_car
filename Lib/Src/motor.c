#include "motor.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "tim.h"

#define MOTOR_MAX 7200
#define MOTOR_MIN -7200

static int my_abs(int a){
    return (a < 0) ? -a : a;
}

//加载电机PWM输出
void load(int motor1,int motor2){
    if(motor1<0){
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, my_abs(motor1));

    if(motor2<0){
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, my_abs(motor2));

}

//限制电机PWM输出范围
void limit(int *motor1,int *motor2){
    if(*motor1>MOTOR_MAX)
        *motor1=MOTOR_MAX;
    else if(*motor1<MOTOR_MIN)
        *motor1=MOTOR_MIN;

    if(*motor2>MOTOR_MAX)
        *motor2=MOTOR_MAX;
    else if(*motor2<MOTOR_MIN)
        *motor2=MOTOR_MIN;
}

// Convenience helpers for directional control
void motor_set_pwms(int left_pwm, int right_pwm){
    int l = left_pwm;
    int r = right_pwm;
    limit(&l, &r);
    load(l, r);
}

void motor_stop(void){
    motor_set_pwms(0, 0);
}

void motor_forward(int pwm){
    // move forward: both motors positive
    motor_set_pwms(pwm, pwm);
}

void motor_backward(int pwm){
    // move backward: both motors negative
    motor_set_pwms(-pwm, -pwm);
}

void motor_turn_left(int pwm){
    // left turn: left motor backward, right motor forward
    motor_set_pwms(-pwm, pwm);
}

void motor_turn_right(int pwm){
    // right turn: left motor forward, right motor backward
    motor_set_pwms(pwm, -pwm);
}