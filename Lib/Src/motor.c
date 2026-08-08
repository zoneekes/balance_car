#include "motor.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "tim.h"

#define MOTOR_MAX 7200
#define MOTOR_MIN -7200

int abs(int a){
    if(a<0)
        return -a;
    else
        return a;
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
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, abs(motor1));

    if(motor2<0){
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, abs(motor2));

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