#include "encoder.h"
#include "stm32f1xx_hal_tim.h"

int read_speed(TIM_HandleTypeDef *htim){
    int temp=0;
    temp=(short)__HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim,0);
    return temp;
}