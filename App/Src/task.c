#include "task.h"
#include "oled.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include <stdint.h>


char display_buf[50];
uint32_t tick = 0;
extern int encoder_left, encoder_right;
extern float roll;

void task_init(void)
{
  HAL_Delay(20);
  OLED_Init();
  MPU_Init();
  mpu_dmp_init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);//启动PWM输出
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  load(0, 0);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  OLED_NewFrame();
  OLED_PrintASCIIString(0, 0, "Init successed!", &afont12x6, OLED_COLOR_NORMAL);
  OLED_ShowFrame();

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void task_test(void)
{ 
  
  OLED_NewFrame();
  //显示角度
  sprintf(display_buf, "Roll: %.2f", roll);
  OLED_PrintASCIIString(0, 16, display_buf, &afont12x6, OLED_COLOR_NORMAL);
  //显示左右电机的转速
  sprintf(display_buf, "L: %d R: %d", encoder_left, encoder_right);
  OLED_PrintASCIIString(0, 32, display_buf, &afont12x6, OLED_COLOR_NORMAL);

  //显示距离
  int distance = get_distance();
  sprintf(display_buf, "Dist: %d cm", distance);
  OLED_PrintASCIIString(0, 48, display_buf, &afont12x6, OLED_COLOR_NORMAL);
  OLED_ShowFrame();

}


// void read_10ms(){
//   if(uwTick-tick<10) return;
//   tick = uwTick;

//   encoder_left = read_speed(&htim2);
//   encoder_right = -read_speed(&htim4);
// }