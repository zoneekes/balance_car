#include "sr04.h"
#include "stm32f1xx_hal_gpio.h"

extern TIM_HandleTypeDef htim3; //定义定时器句柄

uint32_t time = 0;
uint32_t distance = 0;

//微秒级延时
void delay_us(uint32_t udelay)
{
  __IO uint32_t Delay = udelay * 72 / 8;//(SystemCoreClock / 8U / 1000000U)
    //见stm32f1xx_hal_rcc.c -- static void RCC_Delay(uint32_t mdelay)
  do
  {
    __NOP();
  }
  while (Delay --);
}


int get_distance()
{
 
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); //PA3输出高电平
  delay_us(20); //延时20us
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); //PA3输出低电平

  
  distance = time * 340 / 20000; //计算距离，单位为cm
  return  (int)distance;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  {
    
    if(GPIO_Pin == GPIO_PIN_2) //判断是否为PA2引脚中断
    {
      if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) //判断PA2引脚电平是否为高电平
      {
        __HAL_TIM_SET_COUNTER(&htim3, 0); //清零计数器
        HAL_TIM_Base_Start(&htim3); //启动定时器
      }
      else
      {
        HAL_TIM_Base_Stop(&htim3); //停止定时器
        time = __HAL_TIM_GET_COUNTER(&htim3); //获取计数器的值
        __HAL_TIM_SET_COUNTER(&htim3, 0); //清零计数器
      }
      
    }

    //每10ms执行一次pid控制函数
    if(GPIO_Pin == GPIO_PIN_5) 
    {
      pid_control();
      
    }
  }