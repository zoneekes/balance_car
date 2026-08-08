#ifndef __TASK_H__
#define __TASK_H__

#include <stdio.h>
#include "oled.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "sr04.h"
#include "motor.h"
#include "encoder.h"




void task_init(void);
void task_test(void);
void read_10ms(void);
#endif