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
#include "bluetooth.h"
#include "pid.h"
#include "font.h"


void task_init(void);
void task_test(void);

#endif