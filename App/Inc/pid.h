#ifndef __PID_H__
#define __PID_H__
#include "stm32f1xx_hal.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"

void pid_control(void);

/* Handle a command string received via Bluetooth.
   Supported commands (example formats):
     V_KP=1.23   -- set vertical_kp
     V_KD=0.45   -- set vertical_kd
     S_KP=0.5    -- set speed_kp
     S_KI=0.01   -- set speed_ki
     ST_KP=0.2   -- set steering_kp
     ST_KD=0.02  -- set steering_kd
     T_SPEED=100 -- set target_speed (int)
     T_ANGLE=5   -- set target_angle (int)
   Returns 1 on success, 0 on unrecognized command. */
int pid_handle_command(const char *cmd);

#endif