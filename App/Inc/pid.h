#ifndef __PID_H__
#define __PID_H__
#include "stm32f1xx_hal.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"

void pid_control(void);

/* PID parameters that can be modified at runtime and persisted.
   Declared extern here so other modules (flash storage, bluetooth) can access them. */
extern float vertical_kp;
extern float vertical_kd;
extern float speed_kp;
extern float speed_ki;
extern float steering_kp;
extern float steering_kd;
extern int target_speed;
extern int target_angle;

/* Parse a single command string received via Bluetooth and optionally write a response.
   cmd: null-terminated input line (no trailing CR/LF required)
   resp: buffer to receive NUL-terminated response (may be NULL)
   resp_len: length of resp buffer
   Returns 1 on success, 0 on failure/unrecognized command. */
int pid_handle_command(const char *cmd, char *resp, size_t resp_len);

#endif