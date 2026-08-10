#include "pid.h"
#include "inv_mpu.h"
#include "mpu6050.h"
#include <sys/_intsup.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MED_OFFSET 9.52
int encoder_left = 0;//左边电机的速度
int encoder_right = 0;//右边电机的速度

float pitch=0, roll=0, yaw=0;//姿态角
short gyro_X, gyro_Y, gyro_Z;//陀螺仪角速度
short acc_X, acc_Y, acc_Z;//加速度计数据

float vertical_kp = 680.0f, vertical_kd = 3.145f;//直立环PD控制器参数(kp-0~1000, kd-0~10)
float speed_kp = 0.06 , speed_ki;//速度环PI控制器参数(kp-0~1, ki=kp/200)
float steering_kp = 1.0f, steering_kd = 0.1f;//转向环PD控制器参数
uint8_t flag_stop;//停止标志位

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

//PID控制器参数中间变量
int vertical_out, speed_out, steering_out,motor_left_out, motor_right_out;
int target_speed = 0;//目标速度
int target_angle = 0;//目标角度

float angle_offset = MED_OFFSET;//平衡时小车的角度值偏移

//直立环PD控制器  
//med:目标角度，angle:当前角度，gyro_Y:陀螺仪Y轴角速度
//返回值为控制量
int vertical_pd_control(float med, float angle, float gyro_p) {
    int temp=0;
    temp = vertical_kp * (angle - med) + vertical_kd * gyro_p;
    return temp;
}

//速度环PI控制器
//med:目标速度，encoder_L:左轮编码器速度，encoder_R:右轮编码器速度
int speed_pi_control(int med, int encoder_L, int encoder_R) {
    int error;
    static float error_lowout_last,encoder_S;
    static float a=0.7f;
    int error_lowout;
    speed_ki = speed_kp / 200.0f; //积分系数为比例系数的1/200
    //计算偏差值
    error = (encoder_L + encoder_R) - med;
    //低通滤波
    error_lowout = a * error + (1 - a) * error_lowout_last;
    error_lowout_last = error_lowout;
    //积分环节
    encoder_S += error_lowout;
    //积分限幅
    if(encoder_S > 10000) encoder_S = 10000;
    else if(encoder_S < -10000) encoder_S = -10000;

    if(flag_stop == 1) {
        encoder_S = 0;
        flag_stop = 0;
    }; //停止时清零积分值
    
    //速度环计算
    int temp = speed_kp * error_lowout + speed_ki * encoder_S;
    return temp;
}

//转向环PD控制器
//med:目标角度，gyro_Z:陀螺仪Z轴角速度
int steering_pd_control(float med,  float gyro_p) {
    int temp=0;
    temp = steering_kp * med + steering_kd * gyro_p;
    return temp;
}

//控制函数
void pid_control(void){
    int pwm_output = 0;
    //读取编码器速度和陀螺仪数据
    encoder_left= read_speed(&htim2);
    encoder_right= -read_speed(&htim4);
    mpu_dmp_get_data(&pitch, &roll, &yaw);
    MPU_Get_Gyroscope(&gyro_X, &gyro_Y, &gyro_Z);
    MPU_Get_Accelerometer(&acc_X, &acc_Y, &acc_Z);

    //计算PID控制器输出
    speed_out = speed_pi_control(target_speed, encoder_left, encoder_right);
    vertical_out = vertical_pd_control(speed_out + angle_offset, roll, gyro_X);
    steering_out = steering_pd_control(target_angle, gyro_Z);  

    //计算电机输出
    pwm_output = vertical_out;
    motor_left_out = pwm_output - steering_out;
    motor_right_out = pwm_output + steering_out;
    limit(&motor_left_out, &motor_right_out);
    load(motor_left_out, motor_right_out);
    
}

// Parse a single command line and optionally fill a response buffer.
int pid_handle_command(const char *cmd, char *resp, size_t resp_len){
    if(cmd == NULL) return 0;

    // make a local copy and strip leading/trailing whitespace and CR/LF
    char buf[64];
    size_t L = strlen(cmd);
    if(L >= sizeof(buf)) L = sizeof(buf)-1;
    strncpy(buf, cmd, L);
    buf[L] = '\0';

    // trim leading spaces
    char *p = buf;
    while(*p == ' ' || *p == '\t') p++;
    // trim trailing CR/LF/space
    char *end = p + strlen(p) - 1;
    while(end >= p && (*end == '\r' || *end == '\n' || *end == ' ' || *end == '\t')){ *end = '\0'; end--; }

    if(strlen(p) == 0) {
        if(resp && resp_len) snprintf(resp, resp_len, "ERR empty\r\n");
        return 0;
    }

    // Handle GET ALL (case-insensitive without relying on strcasecmp)
    {
        char gu[32];
        size_t gu_len = strlen(p);
        if(gu_len >= sizeof(gu)) gu_len = sizeof(gu)-1;
        for(size_t i=0;i<gu_len;i++){
            char c = p[i]; gu[i] = (c >= 'a' && c <= 'z') ? c - 'a' + 'A' : c;
        }
        gu[gu_len] = '\0';
        if(strcmp(gu, "GET ALL") == 0 || strcmp(gu, "GET_ALL") == 0){
            if(resp && resp_len){
                // Format floats as fixed with 3 decimals to avoid linking float printf support
                int vk = (int)(vertical_kp);
                int vkf = (int)((vertical_kp - vk) * 1000.0f + 0.5f);
                int vdk = (int)(vertical_kd);
                int vdkf = (int)((vertical_kd - vdk) * 1000.0f + 0.5f);
                int sk = (int)(speed_kp);
                int skf = (int)((speed_kp - sk) * 1000.0f + 0.5f);
                int sik = (int)(speed_ki);
                int sikf = (int)((speed_ki - sik) * 1000.0f + 0.5f);
                int stk = (int)(steering_kp);
                int stkf = (int)((steering_kp - stk) * 1000.0f + 0.5f);
                int stdk = (int)(steering_kd);
                int stdkf = (int)((steering_kd - stdk) * 1000.0f + 0.5f);
                snprintf(resp, resp_len, "V_KP=%d.%03d V_KD=%d.%03d S_KP=%d.%03d S_KI=%d.%03d ST_KP=%d.%03d ST_KD=%d.%03d T_SPEED=%d T_ANGLE=%d\r\n",
                    vk, vkf, vdk, vdkf, sk, skf, sik, sikf, stk, stkf, stdk, stdkf, target_speed, target_angle);
            }
            return 1;
        }
        // Handle SAVE / LOAD commands
        if(strcmp(gu, "SAVE") == 0){
            extern int flash_save_pid_params(void);
            int r = flash_save_pid_params();
            if(resp && resp_len){
                if(r == 0) snprintf(resp, resp_len, "OK SAVE\r\n");
                else snprintf(resp, resp_len, "ERR SAVE_FAILED %d\r\n", r);
            }
            return (r == 0) ? 1 : 0;
        }
        if(strcmp(gu, "LOAD") == 0){
            extern int flash_load_pid_params(void);
            int r = flash_load_pid_params();
            if(resp && resp_len){
                if(r == 0) snprintf(resp, resp_len, "OK LOAD\r\n");
                else snprintf(resp, resp_len, "ERR LOAD_FAILED %d\r\n", r);
            }
            return (r == 0) ? 1 : 0;
        }
    }

    // Generic KEY=VALUE parsing
    char *eq = strchr(p, '=');
    if(eq == NULL){
        if(resp && resp_len) snprintf(resp, resp_len, "ERR no_equal\r\n");
        return 0;
    }
    *eq = '\0';
    char *key = p;
    char *val = eq + 1;
    // uppercase key for comparison simplicity
    for(char *t = key; *t; ++t) if(*t >= 'a' && *t <= 'z') *t = *t - 'a' + 'A';

    // parse float/int value
    if(strncmp(key, "V_KP", 5) == 0){
        float v = strtof(val, NULL);
        if(v < 0.0f) v = 0.0f; if(v > 1000.0f) v = 1000.0f;
        __disable_irq(); vertical_kp = v; __enable_irq();
        if(resp && resp_len){ int vk=(int)vertical_kp; int vkf=(int)((vertical_kp-vk)*1000.0f+0.5f); snprintf(resp, resp_len, "OK V_KP=%d.%03d\r\n", vk, vkf); }
        return 1;
    }
    if(strncmp(key, "V_KD", 5) == 0){
        float v = strtof(val, NULL);
        if(v < 0.0f) v = 0.0f; if(v > 10.0f) v = 10.0f;
        __disable_irq(); vertical_kd = v; __enable_irq();
        if(resp && resp_len){ int vdk=(int)vertical_kd; int vdkf=(int)((vertical_kd-vdk)*1000.0f+0.5f); snprintf(resp, resp_len, "OK V_KD=%d.%03d\r\n", vdk, vdkf); }
        return 1;
    }
    if(strncmp(key, "S_KP", 5) == 0){
        float v = strtof(val, NULL);
        if(v < 0.0f) v = 0.0f; if(v > 5.0f) v = 5.0f;
        __disable_irq(); speed_kp = v; __enable_irq();
        if(resp && resp_len){ int sk=(int)speed_kp; int skf=(int)((speed_kp-sk)*1000.0f+0.5f); snprintf(resp, resp_len, "OK S_KP=%d.%03d\r\n", sk, skf); }
        return 1;
    }
    if(strncmp(key, "S_KI", 5) == 0){
        float v = strtof(val, NULL);
        if(v < 0.0f) v = 0.0f; if(v > 1.0f) v = 1.0f;
        __disable_irq(); speed_ki = v; __enable_irq();
        if(resp && resp_len){ int sk=(int)speed_ki; int skf=(int)((speed_ki-sk)*1000.0f+0.5f); snprintf(resp, resp_len, "OK S_KI=%d.%03d\r\n", sk, skf); }
        return 1;
    }
    if(strncmp(key, "ST_KP", 6) == 0){
        float v = strtof(val, NULL);
        if(v < 0.0f) v = 0.0f; if(v > 20.0f) v = 20.0f;
        __disable_irq(); steering_kp = v; __enable_irq();
        if(resp && resp_len){ int sk=(int)steering_kp; int skf=(int)((steering_kp-sk)*1000.0f+0.5f); snprintf(resp, resp_len, "OK ST_KP=%d.%03d\r\n", sk, skf); }
        return 1;
    }
    if(strncmp(key, "ST_KD", 6) == 0){
        float v = strtof(val, NULL);
        if(v < 0.0f) v = 0.0f; if(v > 5.0f) v = 5.0f;
        __disable_irq(); steering_kd = v; __enable_irq();
        if(resp && resp_len){ int sk=(int)steering_kd; int skf=(int)((steering_kd-sk)*1000.0f+0.5f); snprintf(resp, resp_len, "OK ST_KD=%d.%03d\r\n", sk, skf); }
        return 1;
    }
    if(strncmp(key, "T_SPEED", 7) == 0){
        int v = atoi(val);
        if(v > 2000) v = 2000; if(v < -2000) v = -2000;
        __disable_irq(); target_speed = v; __enable_irq();
        if(resp && resp_len) snprintf(resp, resp_len, "OK T_SPEED=%d\r\n", target_speed);
        return 1;
    }
    if(strncmp(key, "T_ANGLE", 7) == 0){
        int v = atoi(val);
        if(v > 30) v = 30; if(v < -30) v = -30;
        __disable_irq(); target_angle = v; __enable_irq();
        if(resp && resp_len) snprintf(resp, resp_len, "OK T_ANGLE=%d\r\n", target_angle);
        return 1;
    }

    // Movement commands - accept CMD=FORWARD:100 or CMD=FORWARD (default pwm 1000)
    if(strncmp(key, "CMD", 3) == 0){
        // include motor control functions from motor.h
        int pwm = 1000; // default pwm magnitude
        if(*val != '\0'){
            // allow optional value after comma or : e.g. FORWARD:800 or FORWARD,800
            char *sep = strpbrk(val, ":,");
            if(sep){ *sep = '\0'; pwm = atoi(sep+1); }
        }
        // uppercase command token in val
        for(char *t = val; *t; ++t) if(*t >= 'a' && *t <= 'z') *t = *t - 'a' + 'A';
        if(strcmp(val, "FORWARD") == 0){ motor_forward(pwm); if(resp) snprintf(resp, resp_len, "OK CMD=FORWARD PWM=%d\r\n", pwm); return 1; }
        if(strcmp(val, "BACKWARD") == 0 || strcmp(val, "BACK") == 0){ motor_backward(pwm); if(resp) snprintf(resp, resp_len, "OK CMD=BACKWARD PWM=%d\r\n", pwm); return 1; }
        if(strcmp(val, "LEFT") == 0){ motor_turn_left(pwm); if(resp) snprintf(resp, resp_len, "OK CMD=LEFT PWM=%d\r\n", pwm); return 1; }
        if(strcmp(val, "RIGHT") == 0){ motor_turn_right(pwm); if(resp) snprintf(resp, resp_len, "OK CMD=RIGHT PWM=%d\r\n", pwm); return 1; }
        if(strcmp(val, "STOP") == 0){ motor_stop(); if(resp) snprintf(resp, resp_len, "OK CMD=STOP\r\n"); return 1; }

        if(resp && resp_len) snprintf(resp, resp_len, "ERR unknown_cmd\r\n");
        return 0;
    }

    if(resp && resp_len) snprintf(resp, resp_len, "ERR unknown_key\r\n");
    return 0;
}