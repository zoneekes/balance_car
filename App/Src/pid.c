#include "pid.h"
#include "inv_mpu.h"
#include "mpu6050.h"
#include <sys/_intsup.h>

#define MED_OFFSET 0.50
int encoder_left = 0;//左边电机的速度
int encoder_right = 0;//右边电机的速度

float pitch=0, roll=0, yaw=0;//姿态角
short gyro_X, gyro_Y, gyro_Z;//陀螺仪角速度
short acc_X, acc_Y, acc_Z;//加速度计数据

float vertical_kp= 200, vertical_kd = 2.04;//直立环PD控制器参数(kp-0~1000, kd-0~10)
float speed_kp = 0.6, speed_ki = 0.003;//速度环PI控制器参数(kp-0~1, ki-kp/200)
float steering_kp, steering_kd;//转向环PD控制器参数
uint8_t flag_stop;//停止标志位

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

//PID控制器参数中间变量
int vertical_out, speed_out, steering_out,motor_left_out, motor_right_out;
int target_speed ;//目标速度
int target_angle ;//目标角度

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
    int error = 0;
    static float error_lowout_last,encoder_S;
    static float a=0.7;
    int error_lowout;
    //计算偏差值
    error = (encoder_L + encoder_R) - med;
    //低通滤波
    error_lowout = a * error + (1 - a) * error_lowout_last;
    error_lowout_last = error_lowout;
    //积分环节
    encoder_S += error_lowout;
    //积分限幅
    if(encoder_S > 1000) encoder_S = 10000;
    else if(encoder_S < -1000) encoder_S = -10000;

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