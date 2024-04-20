#ifndef __GIMBAL__
#define __GIMBAL__

#include "stm32f4xx.h"
#include "main.h"

#define yaw_Min -90.0f
#define yaw_Max 90.0f

#define pitch_Min -30.0f
#define pitch_Max 3.0f

/* ������֮���ֵ */
#define YawMotorCenter 5150
// #define YawMotorLeft   -1400
// #define YawMotorRight  1752

#define PitchMotorCenter 4200
#define PitchMotorUp 8000
#define PitchMotorDown 100


typedef struct SecOrdFilter{
float k1;
float k2;
float k3;
float k4;
float k5;
float xin;
float x1;
float x2;
float yout;
float y1;
float y2;
} vector;
 


extern float yaw_RC_sensitivity;                  // 遥控模式yaw灵敏度_imu反馈
extern float pitch_RC_sensitivity;                // 遥控模式pitch灵敏度 imu反馈
extern float yaw_RC_sensitivity_motor;            // 遥控模式yaw灵敏度_电机反馈
extern float pitch_RC_sensitivity_motor;          // 遥控模式pitch灵敏度 电机反馈
extern float yaw_PC_Mouse_sensitivity;            // 键鼠模式yaw鼠标灵敏度 imu反馈
extern float pitch_PC_Mouse_sensitivity;          // 键鼠模式pitch鼠标灵敏度 imu反馈
extern float yaw_PC_Mouse_sensitivity_motor;      // 键鼠模式yaw鼠标灵敏度 电机反馈
extern float pitch_PC_Mouse_sensitivity_motor;    // 键鼠模式pitch鼠标灵敏度 电机反馈
extern float yaw_PC_Keyboard_sensitivity;         // 键鼠模式yaw键盘灵敏度 imu反馈
extern float pitch_PC_Keyboard_sensitivity;       // 键鼠模式pitch键盘灵敏度 imu反馈
extern float yaw_PC_Keyboard_sensitivity_motor;   // 键鼠模式yaw键盘灵敏度 电机反馈
extern float pitch_PC_Keyboard_sensitivity_motor; // 键鼠模式pitch键盘灵敏度 电机反馈


//extern SawToothWave wave;
void GimbalTask(void const *pvParameters);

#endif //__GIMBAL__
