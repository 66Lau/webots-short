#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H


#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/types.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <stdio.h>

#define MOTOR_NUM 6
#define TIME_STEP 1

#define RBM 0
#define RFM 1
#define LBM 2
#define LFM 3
#define WHEEL_R 4
#define WHEEL_L 5


//电机结构体
typedef struct motor_feature
{
  WbDeviceTag ID;
  const char *name;
  double MAX_TORQUE;
  double torque;
  double torque_fb;//力矩反馈
  double torque_tgt;//期望力矩
  double omg;
  double angle;
  double angle_last;
}MOTOR;

//编码器结构体
typedef struct position_sensor_feature
{
  WbDeviceTag ID;
  const char *name;
  double resolusion;
  double position;//rad
  double position_last;
  double w;//rad/s
  double w_last;
}POSITION_SENSOR;

extern MOTOR motor[6];
extern POSITION_SENSOR position_sensor[6];
extern double robbot_speed_forward;

void motor_init(double angle_set);
void position_sensor_init();
void velocity_detect();

#endif