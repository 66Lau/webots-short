#ifndef BSP_SENSOR_H
#define BSP_SENSOR_H


#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
#include <webots/types.h>
#include <webots/robot.h>
#include <stdio.h>


#define roll 0
#define pitch 1
#define yaw 2



//imu参数（角度）
typedef struct imu_feature
{
  WbDeviceTag ID;
  const char *name;
  double angle_value[3]; //rad
}IMU;

//accelerometer参数（角加速度）
typedef struct accelerometer_feature
{
  WbDeviceTag accelerometer_ID;
  const char *accelerometer_name;
  double accelerometer_value[3];
}ACCE;

//gyro参数(角速度)
typedef struct gyro_feature
{
  WbDeviceTag ID;
  const char *name;
  double gyro_value[3];
}GYRO;

extern IMU imu;
extern GYRO gyro;

void imu_init();
void imu_angle_detect();
void gyro_init();
void gyro_speed_detect();

#endif