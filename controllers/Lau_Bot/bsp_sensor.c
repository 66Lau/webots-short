#include "bsp_sensor.h"

#define TIME_STEP 1

IMU imu;

void imu_init()
{
  imu.name = "imu";
  imu.ID = wb_robot_get_device(imu.name);
  wb_inertial_unit_enable(imu.ID, (int)TIME_STEP);
  imu.angle_value[yaw] = 0;
  imu.angle_value[pitch] = 0;
  imu.angle_value[roll] = 0;
  printf("imu over!\n");
};

void imu_angle_detect()
{

  imu.angle_value[roll] = wb_inertial_unit_get_roll_pitch_yaw(imu.ID)[roll];
  imu.angle_value[pitch] = wb_inertial_unit_get_roll_pitch_yaw(imu.ID)[pitch];
  imu.angle_value[yaw] = wb_inertial_unit_get_roll_pitch_yaw(imu.ID)[yaw];
  //IMU/GYRO信息
  printf("0---0yaw: %.3f\t 0pitch: %.3f\t 0roll: %.3f\t\n", 
  (imu.angle_value[yaw]), (imu.angle_value[pitch]), (imu.angle_value[roll]));
};