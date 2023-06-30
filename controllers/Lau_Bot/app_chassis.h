#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H
#define _USE_MATH_DEFINES

#include "math_pid.h"
#include "bsp_motor.h"
#include "bsp_sensor.h"
#include "app_control.h"
#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/types.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <math.h>


//平衡环：输入：pitch轴与中垂线夹角
extern PidTypeDef chassis_Balance_pid;

void chassis_init();
void chassis_wheel_init();
void chassis_legs_init();
void chassis_task();
void legs_Inversekinematics(fp32 *input_set_joint_motor_angle,double x, double y, double len_chassis, double len_leg1, double len_leg2);

#endif