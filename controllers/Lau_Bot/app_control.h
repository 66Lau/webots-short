#ifndef APP_CONTROL_H
#define APP_CONTROL_H
#define _USE_MATH_DEFINES

#include "math_pid.h"
#include "bsp_motor.h"
#include "bsp_sensor.h"
#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/types.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>

#include <stdio.h>
#include <math.h>

typedef struct keyboard_feature
{
  int key_get;
  int mode;
  int velocity_forward;
  int velocity_backward;
  int velocity_turnright;
  int velocity_turnleft;
  int vertical_up;
  int vertical_down;
  int jump;
}KEYBOARD;

extern KEYBOARD keyboard;

void remote_keyboard_init();
void remote_keyboard_control();

#endif