/*
 * File:          Lau_Bot.c
 * Date: 6-27
 * Description: 跳跃轮腿仿真
 * Author: 刘航
 * Modifications: 
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <webots/robot.h>
#include <webots/display.h>
#include <webots/types.h>

#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/keyboard.h>

#include "bsp_sensor.h"
#include "bsp_motor.h"
#include "app_chassis.h"




/*
 * You may want to add macros here.
 */
#define TIME_STEP 1

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  imu_init();
  gyro_init();
  motor_init(0);
  position_sensor_init();
  remote_keyboard_init();
  
  

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   printf("robot has been inited\n");


    

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    printf("\n");
    printf("robot thread has started %f second\n",wb_robot_get_time());
    
    remote_keyboard_control();
    imu_angle_detect();
    velocity_detect();
    //gyro_speed_detect();
 

    chassis_init();
    chassis_task();

    // wb_motor_set_position(motor[WHEEL_L].ID, INFINITY);
    // wb_motor_set_velocity(motor[WHEEL_L].ID, -imu.angle_value[pitch] *30);  // 1 rotation per second
    // wb_motor_set_position(motor[WHEEL_R].ID, INFINITY);
    // wb_motor_set_velocity(motor[WHEEL_R].ID, imu.angle_value[pitch] *30);  // 1 rotation per second

    // wb_motor_set_torque(motor[WHEEL_L].ID, imu.angle_value[pitch] *10);  // 1 rotation per second
    // wb_motor_set_torque(motor[WHEEL_R].ID, imu.angle_value[pitch] *10);  // 1 rotation per second
    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
