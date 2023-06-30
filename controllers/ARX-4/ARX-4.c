/*
 * File:          ARX-4.c
 * Date:          2021/5/3
 * Description:
 * Author:        DYH
 * Modifications:
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "include/arx.h"
#include "include/print.h"
#include "include/PID.h"
#include "include/display.h"
#include "include/motor.h"
#include "include/position_sensor.h"
#include "include/inertial_unit.h"
#include "include/accelerometer.h"
#include "include/gyro.h"
#include "include/keyboard.h"
#include "include/mouse.h"
#include "include/types.h"
#include "include/camera.h"
#include "include/robot.h"
#include "include/mathFuch.h"
#include "include/display.h"

#define TIME_STEP 1

Balence_robot arx;

pid_t pid_Knee_1_0,   pid_Knee_1_w,   pid_Knee_1_I;
pid_t pid_Knee_2_0,   pid_Knee_2_w,   pid_Knee_2_I;
pid_t pid_Knee_3_0,   pid_Knee_3_w,   pid_Knee_3_I;
pid_t pid_Knee_4_0,   pid_Knee_4_w,   pid_Knee_4_I;

pid_t pid_acce_1_v,  pid_Wheel_1_0,  pid_Wheel_1_x;
pid_t pid_acce_2_v,  pid_Wheel_2_0,  pid_Wheel_2_x;

//膝关节角度环
float kn_0_out_max  = 20.1, kn_0_out_imax = 4; 
float kp0[5], ki0[5], kd0[5];
//膝关节速度环
float kn_w_out_max = 40, kn_w_out_imax = 20;
float kpw[5], kiw[5] ,kdw[5];
 //膝关节电流环
float kn_I_out_max = 40, kn_I_out_imax = 4;
float kpI[5], kiI[5] ,kdI[5];


//本仿真只验证平衡时跳跃高度, 若想控制其运动, 可以加这一环PID, 输出为偏移角
float acce_v_out_max = 0.1,  acce_v_out_imax = 0.02 ; //输出为偏移角
float acce_kpv[5] , acce_kiv[5] ,  acce_kdv[5];
  
float acce_0_out_max = 50,  acce_0_out_imax = 50 ;  //输出为轮毂电机角速度
float acce_kp0[5],  acce_ki0[5] ,  acce_kd0[5];

float whl_w_out_max = 50 ,  whl_w_out_imax = 50 ;  //输出为轮毂电机扭矩
float whl_kpx[5] ,  whl_kix[5] ,  whl_kdx[5];

int status_sample[4] = {0, 10, 24, 38};
double IMU_offset[4] = {-0.03, -0.02, -0.01, 0};
double f[4] = {0.0011, -0.0705, 1.8689, -30.0};

double breakdown_time;//如果输出过大(如进入正反馈), 仿真环境会崩溃, 可以检测数据异常时间并及时关断所有执行器
double time_highPoint = 0;//收腿命令发出的时间点

//键盘结构体(本仿真不会用到)
WbMouseState mouseNow;
double u_last = 0;            //为了计算鼠标滑动的速度
double v_last = 0;

double time = 0;              //时实时间
int status = 0;               //腿长,用来插值

//扭簧角度, 可添加虚拟扭簧来
double fi_L[3];
double fi_R[3];

float knee_set_angle=-0.2;  //-0.1

//自己撸的数据可视化屏幕
double display_abs[10][100] = {0};
double display_ratio[10][100] = {0};
char display_value[10][20] = {0};//数值

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
//   //系统初始化
//   wb_robot_init();

//   //机器人初始化
//   robot_init();

//   arx.module = NORMAL;
//   pid_init();

//   knee_set_angle = 0;
//   printf("init_over\n");  


//   //wb_display_draw_text(arx.display.ID,"t", 100, 90);
//   //arx.motor[KNEE_R].angle = -0.2;
//  while (wb_robot_step(TIME_STEP) != -1) {
//     //上位机控制
//     reveive_velocity_z();
    
//     //速度角度检测, 数据存入全局变量中
//     velocity_detect();

//     angle_detect();//角度, IMU

//     gyro_detect(); //角速度
    
//     //判断状态, 初始化PID参数

//     if(arx.position_sensor[KNEE_RBM].position
//      - arx.position_sensor[KNEE_RFM].position
//      + arx.position_sensor[KNEE_LBM].position
//      - arx.position_sensor[KNEE_LFM].position > 4 && arx.module == JUMP_UP)
//        {
//          arx.module = JUMP_DOWN;
//          time_highPoint = time;
//          knee_set_angle = -0.4;
//          printf("//////////////turn-JUMP_DOWN////////////////\n");
//        }
//     if(arx.module == JUMP_DOWN && time >= time_highPoint + 0.004){
//         arx.module = NORMAL;
//         knee_set_angle = 0.1;
//         printf("///////////////turn-NORMAL///////////////////\n");
//     }
//     printf("time_highPoint = %.3f, time = %.3f", time_highPoint, time);

//     pid_init();

//     //PID算法计算的值填入电机结构体当中
//     joint_PID_cal();

//     //虚拟扭簧计算, 关节不加扭簧时禁用此函数
//     //joint_spring_cal();

//     //全身控制, 计算轮毂电机扭矩
//     if(time > 0)// && arx.module != JUMP_UP && arx.module != JUMP_DOWN)//站稳或者落下的时候
//     {
//       balance_PID_cal();
//     }

//     //调用webots函数为电机施加扭矩
//     torque_set();

//     //全部曲线显示
//     all_display();

//     //simulator step
//     wb_robot_step((unsigned int)TIME_STEP);
//     /* Process sensor data here */

//     /*
//      * Enter here functions to send actuator commands, like:
//      * wb_motor_set_position(my_actuator, 10.0);
//      */
//     time += (double)TIME_STEP / 1000;
//     /*
//      * Enter here functions to send actuator commands, like:
//      * wb_motor_set_position(my_actuator, 10.0);
//      */
//   };

//   /* Enter your cleanup code here */
//   /* This is necessary to cleanup webots resources */
//   wb_robot_cleanup();

  return 0;
}

//初始化函数
void robot_init(){
  arx.height_of_center = 0;
  arx.mass_body = 30;
  arx.mass_wheel = 0.5;
  arx.radius_of_wheel = 0.065;

  arx.camera_state = 0;

  arx.module = NORMAL;

  arx.displacement = 0;
  arx.displacement_last = 0;
  arx.velocity_target = 0;

  //最初的标定
  motor_init(0);
  position_sensor_init();
  imu_init();
  acce_init();
  gyro_init();
  camera_init();
  display_init();
  key_mouse_init();
}

void motor_init(double angle_set){
  //每加一个电机, (1)在宏定义中修改MOTOR_NUM 赋值, (2)增加代号的数量, (3)在这里写上name
  arx.motor[0].name = "RBM";
  arx.motor[1].name = "RFM";
  arx.motor[2].name = "LBM";
  arx.motor[3].name = "LFM";
  arx.motor[4].name = "wheel1"; //右
  arx.motor[5].name = "wheel2"; //左
  //虚拟扭簧
  arx.motor[6].name = "Spring1"; //右
  arx.motor[7].name = "Spring2"; //左

  int i;
  for( i = 0; i < MOTOR_NUM; i++){
    //获取电机ID
    arx.motor[i].ID = wb_robot_get_device(arx.motor[i].name);
    assert(arx.motor[i].ID);
    //获取最大扭矩
    arx.motor[i].MAX_TORQUE = wb_motor_get_max_torque(arx.motor[i].ID);
    //使能扭矩反馈
    int sampling_period;
    sampling_period = TIME_STEP;// wb_motor_get_torque_feedback_sampling_period(arx.motor[i].ID);
    wb_motor_enable_torque_feedback(arx.motor[i].ID, sampling_period);
    //归零
    arx.motor[i].torque = 0;
    arx.motor[i].omg = 0;
    arx.motor[i].angle = angle_set;
    printf("get motor %s succeed: %d\n", arx.motor[i].name, arx.motor[i].ID);
  }
};

void position_sensor_init(){
  arx.position_sensor[0].name = "RBS";
  arx.position_sensor[1].name = "RFS";
  arx.position_sensor[2].name = "LBS";
  arx.position_sensor[3].name = "LFS";
  arx.position_sensor[4].name = "Swheel1";
  arx.position_sensor[5].name = "Swheel2";
  arx.position_sensor[6].name = "SSpring1";
  arx.position_sensor[7].name = "SSpring2";

  int i;
  for( i = 0; i < MOTOR_NUM; i++){
  arx.position_sensor[i].ID = wb_robot_get_device(arx.position_sensor[i].name);
  assert(arx.position_sensor[i].ID);
  wb_position_sensor_enable(arx.position_sensor[i].ID, (int)TIME_STEP);
  printf("get position senser %s succeed: %d\n",arx.position_sensor[i].name, arx.position_sensor[i].ID);

  arx.position_sensor[i].position = 0;
  arx.position_sensor[i].position_last = 0;
  arx.position_sensor[i].w = 0;
  arx.position_sensor[i].w_last = 0;
  }
};

void imu_init(){
  arx.imu.name = "imu";
  arx.imu.ID = wb_robot_get_device(arx.imu.name);
  wb_inertial_unit_enable(arx.imu.ID, (int)TIME_STEP);
  arx.imu.angle_value[yaw] = 0;
  arx.imu.angle_value[pitch] = 0;
  arx.imu.angle_value[roll] = 0;
  printf("imu over!\n");
};

void acce_init(){
  arx.acce.accelerometer_name = "Acce";
  arx.acce.accelerometer_ID = wb_robot_get_device(arx.acce.accelerometer_name);
  wb_accelerometer_enable(arx.acce.accelerometer_ID, (int)TIME_STEP);
  printf("accelerometer over!\n");
}

void gyro_init(){
  arx.gyro.gyro_ID = wb_robot_get_device("gyro");
  wb_gyro_enable(arx.gyro.gyro_ID, (int)TIME_STEP);
  arx.gyro.gyro_value[yaw] = 0;
  arx.gyro.gyro_value[pitch] = 0;
  arx.gyro.gyro_value[roll] = 0;
  printf("gyro over!\n");
};

void camera_init(){
  arx.camera.camera_ID = wb_robot_get_device("camera");
  wb_camera_enable(arx.camera.camera_ID, (int)TIME_STEP);
  printf("camera over!\n");
};

void key_mouse_init(){
  wb_keyboard_enable((int)TIME_STEP);
  wb_mouse_enable((int)TIME_STEP);
  mouseNow.left = 0;
  mouseNow.middle = 0;
  mouseNow.right = 0;
  mouseNow.u = 0;
  mouseNow.v = 0;
  mouseNow.x = 0;
  mouseNow.y = 0;
  mouseNow.z = 0;
};

void display_init(){
  arx.display.ID = wb_robot_get_device("display");
  printf("display over!\n");
}

void pid_init(){
  kp0[NORMAL] = 6, ki0[NORMAL] = 0.4, kd0[NORMAL] = 1.5;
  kpw[NORMAL] = 4.8, kiw[NORMAL] = 0.1, kdw[NORMAL] = 0.4;
  //acce_kpv[NORMAL] = 0.002 , acce_kiv[NORMAL] = 0.001 ,  acce_kdv[NORMAL] = 0.001;
  acce_kp0[NORMAL] = 60,  acce_ki0[NORMAL] = 0 ,  acce_kd0[NORMAL] = 9;
  whl_kpx[NORMAL] = -0.00,  whl_kix[NORMAL] = 0,  whl_kdx[NORMAL] = -5;

  kp0[JUMP_UP] = 6, ki0[JUMP_UP] = 0.4, kd0[JUMP_UP] = 1.5;
  kpw[JUMP_UP] = 4.8, kiw[JUMP_UP] = 0.1, kdw[JUMP_UP] = 0.4;
  //acce_kpv[JUMP_UP] = 0.002 , acce_kiv[JUMP_UP] = 0.001 ,  acce_kdv[JUMP_UP] = 0.001;
  acce_kp0[JUMP_UP] = 47,  acce_ki0[JUMP_UP] = 0 ,  acce_kd0[JUMP_UP] = 9;
  whl_kpx[JUMP_UP] = -3.3,  whl_kpx[JUMP_UP] = -0.0075,  whl_kpx[JUMP_UP] = 0;

  kp0[JUMP_DOWN] = 6, ki0[JUMP_DOWN] = 0.4, kd0[JUMP_DOWN] = 1.5;
  kpw[JUMP_DOWN] = 4.8, kiw[JUMP_DOWN] = 0.1, kdw[JUMP_DOWN] = 0.4;
  //acce_kpv[JUMP_DOWN] = 0.002 , acce_kiv[JUMP_DOWN] = 0.001 ,  acce_kdv[JUMP_DOWN] = 0.001;
  acce_kp0[JUMP_DOWN] = 60,  acce_ki0[JUMP_DOWN] = 0 ,  acce_kd0[JUMP_DOWN] = 9;
  whl_kpx[JUMP_DOWN] = -5,  whl_kpx[JUMP_DOWN] = -0.0075,  whl_kpx[JUMP_DOWN] = 0;

  int i = arx.module;
  //命名规则1: PID目标值是什么,PID就叫什么
  PID_struct_init(&pid_Knee_1_0, kn_0_out_max, kn_0_out_imax, kp0[i], ki0[i], kd0[i]);
  PID_struct_init(&pid_Knee_2_0, kn_0_out_max, kn_0_out_imax, kp0[i], ki0[i], kd0[i]);
  PID_struct_init(&pid_Knee_3_0, kn_0_out_max, kn_0_out_imax, kp0[i], ki0[i], kd0[i]);
  PID_struct_init(&pid_Knee_4_0, kn_0_out_max, kn_0_out_imax, kp0[i], ki0[i], kd0[i]);
  
  PID_struct_init(&pid_Knee_1_w, kn_w_out_max, kn_w_out_imax, kpw[i], kiw[i], kdw[i]);
  PID_struct_init(&pid_Knee_2_w, kn_w_out_max, kn_w_out_imax, kpw[i], kiw[i], kdw[i]);
  PID_struct_init(&pid_Knee_3_w, kn_w_out_max, kn_w_out_imax, kpw[i], kiw[i], kdw[i]);
  PID_struct_init(&pid_Knee_4_w, kn_w_out_max, kn_w_out_imax, kpw[i], kiw[i], kdw[i]);

  //通过(两环就是力矩,三环就是角速度)驱动,使得0_acce收敛到设定角度
  PID_struct_init(&pid_acce_1_v, acce_v_out_max, acce_v_out_imax, acce_kpv[i], acce_kiv[i], acce_kdv[i]);
  PID_struct_init(&pid_acce_2_v, acce_v_out_max, acce_v_out_imax, acce_kpv[i], acce_kiv[i], acce_kdv[i]);

  //acce_angle + rest angle 为参考值, 计算实时角速度
  PID_struct_init(&pid_Wheel_1_0, acce_0_out_max, acce_0_out_imax, acce_kp0[i], acce_ki0[i], acce_kd0[i]);
  PID_struct_init(&pid_Wheel_2_0, acce_0_out_max, acce_0_out_imax, acce_kp0[i], acce_ki0[i], acce_kd0[i]);

  //以角速度为参考值, 机算时实力矩
  PID_struct_init(&pid_Wheel_1_x, whl_w_out_max, whl_w_out_imax, whl_kpx[i], whl_kix[i], whl_kdx[i]);
  PID_struct_init(&pid_Wheel_2_x, whl_w_out_max, whl_w_out_imax, whl_kpx[i], whl_kix[i], whl_kdx[i]);
}

//传感器检测函数
void angle_detect()
{
  assert(arx.imu.ID);
  arx.imu.angle_value[roll] = wb_inertial_unit_get_roll_pitch_yaw(arx.imu.ID)[roll];
  arx.imu.angle_value[pitch] = wb_inertial_unit_get_roll_pitch_yaw(arx.imu.ID)[pitch];
  arx.imu.angle_value[yaw] = wb_inertial_unit_get_roll_pitch_yaw(arx.imu.ID)[yaw];
  //IMU/GYRO信息
  // printf("0---0yaw: %.3f\t 0pitch: %.3f\t 0roll: %.3f\t", 
  // dgr(arx.imu.angle_value[yaw]), dgr(arx.imu.angle_value[pitch]), dgr(arx.imu.angle_value[roll]));
};

void gyro_detect()
{
  assert(arx.gyro.gyro_ID);
  arx.gyro.gyro_value[roll] = wb_gyro_get_values(arx.gyro.gyro_ID)[roll];
  arx.gyro.gyro_value[pitch] = wb_gyro_get_values(arx.gyro.gyro_ID)[pitch];
  arx.gyro.gyro_value[yaw] = wb_gyro_get_values(arx.gyro.gyro_ID)[yaw];
  //printf("w---Wyaw: %.3f\t Wpitch: %.3f\t Wroll: %.3f\t", 
  //dgr(arx.gyro.gyro_value[yaw]), dgr(arx.gyro.gyro_value[pitch]), dgr(arx.gyro.gyro_value[roll]));
};

void velocity_detect()//轮询所有电机传感器
{
  int i;
  for( i = 0; i < MOTOR_NUM; i++){
    assert(arx.position_sensor[i].ID);
    arx.position_sensor[i].position = wb_position_sensor_get_value(arx.position_sensor[i].ID);//GET THE POSITION
    arx.position_sensor[i].w = arx.position_sensor[i].position - arx.position_sensor[i].position_last;//CAL THE ANGULAR V
    arx.position_sensor[i].position_last = arx.position_sensor[i].position;
    
    arx.motor[i].torque_fb = wb_motor_get_torque_feedback(arx.motor[i].ID);

  }
  //需满足纯滚动条件
  double Z_x_middle = 
  (arx.position_sensor[WHEEL_R].position + arx.position_sensor[WHEEL_L].position) * arx.radius_of_wheel / 2;
                  
};

void joint_PID_cal(){
  double turn_angle = 0;      //未来转弯会用到, 加上转弯差动角
  arx.motor[KNEE_RBM].angle = knee_set_angle + 0;
  arx.motor[KNEE_RFM].angle = -knee_set_angle - 0;
  arx.motor[KNEE_LBM].angle = knee_set_angle + 0;
  arx.motor[KNEE_LFM].angle = -knee_set_angle - 0;//镜像转动

  arx.motor[KNEE_RBM].omg = pid_calc_arx(&pid_Knee_1_0, arx.position_sensor[KNEE_RBM].position, arx.motor[KNEE_RBM].angle);
  arx.motor[KNEE_RFM].omg = pid_calc_arx(&pid_Knee_2_0, arx.position_sensor[KNEE_RFM].position, arx.motor[KNEE_RFM].angle);
  arx.motor[KNEE_LBM].omg = pid_calc_arx(&pid_Knee_3_0, arx.position_sensor[KNEE_LBM].position, arx.motor[KNEE_LBM].angle);
  arx.motor[KNEE_LFM].omg = pid_calc_arx(&pid_Knee_4_0, arx.position_sensor[KNEE_LFM].position, arx.motor[KNEE_LFM].angle);
  //printf("KNEE: 0 set %.3f\tget %.3f\t err %.5f\t",pid_Knee_1_0.set[NOW], pid_Knee_1_0.get[NOW], pid_Knee_1_0.err[NOW]);

  arx.motor[KNEE_RBM].torque = pid_calc_arx(&pid_Knee_1_w, arx.position_sensor[KNEE_RBM].w, arx.motor[KNEE_RBM].omg);
  arx.motor[KNEE_RFM].torque = pid_calc_arx(&pid_Knee_2_w, arx.position_sensor[KNEE_RFM].w, arx.motor[KNEE_RFM].omg);
  arx.motor[KNEE_LBM].torque = pid_calc_arx(&pid_Knee_3_w, arx.position_sensor[KNEE_LBM].w, arx.motor[KNEE_LBM].omg);
  arx.motor[KNEE_LFM].torque = pid_calc_arx(&pid_Knee_4_w, arx.position_sensor[KNEE_LFM].w, arx.motor[KNEE_LFM].omg);
  //printf("KNEE: w set %.3f\tget %.3f\t err %.5f\t",pid_Knee_1_w.set[NOW], pid_Knee_1_w.get[NOW], pid_Knee_1_w.err[NOW]);  
}

void joint_spring_cal(){
  double active_rotate_L = arx.position_sensor[6].position;
  double active_rotate_R = arx.position_sensor[7].position;

  fi_L[NOW] = knee_angle_calc(0.162, 0.04282, 0.1638, 0.0795, rad(45) + 0.31 - active_rotate_L);//全局变量
  fi_R[NOW] = knee_angle_calc(0.162, 0.04282, 0.1638, 0.0795, rad(45) + 0.31 - active_rotate_R);
  printf("fi = %.2f\t %.2f\t", fi_L[NOW], fi_R[NOW]);

  double D_axle = 0;
  printf("D_axle = %.5f", D_axle);

  double omg_L = (fi_L[NOW] - fi_L[LAST]) / TIME_STEP * 1000;
  double omg_R = (fi_R[NOW] - fi_R[LAST]) / TIME_STEP * 1000;
  printf("omg = %.2f\t %.2f\n", omg_L, omg_R);

  double Kspring = 1;
  double Bdamping = 0;

  //120°为零点
  arx.motor[SPRING_L].torque = -Kspring * (fi_L[NOW] - 2) - Bdamping * omg_L;
  arx.motor[SPRING_R].torque = -Kspring * (fi_R[NOW] - 2) - Bdamping * omg_R;

  fi_L[LAST] = fi_L[NOW];
  fi_R[LAST] = fi_R[NOW];
}

/**
 * @description: 
 * 输入速度决定加速度角
 * 插值拟合决定静止偏置角
 * 叠加算得IMU目标角度
 * 通过PID给定一个轮毂电机w, 未到参考角度责令w增大, 即令0 > 零 * 
 * @param {*}
 * @return {*}
 */
void balance_PID_cal(){ 
  double theta = arx.imu.angle_value[pitch];        //pitch倾角 rad
  double omega = arx.gyro.gyro_value[pitch];        //pitch角速度 rad/s
  double Vr = (arx.position_sensor[WHEEL_L].w + arx.position_sensor[WHEEL_R].w) / 2 * 0.065; //轮子线速度 m/s(暂时不用)

  float k0p = 55;
  float k0d = 9;
  float kxp = -0.0075;
  float kxd = -4.5;

  //确定rest_angle
  //printf("status = %d\tmotor_angle = %f\t",status, arx.motor[KNEE_L].angle);
  //f[4]中已经乘了1000
  double rest_angle = 0;

  double balance_x1_pid_out = kxd * (arx.position_sensor[WHEEL_R].w * 0.065 - arx.velocity_target);
  double balance_x2_pid_out = kxd * (arx.position_sensor[WHEEL_L].w * 0.065 - arx.velocity_target);
  double balance_0_pid_out = k0p * (theta - rest_angle) + k0d * (omega - 0);
  arx.motor[WHEEL_R].torque = balance_x1_pid_out + balance_0_pid_out;
  arx.motor[WHEEL_L].torque = balance_x2_pid_out + balance_0_pid_out;

}

//上位机函数
void reveive_velocity_z(){
  int key = -1;
  
  mouseNow = wb_mouse_get_state();
  key = wb_keyboard_get_key();
  if (key != -1) {
    // update var according to 'key' value
    switch (key) {
      case 'C'://跳跃
        knee_set_angle = 1;
        arx.module = JUMP_UP;
        printf("\n//\nJUMP\n//\n");
        break;
      //切换屏幕
      case 'M':
        wb_display_attach_camera(arx.display.ID, arx.camera.camera_ID);
        arx.camera_state = 1;
        break;
      case 'N':
        wb_display_detach_camera(arx.display.ID);
        arx.camera_state = 0;
        break;
      case 'H'://抬升
        if(knee_set_angle < 1)
          knee_set_angle += 0.1;
        break;
      case 'J'://降落
        if(knee_set_angle > 0)
          knee_set_angle -= 0.1;
        break;
      default:
        break;
     }
   }
   else{
    arx.velocity_target = 0;
    arx.w_yaw_target = - 0.2 * (mouseNow.u - u_last) * 1000 / TIME_STEP;    //0.2是系数, 为了增大精度
   }
  u_last = mouseNow.u;
  //printf("key = %d\n",key);
}; 

//施加扭矩
void torque_set(){
  for(int i = 0; i < 4;i++){
    if(arx.motor[i].torque > 35)
      arx.motor[i].torque = 35;
    else if(arx.motor[i].torque < -35)
      arx.motor[i].torque = -35;
  }

  if(arx.motor[WHEEL_L].torque < -10)
    arx.motor[WHEEL_L].torque = -10;
  if(arx.motor[WHEEL_L].torque >  10)
    arx.motor[WHEEL_L].torque =  10;
  if(arx.motor[WHEEL_R].torque < -10)
    arx.motor[WHEEL_R].torque = -10;
  if(arx.motor[WHEEL_R].torque >  10)
    arx.motor[WHEEL_R].torque =  10;


    wb_motor_set_torque( arx.motor[KNEE_RBM].ID, 1 * arx.motor[KNEE_RBM].torque);
    wb_motor_set_torque( arx.motor[KNEE_RFM].ID, 1 * arx.motor[KNEE_RFM].torque);
    wb_motor_set_torque( arx.motor[KNEE_LBM].ID, 1 * arx.motor[KNEE_LBM].torque);
    wb_motor_set_torque( arx.motor[KNEE_LFM].ID, 1 * arx.motor[KNEE_LFM].torque);

  printf("module: %d",arx.module);

  printf("torque feedback: %.3f\t %.3f\t %.3f\t %.3f\n", wb_motor_get_torque_feedback(arx.motor[0].ID),
                                                         wb_motor_get_torque_feedback(arx.motor[1].ID),
                                                         wb_motor_get_torque_feedback(arx.motor[2].ID),
                                                         wb_motor_get_torque_feedback(arx.motor[3].ID));

  if(time > 0 )//&& arx.module != JUMP_UP && arx.module != JUMP_DOWN)
  {
    wb_motor_set_torque(arx.motor[WHEEL_L].ID, -1 * arx.motor[WHEEL_L].torque);
    wb_motor_set_torque(arx.motor[WHEEL_R].ID,  1 * arx.motor[WHEEL_R].torque);
  }
}

/**
 * @description: 数据可视化, 最新数据赋值, 展示, 全部数据后退一格->最新数据赋值
 * @param {variable} 为最新数据, 导入到全局变量display_1中, 每一个display对应一个全局变量(也可能是两个)
 * @return {*}
 */
void display(double variable, double MAX, double MIN, int color, int order, char* value){
  display_abs[order][100] = variable;
  int x = 0;
  double y0;
  for(x = 0; x <= 100; x++){
     display_abs[order][x] = display_abs[order][x+1]; //左面吸取右面的数值,图像时实左移
  }

  //标明x轴
  //printf("MIN %f\n",MIN);
  if(MIN < 0)
    y0 = 108 * MAX / (MAX - MIN);
  else
    y0 = 107;
  wb_display_set_color(arx.display.ID, 1);
  wb_display_draw_line(arx.display.ID, 1, (int)y0, 128, (int)y0);

  //标明数据曲线
  wb_display_set_color(arx.display.ID, color);
  for(x = 0; x <= 100; x++){
    display_ratio[order][x] = 108 * (MAX - display_abs[order][x]) / (MAX - MIN);
    wb_display_draw_pixel(arx.display.ID, x, (int)display_ratio[order][x]);
  }

  //标明数据
 // wb_display_set_color(arx.display.ID, color);
  // wb_display_set_font(arx.display.ID, "楷体", 6, 1);
 // wb_display_draw_text(arx.display.ID, &value, 100, (int)display_ratio[order][100]);
}

void all_display()
{
  //if(arx.camera_state != 0)
  if(1)  {
    //clean the scream();
    wb_display_set_color(arx.display.ID, 0xffffee);
    wb_display_fill_rectangle(arx.display.ID, 0, 0, 128, 128);
    
    double angle_max = 1;
    double angle_min = -0.4;
    
    //绿色:设置角度
    //itoa((int)(knee_set_angle * 1000), display_value[0], 10);
    
    // display(arx.motor[1].angle,
    // angle_max, angle_min, 0x00ff00, 0, "angle");//设置角度
    
    printf("设置角度:\t %.2f\t %.2f\t %.2f\t %.2f\n", 
    arx.motor[0].angle, arx.motor[1].angle, 
    arx.motor[2].angle, arx.motor[3].angle);
    // //arx.motor[KNEE_RBM].angle

    //粉色: 观测角度
    itoa((int)(arx.position_sensor[0].position * 1000), display_value[1], 10);
    
   // display(arx.position_sensor[1].position,
   // angle_max, angle_min, 0xff00ff, 1, display_value[1]);//观测角度
    
    printf("观测角度:\t %.2f\t %.2f\t %.2f\t %.2f\n", 
    arx.position_sensor[0].position, arx.position_sensor[1].position, 
    arx.position_sensor[2].position, arx.position_sensor[3].position);
    
    
    double omg_max =  5;
    double omg_min = -5;
    //蓝色: 设置角速度
    itoa((int)(arx.motor[KNEE_RBM].omg * 1000), display_value[2], 10);
   // display(arx.motor[KNEE_RBM].omg, omg_max, omg_min, 0x0000ff, 2, display_value[2]);//设置角速度
    
    printf("设置角速度:\t %.2f\t %.2f\t %.2f\t %.2f\n",
    arx.motor[0].omg, arx.motor[1].omg,
    arx.motor[2].omg, arx.motor[3].omg);
    
    // printf("误差：%.2f\t p贡献：%.2f\t I贡献：%.2f\t D贡献： %.2f\n", 
    // pid_Knee_1_w.pout,
    // pid_Knee_1_w.iout,
    // pid_Knee_1_w.dout);
 
    //printf("角速度: %f\n",arx.motor[KNEE_RBM].omg);
    //红色: 真实角速度
    itoa((int)(arx.position_sensor[KNEE_RBM].w * 1000), display_value[3], 10);
   // display(arx.position_sensor[KNEE_RBM].w, 0.1, -0.1, 0xff0000, 3, display_value[3]);//真实角速度
    printf("真实角速度:\t %.5f\t %.5f\t %.5f\t %.5f\n", 
    arx.position_sensor[0].w, arx.position_sensor[1].w, 
    arx.position_sensor[2].w, arx.position_sensor[3].w);
    
    printf("误差：%.2f\t p贡献：%.2f\t I贡献：%.2f\t D贡献： %.2f\n", 
    arx.motor[0].omg - arx.position_sensor[0].position, 
    pid_Knee_2_w.pout,
    pid_Knee_2_w.iout,
    pid_Knee_2_w.dout);

    
    printf("knee torque: %.1f\t %.1f\t %.1f\t %.1f\n",arx.motor[0].torque, 
                                                      arx.motor[1].torque,
                                                      arx.motor[2].torque,
                                                      arx.motor[3].torque);
                                                      
    printf("wheel t = %.3f\t, %.3f\n",arx.motor[WHEEL_R].torque, arx.motor[WHEEL_L].torque);
    
        display(arx.position_sensor[WHEEL_L].w * 0.065, 0.05, -0.05, 0xff0000, 0, display_value[0]);
    printf("轮轴运动速度[红]: %.3f\n",arx.position_sensor[WHEEL_L].w * 0.065);
        display(arx.imu.angle_value[pitch],                    0.1, -0.1, 0x00ff00, 1, display_value[1]);
    printf("车身倾角[绿]: %.3f\n",    arx.imu.angle_value[pitch]);
        display(arx.motor[WHEEL_L].torque,                     5, -5, 0x0000ff, 2, display_value[2]);
    printf("轮毂电机力矩蓝]: %.3f\n",arx.motor[WHEEL_L].torque);
  }
}