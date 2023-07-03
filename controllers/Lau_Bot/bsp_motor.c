#include "bsp_motor.h"

MOTOR motor[6];
POSITION_SENSOR position_sensor[6];
double robbot_speed_forward;

void motor_init(double angle_set){
  //每加一个电机, (1)在宏定义中修改MOTOR_NUM 赋值, (2)增加代号的数量, (3)在这里写上name
  motor[0].name = "RBM";
  motor[1].name = "RFM";
  motor[2].name = "LBM";
  motor[3].name = "LFM";
  motor[4].name = "wheel1"; //右
  motor[5].name = "wheel2"; //左
  //虚拟扭簧
//   motor[6].name = "Spring1"; //右
//   motor[7].name = "Spring2"; //左

  int i;
  for( i = 0; i < MOTOR_NUM; i++){
    //获取电机ID
    motor[i].ID = wb_robot_get_device(motor[i].name);
    // assert(motor[i].ID);
    //获取最大扭矩
    motor[i].MAX_TORQUE = wb_motor_get_max_torque(motor[i].ID);
    //使能扭矩反馈
    int sampling_period;
    sampling_period = TIME_STEP;// wb_motor_get_torque_feedback_sampling_period(motor[i].ID);
    wb_motor_enable_torque_feedback(motor[i].ID, sampling_period);
    //归零
    motor[i].torque = 0;
    motor[i].omg = 0;
    motor[i].angle = angle_set;
    printf("get motor %s succeed: %d\n", motor[i].name, motor[i].ID);
  }
};

void position_sensor_init(){
  position_sensor[0].name = "RBS";
  position_sensor[1].name = "RFS";
  position_sensor[2].name = "LBS";
  position_sensor[3].name = "LFS";
  position_sensor[4].name = "Swheel1";
  position_sensor[5].name = "Swheel2";
//   position_sensor[6].name = "SSpring1";
//   position_sensor[7].name = "SSpring2";

  int i;
  for( i = 0; i < MOTOR_NUM; i++){
  position_sensor[i].ID = wb_robot_get_device(position_sensor[i].name);
  //assert(position_sensor[i].ID);
  wb_position_sensor_enable(position_sensor[i].ID, (int)TIME_STEP);
  printf("get position senser %s succeed: %d\n",position_sensor[i].name, position_sensor[i].ID);

  position_sensor[i].position = 0;
  position_sensor[i].position_last = 0;
  position_sensor[i].w = 0;
  position_sensor[i].w_last = 0;
  }
};

void velocity_detect()//轮询所有电机传感器
{
  int i;
  for( i = 0; i < MOTOR_NUM; i++){
    //assert(position_sensor[i].ID);
    position_sensor[i].position = wb_position_sensor_get_value(position_sensor[i].ID);//GET THE POSITION
    position_sensor[i].w = position_sensor[i].position - position_sensor[i].position_last;//CAL THE ANGULAR V
    position_sensor[i].position_last = position_sensor[i].position;
    //printf("motor:%s position:%f\n",position_sensor[i].name, position_sensor[i].position);
    motor[i].torque_fb = wb_motor_get_torque_feedback(motor[i].ID);
    //printf("motor_speed = %f\n",position_sensor[i].w);

  }
  //需满足纯滚动条件
//   double Z_x_middle = 
//   (position_sensor[WHEEL_R].position + position_sensor[WHEEL_L].position) * radius_of_wheel / 2;
  robbot_speed_forward = (-position_sensor[WHEEL_R].w + position_sensor[WHEEL_L].w)/2;
  printf("robbot_speed_forward = %f\n",robbot_speed_forward);      
};

