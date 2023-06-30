#include "app_chassis.h"
/**
 * // 玄学问题：worldinfo下的
 * basictimestep 为32时，前进速度较大时会出现轮胎下陷，同时关节电机容易抖动
 * basictimestep 为16时,均较为正常，但跳跃高度明显变低
 * 8时无轮胎下陷问题但会出现腿部劈叉
 * basictimestep 文档描述
 * The basicTimeStep field defines the duration of the simulation step executed by Webots. 
 * It is a floating point value expressed in milliseconds where the minimum value is 1. 
 * Setting this field to a high value will accelerate the simulation, but will decrease the accuracy and the stability, 
 * especially for physics computations and collision detection. 
 * It is usually recommended to tune this value in order to find a suitable speed/accuracy trade-off.
 * 
 * 
 */


PidTypeDef chassis_Balance_pid;
PidTypeDef chassis_roll_pid;
fp32 balance_pid[3] = {220, 0, 3};
fp32 height_pid[3] = {0.5, 1, 0};
// 两组关节角度（一组两个关节）
fp32 set_joint_motor_angle_right[2] = {0, 0};
fp32 set_joint_motor_angle_left[2] = {0, 0};

double set_x = 0;
double set_y = 0.25;
double diff_vel = 4;

//暂时debug用
int debug_jump2 = 0;

void chassis_init()
{
    chassis_wheel_init();
    chassis_legs_init();


    printf("chassis has inited\n");
}

void chassis_wheel_init()
{
PID_Init(&chassis_Balance_pid, PID_POSITION, balance_pid, (fp32) 100, (fp32) 100);
PID_Init(&chassis_roll_pid, PID_POSITION, height_pid, (fp32) 100, (fp32) 100);


}

void chassis_legs_init()
{
    //框架，待整合

}


void chassis_task()
{



    PID_Calc(&chassis_Balance_pid, (fp32)imu.angle_value[pitch], (fp32)0);
    PID_Calc(&chassis_roll_pid, (fp32)imu.angle_value[roll], (fp32)0);
    wb_motor_set_position(motor[WHEEL_L].ID, INFINITY);
    wb_motor_set_position(motor[WHEEL_R].ID, INFINITY);
    // 动力轮平衡环+转速环
    wb_motor_set_velocity(motor[WHEEL_L].ID, chassis_Balance_pid.out - diff_vel*keyboard.velocity_turnleft  + diff_vel*keyboard.velocity_turnright);
    wb_motor_set_velocity(motor[WHEEL_R].ID, -(chassis_Balance_pid.out - diff_vel*keyboard.velocity_turnright + diff_vel*keyboard.velocity_turnleft));
    // printf("wheel_left_speed%f\n",wb_motor_get_velocity(motor[WHEEL_R].ID));

    // roll轴平衡控制
    set_x = -0.03 * keyboard.velocity_forward + 0.03 * keyboard.velocity_backward;
    set_y = set_y + keyboard.vertical_up*0.01 - keyboard.vertical_down*0.01;
    double set_y_right = set_y - chassis_roll_pid.out;
    double set_y_left = set_y + chassis_roll_pid.out;
    // 腿长限幅（防止干涉和解算极限）
    if (set_y_right<0.23)
        set_y_right = 0.23;
    if (set_y_right>0.35)
        set_y_right = 0.35;

    if (set_y_left<0.23)
        set_y_left = 0.23;
    if (set_y_left>0.35)
        set_y_left = 0.35;

    //跳跃demo，仅做展示，并不完善，待加入轨迹跟踪
    if (keyboard.jump == 1)
    {
        printf("=========== jump ==========\n");
        if(debug_jump2 == 0)
        {
            set_y = 0.45; debug_jump2 = 1;
        }
        else if (debug_jump2 == 1)
        {
            set_y = 0.23; debug_jump2 = 0;
        }


    }

    // 逆运动学解算
    legs_Inversekinematics(set_joint_motor_angle_right, set_x, set_y_right, 0.15, 0.15, 0.288);
    legs_Inversekinematics(set_joint_motor_angle_left, set_x, set_y_left, 0.15, 0.15, 0.288);
    // 五连杆关节位置控制
    wb_motor_set_position(motor[RBM].ID, set_joint_motor_angle_right[1]);
    wb_motor_set_position(motor[RFM].ID, -set_joint_motor_angle_right[0]);
    wb_motor_set_position(motor[LBM].ID, set_joint_motor_angle_left[1]);
    wb_motor_set_position(motor[LFM].ID, -set_joint_motor_angle_left[0]);


}

/**
  * @brief          轮腿机器人五连杆逆解
  * @param[in]      x:机器人五连杆末端相对机器人底盘中点的平行于底盘的距离，
  *                 y:机器人五连杆末端相对机器人底盘中点的重力方向距离(高度)
  * @retval         front_motor_angle:机器人五连杆前关节内角，behind_motor_angle:机器人五连杆后关节内角
  */
void legs_Inversekinematics(fp32 *input_set_joint_motor_angle,double x, double y, double len_chassis, double len_leg1, double len_leg2)
{

    x = x + len_chassis/2; //坐标系转换，从底盘中心转换至底盘后关节作为坐标系原点

    double len_behind2end;
    double len_front2end;

    double behind_theta_1;
    double behind_theta_2;

    double front_theta_1;
    double front_theta_2;

    double front_theta_sum;
    double behind_theta_sum;

    len_behind2end = sqrt(pow(x,2)+pow(y,2));
    len_front2end = sqrt(pow(len_chassis-x,2)+pow(y,2));
        //printf("len_behind2end=%f\n",len_behind2end);
        //printf("len_front2end=%f\n",len_front2end);
    
    double slides2angle(double nei_slide1, double nei_slide2, double opp_slide)
    {
        double angle_mem;
        double angle_den;
        double angle;
        
        angle_mem = pow(nei_slide1,2) + pow(nei_slide2,2) - pow(opp_slide,2);
        angle_den = 2 * nei_slide1 * nei_slide2;
        angle = acos(angle_mem/angle_den);
        return angle;
    }
    behind_theta_1 = slides2angle(len_leg1, len_behind2end, len_leg2);
        //printf("behind_theta_1=%f\n",behind_theta_1);
    behind_theta_2 = slides2angle(len_chassis, len_behind2end, len_front2end);

    front_theta_1 = slides2angle(len_leg1, len_front2end, len_leg2);
        //printf("front_theta_1=%f\n",front_theta_1);
    front_theta_2 = slides2angle(len_chassis, len_front2end, len_behind2end);
        //printf("front_theta_2=%f\n",front_theta_2);

    front_theta_sum = front_theta_1 + front_theta_2;
    behind_theta_sum = behind_theta_1 + behind_theta_2;
    input_set_joint_motor_angle[0] = M_PI - front_theta_sum;
    input_set_joint_motor_angle[1] = M_PI - behind_theta_sum;

    //printf("前关节角度计算值%f\n",input_set_joint_motor_angle[0]);
    //printf("后关节角度计算值%f\n",input_set_joint_motor_angle[1]);
    
    //return input_set_joint_motor_angle;
}
