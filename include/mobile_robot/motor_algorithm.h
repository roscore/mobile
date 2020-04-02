#ifndef MOTOR_ALGORITHM_H
#define MOTOR_ALGORITHM_H 

//pin information
#define motor_DIR1 26
#define motor_PWM1 12
#define motor_EN1A 6
#define motor_EN1B 5

#define motor_DIR2 19
#define motor_PWM2 13
#define motor_EN2A 22 
#define motor_EN2B 27

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <pigpiod_if2.h>

//ros_communication_message type
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

//custom header
#include "ssoni_mobile_msgs/motor_cmd.h"

void Motor_Controller(int motor_num, int direction, int pwm);
void Accel_Controller(int motor_num, int direction, int desired_pwm);

int pinum;

int PWM_range;
int PWM_frequency;
int PWM_limit;

int current_PWM;
bool current_Direction;
int acceleration;

double pwm_value_motor;
double result_rpm;

bool direction;
bool check_position_control;
bool onoff;

int encoder_pulse1;
int encoder_pulse2;

int encoder_pulse_position1;
int encoder_pulse_position2;

int position_max_rpm;
bool check_position;

double acceleration_value;

void speed_controller(int desired_speed);
double position_controller(int desired_angle, int max_rpm);

int encoder_pulse_per_rotation_;
int control_freqency_;
int channel_;

double p_gain_position_;
double p_gain_speed_;

double speed_static_encoder_pulse_;
double speed_error_;
double speed_control_;

double position_static_encoder_pulse_;
double position_error_;
double position_control_;

//ros communication

ros::Subscriber left_motor_joy_sub;
ros::Subscriber right_motor_joy_sub;

ros::Subscriber left_motor_operator_sub;
ros::Subscriber right_motor_oprator_sub;

ros::Subscriber mode_select_sub;

//function
void Initialize();

void Interrupt1A_Falling(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt1A_Rising(int pi, unsigned user_gpio, unsigned level, uint32_t tick);

void Interrupt1B_Falling(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt1B_Rising(int pi, unsigned user_gpio, unsigned level, uint32_t tick);

void Interrupt2A_Falling(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2A_Rising(int pi, unsigned user_gpio, unsigned level, uint32_t tick);

void Interrupt2B_Falling(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2B_Rising(int pi, unsigned user_gpio, unsigned level, uint32_t tick);

void Interrupt_Setting();

int Limit_Function(int pwm);

void left_motor_joy_callback(const ssoni_mobile_msgs::motor_cmd::ConstPtr& msg);
void right_motor_joy_callback(const ssoni_mobile_msgs::motor_cmd::ConstPtr& msg);

void left_motor_operator_callback(const ssoni_mobile_msgs::motor_cmd::ConstPtr& msg);
void right_motor_operator_callback(const ssoni_mobile_msgs::motor_cmd::ConstPtr& msg); 

void mode_select_callback(const std_msgs::Bool::ConstPtr& msg);
bool joy_flag;

#endif // MOTOR_ALGORITHM_H
