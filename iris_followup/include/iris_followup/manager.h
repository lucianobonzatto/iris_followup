#ifndef MANAGER_H
#define MANAGER_H

#include "general.h"
#include "state_machine.h"
#include "follow_controller.h"
#include "land_controller.h"
#include "drone_control.h"

class Manager
{
public:
    Manager();
    ~Manager();

    void Init(DroneControl *drone_control,
              double joyLinearVelocity,
              double joyAngularVelocity);

    void print_parameters();
    void update();

    // void set_pose(geometry_msgs::PoseStamped newPose);
    void set_odom(nav_msgs::Odometry newOdom);
    void set_joy(sensor_msgs::Joy newJoy);
    void set_parameters(std_msgs::Float32MultiArray newParameters);

private:
    sensor_msgs::Joy joy;
    nav_msgs::Odometry odom;
    std_msgs::Float32MultiArray parameters;

    // geometry_msgs::PoseStamped pose;
    // geometry_msgs::PoseStamped lastPose;
    Speed droneVel;

    DroneControl *drone_connection;
    State_Machine state_machine;
    Follow_Controller follow_controller;
    Land_Controller land_controller;

    void STOPPED_action();
    void TAKE_OFF_action();
    void LAND_action();
    void JOY_CONTROL_action();
    void LAND_CONTROL_action();
    void FOLLOW_CONTROL_action();
    void send_velocity(double x_linear, double y_linear, double z_linear, double angular);
    void send_velocity(geometry_msgs::Twist velocity);

    // joy parameters
    ros::Time joy_last_timestamp;
    double joy_linear_velocity;
    double joy_angular_velocity;
};

#endif // MANAGER_H
