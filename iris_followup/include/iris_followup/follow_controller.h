#ifndef FOLLOW_CONTROLLER_H
#define FOLLOW_CONTROLLER_H

#include "general.h"
#include "controllers.cpp"

class Follow_Controller
{
public:
    Follow_Controller();
    ~Follow_Controller();

    void print_parameters();
    geometry_msgs::Twist get_velocity();
    void update_parameters(float *newParameters);

private:
    geometry_msgs::Twist velocity;

    ros::Time track_last_timestamp;

    // PID x_controller;
    // PID y_controller;
    // PID z_controller;
    // PID yaw_controller;
    
    double x_reference;
    double y_reference;
    double z_reference;
    double yaw_reference;
};

#endif // FOLLOW_CONTROLLER_H
