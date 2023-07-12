#ifndef FOLLOW_CONTROLLER_H
#define FOLLOW_CONTROLLER_H

#include "general.h"
#include "tello_controllers.h"

class Follow_Controller
{
public:
    Follow_Controller();
    ~Follow_Controller();

    void print_parameters();
    geometry_msgs::Twist get_velocity(geometry_msgs::Pose pose);    
    void update_parameters(float *newParameters);

private:
    geometry_msgs::Twist velocity;

    ros::Time track_last_timestamp;
    
    TelloPDController pdController;
    
    double x_reference;
    double y_reference;
    double z_reference;
    double yaw_reference;
};

#endif // FOLLOW_CONTROLLER_H
