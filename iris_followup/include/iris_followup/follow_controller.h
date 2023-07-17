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
    geometry_msgs::Twist get_velocity(geometry_msgs::PoseStamped poseStamped, Speed iris_vel);    
    void update_parameters(float *newParameters);

private:
    ros::Time track_last_timestamp;

    // TelloPDController pdController;
    TelloCascadePDPIController pdController;

    Pose setpoint;
};

#endif // FOLLOW_CONTROLLER_H
