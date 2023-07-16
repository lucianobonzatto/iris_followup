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
    geometry_msgs::Twist get_velocity(Speed droneVel);    
    void update_parameters(float *newParameters);

private:

    Pose read_tf(geometry_msgs::TransformStamped tf);

    ros::Time track_last_timestamp;
    // TelloCascadePDPIController controller;
    TelloPDController controller;
    Pose setpoint;
    tf2_ros::Buffer tfBuffer;

};

#endif // FOLLOW_CONTROLLER_H
