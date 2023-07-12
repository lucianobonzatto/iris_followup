#include "follow_controller.h"

Follow_Controller::Follow_Controller()
{
    x_reference = 0;
    y_reference = 0;
    z_reference = 1;
    yaw_reference = 0;

    x_controller.setParameters(0, 0, 0, 0);
    y_controller.setParameters(0, 0, 0, 0);
    z_controller.setParameters(0, 0, 0, 0);
    yaw_controller.setParameters(0, 0, 0, 0);
}

Follow_Controller::~Follow_Controller()
{
}

void Follow_Controller::print_parameters()
{
    double Kp_x, Ki_x, Kd_x, Kf_x;
    double Kp_y, Ki_y, Kd_y, Kf_y;
    double Kp_z, Ki_z, Kd_z, Kf_z;
    double Kp_yaw, Ki_yaw, Kd_yaw, Kf_yaw;
    x_controller.getParameters(&Kp_x, &Ki_x, &Kd_x, &Kf_x);
    y_controller.getParameters(&Kp_y, &Ki_y, &Kd_y, &Kf_y);
    z_controller.getParameters(&Kp_z, &Ki_z, &Kd_z, &Kf_z);
    yaw_controller.getParameters(&Kp_yaw, &Ki_yaw, &Kd_yaw, &Kf_yaw);

    cout << "Inspec_Controller: " << endl;
    cout << "\tKp_x:   " << Kp_x << "\tKi_x:   " << Ki_x << "\tKd_x:   " << Kd_x << endl;
    cout << "\tKp_y:   " << Kp_y << "\tKi_y:   " << Ki_y << "\tKd_y:   " << Kd_y << endl;
    cout << "\tKp_z:   " << Kp_z << "\tKi_z:   " << Ki_z << "\tKd_z:   " << Kd_z << endl;
    cout << "\tKp_yaw: " << Kp_yaw << "\tKi_yaw: " << Ki_yaw << "\tKd_yaw: " << Kd_yaw << endl;
}

void Follow_Controller::update_parameters(float *newParameters)
{
    x_controller.setParameters(newParameters[0], newParameters[1], newParameters[2], 0);
    y_controller.setParameters(newParameters[3], newParameters[4], newParameters[5], 0);
    z_controller.setParameters(newParameters[6], newParameters[7], newParameters[8], 0);
    yaw_controller.setParameters(newParameters[9], newParameters[10], newParameters[11], 0);
}

geometry_msgs::Twist Follow_Controller::get_velocity(geometry_msgs::Pose pose)
{
    velocity.linear.x = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;

    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.angular.z = 0;

    if ((pose.position.x == 0) &&
        (pose.position.y == 0) &&
        (pose.position.z == 0) &&
        (pose.orientation.x == 0) &&
        (pose.orientation.y == 0) &&
        (pose.orientation.z == 0) &&
        (pose.orientation.w == 0))
    {
        return velocity;
    }
    
    x_controller.compute(x_reference, pose.position.x);
    // y_controller.compute(y_reference, pose.position.y);
    // z_controller.compute(z_reference, pose.position.z);
    // yaw_controller.compute(yaw_reference, pose.orientation.x);

    velocity.linear.x = x_controller.getOutput();
    // velocity.linear.y = y_controller.getOutput();
    // velocity.linear.z = z_controller.getOutput();
    // velocity.angular.z = yaw_controller.getOutput();

    return velocity;
}
