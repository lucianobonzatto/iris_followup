#include "follow_controller.h"

Follow_Controller::Follow_Controller()
{
    x_reference = 0;
    y_reference = 0;
    z_reference = 1;
    yaw_reference = 0;

    PID::Builder builder_pd_x;
    PID::Builder builder_pd_y;
    PID::Builder builder_pd_z;
    PID::Builder builder_pd_theta;

    TelloPDController controller(
        builder_pd_x,
        builder_pd_y,
        builder_pd_z,
        builder_pd_theta);

    pdController = controller;
}

Follow_Controller::~Follow_Controller()
{
}

void Follow_Controller::print_parameters()
{
    cout << "Follow_Controller: " << endl;

    double Kp, Kd;
    pdController.get_x(Kp, Kd);
    cout << "\tKp_x: " << Kp << "\tKd_x: " << Kd << endl;
    pdController.get_y(Kp, Kd);
    cout << "\tKp_x: " << Kp << "\tKd_x: " << Kd << endl;
    pdController.get_z(Kp, Kd);
    cout << "\tKp_x: " << Kp << "\tKd_x: " << Kd << endl;
    pdController.get_theta(Kp, Kd);
    cout << "\tKp_x: " << Kp << "\tKd_x: " << Kd << endl;
}

void Follow_Controller::update_parameters(float *newParameters)
{
    // x_controller.setParameters(newParameters[0], newParameters[1], newParameters[2], 0);
    // y_controller.setParameters(newParameters[3], newParameters[4], newParameters[5], 0);
    // z_controller.setParameters(newParameters[6], newParameters[7], newParameters[8], 0);
    // yaw_controller.setParameters(newParameters[9], newParameters[10], newParameters[11], 0);

    pdController.update_x(newParameters[0], newParameters[1]);
    pdController.update_y(newParameters[3], newParameters[4]);
    pdController.update_z(newParameters[6], newParameters[7]);
    pdController.update_theta(newParameters[9], newParameters[10]);
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

    cout << "x -> " << x_reference << "\t" << pose.position.x << endl;
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
