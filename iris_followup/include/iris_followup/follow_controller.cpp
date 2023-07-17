#include "follow_controller.h"

Follow_Controller::Follow_Controller()
{
    pose_setpoint.x = 0;
    pose_setpoint.y = 0;
    pose_setpoint.z = 1.5;
    pose_setpoint.theta = 0;

    PID::Builder builder;

    TelloParallelPDPIController controller(
        builder, builder,
        builder, builder,
        builder, builder,
        builder, builder);

    pdController = controller;
}

Follow_Controller::~Follow_Controller()
{
}

void Follow_Controller::print_parameters()
{
    cout << "Follow_Controller: " << endl;

    double kp_pd, kd_pd, kp_pi, ki_pi;
    kp_pd = kd_pd = kp_pi = ki_pi = 0;
    pdController.get_x(kp_pd, kd_pd, kp_pi, ki_pi);
    cout << "\tx_p_pd: " << kp_pd
         << "\tx_d_pd: " << kd_pd
         << "\tx_p_pi: " << kp_pi
         << "\tx_i_pi: " << ki_pi << endl;

    pdController.get_y(kp_pd, kd_pd, kp_pi, ki_pi);
    cout << "\ty_p_pd: " << kp_pd
         << "\ty_d_pd: " << kd_pd
         << "\ty_p_pi: " << kp_pi
         << "\ty_i_pi: " << ki_pi << endl;

    pdController.get_z(kp_pd, kd_pd, kp_pi, ki_pi);
    cout << "\tz_p_pd: " << kp_pd
         << "\tz_d_pd: " << kd_pd
         << "\tz_p_pi: " << kp_pi
         << "\tz_i_pi: " << ki_pi << endl;

    pdController.get_theta(kp_pd, kd_pd, kp_pi, ki_pi);
    cout << "\tt_p_pd: " << kp_pd
         << "\tt_d_pd: " << kd_pd
         << "\tt_p_pi: " << kp_pi
         << "\tt_i_pi: " << ki_pi << endl;
}

void Follow_Controller::update_parameters(float *newParameters)
{
    pdController.update_x(newParameters[0], newParameters[1], newParameters[2], newParameters[3]);
    pdController.update_y(newParameters[4], newParameters[5], newParameters[6], newParameters[7]);
    pdController.update_z(newParameters[8], newParameters[9], newParameters[10], newParameters[11]);
    pdController.update_theta(newParameters[12], newParameters[13], newParameters[14], newParameters[15]);

    // pdController.update_x(newParameters[0], newParameters[1]);
    // pdController.update_y(newParameters[4], newParameters[5]);
    // pdController.update_z(newParameters[8], newParameters[9]);
    // pdController.update_theta(newParameters[12], newParameters[13]);
}

geometry_msgs::Twist Follow_Controller::get_velocity(geometry_msgs::PoseStamped poseStamped, Speed iris_vel)
{
    geometry_msgs::Twist velocity;
    geometry_msgs::Pose pose = poseStamped.pose;

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

    Pose measurement;
    measurement.x = pose.position.x;
    measurement.y = pose.position.y;
    measurement.z = pose.position.z;
    measurement.theta = pose.orientation.x;

    Speed vel = pdController.control(pose_setpoint, measurement, iris_vel);

    velocity.linear.x = vel.vx;
    velocity.linear.y = vel.vy;
    velocity.linear.z = vel.vz;
    velocity.angular.z = vel.vtheta;

    cout << "x -> " << pose_setpoint.x << "\t" << pose.position.x << "\t" << velocity.linear.x << "\t" << iris_vel.vx << endl;
    cout << "y -> " << pose_setpoint.y << "\t" << pose.position.y << "\t" << velocity.linear.y << "\t" << iris_vel.vy << endl;
    cout << "z -> " << pose_setpoint.z << "\t" << pose.position.z << "\t" << velocity.linear.z << "\t" << iris_vel.vz << endl;
    cout << "theta -> " << pose_setpoint.theta << "\t" << pose.orientation.x << "\t" << velocity.angular.z << "\t" << iris_vel.vtheta << endl;

    return velocity;
}
