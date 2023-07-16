#include "follow_controller.h"

Follow_Controller::Follow_Controller()
{
    setpoint.x = 0;
    setpoint.y = 0;
    setpoint.z = 1;
    setpoint.theta = 0;

    PID::Builder builder;

    TelloCascadePDPIController cont(builder, builder,
                                    builder, builder,
                                    builder, builder,
                                    builder, builder);

    controller = cont;
}

Follow_Controller::~Follow_Controller()
{
}

void Follow_Controller::print_parameters()
{
    std::cout << std::fixed << std::setprecision(4);
    cout << "Follow_Controller: " << endl;

    double kp_pd, kd_pd, kp_pi, ki_pi;
    controller.get_x(kp_pd, kd_pd, kp_pi, ki_pi);
    cout << "\tx_p_pd: " << kp_pd
         << "\tx_d_pd: " << kd_pd
         << "\tx_p_pi: " << kp_pi
         << "\tx_i_pi: " << ki_pi << endl;

    controller.get_y(kp_pd, kd_pd, kp_pi, ki_pi);
    cout << "\ty_p_pd: " << kp_pd
         << "\ty_d_pd: " << kd_pd
         << "\ty_p_pi: " << kp_pi
         << "\ty_i_pi: " << ki_pi << endl;

    controller.get_z(kp_pd, kd_pd, kp_pi, ki_pi);
    cout << "\tz_p_pd: " << kp_pd
         << "\tz_d_pd: " << kd_pd
         << "\tz_p_pi: " << kp_pi
         << "\tz_i_pi: " << ki_pi << endl;

    controller.get_theta(kp_pd, kd_pd, kp_pi, ki_pi);
    cout << "\tt_p_pd: " << kp_pd
         << "\tt_d_pd: " << kd_pd
         << "\tt_p_pi: " << kp_pi
         << "\tt_i_pi: " << ki_pi << endl;
}

void Follow_Controller::update_parameters(float *newParameters)
{
    controller.update_x(newParameters[0], newParameters[1], newParameters[2], newParameters[3]);
    controller.update_y(newParameters[4], newParameters[5], newParameters[6], newParameters[7]);
    controller.update_z(newParameters[8], newParameters[9], newParameters[10], newParameters[11]);
    controller.update_theta(newParameters[12], newParameters[13], newParameters[14], newParameters[15]);
}

geometry_msgs::Twist Follow_Controller::get_velocity(geometry_msgs::PoseStamped poseStamped, Speed droneVel)
{
    Pose measurement;
    geometry_msgs::Twist velocity;

    if (poseStamped.header.stamp.is_zero())
    {
        return velocity;
    }

    measurement.x = poseStamped.pose.position.x;
    measurement.y = poseStamped.pose.position.y;
    measurement.z = poseStamped.pose.position.z;
    measurement.theta = poseStamped.pose.orientation.x;

    Speed vel = controller.control(setpoint, measurement, droneVel);

    velocity.linear.x = vel.vx;
    velocity.linear.y = vel.vy;
    velocity.linear.z = vel.vz;
    velocity.angular.z = vel.vtheta;


    cout << "x -> " << droneVel.vx << "\t" << poseStamped.pose.position.x << "\t" << velocity.linear.x << endl;
    cout << "y -> " << droneVel.vy << "\t" << poseStamped.pose.position.y << "\t" << velocity.linear.y << endl;
    cout << "z -> " << droneVel.vz << "\t" << poseStamped.pose.position.z << "\t" << velocity.linear.z << endl;
    cout << "theta -> " << droneVel.vtheta << "\t" << poseStamped.pose.position.z << "\t" << velocity.linear.z << endl;

    return velocity;
}
