#include "follow_controller.h"

Follow_Controller::Follow_Controller()
{
    setpoint.x = 0;
    setpoint.y = 0;
    setpoint.z = 1;
    setpoint.theta = 0;

    PID::Builder builder;

    TelloParallelPDPIController cont(builder, builder,
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
    cout << "\tkp_pd: " << kp_pd
         << "\tkd_pd: " << kd_pd
         << "\tkp_pi: " << kp_pi
         << "\tki_pi: " << ki_pi << endl;

    controller.get_y(kp_pd, kd_pd, kp_pi, ki_pi);
    cout << "\tkp_pd: " << kp_pd
         << "\tkd_pd: " << kd_pd
         << "\tkp_pi: " << kp_pi
         << "\tki_pi: " << ki_pi << endl;

    controller.get_z(kp_pd, kd_pd, kp_pi, ki_pi);
    cout << "\tkp_pd: " << kp_pd
         << "\tkd_pd: " << kd_pd
         << "\tkp_pi: " << kp_pi
         << "\tki_pi: " << ki_pi << endl;

    controller.get_theta(kp_pd, kd_pd, kp_pi, ki_pi);
    cout << "\tkp_pd: " << kp_pd
         << "\tkd_pd: " << kd_pd
         << "\tkp_pi: " << kp_pi
         << "\tki_pi: " << ki_pi << endl;
}

void Follow_Controller::update_parameters(float *newParameters)
{
    controller.update_x(newParameters[0], newParameters[1], newParameters[2], newParameters[3]);
    controller.update_y(newParameters[4], newParameters[5], newParameters[6], newParameters[7]);
    controller.update_z(newParameters[8], newParameters[9], newParameters[10], newParameters[11]);
    controller.update_theta(newParameters[12], newParameters[13], newParameters[14], newParameters[15]);
}

geometry_msgs::Twist Follow_Controller::get_velocity(geometry_msgs::PoseStamped poseStamped)
{
    geometry_msgs::Twist velocity;
    geometry_msgs::Pose pose = poseStamped.pose;

    if (poseStamped.header.stamp.is_zero())
    {
        return velocity;
    }

    Pose measurement;
    measurement.x = pose.position.x;
    measurement.y = pose.position.y;
    measurement.z = pose.position.z;
    measurement.theta = pose.orientation.x;

    Speed vel = controller.control(setpoint, measurement);

    velocity.linear.x = vel.vx;
    velocity.linear.y = vel.vy;
    velocity.linear.z = vel.vz;
    velocity.angular.z = vel.vtheta;

    cout << "x -> " << setpoint.x << "\t" << pose.position.x << "\t" << velocity.linear.x << endl;
    cout << "y -> " << setpoint.y << "\t" << pose.position.y << "\t" << velocity.linear.y << endl;
    cout << "z -> " << setpoint.z << "\t" << pose.position.z << "\t" << velocity.linear.z << endl;
    cout << "theta -> " << setpoint.theta << "\t" << pose.position.z << "\t" << velocity.linear.z << endl;

    return velocity;
}
