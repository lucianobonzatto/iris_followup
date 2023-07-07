#include "follow_controller.h"

Follow_Controller::Follow_Controller()
{
    // x_reference = 0;
    // y_reference = 0;
    // z_reference = 1;

    // x_controller.setParameters(0, 0, 0, 0);
    // y_controller.setParameters(0, 0, 0, 0);
    // z_controller.setParameters(0, 0, 0, 0);
    // yaw_controller.setParameters(0, 0, 0, 0);

    // controller.set_dt();
    // controller.setOutputLimits();
    // controller.enableAngularInput();
    // controller.enableConditionalIntegration();
    // controller.enableFeedforward();
}

Follow_Controller::~Follow_Controller()
{
}

void Follow_Controller::print_parameters()
{
    // double Kp_x, Ki_x, Kd_x, Kf_x;
    // double Kp_y, Ki_y, Kd_y, Kf_y;
    // double Kp_z, Ki_z, Kd_z, Kf_z;
    // double Kp_yaw, Ki_yaw, Kd_yaw, Kf_yaw;
    // x_controller.getParameters(&Kp_x, &Ki_x, &Kd_x, &Kf_x);
    // y_controller.getParameters(&Kp_y, &Ki_y, &Kd_y, &Kf_y);
    // z_controller.getParameters(&Kp_z, &Ki_z, &Kd_z, &Kf_z);
    // yaw_controller.getParameters(&Kp_yaw, &Ki_yaw, &Kd_yaw, &Kf_yaw);

    // cout << "Follow_Controller: " << endl;
    // cout << "\tKp_x:   " << Kp_x <<   "\tKi_x:   " << Ki_x
    //      << "\tKd_x:   " << Kd_x <<   "\tKf_x:   " << Kf_x << endl;
    // cout << "\tKp_y:   " << Kp_y <<   "\tKi_y:   " << Ki_y
    //      << "\tKd_y:   " << Kd_y <<   "\tKf_y:   " << Kf_y << endl;
    // cout << "\tKp_z:   " << Kp_z <<   "\tKi_z:   " << Ki_z
    //      << "\tKd_z:   " << Kd_z <<   "\tKf_z:   " << Kf_z << endl;
    // cout << "\tKp_yaw: " << Kp_yaw << "\tKi_yaw: " << Ki_yaw
    //      << "\tKd_yaw: " << Kd_yaw << "\tKf_yaw: " << Kf_yaw << endl;

    // switch (level) {
    //     case DebugLevel::BasicInfo:
    //         std::cout << "Current output: " << output << "\n";
    //         break;
    //     case DebugLevel::ParameterInfo:
    //         std::cout << "Parameters [Kp, Ki, Kd, Kf]: [" << Kp << ", " << Ki << ", " << Kd << ", " << Kf << "]\n";
    //         std::cout << "Current dt: " << dt << "\n";
    //         break;
    //     case DebugLevel::DetailedInfo:
    //         std::cout << "Current error: " << error << "\n";
    //         std::cout << "Current integral error: " << integral_error << "\n";
    //         std::cout << "Current output: " << output << "\n";
    //         std::cout << "Output limits: [" << output_min << ", " << output_max << "]\n";
    //         break;
    // }
}

void Follow_Controller::update_parameters(float *newParameters)
{
    // x_controller.setParameters(newParameters[0], newParameters[1], newParameters[2], 0);
    // y_controller.setParameters(newParameters[3], newParameters[4], newParameters[5], 0);
    // z_controller.setParameters(newParameters[6], newParameters[7], newParameters[8], 0);
    // yaw_controller.setParameters(newParameters[9], newParameters[10], newParameters[11], 0);
}

geometry_msgs::Twist Follow_Controller::get_velocity()
{
    velocity.linear.x = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;

    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.angular.z = 0;

    // if ((track.header.stamp.isZero()) ||
    //     (track_last_timestamp == track.header.stamp))
    // {
    //     return velocity;
    // }

    // for (int i = 0; i < track.markers.size(); i++)
    // {
    //     if (track.markers[i].id == TAG_ID)
    //     {
    //         x_controller.compute(x_reference, track.markers[i].pose.pose.position.x);
    //         y_controller.compute(y_reference, track.markers[i].pose.pose.position.y);
    //         z_controller.compute(z_reference, track.markers[i].pose.pose.position.z);
    //         yaw_controller.compute(yaw_reference, track.markers[i].pose.pose.orientation.x);

    //         velocity.linear.x = x_controller.getOutput();
    //         velocity.linear.y = y_controller.getOutput();
    //         velocity.linear.z = z_controller.getOutput();
    //         velocity.angular.z = yaw_controller.getOutput();
    //     }
    // }

    return velocity;
}
