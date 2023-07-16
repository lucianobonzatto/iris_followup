#include "follow_controller.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

Follow_Controller::Follow_Controller()
{
    setpoint.x = 0;
    setpoint.y = 0;
    setpoint.z = 1;
    setpoint.theta = 0;

    PID::Builder builder;

    // TelloCascadePDPIController cont(builder, builder,
    //                                 builder, builder,
    //                                 builder, builder,
    //                                 builder, builder);

    TelloPDController cont(builder,
                           builder,
                           builder,
                           builder);

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
    kp_pd = kd_pd = kp_pi = ki_pi = 0;
    controller.get_x(kp_pd, kd_pd); //, kp_pi, ki_pi);
    cout << "\tx_p_pd: " << kp_pd
         << "\tx_d_pd: " << kd_pd
         << "\tx_p_pi: " << kp_pi
         << "\tx_i_pi: " << ki_pi << endl;

    controller.get_y(kp_pd, kd_pd); //, kp_pi, ki_pi);
    cout << "\ty_p_pd: " << kp_pd
         << "\ty_d_pd: " << kd_pd
         << "\ty_p_pi: " << kp_pi
         << "\ty_i_pi: " << ki_pi << endl;

    controller.get_z(kp_pd, kd_pd); //, kp_pi, ki_pi);
    cout << "\tz_p_pd: " << kp_pd
         << "\tz_d_pd: " << kd_pd
         << "\tz_p_pi: " << kp_pi
         << "\tz_i_pi: " << ki_pi << endl;

    controller.get_theta(kp_pd, kd_pd); //, kp_pi, ki_pi);
    cout << "\tt_p_pd: " << kp_pd
         << "\tt_d_pd: " << kd_pd
         << "\tt_p_pi: " << kp_pi
         << "\tt_i_pi: " << ki_pi << endl;
}

void Follow_Controller::update_parameters(float *newParameters)
{
    // controller.update_x(newParameters[0], newParameters[1], newParameters[2], newParameters[3]);
    // controller.update_y(newParameters[4], newParameters[5], newParameters[6], newParameters[7]);
    // controller.update_z(newParameters[8], newParameters[9], newParameters[10], newParameters[11]);
    // controller.update_theta(newParameters[12], newParameters[13], newParameters[14], newParameters[15]);

    controller.update_x(newParameters[0], newParameters[1]);
    controller.update_y(newParameters[4], newParameters[5]);
    controller.update_z(newParameters[8], newParameters[9]);
    controller.update_theta(newParameters[12], newParameters[13]);
}

Pose Follow_Controller::read_tf(geometry_msgs::TransformStamped tf)
{
    Pose measurement;
    tf::Quaternion quat;
    double roll, pitch, yaw;
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation.x = tf.transform.rotation.x;
    pose.pose.orientation.y = tf.transform.rotation.y;
    pose.pose.orientation.z = tf.transform.rotation.z;
    pose.pose.orientation.w = tf.transform.rotation.w;
    tf::quaternionMsgToTF(pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    measurement.x = pose.pose.position.x;
    measurement.y = pose.pose.position.y;
    measurement.z = pose.pose.position.z;
    measurement.theta = yaw;
    return measurement;
}

geometry_msgs::Twist Follow_Controller::get_velocity(Speed droneVel)
{
    Pose iris_mes, magni_mes;
    geometry_msgs::Twist velocity;
    geometry_msgs::TransformStamped iris_pose, magni_pose;
    tf2_ros::TransformListener tfListener(tfBuffer);

    try
    {
        iris_pose = tfBuffer.lookupTransform("odom_ned", "iris", ros::Time(0), ros::Duration(1));
        magni_pose = tfBuffer.lookupTransform("odom_ned", "base_footprint", ros::Time(0), ros::Duration(1));
    }
    catch (tf2::TransformException &ex)
    {
        return velocity;
    }

    iris_mes = read_tf(iris_pose);
    magni_mes = read_tf(magni_pose);

    cout << "\tx -> " << iris_mes.x
         << "\ty -> " << iris_mes.y
         << "\tz -> " << iris_mes.z
         << "\tt -> " << iris_mes.theta << endl;

    cout << "\tx -> " << magni_mes.x
         << "\ty -> " << magni_mes.y
         << "\tz -> " << magni_mes.z
         << "\tt -> " << magni_mes.theta << endl;

    // Speed vel = controller.control(setpoint, measurement, droneVel);

    // velocity.linear.x = vel.vx;
    // velocity.linear.y = vel.vy;
    // velocity.linear.z = vel.vz;
    // velocity.angular.z = vel.vtheta;

    // cout << "x -> " << droneVel.vx << "\t" << poseStamped.pose.position.x << "\t" << velocity.linear.x << endl;
    // cout << "y -> " << droneVel.vy << "\t" << poseStamped.pose.position.y << "\t" << velocity.linear.y << endl;
    // cout << "z -> " << droneVel.vz << "\t" << poseStamped.pose.position.z << "\t" << velocity.linear.z << endl;
    // cout << "theta -> " << droneVel.vtheta << "\t" << poseStamped.pose.position.z << "\t" << velocity.linear.z << endl;

    return velocity;
}
