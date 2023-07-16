#include "manager.h"

Manager::Manager()
{
}

Manager::~Manager()
{
}

void Manager::Init(DroneControl *drone_control,
                   double joyLinearVelocity,
                   double joyAngularVelocity)
{
  drone_connection = drone_control;
  joy_linear_velocity = joyLinearVelocity;
  joy_angular_velocity = joyAngularVelocity;
}

void Manager::print_parameters()
{
  cout << "================" << endl;
  // cout << "\tpose: "<< pose.header.stamp << endl;
  cout << "\tjoy: " << joy.header.stamp << endl;
  cout << "\todom: " << odom.header << endl;
  cout << "\tstate: " << states_name[state_machine.get_state()] << endl;
  follow_controller.print_parameters();
  land_controller.print_parameters();
}

void Manager::update()
{
  STATES state = state_machine.get_state();
  switch (state)
  {
  case STATES::STOPPED:
    STOPPED_action();
    break;
  case STATES::TAKE_OFF:
    TAKE_OFF_action();
    break;
  case STATES::LAND:
    LAND_action();
    break;
  case STATES::JOY_CONTROL:
    JOY_CONTROL_action();
    break;
  case STATES::LAND_CONTROL:
    LAND_CONTROL_action();
    break;
  case STATES::FOLLOW_CONTROL:
    FOLLOW_CONTROL_action();
    break;
  default:
    break;
  }

  if (!parameters.data.empty())
  {
    follow_controller.update_parameters(&parameters.data[0]);
  }
  if (state_machine.update_state(joy))
  {
    send_velocity(0, 0, 0, 0);
  }
}

void Manager::STOPPED_action()
{
}

void Manager::TAKE_OFF_action()
{
  drone_connection->offboardMode();
  drone_connection->takeOff();
}

void Manager::LAND_action()
{
  drone_connection->land();
}

void Manager::JOY_CONTROL_action()
{
  if (joy.header.stamp.isZero())
  {
    return;
  }
  if (joy_last_timestamp == joy.header.stamp)
  {
    send_velocity(0, 0, 0, 0);
    return;
  }
  joy_last_timestamp = joy.header.stamp;

  send_velocity(joy.axes[1] * joy_linear_velocity,
                joy.axes[0] * joy_linear_velocity,
                joy.axes[4] * joy_linear_velocity,
                joy.axes[3] * joy_angular_velocity);
}

void Manager::LAND_CONTROL_action()
{
  geometry_msgs::Twist velocity;
  velocity = land_controller.get_velocity();
  send_velocity(velocity);
}

void Manager::FOLLOW_CONTROL_action()
{
  geometry_msgs::Twist velocity;
  velocity = follow_controller.get_velocity(droneVel);
  send_velocity(velocity);
}

void Manager::send_velocity(double x_linear, double y_linear, double z_linear, double angular)
{
  drone_connection->cmd_vel(x_linear, y_linear, z_linear, angular);
}

void Manager::send_velocity(geometry_msgs::Twist velocity)
{
  drone_connection->cmd_vel(velocity.linear.x,
                            velocity.linear.y,
                            velocity.linear.z,
                            velocity.angular.z);
}

// void Manager::set_pose(geometry_msgs::PoseStamped newPose)
// {
//   lastPose = pose;
//   pose = newPose;

//   ros::Duration dt = pose.header.stamp - lastPose.header.stamp;
//   double teste = (double)dt.nsec / 1000000000;
//   droneVel.vx = (pose.pose.position.x - lastPose.pose.position.x) / teste;
//   droneVel.vy = (pose.pose.position.y - lastPose.pose.position.y) / teste;
//   droneVel.vz = (pose.pose.position.z - lastPose.pose.position.z) / teste;
//   droneVel.vtheta = (pose.pose.orientation.x - lastPose.pose.orientation.x) / teste;
// }

void Manager::set_odom(nav_msgs::Odometry newOdom)
{
  odom = newOdom;
}

void Manager::set_joy(sensor_msgs::Joy newJoy)
{
  joy = newJoy;
}

void Manager::set_parameters(std_msgs::Float32MultiArray newParameters)
{
  parameters = newParameters;
}
