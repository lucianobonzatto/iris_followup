#include "general.h"
#include "manager.h"
#include "ros_client.h"
#include "drone_control.h"

Manager principal;
static void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
  principal.set_joy(*msg);
}
static void parametersCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
  principal.set_parameters(*msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "manager_control");
  ros::NodeHandle *nh = new ros::NodeHandle();
  ros::Rate loop_rate(20);

  ROSClient ros_client(nh);
  DroneControl drone_control(&ros_client);

  ros::Subscriber joy_sub = nh->subscribe<sensor_msgs::Joy>("/joy_control", 1, &joyCallback);
  ros::Subscriber parameters_sub = nh->subscribe<std_msgs::Float32MultiArray>("/PID/parameters", 1, &parametersCallback);

  principal.Init(&drone_control, 5, 1);

  while(ros::ok()){
    ros::spinOnce();
    principal.print_parameters();
    principal.update();
    
    loop_rate.sleep();
  }

  delete nh;
}