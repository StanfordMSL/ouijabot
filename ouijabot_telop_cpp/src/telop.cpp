// Tele-operation code for OuijaBot (or any ROS-enabled robots)
// Author: Zijian Wang, zjwang@stanford.edu
// Date: Sept 16, 2016

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

// class declaration
class Telop{
private:
  ros::NodeHandle node_handle_; // the underscore is a tradition for notating private variables in a class
  ros::Subscriber joy_subscriber_;
  ros::Publisher velocity_publisher_;
  geometry_msgs::Twist velocity_;

public:
  Telop(); // constructor
  void joystick_callback(const sensor_msgs::JoyConstPtr &joy);

};

// class function implementations
Telop::Telop()
{
  // create subscriber and bind callback function
  joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("/joy", 20, &Telop::joystick_callback, this); // must pass in: 1. the callback function; 2. the object instance by "this" pointer

  // create velocity publisher
  velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("ouijabot1/cmd_vel", 10);
}

void Telop::joystick_callback(const sensor_msgs::JoyConstPtr &joy)
{
  // retrieve the joystick values
  // reference about the Joy message: http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Joy.html
  // reference about the Twist message: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
  velocity_.linear.x = - joy->axes[0]; 
  velocity_.linear.y = joy->axes[1];
  velocity_.angular.z = joy->axes[2];

  // publish the velocity command (Twist type)
  velocity_publisher_.publish(velocity_);
}


// main function
int main(int argc, char **argv)
{

  ros::init(argc, argv, "ouijabot_telop");

  Telop telop; // create an instance of the class

  ros::spin(); // keep the application alive while automatically handle the ROS messages

  return 0;
}
