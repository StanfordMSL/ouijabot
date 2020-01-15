// Tele-operation code that supports multiple OuijaBots
// Author: Zijian Wang, zjwang@stanford.edu
// Date: Nov 22, 2017

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

// class declaration
class Telop{
private:
  ros::NodeHandle node_handle_; // the underscore is a tradition for notating private variables in a class
  ros::Subscriber joy_subscriber_;
  ros::Publisher velocity_pub1_;
  ros::Publisher velocity_pub2_;
  ros::Publisher velocity_pub3_;
  ros::Publisher velocity_pub4_;
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
  velocity_pub1_ = node_handle_.advertise<geometry_msgs::Twist>("vel_out1", 10);
  velocity_pub2_ = node_handle_.advertise<geometry_msgs::Twist>("vel_out2", 10);
  velocity_pub3_ = node_handle_.advertise<geometry_msgs::Twist>("vel_out3", 10);
  velocity_pub4_ = node_handle_.advertise<geometry_msgs::Twist>("vel_out4", 10);
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
  // button 12 is used for selecting all the robots
  if(joy->buttons[6] || joy->buttons[11]) {
    velocity_pub1_.publish(velocity_);}
  if(joy->buttons[7] || joy->buttons[11]) {
    velocity_pub2_.publish(velocity_);}
  if(joy->buttons[8] || joy->buttons[11]) {
    velocity_pub3_.publish(velocity_);}
  if(joy->buttons[9] || joy->buttons[11]) {
    velocity_pub4_.publish(velocity_);}
}


// main function
int main(int argc, char **argv)
{

  ros::init(argc, argv, "ouijabot_telop_mux");

  Telop telop; // create an instance of the class

  ros::spin(); // keep the application alive while automatically handle the ROS messages

  return 0;
}
