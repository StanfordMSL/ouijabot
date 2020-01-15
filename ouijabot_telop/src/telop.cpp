// Tele-operation code for OuijaBot (or any ROS-enabled robots)
// Author: Zijian Wang, zjwang@stanford.edu
// Date: Sept 16, 2016

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

// class declaration
class Telop{
private:
  ros::NodeHandle nh_; // the underscore is a tradition for notating private variables in a class
  ros::Subscriber joy_subscriber_;
  ros::Publisher velocity_publisher_;
  geometry_msgs::Twist velocity_;


  //timer assets 
  double controlLoopFreq_; // frequency for control loop
  double max_lin_vel_, max_ang_vel_; // maximum velocities of the robot
  ros::Timer controlTimer_;
  void controlTimerCB(const ros::TimerEvent& event);
  double my_clamp(double val, double min, double max);

public:
  Telop(); // constructor
  void joystick_callback(const sensor_msgs::JoyConstPtr &joy);

};

// class function implementations
Telop::Telop()
{
  // create subscriber and bind callback function
  joy_subscriber_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 20, &Telop::joystick_callback, this); // must pass in: 1. the callback function; 2. the object instance by "this" pointer

  // create velocity publisher
  velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("vel_out", 10);

  // create main loop timer
  ros::param::get("~control_freq", controlLoopFreq_);
  ros::param::get("~velMax_l", max_lin_vel_);
  ros::param::get("~velMax_a", max_ang_vel_);

  controlTimer_ = nh_.createTimer(
      ros::Duration(1.0/controlLoopFreq_), 
      &Telop::controlTimerCB, this); 
}

void Telop::joystick_callback(const sensor_msgs::JoyConstPtr &joy)
{
  // retrieve the joystick values
  // reference about the Joy message: http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Joy.html
  // reference about the Twist message: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
  velocity_.linear.x = max_lin_vel_*joy->axes[1]; 
  velocity_.linear.y = max_lin_vel_*joy->axes[0];
  velocity_.angular.z = max_ang_vel_*joy->axes[2];
  //velocity_.angular.z = my_clamp(joy->axes[2], -max_ang_vel_, max_ang_vel_);

  /*
  double lin_vel = sqrt(velocity_.linear.x * velocity_.linear.x + velocity_.linear.y * velocity_.linear.y);
  if(lin_vel > 0.0001)
  {
    // This if statement is needed or else divide by zero issues when there is no joystick inputs (lin_vel = 0)
    double new_lin_vel = my_clamp(lin_vel, -max_lin_vel_, max_lin_vel_);
    velocity_.linear.x *= new_lin_vel/lin_vel; 
    velocity_.linear.y *= new_lin_vel/lin_vel;
  } */

}

void Telop::controlTimerCB(const ros::TimerEvent& event) {
  // publish the velocity command (Twist type)
  velocity_publisher_.publish(velocity_);
}

double Telop::my_clamp(double val, double min, double max)
{
  if(val >= max)
  {
    return max;
  }
  else if(val <= min)
  {
    return min;
  }
  return val;
}

// main function
int main(int argc, char **argv)
{

  ros::init(argc, argv, "ouijabot_telop");

  Telop telop; // create an instance of the class

  ros::spin(); // keep the application alive while automatically handle the ROS messages

  return 0;
}
