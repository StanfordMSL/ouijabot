/*--------------------------------------------------------------------------
 
 File Name:         filter.cpp
 Date Created:      2017/08/07
 Date Modified:     2017/08/18
 
 Author:            Eric Cristofalo
 Contact:           eric.cristofalo@gmail.com
 
 Description:       ROS node for estimating realtime pose and odometry via EKF from Optitrack and Ouijibot IMU/wheel encoders
 
 -------------------------------------------------------------------------*/

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <random>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <ouijabot/Wheel_Spd.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions> // matrix exponential

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class filterClass {
    
  ros::NodeHandle nh_;
  
  ros::Subscriber process_mocap_sub_;
  ros::Subscriber process_imu_sub_;
  ros::Subscriber process_wheel_speed_sub_;
  // ros::Subscriber ground_truth_sub_;
  // ros::Subscriber ground_truth_accel_sub_;

  ros::Publisher state_estimate_pub_;
  tf::TransformBroadcaster state_estimate_broadcaster;
  nav_msgs::Odometry stateEstimateMsg;
  
  // Initialize Input Variables
  int displayData, useOptitrack;
  double poseCov, orientCov, velCov, rotRateCov, accelCov, rotAccelCov, wheelMeasCov, gyroMeasCov, accelMeasCov, r_w, r_b;

  // Initialize Filter Variables
  int initCount;
  double count, initTime;
  bool initPoseBool, predictBool, updateBool, gyroBool, wheelSpdBool;
  ros::Time tCur, tPrev;
  ros::Time tCur_, tPrev_;
  VectorXd state, state_, state_gt, y, gyro, accel, wheel_spd;
  MatrixXd Sigma, Sigma_, Q, R;
    
public:
  filterClass(int in_01, int in_02, double in_03, double in_04, double in_05, double in_06, double in_07, double in_08, double in_09, double in_10, double in_11, double in_12, double in_13) {

    // Subscribe to Optitrack Topics
    process_mocap_sub_ = nh_.subscribe("/robot/pose", 10, &filterClass::mocapCallback, this);

    // Subscribe to Ouijibot IMU
    process_imu_sub_ = nh_.subscribe("/ouijabot/imu", 10, &filterClass::imuCallback, this);

    // Subscribe to Ouijibot Wheel Encoders
    process_wheel_speed_sub_ = nh_.subscribe("/ouijabot/wheel_spd", 10, &filterClass::wheelSpeedCallback, this);

    // // Subscribe to Optitrack Ground Truth Topics
    // ground_truth_sub_ = nh_.subscribe("/robot/odom", 10, &filterClass::mocapCallback, this);
    // ground_truth_acc_sub_ = nh_.subscribe("/robot/accel", 10, &filterClass::mocapCallback, this);

    // Publish Odometry Message of State Estimate
    state_estimate_pub_ = nh_.advertise<nav_msgs::Odometry>("/robot/state_estimate", 10);

    // Initialize Variables
    displayData = in_01;
    useOptitrack = in_02;
    r_w = in_03;
    r_b = in_04;
    poseCov = in_05;
    orientCov = in_06;
    velCov = in_07;
    rotRateCov = in_08;
    accelCov = in_09;
    rotAccelCov = in_10;
    wheelMeasCov = in_11;
    gyroMeasCov = in_12;
    accelMeasCov = in_13;

    // Initialize Filter Variables
    tCur = ros::Time::now();
    tPrev = ros::Time::now();
    tCur_ = ros::Time::now();
    tPrev_ = ros::Time::now();
    initPoseBool = true;
    initCount = 0;
    initTime = 0.0;
    predictBool = false;
    updateBool = false;
    gyroBool = false;
    wheelSpdBool = false;

    // Initialize State Variables
    count = 0;
    state = VectorXd::Zero(9); // update: [x,y,theta,sv_x,v_y,omega,a_x,a_y,a_omega]^T
    state_ = VectorXd::Zero(9); // prediction: [x,y,theta,sv_x,v_y,omega,a_x,a_y,a_omega]^T
    state_gt = VectorXd::Zero(9); // ground truth: [x,y,theta,v_x,v_y,omega,a_x,a_y,a_omega]^T
    y = VectorXd::Zero(9); // measurement: [w1, w2, w3, w4, w_gyro, a_x, a_y]^T
    gyro = VectorXd::Zero(3); // gyro measurement
    accel = VectorXd::Zero(3); // accelerometer measurement
    wheel_spd = VectorXd::Zero(4); // wheel speed measurement
    Sigma = 1E6*MatrixXd::Identity(9,9); // update: estimation error covariance matrix
    Sigma_ = 1E6*MatrixXd::Identity(9,9); // update: estimation error covariance matrix
    Q.diagonal() << poseCov, poseCov, orientCov, velCov, velCov, rotRateCov, accelCov, accelCov, rotAccelCov;
    R.diagonal() << wheelMeasCov, wheelMeasCov, wheelMeasCov, wheelMeasCov, gyroMeasCov, accelMeasCov, accelMeasCov;
    cout << "Check Q: " << endl << Q << endl;
    cout << "Check R: " << endl << R << endl;

  }

  void runFilter()
  {
    // Initialize Detection Timer
    //std::clock_t start = std::clock();

    // Prediction Step
    if ( predictBool ) {
    // if ( predictBool && updateBool ) {

      // Time Interval
      tCur_ = ros::Time::now();
      double dt_ = (tCur_ - tPrev_).toSec();
      if ( count==1 ) {
        dt_ = 0.0;
      }

      // Define Property Matrices For Dynamics Matrices
      MatrixXd Rwr = MatrixXd::Zero(2,2); // Active rotation watrix from world frame to local robot frame 
      Rwr <<  cos(state(2)), -sin(state(2)),
              sin(state(2)),  cos(state(2));
      MatrixXd Omega = MatrixXd::Zero(2,2);
      Omega << 0.0, -state(5), state(5), 0.0; 

      // Construct Nonlinear Continuous-Time Dynamics Matrix
      MatrixXd F_nl = MatrixXd::Zero(9,9);
      F_nl.block(0,3,2,2) = Rwr.transpose();
      F_nl(2,5) = 1.0;
      F_nl.block(3,3,2,2) = Omega.transpose();
      F_nl.block(3,6,3,3) = MatrixXd::Identity(3,3);
      F_nl.block(6,6,2,2) = Omega.transpose()*dt_;
      // Discretize Dynamics Matrix
      F_nl = F_nl.exp();
      F_nl.block(0,3,9,3) = F_nl.block(0,3,9,3)*dt_;
      F_nl.block(0,6,9,3) = F_nl.block(0,6,9,3)*dt_*dt_;

      // State Prediction
      state_ = F_nl*state;

      // Construct Linearized Continuous-Time Dynamics Matrix
      MatrixXd F = MatrixXd::Zero(9,9);
      F(0,2) = ( -sin(state(2))*state(3) - cos(state(2))*state(4) );
      F(1,2) = (  cos(state(2))*state(3) - cos(state(2))*state(4) );
      F.block(0,3,2,2) = Rwr.transpose();
      F_nl(2,5) = 1.0;
      F.block(3,3,2,2) = Omega.transpose()*dt_;
      F(3,5) = state(4)*dt_;
      F(4,5) = -state(3)*dt_;
      F.block(3,6,3,3) = MatrixXd::Identity(3,3);
      F(6,5) = state(7)*dt_*dt_;
      F(7,5) = -state(6)*dt_*dt_;
      F.block(6,6,2,2) = Omega.transpose()*dt_;
      // Discretize Dynamics Matrix
      F = F.exp();
      F.block(0,3,9,3) = F.block(0,3,9,3)*dt_;
      F.block(0,6,9,3) = F.block(0,6,9,3)*dt_*dt_;

      // EKF Covariance Prediction in Discrete Time
      // MatrixXd Q_tilde = F*Q*F.transpose()*dt_;
      MatrixXd Q_tilde = Q;
      Sigma_ = F*Sigma*F.transpose() + Q_tilde;

      // End EKF Prediction
      // predictBool = false; // TEMPORARY COMMENTED OUT
      tPrev_ = tCur_;

      if ( displayData ) {
        cout << "Prediction" << endl;
        // cout << "Time Interval: " << endl << dt_ << endl;
        // cout << "Q: " << endl << Q << endl;
        // cout << "Q_tilde: " << endl << Q_tilde << endl;
      }

    }
    else {
      state_ = state;
      Sigma_ = Sigma;
    }

    // Update Step
    if ( updateBool ) {
    // if ( predictBool && updateBool ) {

      // Time Interval
      tCur = ros::Time::now();
      //double dt = (tCur - tPrev).toSec();

      // Measurement Model
      
      // Nonlinear Measurement Matrix from Camera Calibration and True Ball Radius with Correct Camera Transformations
      MatrixXd H = MatrixXd::Zero(7,9);
      double s = sqrt(2.0);
      H(0,3) = -s/(2.0*r_w);  H(0,4) = -s/(2.0*r_w);  H(0,5) = r_b/r_w;
      H(1,3) = -s/(2.0*r_w);  H(1,4) =  s/(2.0*r_w);  H(1,5) = r_b/r_w;
      H(2,3) =  s/(2.0*r_w);  H(2,4) =  s/(2.0*r_w);  H(2,5) = r_b/r_w;
      H(3,3) =  s/(2.0*r_w);  H(3,4) = -s/(2.0*r_w);  H(3,5) = r_b/r_w;
      H.block(4,5,3,3) = MatrixXd::Identity(3,3);

      // Kalman Gain in Discrete Time
      // MatrixXd R_tilde = R/dt;
      MatrixXd R_tilde = R;
      MatrixXd S = (H*Sigma_*H.transpose() + R_tilde);
      MatrixXd KalmanK = Sigma_*H.transpose()*S.inverse();

      // EKF State Update
      state = state_ + KalmanK*(y - H*state_);

      // EKF Covariance Update
      Sigma = Sigma_ - KalmanK*H*Sigma_;
      // cout << "Sigma: " << endl << Sigma(0,0) << endl << Sigma(3,3) << endl << "-----------" << endl;

      // End EKF Update
      updateBool = false;
      // predictBool = false; // TEMPORARY HERE
      tPrev = tCur;

      if ( displayData ) {
        cout << "Update" << endl;
        // cout << "R: " << endl << R << endl;
        // cout << "R_tilde: " << endl << R_tilde << endl;
      }

    }
    else {
      state = state_;
      Sigma = Sigma_;
    }

    ros::Time pubTime = ros::Time::now();
    // Publish State Estimate Message
    // Publish Odometry Transform
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state(2));
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = pubTime;
    odom_trans.header.frame_id = "/odom_frame";
    odom_trans.child_frame_id = "/base_link";
    odom_trans.transform.translation.x = state(0);
    odom_trans.transform.translation.y = state(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    state_estimate_broadcaster.sendTransform(odom_trans);
    // ros::Duration(0.5).sleep(); // sleep for half a second
    // Set Final Pose
    stateEstimateMsg.header.stamp = pubTime;
    stateEstimateMsg.header.frame_id = "/world";
    stateEstimateMsg.pose.pose.position.x = state(0);
    stateEstimateMsg.pose.pose.position.y = state(1);
    stateEstimateMsg.pose.pose.position.z = 0.0;
    stateEstimateMsg.pose.pose.orientation = odom_quat;
    // Set Final Pose Covariance
    stateEstimateMsg.pose.covariance[0] = Sigma(0,0);
    stateEstimateMsg.pose.covariance[1] = Sigma(1,1);
    stateEstimateMsg.pose.covariance[2] = Sigma(2,2);
    // Set Final Velocity
    stateEstimateMsg.child_frame_id = "/base_link";
    stateEstimateMsg.twist.twist.linear.x = state(3);
    stateEstimateMsg.twist.twist.linear.y = state(4);
    stateEstimateMsg.twist.twist.linear.z = 0.0;
    stateEstimateMsg.twist.twist.angular.x = 0.0;
    stateEstimateMsg.twist.twist.angular.y = 0.0;
    stateEstimateMsg.twist.twist.angular.z = state(5);
        // Set Final Velocity Covariance
    stateEstimateMsg.twist.covariance[0] = Sigma(3,3);
    stateEstimateMsg.twist.covariance[1] = Sigma(4,4);
    stateEstimateMsg.twist.covariance[2] = Sigma(5,5);
    // Publish Odometry Message
    state_estimate_pub_.publish(stateEstimateMsg);

    // Display Data in Terminal
    if (displayData) {
      // cout << "Prediction: " << endl << x_ << endl;
      // cout << "Covariance: " << endl << Sigma_(0,0) << endl << Sigma_(1,1) << endl << Sigma_(2,2) << endl << Sigma_(3,3) << endl << Sigma_(4,4) << endl << Sigma_(5,5) << endl;
      // cout << "Update: " << endl << x << endl;
      // cout << "Covariance: " << endl << Sigma(0,0) << endl << Sigma(1,1) << endl << Sigma(2,2) << endl << Sigma(3,3) << endl << Sigma(4,4) << endl << Sigma(5,5) << endl;
      // cout << "------------------------------" << endl;
      // cout << "Current Robot Ground Truth: " << endl << x_rob_gt << endl;
      // cout << "Current Opponent Ground Truth: " << endl << x_opp_gt_pub << endl;
      // cout << "Current Opponent Estimate: " << endl << x_opp_est_pub << endl;
      // cout << "Ground Truth Measurement: " << endl << y_gt << endl;
      // cout << "Current Measurement: " << endl << y << endl;
      // cout << "------------------------------" << endl;
      // cout << "Relative State Ground Truth: " << endl << x_gt << endl;
      // cout << "Relative State Estimate:" << endl << x << endl;
      // cout << "------------------------------" << endl;
    }

    // Update Filter Count
    count++;
  }

  void readMeasurement()
  {

    if ( gyroBool && wheelSpdBool ) {
      // Fill Measurement Vector
      y.head(4) = wheel_spd;
      y(4) = gyro(2);
      y.tail(2) = accel.head(2);
      // Reset Measurements
      gyroBool = false;
      wheelSpdBool = false;
      // Run Filter Update
      updateBool = true;
      runFilter();
    }

  }

  void mocapCallback(const geometry_msgs::PoseStamped& msg)
  {

    if ( initPoseBool ) {

      // Extract Data
      VectorXd state_temp = VectorXd::Zero(9);
      state_temp(0) = msg.pose.position.x;
      state_temp(1) = msg.pose.position.y;
      double qx = msg.pose.orientation.x;
      double qy = msg.pose.orientation.y;
      double qz = msg.pose.orientation.z;
      double qw = msg.pose.orientation.w;
      double phiCur, theCur, psiCur;
      tf::Quaternion q(qx, qy, qz, qw);
      tf::Matrix3x3 m(q);
      m.getRPY(phiCur, theCur, psiCur);
      state_temp(2) = psiCur;
      
      // Add Poses
      tCur = ros::Time::now();
      double dt = (tCur-tPrev).toSec();
      initTime = initTime + dt;
      if ( initTime<1.0 ) {
        initCount++;
        state = state + state_temp;
      }
      else {
        // Average Poses
        state = state/initCount;
        tPrev = ros::Time::now();
        initPoseBool = false;
        predictBool = true;
        std::cout << "Initial Pose:" << endl << state << endl;
      }

    }

  }

  void imuCallback(const sensor_msgs::Imu& msg)
  {

    // Extract Data
    // double qx, qy, qz, qw;
    // qx = msg.orientation.x;
    // qy = msg.orientation.y;
    // qz = msg.orientation.z;
    // qw = msg.orientation.w;
    accel(0) = msg.linear_acceleration.x;
    accel(1) = msg.linear_acceleration.y;
    accel(2) = msg.linear_acceleration.z;
    gyro(0) = msg.angular_velocity.x;
    gyro(0) = msg.angular_velocity.y;
    gyro(0) = msg.angular_velocity.z;

    // Process EKF Measurement
    gyroBool = true;
    readMeasurement();

  }

  void wheelSpeedCallback(const ouijabot::Wheel_Spd& msg)
  {

    // Extract Data
    wheel_spd(0) = msg.wheel_spd[0];
    wheel_spd(0) = msg.wheel_spd[1];
    wheel_spd(0) = msg.wheel_spd[2];
    wheel_spd(0) = msg.wheel_spd[3];

    // Process EKF Measurement
    wheelSpdBool = true;
    readMeasurement();

  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter");
  ros::NodeHandle nh("~");
  
  int DISPLAY_DATA_BOOL;
  nh.param<int>("DISPLAY_DATA_BOOL", DISPLAY_DATA_BOOL, 0);
  int USE_OPTITRACK_BOOL;
  nh.param<int>("USE_OPTITRACK_BOOL", USE_OPTITRACK_BOOL, 0);

  double POS_COVARIANCE;
  nh.param<double>("POS_COVARIANCE", POS_COVARIANCE, 0.0);
  double ORIENT_COVARIANCE;
  nh.param<double>("ORIENT_COVARIANCE", ORIENT_COVARIANCE, 0.0);
  double VEL_COVARIANCE;
  nh.param<double>("VEL_COVARIANCE", VEL_COVARIANCE, 0.0);
  double ROT_RATE_COVARIANCE;
  nh.param<double>("ROT_RATE_COVARIANCE", ROT_RATE_COVARIANCE, 0.0);
  double ACCEL_COVARIANCE;
  nh.param<double>("ACCEL_COVARIANCE", ACCEL_COVARIANCE, 0.0);
  double ROT_ACCEL_COVARIANCE;
  nh.param<double>("ROT_ACCEL_COVARIANCE", ROT_ACCEL_COVARIANCE, 0.0);

  double WHEEL_MEASUREMENT_COVARIANCE;
  nh.param<double>("WHEEL_MEASUREMENT_COVARIANCE", WHEEL_MEASUREMENT_COVARIANCE, 0.0);
  double GYRO_MEASUREMENT_COVARIANCE;
  nh.param<double>("GYRO_MEASUREMENT_COVARIANCE", GYRO_MEASUREMENT_COVARIANCE, 0.0);
  double ACCEL_MEASUREMENT_COVARIANCE;
  nh.param<double>("ACCEL_MEASUREMENT_COVARIANCE", ACCEL_MEASUREMENT_COVARIANCE, 0.0);

  double WHEEL_RADIUS;
  nh.param<double>("WHEEL_RADIUS", WHEEL_RADIUS, 0.0);
  double BODY_RADIUS;
  nh.param<double>("BODY_RADIUS", BODY_RADIUS, 0.0);

  filterClass poseEstimation(
    DISPLAY_DATA_BOOL, 
    USE_OPTITRACK_BOOL, 
    WHEEL_RADIUS,
    BODY_RADIUS, 
    POS_COVARIANCE, 
    ORIENT_COVARIANCE, 
    VEL_COVARIANCE, 
    ROT_RATE_COVARIANCE,
    ACCEL_COVARIANCE,
    ROT_ACCEL_COVARIANCE,
    WHEEL_MEASUREMENT_COVARIANCE,
    GYRO_MEASUREMENT_COVARIANCE,
    ACCEL_MEASUREMENT_COVARIANCE
  );

  while (ros::ok()) {
    ros::spinOnce();
  }
  ros::shutdown();
  
  return 0;
}

