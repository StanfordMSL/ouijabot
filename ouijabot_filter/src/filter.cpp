/*--------------------------------------------------------------------------
 
 File Name:         filter.cpp
 Date Created:      2017/08/07
 Date Modified:     2017/09/08
 
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
// #include <geometry_msgs/Accel.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <ouijabot/Wheel_Spd.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions> // matrix exponential

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class filterClass {
    
  ros::NodeHandle nh_;
  
  ros::Subscriber process_imu_sub_;
  ros::Subscriber process_wheel_speed_sub_;

  ros::Subscriber ground_truth_sub_;
  // ros::Subscriber ground_truth_accel_sub_;

  ros::Publisher state_estimate_pub_;
  tf::TransformBroadcaster state_estimate_broadcaster;
  nav_msgs::Odometry stateEstimateMsg;
  
  // Initialalize Input Variables
  int displayData, useOptitrack;
  double r_w, r_b;
  VectorXd q_cov_diag, r_cov_diag;

  // Initialize Filter Variables
  int initCount;
  double count, initTime;
  bool initPoseBool, predictBool, updateBool, gyroBool, wheelSpdBool;
  ros::Time timeCur, timePrev;
  VectorXd state, state_, state_gt, y, gyro, accel, wheel_spd;
  MatrixXd Sigma, Sigma_, Q, R;
  MatrixXd R_rl;
    
public:
  filterClass(int in_01, int in_02, double in_03, double in_04, vector<double> in_05, vector<double> in_06, vector<double> in_07) {

    // Subscribe to Ouijibot IMU
    process_imu_sub_ = nh_.subscribe("/ouijabot/imu", 10, &filterClass::imuCallback, this);

    // Subscribe to Ouijibot Wheel Encoders
    process_wheel_speed_sub_ = nh_.subscribe("/ouijabot/wheel_spd", 10, &filterClass::wheelSpeedCallback, this);

    // Publish Odometry Message of State Estimate
    state_estimate_pub_ = nh_.advertise<nav_msgs::Odometry>("/robot/state_estimate", 10);

    // Initialize Variables
    displayData = in_01;
    useOptitrack = in_02;
    r_w = in_03;
    r_b = in_04;
    q_cov_diag = VectorXd::Zero(9);
    for ( int i=0; i<9; i++ ) {
      q_cov_diag(i) = in_05[i];
    }
    r_cov_diag = VectorXd::Zero(7);
    for ( int i=0; i<7; i++ ) {
      r_cov_diag(i) = in_06[i];
    }
    MatrixXd R_temp = MatrixXd::Zero(3,3);
    for ( int i=0; i<3; i++ ) {
      for ( int j=0; j<3; j++) {
        R_temp(i,j) = in_07[i*3+j];
      }
    }
    R_rl = MatrixXd(2,2);
    R_rl = R_temp.block(0,0,2,2);

    // Subscribe to Optitrack Ground Truth Topics
    // Required mocap_interface_odom node to be publishing "ground truth" odometry data (position, derived velocity, and derived acceleration)
    initPoseBool = false;
    if ( useOptitrack ) {
      initPoseBool = true; // Initialize the filter with current Optitrack measurement
      ground_truth_sub_ = nh_.subscribe("/robot/pose", 10, &filterClass::odomCallback, this);
      // ground_truth_accel_sub_ = nh_.subscribe("/robot/accel", 10, &filterClass::odomAccelCallback, this);
    }

    // Initialize Filter Variables
    timeCur = ros::Time::now();
    timePrev = ros::Time::now();
    initCount = 0;
    initTime = 0.0;
    predictBool = false;
    updateBool = false;
    gyroBool = false;
    wheelSpdBool = false;

    // Initialize State Variables
    count = 0;
    state = VectorXd::Zero(9); // update: [x,y,theta,v_x,v_y,omega,a_x,a_y,a_omega]^T
    state_ = VectorXd::Zero(9); // prediction: [x,y,theta,v_x,v_y,omega,a_x,a_y,a_omega]^T
    state_gt = VectorXd::Zero(9); // ground truth: [x,y,theta,v_x,v_y,omega,a_x,a_y,a_omega]^T
    y = VectorXd::Zero(7); // measurement: [w1, w2, w3, w4, w_gyro, a_x, a_y]^T
    gyro = VectorXd::Zero(3); // gyro measurement
    accel = VectorXd::Zero(3); // accelerometer measurement
    wheel_spd = VectorXd::Zero(4); // wheel speed measurement
    Q = q_cov_diag.asDiagonal(); // dynamics covariance matrix
    R = r_cov_diag.asDiagonal(); // measurement covariance matrix
    Sigma = 1E6*MatrixXd::Identity(9,9); // update: estimation error covariance matrix
    Sigma_ = 1E6*MatrixXd::Identity(9,9); // update: estimation error covariance matrix

    // double phiCur, theCur, psiCur;
    // psiCur = M_PI/2.0;
    // theCur = 0.0;
    // phiCur = M_PI;
    // MatrixXd R_phi = MatrixXd::Zero(3,3);
    // MatrixXd R_the = MatrixXd::Zero(3,3);
    // MatrixXd R_psi = MatrixXd::Zero(3,3);
    // R_phi <<  1.0    ,   0.0         ,   0.0         ,
    //           0.0    ,   cos(phiCur) ,   -sin(phiCur),
    //           0.0    ,   sin(phiCur) ,   cos(phiCur) ;
    // R_the <<  cos(theCur)    ,   0.0 ,   sin(theCur) ,
    //           0.0            ,   1.0 ,   0.0         ,
    //           -sin(theCur)   ,   0.0 ,   cos(theCur) ;
    // R_psi <<  cos(psiCur)    ,   -sin(psiCur),   0.0,
    //           sin(psiCur)    ,   cos(psiCur) ,   0.0,
    //           0.0            ,   0.0         ,   1.0;
    // // Quad Orientation
    // MatrixXd R_test = MatrixXd::Zero(3,3);
    // R_test = R_psi*R_the*R_phi;
    // cout << "ROTATION TEST:" << endl << R_test;

  }

  void runFilter()
  {
    // Initialize Detection Timer
    //std::clock_t start = std::clock();

    // Time Interval
    timeCur = ros::Time::now();
    if ( count==0 ) {
      timePrev = ros::Time::now();
    }
    double dt = (timeCur - timePrev).toSec();

    // Prediction Step
    // if ( predictBool ) {
    // if ( predictBool && updateBool ) {
    if ( updateBool ) {

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
      F_nl.block(3,3,2,2) = Omega.transpose()*dt;
      F_nl.block(3,6,3,3) = MatrixXd::Identity(3,3);
      F_nl.block(6,6,2,2) = Omega.transpose()*dt;
      // Discretize Dynamics Matrix
      F_nl = F_nl.exp();
      F_nl.block(0,3,3,3) = F_nl.block(0,3,3,3)*dt;
      F_nl.block(0,6,3,3) = F_nl.block(0,6,3,3)*dt*dt;
      F_nl.block(3,6,3,3) = F_nl.block(3,6,3,3)*dt;

      // State Prediction
      state_ = F_nl*state;

      // Construct Linearized Continuous-Time Dynamics Matrix
      MatrixXd F = MatrixXd::Zero(9,9);
      F(0,2) = ( -sin(state(2))*state(3) - cos(state(2))*state(4) )*dt;
      F(1,2) = (  cos(state(2))*state(3) - cos(state(2))*state(4) )*dt;
      F.block(0,3,2,2) = Rwr.transpose();
      F_nl(2,5) = 1.0;
      F.block(3,3,2,2) = Omega.transpose()*dt;
      F(3,5) = state(4)*dt;
      F(4,5) = -state(3)*dt;
      F.block(3,6,3,3) = MatrixXd::Identity(3,3);
      F(6,5) = state(7)*dt;
      F(7,5) = -state(6)*dt;
      F.block(6,6,2,2) = Omega.transpose()*dt;
      // Discretize Dynamics Matrix
      F = F.exp();
      F.block(0,3,3,3) = F.block(0,3,3,3)*dt;
      F.block(0,6,3,3) = F.block(0,6,3,3)*dt*dt;
      F.block(3,6,3,3) = F.block(3,6,3,3)*dt;

      // EKF Covariance Prediction in Discrete Time
      // MatrixXd Q_tilde = F*Q*F.transpose()*dt;
      MatrixXd Q_tilde = Q;
      Sigma_ = F*Sigma*F.transpose() + Q_tilde;

      // End EKF Prediction
      // predictBool = false; // TEMPORARY COMMENTED OUT
      timePrev = timeCur;

      if ( displayData ) {
        cout << "Prediction" << endl;
        cout << "Time Interval: " << endl << dt << endl;
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

      // Linear Measurement Matrix
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

    // Convert to Global Reference Frame
    // Somewhat guessing on the transformations for the moment (see negative sign in gyro as well)
    VectorXd state_output = VectorXd::Zero(9);
    state_output.segment(0,2) = -R_rl*state.segment(0,2);
    state_output(2) = state(2);
    state_output.segment(3,2) = -R_rl*state.segment(3,2);
    state_output(5) = state(5);
    state_output.segment(6,2) = -R_rl*state.segment(6,2);
    state_output(8) = state(8);

    // Publish State Estimate Message
    ros::Time pubTime = ros::Time::now();
    // Publish Odometry Transform
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state_output(2));
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = pubTime;
    odom_trans.header.frame_id = "/odom_frame_est";
    odom_trans.child_frame_id = "/base_link_est";
    odom_trans.transform.translation.x = state_output(0);
    odom_trans.transform.translation.y = state_output(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    state_estimate_broadcaster.sendTransform(odom_trans);
    // ros::Duration(0.5).sleep(); // sleep for half a second
    // Set Final Pose
    stateEstimateMsg.header.stamp = pubTime;
    stateEstimateMsg.header.frame_id = "/odom_frame_est";
    stateEstimateMsg.pose.pose.position.x = state_output(0);
    stateEstimateMsg.pose.pose.position.y = state_output(1);
    stateEstimateMsg.pose.pose.position.z = 0.0;
    stateEstimateMsg.pose.pose.orientation = odom_quat;
    // Set Final Pose Covariance
    stateEstimateMsg.pose.covariance[0] = Sigma(0,0);
    stateEstimateMsg.pose.covariance[1] = Sigma(1,1);
    stateEstimateMsg.pose.covariance[2] = Sigma(2,2);
    // Set Final Velocity
    stateEstimateMsg.child_frame_id = "/base_link_est";
    stateEstimateMsg.twist.twist.linear.x = state_output(3);
    stateEstimateMsg.twist.twist.linear.y = state_output(4);
    stateEstimateMsg.twist.twist.linear.z = 0.0;
    stateEstimateMsg.twist.twist.angular.x = 0.0;
    stateEstimateMsg.twist.twist.angular.y = 0.0;
    stateEstimateMsg.twist.twist.angular.z = state_output(5);
        // Set Final Velocity Covariance
    stateEstimateMsg.twist.covariance[0] = Sigma(3,3);
    stateEstimateMsg.twist.covariance[1] = Sigma(4,4);
    stateEstimateMsg.twist.covariance[2] = Sigma(5,5);

    // Publish Odometry Message
    state_estimate_pub_.publish(stateEstimateMsg);

    // Display Data in Terminal
    if (displayData) {
      cout << "Prediction: " << endl << state_ << endl;
      // cout << "Covariance: " << endl << Sigma_(0,0) << endl << Sigma_(1,1) << endl << Sigma_(2,2) << endl << Sigma_(3,3) << endl << Sigma_(4,4) << endl << Sigma_(5,5) << endl;
      cout << "Update: " << endl << state << endl;
      // cout << "Covariance: " << endl << Sigma(0,0) << endl << Sigma(1,1) << endl << Sigma(2,2) << endl << Sigma(3,3) << endl << Sigma(4,4) << endl << Sigma(5,5) << endl;
      // cout << "------------------------------" << endl;
      // cout << "Current Ground Truth: " << endl << state_gt << endl;
      // cout << "Current Estimate: " << endl << state << endl;
      cout << "Current Measurement: " << endl << y << endl;
      cout << "------------------------------" << endl;
    }

    // Update Filter Count
    count++;
  }

  void readMeasurement()
  {
    if ( gyroBool && wheelSpdBool && !initPoseBool ) {
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

  void imuCallback(const sensor_msgs::Imu& msg)
  {
    // Extract Data
    // double qx, qy, qz, qw;
    // qx = msg.orientation.x;
    // qy = msg.orientation.y;
    // qz = msg.orientation.z;
    // qw = msg.orientation.w;
    // Accelerometer is positioned with a rotation of pi about the z-axis!
    accel(0) = -msg.linear_acceleration.x;
    accel(1) = -msg.linear_acceleration.y;
    accel(2) = msg.linear_acceleration.z;
    // Filter Accelerometer Noise
    if ( abs(accel(0)) < 0.25 ) accel(0) = 0;
    if ( abs(accel(1)) < 0.25 ) accel(1) = 0;
    if ( abs(accel(2)) < 0.25 ) accel(2) = 0;
    // Convert to gyro measurement to radians/second
    gyro(0) = msg.angular_velocity.x*M_PI/180.0;
    gyro(1) = msg.angular_velocity.y*M_PI/180.0;
    gyro(2) = -msg.angular_velocity.z*M_PI/180.0; // need negative sign here with negatives in final output
    // Process EKF Measurement
    gyroBool = true;
    readMeasurement();
  }

  void wheelSpeedCallback(const ouijabot::Wheel_Spd& msg)
  {
    // Extract Data
    // Convert from linear velocity to angular velocity (radians/second) with wheel radius
    wheel_spd(0) = msg.wheel_spd[0]/r_w;
    wheel_spd(1) = msg.wheel_spd[1]/r_w;
    wheel_spd(2) = msg.wheel_spd[2]/r_w;
    wheel_spd(3) = msg.wheel_spd[3]/r_w;
    // Process EKF Measurement
    wheelSpdBool = true;
    readMeasurement();
  }

  void odomCallback(const geometry_msgs::PoseStamped& msg)
  {
    if ( useOptitrack ) {
      // Extract Position
      VectorXd state_temp = VectorXd::Zero(9);
      state_temp(0) = msg.pose.position.x;  // x
      state_temp(1) = msg.pose.position.y;  // x
      double qx = msg.pose.orientation.x;
      double qy = msg.pose.orientation.y;
      double qz = msg.pose.orientation.z;
      double qw = msg.pose.orientation.w;
      double phiCur, theCur, psiCur;
      tf::Quaternion q(qx, qy, qz, qw);
      tf::Matrix3x3 m(q);
      m.getRPY(phiCur, theCur, psiCur);
      state_temp(2) = psiCur;               // theta
      // // Extract Velocity
      // state_temp(3) = msg.pose.orinetation.linear.x;   // v_x
      // state_temp(4) = msg.pose.orinetation.linear.y;   // v_y
      // state_temp(5) = msg.pose.orinetation.angular.z;  // omega
      // Compute Initial Pose
      if ( initPoseBool ) {
        if ( initCount==0) {
          timePrev = ros::Time::now();
        }
        timeCur = ros::Time::now();
        double dt = (timeCur-timePrev).toSec();
        initTime = initTime + dt;
        if ( initTime<10.0 ) {
          initCount++;
          state = state + state_temp;
        }
        else {
          // Average Poses
          state = state/double(initCount);
          // Convert to Local Robot Frame
          // Somewhat guessing on the transformations for the moment (see negative sign in gyro as well)
          VectorXd state_output = VectorXd::Zero(9,1);
          state.segment(0,2) = -R_rl.transpose()*state.segment(0,2);
          state(2) = state(2);
          // Finish Up
          timePrev = ros::Time::now();
          initPoseBool = false;
          predictBool = true;
          cout << "ouijabot_filter: initial mean pose after " << double(initCount) << " iterations and " << initTime << " seconds: " << endl << state << endl;
        }
      }
      else {
        state_gt.head(6) = state_temp.head(6);
      }
    }
  }

  // void odomAccelCallback(const geometry_msgs::Accel& msg) {
  //   if ( useOptitrack ) {
  //     state_gt(6) = msg.linear.x;
  //     state_gt(7) = msg.linear.y;
  //     state_gt(8) = msg.angular.z;
  //   }
  // }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter");
  ros::NodeHandle nh("~");
  
  int display_data_flag;
  nh.param<int>("display_data_flag", display_data_flag, 0);
  int use_optitrack_flag;
  nh.param<int>("use_optitrack_flag", use_optitrack_flag, 0);

  double wheel_radius;
  nh.getParam( "wheel_radius", wheel_radius );
  if (!nh.hasParam("wheel_radius")) {
    wheel_radius = 0.025;
  }
  double body_radius;
  nh.getParam( "body_radius", body_radius );
  if (!nh.hasParam("body_radius")) {
    body_radius = 0.1;
  }

  std::vector<double> q_cov_diag;
  nh.getParam( "q_cov_diag", q_cov_diag );
  if (!nh.hasParam("q_cov_diag")) {
    q_cov_diag = {0.0001, 0.0001, 0.0001, 0.001, 0.001, 0.001, 0.01, 0.01, 0.01};
  }
  std::vector<double> r_cov_diag;
  nh.getParam( "r_cov_diag", r_cov_diag );
  if (!nh.hasParam("r_cov_diag")) {
    r_cov_diag = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
  }

  std::vector<double> rotation_robot_local;
  nh.getParam( "rotation_robot_local", rotation_robot_local );
  if (!nh.hasParam("rotation_robot_local")) {
    rotation_robot_local = {0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0};
  }

  filterClass filter(
    display_data_flag,
    use_optitrack_flag,
    wheel_radius,
    body_radius,
    q_cov_diag,
    r_cov_diag,
    rotation_robot_local
  );

  ros::Rate r(20); // 20 hz
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();
  
  return 0;
}

