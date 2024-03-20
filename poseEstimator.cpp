/*
   +----------------------------------------------------------------------+
   | Author: Chukwuemeka Oguike
   | Date: 3/7/2019
   +----------------------------------------------------------------------+
*/

#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include "poseEstimator.h"


const double wheel_base = 1;
const double rear_wheel_dist = 0.75;
const double front_wheel_rad = 0.2;
const double back_wheel_rad = 0.2;
const double pi = 3.142;

poseEstimator::poseEstimator(double time, double ticks) {
  previous_time = time;
  previous_ticks = ticks;
  prev_encoder_X = 0.0;
  prev_encoder_Y = 0.0;
  prev_encoder_TH = 0.0;
  prev_imu_X = 0.0;
  prev_imu_Y = 0.0;
  prev_imu_TH = 0.0;
  encoder_pose = {0, 0, 0};
  imu_pose = {0, 0, 0};

}


vector<double> poseEstimator::estimate(double time, double steering_angle, int encoder_ticks, double angular_velocity){

  double dt = time - previous_time;
  previous_time = time;

  double angular_wheel_vel = 2*pi/512 * (encoder_ticks - previous_ticks)/dt;
  previous_ticks = encoder_ticks;

  //calculate state with encodder data
  encoder_pose[2] = prev_encoder_TH + (angular_wheel_vel*front_wheel_rad*sin(steering_angle)/wheel_base)*dt;//calculate yaw (integrate yaw rate)
  encoder_pose[0] = prev_encoder_X + (angular_wheel_vel*front_wheel_rad*cos(steering_angle)*cos(encoder_pose[2]))*dt;
  encoder_pose[1] = prev_encoder_Y + (angular_wheel_vel*front_wheel_rad*cos(steering_angle)*sin(encoder_pose[2]))*dt;

  //update parameters
  prev_encoder_X = encoder_pose[0];
  prev_encoder_Y = encoder_pose[1];
  prev_encoder_TH = encoder_pose[2];

  //calculate states with imu data
  imu_pose[2] = prev_imu_TH + angular_velocity*dt;
  imu_pose[0] = prev_imu_X + ((angular_velocity*wheel_base/sin(steering_angle))*cos(steering_angle)*cos(imu_pose[2]))*dt;
  imu_pose[1] = prev_imu_Y + ((angular_velocity*wheel_base/sin(steering_angle))*cos(steering_angle)*sin(imu_pose[2]))*dt;

  //update parameters
  prev_imu_X = imu_pose[0];
  prev_imu_Y = imu_pose[1];
  prev_imu_TH = imu_pose[2];

return complementaryFilter(encoder_pose,imu_pose);

}

vector<double> poseEstimator::encoderPose(){

  return encoder_pose;
}

vector<double> poseEstimator::imuPose(){

  return imu_pose;
}

vector<double> poseEstimator::complementaryFilter(vector<double> &encoder_state, vector<double> &imu_state){

vector<double> filtered_pose(3);

double alpha = 0.05;

filtered_pose[0] = alpha*encoder_pose[0] + (1-alpha)*imu_pose[0];
filtered_pose[1] = alpha*encoder_pose[1] + (1-alpha)*imu_pose[1];
filtered_pose[2] = alpha*encoder_pose[2] + (1-alpha)*imu_pose[2];

return filtered_pose;

}
