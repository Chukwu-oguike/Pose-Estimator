/*
   +----------------------------------------------------------------------+
   | Author: Chukwuemeka Oguike
   | Date: 3/7/2019
   +----------------------------------------------------------------------+
*/

#include <math.h>
#include <vector>
#include <fstream>

using namespace std;

class poseEstimator {

  public:

      //initializes states
      poseEstimator(double time, double ticks);

      // function estimates and returns the pose of a mobile robot
      vector<double> estimate(double time, double steering_angle, int encoder_ticks, double angular_velocity);

      // function fuses pose estimates calculated from encoder with those calculated from imu
      vector<double> complementaryFilter(vector<double> &encoder_state, vector<double> &imu_state);

      //function returns the pose estimatied from encoder data
      vector<double> encoderPose();

      //function returns the pose estimated from imu data
      vector<double> imuPose();

  private:

      //variables to hold previous states
      double previous_time;
      int previous_ticks;
      double prev_encoder_X;
      double prev_encoder_Y;
      double prev_encoder_TH;
      double prev_imu_X;
      double prev_imu_Y;
      double prev_imu_TH;
      vector<double> encoder_pose;
      vector<double> imu_pose;


};
