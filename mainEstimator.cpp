#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>
#include "poseEstimator.h"

using namespace std;

int main() {

    double time1;
    double steering_angle1;
    int encoder_ticks1;
    double angular_velocity1;
    vector<double> comp_est;
    vector<double> encod_est;
    vector<double> imu_est;

    ifstream inFile;
    ofstream outFile;

    //the first row of data is ignored because dt is 0 in most data sets (i.e the time stamp is zero)
    inFile.open("dataset0.txt");
    inFile >> time1 >> encoder_ticks1 >> angular_velocity1 >> steering_angle1;

    //estimator is initialized with the first row in data set
    poseEstimator test(time1, encoder_ticks1);

    outFile.open("test1.txt");

    for(int i = 0; i <= 10; i++){

        inFile >> time1 >> encoder_ticks1 >> angular_velocity1 >> steering_angle1;

        comp_est = test.estimate(time1, steering_angle1, encoder_ticks1, angular_velocity1);
        encod_est = test.encoderPose();
        imu_est = test.imuPose();

        outFile <<comp_est[0] <<" " << comp_est[1] <<" "<< comp_est[2] <<" "
                <<encod_est[0] <<" " << encod_est[1] <<" "<< encod_est[2] <<" "
                <<imu_est[0] <<" " << imu_est[1] <<" "<< imu_est[2] <<endl;

    }

    inFile.close();
    outFile.close();

    return 0;
}
