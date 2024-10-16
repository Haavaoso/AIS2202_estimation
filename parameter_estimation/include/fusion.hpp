//
// Created by havso on 09/10/2024.
//



#ifndef FUSION_HPP
#define FUSION_HPP

using namespace Eigen;
class Fusion {
public:
    Fusion(double mass)
        : mass(mass){
    }



    // Process experiment data: accelerometer, wrench, orientation
    void insertData(const std::vector<Vector3d>& accel_data,
                               const std::vector<VectorXd>& wrench_data,
                               const std::vector<Vector3d>& orientation_data) {
    }

private:
    double mass;
    Vector3d inertia;
    VectorXd x;  // State vector
    MatrixXd P;  // Covariance matrix
    MatrixXd A;  // State transition matrix
    MatrixXd B;  // Control input matrix
    MatrixXd Q;  // Process noise covariance
    MatrixXd R_f;  // FTS measurement noise covariance
    MatrixXd R_a;  // IMU measurement noise covariance
    MatrixXd H_f;  // FTS output matrix
    MatrixXd H_a;  // IMU output matrix

};




#endif //FUSION_HPP
