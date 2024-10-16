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
    void insertData(const rapidcsv::Document& accel,
        const rapidcsv::Document& wrench,
        const rapidcsv::Document& orientation) {

        /*accel_data_ = std::vector(accel.GetColumnCount(), std::vector<double>(accel.GetRowCount()));
        wrench_data_ = std::vector(wrench.GetColumnCount(), std::vector<double>(wrench.GetRowCount()));
        orientation_data_ = std::vector(orientation.GetColumnCount(), std::vector<double>(orientation.GetRowCount()));*/

        accel_data_ = std::vector<std::vector<double>>(accel.GetColumnCount(), std::vector<double>(accel.GetRowCount()));
        wrench_data_ = std::vector<std::vector<double>>(wrench.GetColumnCount(), std::vector<double>(wrench.GetRowCount()));
        orientation_data_ = std::vector<std::vector<double>>(orientation.GetColumnCount(), std::vector<double>(orientation.GetRowCount()));


        for (int i = 0; i < accel.GetColumnCount(); i++) {
            accel_data_[i] = accel.GetColumn<double>(i);
        }
        for (int i = 0; i < wrench.GetColumnCount(); i++) {
            wrench_data_[i] = wrench.GetColumn<double>(i);
        }
        for (int i = 0; i < orientation.GetColumnCount(); i++) {
            orientation_data_[i] = orientation.GetColumn<double>(i);
        }
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


    std::vector<std::vector<double>> accel_data_{};
    std::vector<std::vector<double>> wrench_data_{};
    std::vector<std::vector<double>> orientation_data_{};
};




#endif //FUSION_HPP
