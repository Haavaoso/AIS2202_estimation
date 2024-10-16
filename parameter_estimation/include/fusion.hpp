#include <utility>

//
// Created by havso on 09/10/2024.
//



#ifndef FUSION_HPP
#define FUSION_HPP

using namespace Eigen;
class Fusion {
public:
    Fusion(double mass, Vector3d r)
        : mass_(mass), mass_center_(std::move(r)){
    }



    // Process experiment data: accelerometer, wrench, orientation
    void insertData(const rapidcsv::Document& accel,
        const rapidcsv::Document& wrench,
        const rapidcsv::Document& orientation) {

        accel_data_ = std::vector(accel.GetColumnCount(), std::vector<double>(accel.GetRowCount()));
        wrench_data_ = std::vector(wrench.GetColumnCount(), std::vector<double>(wrench.GetRowCount()));
        orientation_data_ = std::vector(orientation.GetColumnCount(), std::vector<double>(orientation.GetRowCount()));

        /*
        accel_data_ = std::vector<std::vector<double>>(accel.GetColumnCount(), std::vector<double>(accel.GetRowCount()));
        wrench_data_ = std::vector<std::vector<double>>(wrench.GetColumnCount(), std::vector<double>(wrench.GetRowCount()));
        orientation_data_ = std::vector<std::vector<double>>(orientation.GetColumnCount(), std::vector<double>(orientation.GetRowCount()));
        */


        for (int i = 0; i < accel.GetColumnCount(); i++) {
            accel_data_[i] = accel.GetColumn<double>(i);
        }
        for (int i = 0; i < wrench.GetColumnCount(); i++) {
            wrench_data_[i] = wrench.GetColumn<double>(i);
        }
        for (int i = 0; i < orientation.GetColumnCount(); i++) {
            orientation_data_[i] = orientation.GetColumn<double>(i);
        }
        prev_time_ = accel_data_[0][0];
    }

    Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
        Eigen::Matrix3d skew;
        skew << 0,      -v.z(),   v.y(),
                v.z(),   0,      -v.x(),
               -v.y(),  v.x(),    0;
        return skew;
    }


    void updateMatrix(int i) {

        A_ = MatrixXd::Identity(9, 9);

        B_.resize(3);

        B_[0] = MatrixXd::Identity(3, 3);

        B_[1] = mass_ * MatrixXd::Identity(3, 3);
        B_[2] = mass_ * skewSymmetric(mass_center_);
        time_step_ = accel_data_[0][i] - prev_time_;

        std::cout << "Liker" << std::endl;

        Q_.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);
        std::cout << "du?" << std::endl;

        Q_.block<3, 3>(3, 3) = mass_ * Matrix3d::Identity(3, 3);
        std::cout << "Du" << std::endl;

        double fuck_trippel_trumf = sqrt(mass_center_[0]*mass_center_[0] + mass_center_[1]*mass_center_[1] + mass_center_[2]*mass_center_[2]);

        Q_.block<3, 3>(6, 6) = mass_ * fuck_trippel_trumf *  Matrix3d::Identity(3, 3);

        std::cout << "Rimming?" << std::endl;
        Q_ = time_step_ * Q_ * 1;
        //std::cout << Q_;



        H_f_.block<3, 3>(0,3) = MatrixXd::Identity(3, 3); //BØR ETTERSEES
        H_f_.block<3, 3>(3,6) = MatrixXd::Identity(3, 3);
        //std::cout << H_f_ << std::endl;


        H_a_.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3); //BØR ETTERSEES
        //std::cout << H_a_ << std::endl;








        prev_time_ = accel_data_[0][i];
    }



private:
    double mass_;
    double time_step_{};
    double prev_time_ = 0.0;

    Vector3d mass_center_;
    Vector3d inertia;
    VectorXd x_;  // State vector
    MatrixXd P_;  // Covariance matrix
    MatrixXd A_;  // State transition matrix
    std::vector<Matrix3d> B_;  // Control input matrix
    MatrixXd Q_ = MatrixXd::Zero(9, 9);  // Process noise covariance
    MatrixXd R_f;  // FTS measurement noise covariance
    MatrixXd R_a;  // IMU measurement noise covariance
    MatrixXd H_f_ = MatrixXd::Zero(9, 9);  // FTS output matrix
    MatrixXd H_a_ = MatrixXd::Zero(9, 9);  // IMU output matrix


    std::vector<std::vector<double>> accel_data_{};
    std::vector<std::vector<double>> wrench_data_{};
    std::vector<std::vector<double>> orientation_data_{};
};




#endif //FUSION_HPP
