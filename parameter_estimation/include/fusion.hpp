#include <utility>

//
// Created by havso on 09/10/2024.
//




#ifndef FUSION_HPP
#define FUSION_HPP

using namespace Eigen;
class Fusion {
public:
    Fusion(double mass, Vector3d r, std::vector<double> F, std::vector<double> T, std::vector<double> A)
        : mass_(mass), mass_center_(std::move(r)), varF_(F), varT_(T), varA_(A) {
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

        Q_.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);

        Q_.block<3, 3>(3, 3) = mass_ * Matrix3d::Identity(3, 3);

        double fuck_trippel_trumf = sqrt(mass_center_[0]*mass_center_[0] + mass_center_[1]*mass_center_[1] + mass_center_[2]*mass_center_[2]);

        Q_.block<3, 3>(6, 6) = mass_ * fuck_trippel_trumf *  Matrix3d::Identity(3, 3);

        Q_ = time_step_ * Q_ * sigmak_;



        H_f_.block<3, 3>(0,3) = MatrixXd::Identity(3, 3); //BØR ETTERSEES
        H_f_.block<3, 3>(3,6) = MatrixXd::Identity(3, 3);

        H_a_.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3); //BØR ETTERSEES


        R_a_ = MatrixXd::Identity(3, 3);
        R_f_ = MatrixXd::Identity(6, 6);
        //R_ = MatrixXd::Identity(9, 9);

        Z_c_.block<3, 3>(0, 0) = mass_*MatrixXd::Identity(3, 3);
        Z_c_.block<3, 3>(3, 0) = MatrixXd::Identity(3, 3);
        Z_c_.block<3, 1>(3, 0) = -mass_*mass_center_;
        Z_c_.block<3, 3>(3, 6) = MatrixXd::Identity(3, 3);
        std::cout << Z_c_ << std::endl;

        R_a_ << s_a*varA_[0], 0, 0,
            0, s_a*varA_[1], 0,
            0, 0, s_a*varA_[2];

        R_f_ << s_f*varF_[0], 0, 0, 0, 0, 0,
            0, s_f*varF_[1], 0, 0, 0, 0,
            0, 0, s_f*varF_[2], 0, 0, 0,
            0, 0, 0, s_t*varT_[0], 0, 0,
            0, 0, 0, 0, s_t*varT_[1], 0,
            0, 0, 0, 0, 0, s_t*varT_[2];

        R_.block<3, 3>(0, 0) = R_a_;
        R_.block<6, 6>(3, 3) = R_f_;

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

    MatrixXd R_f_;  // FTS measurement noise covariance
    MatrixXd R_a_;  // IMU measurement noise covariance
    MatrixXd R_ =  MatrixXd::Zero(9, 9);;

    MatrixXd H_f_ = MatrixXd::Zero(6, 9);  // FTS output matrix
    MatrixXd H_a_ = MatrixXd::Zero(3, 9);  // IMU output matrix
    MatrixXd H_ = MatrixXd::Zero(6, 9);  // IMU output matrix

    MatrixXd Z_c_ = MatrixXd::Zero(6, 9);  // IMU output matrix



    double sigmak_ = 0.5;
    std::vector<double> varF_;
    std::vector<double> varT_;
    std::vector<double> varA_;

    double s_a = 1;
    double s_f = 1;
    double s_t = 1;

    std::vector<std::vector<double>> accel_data_{};
    std::vector<std::vector<double>> wrench_data_{};
    std::vector<std::vector<double>> orientation_data_{};
};




#endif //FUSION_HPP
