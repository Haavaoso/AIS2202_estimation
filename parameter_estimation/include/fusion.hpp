#include <utility>

//
// Created by havso on 09/10/2024.
//




#ifndef FUSION_HPP
#define FUSION_HPP

using namespace Eigen;
class Fusion {
public:
    Fusion(double mass, Vector3d massCenterEstimate, std::vector<double> forceVariance,
        std::vector<double> torqueVariance, std::vector<double> accelVariance)
        : mass_(mass), mass_center_(std::move(massCenterEstimate)), varF_(forceVariance), varT_(torqueVariance), varA_(accelVariance) {

        A_ = MatrixXd::Identity(9, 9);

        B_.block<3,3>(0,0) =  MatrixXd::Identity(3, 3);
        B_.block<3,3>(3,0) =  mass*MatrixXd::Identity(3, 3);
        B_.block<3,3>(6,0) =  mass*skewSymmetric(massCenterEstimate);

        Q_base_matrix_.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);

        Q_base_matrix_.block<3, 3>(3, 3) = mass_ * Matrix3d::Identity(3, 3);

        double fuck_trippel_trumf = sqrt(mass_center_[0]*mass_center_[0] + mass_center_[1]*mass_center_[1] + mass_center_[2]*mass_center_[2]);

        Q_base_matrix_.block<3, 3>(6, 6) = mass_ * fuck_trippel_trumf *  Matrix3d::Identity(3, 3);

        H_f_.block<3, 3>(0,3) = MatrixXd::Identity(3, 3); //BØR ETTERSEES
        H_f_.block<3, 3>(3,6) = MatrixXd::Identity(3, 3);
        H_a_.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3); //BØR ETTERSEES


        R_a_ = MatrixXd::Identity(3, 3); // Sensor varians
        R_f_ = MatrixXd::Identity(6, 6);

        R_a_ << s_a_*varA_[0], 0, 0,
           0, s_a_*varA_[1], 0,
           0, 0, s_a_*varA_[2];

        R_f_ << s_f_*varF_[0], 0, 0, 0, 0, 0,
            0, s_f_*varF_[1], 0, 0, 0, 0,
            0, 0, s_f_*varF_[2], 0, 0, 0,
            0, 0, 0, s_t_*varT_[0], 0, 0,
            0, 0, 0, 0, s_t_*varT_[1], 0,
            0, 0, 0, 0, 0, s_t_*varT_[2];

        R_.block<3, 3>(0, 0) = R_a_;
        R_.block<6, 6>(3, 3) = R_f_;

        //Rotationmatrix from imu to fts frame
        RotationMatrix_IMU_to_FTS_ <<
            0, 0, -1,
            -1, 0, 0,
            0, 1, 0;

        x_ = VectorXd::Zero(9);


        Z_c_.block<3, 3>(0, 0) = -mass_*MatrixXd::Identity(3, 3); // Sensor mapping matrise
        Z_c_.block<3, 3>(3, 0) = MatrixXd::Identity(3, 3);
        Z_c_.block<3, 1>(3, 0) = -mass_*mass_center_;
        Z_c_.block<3, 3>(3, 6) = MatrixXd::Identity(3, 3);
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

    Matrix3d skewSymmetric(const Eigen::Vector3d& v) { // SPØR OM DITTA E RIKTIG=!!=!=!
        Eigen::Matrix3d skew;
        skew << 0,      -v.z(),   v.y(),
                v.z(),   0,      -v.x(),
               -v.y(),  v.x(),    0;
        return skew;
    }


    void updateMatrix(int i) {
        time_step_ = accel_data_[0][i] - prev_time_;
        std::cout << "Check 1" << std::endl;
        Q_ = time_step_ * Q_base_matrix_ * sigmak_;
        std::cout << "Check 2" << std::endl;

        //da imu ikkje er rotert samme måte som fts må ditta skje.
        Vector3d thisIterationAccelData { accel_data_[1][i], accel_data_[2][i], accel_data_[3][i]};
        std::cout << "Check 3" << std::endl;
        Vector3d thisIterationAccelData_FTS_frame = thisIterationAccelData.transpose()*RotationMatrix_IMU_to_FTS_;
        std::cout << "Check 4" << std::endl;

        //Oppdater tilstandsvariabel for nåverande iterasjon
        x_ << thisIterationAccelData_FTS_frame[0], thisIterationAccelData_FTS_frame[1], thisIterationAccelData_FTS_frame[2],
        wrench_data_[1][i], wrench_data_[2][i], wrench_data_[3][i],
        wrench_data_[4][i], wrench_data_[5][i], wrench_data_[6][i];
        std::cout << "Check 5" << std::endl;

        control_input_ = (thisIterationAccelData_FTS_frame - previousIterationAccelData_FTS_frame)*frequency_scalar_;
        std::cout << "Check 6" << std::endl;


        Z_ = Z_c_*x_;
        std::cout << "Check 7" << std::endl;
        previousIterationAccelData_FTS_frame = thisIterationAccelData_FTS_frame;
        std::cout << "Check 8" << std::endl;
        prev_time_ = accel_data_[0][i];
        std::cout << "Check 9" << std::endl;
    }

    MatrixXd getZ() {
        return Z_;
    }

    MatrixXd getQ() {
        return Q_;
    }

    Vector3d getU() {
        return control_input_;
    }

    MatrixXd getA() {
        return A_;
    }

    MatrixXd getB() {
        return B_;
    }

    MatrixXd getH() {
        return H_;
    }

    MatrixXd getR() {
        return R_;
    }
    VectorXd getX() {
        return x_;
    }




private:
    double mass_;
    double time_step_{};
    double prev_time_ = 0.0;
    double s_f_ = 250;
    double s_t_ = 5000;
    double s_a_ = 100;
    double f_r_ = 100.2;
    double f_f_ = 698.3;
    double f_a_ = 254.3;
    double frequency_scalar_ = f_r_/(f_f_+f_a_);


    Vector3d mass_center_;
    Vector3d inertia;
    VectorXd x_;  // State vector
    Vector3d control_input_ = Vector3d::Zero(3);
    MatrixXd P_;  // Covariance matrix
    MatrixXd A_;  // State transition matrix
    MatrixXd B_ = MatrixXd::Zero(9, 3);  // Control input matrix

    MatrixXd Q_base_matrix_ = MatrixXd::Zero(9, 9);
    MatrixXd Q_ = MatrixXd::Zero(9, 9);  // Process noise covariance

    MatrixXd R_f_;  // FTS measurement noise covariance
    MatrixXd R_a_;  // IMU measurement noise covariance
    MatrixXd R_ =  MatrixXd::Zero(9, 9);

    MatrixXd H_f_ = MatrixXd::Zero(6, 9);  // FTS output matrix
    MatrixXd H_a_ = MatrixXd::Zero(3, 9);  // IMU output matrix
    MatrixXd H_ = MatrixXd::Zero(6, 9);  // IMU output matrix

    MatrixXd Z_c_ = MatrixXd::Zero(6, 9);  // IMU output matrix
    MatrixXd Z_ = MatrixXd::Zero(6, 1);  // sensor matrise

    Matrix3d RotationMatrix_IMU_to_FTS_ = Matrix3d::Identity(3, 3);
    Vector3d previousIterationAccelData_FTS_frame = Vector3d::Zero(3);



    double sigmak_ = 0.5;
    std::vector<double> varF_;
    std::vector<double> varT_;
    std::vector<double> varA_;

    std::vector<std::vector<double>> accel_data_{};
    std::vector<std::vector<double>> wrench_data_{};
    std::vector<std::vector<double>> orientation_data_{};
};

#endif //FUSION_HPP
