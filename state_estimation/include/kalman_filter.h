#ifndef ESTIMATION_KALMAN_FILTER_H
#define ESTIMATION_KALMAN_FILTER_H
#include <iostream>

class kalman_filter {
public:
    kalman_filter(
        const Eigen::VectorXd &x_in,
        const Eigen::MatrixXd &P_in,
        const Eigen::MatrixXd &A_in,
        const Eigen::MatrixXd &B_in,
        const Eigen::MatrixXd &H_in,
        const Eigen::MatrixXd &Q_in,
        const Eigen::MatrixXd &R_in
    ) {
        x = x_in;
        P = P_in;
        A = A_in;
        B = B_in;
        H = H_in;
        Q = Q_in;
        R = R_in;
    }

    void priori(const Eigen::Vector3d &u, const Eigen::MatrixXd& Q) {
        x = A * x + B * u; // Predicted state estimate
        P = A * P * A.transpose() + Q; // Predicted covariance estimate
        //std::cout << "P: " << P.rows() << "x" << P.cols() << std::endl;
    }

    void posteriori(const Eigen::VectorXd &z, const Eigen::MatrixXd &r, const Eigen::MatrixXd &h) {
        //std::cout << "H: " << h.rows() << "x" << h.cols() << std::endl;
        //std::cout << "R: " << r.rows() << "x" << r.cols() << std::endl;
        Eigen::MatrixXd S = h * P * h.transpose() + r; // Innovation covariance
        //std::cout << "S: " << S.rows() << "x" << S.cols() << std::endl;
        K = P * h.transpose() * S.inverse()*0.1; // Kalman gain
        //std::cout << "K = " << K << "\n" << std::endl;
        x = x + K * (z - h * x); // Updated state estimate
        P = (Eigen::MatrixXd::Identity(x.size(), x.size()) - K * h) * P; // Updated covariance
    }

    Eigen::VectorXd getX() {
        return x;
    }

    Eigen::VectorXd x; // State vector
    Eigen::MatrixXd P; // Covariance matrix
    Eigen::MatrixXd A; // State transition model
    Eigen::MatrixXd B; // Control input model
    Eigen::MatrixXd H; // Measurement model
    Eigen::MatrixXd Q; // Process noise covariance
    Eigen::MatrixXd R; // Measurement noise covariance
    Eigen::MatrixXd K; // Kalman gain
};


#endif
