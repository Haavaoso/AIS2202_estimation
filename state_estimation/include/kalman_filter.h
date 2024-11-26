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
        x = A * x + B * u;
        P = A * P * A.transpose() + Q;
    }

    void posteriori(const Eigen::VectorXd &z, const Eigen::MatrixXd &r, const Eigen::MatrixXd &h) {
        Eigen::MatrixXd S = h * P * h.transpose() + r;
        K = P * h.transpose() * S.inverse();
        x = x + K * (z - h * x);
        P = (Eigen::MatrixXd::Identity(x.size(), x.size()) - K * h) * P;
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
