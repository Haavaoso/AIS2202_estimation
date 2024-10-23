
#include <Eigen/Dense>
#include "include/kalman_filter.h"


kalman_filter::kalman_filter(
        const Eigen::VectorXd& x_in,
        const Eigen::MatrixXd& P_in,
        const Eigen::MatrixXd& A_in,
        const Eigen::MatrixXd& B_in,
        const Eigen::MatrixXd& H_in,
        const Eigen::MatrixXd& Q_in,
        const Eigen::MatrixXd& R_in
)
{
    x = x_in;
    P = P_in;
    A = A_in;
    B = B_in;
    H = H_in;
    Q = Q_in;
    R = R_in;
}

void kalman_filter::predict(const Eigen::VectorXd& u){
    x = A * x + B * u;     // Predicted state estimate
    P = A * P * A.transpose() + Q;  // Predicted covariance estimate
}

void kalman_filter::update(const Eigen::VectorXd& z){
    Eigen::MatrixXd S = H * P * H.transpose() + R;  // Innovation covariance
    K = P * H.transpose() * S.inverse();            // Kalman gain
    x = x + K * (z - H * x);                        // Updated state estimate
    P = (Eigen::MatrixXd::Identity(x.size(), x.size()) - K * H) * P; // Updated covariance
}
