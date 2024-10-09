#ifndef ESTIMATION_KALMAN_FILTER_H
#define ESTIMATION_KALMAN_FILTER_H


namespace estimation {
class kalman_filter
{
public:
    kalman_filter(
            const Eigen::VectorXd& x_in,
            const Eigen::MatrixXd& P_in,
            const Eigen::MatrixXd& A_in,
            const Eigen::MatrixXd& B_in,
            const Eigen::MatrixXd& H_in,
            const Eigen::MatrixXd& Q_in,
            const Eigen::MatrixXd& R_in
    );
    void predict(const Eigen::VectorXd& u);
    void update(const Eigen::VectorXd& z);
    Eigen::VectorXd x;     // State vector
    Eigen::MatrixXd P;     // Covariance matrix
    Eigen::MatrixXd A;     // State transition model
    Eigen::MatrixXd B;     // Control input model
    Eigen::MatrixXd H;     // Measurement model
    Eigen::MatrixXd Q;     // Process noise covariance
    Eigen::MatrixXd R;     // Measurement noise covariance
    Eigen::MatrixXd K;     // Kalman gain
};
}

#endif
