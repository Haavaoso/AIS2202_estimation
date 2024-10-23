//
// Created by havso on 23/10/2024.
//

#ifndef KOMPIS_HPP
#define KOMPIS_HPP
#include <Eigen/Dense>
#include <vector>
#include <iostream>

using namespace Eigen;

class Fusion {
public:
    Fusion(double mass, const Vector3d& r)
        : mass_(mass), mass_center_(r) {
        initializeMatrices();
    }

    void initializeMatrices() {
        // System transition matrix, A
        A_ = MatrixXd::Identity(9, 9); // 9x9 Identity matrix

        // Input matrix, B
        B_ = MatrixXd::Zero(9, 3);
        B_.block<3, 3>(0, 0) = Matrix3d::Identity();
        B_.block<3, 3>(3, 0) = mass_ * Matrix3d::Identity();
        B_.block<3, 3>(6, 0) = mass_ * skewSymmetric(mass_center_);

        // Initialize process noise covariance matrix, Q
        Q_ = MatrixXd::Zero(9, 9);
        updateProcessNoise();

        // Measurement noise covariance matrix, R (assuming known sensor noise characteristics)
        R_ = MatrixXd::Zero(6, 6); // Adjust size according to your measurement vector
        R_.topLeftCorner<3, 3>() = Matrix3d::Identity() * 0.01; // Example values
        R_.bottomRightCorner<3, 3>() = Matrix3d::Identity() * 0.02;
    }

    void updateProcessNoise(double delta_t = 1.0, double sigma_k = 0.1) {
        // Process noise covariance Q update
        Q_.topLeftCorner<3, 3>() = delta_t * Matrix3d::Identity() * sigma_k;
        Q_.block<3, 3>(3, 3) = delta_t * mass_ * Matrix3d::Identity() * sigma_k;
        Q_.block<3, 3>(6, 6) = delta_t * mass_ * mass_center_.norm() * Matrix3d::Identity() * sigma_k;
    }

    Matrix3d skewSymmetric(const Vector3d& v) {
        Matrix3d skew;
        skew << 0, -v.z(), v.y(),
                v.z(), 0, -v.x(),
                -v.y(), v.x(), 0;
        return skew;
    }

    // Simulate update step of Kalman filter
    void update(VectorXd& x, const VectorXd& u, const VectorXd& z) {
        // Predict
        VectorXd x_pred = A_ * x + B_ * u;
        MatrixXd P_pred = A_ * P_ * A_.transpose() + Q_;

        // Update
        MatrixXd K = P_pred * H_.transpose() * (H_ * P_pred * H_.transpose() + R_).inverse();
        x = x_pred + K * (z - H_ * x_pred);
        P_ = (MatrixXd::Identity(x.size(), x.size()) - K * H_) * P_pred;
    }

private:
    double mass_;
    Vector3d mass_center_;
    MatrixXd A_, B_, Q_, R_, P_;
    MatrixXd H_; // Output matrix needs to be defined according to your sensor output
};

int main() {
    // Example usage
    Fusion fusion(5.0, Vector3d(1.0, 0.0, 0.0));
    VectorXd x = VectorXd::Zero(9); // State vector
    VectorXd u = VectorXd::Zero(3); // Control input
    VectorXd z = VectorXd::Zero(6); // Measurement vector
    fusion.update(x, u, z);

    return 0;
}

#endif //KOMPIS_HPP
