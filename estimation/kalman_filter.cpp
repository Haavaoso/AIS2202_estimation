#include <iostream>
#include <vector>
#include "rapidcsv.h"
#include <Eigen/Dense>
#include "include/kalman_filter.h"
#include "include/variance.h"


int main() {
    using namespace estimation;


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




    rapidcsv::Document doc0("../dataset/0-steady-state_accel.csv");
    rapidcsv::Document doc1("../dataset/0-steady-state_wrench.csv");

    double varAx = variance(doc0, 0);
    double varAy = variance(doc0, 1);
    double varAz = variance(doc0, 2);

    double varFx = variance(doc0, 2);
    double varFy = variance(doc0, 1);
    double varFz = variance(doc0, 2);
    double varTx = variance(doc0, 3);
    double varTy = variance(doc0, 4);
    double varTz = variance(doc0, 5);



    return 0;


}
