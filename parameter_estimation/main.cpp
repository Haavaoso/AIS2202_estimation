#include <iostream>
#include <vector>
#include "rapidcsv.h"
#include <Eigen/Dense>
#include "include/ParameterEstimation.h"

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A) {
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

int main() {
    rapidcsv::Document doc("../dataset/0-calibration_fts-accel.csv");

    ParameterEstimation param(doc);
    std::cout << "CLASS: " << param.getForceBiasMatrix() << std::endl;
    std::cout << "CLASS: " << param.getTorqueBiasMatrix() << std::endl;
    std::cout << "CLASS: " << param.getMassEstimate() << std::endl;
    return 0;
}