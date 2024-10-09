#include <iostream>
#include <vector>
#include "rapidcsv.h"
#include <Eigen/Dense>
#include "include/ParameterEstimation.h"

int main() {
    rapidcsv::Document doc("../dataset/0-calibration_fts-accel.csv");

    ParameterEstimation param(doc);
    /*
    std::cout << "CLASS: " << param.getForceBiasMatrix() << std::endl;
    std::cout << "CLASS: " << param.getTorqueBiasMatrix() << std::endl;*/
    std::cout << "CLASS: " << param.getMassEstimate() << std::endl;

    std::cout << "CLASS: " << param.getCenterMassVector() << std::endl;

    return 0;
}