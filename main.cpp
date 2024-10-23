#include "parameter_estimation/include/ParameterEstimation.h"
#include "parameter_estimation/include/fusion.hpp"
#include "parameter_estimation/include/variance.h"
#include "state_estimation/include/kalman_filter.h"
#include <iostream>
#include <vector>
#include "rapidcsv.h"
#include <Eigen/Dense>

int main() {
    rapidcsv::Document biasDoc("../dataset/0-calibration_fts-accel.csv");

    rapidcsv::Document documentAccel("../dataset/1-baseline_accel.csv");
    rapidcsv::Document documentOrientations("../dataset/1-baseline_accel.csv");
    rapidcsv::Document documentWrench("../dataset/1-baseline_accel.csv");

    ParameterEstimation parameter_estimation(biasDoc);

    auto forceBias = parameter_estimation.getForceBiasVector();
    auto torqueBias = parameter_estimation.getTorqueVector();
    auto massEstimate = parameter_estimation.getMassEstimate();
    auto centerMassEstimate = parameter_estimation.getCenterMassVector();

    std::vector<double> varForce(3);
    std::vector<double> varTorque(3);
    std::vector<double> varAccel(3);

    //NOE SKJER HÆR
    {
        varForce[0] = variance(documentWrench, 0);
        varForce[1] = variance(documentWrench, 1);
        varForce[2] = variance(documentWrench, 2);
        varTorque[0] = variance(documentWrench, 3);
        varTorque[1] = variance(documentWrench, 4);
        varTorque[2] = variance(documentWrench, 5);
        varAccel[0] = variance(documentAccel, 0);
        varAccel[1] = variance(documentAccel, 1);
        varAccel[2] = variance(documentAccel, 2);
        std::cout << "Check" << std::endl;
    }

    Fusion fusion(massEstimate,centerMassEstimate,varForce,varTorque,varAccel);

    /*kalman_filter kalman_filter();

    for (int i = 0; i < documentAccel.GetRowCount(); i++) {
        kalman_filter().predict();
        kalman_filter().update(z);
    }
    */
}

