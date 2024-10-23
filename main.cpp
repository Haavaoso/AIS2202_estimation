#include "parameter_estimation/include/ParameterEstimation.h"
#include "parameter_estimation/include/fusion.hpp"
#include "parameter_estimation/include/variance.h"
#include "state_estimation/include/kalman_filter.h"
#include <iostream>
#include <vector>

int main() {
    rapidcsv::Document biasDoc("../dataset/0-calibration_fts-accel.csv");
    rapidcsv::Document accelVarianceDoc("../dataset/0-steady-state_accel.csv");
    rapidcsv::Document wrenchVarianceDoc("../dataset/0-steady-state_wrench.csv");

    rapidcsv::Document documentAccel("../dataset/1-baseline_accel.csv");
    rapidcsv::Document documentOrientations("../dataset/1-baseline_orientations.csv");
    rapidcsv::Document documentWrench("../dataset/1-baseline_wrench.csv");

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
        varForce[0] = variance(wrenchVarianceDoc, 0);
        varForce[1] = variance(wrenchVarianceDoc, 1);
        varForce[2] = variance(wrenchVarianceDoc, 2);
        varTorque[0] = variance(wrenchVarianceDoc, 3);
        varTorque[1] = variance(wrenchVarianceDoc, 4);
        varTorque[2] = variance(wrenchVarianceDoc, 5);
        varAccel[0] = variance(accelVarianceDoc, 0);
        varAccel[1] = variance(accelVarianceDoc, 1);
        varAccel[2] = variance(accelVarianceDoc, 2);
    }

    Fusion fusion(massEstimate,centerMassEstimate,varForce,varTorque,varAccel);
    fusion.insertData(documentAccel,documentWrench,documentOrientations);
    fusion.updateMatrix(0);

    /*kalman_filter kalman_filter();

    for (int i = 0; i < documentAccel.GetRowCount(); i++) {
        kalman_filter().predict();
        kalman_filter().update(z);
    }
    */
}

