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

    rapidcsv::Document accel("../dataset/1-baseline_accel.csv");
    rapidcsv::Document orientations("../dataset/1-baseline_accel.csv");
    rapidcsv::Document wrench("../dataset/1-baseline_accel.csv");

    ParameterEstimation parameter_estimation(biasDoc);

    kalman_filter kalman_filter();

    auto forceBias = parameter_estimation.getForceBiasVector();
    auto torqueBias = parameter_estimation.getTorqueVector();
    auto massEstimate = parameter_estimation.getMassEstimate();
    auto centerMassEstimate = parameter_estimation.getCenterMassVector();

    for (int i = 0; i < accel.GetRowCount(); i++) {
        kalman_filter().predict();
        kalman_filter().update(z);
    }
}

