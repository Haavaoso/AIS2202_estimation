#include <iostream>
#include <vector>
#include "rapidcsv.h"
#include <Eigen/Dense>
#include "include/ParameterEstimation.h"
#include "include/variance.h"
#include "include/fusion.hpp"
#include <iostream>
#include <Eigen/Dense>


int main() {
    rapidcsv::Document doc("../dataset/0-calibration_fts-accel.csv");

    ParameterEstimation param(doc);

    std::cout << "force: " << "\n" << param.getForceBiasMatrix() << std::endl;
    std::cout << "torqe: " << "\n" <<param.getTorqueBiasMatrix() << std::endl;
    std::cout << "IMU" << "\n" <<param.getImuBiasMatrix() << std::endl;
    std::cout << "massEstimate: " <<"\n" << param.getMassEstimate() << std::endl;
    std::cout << "centerMassEstimate: " <<"\n" << param.getCenterMassVector() << std::endl;




    rapidcsv::Document doc0("../dataset/0-steady-state_accel.csv");
    rapidcsv::Document doc1("../dataset/0-steady-state_wrench.csv");
    std::vector<double> varF(3);
    std::vector<double> varT(3);
    std::vector<double> varA(3);

//NOE SKJER HÆR
    {
        varF[0] = variance(doc1, 0);
        varF[1] = variance(doc1, 1);
        varF[2] = variance(doc1, 2);
        varT[0] = variance(doc1, 3);
        varT[1] = variance(doc1, 4);
        varT[2] = variance(doc1, 5);
        varA[0] = variance(doc0, 0);
        varA[1] = variance(doc0, 1);
        varA[2] = variance(doc0, 2);
        std::cout << "Check" << std::endl;
    }





    rapidcsv::Document doc_accel_data("../dataset/1-baseline_accel.csv");
    // std::cout << doc_accel_data.GetColumn<double>(0)[0] << std::endl;
    rapidcsv::Document doc_wrench_data("../dataset/1-baseline_wrench.csv");
    rapidcsv::Document doc_orientation_data("../dataset/1-baseline_orientations.csv");



    Fusion fusion(param.getMassEstimate(), param.getCenterMassVector(), varF, varT, varA);

    // Process the experiment data through the Fusion class
    fusion.insertData(doc_accel_data, doc_wrench_data, doc_orientation_data);

    std::cout << "snack" << std::endl;

    fusion.updateMatrix(1);


    return 0;
}