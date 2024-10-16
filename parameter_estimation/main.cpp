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




//Noe skjer hær
    {
        rapidcsv::Document doc0("../dataset/0-steady-state_accel.csv");
        rapidcsv::Document doc1("../dataset/0-steady-state_wrench.csv");
        double varFx = variance(doc1, 0);
        double varFy = variance(doc1, 1);
        double varFz = variance(doc1, 2);
        double varTx = variance(doc1, 3);
        double varTy = variance(doc1, 4);
        double varTz = variance(doc1, 5);

        double varAx = variance(doc0, 0);
        double varAy = variance(doc0, 1);
        double varAz = variance(doc0, 2);
        std::cout << "Check" << std::endl;
    }





    // Helper function to read CSV (assumed CSV reader is available or implemented)
    std::vector<Eigen::Vector3d> readAccelData(const std::string& filename);
    std::vector<Eigen::VectorXd> readWrenchData(const std::string& filename);
    std::vector<Eigen::Vector3d> readOrientationData(const std::string& filename);

    rapidcsv::Document doc_accel_data("../dataset/1-baseline_accel.csv");
    rapidcsv::Document doc_wrench_data("../dataset/1-baseline_wrench.csv");
    rapidcsv::Document doc_orientation_data("../dataset/1-baseline_orientation.csv");



    Fusion fusion(param.getMassEstimate());


    // Process the experiment data through the Fusion class
    fusion.insertData(doc_accel_data, doc_wrench_data, doc_orientation_data);




    return 0;
}