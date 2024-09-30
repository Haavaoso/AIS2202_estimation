#include <iostream>
#include <vector>
#include "rapidcsv.h"
#include <Eigen/Dense>

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}


bool file_exists(const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}
int main()
{
    std::string filePath = "../dataset/0-calibration_fts-accel.csv";
    if (!file_exists(filePath)) {
        std::cerr << "Error: File does not exist at " << filePath << std::endl;
        return -1;
    }
    // Use least squares to estimate the (constant) mass and mass center of an unknown robot end effector tool.

    try {
        rapidcsv::Document doc(filePath);



        std::vector<std::vector<float>> bufferVector(6);
        std::vector<float> biasVector(6);

        for (int i = 0; i < biasVector.size(); i++) {
            bufferVector[i] = doc.GetColumn<float>(i);
        }

        for (int i = 0; i < bufferVector.size(); i++) {
            float buffer = 0;
            for (int j = 0; j < bufferVector[i].size(); j++) {
                //std::cout << bufferVector[i][j] << std::endl;
                buffer += bufferVector[i][j];
            }
            biasVector[i] = buffer/bufferVector[i].size();
        }


        for (int i = 0; i < biasVector.size(); i++) {
            std::cout << i << ": " << biasVector[i] << std::endl;
        }


    } catch (...) {
        std::cerr << "Error: Did not load doc" << std::endl;
    }


    return 0;
}
