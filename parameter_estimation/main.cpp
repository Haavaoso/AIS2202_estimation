#include <iostream>
#include <vector>
#include "rapidcsv.h"
#include <Eigen/Dense>

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}
float fx_bias;
int main()
{
    try {
        rapidcsv::Document doc("../dataset/0-calibration_fts-accel.csv");

        std::vector<std::vector<float>> bufferVector(6);
        std::vector<float> biasVector(6);

        for (int i = 0; i < bufferVector.size(); i++) {
            bufferVector[i] = doc.GetColumn<float>(i);


        }

        for (int i = 0; i < bufferVector.size(); i++) {
            float buffer = 0;
            for (int j = 0; j < bufferVector[i].size(); j++) {
                buffer += bufferVector[i][j];
            }
            biasVector[i] = buffer/bufferVector[i].size();
        }


        for (int i = 0; i < biasVector.size(); i++) {
            std::cout << i << ": " << biasVector[i] << std::endl;
        }

        std::cout << "Doc loaded: ";



    } catch (...) {
        std::cerr << "Error: Did not load doc" << std::endl;
    }


    return 0;
}
