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
    /*
    // Check if the file exists by opening it
    std::ifstream file();
    if (!file)
    {
        std::cerr << "Error: OOOF!" << std::endl;
        return -1;
    }
    */

    // Use least squares to estimate the (constant) mass and mass center of an unknown robot end effector tool.

    try {
        rapidcsv::Document doc("C:/libs/dataset/0-calibration_fts-accel.csv", rapidcsv::LabelParams(-1, -1));



        std::vector<std::vector<float>> bufferVector(6);
        std::vector<float> biasVector(6);


        for (int i = 0; i < 5; i++) {
            bufferVector[i] = doc.GetColumn<float>(i);
        }

        /*std::cout << "loaded colums" << std::endl;
        std::cout << bufferVector[0].size() << std::endl;
        for (int i = 0; i < bufferVector[0].size(); i++) {
            std::cout << i << ": " << bufferVector[0][i] << std::endl;
        }*/


        for (int i = 0; i < bufferVector.size(); i++) {
            float buffer = 0;
            for (int j = 0; j < bufferVector[i].size(); j++) {
                //std::cout << bufferVector[i][j] << std::endl;
                buffer += bufferVector[i][j];
            }

            //std::cout << "NEW" << std::endl;
            biasVector[i] = buffer/bufferVector[i].size();
            //std::cout << biasVector[i] << std::endl;
        }


        for (int i = 0; i < biasVector.size(); i++) {
            std::cout << i << ": " << biasVector[i] << std::endl;
            //std::cout << bufferVector[i].size() << std::endl;
        }

        std::cout << "Doc loaded: ";



    } catch (...) {
        std::cerr << "Error: Did not load doc" << std::endl;
    }


    return 0;
}
