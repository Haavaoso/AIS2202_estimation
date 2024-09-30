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
        rapidcsv::Document doc("C:/libs/dataset/0-calibration_fts-accel.csv");



        try {
            std::vector<float> fx = doc.GetColumn<float>("fx");
            std::cout << "JIPPU:: " << fx.size() << std::endl;
            float buffer = 0;
            for (int i = 0; i < fx.size(); i++) {
                buffer += fx[i];
            }
            fx_bias = buffer/fx.size();

        } catch (const std::invalid_argument& e) {
            std::cerr << "Error: Fuck dette" << e.what() << std::endl;
        }


        std::cout << "Doc loaded: ";



    } catch (...) {
        std::cerr << "Error: Did not load doc" << std::endl;
    }




    std::cout << fx_bias << std::endl;
    return 0;
}
