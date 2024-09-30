#include <iostream>
#include <vector>
#include "rapidcsv.h"
#include <Eigen/Dense>

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

int main()
{

    rapidcsv::Document doc("0-calibration_fts-accel.csv");

    Eigen::VectorXd G(72);
    Eigen::VectorXd F(72);



    /*
    //the pseudo_inverse function is applied to the term G.transpose() * G because you cannot
    //directly divide matrices. The pseudo-inverse takes the place of division.
    Eigen::Vector3d r = pseudo_inverse(G.transpose() * G) * (G.transpose() * F);
    */

    std::cout << "r" << std::endl;
    return 0;
}
