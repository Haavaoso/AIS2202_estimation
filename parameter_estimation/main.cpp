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

        std::vector<std::vector<double>> bufferVector(6);
        std::vector<double> biasVector(6);

        for (int i = 0; i < biasVector.size(); i++) {
            bufferVector[i] = doc.GetColumn<double>(i);
        }

        for (int i = 0; i < bufferVector.size(); i++) {
            double buffer = 0;
            for (int j = 0; j < bufferVector[i].size(); j++) {
                //std::cout << bufferVector[i][j] << std::endl;
                buffer += bufferVector[i][j];
            }
            biasVector[i] = buffer/bufferVector[i].size();
        }


        for (int i = 0; i < biasVector.size(); i++) {
            std::cout << i << ": " << biasVector[i] << std::endl;
        }

        //Create rotation matrix of the FTS
        std::vector<std::vector<double>> r1x(3);
        std::vector<std::vector<double>> r2x(3);
        std::vector<std::vector<double>> r3x(3);

        std::vector<Eigen::Matrix3d> rotationMatrices(24);

        for (int i = 0; i < 3; i++) {
            r1x[i] = doc.GetColumn<double>(i+12);
            r2x[i] = doc.GetColumn<double>(i+15);
            r3x[i] = doc.GetColumn<double>(i+18);
        }


        for (int i = 0; i < rotationMatrices.size(); i++) {
            for (int j = 0; j < 3; j++) {
                rotationMatrices[i](0, j) = r1x[j][i];
                rotationMatrices[i](1, j) = r2x[j][i];
                rotationMatrices[i](2, j) = r3x[j][i];
            }
        }

        //Test Matrix
        /*
        Eigen::MatrixXd ftsMat(3, 3);  // Creates a 3x3 matrix of doubles
        ftsMat(0, 0) = -1;
        ftsMat(1, 0) = 5.51269E-06;
        ftsMat(2, 0) = 0.000170476;
        ftsMat(0, 1) = 0.000170476;
        ftsMat(1, 1) = 1.20441E-05;
        ftsMat(2, 1) = 1;
        ftsMat(0, 2) = 5.51063E-06;
        ftsMat(1, 2) = 1;
        ftsMat(2, 2) = -1.20451E-05;
        std::cout << ftsMat << std::endl;
        */


        //GRAVITY VECTOR [X, Y, Z]
        ;  // This will store the transposed matrix
        std::vector<Eigen::Vector3d> gravity_XYZ(24);
        std::vector<std::vector<double>> gravityBuffer(3, std::vector<double>(24)); // 3 vectors each of size 24
        std::vector<Eigen::VectorXd> G_I(24);  // Use RowVector3f to store transposed vector

        for (int i = 0; i < gravityBuffer.size(); i++) {
            gravityBuffer[i] = doc.GetColumn<double>(i+9);
        }


         for (int i = 0; i < 24; i++) {
             for (int j = 0; j < 3; j ++){
                 gravity_XYZ[i][j] = bufferVector[j][i];
             }
             G_I[i] = gravity_XYZ[i].transpose();  // TRANSPOSED IN ROW VECTOR
         }

        std::vector<Eigen::Vector3d> s_G_i(24);  // Change from Matrix3f to Vector3f

        for (int i = 0; i < rotationMatrices.size(); i++) {
            s_G_i[i] = rotationMatrices[i] * G_I[i];
        }

        Eigen::VectorXd largeF(72);
        Eigen::VectorXd largeG(72);
        Eigen::VectorXd largeT(72);


         for (int holyShit = 0; holyShit < 72; holyShit++) {
             int ogga = 0;
             int hehe;
             if (holyShit < 24) {
                 ogga = 0;
                 hehe = 0;
             }
             else if ( holyShit < 48) {
                 ogga = 1;
                 hehe = 24;
             }
             else {
                 ogga = 2;
                 hehe = 48;
             }
             largeF[holyShit] = bufferVector[ogga][holyShit - hehe];
             largeG[holyShit] = gravityBuffer[ogga][holyShit - hehe];
             largeT[holyShit] = bufferVector[3+ogga][holyShit - hehe];
         }




        double mass = (largeF.transpose().dot(largeG)) / (largeG.transpose().dot(largeG));
        std::cout << "mass: " << mass  << std::endl;

        Eigen::MatrixXd largeA(3 * gravity_XYZ.size(), 3);
        for (int i = 0; i < gravity_XYZ.size(); i++) {
            Eigen::MatrixXd A_x(3, 3);
            A_x(0, 0) = 0;
            A_x(0, 1) = gravity_XYZ[i][2];
            A_x(0, 2) = -gravity_XYZ[i][1];

            A_x(1, 0) = -gravity_XYZ[i][2];
            A_x(1, 1) = gravity_XYZ[i][0];
            A_x(1, 2) = gravity_XYZ[i][2];

            A_x(2, 0) = gravity_XYZ[i][1];
            A_x(2, 1) = -gravity_XYZ[i][0];
            A_x(2, 2) = 0;
            largeA.block<3,3>(i*3, 0) = A_x;

        }

        // Calculate the product A^T * A
        auto ATA = largeA.transpose() * largeA;

        // Compute the pseudo-inverse of ATA
        auto ATA_pseudo_inv = pseudo_inverse(ATA);

        // Multiply by A^T and T, and scale by 1/mass
        auto r = (1.0 / mass) * (ATA_pseudo_inv *(largeA.transpose()) * (largeT));

        std::cout << "Center of Mass: " << r << std::endl;



        // std::cout << "not mass: " << r  << std::endl;


    } catch (...) {
        std::cerr << "Error: Did not load doc" << std::endl;
    }


    return 0;
}
