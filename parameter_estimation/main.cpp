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



        //Create rotation matrix of the FTS
        std::vector<std::vector<float>> r1x(3);
        std::vector<std::vector<float>> r2x(3);
        std::vector<std::vector<float>> r3x(3);

        std::vector<Eigen::Matrix3f> rotationMatrices(24);

        for (int i = 0; i < 3; i++) {
            r1x[i] = doc.GetColumn<float>(i+12);
            r2x[i] = doc.GetColumn<float>(i+15);
            r3x[i] = doc.GetColumn<float>(i+18);
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
        std::vector<Eigen::Vector3f> gravity_XYZ(24);
        std::vector<std::vector<float>> gravityBuffer(3, std::vector<float>(24)); // 3 vectors each of size 24
        std::vector<Eigen::VectorXf> G_I(24);  // Use RowVector3f to store transposed vector

        for (int i = 0; i < gravityBuffer.size(); i++) {
            gravityBuffer[i] = doc.GetColumn<float>(i+9);
        }


         for (int i = 0; i < 24; i++) {
             for (int j = 0; j < 3; j ++){
                 gravity_XYZ[i][j] = bufferVector[j][i];
             }
             G_I[i] = gravity_XYZ[i].transpose();  // TRANSPOSED IN ROW VECTOR
         }

        std::vector<Eigen::Vector3f> s_G_i(24);  // Change from Matrix3f to Vector3f

        for (int i = 0; i < rotationMatrices.size(); i++) {
            s_G_i[i] = rotationMatrices[i] * G_I[i];
        }

        Eigen::VectorXf largeF(72,1);
        Eigen::VectorXf largeG(72,1);


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
         }




        auto coolMass = (largeF.transpose() * largeG) / (largeG.transpose() * largeG);

         std::cout << coolMass << std::endl;



    } catch (...) {
        std::cerr << "Error: Did not load doc" << std::endl;
    }


    return 0;
}
