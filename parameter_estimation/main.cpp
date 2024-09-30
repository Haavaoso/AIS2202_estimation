#include <iostream>
#include <vector>
#include "rapidcsv.h"
#include <Eigen/Dense>

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A) {
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

float fx_bias;

int main() {
    rapidcsv::Document doc("../dataset/0-calibration_fts-accel.csv");
    int rows = doc.GetRowCount();
    int cols = doc.GetColumnCount();

    Eigen::MatrixXd mTable(rows,cols);
    std::vector vTable(cols, std::vector<float>(rows));

    for (int i = 0; i < cols;i++) {
        vTable[i] = doc.GetColumn<float>(i);

        for (int j = 0; j < rows; j++) {
            mTable(j,i) = vTable[i][j];
        }
    }

    Eigen::MatrixXd forceMatrix(rows,3);
    for (int i = 0; i < 3; i++) {
        forceMatrix.col(i) = mTable.col(i);
    }
    Eigen::MatrixXd torqueMatrix(rows,3);
    for (int i = 0; i < 3; i++) {
        torqueMatrix.col(i) = mTable.col(i+3);
    }
    Eigen::MatrixXd forceBiasMatrix(1,3);
    for (int i = 0; i < 3; i++) {
        forceBiasMatrix(0,i) = forceMatrix.col(i).mean();
    }
    std::cout << forceBiasMatrix<< std::endl;


/*
    std::vector<std::vector<float> > bufferVector(6);
    std::vector<float> biasVector(6);

    for (int i = 0; i < bufferVector.size(); i++) {
        bufferVector[i] = doc.GetColumn<float>(i);
    }

    for (int i = 0; i < bufferVector.size(); i++) {
        float buffer = 0;
        for (int j = 0; j < bufferVector[i].size(); j++) {
            buffer += bufferVector[i][j];
        }
        biasVector[i] = buffer / bufferVector[i].size();
    }


    for (int i = 0; i < biasVector.size(); i++) {
        std::cout << i << ": " << biasVector[i] << std::endl;
    }
    */
    return 0;
}
