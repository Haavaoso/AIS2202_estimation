#include <iostream>
#include <vector>
#include "rapidcsv.h"
#include <Eigen/Dense>

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A) {
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

int main() {
    rapidcsv::Document doc("../dataset/0-calibration_fts-accel.csv");
    int rows = doc.GetRowCount();
    int cols = doc.GetColumnCount();

    Eigen::MatrixXd mTable(rows, cols);
    std::vector vTable(cols, std::vector<float>(rows));

    for (int i = 0; i < cols; i++) {
        vTable[i] = doc.GetColumn<float>(i);

        for (int j = 0; j < rows; j++) {
            mTable(j, i) = vTable[i][j];
        }
    }
    Eigen::MatrixX3d forceMatrix = mTable(Eigen::all, Eigen::seq(0, 2));
    Eigen::MatrixX3d torqueMatrix = mTable(Eigen::all, Eigen::seq(3, 5));

    std::vector<Eigen::Matrix3d> rotationMatrices(rows);
    for (int i = 0; i < rows; i++) {
        Eigen::MatrixXd rotationMatrix = mTable(i, Eigen::seq(12, 20));
        Eigen::Matrix3d a;
        a.row(0) = rotationMatrix(0, Eigen::seq(0, 2));
        a.row(1) = rotationMatrix(0, Eigen::seq(3, 5));
        a.row(2) = rotationMatrix(0, Eigen::seq(6, 8));
        rotationMatrices[i] = a;
    }

    Eigen::MatrixX3d forceBiasMatrix(1, 3);
    for (int i = 0; i < 3; i++) {
        forceBiasMatrix(0, i) = forceMatrix.col(i).mean();
    }
    Eigen::MatrixX3d torqueBiasMatrix(1, 3);
    for (int i = 0; i < 3; i++) {
        torqueBiasMatrix(0, i) = torqueMatrix.col(i).mean();
    }

    std::vector<double> a(72);

    Eigen::Vector<double, 72> F;
    Eigen::Vector<double, 72> G;
    F << mTable.col(0), mTable.col(1), mTable.col(2);
    G << mTable.col(9), mTable.col(10), mTable.col(11);

    auto massEstimate = G.transpose().dot(F) / G.transpose().dot(G);
    std::cout << forceBiasMatrix << std::endl;
    std::cout << torqueBiasMatrix << std::endl;
    std::cout << massEstimate << std::endl;





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
