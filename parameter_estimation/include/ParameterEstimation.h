#include <utility>

//
// Created by morte on 02/10/2024.
//

#ifndef PARAMETERESTIMATION_H
#define PARAMETERESTIMATION_H

class ParameterEstimation {
    public:
    explicit ParameterEstimation(rapidcsv::Document doc) : doc_(std::move(doc)) {
        cols_ = doc_.GetColumnCount();
        rows_ = doc_.GetRowCount();

        mTable_ = Eigen::MatrixXd(rows_, cols_);
        vTable_ = std::vector(cols_, std::vector<float>(rows_));

        for (int i = 0; i < cols_; i++) {
            vTable_[i] = doc_.GetColumn<float>(i);
            for (int j = 0; j < rows_; j++) {
                mTable_(j, i) = vTable_[i][j];
            }
        }
        forceMatrix_ = mTable_(Eigen::all, Eigen::seq(0, 2));
        torqueMatrix_ = mTable_(Eigen::all, Eigen::seq(3, 5));

        forceBiasMatrix_ =  Eigen::MatrixX3d(1,3);
        torqueBiasMatrix_ = Eigen::MatrixX3d(1,3);

        F_ << mTable_.col(0), mTable_.col(1), mTable_.col(2);
        G_ << mTable_.col(9), mTable_.col(10), mTable_.col(11);
        massEstimate_ = G_.transpose().dot(F_) / G_.transpose().dot(G_);

    };

    Eigen::MatrixX3d getForceMatrix() {
        return forceMatrix_;
    }

    Eigen::MatrixX3d getTorqueMatrix() {
        return torqueMatrix_;
    }

    Eigen::MatrixX3d getForceBiasMatrix() {
        for (int i = 0; i < 3; i++) {
            forceBiasMatrix_(0, i) = forceMatrix_.col(i).mean();
        }
        return forceBiasMatrix_;
    }

    Eigen::MatrixX3d getTorqueBiasMatrix() {
        for (int i = 0; i < 3; i++) {
            torqueBiasMatrix_(0, i) = torqueMatrix_.col(i).mean();
        }
        return torqueBiasMatrix_;
    }

    std::vector<Eigen::Matrix3d> GetRotationMatrices() {
        for (int i = 0; i < rows_; i++) {
            Eigen::MatrixXd rotationMatrix = mTable_(i, Eigen::seq(12, 20));
            Eigen::Matrix3d a;
            a.row(0) = rotationMatrix(0, Eigen::seq(0, 2));
            a.row(1) = rotationMatrix(0, Eigen::seq(3, 5));
            a.row(2) = rotationMatrix(0, Eigen::seq(6, 8));
            rotationMatrices_[i] = a;
        }
        return rotationMatrices_;
    }

    Eigen::Vector<double, 72> getForceVector() {
        return F_;
    }

    Eigen::Vector<double, 72> getGravityVector() {
        return G_;
    }

    double getMassEstimate() {
        return massEstimate_;
    }

private:
    rapidcsv::Document doc_;
    Eigen::MatrixXd mTable_;
    std::vector<std::vector<float>> vTable_;

    Eigen::MatrixX3d forceMatrix_;
    Eigen::MatrixX3d torqueMatrix_;
    Eigen::MatrixX3d forceBiasMatrix_;
    Eigen::MatrixX3d torqueBiasMatrix_;
    std::vector<Eigen::Matrix3d> rotationMatrices_;

    Eigen::Vector<double, 72> F_;
    Eigen::Vector<double, 72> G_;
    double massEstimate_;

    int rows_;
    int cols_;
};



#endif //PARAMETERESTIMATION_H
