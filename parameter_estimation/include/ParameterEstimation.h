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
        vTable_ = std::vector(cols_, std::vector<double>(rows_));

        for (int i = 0; i < cols_; i++) {
            vTable_[i] = doc_.GetColumn<double>(i);
            for (int j = 0; j < rows_; j++) {
                mTable_(j, i) = vTable_[i][j];
            }
        }
        initialize_();
    }

    Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A) {
        return A.completeOrthogonalDecomposition().pseudoInverse();
    }

    Eigen::MatrixX3d getForceMatrix() {
        return forceMatrix_;
    }

    Eigen::MatrixX3d getTorqueMatrix() {
        return torqueMatrix_;
    }

    Eigen::Vector3d getForceBiasMatrix() {
        for (int i = 0; i < 3; i++) {
            forceBiasVector_(i) = forceMatrix_.col(i).mean();
        }
        return forceBiasVector_;
    }

    Eigen::Vector3d getTorqueBiasMatrix() {
        for (int i = 0; i < 3; i++) {
            torqueBiasVector_(i) = torqueMatrix_.col(i).mean();
        }
        return torqueBiasVector_;
    }

    std::vector<Eigen::Matrix3d> getRotationMatrices() {
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

    Eigen::MatrixXd getLargeR() {
        int num_matrices = rotationMatrices_.size();

        Eigen::MatrixXd r(num_matrices * 3, 3);

        for (int i = 0; i < num_matrices; ++i) {
            r.block<3, 3>(i * 3, 0) = rotationMatrices_[i];
        }
        return r;
    }

    Eigen::Vector<double, 72> getForceVector() {
        return F_;
    }

    Eigen::Vector<double, 72> getGravityVector() {
        return G_;
    }

    Eigen::Vector<double, 72> getTorqeVector() {
        return T_;
    }

    double getMassEstimate() {
        return massEstimate_;
    }

    Eigen::Vector3d getCenterMassVector() {
        Eigen::Vector<double, 72> T;
        for (int i = 0; i < 24; i++) {
            T.segment<3>(i * 3) << mTable_(i,3), mTable_(i,4), mTable_(i,5);
        }
        Eigen::MatrixXd A(72,3);
        for (int i = 0; i < 24; i++) {
            Eigen::MatrixXd A_i{
                {0 ,mTable_(i,11), -mTable_(i,10)},
                {-mTable_(i,11), 0,mTable_(i,9)},
                {mTable_(i,10),-mTable_(i,9),0}
            };
            A.block<3,3>(i*3, 0) = A_i;
        }
        auto ATA_inv = pseudo_inverse(A.transpose()*A);
        auto l = A.transpose()*T;
        auto mat = ATA_inv*l;
        auto r = 1.0 / massEstimate_*mat;
        return r;
    }

    /*Eigen::Vector3d getEstimationMass() {
        double = massEstimate_*


        return 0;
    }*/

private:
    void initialize_() {
        forceMatrix_ = mTable_(Eigen::all, Eigen::seq(0, 2));
        torqueMatrix_ = mTable_(Eigen::all, Eigen::seq(3, 5));

        forceBiasVector_ =  Eigen::Vector3d(0,0,0);
        torqueBiasVector_ = Eigen::Vector3d(0,0,0);

        F_ << mTable_.col(0), mTable_.col(1), mTable_.col(2);
        G_ << mTable_.col(9), mTable_.col(10), mTable_.col(11);
        T_ << mTable_.col(3), mTable_.col(4), mTable_.col(5);
        A_ = Eigen::MatrixX3d(rows_,3);
        massEstimate_ = G_.transpose().dot(F_) / G_.transpose().dot(G_);
    }

    rapidcsv::Document doc_;
    Eigen::MatrixXd mTable_;
    std::vector<std::vector<double>> vTable_;

    Eigen::MatrixX3d forceMatrix_;
    Eigen::MatrixX3d torqueMatrix_;
    Eigen::Vector3d forceBiasVector_;
    Eigen::Vector3d torqueBiasVector_;
    Eigen::MatrixX3d A_;
    std::vector<Eigen::Matrix3d> rotationMatrices_;
    Eigen::MatrixXd largeR;

    Eigen::Vector<double, 72> F_;
    Eigen::Vector<double, 72> G_;
    Eigen::Vector<double, 72> T_;
    double massEstimate_;

    int rows_;
    int cols_;
};



#endif //PARAMETERESTIMATION_H
