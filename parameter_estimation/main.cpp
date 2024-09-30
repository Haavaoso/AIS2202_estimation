#include <rapidcsv.h>
#include <Eigen/Dense>

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

int main()
{
    Eigen::VectorXd G(72);
    Eigen::VectorXd F(72);

    Eigen::Vector3d r = (G.transpose()*G)

    return 0;
}
