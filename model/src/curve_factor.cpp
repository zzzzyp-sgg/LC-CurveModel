#include "curve_factor.h"
#include <Eigen/Dense>

bool CurveFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    // plane: y = kx + h
    // mesh:  z = a * (x - b)^2 + c
    const double *plane = parameters[0];
    const double *mesh = parameters[1];

    residuals[0] = info_ * (plane[0] * x_ + plane[1] - y_);
    residuals[1] = info_ * (mesh[0] * pow(x_ - mesh[1], 2) + mesh[2] - z_);

    if (jacobians)
    {
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> jacobian(jacobians[0]);
            jacobian.setZero();
            jacobian(0, 0) = x_;
            jacobian(0, 1) = 1;
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian(jacobians[1]);
            jacobian.setZero();
            jacobian(1, 0) = pow(x_ - mesh[1], 2);
            jacobian(1, 1) = -2 * mesh[0] * (x_ - mesh[1]);
            jacobian(1, 2) = 1;
        }
    }

    return true;
}