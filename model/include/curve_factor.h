/**
 * @file   curve_factor.h
 * @brief  This file defines 3d curve residual class based on ceres.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#ifndef CURVE_FACTOR_H
#define CURVE_FACTOR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>

class CurveFactor : public ceres::SizedCostFunction<2, 2, 3>
{
public:
    CurveFactor(const Eigen::Vector3d &input_point, double info = 1.0) : info_(info)
    {
        x_ = input_point.x();
        y_ = input_point.y();
        z_ = input_point.z();
    }

    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

private:
    double x_, y_, z_;
    double info_;
};

#endif  // CURVE_FACTOR_H