/**
 * @file curve_factor_auto.h
 * @brief Optimization for curve fitting, auto-diff.
 * @author Yipeng Zhao
 * @date 2024-07
*/

#ifndef CURVE_FACTOR_AUTO_H
#define CURVE_FACTOR_AUTO_H

#include <ceres/ceres.h>

class CurveFactorAuto
{
public:
    CurveFactorAuto(const Eigen::Vector3d &input_point, double info = 1.0) : info_(info)
    {
        x_ = input_point.x();
        y_ = input_point.y();
        z_ = input_point.z();
    }

    template <typename T>
    bool operator()(const T* plane, const T* mesh, T* residual) const {
        
        residual[0] = info_ * (T(y_) - plane[0] * T(x_) - plane[1]);
        residual[1] = info_ * (T(z_) - mesh[0] * ceres::pow(T(x_) - mesh[1], 2) - mesh[2]);

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d &input_point, double info = 1) {
        return new ceres::AutoDiffCostFunction<CurveFactorAuto, 2, 2, 3>
            (new CurveFactorAuto(input_point, info));
    }
private:
    double x_, y_, z_;
    double info_;
};

#endif