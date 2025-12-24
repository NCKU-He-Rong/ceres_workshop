#ifndef POSE_MANIFOLD_HPP_
#define POSE_MANIFOLD_HPP_

#include "ceres/ceres.h"
#include <Eigen/Dense>



class PoseManifold: public ceres::Manifold
{
public:
    int AmbientSize() const override;
    int TangentSize() const override;

    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override;
    bool PlusJacobian(const double* x, double* jacobian) const override ; 
    bool Minus(const double* y, const double*x, double* y_minus_x) const override;
    bool MinusJacobian(const double*x, double* jacobian) const override;
};
#endif