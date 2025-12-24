#include "pose_manifold.hpp"
#include "rotation.hpp"



int PoseManifold::AmbientSize() const 
{
    return 6;
}

int PoseManifold::TangentSize() const
{
    return 6;
}

bool PoseManifold::Plus(const double* x, const double* delta, double* x_plus_delta) const
{
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> pose_x(x);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> pose_delta(delta);

    Rotation::se3_exp(pose_x) * Rotation::se3_exp(pose_delta);
    

    

}

bool PoseManifold::PlusJacobian(const double* x, double* jacobian) const
{


}

bool PoseManifold::Minus(const double* y, const double* x, double* y_minus_x) const
{
    
}

bool PoseManifold::MinusJacobian(const double* x, double* jacobian) const
{

}