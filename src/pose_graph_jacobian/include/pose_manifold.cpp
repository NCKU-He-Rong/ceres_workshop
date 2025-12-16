#include <pose_manifold.hpp>
#include <Eigen/Dense>
#include <rotation.hpp>


int PoseManifold::AmbientSize() const
{
    return 7;
}

int PoseManifold::TangentSize() const
{
    return 6;
}

bool PoseManifold::Plus(const double* x, const double* delta, double* x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> t(x);
    Eigen::Map<const Eigen::Quaterniond>q(x+3);

    Eigen::Map<const Eigen::Vector3d> delta_t(delta);
    Eigen::Quaterniond delta_q = Rotation::rotvec2quaternion(Eigen::Map<const Eigen::Vector3d>(delta+3));

    Eigen::Map<Eigen::Vector3d> t_plus(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q_plus(x_plus_delta+3);

    // 
    t_plus = t + t_plus;
    q_plus = (q * delta_q).normalized();

    return true;

}

bool PoseManifold::PlusJacobian(const double* x, double* jacobian) const
{
    
}

bool PoseManifold::Minus(const double* y, const double* x, double* y_minus_x) const
{
    Eigen::Map<const Eigen::Vector3d> t_y(y);
    Eigen::Map<const Eigen::Quaterniond> q_y(y+3);

    Eigen::Map<const Eigen::Vector3d> t_x(x);
    Eigen::Map<const Eigen::Quaterniond> q_x(x+3);

    Eigen::Map<const Eigen::Vector3d> t_y_minus_x(y_minus_x);
    Eigen::Map<const Eigen::Vector3d> q_y_minus_x(y_minus_x+3);

    t_y_minus_x = t_y - t_x;
    q_y_minus_x = Rotation::quaternion2rotvec((q_x.inverse() * q_y).normalized());

}

bool PoseManifold::MinusJacobian(const double* x, double* jacobian) const
{



}