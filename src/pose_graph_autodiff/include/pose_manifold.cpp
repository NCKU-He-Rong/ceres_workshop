#include "pose_manifold.hpp"
#include <Eigen/Dense>
#include <rotation.hpp>


// 我們表示SE3是透過位置(3) + 四元數姿態(4), 注意順序先位置再姿態,同時四元數姿態是以qx, qy, qz, qw存放
int PoseManifold::AmbientSize() const
{
    return 7;
}

int PoseManifold::TangentSize() const
{
    return 6;
}


// 這邊要注意本pose_manifold.hpp 跟 pose_manifold.cpp都是針對位置跟姿態分開求導的情況
// 如果針對比叫傳統教科書上面的一起將SE3包在一起的,本實作方法就比較不適用(同時本實作也是基於右擾動)
// 值得說明的是,前者也是目前比較主流的方式了
bool PoseManifold::Plus(const double* x, const double* delta, double* x_plus_delta) const
{
    // 這邊注意Quaterniond的建構順序是w,x,y,z
    // 也就是Eigen::Quaterniond q(qw, qx, qy, qz),但Quaterniond.coeff()的時候卻是顯示x,y,z,w
    // 但注意Eigen::Quaternion內部記憶的的存放則是qx, qy, qz, qw
    // 所以以下的寫法,其實代表的是x[0] = qx, x[1] = qy, x[2] = qz, x[3] = qw
    Eigen::Map<const Eigen::Vector3d> t(x);
    Eigen::Map<const Eigen::Quaterniond>q(x+3);

    Eigen::Map<const Eigen::Vector3d> delta_t(delta);
    Eigen::Quaterniond delta_q = Rotation::rotvec2quaternion(Eigen::Map<const Eigen::Vector3d>(delta+3));

    Eigen::Map<Eigen::Vector3d> t_plus(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q_plus(x_plus_delta+3);

    // 位置更新 
    t_plus = t + t_plus;

    // 姿態右擾動更新
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