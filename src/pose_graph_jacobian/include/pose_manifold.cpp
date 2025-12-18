#include "pose_manifold.hpp"
#include <Eigen/Dense>
#include <rotation.hpp>

/*
    侷限性: 右擾動、位置再姿態、w, qx, qy, qz存放四元數
*/

// 我們表示SE3是透過位置(3) + 四元數姿態(4), 注意順序先位置再姿態,同時四元數姿態是以qw, qx, qy, qz存放(我自己習慣的)
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
    // 所以以下的寫法,其實代表的是x[3] = qx, x[4] = qy, x[5] = qz, x[6] = qw
    // Eigen::Map<const Eigen::Quaterniond>q(x+3);
    // 這邊以下雖然也可以寫成Eigen::Map<const Eigen::Vector3d> t(x);
    // 但是其實意義不大(可能會快一點,因為不用再重新宣告記憶體),但這邊為了追求統一(跟Eigen::Quaterniond q一致)
    // 所以改成這個以下這個
    const Eigen::Vector3d t(x[0], x[1], x[2]);
    const Eigen::Quaterniond q(x[3], x[4], x[5], x[6]);
    
    const Eigen::Vector3d delta_t(delta[0], delta[1], delta[2]);
    const Eigen::Quaterniond delta_q = Rotation::rotvec2quaternion(Eigen::Vector3d(delta[3], delta[4], delta[5]));

    // 位置更新 
    Eigen::Map<Eigen::Vector3d> t_plus(x_plus_delta);
    t_plus = t + delta_t;

    // 姿態右擾動更新
    // 注意這邊不要用Eigen::Map<Eigen::Quaterniond> q_plus(x_plus_delta+3)來更新
    // 因為一樣卡到Eigen的記憶體存放順序
    Eigen::Quaterniond q_plus = (q * delta_q).normalized();
    if (q_plus.w() < 0) {q_plus.coeffs() *= -1;}
    
    x_plus_delta[3] = q_plus.w();
    x_plus_delta[4] = q_plus.x();
    x_plus_delta[5] = q_plus.y();
    x_plus_delta[6] = q_plus.z();

    return true;

}

bool PoseManifold::PlusJacobian(const double* x, double* jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j_plus(jacobian);
    j_plus.setZero();
    j_plus.topRows<6>().setIdentity();
    return true;

}

bool PoseManifold::Minus(const double* y, const double* x, double* y_minus_x) const
{
    // 這邊以下雖然也可以寫成Eigen::Map<const Eigen::Vector3d> t_y(y);
    // 但是其實意義不大(可能會快一點,因為不用再重新宣告記憶體),但這邊為了追求統一(跟Eigen::Quaterniond q一致)
    // 所以改成這個以下這個
    const Eigen::Vector3d t_y(y[0], y[1], y[2]);
    const Eigen::Quaterniond q_y(y[3], y[4], y[5], y[6]);

    const Eigen::Vector3d t_x(x[0], x[1], x[2]);
    const Eigen::Quaterniond q_x(x[3], x[4], x[5], x[6]);

    Eigen::Map<Eigen::Vector3d> t_y_minus_x(y_minus_x);
    Eigen::Map<Eigen::Vector3d> q_y_minus_x(y_minus_x+3);

    t_y_minus_x = t_y - t_x;
    q_y_minus_x = Rotation::quaternion2rotvec((q_x.inverse() * q_y).normalized());

    return true;
}

bool PoseManifold::MinusJacobian(const double* x, double* jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j_minus(jacobian);
    j_minus.setZero();
    j_minus.topRows<6>().setIdentity();
    return true;
}