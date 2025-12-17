#ifndef COST_FUNCTOR_HPP_
#define COST_FUNCTOR_HPP_

#include "ceres/ceres.h"
#include <Eigen/Dense>
#include "rotation.hpp"


struct costfunctor
{
    costfunctor(const Eigen::Vector3d &t, 
                const Eigen::Quaterniond &q,
                const Eigen::Matrix<double, 21, 1> &info_vec)
                : t_mes(t), q_mes(q)
    {
        int k = 0;
        for(int i=0;i<6;i++)
        {
            for(int j=i;j<6;j++)
            {
                info_mat(i, j) = info_vec(k++);
            } 
        }
        info_mat = info_mat.selfadjointView<Eigen::Upper>();
        sqrt_mat = info_mat.cwiseAbs().cwiseSqrt();
        
        q_mes.normalize();
    }

    // 注意這邊跟我們傳統的residual定義不一樣
    // 同時一定不能定義q* * q當作residual(雖然他們是一維),因為residual最小不是(0, 0, 0, 0);
    // 因為後我們將用NumericDiffCostFunction,所以這邊我們不用template <typename T>
    bool operator()(const double* const xi, const double* const xj, double* residual) const
    {
        Eigen::Vector3d t_xi(xi[0], xi[1], xi[2]);
        Eigen::Quaterniond q_xi(xi[3], xi[4], xi[5], xi[6]); 
        q_xi.normalize();   // 這邊針對NumericDiffCostFunction很關鍵

        Eigen::Vector3d t_xj(xj[0], xj[1], xj[2]);
        Eigen::Quaterniond q_xj(xj[3], xj[4], xj[5], xj[6]);
        q_xj.normalize();   // 這邊針對NumericDiffCostFunction很關鍵

        Eigen::Map<Eigen::Vector3d> r_t(residual);
        Eigen::Map<Eigen::Vector3d> r_q(residual+3);

        // 位置誤差
        r_t = sqrt_mat.block<3, 3>(0, 0) * (q_mes * q_xj.inverse() * t_xi - q_mes * q_xj.inverse() * t_xj + t_mes);

        // 姿態誤差
        r_q = sqrt_mat.block<3, 3>(3, 3) * (Rotation::quaternion2rotvec(q_mes * q_xj.inverse() * q_xi));

        return true;
    }


    Eigen::Vector3d t_mes;
    Eigen::Quaterniond q_mes;
    Eigen::Matrix<double, 6, 6> info_mat;
    Eigen::Matrix<double, 6, 6> sqrt_mat;
    
};






#endif