#ifndef COSTFUNCTORE_HPP_
#define COSTFUNCTORE_HPP_


#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "rotation.hpp"


class CostFunctor: public ceres::SizedCostFunction<6, 7, 7>
{
public:
    CostFunctor(const Eigen::Vector3d &t,
                const Eigen::Quaterniond &q,
                const Eigen::Matrix<double, 21, 1> &info_vec)
                : t_mes(t), q_mes(q)
    {
        int k = 0;
        for (int i=0;i<6;i++)
        {
            for (int j=i;j<6;j++)
            {
                info_mat(i, j) = info_vec(k++);
            }
        }
        info_mat = info_mat.selfadjointView<Eigen::Upper>();
        sqrt_info_mat = info_mat.cwiseAbs().cwiseSqrt();

        q_mes.normalize();
    }

    bool Evaluate(const double* const* parameters,
                  double * residual,
                  double ** jacobians) const override
    {
        Eigen::Vector3d t_xi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond q_xi(parameters[0][3], parameters[0][4], parameters[0][5], parameters[0][6]);

        Eigen::Vector3d t_xj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond q_xj(parameters[1][3], parameters[1][4], parameters[1][5], parameters[1][6]);

        Eigen::Map<Eigen::Vector3d> r_t_w(residual);
        Eigen::Map<Eigen::Vector3d> r_q_w(residual+3);

        r_t_w = sqrt_info_mat.block<3, 3>(0, 0) * (q_mes * q_xj.inverse() * t_xi - q_mes * q_xj.inverse() * t_xj + t_mes);

        Eigen::Vector3d r_q = Rotation::QuaternionToRotvec((q_mes * q_xj.inverse() * q_xi).normalized());
        r_q_w = sqrt_info_mat.block<3, 3>(3, 3) * r_q;

        
        if (jacobians != nullptr)
        {
            if (jacobians[0] != nullptr)
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j_xi(jacobians[0]);
                j_xi.setZero();
                j_xi.block<3, 3>(0, 0) = (q_mes * q_xj.inverse()).toRotationMatrix();
                j_xi.block<3, 3>(3, 3) = Rotation::SO3RightJacobianInverse(r_q);
                j_xi = sqrt_info_mat * j_xi;
            }

            if (jacobians[1] != nullptr)
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j_xj(jacobians[1]);
                j_xj.setZero();
                j_xj.block<3, 3>(0, 0) = -(q_mes * q_xj.inverse()).toRotationMatrix();
                j_xj.block<3, 3>(3, 3) = -Rotation::SO3LeftJacobianInverse(r_q) * q_mes.toRotationMatrix();
                j_xj = sqrt_info_mat * j_xj;
            }

        }

        return true;
    }



private:
    Eigen::Vector3d t_mes;
    Eigen::Quaterniond q_mes;
    Eigen::Matrix<double, 6, 6> info_mat;
    Eigen::Matrix<double, 6, 6> sqrt_info_mat;
};
#endif