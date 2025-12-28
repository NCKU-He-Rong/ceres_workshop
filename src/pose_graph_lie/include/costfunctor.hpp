#ifndef COSTFUNCTOR_HPP_
#define COSTFUNCTOR_HPP_

#include "ceres/ceres.h"
#include <Eigen/Dense>
#include "rotation.hpp"

class CostFunctor: public ceres::SizedCostFunction<6, 6, 6> {
 public:
  CostFunctor(const Eigen::Vector3d &t_mes, const Eigen::Quaterniond &q_mes, Eigen::Matrix<double, 21, 1> &info) {
    mes_.setZero();
    mes_(3, 3) = 1.0;
    info_.setZero();
    sqrt_info_.setZero();

    mes_.block<3, 3>(0, 0) = q_mes.normalized().toRotationMatrix();
    mes_.block<3, 1>(0, 3) = t_mes;

    int k = 0;
    for (int i=0;i<6;i++) {
      for (int j=i;j<6;j++) {
        info_(i, j) = info(k++);
      }
    }

    info_ = info_.selfadjointView<Eigen::Upper>();
    sqrt_info_ = info_.cwiseAbs().cwiseSqrt();
  }

  bool Evaluate (const double* const * parameters, 
                double* residual,
                double** jacobian) const {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> pose_x(parameters[0]);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> pose_y(parameters[1]);
    Eigen::Matrix<double, 6, 1> r = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Map<Eigen::Matrix<double, 6, 1>> r_w(residual);
    r = Rotation::SE3Log(mes_ * (Rotation::SE3Exp(pose_y).inverse()) * 
                         Rotation::SE3Exp(pose_x));
    r_w = sqrt_info_  * r;
    
    if (jacobian != nullptr) {
      if (jacobian[0] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> j_xi(jacobian[0]);
        j_xi = Rotation::SE3RightJacobianInverse(r);
        j_xi = sqrt_info_ * j_xi;
      }
      if (jacobian[1] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> j_xj(jacobian[1]);
        j_xj = -Rotation::SE3RightJacobianInverse(r) * 
                Rotation::SE3Adjoint(Rotation::SE3Exp(pose_x).inverse() * 
                                     Rotation::SE3Exp(pose_y));
        j_xj = sqrt_info_ * j_xj;
      }
    }
    return true;
  }
  
 private:
  Eigen::Matrix<double, 4, 4> mes_;
  Eigen::Matrix<double, 6, 6> info_;
  Eigen::Matrix<double, 6, 6> sqrt_info_;
};








#endif  // COSTFUNCTOR_HPP_