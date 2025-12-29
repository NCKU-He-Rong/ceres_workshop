#ifndef ROTATION_HPP_
#define ROTATION_HPP_

/*
 * Copyright (C) 2025 IEC Lab, DAA, NCKU
 *
 *     Author : Rong He
 *    Contact : P48101021@gs.ncku.edu.tw
 * 
 *  This rotation.hpp implementation has been rigorously cross-validated 
 *  against Sophus library.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Eigen/Dense>
#include "debug.hpp"

class Rotation {
 public:
  // 將角軸 (rotation vector / angle-axis) 轉換為四元數 (quaternion)
  //
  // 說明：
  // - 以下等效閉式公式(如 cos(theta/2), sin(theta/2)*axis)同時幫我處理 theta 接近 0 時，
  //   axis 的正規化數值不穩
  // - 同時加入「方向性約束」: 因為q 與 -q 表示同一旋轉，這裡統一強制 q.w() >= 0
  static Eigen::Quaterniond RotvecToQuaternion(const Eigen::Vector3d& rotvec) {
    const double angle = rotvec.norm();
    const Eigen::Vector3d axis = rotvec.normalized();

    // 注意：Eigen 不會自動替 quaternion 統一符號 (q 與 -q 表示同一旋轉)
    Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis));

    // 統一符號慣例：強制 q.w() >= 0。
    if (q.w() < 0.0) {
      q.coeffs() *= -1.0;
    }
    return q;
  }

  // 將四元數 (quaternion) 轉換為角軸 (rotation vector / angle-axis)
  //
  // 說明：
  // - 不建議用 theta = 2 * acos(q.w()):
  //   因為 acos 在 1 跟 -1 附近非常敏感 (對應到0度跟180度)，theta 接近 0 時數值精度容易變差
  // - 常見較穩健的作法是使用 atan2 形式 (例如 2*atan2(||q_vec||, q_w))
  // - 這裡直接使用 Eigen::AngleAxisd(q)，Eigen 會採取上面那個較穩健的策略,其輸出角度範圍為 [0, pi] 
  //   因 2*atan2(||q_vec|| > 0
  static Eigen::Vector3d QuaternionToRotvec(const Eigen::Quaterniond& quaternion) {
    // 若以 quaternion 建構 AngleAxisd，Eigen 因為上面公式的關係，其輸出角度範圍一定為 [0, pi] 
    const Eigen::AngleAxisd angle_axis(quaternion);
    return angle_axis.angle() * angle_axis.axis();
  }

  // so(3) hat: phi -> phi^ (skew-symmetric)
  // 說明：
  // - 這邊我們這邊不規範vec的norm()值一定要在pi之內，是因為不是只有李代數才會用到SO3Hat，
  //   其他任意向量也會用到，例如SE3RightJacobian() 
  static Eigen::Matrix3d SO3Hat(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;
    mat << 0.0, -vec.z(), vec.y(),
        vec.z(), 0.0, -vec.x(),
        -vec.y(), vec.x(), 0.0;
    return mat;
  }

  // so(3) vee: phi^ (skew-symmetric) -> phi
  static Eigen::Vector3d SO3Vee(const Eigen::Matrix3d& mat) {
    return Eigen::Vector3d(mat(2, 1), mat(0, 2), mat(1, 0));
  }

  // so(3) exp: phi -> R
  // 說明：
  // - 這邊不應該去限制 vec.norm()值一定得在 [0, pi]，因為我們在最佳化過程中，
  //   有機會得到不是在 [0, pi] 更新量，這個更新量是對的，所以我們不能將之視為錯誤
  // - 這邊我們直接嫁接 RotvecToQuaternion() 函數，其不論輸入的 vec 是否在範圍內
  //   一定會輸出q.w()>0 的 quaternion，我們在藉此轉換成 DCM
  static Eigen::Matrix3d SO3Exp(const Eigen::Vector3d& vec) {
    return RotvecToQuaternion(vec).toRotationMatrix();
  }

  // so(3) log: R -> phi
  // 說明：
  // - 如果要用公式去進行轉換(1 / (2 * sin(theta))) 的那個，就要考慮到三種情況
  //   theta = 0，theta = pi，以及其他，其中theta = pi需要透過 R-I 的nullspace vector來求解
  static Eigen::Vector3d SO3Log(const Eigen::Matrix3d& mat) {
    ensure(std::abs(mat.determinant() - 1.0) <= kTol,
           "[SO3Log] Input Matrix is not a rotation matrix (det != 1)");
    
    Eigen::Quaterniond quat(mat);
    quat.normalize();
    return QuaternionToRotvec(quat);
  }


  // so(3) 右 Jacobian: Jr(phi)
  // 說明：
  // - 這邊一樣不能去限制 vec.norm() 一定要在 [0, pi] 之間，
  //   因為我們在最佳化過程中，有機會得到不是在 [0, pi] 更新量，
  //   尤其在以 SE3 當作最佳化變量的時候，我們可能得到一個 6 * 1 的更新量，
  //   但是其旋轉部份，並不在 [0, pi] 之間，但該更新量要進行SE3Exp 的時候，
  //   會需要SO3LeftJacobian()，去反推平移部份，這個時候SO3LeftJacobian()要代入的是
  //   不在 [0, pi] 之間的 SO3 李代數, 因此所有的SO3Jacobian系列都不能限制 vec 一定要在 [0, pi]
  static Eigen::Matrix3d SO3RightJacobian(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;

    const double theta = vec.norm();
    const Eigen::Vector3d axis = vec.normalized();

    if (theta < kTol) {
      mat.setIdentity();
      return mat;
    }

    const double coeff_1 = std::sin(theta) / theta;
    const double coeff_2 = (1.0 - std::cos(theta)) / theta;

    mat = coeff_1 * Eigen::Matrix3d::Identity() +
          (1.0 - coeff_1) * axis * axis.transpose() -
          coeff_2 * SO3Hat(axis);
    return mat;
  }

  // so(3) 右 Jacobian inverse: Jr^{-1}(phi)
  static Eigen::Matrix3d SO3RightJacobianInverse(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;

    const double theta = vec.norm();
    const Eigen::Vector3d axis = vec.normalized();

    if (theta < kTol) {
      mat.setIdentity();
      return mat;
    }

    // 小技巧：避免直接用 1/tan()，其90度時雖不一定報錯，但可能回傳非常接近 0 的數值，雖然正確，但是盡量避免
    const double coeff =
          (theta / 2.0) * std::cos(theta / 2.0) / std::sin(theta / 2.0);

    mat = coeff * Eigen::Matrix3d::Identity() +
          (1.0 - coeff) * axis * axis.transpose() +
          (theta / 2.0) * SO3Hat(axis);
    return mat;
  }

  // so(3) 左 Jacobian inverse: Jl(phi)
  static Eigen::Matrix3d SO3LeftJacobian(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;

    const double theta = vec.norm();
    const Eigen::Vector3d axis = vec.normalized();

    if (theta < kTol) {
      mat.setIdentity();
      return mat;
    }

    const double coeff_1 = std::sin(theta) / theta;
    const double coeff_2 = (1.0 - std::cos(theta)) / theta;

    mat = coeff_1 * Eigen::Matrix3d::Identity() +
          (1.0 - coeff_1) * axis * axis.transpose() +
          coeff_2 * SO3Hat(axis);
    return mat;
  }

  // so(3) 左 Jacobian inverse: Jl^{-1}(phi)
  static Eigen::Matrix3d SO3LeftJacobianInverse(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;

    const double theta = vec.norm();
    const Eigen::Vector3d axis = vec.normalized();

    if (std::abs(theta) < kTol) {
      mat.setIdentity();
      return mat;
    }

    // 同上理由，避免使用 1/tan()
    const double coeff =
        (theta / 2.0) * std::cos(theta / 2.0) / std::sin(theta / 2.0);

    mat = coeff * Eigen::Matrix3d::Identity() +
          (1.0 - coeff) * axis * axis.transpose() -
          (theta / 2.0) * SO3Hat(axis);
    return mat;
  }

  // se(3) hat：se -> 4x4 矩陣
  static Eigen::Matrix4d SE3Hat(const Eigen::Matrix<double, 6, 1>& vec) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
    mat.block<3, 3>(0, 0) = SO3Hat(vec.tail<3>());
    mat.block<3, 1>(0, 3) = vec.head<3>();
    return mat;
  }

  // se(3) vee：4x4 矩陣 -> se
  static Eigen::Matrix<double, 6, 1> SE3Vee(const Eigen::Matrix4d& mat) {
    Eigen::Matrix<double, 6, 1> vec = Eigen::Matrix<double, 6, 1>::Zero();
    vec.head<3>() = mat.block<3, 1>(0, 3);
    vec.tail<3>() = SO3Vee(mat.block<3, 3>(0, 0));
    return vec;
  }

  // se(3) exp: se -> T
  static Eigen::Matrix4d SE3Exp(const Eigen::Matrix<double, 6, 1>& vec) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
    mat(3, 3) = 1.0;

    mat.block<3, 3>(0, 0) = SO3Exp(vec.tail<3>());
    mat.block<3, 1>(0, 3) = SO3LeftJacobian(vec.tail<3>()) * vec.head<3>();
    return mat;
  }

  // se(3) log: T -> se
  static Eigen::Matrix<double, 6, 1> SE3Log(const Eigen::Matrix4d& mat) {
    ensure(std::abs(mat(3, 3) - 1.0) < kTol,
           "[SE3Log] Input Matrix is not a transformation matrix");

    Eigen::Matrix<double, 6, 1> vec = Eigen::Matrix<double, 6, 1>::Zero();
    vec.tail<3>() = SO3Log(mat.block<3, 3>(0, 0));
    vec.head<3>() = SO3LeftJacobianInverse(vec.tail<3>()) * mat.block<3, 1>(0, 3);
    return vec;
  }

  // se(3) 右 Jacobian (此處為近似)
  static Eigen::Matrix<double, 6, 6> SE3RightJacobian(const Eigen::Matrix<double, 6, 1>& vec) {
    Eigen::Matrix<double, 6, 6> mat = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> coeff = Eigen::Matrix<double, 6, 6>::Zero();
    coeff.block<3, 3>(0, 0) = SO3Hat(vec.tail<3>());
    coeff.block<3, 3>(3, 3) = SO3Hat(vec.tail<3>());
    coeff.block<3, 3>(0, 3) = SO3Hat(vec.head<3>());

    mat = Eigen::Matrix<double, 6, 6>::Identity() - 0.5 * coeff;
    return mat;
  }

  // se(3) 右 Jacobian inverse (此處為近似)
  static Eigen::Matrix<double, 6, 6> SE3RightJacobianInverse(const Eigen::Matrix<double, 6, 1>& vec) {
    Eigen::Matrix<double, 6, 6> mat = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> coeff = Eigen::Matrix<double, 6, 6>::Zero();
    coeff.block<3, 3>(0, 0) = SO3Hat(vec.tail<3>());
    coeff.block<3, 3>(3, 3) = SO3Hat(vec.tail<3>());
    coeff.block<3, 3>(0, 3) = SO3Hat(vec.head<3>());

    mat = Eigen::Matrix<double, 6, 6>::Identity() + 0.5 * coeff;
    return mat;
  }

  // se(3) 左 Jacobian (此處為近似)
  static Eigen::Matrix<double, 6, 6> SE3LeftJacobian(const Eigen::Matrix<double, 6, 1>& vec) {
    Eigen::Matrix<double, 6, 6> mat = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> coeff = Eigen::Matrix<double, 6, 6>::Zero();
    coeff.block<3, 3>(0, 0) = SO3Hat(vec.tail<3>());
    coeff.block<3, 3>(3, 3) = SO3Hat(vec.tail<3>());
    coeff.block<3, 3>(0, 3) = SO3Hat(vec.head<3>());

    mat = Eigen::Matrix<double, 6, 6>::Identity() + 0.5 * coeff;
    return mat;
  }

  // se(3) 左 Jacobian inverse (此處為近似)
  static Eigen::Matrix<double, 6, 6> SE3LeftJacobianInverse(const Eigen::Matrix<double, 6, 1>& vec) {
    Eigen::Matrix<double, 6, 6> mat;
    Eigen::Matrix<double, 6, 6> coeff = Eigen::Matrix<double, 6, 6>::Zero();
    coeff.block<3, 3>(0, 0) = SO3Hat(vec.tail<3>());
    coeff.block<3, 3>(3, 3) = SO3Hat(vec.tail<3>());
    coeff.block<3, 3>(0, 3) = SO3Hat(vec.head<3>());

    mat = Eigen::Matrix<double, 6, 6>::Identity() - 0.5 * coeff;
    return mat;
  }

  // se(3) se3 的 Adjoint 矩陣
  static Eigen::Matrix<double, 6, 6> SE3Adjoint(const Eigen::Matrix4d &mat) {
    Eigen::Matrix3d rot_mat = mat.block<3, 3>(0, 0);
    Eigen::Vector3d t = mat.block<3, 1>(0, 3);

    ensure(std::abs(mat(3, 3) - 1.0) < kTol,
           "[SE3Adjoint] Input Matrix is not a transformation matrix (T(3, 3) != 1)");
    ensure(std::abs(rot_mat.determinant() - 1.0) <= kTol,
           "[SE3Adjoint] Input Matrix is not a transformation matrix (det(R) != 1)");

    Eigen::Matrix<double, 6, 6> adj_mat = Eigen::Matrix<double, 6, 6>::Zero();
    adj_mat.block<3, 3>(0, 0) = rot_mat;
    adj_mat.block<3, 3>(3, 3) = rot_mat;
    adj_mat.block<3, 3>(0, 3) = SO3Hat(t) * rot_mat;
    return adj_mat;
  }

  // 弧度轉換成角度
  static double RadToDeg(const double &angle) {
    return angle / kPi * 180.0;
  }

  // 角度轉換成弧度
  static double DegToRad(const double &angle) {
    return angle / 180.0 * kPi;
  }

  // x軸旋轉矩陣 (單位為角度)
  static Eigen::Matrix3d RotXDeg(const double &angle) {
    Eigen::Matrix3d mat;
    double value = DegToRad(angle);
    mat << 1, 0, 0, 
        0, std::cos(value), -std::sin(value),
        0, std::sin(value), std::cos(value);
    return mat;
  }

  // y軸旋轉矩陣 (單位為角度)
  static Eigen::Matrix3d RotYDeg(const double &angle) {
    Eigen::Matrix3d mat;
    double value = DegToRad(angle);
    mat << std::cos(value), 0, std::sin(value), 
        0, 1, 0, 
        -std::sin(value), 0, std::cos(value);
    return mat;
  }

  // z軸旋轉矩陣 (單位為角度)
  static Eigen::Matrix3d RotZDeg(const double &angle) {
    Eigen::Matrix3d mat;
    double value = DegToRad(angle);
    mat << std::cos(value), -std::sin(value), 0,
           std::sin(value), std::cos(value), 0,
           0, 0, 1;
    return mat;
  }
  

 private:
  static constexpr double kTol = 1e-12;
  static constexpr double kPi = 3.14159265358979323846;
};

#endif  // ROTATION_HPP_
