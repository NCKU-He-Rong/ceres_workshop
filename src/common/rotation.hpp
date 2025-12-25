#ifndef ROTATION_HPP_
#define ROTATION_HPP_

/*
 * Copyright (C) 2025 IEC Lab, DAA, NCKU
 *
 *     Author : Rong He
 *    Contact : P48101021@gs.ncku.edu.tw
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
  static Eigen::Quaterniond rotvec2quaternion(const Eigen::Vector3d& rotvec) {
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
  // - 不建議用 theta = 2 * acos(q.w())：
  //   因為 acos 在 1 跟 -1 附近非常敏感 (對應到0度跟180度)，theta 接近 0 時數值精度容易變差
  // - 常見較穩健的作法是使用 atan2 形式 (例如 2*atan2(||q_vec||, q_w))
  // - 這裡直接使用 Eigen::AngleAxisd(q)，Eigen 會採取上面那個較穩健的策略並把角度規範到 [0, pi]
  static Eigen::Vector3d quaternion2rotvec(const Eigen::Quaterniond& quaternion) {
    const Eigen::Quaterniond q(quaternion);

    // 若以 quaternion 建構 AngleAxisd，Eigen 會協助規範 angle/axis（[0, pi]）。
    const Eigen::AngleAxisd angle_axis(q);
    return angle_axis.angle() * angle_axis.axis();
  }

  // so(3) hat: phi -> phi^ (skew-symmetric)
  // 說明：
  // - 這邊我們這邊不規範vec的norm()值一定要在pi之內，是因為不是只有李代數才會用到so3_hat，
  //   其他任意向量也會用到，例如se3_jr() 
  static Eigen::Matrix3d so3_hat(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;
    mat << 0.0, -vec.z(), vec.y(),
        vec.z(), 0.0, -vec.x(),
        -vec.y(), vec.x(), 0.0;
    return mat;
  }

  // so(3) vee: phi^ (skew-symmetric) -> phi
  static Eigen::Vector3d so3_vee(const Eigen::Matrix3d& mat) {
    return Eigen::Vector3d(mat(2, 1), mat(0, 2), mat(1, 0));
  }

  // so(3) exp: phi -> R
  static Eigen::Matrix3d so3_exp(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;

    const double theta = vec.norm();
		// 這邊加入 kPi + kTol 是因為有時候會有一些毛刺theta很接近pi，但會超出一點點(約1e-12)，
		// 因此如果透過 theta <= (kPi) 檢查，將會出錯
    ensure(theta <= (kPi + kTol),
           "Angle-axis representation requires |theta| <= pi");

    const Eigen::Vector3d axis = vec.normalized();

    // 小角度時直接回傳 I，避免數值放大
    if (theta < kTol) {
      mat.setIdentity();
      return mat;
    }

    // Rodrigues 公式
    mat = (1.0 - std::cos(theta)) * axis * axis.transpose() +
          std::sin(theta) * so3_hat(axis) +
          std::cos(theta) * Eigen::Matrix3d::Identity();
    return mat;
  }

  // so(3) log: R -> phi
  static Eigen::Vector3d so3_log(const Eigen::Matrix3d& mat) {
    ensure(std::abs(mat.determinant() - 1.0) <= kTol,
           "Input Matrix is not a rotation matrix (det != 1)");

    // axis_raw = 2*sin(theta)*axis
    const Eigen::Vector3d axis_raw(mat(2, 1) - mat(1, 2),
                                   mat(0, 2) - mat(2, 0),
                                   mat(1, 0) - mat(0, 1));

    // 傳統用 acos 在 1 跟 -1 附近非常敏感 (對應到0度跟180度)，這裡改用 atan2 較穩定
    const double cth = (mat.trace() - 1.0) / 2.0;
    const double axis_raw_norm = axis_raw.norm();
    // 雖然 std::atan2的範圍是[-pi, pi]，但是axis_raw_norm 一定大於0，
    // 因此下面的theta範圍是[0, pi]
    const double theta = std::atan2(0.5 * axis_raw_norm, cth);  

    if (theta < kTol) {
      return Eigen::Vector3d::Zero();
    }

    // 當 theta ≈ pi 時，R 會趨近對稱矩陣，此時
		// axis_raw = 2*sin(theta)*axis ≈ 0（因 sin(pi)=0），無法直接從 axis_raw 取回旋轉軸。
		// 因此改以 (R - I) 的零空間 (nullspace) 取得旋轉軸：
		// - A = (R - I) 的 rank = 2 (必定)
		// - 其零空間維度為 1，對應的向量即為旋轉軸 axis
		// - 在 SVD 中可由 V 的最後一個 column 取得 (0奇異值對應的奇異向量)
		if (std::abs(theta - kPi) < kTol) {
				const Eigen::Matrix3d A = mat - Eigen::Matrix3d::Identity();
			
				// Eigen 的分解需要明確指定要計算哪些項目，此處只需要 V 以取得 nullspace。
				// (若需要 U/V 完整資訊，可用：ComputeFullU | ComputeFullV)
				const Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullV);
			
				Eigen::Vector3d axis = svd.matrixV().col(2);  // nullspace direction
				axis.normalize();
				return theta * axis;
		}

		Eigen::Vector3d axis = axis_raw;
		axis.normalize();
		return theta * axis;
	}

  // so(3) 右 Jacobian: Jr(phi)
  static Eigen::Matrix3d so3_jr(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;

    const double theta = vec.norm();
    ensure(theta <= (kPi + kTol),
           "Angle-axis representation requires |theta| <= pi");

    const Eigen::Vector3d axis = vec.normalized();

    if (theta < kTol) {
      mat.setIdentity();
      return mat;
    }

    const double coeff_1 = std::sin(theta) / theta;
    const double coeff_2 = (1.0 - std::cos(theta)) / theta;

    mat = coeff_1 * Eigen::Matrix3d::Identity() +
          (1.0 - coeff_1) * axis * axis.transpose() -
          coeff_2 * so3_hat(axis);
    return mat;
  }

  // so(3) 右 Jacobian inverse: Jr^{-1}(phi)
  static Eigen::Matrix3d so3_jr_inv(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;

    const double theta = vec.norm();
    ensure(theta <= (kPi + kTol),
           "Angle-axis representation requires |theta| <= pi");

    const Eigen::Vector3d axis = vec.normalized();

    if (theta < kTol) {
      mat.setIdentity();
      return mat;
    }

    // 小技巧：避免直接用 1/tan()，其90度時雖不一定報錯，但可能回傳非常接近 0 的數值，
		// 雖然正確，但是盡量避免
    const double coeff =
        (theta / 2.0) * std::cos(theta / 2.0) / std::sin(theta / 2.0);

    mat = coeff * Eigen::Matrix3d::Identity() +
          (1.0 - coeff) * axis * axis.transpose() +
          (theta / 2.0) * so3_hat(axis);
    return mat;
  }

  // so(3) 左 Jacobian inverse: Jl(phi)
  static Eigen::Matrix3d so3_jl(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;

    const double theta = vec.norm();
    ensure(theta <= (kPi + kTol),
           "Angle-axis representation requires |theta| <= pi");

    const Eigen::Vector3d axis = vec.normalized();

    if (theta < kTol) {
      mat.setIdentity();
      return mat;
    }

    const double coeff_1 = std::sin(theta) / theta;
    const double coeff_2 = (1.0 - std::cos(theta)) / theta;

    mat = coeff_1 * Eigen::Matrix3d::Identity() +
          (1.0 - coeff_1) * axis * axis.transpose() +
          coeff_2 * so3_hat(axis);
    return mat;
  }

  // so(3) 左 Jacobian inverse: Jl^{-1}(phi)
  static Eigen::Matrix3d so3_jl_inv(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;

    const double theta = vec.norm();
    ensure(theta <= (kPi + kTol),
           "Angle-axis representation requires |theta| <= pi");

    const Eigen::Vector3d axis = vec.normalized();

    if (std::abs(theta) < kTol) {
      mat.setIdentity();
      return mat;
    }

    // 同上: 避免使用 1/tan()
    const double coeff =
        (theta / 2.0) * std::cos(theta / 2.0) / std::sin(theta / 2.0);

    mat = coeff * Eigen::Matrix3d::Identity() +
          (1.0 - coeff) * axis * axis.transpose() -
          (theta / 2.0) * so3_hat(axis);
    return mat;
  }

  // se(3) hat：se -> 4x4 矩陣
  static Eigen::Matrix4d se3_hat(const Eigen::Matrix<double, 6, 1>& vec) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
    mat.block<3, 3>(0, 0) = so3_hat(vec.tail<3>());
    mat.block<3, 1>(0, 3) = vec.head<3>();
    return mat;
  }

  // se(3) vee：4x4 矩陣 -> se
  static Eigen::Matrix<double, 6, 1> se3_vee(const Eigen::Matrix4d& mat) {
    Eigen::Matrix<double, 6, 1> vec = Eigen::Matrix<double, 6, 1>::Zero();
    vec.head<3>() = mat.block<3, 1>(0, 3);
    vec.tail<3>() = so3_vee(mat.block<3, 3>(0, 0));
    return vec;
  }

  // se(3) exp: se -> T
  static Eigen::Matrix4d se3_exp(const Eigen::Matrix<double, 6, 1>& vec) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
    mat(3, 3) = 1.0;

    mat.block<3, 3>(0, 0) = so3_exp(vec.tail<3>());
    mat.block<3, 1>(0, 3) = so3_jl(vec.tail<3>()) * vec.head<3>();
    return mat;
  }

  // se(3) log: T -> se
  static Eigen::Matrix<double, 6, 1> se3_log(const Eigen::Matrix4d& mat) {
    ensure(std::abs(mat(3, 3) - 1.0) < kTol,
           "Input Matrix is not a transformation matrix");

    Eigen::Matrix<double, 6, 1> vec = Eigen::Matrix<double, 6, 1>::Zero();
    vec.tail<3>() = so3_log(mat.block<3, 3>(0, 0));
    vec.head<3>() = so3_jl_inv(vec.tail<3>()) * mat.block<3, 1>(0, 3);
    return vec;
  }

  // se(3) 右 Jacobian (此處為近似)
  static Eigen::Matrix<double, 6, 6> se3_jr(const Eigen::Matrix<double, 6, 1>& vec) {
    Eigen::Matrix<double, 6, 6> mat;

    ensure(vec.tail<3>().norm() <= (kPi + kTol),
           "Angle-axis representation requires |theta| <= pi");

    Eigen::Matrix<double, 6, 6> coeff = Eigen::Matrix<double, 6, 6>::Zero();
    coeff.block<3, 3>(0, 0) = so3_hat(vec.tail<3>());
    coeff.block<3, 3>(3, 3) = so3_hat(vec.tail<3>());
    coeff.block<3, 3>(0, 3) = so3_hat(vec.head<3>());

    mat = Eigen::Matrix<double, 6, 6>::Identity() - 0.5 * coeff;
    return mat;
  }

  // se(3) 右 Jacobian inverse (此處為近似)
  static Eigen::Matrix<double, 6, 6> se3_jr_inv(const Eigen::Matrix<double, 6, 1>& vec) {
    Eigen::Matrix<double, 6, 6> mat;

    ensure(vec.tail<3>().norm() <= (kPi + kTol),
           "Angle-axis representation requires |theta| <= pi");

    Eigen::Matrix<double, 6, 6> coeff = Eigen::Matrix<double, 6, 6>::Zero();
    coeff.block<3, 3>(0, 0) = so3_hat(vec.tail<3>());
    coeff.block<3, 3>(3, 3) = so3_hat(vec.tail<3>());
    coeff.block<3, 3>(0, 3) = so3_hat(vec.head<3>());

    mat = Eigen::Matrix<double, 6, 6>::Identity() + 0.5 * coeff;
    return mat;
  }

  // se(3) 左 Jacobian (此處為近似)
  static Eigen::Matrix<double, 6, 6> se3_jl(const Eigen::Matrix<double, 6, 1>& vec) {
    Eigen::Matrix<double, 6, 6> mat;

    ensure(vec.tail<3>().norm() <= (kPi + kTol),
           "Angle-axis representation requires |theta| <= pi");

    Eigen::Matrix<double, 6, 6> coeff = Eigen::Matrix<double, 6, 6>::Zero();
    coeff.block<3, 3>(0, 0) = so3_hat(vec.tail<3>());
    coeff.block<3, 3>(3, 3) = so3_hat(vec.tail<3>());
    coeff.block<3, 3>(0, 3) = so3_hat(vec.head<3>());

    mat = Eigen::Matrix<double, 6, 6>::Identity() + 0.5 * coeff;
    return mat;
  }

  // se(3) 左 Jacobian inverse (此處為近似)
  static Eigen::Matrix<double, 6, 6> se3_jl_inv(const Eigen::Matrix<double, 6, 1>& vec) {
    Eigen::Matrix<double, 6, 6> mat;

    ensure(vec.tail<3>().norm() <= (kPi + kTol),
           "Angle-axis representation requires |theta| <= pi");

    Eigen::Matrix<double, 6, 6> coeff = Eigen::Matrix<double, 6, 6>::Zero();
    coeff.block<3, 3>(0, 0) = so3_hat(vec.tail<3>());
    coeff.block<3, 3>(3, 3) = so3_hat(vec.tail<3>());
    coeff.block<3, 3>(0, 3) = so3_hat(vec.head<3>());

    mat = Eigen::Matrix<double, 6, 6>::Identity() - 0.5 * coeff;
    return mat;
  }

 private:
  static constexpr double kTol = 1e-12;
  static constexpr double kPi = 3.14159265358979323846;
};

#endif  // ROTATION_HPP_
