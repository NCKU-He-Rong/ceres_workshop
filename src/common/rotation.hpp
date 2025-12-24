#ifndef ROTATION_HPP_
#define ROTATION_HPP_

#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>


class Rotation
{
public:
    static constexpr double tol = 1e-12;

    // 將角軸表示法轉換成四元數表示法
    // 這邊不使用公式的原因是(就是cos(theta/2)...),程式還需要去判斷theta是否靠近0
    // 否則計算 axis vector的時候有機會爆炸
    // 但以下寫法都已經幫我們處理好了
    static Eigen::Quaterniond rotvec2quaternion(const Eigen::Vector3d &rotvec)
    {
        double angle = rotvec.norm();
        Eigen::Vector3d vec = rotvec.normalized();

        // 這個打法Eigen,並不會自動幫我處理好Quaternion的方向性約束
        Eigen::Quaterniond q(Eigen::AngleAxisd(angle, vec));

        // 進行方向性的約束因為(q跟-q都是同一個涵義)
        if (q.w() < 0) 
        {
            q.coeffs() *= -1;
        }
        return q;
    }

    // 將四元數表示法轉換成角軸表示法
    // 這邊一樣不使用公式,尤其theta = acos(q.w()) * 2 
    // 原因在於acos對於1附近的數值很敏感,可以參閱他的曲線: https://www.rapidtables.org/zh-TW/math/trigonometry/arccos.html
    // 所以對於theta=0附近的數值,解算精度不高
    // 真實上,我們必須避免透過acos解算小theta
    // 我們會用theta = atan(q.vec().norm(), q.w()) * 2 (atan的曲線對於theta=0的曲線也很陡峭,但沒acos嚴重)
    // 同樣的以下打法都已經處理好了
    static Eigen::Vector3d quaternion2rotvec(const Eigen::Quaterniond &quaternion)
    {   
        Eigen::Quaterniond q(quaternion);
        // 如果AngleAxisd是透過theta跟axis初始化,那其產出angle跟axis的時候"不會"幫我們都處理好(0~pi)之間
        // 但如果AngleAxisd是透過quaternion初始化, 那其產出angle跟axis的時候"會"幫我們都處理好(0~pi)之間
        Eigen::AngleAxisd rotvec(q);
        return rotvec.angle() * rotvec.axis();
        
    }

    static Eigen::Matrix3d so3_hat(const Eigen::Vector3d &vec)
    {
        Eigen::Matrix3d mat;
        mat.setZero();

        mat(0, 1) = -vec(2); mat(1, 0) =  vec(2);
        mat(0, 2) =  vec(1); mat(2, 0) = -vec(1);
        mat(1, 2) = -vec(0); mat(2, 1) =  vec(0);

        return mat;
    }

    static Eigen::Vector3d so3_vee(const Eigen::Matrix3d &mat)
    {
        Eigen::Vector3d vec;
        vec.setZero();
        vec(0) = mat(2, 1);
        vec(1) = mat(0, 2);
        vec(2) = mat(1, 0);
        
        return vec;
    }

    static Eigen::Matrix3d so3_exp(const Eigen::Vector3d &vec)
    {
        Eigen::Matrix3d mat;

        double theta = vec.norm();
        ensure(theta <= (M_PI + tol), "Angle-axis representation requires |theta| <= pi");

        Eigen::Vector3d axis = vec.normalized();

        if (abs(theta) < tol)
        {   
            mat.setIdentity();
            return mat;

        }
        else
        {
            mat = (1-cos(theta))*axis*axis.transpose() + sin(theta) * so3_hat(axis) + cos(theta) * Eigen::Matrix3d::Identity();
            return mat;
        }
    }

    static Eigen::Vector3d so3_log(const Eigen::Matrix3d &mat)
    {
        Eigen::Vector3d vec;

        ensure(abs(mat.determinant() - 1) <= tol, "Input Matrix is not a rotation matrix (det != 1)");


        // 
        Eigen::Vector3d axis(mat(2, 1) - mat(1, 2), 
                             mat(0, 2) - mat(2, 0),
                             mat(1, 0) - mat(0, 1));

        // 這邊傳統透過arccos的方式,在M_PI跟0附近數值相當敏感,所以要用atan2比較好
        double cth = (mat.trace() - 1.0 ) / 2.0;
        double sin = axis.norm();

        double theta = atan2(0.5*sin, cth);   // [-pi-pi]
        
        if (abs(theta) < tol)
        {
            return Eigen::Vector3d::Zero();
        }
        else if (abs(theta - M_PI) < tol)
        {
            Eigen::Matrix3d A = mat - Eigen::Matrix3d::Identity();
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullV);
            Eigen::Vector3d axis = svd.matrixV().col(2);
            axis.normalize();
            
            return theta * axis;
        }
        else
        {
            axis.normalize();
            return theta * axis;
        }
    }

    static Eigen::Matrix3d so3_jr(const Eigen::Vector3d &vec)
    {
        Eigen::Matrix3d mat;

        double theta = vec.norm();

        // 注意這個如果CMake設定成Release Mode就完全不會檢查這個了,直接跳過
        ensure(theta <= (M_PI + tol), "Angle-axis representation requires |theta| <= pi");

        Eigen::Vector3d axis = vec.normalized();

        if (abs(theta) < tol)
        {
            mat.setIdentity();
            return mat;
        }
        else
        {
            double coeff_1 = sin(theta) / theta;
            double coeff_2 = (1-cos(theta)) / theta;

            mat = coeff_1 * Eigen::Matrix3d::Identity() + 
                 (1-coeff_1) * axis * axis.transpose() - 
                  coeff_2 * so3_hat(axis);
            
            return mat;
        }
    }

    // 要確保abs(theta)是小於pi的
    static Eigen::Matrix3d so3_jr_inv(const Eigen::Vector3d &vec)
    {
        Eigen::Matrix3d mat;

        double theta = vec.norm();
        // 注意這個如果CMake設定成Release Mode就完全不會檢查這個了,直接跳過
        ensure(theta <= (M_PI + tol), "Angle-axis representation requires |theta| <= pi");

        Eigen::Vector3d axis = vec.normalized();

        if (abs(theta) < tol)
        {
            mat.setIdentity();
            return mat;
        }
        else
        {   
            // 這邊有一個技巧,不要使用1/tan(),雖然它會給也不會報錯,給你很接近0的數字
            double coeff = (theta / 2)  * cos(theta / 2) / sin(theta / 2);
            mat = coeff * Eigen::Matrix3d::Identity() + 
                 (1- coeff) * axis * axis.transpose() + 
                 (theta / 2) * so3_hat(axis);
            
            return mat;
        }

        return mat;
    }

    // 要確保abs(theta)是小於pi的
    static Eigen::Matrix3d so3_jl(const Eigen::Vector3d &vec)
    {
        Eigen::Matrix3d mat;

        double theta = vec.norm();
        // 注意這個如果CMake設定成Release Mode就完全不會檢查這個了,直接跳過
        ensure(theta <= (M_PI + tol), "Angle-axis representation requires |theta| <= pi");

        Eigen::Vector3d axis = vec.normalized();

        if (abs(theta) < tol)
        {
            mat.setIdentity();
            return mat;
        }
        else
        {
            double coeff_1 = sin(theta) / theta;
            double coeff_2 = (1-cos(theta)) / theta;

            mat = coeff_1 * Eigen::Matrix3d::Identity() + 
                 (1-coeff_1) * axis * axis.transpose() + 
                  coeff_2 * so3_hat(axis);
            
            return mat;
        }
    }

    // 要確保abs(theta)是小於pi的
    static Eigen::Matrix3d so3_jl_inv(const Eigen::Vector3d &vec)
    {
        Eigen::Matrix3d mat;

        double theta = vec.norm();

        // 注意這個如果CMake設定成Release Mode就完全不會檢查這個了,直接跳過
        ensure(theta <= (M_PI + tol) , "Angle-axis representation requires |theta| <= pi");
        
        Eigen::Vector3d axis = vec.normalized();

        if (abs(theta) < tol)
        {
            mat.setIdentity();
            return mat;
        }
        else
        {   
            // 這邊有一個技巧,不要使用1/tan(),雖然它會給也不會報錯,給你很接近0的數字
            double coeff = (theta / 2)  * cos(theta / 2) / sin(theta / 2);
            mat = coeff * Eigen::Matrix3d::Identity() + 
                 (1- coeff) * axis * axis.transpose() - 
                 (theta / 2) * so3_hat(axis);
            
            return mat;
        }
        return mat;
    }


    static Eigen::Matrix4d se3_hat(const Eigen::Matrix<double, 6, 1> &vec)
    {
        Eigen::Matrix4d mat;
        mat.setZero();

        mat.block<3, 3>(0, 0) = so3_hat(vec.tail<3>());
        mat.block<3, 1>(0, 3) = vec.head<3>();

        return mat;
    }

    static Eigen::Matrix<double, 6, 1> se3_vee(const Eigen::Matrix4d &mat)
    {
        Eigen::Matrix<double, 6, 1> vec;
        vec.setZero();
        vec.head<3>() = mat.block<3, 1>(0, 3);
        vec.tail<3>() = so3_vee(mat.block<3, 3>(0, 0));

        return vec;
    }

    static Eigen::Matrix4d se3_exp(const Eigen::Matrix<double, 6, 1> &vec)
    {
        Eigen::Matrix4d mat;
        mat.setZero();
        mat(3, 3) = 1;

        mat.block<3, 3>(0, 0) = so3_exp(vec.tail<3>());
        mat.block<3, 1>(0, 3) = so3_jl(vec.tail<3>()) * vec.head<3>();

        return mat;
    }

    static Eigen::Matrix<double, 6, 1> se3_log(const Eigen::Matrix4d &mat)
    {
        ensure(abs(mat(3, 3) - 1.0) < tol, "Input Matrix is not a transformation matrix");
        Eigen::Matrix<double, 6, 1> vec;
        vec.setZero();
        vec.tail<3>() = so3_log(mat.block<3, 3>(0, 0));
        vec.head<3>() = so3_jl_inv(vec.tail<3>()) * mat.block<3, 1>(0, 3);

        return vec;
    }

    static Eigen::Matrix<double, 6, 6> se3_jr(const Eigen::Matrix<double, 6, 1> &vec)
    {
        Eigen::Matrix<double, 6, 6> mat;

        ensure(vec.tail<3>().norm() <= (M_PI + tol) , "Angle-axis representation requires |theta| <= pi");

        Eigen::Matrix<double, 6, 6> coeff;
        coeff.setZero();
        coeff.block<3, 3>(0, 0) = so3_hat(vec.tail<3>());
        coeff.block<3, 3>(3, 3) = so3_hat(vec.tail<3>());
        coeff.block<3, 3>(0, 3) = so3_hat(vec.head<3>());

        mat = Eigen::Matrix<double, 6, 6>::Identity() - 0.5*coeff;
    
        return mat;
    }

    static Eigen::Matrix<double, 6, 6> se3_jr_inv(const Eigen::Matrix<double, 6, 1> &vec)
    {
        Eigen::Matrix<double, 6, 6> mat;

        ensure(vec.tail<3>().norm() <= (M_PI + tol) , "Angle-axis representation requires |theta| <= pi");

        Eigen::Matrix<double, 6, 6> coeff;
        coeff.setZero();
        coeff.block<3, 3>(0, 0) = so3_hat(vec.tail<3>());
        coeff.block<3, 3>(3, 3) = so3_hat(vec.tail<3>());
        coeff.block<3, 3>(0, 3) = so3_hat(vec.head<3>());

        mat = Eigen::Matrix<double, 6, 6>::Identity() + 0.5*coeff;

        return mat;
    }

    static Eigen::Matrix<double, 6, 6> se3_jl(const Eigen::Matrix<double, 6, 1> &vec)
    {
        Eigen::Matrix<double, 6, 6> mat;

        ensure(vec.tail<3>().norm() <= (M_PI + tol) , "Angle-axis representation requires |theta| <= pi");

        Eigen::Matrix<double, 6, 6> coeff;
        coeff.setZero();
        coeff.block<3, 3>(0, 0) = so3_hat(vec.tail<3>());
        coeff.block<3, 3>(3, 3) = so3_hat(vec.tail<3>());
        coeff.block<3, 3>(0, 3) = so3_hat(vec.head<3>());

        mat = Eigen::Matrix<double, 6, 6>::Identity() + 0.5*coeff;

        return mat;
    }

    static Eigen::Matrix<double, 6, 6> se3_jl_inv(const Eigen::Matrix<double, 6, 1> &vec)
    {
        Eigen::Matrix<double, 6, 6> mat;

        ensure(vec.tail<3>().norm() <= (M_PI + tol) , "Angle-axis representation requires |theta| <= pi");

        Eigen::Matrix<double, 6, 6> coeff;
        coeff.setZero();
        coeff.block<3, 3>(0, 0) = so3_hat(vec.tail<3>());
        coeff.block<3, 3>(3, 3) = so3_hat(vec.tail<3>());
        coeff.block<3, 3>(0, 3) = so3_hat(vec.head<3>());

        mat = Eigen::Matrix<double, 6, 6>::Identity() - 0.5*coeff;
    
        return mat;
    }

private:
    static inline void ensure(bool cond, const char* msg)
    {
        if (!cond)
        {
            throw std::runtime_error(std::string("[Error]") + msg);
        }
    }

};
#endif