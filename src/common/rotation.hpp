#ifndef ROTATION_HPP_
#define ROTATION_HPP_

#include <Eigen/Dense>


class Rotation
{
public:
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

    // TODO: 要確保abs(theta)是小於pi的
    // TODO: 找看看比較不會有theta在下面的情況
    static Eigen::Matrix3d so3_jr(const Eigen::Vector3d &vec)
    {
        Eigen::Matrix3d mat;

        double theta = vec.norm();
        Eigen::Vector3d axis = vec.normalized();

        if (theta < 1e-12)
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
        Eigen::Vector3d axis = vec.normalized();

        if (theta < 1e-12)
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
        Eigen::Vector3d axis = vec.normalized();

        if (theta < 1e-12)
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
        Eigen::Vector3d axis = vec.normalized();

        if (theta < 1e-12)
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



};
#endif