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
};
#endif