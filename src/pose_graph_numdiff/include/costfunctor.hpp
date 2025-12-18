#ifndef COSTFUNCTOR_HPP_
#define COSTFUNCTOR_HPP_

#include "ceres/ceres.h"
#include <Eigen/Dense>
#include "rotation.hpp"


struct CostFunctor
{
    CostFunctor(const Eigen::Vector3d &t, 
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
        // 這邊透過selfadjointView來進行上三角的對稱映射
        // 雖然只需要對角元素就好
        info_mat = info_mat.selfadjointView<Eigen::Upper>();
        // 這個操作基本上是針對對角元素才可以如此進行(殘差之間沒有關聯)
        // 不用擔心sqrt(0),它還是會是0
        sqrt_mat = info_mat.cwiseAbs().cwiseSqrt();
        
        q_mes.normalize();
    }

    // 注意這邊跟我們傳統的residual定義不一樣
    // 因為我們在pose_manifold中我們是分開處理姿態跟位置的,所以傳統包在一起的T矩陣殘差,需要更改一下
    // 同時一定不能定義q.inverse() * q當作residual(雖然他們是一維),因為對應residual最小不是(0, 0, 0, 0),而是(1, 0, 0, 0)
    // 因為後我們將用NumericDiffCostFunction,所以這邊我們不用template <typename T>
    // AutoDiffCostFunction太多限制了,體現在模板的要求上面
    bool operator()(const double* const xi, const double* const xj, double* residual) const
    {
        Eigen::Vector3d t_xi(xi[0], xi[1], xi[2]);
        Eigen::Quaterniond q_xi(xi[3], xi[4], xi[5], xi[6]); 
        // 這邊針對NumericDiffCostFunction很關鍵
        // 因為NumericDiffCostFunction會去計算r(x+delta_x) - r(x-delta_x)來進行差分求導
        // 但相當可惜的是他的加法直接是歐式加法,不管我們定義的pose_manidfold,所以這邊operator()的
        // 引數有機會傳入非單位四元數的狀態,下面來自官方的說明(https://raw.githubusercontent.com/ceres-solver/ceres-solver/master/docs/source/nnls_modeling.rst)
        /*
            若你的 cost function 依賴某個必須位於流形上的參數區塊，
            而該 functor 在此參數區塊不位於流形時無法被評估，
            則對這類 functor 進行數值微分可能會遇到問題
            這是因為 Ceres 的數值微分是透過擾動 cost functor 所依賴之參數區塊的各個座標分量來完成的
            此種擾動隱含假設該參數區塊是位於歐式流形（Euclidean manifold）上，
            而非該參數區塊實際對應的流形。因此，部分被擾動後的點可能不再位於該流形上,
            例如，考慮一個四維參數區塊，其被解讀為單位四元數。對此參數區塊的座標分量進行擾動，
            將會違反其單位范數（unit norm）的性質 要修正此問題，
            需使 :class:NumericDiffCostFunction 能夠知悉每個參數區塊所對應的 :class:Manifold，
            並且只在各參數區塊的局部切空間（local tangent space）中生成擾動,
            目前我們認為此問題尚不足以嚴重到需要更改 :class:NumericDiffCostFunction 的 API
            此外，在多數情況下，在 functor 使用該點之前，先將偏離流形的點投影回流形
            通常相對容易,以四元數為例，在使用前先將該 4 維向量正規化即可解決
        */
        q_xi.normalize();   

        Eigen::Vector3d t_xj(xj[0], xj[1], xj[2]);
        Eigen::Quaterniond q_xj(xj[3], xj[4], xj[5], xj[6]);
        // 這邊針對NumericDiffCostFunction很關鍵,理由同上
        q_xj.normalize();   

        Eigen::Map<Eigen::Vector3d> r_t(residual);
        Eigen::Map<Eigen::Vector3d> r_q(residual+3);

        // 位置誤差
        r_t = sqrt_mat.block<3, 3>(0, 0) * (q_mes * q_xj.inverse() * t_xi - q_mes * q_xj.inverse() * t_xj + t_mes);

        // 姿態誤差
        r_q = sqrt_mat.block<3, 3>(3, 3) * (Rotation::quaternion2rotvec((q_mes * q_xj.inverse() * q_xi).normalized()));

        return true;
    }


    Eigen::Vector3d t_mes;
    Eigen::Quaterniond q_mes;
    Eigen::Matrix<double, 6, 6> info_mat;
    Eigen::Matrix<double, 6, 6> sqrt_mat;
    
};






#endif