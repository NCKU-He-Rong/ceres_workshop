#ifndef POSE_MAINFOLD_HPP_
#define POSE_MAINFOLD_HPP_


#include <ceres/ceres.h>
#include <Eigen/Dense>


// ceres 2.2版本的ceres中已經沒有舊版中處理manifold的ceres::LocalParameterization
// 改成用ceres::Manifold去進行處理,這邊都推薦用新版的去進行處理比較好
// 但有些舊專案還是都是用ceres::LocalParameterization,所以編譯的時後要注意一下(VINS-MONO)
class PoseManifold: public ceres::Manifold
{
public:
    // 這邊有幾個是一定要定義的funcion (const override)
    // AmbientSize指的就是全域的Pose維度,像是如果我們使用位置+四元數,那就是3+4,也是最佳化的變量維度
    // 這邊比較可惜,不能指定2維的DCM當作最佳化變量,最後外面傳給ceres problem的最佳化變量維度,也是要維持這個AmbientSize
    int AmbientSize() const override;

    // 相對於AmbientSize,TangentSize就是delta變量的維度,也就是最佳化過程中篇導Jacobian的column維度
    // 大部分就是位置+姿態李群 => 3+3等於6個
    int TangentSize() const override;

    // 定義一個delta(TangentSize)近來之後,如何去更新你原本的狀態x(AmbientSize),最後得到更新後的狀態x_plus_delta(AmbientSize)
    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override;

    // 因為ceres只會進行以下的求導:
    //
    //    ∂ f(x0 ⊞ delta_x)    |                            ∂ f(x)    |   
    // ------------------------|              equal =>   -------------|
    //     ∂ (x0 ⊞ delta_x)    | delta_x = 0                 ∂ (x)    | x = x0
    // 
    // 其維度大小為: residual維度 * AmbientSize,包含我們後續要在Evaluate中定義的維度一定都要是這個: residual維度 * AmbientSize
    // 但是我們需要的是以下的東西,這樣我們才可以對應到最佳化過程中的Jacobian
    // 
    //    ∂ f(x0 ⊞ delta_x)    |
    // ------------------------|
    //     ∂ (delta_x)         | delta_x = 0
    // 
    // 所以我們需要多定義一個Jacobian,也就是PlusJacobian來建立連鎖律,好讓ceres可以推導出上面的那個終極jacobian
    // 
    //    ∂ f(x0 ⊞ delta_x)    |                   ∂ f(x0 ⊞ delta_x)     |                    ∂ (x0 ⊞ delta_x)   |    
    // ------------------------|               = ------------------------|              *   ---------------------|
    //     ∂ (delta_x)         | delta_x = 0       ∂ (x0 ⊞ delta_x)      | delta_x = 0           ∂ (delta_x)     | delta_x = 0
    // 
    // 最後一項就是PlusJacobian,其維度為AmbientSize * TangentSize
    //
    //
    // 
    // 平常我們接觸的最佳化變量為：
    // 
    //    ∂ f(x0 + delta_x)    |                   ∂ f(x0 + delta_x)     |                    ∂ (x0 + delta_x)   |    
    // ------------------------|               = ------------------------|              *   ---------------------|
    //     ∂ (delta_x)         | delta_x = 0       ∂ (x0 + delta_x)      | delta_x = 0           ∂ (delta_x)     | delta_x = 0
    // 其等於：
    //    ∂ f(x0 + delta_x)    |                         ∂ f(x)          |                   
    // ------------------------|               = ------------------------|              *    I
    //     ∂ (delta_x)         | ddelta_xelta_x = 0              ∂ (x)          | x = x0
    // 最終等於:
    //    ∂ f(x0 + delta_x)    |                         ∂ f(x)          |                   
    // ------------------------|               = ------------------------|        
    //     ∂ (delta_x)         | delta_x = 0              ∂ (x)          | x = x0            
    bool PlusJacobian(const double* x, double* jacobian) const override;

    // 以下這兩個函數官方基本上也沒說明會如何用它(學理上),甚至在2023年說可以直接用dummy的方式去實做它
    // 並不會出現任何問題,可以參閱: https://github.com/ceres-solver/ceres-solver/issues/1003
    // 但是他說可能新版ceres之後,就會使用(但也沒具體說會如何使用), 可能就會引入問題
    // 但比較慶幸的是,我們後續因為一些trick,會讓這兩個函數,基本上是固定的型態(學理上也成立的那種)
    // 所以我們還是會實做, 但不構成問題
    // 定義一個更新後的狀態y(AmbientSize)與更新前的狀態x(AmbientSize),他們之間差距多少增量y_minus_x(TangentSize)
    // 也就是x ⊞ y_minus_x = y
    // 或者定義y ⊟ x = y_minus_x
    bool Minus(const double* y, const double* x, double* y_minus_x) const override;

    // 這邊定義:
    //
    //      ∂ (y ⊟ x)      |    
    // --------------------|
    //        ∂ (y)        | y = x 
    bool MinusJacobian(const double* x, double* jacobian) const override;
};
#endif