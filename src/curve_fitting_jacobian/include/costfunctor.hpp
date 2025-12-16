#ifndef COSTFUNCTOR_HPP_
#define COSTFUNCTOR_HPP_

#include "ceres/ceres.h"
#include <Eigen/Dense>

// 定義residual
// 這邊的1就是residual維度
// 這邊的3就是state的維度
// 基本上跟AutoDiffCostFunction後面的模板一樣
// ceres中用到eigen一律使用Map比較好,可以建議連結關係
class CostFunctor: public ceres::SizedCostFunction<1, 3>
{
public:
    CostFunctor(double x, double y): x_(x), y_(y) {}

    // ceres的函數都差不多,都是bool開頭,const or const override結尾
    // 注意這邊的引數的打法就是固定的了,不能自己多新增東西,因為是overide的
    // 不像oeprator()可以多新增東西
    bool Evaluate(double const * const * parameter, 
                          double * residual, 
                          double ** jacobians) const override
    {   
        // get the current estimation
        // 這邊第一個[0]代表的是第一個parameter
        // 但這邊我們只有一個parameter(因為我們把它都集中在一起了),所以都是[0]
        // 這邊第二個為每一個parameter的元素index
        const double a = parameter[0][0];
        const double b = parameter[0][1];
        const double r = parameter[0][2];
        
        // compute the residual
        Eigen::Map<Eigen::Matrix<double, 1, 1>> cost(residual);
        // 雖然這邊可以寫成residual[0] = y_- (r / (a + std::exp(-b*x_)));
        // 但這邊我一樣透過Eigen Map的方式去處理,保留未來可能叫複雜的情況
        cost(0) = y_- (r / (a + std::exp(-b*x_)));

        // compute the jacobian
        if (jacobians)
        {   
            // 這邊的[0]就是針對第一個parameter的jacobian
            // 同時對任何parameter的jacobians,都是用一維陣列以RowMajor的方式存放
            // 也就是以列的方式去將學理中的jacobians,鋪成一個一維陣列(第二列移到第一列的最右邊,以此類推)
            if (jacobians[0])
            {   
                // 但老實說,如果jacobian為vector,Eigen::RowMajor其實沒啥用
                // 所以以下也可以用
                // Eigen::Map<Eigen::Matrix<double, 3, 1>> j(jacobians[0]);
                // 這邊補充一下, 在eigen裡面的col vector,也就是維度=n*1的不能指定他是RowMajor
                // 同理row vector,也就是維度=1*n的不能指定他是ColMajor,雖然涵義差不多,但是編譯過程會報錯
                // 所以以下不能寫: Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::RowMajor>>
                // 雖然他們看起來好像都是連接到相同的連續記憶體,但編譯階段就是不給你過
                // 但針對這個例子(一維jacobian),我們可以直接寫Eigen::Map<Eigen::Matrix<double, 3, 1>>就好
                // 它一樣會配置三個連續的記憶體空間給你
                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> j(jacobians[0]);

                // 請注意平舖的概念
                // 在其他例子中,假設有出現j(3),其意思就是第二個殘差元素對第一個parameter中的第一個元素篇導的結果
                j(0) = r / std::pow(a + std::exp(-b * x_), 2);
                j(1) = -(r * x_ * std::exp(-b * x_)) / std::pow(a + std::exp(-b*x_), 2);
                j(2) = -1 / (a + std::exp(-b * x_));
            }
        }

        // return
        return true;
    }
private:
    const double x_;
    const double y_;

};

#endif
