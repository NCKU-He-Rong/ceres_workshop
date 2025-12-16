#ifndef COSTFUNCTOR_HPP_
#define COSTFUNCTOR_HPP_

#include "ceres/ceres.h"
#include <Eigen/Dense>

// define the costfunctor
class CostFunctor: public ceres::SizedCostFunction<1, 2, 1>
{
public:
    CostFunctor(double x, double y): x_(x), y_(y) {}

    // define the evaluate
    // 注意這邊的引數的打法就是固定的了,不能自己多新增東西,因為是overide的
    // 不像oeprator()可以多新增東西
    bool Evaluate(double const * const * parameter, 
                          double * residual, 
                          double ** jacobians) const override
    {
        // get the current estimated
        // parameter[0]就是指到第1個參數(2元素)
        // parameter[1]就是指到第2個參數(1元素)
        double a = parameter[0][0];
        double b = parameter[0][1];
        double r = parameter[1][0];

        // calculate the residual
        // 雖然這邊可以寫成residual[0] = y_- (r / (a + std::exp(-b*x_)));
        // 但這邊我一樣透過Eigen Map的方式去處理,保留未來可能叫複雜的情況
        Eigen::Map<Eigen::Matrix<double, 1, 1>> r_(residual);
        r_(0) = y_ - (r / (a + std::exp(-b * x_)));

        // calcualte the jacobian 
        if (jacobians)
        {
            // 對應到對第1個的優化變量的微分jacobian
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 2, Eigen::RowMajor>> j_xi(jacobians[0]);
                j_xi(0) = r / std::pow(a + std::exp(-b * x_), 2);
                j_xi(1) = -(r * x_ * std::exp(-b * x_)) / std::pow(a + std::exp(-b*x_), 2);
            }

            // 對應到對第2個的優化變量的微分jacobian
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1>> j_xj(jacobians[1]);
                j_xj(0) = -1 / (a + std::exp(-b * x_));
            }
        }

        return true;
    }

private:
    double x_;
    double y_;

};

#endif