#ifndef MEAN_COST_HPP
#define MEAN_COST_HPP
#include <ceres/ceres.h>



class MeanCostFunctor: public ceres::SizedCostFunction<2, 2>
{
public:
    MeanCostFunctor(double a, double b) : a_(a), b_(b) {}
    virtual ~MeanCostFunctor() {}

    virtual bool Evaluate(double const* const* parameters,
                          double * residual,
                          double ** jacobians) const override
    {
        const double x1 = parameters[0][0];
        const double x2 = parameters[0][1];

        residual[0] = a_ - x1;
        residual[1] = b_ - x2;

        if (jacobians != nullptr)
        {
            if (jacobians[0] != nullptr)
            {
                jacobians[0][0] = -1;
                jacobians[0][1] = 0;
                jacobians[0][2] = 0;
                jacobians[0][3] = -1;
            }
        }

        return true;
    }

private:
    double a_;
    double b_;
};
#endif