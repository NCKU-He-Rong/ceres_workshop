#ifndef LINE_FIT_COST_HPP
#define LINE_FIT_COST_HPP

#include <ceres/ceres.h>

// if we want to custom the jacobian matrix, we need to inherit the ceres::SizedCostFunction
// the first 1 means the residual dimension
// the second 1 means the first parameter dimension
// the third 1 means the second parameter dimension
class CustomJacLineFitCost: public ceres::SizedCostFunction<1, 2>
{
public:
    CustomJacLineFitCost(double x, double y): x_(x), y_(y) {}

    virtual ~CustomJacLineFitCost() {}

    // the Evaluate function's arguments is fixed by ceres including the double type
    virtual bool Evaluate(double const* const* parameters,
                          double* residual,
                          double** jacobians) const override
    {
        // the first 0 means the first parameter block
        const double a = parameters[0][0];
        const double b = parameters[0][1];

        residual[0] = y_ - (a * x_ + b);

        if (jacobians != nullptr)
        {
            // the 0 in jacobians[0] means the first parameter block
            // the second 0 in jacobians[0][0] means the first element 
            // of the jacobian matrix about the first parameter 
            // in each jacobians[0], ceres will flatten the jacobian matrix 
            // into one dimension array with row-major order
            if (jacobians[0] != nullptr)
            {
                jacobians[0][0] = -x_;
                jacobians[0][1] = -1;
            }
        }

        return true;
    }

private:
    double x_;
    double y_;

};

#endif