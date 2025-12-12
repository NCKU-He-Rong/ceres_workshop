#include <iostream>
#include <vector>
#include <ceres/ceres.h>
#include <mean_cost.hpp>



int main(int argc, char** argv)
{
    std::vector<double> x_data = {1.0, 2.0, 3.0};
    std::vector<double> y_data = {3.0, 4.0, 5.0};

    double x[2] = {0.0, 0.0};

    ceres::Problem problem;

    for (int i=0;i<x_data.size();i++)
    {
        ceres::CostFunction * cost_function = new MeanCostFunctor(x_data[i], y_data[i]);

        problem.AddResidualBlock(cost_function, nullptr, x);

    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    std::cout << "Initial State: " << "x[0]: " << x[0] << " x[1]: " << x[1] << std::endl;

    ceres::Solve(options, &problem, &summary);
    std::cout << "Final State: " << "x[0]: " << x[0] << " x[1]: " << x[1] << std::endl;

    return 0;
}