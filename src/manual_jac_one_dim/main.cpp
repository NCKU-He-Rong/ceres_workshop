#include <iostream>
#include <vector>
#include <ceres/ceres.h>
#include <line_fit_cost.hpp>



int main(int argc, char** argv)
{
    std::vector<double> x_data = {1.0, 2.0, 3.0};
    std::vector<double> y_data = {5.0, 8.0, 11.0};

    double x[2] = {0.0, 0.0};

    ceres::Problem problem;

    for (int i = 0; i < x_data.size(); i++)
    {
        ceres::CostFunction* cost_function = new CustomJacLineFitCost(x_data[i], y_data[i]);

        problem.AddResidualBlock(cost_function, nullptr, x);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    std::cout << "Initial stata: " << "a: " << x[0] << " b: " << x[1] << std::endl;

    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;
    std::cout << "Final stata: " << "a: " << x[0] << " b: " << x[1] << std::endl;

    return 0;
}