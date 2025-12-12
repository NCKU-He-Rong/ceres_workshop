#include <iostream>
#include <vector>
#include <ceres/ceres.h>


struct LineFittingCostFunctor
{
    LineFittingCostFunctor(double x, double y): x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T* const a, const T* const b, T* residual) const
    {
        residual[0] = T(y_) - (a[0] * x_ + b[0]);

        return true;
    }
    
    double x_;
    double y_;
};



int main(int argc, char ** argv)
{
    std::vector<double> x_data = {1.0, 2.0, 3.0};
    std::vector<double> y_data = {5.0, 8.0, 11.0};

    double a = 0.0;   // 3
    double b = 0.0;   // 2

    ceres::Problem problem;

    for (int i = 0; i<x_data.size();i++)
    {
        ceres::CostFunction * cost_function = new ceres::AutoDiffCostFunction<LineFittingCostFunctor, 1, 1, 1>
        (new LineFittingCostFunctor(x_data[i], y_data[i]));

        problem.AddResidualBlock(cost_function, nullptr, &a, &b);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    std::cout << "Initial stata: " << "a: " << a << " b: " << b << std::endl;

    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;
    std::cout << "Final stata: " << "a: " << a << " b: " << b << std::endl;

    return 0;
}
