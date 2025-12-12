#include <iostream>
#include <vector>
#include <ceres/ceres.h>


// we can use class instead of struct
// the ceres don't care about it
struct MeanCostFunctor
{
    // constructor
    MeanCostFunctor(double a): a_(a) {}

    // compute the residual
    // x: parameter
    // residual: output residual
    template <typename T>
    bool operator()(const T* const x, T* residual) const
    {
        residual[0] = x[0] - T(a_);
        return true; 
    }

    // measurement
    double a_;
};



// main function
int main(int argc, char** argv)
{
    // measurements data
    std::vector<double> data = {1.0, 2.0, 5.0, 7.0};

    // initial value of parameter x
    double x = 0.0;

    // define the problem
    ceres::Problem problem;

    // create the residual blocks and add them to the problem
    for (double a: data)
    {   
        // the first 1 means the residual dimension
        // the second 1 means the parameter dimension
        // with these two dimensions, the jacobian's dimension can be determined
        ceres::CostFunction * cost_function = new ceres::AutoDiffCostFunction<MeanCostFunctor, 1, 1>
                                             (new MeanCostFunctor(a));
        
        //the nullptr means we don't use loss function here(loss function means the robust kenel, e.g. Huber)
        problem.AddResidualBlock(cost_function, nullptr, &x);
    }

    // configure the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;   // print the each iteration result to stdout

    // create the summary to hold the optimization information
    ceres::Solver::Summary summary;
    std::cout << "Initial state: " << "x: " << x << std::endl;

    // solve the problem
    ceres::Solve(options, &problem, &summary);

    // print the report
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "Final state: " << "x: " << x << std::endl;

    // return
    return 0;
}