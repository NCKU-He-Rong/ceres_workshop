// include the ceres
#include "ceres/ceres.h"

// include data reader header file
#include "txtio.hpp"

// include the other header file 
#include <iostream>
#include <chrono>

// include the self-defined cost
#include "include/costfunctor.hpp"



// define the main 
int main(int argc, char*argv[])
{
    // read the data
    TxtIO data("../data/curve_fitting_data.txt");
    data.read();

    // get the data len
    int dataLen = data.getRowInfo();

    // define the ceres problem 
    ceres::Problem problem;

    // define the state
    double state_ab[] = {2.0, 2.0};
    double state_r[] = {2.0};


    // add residual block 
    for (int i=0;i< dataLen;i++)
    {
        ceres::CostFunction* cost_function = new CostFunctor(data(i, 0), data(i, 1));
        problem.AddResidualBlock(cost_function, nullptr, state_ab, state_r);
    }

    // define the options
    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // define the summary
    ceres::Solver::Summary summary;

    // calculate the time spent
    auto start = std::chrono::steady_clock::now();
    
    // optimize
    ceres::Solve(options, &problem, &summary);

    // calculate the time spent
    auto end = std::chrono::steady_clock::now();
    auto dura = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // print the result
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "[Truth]: " << 3.0 << " " << 6.0 << " " << 3.0 << std::endl;
    std::cout << "[Result]: " << state_ab[0] << " " << state_ab[1] << " " << state_r[0] << std::endl;
    std::cout << "[Time spent]: " << dura / 1000.0  << " ms" << std::endl;

    return 0;

}

