// include ceres
#include "ceres/ceres.h"

// include the data reader
#include "txtio.hpp"
#include <iostream>
#include <chrono>

// include the self-defined cost
#include "include/costfunctor.hpp"

// main function
int main(int argc, char*argv[])
{   
    // read data
    TxtIO data("../data/curve_fitting_data.txt");
    data.read();

    // get the data length 
    int dataLen = data.getRowInfo();

    // defien the state
    double state[] = {2.0, 2.0, 2.0};

    // create the optimization problem
    ceres::Problem problem;

    
    // add residual
    for (int i= 0;i<dataLen;i++)
    {   
        // 因為我們自己明定了jacobian,所以就不需要
        // 依賴AutoDiffCostFunction或者NumericDiffCostFunction
        // 來幫我們求取jacobian了
        // 所以直接進行new Costfunctor就好
        ceres::CostFunction* cost_function = new CostFunctor(data(i, 0), data(i, 1));
        problem.AddResidualBlock(cost_function, nullptr, state);
    }
    

    // option setting
    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // 
    ceres::Solver::Summary summary;

    // optimization and calculation time spent
    auto start = std::chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    auto end = std::chrono::steady_clock::now();
    auto dura = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

     // 印出結論
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "[Truth] " << 3.0 << " " << 6.0 << " " << 3.0 << std::endl;
    std::cout << "[Result] " << state[0] << " " << state[1] << " " << state[2] << std::endl;
    std::cout << "[Time Spent] " << dura / 1000.0 << "ms" << std::endl;

    // return
    return 0;
}