// include the ceres
#include "ceres/ceres.h"

// include data reader header file
#include "txtio.hpp"

// include the other header file 
#include <iostream>
#include <chrono>

// define the costfunctor
struct CostFunctor
{
    CostFunctor(double x, double y): x_(x), y_(y) {}
    
    // define the operator()
    // 多引數版本的operator
    template<typename T>
    bool operator()(const T* const x1, const T* const x2, T * residual) const  
    {
        // calculate the residual
        // 這邊也可以透過利用Eigen::Map<Eigen::Matrix<dobule, 3, 1, Eigen::RowMajor>>之類的來簡化複雜的計算
        residual[0] = T(y_) - (x2[0] / (x1[0] + ceres::exp(-x1[1] * x_)));
        return true;
    }
    
    // define the data
    double x_;
    double y_;
};

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
        // 這邊模板都是一樣編排的
        // 第一個就是residual的維度,接下來的就是一號的狀態的維度,二號的狀態的維度
        // 對應到operator()裡面的引數順序
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 2, 1>
                                            (new CostFunctor(data(i, 0), data(i, 1)));
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

