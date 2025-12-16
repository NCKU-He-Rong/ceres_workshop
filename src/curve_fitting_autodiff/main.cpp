// 這個就include基本上全部的ceres的header file
#include "ceres/ceres.h"   

// include the data reader
#include <txtio.hpp>

// include the other file
#include <iostream>
#include <chrono>


// define the residual struct
// 這邊也可以改成class,不一定要用struct
// 只不過要確保class中的成員"函數"包含operator()都要是public的,這樣ceres才可以呼叫到
// 其他measurement的變量一樣可以維持private
struct CostFunctor
{   
    // 透過建構子傳入measurement與不是優化變量的變數
    CostFunctor(const double x, const double y): x_(x), y_(y) {};

    // define the residual
    // x 就是目前的最佳化狀態
    // 這些引數是可以多新增的例如:
    // bool operator()(const T* const x1, const T* const x2, T * residual) const
    // 並不固定,但要對應到後面的problem.AddResidualBlock中塞入最佳化變數的數量跟對象
    // 這邊注意如果沒有要自己定義jacobian(繼承SizedCostFunction),就要用template
    template<typename T>
    bool operator()(const T* const x, T* residual) const
    {   
        //記得這邊不能用std::exp因為它不支援自動偏導(autodiff), 可以用ceres::<數學運算>,可以用ceres支援很多數學運算
        residual[0] = T(y_) - (x[2] / (x[0] + ceres::exp(-x[1]*x_))); 
        return true;
    }

private:
    const double x_;   // x const
    const double y_;   // measurement 

};

// main function
int main(int argc, char *argv[])
{
    // read the data
    TxtIO data("../data/curve_fitting_data.txt");
    data.read();

    // get the # data 
    int dataLen = data.getRowInfo();
    
    // define the state
    double state[] = {2.0, 2.0, 2.0};

    // define the optimization problem
    ceres::Problem problem;

    // add residual block
    for(int i=0;i<dataLen;i++)
    {   
        // 這邊的1跟3,分別為residual跟優化狀態的維度
        ceres::CostFunction* cost_fuction = new ceres::AutoDiffCostFunction<CostFunctor, 1, 3>
                                           (new CostFunctor(data(i,0), data(i,1)));
        // 在proble中加入residual
        // 這邊的nullptr原本是要填入loss function(ceres::HuberLoss, ceres::CauchyLoss...),
        // 可以填入huber之類的,如果是nullptr,cost就單純是這個residual的平方
        problem.AddResidualBlock(cost_fuction, nullptr, state); 
    }
    
    // 設定最佳化的參數設定
    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;   // 等同於g2o中的verbose

    // 總結報告
    ceres::Solver::Summary summary;

    // 計時
    auto start = std::chrono::steady_clock::now();

    // 最佳化
    ceres::Solve(options, &problem, &summary);

    // 計時
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