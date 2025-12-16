// include the ceres header file
#include "ceres/ceres.h"

// include the other header file
#include <iostream>
#include <chrono>
#include <fstream>

// include the sophus header file
#include "sophus/se3.hpp"

#include <thread>


// define the local parameter for the Pose
// 這邊把握一個原則,就是我們會需要修改的,引數不會是const
// 如果要修改的就會透過Eigen::Map<沒有const..>產生連結
// 不修改的就會透過Eigen::Map<const...>建立有連結但不能修改的
class PoseLocalParameterization: public ceres::LocalParameterization
{
    // 這些重載包含引數的打法都是固定的,不能隨意變更
    // 只要前面有加上virtual就是都不能改
    virtual bool Plus(const double * x, 
                      const double * delta, 
                      double * x_plus_delta) const 
    {   
        // get the current pose in eigen format
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> pose(x);

        // get the delta pose in the eigen format
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> pose_delta(delta);

        // get the updated pose in eigen format
        Eigen::Map<Eigen::Matrix<double, 6, 1>> pose_update(x_plus_delta);

        // define the update
        pose_update = (Sophus::SE3d::exp(pose) * Sophus::SE3d::exp(pose_delta)).log();

        return true;

    }
    // 這些重載包含引數的打法都是固定的,不能隨意變更
    // 這邊這個ComputeJacobian含意到底是啥呢
    // 我們可以過連鎖律來解釋
    // 
    //    ∂ f(x (*) delta_x)             ∂ (x * delta_x )            ∂ f(x (*) delta_x)
    // ------------------------   *  ------------------------  =  -------------------------
    //     ∂ (x * delta_x )                ∂ (delta_x )                 ∂ (delta_x )
    //
    // 其中第二項就是LocalParameterization這邊的ComputeJacobian所要定義的
    // 而這邊我們直接定義為單位矩陣是因為
    // 我們在後面的的Residuald Block中直接去定義了結果也就是最右邊的
    // 那邊理論上右放上最左邊的那項的定義,也因此我們只能將第二項變成單位矩陣
    // 反之ceres只看最右邊的結果去計算

    // 同時我們會用到LocalParameterization,基本上都是跟流型相關
    // 因為太常用了,所以ceres自己也有定義一個同時我們會用到LocalParameterization是跟四元數相關
    // 叫EigenQuaternionManifold(新版),QuaternionParameterization(舊版)
    // 可以參考https://github.com/ceres-solver/ceres-solver/blob/c29b5257e23f91d6a47c4db9d57350ed4985ea46/internal/ceres/manifold.cc#L62
    // 裡面它定義的QuaternionPlusImpl與QuaternionPlusJacobianImpl
    // 與對應的推導https://www.cnblogs.com/vivian187/p/16502590.html
    // 會發現他的四元數擾動定義跟我們的不一樣
    // 他們是[1 theta*n],所以他們更新的時候也就對應用[1 theta*n]去更新 (sin(theta)接近theta)
    // 但比較符合物理的是[1 (theta/2)*n], 更新就用[1 (theta/2)*n]去更新 (sin(theta/2)接近theta/2) -> VINS的作法,他們不用ceres中現成的
    // 但這沒關係,用啥擾動推導,就用啥擾動更新,結果都是對的

    // 只要前面有加上virtual就是都不能改
    // 這邊會等於Identity的關係
    virtual bool ComputeJacobian(const double *x, double * jacabian) const 
    {
        // 這邊的RowMajor,對於ceres中的jacabian非常關鍵
        // 請看下面的Evaluate()中的Eigen::RowMajor的說明
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> j(jacabian);
        j.topRows<6>().setIdentity();

        // return
        return true;
    }

    virtual int GlobalSize() const
    {
        return 6;   // 這邊表示原本的姿態表示: 3(李群)  + 這邊位置表示: 3    -   3+3 = 6
    }

    virtual int LocalSize() const
    {
        return 6;   // 這邊表示姿態表示: 3(李群)  + 這邊位置表示: 3    -   3+3 = 7
    }
};


// define the Factor Edge
class PoseGraphFactor: public ceres::SizedCostFunction<6, 6, 6>
{
public: 
    PoseGraphFactor(const double * x, const double * information) 
    {
        // 透過這種方式去成立mes,比較省時間
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> mes(x);
        mes_ = Sophus::SE3d::exp(mes);

        // information matrix
        Eigen::Matrix<double, 6, 6> InformationMatrix = Eigen::Matrix<double, 6, 6>::Zero();
        informationMatrix(0, 0) = information[0];
        informationMatrix(0, 1) = information[1];  informationMatrix(1, 0) = informationMatrix(0, 1);
        informationMatrix(0, 2) = information[2];  informationMatrix(2, 0) = informationMatrix(0, 2);
        informationMatrix(0, 3) = information[3];  informationMatrix(3, 0) = informationMatrix(0, 3);
        informationMatrix(0, 4) = information[4];  informationMatrix(4, 0) = informationMatrix(0, 4);
        informationMatrix(0, 5) = information[5];  informationMatrix(5, 0) = informationMatrix(0, 5);

        informationMatrix(1, 1) = information[6];
        informationMatrix(1, 2) = information[7];  informationMatrix(2, 1) = informationMatrix(1, 2);
        informationMatrix(1, 3) = information[8];  informationMatrix(3, 1) = informationMatrix(1, 3);
        informationMatrix(1, 4) = information[9];  informationMatrix(4, 1) = informationMatrix(1, 4);
        informationMatrix(1, 5) = information[10]; informationMatrix(5, 1) = informationMatrix(1, 5);

        informationMatrix(2, 2) = information[11]; 
        informationMatrix(2, 3) = information[12]; informationMatrix(3, 2) = informationMatrix(2, 3);
        informationMatrix(2, 4) = information[13]; informationMatrix(4, 2) = informationMatrix(2, 4);
        informationMatrix(2, 5) = information[14]; informationMatrix(5, 2) = informationMatrix(2, 5);

        informationMatrix(3, 3) = information[15];
        informationMatrix(3, 4) = information[16]; informationMatrix(4, 3) = informationMatrix(3, 4);
        informationMatrix(3, 5) = information[17]; informationMatrix(5, 3) = informationMatrix(3, 5);

        informationMatrix(4, 4) = information[18];
        informationMatrix(4, 5) = information[19]; informationMatrix(5, 4) = informationMatrix(4, 5);

        informationMatrix(5, 5) = information[20]; 

        // get the square weight 
        // 這邊會多新增cwiseAbs(),因為parking-garage.g2o資料集權重有負的,所以Sqrt會出事(產出nan)
        sqrt_informationMatrix = informationMatrix.cwiseAbs().cwiseSqrt();

    }

    virtual bool Evaluate(double const * const * parameter,
                          double * residual,
                          double ** jacobians) const 
    {
        // get the current estimated
        // 這邊補充如果透過Eigen::Matrix<double, 6, 1> xi(parameter[0]);去成立eigen matrix
        // 透過這種方式從array去成立eigen就可以避開連結關係了
        // 但我們在這邊比較少用,我們用map const就好,直接去避開修改到他的數值
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> xi(parameter[0]);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> xj(parameter[1]);
        Eigen::Map<Eigen::Matrix<double, 6, 1>> r(residual);

        // transform the pose to lie groups format
        Sophus::SE3d pose_i = Sophus::SE3d::exp(xi);
        Sophus::SE3d pose_j = Sophus::SE3d::exp(xj);  

        // calculate the error and weighting residual
        Eigen::Matrix<double, 6, 1> error = (mes_.inverse() * pose_i.inverse() * pose_j).log();
        r = sqrt_informationMatrix * error;

        // calcualte the jacobian
        if (jacobians)
        {
            // create the jr inverse matrix
            Eigen::Matrix<double, 6, 6> Jrinv = Eigen::Matrix<double, 6, 6>::Identity();
            Jrinv.block(0, 0, 3, 3) += 0.5*Sophus::SO3d::hat(error.tail(3));
            Jrinv.block(0, 3, 3, 3) += 0.5*Sophus::SO3d::hat(error.head(3));
            Jrinv.block(3, 3, 3, 3) += 0.5*Sophus::SO3d::hat(error.tail(3));
            
            // 對x_i的偏導
            if (jacobians[0])
            {
                // 這邊的Eigen::RowMajor是一個非常重要的技巧
                // 參考: https://blog.csdn.net/juluwangriyue/article/details/122274335
                // jacobian為一個array型態,其編排方式為:
                // jacobians[0]就是殘差對第一個優化變量的所有偏導數(一定是一維陣列),方式為
                // 假設殘差維度為6,優化變量也是6,那jacobians[0]就是
                // [[x,x,x,x,x,x],[x,x,x,x,x,x],[x,x,x,x,x,x],[x,x,x,x,x,x],[x,x,x,x,x,x],[x,x,x,x,x,x]]  ->這個是一維陣列,只是為了方便理解而引入兩個[]
                // 其中第一個[x,x,x,x,x,x],就是第一個殘差元素,對優化變量的偏導(共六個)
                // 其中第二個[x,x,x,x,x,x],就是第二個殘差元素,對優化變量的偏導(共六個)
                // 以此類推
                // 所以我們可以知道jacobians[0][7]就是
                // 殘差對第一個優化變量的所有偏導數,就是第二個殘差元素對優化變量第一個元素的偏導
                // ceres會把對同一個優化變量的偏導數,展平變成一維,不管你的jacobian維度是多少
                // 但我們知道我們自己推導的jaconbian通常都是用矩陣方式去表達會比較好
                // 所以我們就透過Map方式將Eigen Matrix與這個array進行連結
                // 但是我們必須要透過Eigen::RowMajor去轉換
                // 因為Eigen主要存放數據的方式是透過ColMajor,可以參考: https://blog.csdn.net/juluwangriyue/article/details/122274335
                // 也就是透過Eigen::Matrix<...>.data[i]中依序下去索引
                // 對應到會是ColWise的順序
                // 所以我們要建立連結,就是要透過Eigen::RowMajor的方式,建立連結
                // 這樣記憶體的編排位置與連結方式,才會對應到我們熟知的Jacobian的樣子
                // 這樣之後我們就可以直接對Eigen進行直覺操作
                // 所以之後統一就是透過Eigen::Map<...Eigen::RowMajor>> 的ㄈ上去映射jacobians
                // 但要注意有關於一維jacobian的問題
                // 請參閱ch3
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> j_xi(jacobians[0]);
                j_xi = - sqrt_informationMatrix * Jrinv * (pose_j.inverse() * pose_i).Adj();
            }

            // 對x_j的偏導
            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> j_xj(jacobians[1]);
                j_xj = sqrt_informationMatrix * Jrinv;
            }
        }

        // return
        return true;
    }

private:
    Sophus::SE3d mes_;
    Eigen::Matrix<double, 6, 6> informationMatrix;
    Eigen::Matrix<double, 6, 6> sqrt_informationMatrix;
};



// main fucntion
int main(int argc, char*argv[])
{   
    // define the data file 
    std::string fileName;
    if (argc == 2) { fileName = argv[1];}
    else { fileName = "../data/sphere.g2o";}   
    std::ifstream fin(fileName);

    // get the vetex number and get the edge number
    int vertexLen = 0;
    int edgeLen = 0;
    while(!fin.eof())
    {
        std::string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") { vertexLen++;}
        if (name == "EDGE_SE3:QUAT") { edgeLen++; }
    }
    std::cout << "The Vertex Number is: " << vertexLen << std::endl;
    std::cout << "The Edge Number is: " << edgeLen << std::endl;

    // open the file again
    fin.clear(); // 清除 EOF 狀態
    fin.seekg(0, std::ios::beg); // 將檔案指標移回檔案開頭

    // create the state array
    double state[vertexLen][6];

    // create the problem
    ceres::Problem problem;

    // add residual block and local parameterization
    while(!fin.eof())
    {
        // get the name
        std::string name;
        fin >> name;
        
        // add vertex
        if (name == "VERTEX_SE3:QUAT")
        {
            // define the vertex index
            int index; fin >> index;

            Eigen::Vector3d t;
            fin >> t(0);
            fin >> t(1);
            fin >> t(2);

            Eigen::Quaterniond q;
            fin >> q.x();
            fin >> q.y();
            fin >> q.z();
            fin >> q.w();

            // create the estimate 
            Sophus::SE3d pose(q, t);

            // create the state
            for (int i=0;i<6;i++)
            {
                state[index][i] = pose.log()[i];
            }

            // create the local parameterization
            ceres::LocalParameterization * local_parameterization = 
                new PoseLocalParameterization();

            problem.AddParameterBlock(state[index], 6, local_parameterization);

            // fix the first vertex
            if (index == 0)
            {   
                // 透過SetParameterBlockConstant來fix相關參數
                problem.SetParameterBlockConstant(state[index]);
            }
        }

        // add edge
        if (name == "EDGE_SE3:QUAT")
        {
            // get the edge i nad j index
            int index_i, index_j;
            fin >> index_i;
            fin >> index_j;

            Eigen::Vector3d t;
            fin >> t(0);
            fin >> t(1);
            fin >> t(2);
            Eigen::Quaterniond q;
            fin >> q.x();
            fin >> q.y();
            fin >> q.z();
            fin >> q.w();

            // create the estimate 
            Eigen::Matrix<double, 6, 1> mes = Sophus::SE3d(q, t).log();

            // get the information matrix
            double InformationMatrix[21];
            for (int i=0 ; i<21 ; i++)
            {
                fin >> InformationMatrix[i];
            }

            // create the residual block
            problem.AddResidualBlock( 
                new PoseGraphFactor(mes.data(), InformationMatrix),    // .data()是將Eigen變成指針array型態
                nullptr, state[index_i], state[index_j]
                );
        }
        
    }

    // optimization setting
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    // 稀疏大範圍就用ceres::SPARSE_SCHUR或者ceres::SPARSE_NORMAL_CHOLESKY;
    // 這邊如果用Dense的話就會非常慢,如ceres::DENSE_QR
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;  
    options.minimizer_progress_to_stdout = true;

    // optimize the problem
    ceres::Solver::Summary summary;
    auto start = std::chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    auto end = std::chrono::steady_clock::now();
    auto dura = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // print the resul
    std::cout << summary.FullReport() << std::endl;   // 也可以用summary.BriefReport()
    std::cout << "[Time Spent] " << dura / 1000.0 << " ms" << std::endl;

    // save the result 
    std::ofstream fout("result.txt");
    for (int i=0;i<vertexLen;i++)
    {
        Sophus::SE3d pose = Sophus::SE3d::exp(Eigen::Matrix<double, 6, 1>(state[i]));
        fout << pose.translation()(0) << " " 
             << pose.translation()(1)<< " " 
             << pose.translation()(2) << std::endl;
    }

    //return 
    return 0;
}