// include the ceres header file
#include "ceres/ceres.h"

// include the other header file
#include <iostream>
#include <chrono>
#include <fstream>

#include "include/pose_manifold.hpp"
#include "include/costfunctor.hpp"


// main fucntion
int main(int argc, char*argv[])
{   
    // define the data file 
    std::string fileName;
    if (argc == 2) 
    { 
        fileName = argv[1];
    }
    else 
    { 
        fileName = "../data/sphere.g2o";
    }   
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
    fin.clear();                  // 清除 EOF 狀態
    fin.seekg(0, std::ios::beg);  // 將檔案指標移回檔案開頭

    // create the state array
    double state[vertexLen][7];

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
            q.normalize();

            ceres::Manifold * manifold = new PoseManifold();

            state[index][0] = t(0);
            state[index][1] = t(1);
            state[index][2] = t(2);
            state[index][3] = q.w();
            state[index][4] = q.x();
            state[index][5] = q.y();
            state[index][6] = q.z();

            problem.AddParameterBlock(state[index], 7, manifold);

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
            q.normalize();
            
            // get the information matrix
            Eigen::Matrix<double, 21, 1> info_vec;
            for (int i=0 ; i<21 ; i++)
            {
                fin >> info_vec(i);
            }

            ceres::CostFunction* cost_function = new ceres::NumericDiffCostFunction<CostFunctor, ceres::CENTRAL, 6, 7, 7>
                                                (new CostFunctor(t, q, info_vec));
            
            problem.AddResidualBlock(cost_function, nullptr, state[index_i], state[index_j]);
        }
    }

    // optimization setting
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    // 稀疏大範圍就用ceres::SPARSE_SCHUR或者ceres::SPARSE_NORMAL_CHOLESKY;
    // 這邊如果用Dense的話就會非常慢,如ceres::DENSE_QR
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;  
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 4;

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
        fout << state[i][0] << " " 
             << state[i][1] << " " 
             << state[i][2] << std::endl;
    }

    //return 
    return 0;
}