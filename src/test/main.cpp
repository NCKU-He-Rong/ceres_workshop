#include <iostream>
#include "rotation.hpp"
#include <Eigen/Dense>



int main(int argc, char** argv)
{   
    // double angle = std::stod(argv[1]);
    // double theta = angle * M_PI / 180.0;

    // Eigen::Vector3d axis(2.0, -1.0, 3.0);
    // axis.normalize();

    // Eigen::Quaterniond q_1 = Rotation::RotvecToQuaternion(theta * axis);
    // Eigen::Quaterniond q_2(Eigen::AngleAxisd(theta, axis));


    // std::cout << q_1.coeffs().transpose() << std::endl;
    // std::cout << q_2.coeffs().transpose() << std::endl;
    // std::cout << Eigen::AngleAxisd(theta, axis).angle() << std::endl;


    // Eigen::Vector3d angleaxis_1 = Rotation::QuaternionToRotvec(q_2);
    // Eigen::Vector3d angleaxis_2 = Eigen::AngleAxisd(q_2).angle() * Eigen::AngleAxisd(q_2).axis();
    // std::cout << (theta * axis).transpose() << std::endl;
    // std::cout << angleaxis_1.transpose() << std::endl;
    // std::cout << angleaxis_2.transpose() << std::endl;


    // double data[] = {1.0, 0.0, 0.0, 0.0};
    // Eigen::Map<Eigen::Quaterniond> q_3(data);
    // Eigen::Quaterniond q_4(data[0], data[1], data[2], data[3]);
    // std::cout << q_3.w() << " " << q_3.x() << " " << q_3.y() << " " << q_3.z() << std::endl;
    // std::cout << q_4.w() << " " << q_4.x() << " " << q_4.y() << " " << q_4.z() << std::endl;

    // // 四元數的q跟-q涵義是一樣的,旋轉矩陣也是
    // std::cout << q_1.coeffs().transpose() << std::endl;
    // std::cout << q_1.toRotationMatrix() << std::endl;
    // q_1.coeffs() *= -1;
    // std::cout << q_1.coeffs().transpose() << std::endl;
    // std::cout << q_1.toRotationMatrix() << std::endl;


    // // 
    // Eigen::Matrix3d A;
    // A.setZero();
    // A(0, 0) = 2.0; A(1, 1) = 2.0;
    // std::cout << A.cwiseAbs().cwiseSqrt() << std::endl;

    // Eigen::Vector3d vec(1.0, 2.0, 3.0);
    // std::cout << Rotation::SO3Hat(vec) << std::endl;

    // std::cout << 1 / tan(M_PI / 2.0) << std::endl;


    // // 這邊會觸發assert,不論release or debug mode(因為我加入了#undef NDEBUG in rotation.hpp)
    // // std::cout << Rotation::so3_jl_inv(theta*axis) << std::endl;
    // // std::cout << Rotation::so3_jl_inv(Rotation::quaternion2rotvec(q_1)) << std::endl;


    // Eigen::Matrix<double, 6, 7> j_1;
    // j_1.setZero();
    // j_1.topRows<6>().setIdentity();

    // Eigen::Matrix<double, 6, 7> j_2;
    // j_2.setZero();
    // j_2.rightCols<6>().setIdentity();

    // std::cout << j_1 << std::endl;
    // std::cout << std::endl;
    // std::cout << j_2 << std::endl;

    // Eigen::Matrix<double, 6, 1> se3;
    // se3.setRandom();
    // std::cout << se3 << std::endl;
    // std::cout << Rotation::SE3Hat(se3) << std::endl;
    // std::cout << Rotation::SE3Vee(Rotation::SE3Hat(se3)) << std::endl;

    // Eigen::Matrix3d rot_mat = Rotation::SO3Exp(theta * axis);
    // Eigen::Vector3d rot_lie = Rotation::SO3Log(rot_mat);
    // std::cout << axis << std::endl;
    // std::cout << (theta * axis).transpose() << std::endl;
    // std::cout << (rot_lie).transpose() << std::endl;

    // std::cout << "---" << std::endl;
    // Eigen::Matrix4d T_mat;
    // T_mat.setZero();T_mat(3, 3) = 1;
    // T_mat.block<3, 1>(0, 3) = Eigen::Vector3d::Random();
    // T_mat.block<3, 3>(0, 0) = rot_mat;
    // Eigen::Matrix<double, 6, 1> T_lie = Rotation::SE3Log(T_mat);
    // Eigen::Matrix4d T_mat_recover = Rotation::SE3Exp(T_lie);
    // std::cout << T_mat << std::endl;
    // std::cout << T_mat_recover << std::endl;


    // std::cout << q_1.toRotationMatrix() << std::endl;
    // std::cout << Eigen::AngleAxisd(q_1).angle() << std::endl;
    // q_1.coeffs() *= -1;
    // std::cout << Eigen::AngleAxisd(q_1).angle() << std::endl;

    // double theta = 5 * M_PI;
    // Eigen::Vector3d axis = Eigen::Vector3d::Random();
    // axis.normalize();
    // Eigen::Vector3d axis_1 = Rotation::SO3Wrapping(theta * axis);
    
    // std::cout << axis_1 << std::endl;
    // std::cout << (theta * axis).transpose() << std::endl;
    // std::cout << axis_1.norm() << std::endl;

    Eigen::Matrix3d rotation_matrix = Rotation::RotXDeg(180) * Rotation::RotZDeg(0);
    Eigen::Vector3d translation(1.0, 2.0, 3.0);
    Eigen::Matrix4d transformation_matrix;
    transformation_matrix.setZero();
    transformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
    transformation_matrix.block<3, 1>(0, 3) = translation;
    transformation_matrix(3, 3) = 1.0;

    std::cout << transformation_matrix << std::endl;
    Eigen::Matrix<double, 6, 1> lie = Rotation::SE3Log(transformation_matrix);
    std::cout << lie << std::endl;

    double theta_1 = 2.5 * M_PI;
    double theta_2 = 0.5 * M_PI;
    Eigen::Vector3d axis = Eigen::Vector3d::Random();
    axis.normalize();

    std::cout << Rotation::SO3LeftJacobian(theta_1 * axis) << std::endl;
    std::cout << Rotation::SO3LeftJacobian(theta_2 * axis) << std::endl;


    return 0;
}