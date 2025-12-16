#include <iostream>
#include <rotation.hpp>
#include <Eigen/Dense>



int main(int argc, char** argv)
{
    double theta = 0.00000000000000 * M_PI / 180.0;

    Eigen::Vector3d axis(2.0, 1.0, 3.0);
    axis.normalize();

    Eigen::Quaterniond q_1 = Rotation::rotvec2quaternion(theta * axis);
    Eigen::Quaterniond q_2(Eigen::AngleAxisd(theta, axis));


    std::cout << q_1.coeffs().transpose() << std::endl;
    std::cout << q_2.coeffs().transpose() << std::endl;


    Eigen::Vector3d angleaxis_1 = Rotation::quaternion2rotvec(q_2);
    Eigen::Vector3d angleaxis_2 = Eigen::AngleAxisd(q_2).angle() * Eigen::AngleAxisd(q_2).axis();
    std::cout << (theta * axis).transpose() << std::endl;
    std::cout << angleaxis_1.transpose() << std::endl;
    std::cout << angleaxis_2.transpose() << std::endl;


    double data[] = {1.0, 0.0, 0.0, 0.0};
    Eigen::Map<Eigen::Quaterniond> q_3(data);
    Eigen::Quaterniond q_4(data[0], data[1], data[2], data[3]);
    std::cout << q_3.w() << " " << q_3.x() << " " << q_3.y() << " " << q_3.z() << std::endl;
    std::cout << q_4.w() << " " << q_4.x() << " " << q_4.y() << " " << q_4.z() << std::endl;


    return 0;
}