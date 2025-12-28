#include "pose_manifold.hpp"
#include "rotation.hpp"

int PoseManifold::AmbientSize() const {
	return 6;
}

int PoseManifold::TangentSize() const {
	return 6;
}

bool PoseManifold::Plus(const double* x, const double* delta, double* x_plus_delta) const {
	Eigen::Map<const Eigen::Matrix<double, 6, 1>> pose_x(x);
	Eigen::Map<const Eigen::Matrix<double, 6, 1>> pose_delta(delta);
	Eigen::Map<Eigen::Matrix<double, 6, 1>> pose_plus(x_plus_delta);

	pose_plus = Rotation::SE3Log(Rotation::SE3Exp(pose_x) * 
															 Rotation::SE3Exp(pose_delta));
	return true;
}

bool PoseManifold::PlusJacobian(const double* x, double* jacobian) const {
	Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>j(jacobian);
	j.setIdentity();
	return true;
}

bool PoseManifold::Minus(const double* y, const double* x, double* y_minus_x) const {
	Eigen::Map<const Eigen::Matrix<double, 6, 1>> pose_y(y);
	Eigen::Map<const Eigen::Matrix<double, 6, 1>> pose_x(x);
	Eigen::Map<Eigen::Matrix<double, 6, 1>> pose_minus(y_minus_x);

	pose_minus = Rotation::SE3Log(Rotation::SE3Exp(pose_x).inverse() * 
							    							Rotation::SE3Exp(pose_y));
	return true;
}

bool PoseManifold::MinusJacobian(const double* x, double* jacobian) const {
	Eigen::Map<Eigen::Matrix<double, 6, 6,  Eigen::RowMajor>> j(jacobian);
	j.setIdentity();
	return true;
}