#include <utils/principal_plane.h>

using namespace Eigen;

PrincipalPlaneAccumulator::PrincipalPlaneAccumulator() {
	clear();
}

void PrincipalPlaneAccumulator::clear() {
	count_ = 0;
	sum_ = Eigen::Vector3d(0, 0, 0);
	xx_ = 0;
	xy_ = 0;
	xz_ = 0;
	yy_ = 0;
	yz_ = 0;
	zz_ = 0;
}

void PrincipalPlaneAccumulator::addPoint(Eigen::Vector4f point) {

	Eigen::Vector3d p = Eigen::Vector3d(point[0], point[1], point[2]);
	sum_ += p;
	xx_ += p[0] * p[0];
	xy_ += p[0] * p[1];
	xz_ += p[0] * p[2];
	yy_ += p[1] * p[1];
	yz_ += p[1] * p[2];
	zz_ += p[2] * p[2];
	count_++;
}

Eigen::Vector4f PrincipalPlaneAccumulator::plane() {
	Matrix3d mat;
	mat << xx_, xy_, xz_,
	       xy_, yy_, yz_,
	       xz_, yz_, zz_;

	Vector3d plane = mat.ldlt().solve(sum_);

	return Vector4f(plane[0], plane[1], plane[2], 0);
}

Vector4f PrincipalPlaneAccumulator::centerPoint() {
	Vector3d c = sum_ * (1.0 / double(count_));
	return Vector4f(c[0], c[1], c[2], 0.0f);
}

