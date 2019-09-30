#ifndef FILE_PRINCIPAL_PLANE_H
#define FILE_PRINCIPAL_PLANE_H

#include <Eigen/Eigen>

/**
 * @brief The PrincipalPlaneAccumulator class
 * This class is used to calculate the principal plane for each patch.
 * Maybe at some point this class could become more than that and calculate and contain information
 * about curvature and stuff.
 */
class PrincipalPlaneAccumulator {
public:

	PrincipalPlaneAccumulator();
	
	void clear();

	void addPoint(Eigen::Vector4f point);

	//calc the plane we need to have
	Eigen::Vector4f plane();

	Eigen::Vector4f centerPoint();

private:

	int count_;
	Eigen::Vector3d sum_;
	double xx_;
	double xy_;
	double xz_;
	double yy_;
	double yz_;
	double zz_;

};

#endif
