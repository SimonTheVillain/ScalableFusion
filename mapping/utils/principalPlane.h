#ifndef FILE_PRINCIPAL_PLANE_H
#define FILE_PRINCIPAL_PLANE_H

#include <Eigen/Eigen>


/**
 * @brief The PrincipalPlaneAccumulator class
 * This class is used to calculate the principal plane for each patch.
 * Maybe at some point this class could become more than that and calculate and contain information
 * about curvature and stuff.
 */
class PrincipalPlaneAccumulator{
private:
    int count=0;
    Eigen::Vector3d sum;//(0,0,0);
    double xx;
    double xy;
    double xz;
    double yy;
    double yz;
    double zz;
public:
    PrincipalPlaneAccumulator();
    ~PrincipalPlaneAccumulator();
    void clear();
    //
    void addPoint(Eigen::Vector4f point);

    //calc the plane we need to have
    Eigen::Vector4f plane();
    Eigen::Vector4f centerPoint();


};


#endif
