#include "principalPlane.h"

using namespace Eigen;

PrincipalPlaneAccumulator::PrincipalPlaneAccumulator()
{
    clear();
}

PrincipalPlaneAccumulator::~PrincipalPlaneAccumulator()
{

}

void PrincipalPlaneAccumulator::clear()
{

    count=0;
    sum = Eigen::Vector3d(0,0,0);//(0,0,0);
    xx = 0;
    xy = 0;
    xz = 0;
    yy = 0;
    yz = 0;
    zz = 0;
}

void PrincipalPlaneAccumulator::addPoint(Eigen::Vector4f point)
{
    Eigen::Vector3d p = Eigen::Vector3d(point[0],point[1],point[2]);
    sum += p;
    xx += p[0]*p[0];
    xy += p[0]*p[1];
    xz += p[0]*p[2];
    yy += p[1]*p[1];
    yz += p[1]*p[2];
    zz += p[2]*p[2];
    count++;

}

Eigen::Vector4f PrincipalPlaneAccumulator::plane()
{
    Matrix3d mat;
    mat << xx, xy, xz,
            xy, yy, yz,
            xz, yz, zz;

    Vector3d plane = mat.ldlt().solve(sum);
    return Vector4f(plane[0],plane[1],plane[2],0);
}


Vector4f PrincipalPlaneAccumulator::centerPoint()
{
    Vector3d c = sum*(1.0/double(count));
    return Vector4f(c[0],c[1],c[2],0.0f);
}

