#ifndef FILE_NORM_EST_H
#define FILE_NORM_EST_H
#include <cublas.h>
#include <Eigen/Eigen>


void cudaCalcNormals(cudaSurfaceObject_t dStdMinStd, cudaSurfaceObject_t points,
                     cudaSurfaceObject_t normals,
                     int width, int height,float threshold);





void cudaCalcPoints(cudaSurfaceObject_t dStdMinStd, cudaSurfaceObject_t points,
                int width, int height, Eigen::Vector4f fxycxy);




#endif


