#ifndef FILE_NORM_EST_H
#define FILE_NORM_EST_H

#include <Eigen/Eigen>

void cudaCalcNormals(cudaSurfaceObject_t d_std_min_std, 
                     cudaSurfaceObject_t points,
                     cudaSurfaceObject_t normals,
                     int width, int height, float threshold);

void cudaCalcPoints(cudaSurfaceObject_t d_std_min_std, 
                    cudaSurfaceObject_t points,
                    int width, int height, Eigen::Vector4f fxycxy);

#endif // FILE_NORM_EST_H


