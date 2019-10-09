#ifndef FILE_REPROJECT_H
#define FILE_REPROJECT_H

#include <Eigen/Eigen>

void reproject(uint16_t *depth_in, uint16_t *depth_out16, uint32_t *depth_out32,
               int width, int height,
               Eigen::Matrix4f pose_transform,
               Eigen::Vector4f depth_intrinsics);

#endif // FILE_REPROJECT_H
