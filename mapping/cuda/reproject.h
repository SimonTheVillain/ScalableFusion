#ifndef FILE_REPROJECT_H
#define FILE_REPROJECT_H
#include <Eigen/Eigen>
#include <stdint.h>

void reproject(uint16_t *depthIn, uint16_t *depthOut16, uint32_t *depthOut32,
               int width, int height,
               Eigen::Matrix4f poseTransform,
               Eigen::Vector4f depthIntrinsics);


#endif
