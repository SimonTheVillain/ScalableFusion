#ifndef FILE_VERTEX_UPDATE_H
#define FILE_VERTEX_UPDATE_H

#include <vector>

#include <cuda.h>
#include <cublas.h>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>

#include "gpuMeshStructure.h"
#include "stdTexUpdate.h"

#ifdef __CUDACC__

__global__ 
void vertexUpdate_kernel(const cudaSurfaceObject_t geometry_input, //the sensor input adapted by standard deviations
                         int width, int height, //sensor resolution
                         gpu::UpdateDescriptor *descriptors,
                         Eigen::Vector4f cam_pos,//camera position
                         Eigen::Matrix4f pose, // because we want the vertex position relative to the camera
                         Eigen::Matrix4f proj_pose, //to get the position of the point on the image.
                         GpuVertex *vertices, Eigen::Vector2f *tex_pos,
                         GpuTriangle *triangles, GpuPatchInfo *patch_infos);


__global__ 
void calcCenter_kernel(gpu::GeometryUpdate::CalcCenterTask *tasks,
                       Eigen::Vector4f* centers);



__global__ 
void calcRadius_kernel(gpu::GeometryUpdate::CalcCenterTask *tasks,
                       Eigen::Vector4f* centers_radii);

#endif

#endif
