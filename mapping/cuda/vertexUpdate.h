#ifndef FILE_VERTEX_UPDATE_H
#define FILE_VERTEX_UPDATE_H

#include <cuda.h>
#include <cublas.h>
#include <vector>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include "gpuMeshStructure.h"
#include "stdTexUpdate.h"


namespace gpu{
    struct GeometryUpdate::CalcCenterTask;
}
//struct gpu::GeometryUpdate::CalcCenterTask;

#ifdef __CUDACC__
__global__ void vertexUpdate_kernel(const cudaSurfaceObject_t geometryInput, //the sensor input adapted by standard deviations
                                     int width,int height, //sensor resolution
                                     gpu::UpdateDescriptor* descriptors,
                                     Eigen::Vector4f camPos,//camera position
                                     Eigen::Matrix4f _pose, // because we want the vertex position relative to the camera
                                     Eigen::Matrix4f proj_pose, //to get the position of the point on the image.
                                     GpuVertex *vertices, Eigen::Vector2f *texPos,
                                     GpuTriangle* triangles,GpuPatchInfo* patchInfos);


__global__ void calcCenter_kernel(gpu::GeometryUpdate::CalcCenterTask* tasks,
                                    Eigen::Vector4f* centers);



__global__ void calcRadius_kernel(gpu::GeometryUpdate::CalcCenterTask* tasks,
                                    Eigen::Vector4f* centersRadii);

#endif






#endif
