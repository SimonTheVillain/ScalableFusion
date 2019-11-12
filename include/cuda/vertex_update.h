#ifndef FILE_VERTEX_UPDATE_H
#define FILE_VERTEX_UPDATE_H

#include <Eigen/Eigen>

#include "gpu_mesh_structure.h"
#include "std_tex_update.h"

using namespace Eigen;

#ifdef __CUDACC__

__global__ 
void vertexUpdate_kernel(const cudaSurfaceObject_t geometry_input, //the sensor input adapted by standard deviations
                         int width, int height, //sensor resolution
                         gpu::UpdateDescriptor *descriptors,
                         Vector4f cam_pos,//camera position
                         Matrix4f pose, // because we want the vertex position relative to the camera
                         Matrix4f proj_pose, //to get the position of the point on the image.
                         GpuVertex *vertices, Vector2f *tex_pos,
                         GpuTriangle *triangles, GpuPatchInfo *patch_infos);

__global__ 
void calcCenter_kernel(gpu::GeometryUpdate::CalcCenterTask *tasks,
                       Vector4f *centers);

__global__ 
void calcRadius_kernel(gpu::GeometryUpdate::CalcCenterTask *tasks,
                       Vector4f *centers_radii);

#endif // __CUDACC__

#endif // FILE_VERTEX_UPDATE_H
