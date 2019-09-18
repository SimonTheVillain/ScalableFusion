#ifndef FILE_STD_TEX_UPDATE_H
#define FILE_STD_TEX_UPDATE_H
/**
 * the code that allowes us to update the the standard deviation and deviation texture.
 * maybe we also update the vertices with functions defined within this file
 */

#include <vector>

#include <cuda.h>
#include <cublas.h>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>

#include "gpuMeshStructure.h"
#include "geomUpdate.h"

/**
 * Discussion whats doable with a surface object and what is not:
 * CUDA arrays are opaque memory layouts optimized for texture fetching.
 * They are one dimensional, two dimensional, or three-dimensional and composed of elements,
 * each of which has 1, 2 or 4 components that may be signed or unsigned 8-, 16-, or 32-bit integers,
 * 16-bit floats, or 32-bit floats. CUDA arrays are only accessible by kernels through texture
 * fetching as described in Texture Memory or surface reading and writing as described in Surface Memory.
 * SO: we have to have a surface object.... lets see about that...
 * ALSO: reading and writing to a surface from the same thread is baaaad when only doing one surface at only one
 * workgroup it might be OK, but doing this for further workgroups this might be a pretty bad idea:
 * https://devtalk.nvidia.com/default/topic/686211/using-surfaces-in-a-stack-implementation/
 */

using namespace std;
using namespace Eigen;

void updateGeomTexturesOfPatches(const cudaSurfaceObject_t geometry_input, //the sensor input adapted by standard deviations
                                 int width, int height, //sensor resolution
                                 const vector<gpu::UpdateDescriptor> &descriptors,
                                 Vector4f cam_pos,
                                 Matrix4f pose, // because we want the vertex position relative to the camera
                                 Matrix4f proj_pose, //to get the position of the point on the image.
                                 GpuVertex *vertices, Vector2f *tex_pos,
                                 GpuTriangle *triangles, 
                                 GpuPatchInfo *patch_infos); //pointer to the geometric data


#ifdef __CUDACC__
__global__ 
void updateGeomTex_kernel(const cudaSurfaceObject_t geometry_input, //the sensor input adapted by standard deviations
                          int width, int height, //sensor resolution
                          gpu::UpdateDescriptor* descriptors,
                          Vector4f cam_pos, //camera position
                          Matrix4f _pose, // because we want the vertex position relative to the camera
                          Matrix4f proj_pose, //to get the position of the point on the image.
                          GpuVertex *vertices, Vector2f *tex_pos,
                          GpuTriangle *triangles, GpuPatchInfo *patch_infos);
#endif

//TODO: a simple dilation kernel

struct DilationDescriptor {
		cudaSurfaceObject_t target;
		//theoretically output and input should be different but we do this to not have
		//invalid references(red lines) in the lookup texture
		int width;
		int height;
		int x;
		int y;
};

struct InitDescriptor {
		cudaSurfaceObject_t output;

		cudaSurfaceObject_t reference_texture;

		cv::Point2i ref_offset;
		cv::Point2i out_offset;
		int width;
		int height;
};

void dilateLookupTextures(const vector<DilationDescriptor> &descriptors);

void stdTexInit(const cudaTextureObject_t input, 
                const vector<InitDescriptor> &descriptors,
                Matrix4f proj_pose,
                GpuVertex *vertices, Vector2f *tex_pos,
                GpuTriangle *triangles, GpuPatchInfo *patch_infos); //somehow we now need the gpu textures

void shiftVerticesALittle(GpuVertex *vertices, size_t from, size_t to);

#endif
