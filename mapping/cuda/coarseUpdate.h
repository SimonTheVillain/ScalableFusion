#ifndef FILE_COARSE_UPDATE_H
#define FILE_COARSE_UPDATE_H

#include <vector>

#include <Eigen/Eigen>
#include <cuda.h>
#include <cublas.h>

#include "gpuMeshStructure.h"

using namespace std;

//TODO: the next two functions could be generalized (its a index attached to some structure to store
struct CoarseUpdateVisDescriptor {
	int vert_ind   = -1;//the index of the vector
	int visibility = 1;
};

struct CoarseUpdateVertDescriptor {
	int vert_ind = -1;//the index of the vector
	GpuCoarseVertex vert;
};

struct CalcMeanColorDescriptor {
	int vert_ind = -1;
	float scale = 1.0f; // scale the color (i.e. when its normalized)

	//reference to the image and where on the image
	cudaSurfaceObject_t color;
	bool hdr = false;
	int x;
	int y;
	int width;
	int height;
};

struct CalcPatchCenter {
	//todo:
	//reference to the patch and information of its size.
	int store_at_vertex = -1;
};

void coarseUpdateVis(vector<CoarseUpdateVisDescriptor> descriptors,
                     int *gpuVisBuf);

void coarseUpdateVisibility(vector<int> enablePatches, 
                            vector<int> disable_patches, int *gpu_vis_buffer);

void coarseUpdateVert(vector<CoarseUpdateVisDescriptor> descriptors,
                      GpuCoarseVertex *vertices);

void calcMeanColor(vector<CalcMeanColorDescriptor> descriptors, 
                   GpuCoarseVertex *vertices, Eigen::Vector4f *colors);

#endif
