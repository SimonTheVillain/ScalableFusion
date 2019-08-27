#ifndef FILE_COARSE_UPDATE_H
#define FILE_COARSE_UPDATE_H

#include <Eigen/Eigen>

#include <cuda.h>
#include <cublas.h>

#include <vector>


#include "gpuMeshStructure.h"


//TODO: the next two functions could be generalized (its a index attached to some structure to store
struct CoarseUpdateVisDescriptor{
    int vertInd=-1;//the index of the vector
    int visibility=1;
};


void coarseUpdateVis(std::vector<CoarseUpdateVisDescriptor> descriptors,int* gpuVisBuf);


void coarseUpdateVisibility(std::vector<int> enablePatches, std::vector<int> disablePatches,int* gpuVisBuffer);

struct CoarseUpdateVertDescriptor{
    int vertInd=-1;//the index of the
    GpuCoarseVertex vert;

};


void coarseUpdateVert(std::vector<CoarseUpdateVisDescriptor> descriptors,GpuCoarseVertex* vertices);



struct CalcMeanColorDescriptor{
    int vertInd=-1;
    //set the
    float scale=1.0f;//scale the color (i.e. when its normalized)

    //reference to the image and where on the image
    cudaSurfaceObject_t color;
    bool hdr = false;
    int x;
    int y;
    int width;
    int height;
};

void calcMeanColor(std::vector<CalcMeanColorDescriptor> descriptors, GpuCoarseVertex* vertices, Eigen::Vector4f *colors);


struct CalcPatchCenter{
    //todo:
    //reference to the patch and information of its size.


    int storeAtVertex=-1;
};






#endif
