#include "geomUpdate.h"

#include "vertexUpdate.h"
#include "stdTexUpdate.h"
#include "helper_math.h"
#include "surfaceRead.h"
#include "xtionCameraModel.h"

#include "../base/meshStructure.h"

using namespace Eigen;

void gpu::GeometryUpdate::calcCenterAndRadius(std::vector<std::shared_ptr<MeshPatch>> &patches) {

    //skip all of this if the vector is empty
    if(patches.size()==0){
        return;
    }
    std::vector<gpu::GeometryUpdate::CalcCenterTask> tasks;

    int debug=0;
    for(std::shared_ptr<MeshPatch> patch : patches){
        gpu::GeometryUpdate::CalcCenterTask task;
        std::shared_ptr<MeshPatchGpuHandle> gpuPatch = patch->gpu.lock();
        if(gpuPatch == nullptr){
            assert(0);
        }
        std::shared_ptr<VertexBufConnector> buffer = gpuPatch->verticesSource;
        task.vertices = buffer->getStartingPtr();
        task.count = buffer->getSize();
        tasks.push_back(task);
        debug++;

    }

    //allocate gpu memory
    gpu::GeometryUpdate::CalcCenterTask* gpuTasks;
    size_t bytes = sizeof(gpu::GeometryUpdate::CalcCenterTask)*tasks.size();
    cudaMalloc(&gpuTasks,bytes);
    cudaMemcpy(gpuTasks,&tasks[0],bytes,cudaMemcpyHostToDevice);
    Eigen::Vector4f *gpuCenters;
    cudaMalloc(&gpuCenters,sizeof(Eigen::Vector4f)*tasks.size());



    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );



    //calc center and radius in one run
    dim3 block(256);
    dim3 grid(tasks.size());
    bytes = sizeof(Eigen::Vector4f)*block.x;
    calcCenterAndRadiusKernelCall(grid,block,bytes,gpuTasks,gpuCenters);
//    calcCenter_kernel<<<grid,block,bytes>>>(gpuTasks,gpuCenters);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    Eigen::Vector4f centers[tasks.size()];
    cudaMemcpy(centers,gpuCenters,sizeof(Eigen::Vector4f)*tasks.size(),cudaMemcpyDeviceToHost);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );


    cudaFree(gpuTasks);
    cudaFree(gpuCenters);
    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    for(size_t i = 0; i< tasks.size();i++){
        //std::cout << "new center" << std::endl << centers[i] << std::endl;
        //TODO: put this back in, because it is crashing

        patches[i]->setSphere(centers[i].block<3,1>(0,0),
                              sqrt(centers[i][3]));

    }
    //calc radius

    //download it,


    //and last step:
    //maybe even set this stuff on the patch itself. or all patches at once?
    //it at least needs to be efficiently set for the octree

    //assert(0);

}