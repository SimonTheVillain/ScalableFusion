#include "geomUpdate.h"

#include "vertexUpdate.h"
#include "stdTexUpdate.h"
#include "helper_math.h"
#include "surfaceRead.h"
#include "xtionCameraModel.h"

#include "../base/meshStructure.h"

using namespace Eigen;



//We only should do a texture update for patches whose all neighbour patches are loaded.
//this means we do the vertex update for all patches that are loaded, but only do the textre update
//where all the neighbouring patches are present
//TODO: get rid of this integer debug value
int gpu::updateGeometry(const cudaSurfaceObject_t geometryInput, //the sensor input adapted by standard deviations
                   int width,int height, //sensor resolution
                   const std::vector<gpu::UpdateDescriptor> &descriptors,
                   Eigen::Vector4f camPos,
                   Eigen::Matrix4f _pose, // because we want the vertex position relative to the camera
                   Eigen::Matrix4f proj_pose, //to get the position of the point on the image.
                   GpuVertex *vertices, Eigen::Vector2f *texPos,
                   GpuTriangle* triangles,GpuPatchInfo* patchInfos){ //pointer to the geometric data


    if(descriptors.size()==0){
        return-1;
    }
    dim3 block(1024);
    dim3 grid(descriptors.size());
    gpu::UpdateDescriptor *descs;
    cudaMalloc(&descs,descriptors.size()*sizeof(gpu::UpdateDescriptor));
    cudaMemcpy(descs,&descriptors[0],descriptors.size()*sizeof(gpu::UpdateDescriptor),cudaMemcpyHostToDevice);//copy it to the device

    gpuErrchk( cudaPeekAtLastError() );



    cudaDeviceSynchronize();//just for debug!!!
    gpuErrchk( cudaPeekAtLastError() );//delete

    //first we update the vertices

    vertexUpdate_kernel<<< grid,block >>>(
            geometryInput, //the sensor input adapted by standard deviations
                    width,height, //sensor resolution
                    descs,
                    camPos,
                    _pose, // because we want the vertex position relative to the camera
                    proj_pose, //to get the position of the point on the image.
                    vertices, texPos,
                    triangles,patchInfos);

    cudaDeviceSynchronize();//just for debug!!!
    gpuErrchk( cudaPeekAtLastError() );//delete

    //TODO: remove
    //debug purposes. We are at the guessing stage of debugging
    /*
    int debugSize = descriptors.size();
    for(int i=0;i<descriptors.size();i++){
        UpdateDescriptor descriptor = descriptors[i];
        //WHOOOOT references and geometry are the same? TODO: don't think this is the case!
        dim3 grid2(i+1);
        updateGeomTex_kernel<<< grid2,block>>>(
                geometryInput, //the sensor input adapted by standard deviations
                        width,height, //sensor resolution
                        descs,
                        camPos,
                        _pose, // because we want the vertex position relative to the camera
                        proj_pose, //to get the position of the point on the image.
                        vertices, texPos,
                        triangles,patchInfos);



        //then we update the texture.
        //why is this crashing????



        cudaDeviceSynchronize();
        if (cudaPeekAtLastError() != cudaSuccess){
            return i;
        }
        gpuErrchk( cudaPeekAtLastError() );
    }
     */

    updateGeomTex_kernel<<< grid,block>>>(
                               geometryInput, //the sensor input adapted by standard deviations
                               width,height, //sensor resolution
                               descs,
                               camPos,
                               _pose, // because we want the vertex position relative to the camera
                               proj_pose, //to get the position of the point on the image.
                               vertices, texPos,
                               triangles,patchInfos);



    //then we update the texture.
    //why is this crashing????


    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );
    cudaFree(descs);

    return -1;
}


__global__
void checkVertexValidity_kernel(
        const cudaSurfaceObject_t sensorImage,
        int width, int height,
        Eigen::Matrix4f _pose,
        Eigen::Matrix4f proj_pose,
        gpu::GeometryValidityChecks::VertexTask *tasks,
        GpuVertex *vertices){

    uint32_t k = blockIdx.x;
    gpu::GeometryValidityChecks::VertexTask task=tasks[k];

    uint32_t i = threadIdx.x;
    while(i<task.size){
        GpuVertex vertex = vertices[i+task.startSource];

        //TODO: all the logic!!!!!
        //calculate position on the sensor
        Eigen::Vector4f p = vertex.p;
        p[3]=1;
        Eigen::Vector4f p_cam = _pose*p;
        float z = p_cam[2];

        Eigen::Vector4f posOnSensor = proj_pose * p;

        float u=posOnSensor[0]/posOnSensor[3];
        float v=posOnSensor[1]/posOnSensor[3];

        float4 sensor =
                readSensor(u,v,
                                 sensorImage,
                                 width,height,
                           0.05);//threshold =0.1


        float threshold = xtionStdToThresholdSeg(sensor.y);//the second one is the surface

        if(z < (sensor.x-threshold) && !isnan(sensor.x)){
            //invalidate vertex
            //for debug purposes do it in the source
            //TODO: do it at destRect
            vertices[i+task.startSource].valid=0;
        }

        vertices[i+task.startDest] = vertex;
        i+=blockDim.x;
    }



}

void gpu::GeometryValidityChecks::checkVertexValidity(
                     const cudaSurfaceObject_t sensor,
                     int width, int height,
                     Eigen::Matrix4f _pose,
                     Eigen::Matrix4f proj_pose,
                     std::vector<GeometryValidityChecks::VertexTask> tasks,
                     GpuVertex *vertices){
    //return;
    if(tasks.size()==0){
        return;
    }
    dim3 block(256);
    dim3 grid(tasks.size());
    gpu::GeometryValidityChecks::VertexTask *gpuTasks;

    cudaMalloc(&gpuTasks,
               sizeof(gpu::GeometryValidityChecks::VertexTask)*tasks.size());

    cudaMemcpy(gpuTasks,&(tasks[0]),
            sizeof(gpu::GeometryValidityChecks::VertexTask)*tasks.size(),
            cudaMemcpyHostToDevice);

    checkVertexValidity_kernel<<<grid,block>>>(sensor,
                                               width,height,
                                               _pose,
                                               proj_pose,
                                               gpuTasks,
                                               vertices);

    cudaFree(gpuTasks);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

}








__global__
void checkTriangleValidity_kernel(gpu::GeometryValidityChecks::TriangleTask *tasks,
                                  const cudaSurfaceObject_t sensor,
                                  int width, int height,
                                  Eigen::Matrix4f _pose,
                                  Eigen::Matrix4f proj_pose,
                                  GpuVertex *vertices,
                                  Eigen::Vector2f *texPos,
                                  GpuTriangle *triangles,
                                  GpuPatchInfo *patchInfos){
    uint32_t k = blockIdx.x;


}
void gpu::GeometryUpdate::calcCenterAndRadiusKernelCall(dim3 grid, dim3 block,size_t bytes,
                                                   CalcCenterTask * gpuTasks,Eigen::Vector4f *results){
    calcCenter_kernel<<<grid,block,bytes>>>(gpuTasks,results);

}
/*
void GeometryUpdate::calcCenterAndRadius(std::vector<std::shared_ptr<MeshPatch>> &patches) {

    std::shared_ptr<float> debugTestShared(new float);
    *debugTestShared = 1.0f;
    std::weak_ptr<float> debugTestWeak = debugTestShared;

    std::cout << *debugTestWeak.lock() << std::endl;
    //TODO: implement this! we really need to
    if(patches.size()==0){
        return;
    }
    std::vector<GeometryUpdate::CalcCenterTask> tasks;

    int debug=0;
    for(std::shared_ptr<MeshPatch> patch : patches){
        GeometryUpdate::CalcCenterTask task;
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
    GeometryUpdate::CalcCenterTask* gpuTasks;
    size_t bytes = sizeof(GeometryUpdate::CalcCenterTask)*tasks.size();
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
    calcCenter_kernel<<<grid,block,bytes>>>(gpuTasks,gpuCenters);

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
        std::cout << "new center" << std::endl << centers[i] << std::endl;
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
 */

void gpu::GeometryValidityChecks::checkTriangleValidity(std::vector<TriangleTask> tasks,
                                                   const cudaSurfaceObject_t sensor,
                                                   int width, int height,
                                                   Eigen::Matrix4f _pose,
                                                   Eigen::Matrix4f proj_pose,
                                                   GpuVertex *vertices,
                                                   Eigen::Vector2f *texPos,
                                                   GpuTriangle *triangles,
                                                   GpuPatchInfo *patchInfos)
{
    assert(0); //this is not a priority yet.
    if(tasks.size()==0){
        return;
    }
    dim3 block(256);
    dim3 grid(tasks.size());
    GeometryValidityChecks::TriangleTask *gpuTasks;

    cudaMalloc(&gpuTasks,
               sizeof(GeometryValidityChecks::TriangleTask)*tasks.size());

    cudaMemcpy(gpuTasks,&(tasks[0]),
            sizeof(GeometryValidityChecks::TriangleTask)*tasks.size(),
            cudaMemcpyHostToDevice);

    checkTriangleValidity_kernel<<<grid,block>>>(gpuTasks,
                                                 sensor,
                                                 width,height,
                                                 _pose,
                                                 proj_pose,
                                                 vertices,
                                                 texPos,
                                                 triangles,
                                                 patchInfos);


    cudaFree(gpuTasks);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

}



