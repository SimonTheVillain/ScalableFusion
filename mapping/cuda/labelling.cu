#include "labelling.h"
#include "gpuErrchk.h"
#include "surfaceRead.h"

using namespace Eigen;

namespace gpu{

__global__ void labelSurfaces_kernel(Labelling::SegProjTask* tasks,
                                    const cudaSurfaceObject_t labelling,
                                    const cudaSurfaceObject_t geomBuffer,//the depth and geometry at the current view
                                    cv::Size2i resolution,
                                    Eigen::Matrix4f _pose,
                                    Eigen::Matrix4f proj_pose,
                                    GpuVertex *vertices,
                                    Eigen::Vector2f *texPos,
                                    GpuTriangle* triangles,
                                    GpuPatchInfo* patchInfos){

    const int width = resolution.width;
    const int height = resolution.height;
    uint32_t k = blockIdx.x;
    Labelling::SegProjTask &task=tasks[k];
    uint32_t i= threadIdx.x;


    uint32_t vertexSourceOffset = task.vertexDestStartInd;
    //uint32_t vertexDestOffset = task.vertexDestinationStartInd;

    int pixelCount = task.destination.width*task.destination.height;
    //int l=0;
    while(i<pixelCount){
        //same procedure as in stdTexUpdate.cu
        int x=i%task.destination.width;
        int y=i/task.destination.width;

        int xRef = x+task.lookup.x;
        int yRef = y+task.lookup.y;

        int xDest = x+task.destination.x;
        int yDest = y+task.destination.y;


        //placeholder code for serious labelling!
        //l++;

        /*
        surf2Dwrite(l*1000,task.destSurf,//l*1000
                    xDest*sizeof(int4) + task.subchannel * sizeof(int),
                    //this could lead to alignment issues
                    yDest);
        */

        /*
        int4 data = make_int4(100000,0,100000,0);
        surf2Dwrite(data,task.destSurf,
                    xDest*sizeof(int4),yDest);

        */

        //DEBUG THINGY END THIS AND PUT A UNIFORM COLOR IN PLACE!!!!!!!<
        if(false){
            float4 data = make_float4(0.5f,0,0,0);
            surf2Dwrite(data,task.destSurf,
                        xDest*sizeof(float4),yDest);
            i+=blockDim.x;
            continue;
        }

        //readout the lookup shizzle
        float4 ref;
        if(xRef>=1024 || yRef>=1024){
            printf("This is not right!!!! %d, %d \n",xRef,yRef);
        }
        //TODO: find out why this is creating the memory exception
        surf2Dread(&ref,task.lookupSurf,
                   xRef*sizeof(Vector4f),yRef);

        //get point in space shizzle:
        float bary[3]= {ref.y,ref.z,ref.w};
        int32_t triangleId= *((int*)(&ref.x));

        if(triangleId<0){
            i+=blockDim.x;
            continue;
        }

        if(false){
            //another debug!!!!! The illegal memory access is already happening here!!!
            float4 data = make_float4(0.5f,0,0,0);
            surf2Dwrite(data,task.destSurf,
                        xDest*sizeof(float4),yDest);
            i+=blockDim.x;
            continue;
        }


        GpuTriangle &triangle = triangles[triangleId];

        Vector4f point(0,0,0,0);
        for(int j=0;j<3;j++){
            point += vertices[vertexSourceOffset+triangle.indices[j]].p*bary[j];
        }


        if(k == 0){
            //debug
            Vector4f p = _pose * point;

            //printf("%f %f %f %f \n",p[0],p[1],p[2],p[3]);
        }
        //project point shizzle:
        Vector4f texCoord = proj_pose*point;
        float u=texCoord[0]/texCoord[3];
        float v=texCoord[1]/texCoord[3];

        int ui = round(u);
        int vi = round(v);


        if(u<0 || u>width || v<0 || v>height || true){
            //debug

            //printf("%f %f  \n",u,v);
        }

        //TODO: read old label if we already have a label!!!!!!!!
        int oldLabel;
        int4 oldLabel4;
        surf2Dread(&oldLabel4,task.destSurf,
                    xDest*sizeof(int4),yDest);
        oldLabel = oldLabel4.x;
        if(oldLabel != -1){
            //TODO: when creating the label textures for the patches they need to be initialized with -1
            //don't overwrite the old label if it already exists
            i+=blockDim.x;
            continue;
        }



        int newLabel;
        if(ui<0 || vi<0 || ui>=width || vi>=height ||
            u<0 || u<0 || u>=width || v>=height){
            //this should actually barely happen!!!!!!
            //i+=blockDim.x;
            //continue;
            newLabel=-1;
            newLabel = -4;//debug
        }else{
            //we are not out of bounds
            float depth;
            surf2Dread(&depth,geomBuffer,ui*sizeof(float),vi);
            if(isnan(depth)){
                newLabel = -1;
                newLabel = -2; // debug
            }else{
                float depthThreshold = 0.005f;//5mm should be enough (or we make it relative to the depth)
                Vector4f posCamFrame = _pose*point;
                if(fabs(posCamFrame[2] - depth) < depthThreshold){
                    newLabel = -1;
                    newLabel = -3;//debug
                }else{
                    //read the new label!!!!!
                    surf2Dread(&newLabel,labelling,ui*sizeof(int),vi);
                    newLabel = 100000;//debug
                }

            }


            //now check if the

        }


        //write the label


        surf2Dwrite(newLabel,task.destSurf,
                    xDest*sizeof(Eigen::Vector4i) + task.subchannel * sizeof(int),
                    //this could lead to alignment issues
                    yDest);

        i+=blockDim.x;
    }


}

void Labelling::labelSurfaces(std::vector<Labelling::SegProjTask> tasks,
                              const cudaSurfaceObject_t labelling,
                              const cudaSurfaceObject_t geomBuffer,//the depth and geometry at the current view
                              cv::Size2i resolution,
                              Eigen::Matrix4f _pose,
                              Eigen::Matrix4f proj_pose,
                              GpuVertex *vertices,
                              Eigen::Vector2f *texPos,
                              GpuTriangle* triangles,
                              GpuPatchInfo* patchInfos){


    if(tasks.size()==0){
        return;
    }
    dim3 block(256);
    dim3 grid(tasks.size());
    size_t taskBytes = sizeof(Labelling::SegProjTask)*tasks.size();
    Labelling::SegProjTask *tasksGpu;
    cudaMalloc(&tasksGpu,taskBytes);
    cudaMemcpy(tasksGpu,&(tasks[0]),
            taskBytes,
            cudaMemcpyHostToDevice);

    //TODO: call the kernel
    labelSurfaces_kernel<<<grid,block>>>(tasksGpu,
                                         labelling,
                                         geomBuffer,
                                         resolution,
                                         _pose,
                                         proj_pose,
                                         vertices,
                                         texPos,
                                         triangles,
                                         patchInfos);

    cudaFree(tasksGpu);
    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );



}


template<class T>
__global__ void initializeSurfaceLabels_kernel(Labelling::InitializeTask* tasks,T value){
    uint32_t k = blockIdx.x;
    Labelling::InitializeTask &task=tasks[k];
    uint32_t i= threadIdx.x;



    int pixelCount = task.destRect.width*task.destRect.height;
    while(i<pixelCount) {

        int x=i%task.destRect.width;
        int y=i/task.destRect.width;

        int xDest = x+task.destRect.x;
        int yDest = y+task.destRect.y;

        surf2Dwrite(value,task.destSurf,
                    xDest*sizeof(T),
                    yDest);


        i+=blockDim.x;
    }
}

template<class T>
void Labelling::initializeSurfaces(std::vector<Labelling::InitializeTask> tasks, T value) {
    if(tasks.size()==0){
        return;
    }
    dim3 block(256);
    dim3 grid(tasks.size());


    size_t taskBytes = sizeof(Labelling::InitializeTask)*tasks.size();
    Labelling::InitializeTask *tasksGpu;
    cudaMalloc(&tasksGpu,taskBytes);
    cudaMemcpy(tasksGpu,&(tasks[0]),
               taskBytes,
               cudaMemcpyHostToDevice);


    //do the kernel call!!!!!!
    initializeSurfaceLabels_kernel<<<grid,block>>>(tasksGpu,value);


    cudaFree(tasksGpu);
    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );




}

//specialization
template
void Labelling::initializeSurfaces(std::vector<Labelling::InitializeTask> tasks,
                               int32_t value);

template<>
void Labelling::initializeSurfaces(std::vector<Labelling::InitializeTask> tasks,
                               Eigen::Vector4f value){
    float4 value2 = make_float4(value[0],value[1],value[2],value[3]);
    Labelling::initializeSurfaces(tasks,value2);
}


}
