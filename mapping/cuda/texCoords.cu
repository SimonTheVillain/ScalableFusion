
#include "texCoords.h"
#include "gpuMeshStructure.h"
#include "gpuErrchk.h"
//i would like to include this: but somehow its failing utterly
//#include "../base/meshStructure.h"

#include <limits>



__global__ void genTexCoords_kernel(TexCoordGen::Task* tasks,
                                    Eigen::Matrix4f proj_pose,
                                    GpuPatchInfo* patchInfos,
                                    GpuVertex* vertices){
    int k = blockIdx.x;
    TexCoordGen::Task &task = tasks[k];
    int i=threadIdx.x;
    //ETC: but actually i should work on something different

    //do everything that also is in scaleableMapTexturing.cpp
    while(i<task.triangleCount){

        for(size_t j=0;j<3;j++){
            //get the vertex position of this triangle:
            GpuTriangle triangle = task.triangles[i];
            GpuPatchInfo patchInfo= patchInfos[triangle.patch_info_inds[j]];
            int ind = patchInfo.vertex_source_start_ind + triangle.indices[j];
            Eigen::Vector4f p=vertices[ind].p;

            //now do the projection
            Eigen::Vector4f proj=proj_pose * p;



            //and store the texture coordinate
            Eigen::Vector2f texCoord =Eigen::Vector2f(
                        proj[0]/proj[3],proj[1]/proj[3]);

            Eigen::Vector2f scaled = Eigen::Vector2f(
                        (texCoord[0] - task.offset_x) * task.scale_x ,
                        (texCoord[1] - task.offset_y) * task.scale_y) ;
            //TODO: put the point back into bounds
            task.coords[task.triangles[i].tex_indices[j]] = scaled;
            //printf("unscaled: %f %f \n",texCoord[0],texCoord[1]);
            //printf("scaled: %f %f \n",scaled[0],scaled[1]);
        }


        i+=blockDim.x;
    }

    //ooh shit this also needs the bounds
}


void TexCoordGen::genTexCoords(std::vector<TexCoordGen::Task> tasks,
                               Eigen::Matrix4f proj_pose,
                               GpuPatchInfo* patchInfos,
                               GpuVertex* gpuVertices)
{
    if(tasks.size()==0){
        return;
    }
    TexCoordGen::Task* gpuTasks;
    int bytes = sizeof(TexCoordGen::Task)*tasks.size();
    cudaMalloc(&gpuTasks,sizeof(TexCoordGen::Task)*tasks.size());
    cudaMemcpy(gpuTasks,&(tasks[0]),bytes,cudaMemcpyHostToDevice);

    dim3 block(128);
    dim3 grid(tasks.size());


    genTexCoords_kernel<<<grid,block>>>(gpuTasks,proj_pose,
                                        patchInfos,gpuVertices
                                        );


    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    cudaFree(gpuTasks);
}


/*

void TexCoordGen::genTexCoords(std::vector<std::shared_ptr<MeshPatch>> patches,
                               std::vector<std::shared_ptr<MeshTexture>> texPatches,
                               std::vector<cv::Rect2f> bounds,
                               Eigen::Matrix4f proj_pose,
                               GpuPatchInfo* patchInfos,
                               GpuVertex* gpuVertices)
{
    std::vector<TexCoordGen::Task> tasks;
    for(size_t i=0;i<patches.size();i++){
        std::cout << "TODO: there are multiple triangle sources like stitches "
                     "for each patch" << std::endl;
        MeshPatch* patch = patches[i].get();
        std::shared_ptr<MeshPatchGpuHandle> gpu = patch->gpu.lock();
        TexCoordGen::Task task;
        task.triangles = gpu->triangles->getStartingPtr();
        task.triangleCount = patch->triangles.size();
        task.bound = bounds[i];

        //the tex patch has a parent patch as member... don't need to put the
        //patches to the parameters
        //task.coords = texPatches[i]->gpuTexCoordBuf.lock()->getStartingPtr();


        //tasks.push_back(task);
    }

    genTexCoords(tasks,proj_pose,patchInfos,gpuVertices);

}
*/
struct BoundResult{
    Eigen::Vector4f bound;
    int32_t targetInd = -1;
    uint32_t placeholders[3];
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

__global__ void getTexCoordBounds_kernel(   TexCoordGen::BoundTask* tasks,
                                            Eigen::Matrix4f proj_pose,
                                            GpuPatchInfo* patchInfos,
                                            GpuVertex* vertices,
                                            BoundResult* results){
    __shared__ extern Eigen::Vector4f bounds[];
    int k = blockIdx.x;
    TexCoordGen::BoundTask &task = tasks[k];
    int i=threadIdx.x;
    //ETC: but actually i should work on something different
    Eigen::Vector4f bound(FLT_MAX,
                          FLT_MAX,
                          -FLT_MAX,
                          -FLT_MAX);
    //do everything that also is in scaleableMapTexturing.cpp
    while(i<task.triangleCount){
        GpuTriangle triangle = task.triangles[i];
        for(size_t j=0;j<3;j++){
            //get the vertex position of this triangle:
            GpuPatchInfo patchInfo= patchInfos[triangle.patch_info_inds[j]];
            int ind = patchInfo.vertex_source_start_ind + triangle.indices[j];
            Eigen::Vector4f p=vertices[ind].p;

            //now do the projection
            Eigen::Vector4f proj=proj_pose * p;


            //and store the texture coordinate
            Eigen::Vector2f texCoord =Eigen::Vector2f(
                        proj[0]/proj[3],proj[1]/proj[3]);
            bound[0] = min(bound[0],texCoord[0]);
            bound[1] = min(bound[1],texCoord[1]);
            bound[2] = max(bound[2],texCoord[0]);
            bound[3] = max(bound[3],texCoord[1]);


            if(texCoord[0]<-6000){
                //TODO: find out why this tex coord is zero and handle any issues coming with this
                printf("type = %d, p= %f %f %f %f \n uv= %f %f \n task index = %d, traingle %d\n [texCoords.cu]/whyever this fails\n",task.debugType,
                        p[0],p[1],p[2],p[3],texCoord[0],texCoord[1],k,triangle.indices[j]);
            }
        }


        i+=blockDim.x;
    }

    //return;
    //bounds[threadIdx.x] = bound;
    //__syncthreads();

    int tid = threadIdx.x;
    bounds[tid] = bound;
    __syncthreads();
    for(int s=blockDim.x/2;s>0;s>>=1){
        if(tid>=s){
            //return;
        }
        if(tid<s){
            bounds[tid][0] = min(bounds[tid][0],bounds[tid+s][0]);
            bounds[tid][1] = min(bounds[tid][1],bounds[tid+s][1]);
            bounds[tid][2] = max(bounds[tid][2],bounds[tid+s][2]);
            bounds[tid][3] = max(bounds[tid][3],bounds[tid+s][3]);
        }
        __syncthreads();
    }
    if(tid==0){
        results[k].bound = bounds[0];
        results[k].targetInd = task.targetInd;
        //TODO: output
        //vertices[desc.vertInd].c = s_colors[0]*(1.0f/float(absolutePixCount));
    }


}

std::vector<cv::Rect2f> TexCoordGen::getPotentialTexCoordBounds(std::vector<TexCoordGen::BoundTask> tasks,
                                                                Eigen::Matrix4f proj_pose, int resultCount,
                                                                GpuPatchInfo* patchInfos,GpuVertex* vertices)
{
    if(tasks.size()==0){
        std::vector<cv::Rect2f> empty;
        return empty;
    }
    TexCoordGen::BoundTask* gpuTasks;
    cudaMalloc(&gpuTasks,sizeof(TexCoordGen::BoundTask)*tasks.size());
    BoundResult* gpuResults;
    cudaMalloc(& gpuResults,sizeof(BoundResult)*tasks.size());
    cudaMemcpy( gpuTasks,&(tasks[0]),
            sizeof(TexCoordGen::BoundTask) * tasks.size(),
            cudaMemcpyHostToDevice);


    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    dim3 block(128);
    dim3 grid( tasks.size() );
    int shared = block.x*sizeof(Eigen::Vector4f);
    if(grid.x>65000){
        grid.x=65000;
        printf("This is too many elements in the grid! fix this");
    }
    getTexCoordBounds_kernel<<<grid,block,shared>>>(   gpuTasks,
                                proj_pose,
                                patchInfos,
                                vertices,
                                gpuResults);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    std::vector<BoundResult> rawRes(tasks.size());
    cudaMemcpy(&(rawRes[0]),gpuResults,
            sizeof(BoundResult)* tasks.size(),
            cudaMemcpyDeviceToHost);


    cudaFree(gpuTasks);
    cudaFree(gpuResults);
    std::vector<Eigen::Vector4f> results(resultCount,
                                         Eigen::Vector4f(FLT_MAX,FLT_MAX,
                                                         -FLT_MAX,-FLT_MAX));


    for(size_t i=0;i<rawRes.size();i++){
        BoundResult result = rawRes[i];

        int ind = result.targetInd;

        results[ind][0] = min(results[ind][0],result.bound[0]);
        results[ind][1] = min(results[ind][1],result.bound[1]);
        results[ind][2] = max(results[ind][2],result.bound[2]);
        results[ind][3] = max(results[ind][3],result.bound[3]);


    }



    std::vector<cv::Rect2f> bounds(resultCount);
    for(size_t i=0;i<bounds.size();i++){
        bounds[i].x = results[i][0];
        bounds[i].y = results[i][1];
        bounds[i].width = results[i][2]-results[i][0];
        bounds[i].height = results[i][3]-results[i][1];
    }
    return bounds;
}

/*
std::vector<cv::Rect2f> TexCoordGen::getPotentialTexCoordBounds(
        std::vector<std::shared_ptr<MeshPatch>> patches,
        Eigen::Matrix4f proj_pose)
{
    //compile list of tasks for these patches.


    //run on gpu


    //collect the results
    std::vector<cv::Rect2f> bounds;
    return bounds;

}
*/


