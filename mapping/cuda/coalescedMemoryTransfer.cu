#include "coalescedMemoryTransfer.h"
#include "gpuErrchk.h"
#include <iostream>

using namespace std;
using namespace Eigen;

__global__
void downloadVertices_kernel(GpuVertex** ptrs,GpuVertex* out,int count){
    const int i=threadIdx.x + blockIdx.x*blockDim.x;
    if(i>=count){
        return;
    }


    out[i]= *(ptrs[i]);//looking at this it would be a pain not to template this:


    //TWO methods:
    //gather: collect spread stuff in an array
    //scatter: fill spread containers with data
}




void downloadVertices(std::vector<GpuVertex*> gpuVertices,GpuVertex* data){

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    const int size = gpuVertices.size();
    GpuVertex* gpuData;
    cudaMalloc(&gpuData,sizeof(GpuVertex)*size);
    GpuVertex** gpuPtrs;
    cudaMalloc(&gpuPtrs,sizeof(GpuVertex*)*size);
    cudaMemcpy( gpuPtrs,&(gpuVertices[0]),
                sizeof(GpuVertex*)*size,
                cudaMemcpyHostToDevice);

    dim3 block(512);
    dim3 grid( size/block.x + 1 );


    downloadVertices_kernel<<<grid,block>>>(gpuPtrs,gpuData,size);


    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    cudaMemcpy( &(data[0]),gpuData,
                sizeof(GpuVertex)*size,
                cudaMemcpyDeviceToHost);


    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    cudaFree(gpuData);
    cudaFree(gpuPtrs);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );
}


/*
template<typename T>
void CoalescedGpuTransfer::upload<T>(
        void upload(std::vector<T> source,std::vector<CoalescedGpuTransfer::Task> tasks){

}
*/



template<typename T>
__global__ void upload_kernel(T* source,CoalescedGpuTransfer::Task* tasks){
    CoalescedGpuTransfer::Task task = tasks[blockIdx.x];
    int i=threadIdx.x;
    while(i<task.count){
        T unit = source[task.start + i];
        ((T*)(task.target))[i] = unit;
        i+=blockDim.x;
    }
}

template<typename T>
void CoalescedGpuTransfer::upload(std::vector<T> source,std::vector<CoalescedGpuTransfer::Task> tasks){

    if(tasks.size()==0){
        return;
    }
    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );
    dim3 block(256);
    dim3 grid(tasks.size());

    T* gpuSrc;
    CoalescedGpuTransfer::Task* gpuTasks;
    cudaMalloc(&gpuSrc,sizeof(T)*source.size());
    cudaMalloc(&gpuTasks,sizeof(CoalescedGpuTransfer::Task)*tasks.size());
    cudaMemcpy(gpuSrc,&(source[0]),
            sizeof(T)*source.size(),
            cudaMemcpyHostToDevice);
    cudaMemcpy(gpuTasks,&(tasks[0]),
            sizeof(CoalescedGpuTransfer::Task)*tasks.size(),
            cudaMemcpyHostToDevice);

    upload_kernel<<<grid,block>>>(gpuSrc,gpuTasks);


    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    cudaFree(gpuSrc);
    cudaFree(gpuTasks);

}

template<typename T>
__global__ void upload_kernel(T* src,T** dst,int size){
   int i=threadIdx.x + blockIdx.x*blockDim.x;
   if(i<size){
       *(dst[i]) = src[i];
   }
}

template<typename T>
void CoalescedGpuTransfer::upload(std::vector<T> source,std::vector<T*> gpuDst){
    if(source.size()!=gpuDst.size()){
        assert(0); //you are misusing this and you deserve your program to crash
    }
    if(source.size()==0){
        return;
    }
    dim3 block(256);
    dim3 grid(source.size()/block.x + 1);
    T* src;
    T** dst;


    cudaMalloc(&src,sizeof(T)*source.size());
    cudaMalloc(&dst,sizeof(T*)*source.size());

    cudaMemcpy(src,&(source[0]),
                sizeof(T)*source.size(),
                cudaMemcpyHostToDevice);
    cudaMemcpy(dst,&(gpuDst[0]),
                sizeof(T*)*source.size(),
                cudaMemcpyHostToDevice);

    upload_kernel<<<grid,block>>>(src,dst,source.size());



    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    cudaFree(src);
    cudaFree(dst);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );
}



template<typename T>
__global__ void device2DeviceSameBuf_kernel(T* buf,
                                            CoalescedGpuTransfer::TaskD2D* tasks){
    CoalescedGpuTransfer::TaskD2D task = tasks[blockIdx.x];
    int i=threadIdx.x;
    while(i<task.count){
        buf[i + task.destinationIndex] = buf[i + task.sourceIndex];
        i+=blockDim.x;
    }
}

template<typename T>
void CoalescedGpuTransfer::device2DeviceSameBuf(T* buffer,
                                                std::vector<TaskD2D> tasks){

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    if(tasks.size()==0){
        return;
    }
    size_t bytes = sizeof(TaskD2D) * tasks.size();
    TaskD2D* tasksGpu;
    cudaMalloc(&tasksGpu,bytes);
    cudaMemcpy(static_cast<void*>(tasksGpu),
               static_cast<void*>(&(tasks[0])),bytes,
                cudaMemcpyHostToDevice);


    dim3 block(256);
    dim3 grid(tasks.size());

    device2DeviceSameBuf_kernel<<<grid,block>>>(buffer,tasksGpu);




    cudaFree(tasksGpu);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

}

//THIS IS NOT EFFICIENT
template<typename T>
__global__ void upload_kernel(CoalescedGpuTransfer::SetTaskTemp<T>* tasks, int count){
    const int k = blockIdx.x * blockDim.x + threadIdx.x;
    /*const int simultaneousBytes = sizeof(Vector4f);
    const int taskInd;
    //calculate how many bytes come in front,
    //calculate the bytes in the back
    //copy them
    */
    //or just screw it and do the simple thing:
    if(k>=count){
        return;
    }
    *(tasks[k].dst) = tasks[k].value;

}
template<typename T>
void CoalescedGpuTransfer::upload(std::vector<SetTaskTemp<T>> tasks){
    cout << "THIS IS NOT EFFICIENT!!!" << endl;
    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    if(tasks.size()==0){
        return;
    }
    SetTaskTemp<T>* tasksGpu;
    size_t bytes = sizeof(SetTaskTemp<T>) * tasks.size();


    cudaMalloc(&tasksGpu,bytes);
    cudaMemcpy(static_cast<void*>(tasksGpu),
               static_cast<void*>(&(tasks[0])),bytes,
                cudaMemcpyHostToDevice);

    dim3 block(256);
    //dim3 grid(tasks.size()*(sizeof(TaskTemp<T>)/16 + 2);//16 byte alignment
    dim3 grid(tasks.size() / block.x +1);

    upload_kernel<<<grid,block>>>(tasksGpu,tasks.size());



    //this is going to be disgusting:
    for(size_t i=0;i<tasks.size();i++){
        //doing these copies one after another (inefficiently like i dont care)
        cudaMemcpy(static_cast<void*>(tasks[i].dst),
                   static_cast<void*>(&(tasks[i].value)),sizeof(SetTaskTemp<T>),
                    cudaMemcpyHostToDevice);
    }



    cudaFree(tasksGpu);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

}

//THIS IS NOT EFFICIENT
template<typename T>
__global__ void copy_kernel(CoalescedGpuTransfer::CpyTaskTemp<T>* tasks, int count){
    const int k = blockIdx.x * blockDim.x + threadIdx.x;
    /*const int simultaneousBytes = sizeof(Vector4f);
    const int taskInd;
    //calculate how many bytes come in front,
    //calculate the bytes in the back
    //copy them
    */
    //or just screw it and do the simple thing:
    if(k>=count){
        return;
    }
    *(tasks[k].dst) = *(tasks[k].src);

}
template<typename T>
void CoalescedGpuTransfer::copy(std::vector<CpyTaskTemp<T>> tasks){
    cout << "THIS IS NOT EFFICIENT!!!" << endl;
    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    if(tasks.size()==0){
        return;
    }
    CpyTaskTemp<T>* tasksGpu;
    size_t bytes = sizeof(CpyTaskTemp<T>) * tasks.size();


    cudaMalloc(&tasksGpu,bytes);
    cudaMemcpy(static_cast<void*>(tasksGpu),
               static_cast<void*>(&(tasks[0])),bytes,
                cudaMemcpyHostToDevice);

    dim3 block(256);
    //dim3 grid(tasks.size()*(sizeof(TaskTemp<T>)/16 + 2);//16 byte alignment
    dim3 grid(tasks.size() / block.x +1);

    copy_kernel<<<grid,block>>>(tasksGpu,tasks.size());



    cudaFree(tasksGpu);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

}

__global__ void download_kernel(CoalescedGpuTransfer::DirectTask *tasks){
    const int k = blockIdx.x;
    CoalescedGpuTransfer::DirectTask task = tasks[k];
    int i = threadIdx.x;
    uint32_t* src = static_cast<uint32_t*>(task.src);
    uint32_t* dst = static_cast<uint32_t*>(task.dst);

    int words = task.byteCount/4;
    while(i<words){
        dst[i]=src[i];
        i+=blockDim.x;
    }


    i = threadIdx.x;
    int leftover = task.byteCount%4;
    if(i<leftover){
        uint8_t* src8 = static_cast<uint8_t*>(task.src);
        uint8_t* dst8 = static_cast<uint8_t*>(task.dst);
        dst8[task.byteCount - leftover + i] = src8[task.byteCount - leftover + i];

    }
}

void CoalescedGpuTransfer::download(std::vector<CoalescedGpuTransfer::DirectTask> tasks) {
    if(tasks.size() == 0){
        return;
    }
    //cout << "warning this has not been debugged" << endl;
    int wordCount=0; //count of 4 byte values
    //list of starting indices
    vector<int> startingIndices(tasks.size());
    for(int i=0;i<tasks.size();i++){
        startingIndices[i] = wordCount;
        wordCount += tasks[i].byteCount/4;
        if(tasks[i].byteCount%4){
            wordCount++;
        }
    }

    //setup continuous buffer to store
    int* resultGpu;

    cudaMalloc(&resultGpu,wordCount*4);

    std::vector<CoalescedGpuTransfer::DirectTask> tasks2 = tasks;
    for(int i=0;i<tasks.size();i++){
        //instead of the destination pointing to cpu memory as in tasks in tasks2 it should point to GPU mem.
        tasks2[i].dst = &resultGpu[startingIndices[i]];
    }

    //setup and fill buffer for task list
    CoalescedGpuTransfer::DirectTask *tasksGpu;
    size_t bytes = sizeof(CoalescedGpuTransfer::DirectTask)*tasks.size();
    cudaMalloc(&tasksGpu,bytes);
    cudaMemcpy(static_cast<void*>(tasksGpu),
               static_cast<void*>(&(tasks2[0])),bytes,
               cudaMemcpyHostToDevice);


    //run kernel
    dim3 block(256);
    //dim3 grid(tasks.size()*(sizeof(TaskTemp<T>)/16 + 2);//16 byte alignment
    dim3 grid(tasks.size());
    download_kernel<<<grid,block>>>(tasksGpu);

    //download continuous buffer
    vector<uint32_t> result(wordCount*4);
    cudaMemcpy(static_cast<void*>(&(result[0])),
               static_cast<void*>(resultGpu),wordCount*4,
               cudaMemcpyDeviceToHost);

    //destroy buffers
    cudaFree(resultGpu);
    cudaFree(tasksGpu);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    //fill everything to target
    for(int i=0;i<tasks.size();i++){
        //copy the results to the real target...
        memcpy(tasks[i].dst,&result[startingIndices[i]],tasks[i].byteCount);
    }

}



//instantiate this templated method
template void CoalescedGpuTransfer::upload(std::vector<GpuPatchInfo> source,
                                std::vector<GpuPatchInfo*> gpuDst);

//instantiate these templated methods
template void CoalescedGpuTransfer::upload(std::vector<GpuVertex> source,
                                std::vector<CoalescedGpuTransfer::Task> tasks);
template void CoalescedGpuTransfer::upload(std::vector<GpuTriangle> source,
                                std::vector<CoalescedGpuTransfer::Task> tasks);

template void CoalescedGpuTransfer::upload(std::vector<Eigen::Vector2f> source,
                                std::vector<CoalescedGpuTransfer::Task> tasks);


template void CoalescedGpuTransfer::device2DeviceSameBuf(Eigen::Vector2f* buf,
std::vector<CoalescedGpuTransfer::TaskD2D> tasks);

template void CoalescedGpuTransfer::upload(std::vector<CoalescedGpuTransfer::SetTaskTemp<GpuTextureInfo>> tasks);

template void CoalescedGpuTransfer::copy(std::vector<CoalescedGpuTransfer::CpyTaskTemp<GpuTextureInfo>> tasks);
