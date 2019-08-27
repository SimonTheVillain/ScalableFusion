#include "coarseUpdate.h"
#include "gpuErrchk.h"

//stupid debug shit!!!
#include <iostream>
using namespace std;

using namespace Eigen;
__global__ void reduceColorAtPatches_kernel(CalcMeanColorDescriptor* descriptors,GpuCoarseVertex* vertices,Vector4f *colors){
    extern __shared__ Vector4f s_colors[];//this should not necessarily hardcoded
    const int k = blockIdx.x;
    Vector4f color(0,0,0,0);

    //sum up over all the elements
    CalcMeanColorDescriptor &desc = descriptors[k];


    if(desc.vertInd < 0){
        return;
    }

    int absolutePixCount = desc.height*desc.width;

    int i=threadIdx.x;
    while(i<absolutePixCount){
        int x=i%desc.width + desc.x;
        int y=i/desc.width + desc.y;

        Vector4f pixColor;
        if(desc.hdr){
            //if the image is stored as hdr we load it as such
            printf("TODO!!!!! \n");
        }else{
            //otherwise we load it as 8 bit per color image
            uchar4 c;
            surf2Dread(&c,desc.color,x*sizeof(uchar4),y);
            pixColor = Vector4f(c.x,c.y,c.z,c.w)*(desc.scale/255.0f);
            if(k==0){
                //printf("%d %d %d %d \n",int(c.x),int(c.y),int(c.z),int(c.w));
                //printf("%f %f %f %f \n",pixColor[0],pixColor[1],pixColor[2],pixColor[3]);
            }
        }
        color+=pixColor;

        i+=blockDim.x;
    }



    //Do reduction to get the mean color
    int tid = threadIdx.x;
    s_colors[tid]=color;
    __syncthreads();
    for(int s=blockDim.x/2;s>0;s>>=1){
        if(tid>=s){
            //return;//TODO: maybe add this back in!?
        }
        if(tid<s){
            s_colors[tid]+=s_colors[tid+s];
        }
        __syncthreads();
    }
    if(tid==0 &&  desc.vertInd >= 0){
        vertices[desc.vertInd].c = s_colors[0]*(1.0f/float(absolutePixCount));
    }
    //we should be able to get rid of the loop
    //it will be different beginning with cuda 9 though
    //http://developer.download.nvidia.com/compute/cuda/1.1-Beta/x86_website/projects/reduction/doc/reduction.pdf
}


void calcMeanColor(std::vector<CalcMeanColorDescriptor> descriptors,
                                           GpuCoarseVertex* vertices,
                                           Vector4f* colors){
    if(descriptors.size()==0){
        return;
    }
        //lets do 1024 threads for each patch
    dim3 block(1024);
    dim3 grid(descriptors.size());

    Vector4f* colorsGpu = 0;
    if(colors){
        cudaMalloc(&colorsGpu,descriptors.size()*sizeof(Vector4f));
    }

    CalcMeanColorDescriptor *descs;
    cudaMalloc(&descs,descriptors.size()*sizeof(CalcMeanColorDescriptor));
    cudaMemcpy(descs,&descriptors[0],descriptors.size()*sizeof(CalcMeanColorDescriptor),cudaMemcpyHostToDevice);//copy it to the device

    gpuErrchk( cudaPeekAtLastError() );


    //do the kernel call
    reduceColorAtPatches_kernel<<< grid,block,
            sizeof(Vector4f)*block.x>>>(descs,vertices,colorsGpu);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );
    cudaFree(descs);

    if(colors){
        cudaMemcpy(colorsGpu,colors,descriptors.size()*sizeof(Vector4f),cudaMemcpyDeviceToHost);//copy it to the device
        cudaFree(colorsGpu);

        cudaDeviceSynchronize();
        gpuErrchk( cudaPeekAtLastError() );
    }
}


__global__ void setVisibility_kernel(   int* visibIndices,
                                        size_t nrIndices,
                                        int* gpuVisBuffer,
                                        int value){
    const int i = threadIdx.x + blockIdx.x*blockDim.x;
    if(i>=nrIndices){
        return;
    }

    int index=visibIndices[i];
    gpuVisBuffer[index]=value;
}


void coarseUpdateVisibility(std::vector<int> enablePatches,
                            std::vector<int> disablePatches,
                            int* gpuVisBuffer){

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );
    int *data;
    cudaMalloc(&data,sizeof(int)*std::max(enablePatches.size(),disablePatches.size()));

    //cout << data << endl;
    dim3 block(256);

    dim3 grid(enablePatches.size()/block.x+1);

    cudaMemcpy(data,&enablePatches[0],
            enablePatches.size()*sizeof(int),
            cudaMemcpyHostToDevice);//copy it to the device

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    if( enablePatches.size() != 0 ){
        setVisibility_kernel<<<grid,block>>>(data,
                                             enablePatches.size(),
                                             gpuVisBuffer,1);//1
    }

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );



    grid.x=disablePatches.size()/block.x+1;

    cudaMemcpy(data,&disablePatches[0],
            disablePatches.size()*sizeof(int),
            cudaMemcpyHostToDevice);//copy it to the device
    if( disablePatches.size() != 0 ){
        setVisibility_kernel<<<grid,block>>>(data,
                                             disablePatches.size(),
                                             gpuVisBuffer,0);
    }



    cudaError err = cudaPeekAtLastError();
    if(err != cudaSuccess){
        cout << data << endl;// i fear the pointer is empty for some weird reason
    }
    cudaDeviceSynchronize();
    gpuErrchk( err );

    cudaFree(data);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

}
