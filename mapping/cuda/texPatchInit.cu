#include "texPatchInit.h"


#include <iostream>
#include <stdio.h>
#include <assert.h>

#include "gpuErrchk.h"


/**
 * right now we do this for a 4 channel floatt texture, in future this has to work with multiple texture types
 * + maybe extend this for streams
 */

__global__ void copyToTinyPatches_kernel(const cudaTextureObject_t input,const CopyDescriptor* copies){
    unsigned int k = blockIdx.x;
    const CopyDescriptor &copy = copies[k];
    const unsigned int requiredPixel=copy.targetHeight*copy.targetWidth;
    unsigned int i = threadIdx.x;
    while(i<requiredPixel){
        int x=i%copy.targetWidth;
        int y=i/copy.targetWidth;


        ///TODO: i am not sure about the 0.5f ... as usual
        /// TODO: think about it I THOUGHT AND I THINK THIS IS RIGHT
        /// BUT MANY OF THESE CALCULATIONS COULD BE OPTIMIZED!!!!!
        float u= copy.x + copy.width * ((float)x+0.5f)/(float)copy.targetWidth; //meybe we should store the inverse of width/height
        float v= copy.y + copy.height * ((float)y+0.5f)/(float)copy.targetHeight;

        //now shift x and y to the according place in the texture atlas.
        x=x+copy.targetX;
        y=y+copy.targetY;


        //i feel the coordinates are unfortunately not normalized (maybe we should set the texture accordingly
        float4 color = tex2D<float4>(input, u,v);
        //uchar4 c = tex2D<uchar4>(input,u,v);


        //color=make_float4(1,0,0,1);
        //surf2Dwrite(color,copy.output,x*4*4,y);
        //printf("pix Coord x:%d y:%d of %d x %d \n",x,y,copy.targetWidth,copy.targetHeight);
        //color= make_float4(c.x,c.y,c.z,1);//debug (color coding the output)
        uchar4 colorBytes=make_uchar4(color.x*255,color.y*255,color.z*255,color.w*255);
        surf2Dwrite(colorBytes,copy.output,x*4,y);
        //surf2Dwrite(color,copy.output,x*4*4,y);

        //printf("%d x %d\n",x,y);

        //do the next block
        i+=blockDim.x;
    }
}

void copyToTinyPatches(const cudaTextureObject_t input,const std::vector<CopyDescriptor> &descriptors){
    dim3 block(256);//smaller is better for smaller images. how is it with bigger images?
    //the ideal way would be to start blocks of different sizes. (or maybe use CUDAs dynamic parallelism)
    //for higher resolution images 1024 or even higher threadcounts might be useful
    dim3 grid(descriptors.size());
    if(grid.x==0){
        return;
    }

    //create and copy the descriptors to a gpu buffer
    CopyDescriptor *descs;
    cudaMalloc(&descs,descriptors.size()*sizeof(CopyDescriptor));
    cudaMemcpy(descs,&descriptors[0],descriptors.size()*sizeof(CopyDescriptor),cudaMemcpyHostToDevice);//copy it to the device

    //start kernel
    copyToTinyPatches_kernel<<<grid,block>>>(input,descs);

    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );
    //delete the gpu buffer
    cudaFree(descs);



}
