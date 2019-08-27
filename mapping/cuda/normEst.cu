#include "normEst.h"
#include "helper_math.h"
#include <iostream>
#include <cstdio>

using namespace Eigen;
using namespace std;


#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess)
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) assert(false);
   }
}

//gauss where the width is in line with sigma, but integrating over the curve would not yield 1 as
//it should be as simple as possible
__device__ float guassUnscaled(float x,float sigma){
    return exp(-x*x/(2.0f*sigma*sigma));
}

//the tenth attempt of creating normals.... it is sort of stupid since the normals are in the camera frame.
//maybe we do a transformation from camera space to world space in a different kernel
__global__ void cudaCalcNormals_kernel(cudaSurfaceObject_t dStdMinStd, cudaSurfaceObject_t points,
                                cudaSurfaceObject_t normals,
                                int width, int height,float threshold){


    int x=blockIdx.x * blockDim.x + threadIdx.x;
    int y=blockIdx.y * blockDim.y + threadIdx.y;
    if(x>=width || y>=height){
        return;
    }

    float3 ver = make_float3(0,0,0);
    float3 hor = make_float3(0,0,0);
    float weights[3][3] = {{1,2,1},
                        {0,0,0},
                        {-1,-2,-1}};
    float4 center;
    surf2Dread(&center,points,x*sizeof(float4),y);
    float4 sensor;
    surf2Dread(&sensor,dStdMinStd,x*sizeof(float4),y);
    float sigma = sensor.z*0.2f;//the quantisation noise on the sensor

    //sigma=0.5;
    //for debug:
    //surf2Dwrite(center,normals,x*sizeof(float4),y);
    //return;

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            int _x = x+j-1;
            int _y = y+i-1;
            if(_x>=width || _y>=height || _x<0 || _y<0){
                continue;
            }
            float4 point;
            surf2Dread(&point,points,_x*sizeof(float4),_y);

            float w=guassUnscaled(center.z-point.z,sigma);
            //if(fabs(center.z-point.z)<threshold){
                float vw = w*weights[i][j];//vertical weights
                float hw = w*weights[j][i];//hoizontal weights
                //TODO: test if this is right
                float4 dist=point-center;
                ver += vw*make_float3(dist.x,dist.y,dist.z);
                hor += hw*make_float3(dist.x,dist.y,dist.z);
            //}

        }
    }



    //create the normal from the cross product
    //(why couldn't i just use eigen for this)
    float3 cross;
    cross.x = ver.y*hor.z-ver.z*hor.y;
    cross.y = ver.z*hor.x-ver.x*hor.z;
    cross.z = ver.x*hor.y-ver.y*hor.x;


    float _length=sqrt(cross.x*cross.x+cross.y*cross.y+cross.z*cross.z);
    if(_length==0){
        float4 normal=make_float4(0,0,-1.0f,1.0f);
        surf2Dwrite(normal,normals,x*sizeof(float4),y);
    }else{
        _length=1.0f/_length;
        float4 normal=make_float4(cross.x*_length,cross.y*_length,cross.z*_length,1.0f);
        surf2Dwrite(normal,normals,x*sizeof(float4),y);
    }
}




void cudaCalcNormals(cudaSurfaceObject_t dStdMinStd, cudaSurfaceObject_t points,
                     cudaSurfaceObject_t normals,
                     int width, int height,float threshold){
    dim3 block(32,32);
    dim3 grid((width+block.x-1)/block.x,(height+block.y-1)/block.y);

    cudaCalcNormals_kernel<<<grid,block>>>(dStdMinStd,points,
                                           normals,width,height,threshold);

    cudaDeviceSynchronize();

}

__global__ void calcPoints_kernel(cudaSurfaceObject_t dStdMinStd, cudaSurfaceObject_t points,
                int width, int height, Eigen::Vector4f _fxycxy){

    int x=blockIdx.x * blockDim.x + threadIdx.x;
    int y=blockIdx.y * blockDim.y + threadIdx.y;
    if(x>=width || y>=height){
        return;
    }
    float4 depth;
    surf2Dread(&depth,dStdMinStd,x*sizeof(float4),y);

    float z=depth.x;
    float _fx=_fxycxy[0];
    float _fy=_fxycxy[1];
    float cx=_fxycxy[2];
    float cy=_fxycxy[3];
    float4 point;
    point.x=(float(x)-cx)*z*_fx;
    point.y=(float(y)-cy)*z*_fy;
    point.z=z;
    point.w = 1.0f;
    surf2Dwrite(point,points,x*sizeof(float4),y);

}


void cudaCalcPoints(cudaSurfaceObject_t dStdMinStd, cudaSurfaceObject_t points,
                int width, int height, Eigen::Vector4f fxycxy){
    dim3 block(32,32);
    dim3 grid((width+block.x-1)/block.x,(height+block.y-1)/block.y);
    Vector4f _fxycxy=fxycxy;
    _fxycxy[0] = 1.0f/fxycxy[0];//invert the last two elements so we don't have to do it in the kernel
    _fxycxy[1] = 1.0f/fxycxy[1];

    calcPoints_kernel<<<grid,block>>>(dStdMinStd,points,width,height,_fxycxy);

    cudaDeviceSynchronize();

}
