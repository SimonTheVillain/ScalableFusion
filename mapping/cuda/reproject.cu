#include "reproject.h"
#include <stdio.h>


__global__
void reproject_kernel(uint16_t *depthIn,uint32_t *depthOut,
                      int width,int height,
                      Eigen::Matrix4f poseTransform,
                      Eigen::Vector4f depthIntrinsics){
    int x=blockIdx.x * blockDim.x + threadIdx.x;
    int y=blockIdx.y * blockDim.y + threadIdx.y;

    if(x>=width || y>=height){
        return;
    }

    //i think the depth is in millimeters
    //depth
    float fxD=depthIntrinsics[0];
    float fyD=depthIntrinsics[1];
    float cxD=depthIntrinsics[2];
    float cyD=depthIntrinsics[3];
    float z = float(depthIn[x+y*width])*0.001f;//from millimeter to meter

    if(z==0){
        return;
    }

    Eigen::Vector4f p((float(x)-cxD)*z/fxD,
                (float(y)-cyD)*z/fyD,
                z,
                1.0f);

    Eigen::Vector4f pnew= poseTransform*p;

    //now project:
    float zTarget=pnew[2];
    float xTarget=pnew[0]/zTarget*fxD+cxD;
    float yTarget=pnew[1]/zTarget*fyD+cyD;
    //printf(" w %f \n",pnew[3]);


    /*
    if(x==100 && y==100){
        printf("%f %f %f %f\n",p[0],p[1],p[2],p[3]);
        printf("%f %f %f %f\n",pnew[0],pnew[1],pnew[2],pnew[3]);
        printf("%f %f %f %f\n",fxD,fyD,cxD,cyD);
        printf("%f %f\n", (float(x)-cxD),(float(x)-cxD)*fxD);
    }
    */

    int xt=round(xTarget);
    int yt=round(yTarget);

    if(xt>=width || yt>=height ||
            xt<0 || yt<0){
        return;
    }
    atomicMin(&depthOut[xt+yt*width],
              uint32_t(zTarget*1000.0f));
    //depthOut[xt+yt*width] = uint16_t(zTarget*1000.0f);
}


__global__
void typecastAndMasking_kernel(uint16_t *depthOut16,uint32_t *depthOut32,
                      int width,int height){
    int x=blockIdx.x * blockDim.x + threadIdx.x;
    int y=blockIdx.y * blockDim.y + threadIdx.y;

    if(x>=width || y>=height){
        return;
    }

    uint32_t z = depthOut32[x +  y*width];
    if(z>65000){
        depthOut16[x +  y*width] = 0;
    }else{
        depthOut16[x +  y*width] = z;
    }

}

void reproject(uint16_t *depthIn,uint16_t *depthOut16,
               uint32_t *depthOut32,
               int width,int height,
               Eigen::Matrix4f poseTransform,
               Eigen::Vector4f depthIntrinsics){
    dim3 block(32,32);
    dim3 grid((width+block.x-1)/block.x,(height+block.y-1)/block.y);

    reproject_kernel<<<grid,block>>>(depthIn,depthOut32,
                                     width,height,
                                     poseTransform,
                                     depthIntrinsics);
    typecastAndMasking_kernel<<<grid,block>>>(  depthOut16,depthOut32,
                                                width,height);


}
