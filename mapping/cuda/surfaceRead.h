#ifndef FILE_SURFACE_READ_H
#define FILE_SURFACE_READ_H


#include <Eigen/Eigen>
#include <iostream>
#include <stdio.h>
#include <assert.h>
#include "helper_math.h"
#include "gpuErrchk.h"
#include "geomUpdate.h"

#ifdef __CUDACC__

__device__ inline float2 unnormalizeTexCoords(float2 in,
                                              cv::Rect2i &rect){
    return make_float2(in.x*float(rect.width-1)+rect.x,
                       in.y*float(rect.height-1)+rect.y);

}
/*
__device__ inline float4 gpu::readFloat4FromAtlas(int x, int y,
                                             gpu::TexAtlasSubSurface surface){

}
*/
__device__ inline float4 readFloat4FromAtlas(float2 pos,
                                            gpu::TexAtlasSubSurface surface){
    /*if(x<0 || y<0 || x>1.0 || y>1.0){
        return make_float4(NAN,NAN,NAN,NAN);
    }

    float X = x*(surface.rect.width-1)-0.5;
    float Y = y*(surface.rect.height-1)-0.5;
    */

}

//TODO: adapt this for texAtlas
__device__ inline float4 readSensor(float x,float y, const cudaSurfaceObject_t surface,int width,int height,float threshold){

    // with the new version lets just take the closest:

    int xClosest = x-0.5f;
    int yClosest = y-0.5f;


    if(xClosest<0 || yClosest<0 || xClosest>=width || yClosest>=height){
        return make_float4(NAN,NAN,NAN,NAN);
    }
    float4 closest;
    surf2Dread(&closest,surface,int(xClosest)*sizeof(float4),int(yClosest));

    //return closest;
    //This is the old version that interpolates unconditionally!
    //which is wrong of course!


    //there is a lot that plays into this
    //think about it simon
    if(x<0 || y<0 || x>(width-1) || y>(height-1)){
         if(x==0 && blockIdx.x==0){
             //printf("shit are we out of bounds? %f of %d \n",y,height);
         }
        return make_float4(NAN,NAN,NAN,NAN);
    }
    float X = x-float(int(x));
    float Y = y-float(int(y));

    if(X==0 && int(x)==width-1){
        X=1.0f;
        x=x-1.0f;
    }

    if(Y==0 && int(y)==height-1){
        Y=1.0f;
        y=y-1.0f;
    }


    //bottom:
    float4 Q00;
    surf2Dread(&Q00,surface,int(x)*sizeof(float4),int(y));
    float4 Q01;
    surf2Dread(&Q01,surface,(int(x)+1)*sizeof(float4),int(y));
    float4 Q0x=X*Q01 + (1.0f-X)*Q00;

    //top
    float4 Q10;
    surf2Dread(&Q10,surface,(int(x))*sizeof(float4),int(y)+1);
    float4 Q11;
    surf2Dread(&Q11,surface,(int(x))*sizeof(float4),int(y)+1);
    float4 Q1x=X*Q11 + (1.0f-X)*Q10;

    if(abs(Q00.x-Q10.x)>threshold ||
            abs(Q01.x-Q11.x)>threshold ||
            abs(Q00.x-Q01.x)>threshold){
        //in case all the points are close in depth we interpolate
        // if not we just put out the pixel to this sample position
        return closest;
    }
    //mix the middle;
    float4 Qyx = Y*Q1x + (1.0f-Y)*Q0x;

    return Qyx;


}


/**
 * @brief readBilinear
 * This is a function to read interpolated pixel from a surface object,
 * The pixel center are at full integer positions in comparison to the
 * cuda texture objects.
 * @param x
 * @param y
 * @param surface
 * @param width
 * @param height
 * @return
 */
//TODO: adapt this for texture atlas
__device__ inline float4 readBilinear(float x,float y, const cudaSurfaceObject_t surface, int width, int height){

    if(x<0 || y<0 || x>(width-1) || y>(height-1)){
         if(x==0 && blockIdx.x==0){
             //printf("shit are we out of bounds? %f of %d \n",y,height);
         }
        return make_float4(NAN,NAN,NAN,NAN);
    }
    float X = x-float(int(x));
    float Y = y-float(int(y));

    //in the edge case of being on the upper bound we change the composition of the
    //pixel coordinates a little
    //changing from [) to [] .... you know what i mean. (hopefully)
    if(X==0 && int(x)==width-1){
        X=1.0f;
        x=x-1.0f;
    }

    if(Y==0 && int(y)==height-1){
        Y=1.0f;
        y=y-1.0f;
    }

    //bottom:
    float4 Q00;
    surf2Dread(&Q00,surface,int(x)*sizeof(float4),int(y));
    float4 Q01;
    surf2Dread(&Q01,surface,(int(x)+1)*sizeof(float4),int(y));
    float4 Q0x=X*Q01 + (1.0f-X)*Q00;

    //top
    float4 Q10;
    surf2Dread(&Q10,surface,(int(x))*sizeof(float4),int(y)+1);
    float4 Q11;
    surf2Dread(&Q11,surface,(int(x)+1)*sizeof(float4),int(y)+1);
    float4 Q1x=X*Q11 + (1.0f-X)*Q10;

    //mix the middle;
    float4 Qyx = Y*Q1x + (1.0f-Y)*Q0x;

    return Qyx;


}

__device__ inline float4 readFloat4F16(const cudaSurfaceObject_t surface,int x,int y){
    __half data[4];
    surf2Dread((ushort4*)data,surface,x*sizeof(ushort4),y);
    float4 output=make_float4(__half2float(data[0]),
                              __half2float(data[1]),
                              __half2float(data[2]),
                              __half2float(data[3]));
    return output;

}
/*
__device__ inline void writeFloat4F16(float4 in,cudaSurfaceObject_t surface,int x,int y){
    ushort4 data = make_ushort4(__float2half_rn(in.x),
                                __float2half_rn(in.y),
                                __float2half_rn(in.z),
                                __float2half_rn(in.w));
    surf2Dwrite(data,surface,x*sizeof(ushort4),y);
}
 */

//TODO: adapt this for texture atlas
__device__ inline float4 readBilinear16F(float x,float y, const cudaSurfaceObject_t surface, int width, int height){

    if(x<0 || y<0 || x>(width-1) || y>(height-1)){
         if(x==0 && blockIdx.x==0){
             //printf("shit are we out of bounds? %f of %d \n",y,height);
         }
        return make_float4(NAN,NAN,NAN,NAN);
    }
    float X = x-float(int(x));
    float Y = y-float(int(y));

    //in the edge case of being on the upper bound we change the composition of the
    //pixel coordinates a little
    //changing from [) to [] .... you know what i mean. (hopefully)
    if(X==0 && int(x)==width-1){
        X=1.0f;
        x=x-1.0f;
    }

    if(Y==0 && int(y)==height-1){
        Y=1.0f;
        y=y-1.0f;
    }

    //bottom:
    float4 Q00 = readFloat4F16(surface,x,y);
    //surf2Dread(&Q00,surface,int(x)*sizeof(float4),int(y));
    float4 Q01 = readFloat4F16(surface,x+1,y);
    //surf2Dread(&Q01,surface,(int(x)+1)*sizeof(float4),int(y));
    float4 Q0x=X*Q01 + (1.0f-X)*Q00;

    //top
    float4 Q10 = readFloat4F16(surface,x,y+1);;
    //surf2Dread(&Q10,surface,(int(x))*sizeof(float4),int(y)+1);
    float4 Q11 = readFloat4F16(surface,x+1,y+1);;
    //surf2Dread(&Q11,surface,(int(x))*sizeof(float4),int(y)+1);
    float4 Q1x=X*Q11 + (1.0f-X)*Q10;

    //mix the middle;
    float4 Qyx = Y*Q1x + (1.0f-Y)*Q0x;

    return Qyx;


}


/**
 * @brief readBilinearGLStyle
 * opengl style texture sampling is using normalized coordinates. The pixel centers are
 * @param x
 * @param y
 * @param surface
 * @param width
 * @param height
 * @return
 */
//TODO: adapt this for the texture atlas
__device__ inline float4 readBilinearGLStyle(float x,float y, const cudaSurfaceObject_t surface,int width,int height){

    x=x*float(width-1)-0.5f;
    y=y*float(height-1)-0.5f;

    //TODO: do some simple thresholding
    x=fmin(fmax(x,0),width-1);
    y=fmin(fmax(y,0),height-1);
    return readBilinear(x,y,
                        surface,
                        width,height);
}



/*__device__ Vector4f updateGeomTex(Vector4f pointInSpace,Vector4f oldTexValue){

}*/

__device__ inline float4 calcSurfaceUpdate(float4 surface_k,float4 sensor, //the vector of sensor data and of what is on the surface
                                      float d, float dUp){

    float epsilon = 0.001; // if sigma comes this close to
    float dev_k = surface_k.x;

    //deviation as it is perceived to this surface right now
    float dev_s = d-sensor.x;

    //the minimum possible standard deviation
    float s_m = min(sensor.z,surface_k.z);
    float s_k = surface_k.y;
    float s_s = sensor.y;

    //standard deviation to the lower bound of standard deviation
    float s_s_ = s_s-s_m;
    float s_k_ = s_k-s_m;

    float s_k1_ = s_k_*s_s_/(s_k_+s_s_);
    if(s_k1_ < epsilon){
        s_k1_=0; //set the standard deviation update
        //to zero if it is too close to zero
    }
    float dev_k1=(dev_s/s_s+dev_k/s_k_)*s_k1_;

    float4 surface_k1;
    surface_k1.x = dev_k1-(d-dUp);
    //surface_k1.x=0;//debug
    surface_k1.y = s_k1_+s_m;
    surface_k1.z = s_m;
    surface_k1.w = surface_k.w;

    return surface_k1;
}

/*
__device__ inline bool updateVertex(FullGpuVertex &vertex_k,
                                      const cudaSurfaceObject_t &sensorInput,
                                      int sensorWidth,int sensorHeight,
                                      Eigen::Vector4f camPos,
                                      Eigen::Matrix4f _pose,
                                      Eigen::Matrix4f proj_pose,
                                      UpdateDescriptor &descriptor){
    Eigen::Vector4f pos(vertex_k.p.x,
                 vertex_k.p.y,
                 vertex_k.p.z,
                 1.0f);
    Eigen::Vector4f toCamera = camPos-pos;//TODO: we need the camera position
    Eigen::Vector3f toCamN = toCamera.block<3,1>(0,0)*1.0f/toCamera.block<3,1>(0,0).norm();
    float3 front = make_float3(toCamN[0],toCamN[1],toCamN[2]);

    if(blockIdx.x==1 && false){//somehow the first element on the surface is NAN
        printf("camPos = %f %f %f %f\n"
               "pos: %f %f %f %f\n"
               "toCamN %f %f %f\n"
               "norm %f \n",
               camPos[0],camPos[1],camPos[2],camPos[3],
               pos[0],pos[1],pos[2],pos[3],
               toCamN[0],toCamN[1],toCamN[2],
               toCamN.block<3,1>(0,0).norm());
    }
    //toCamN[3]=1;//only if we really need this
    Eigen::Vector4f posCamFrame = _pose*pos;

    //read out sensor data.
    Eigen::Vector4f texCoord = proj_pose*pos;

    float u=texCoord[0]/texCoord[3];
    float v=texCoord[1]/texCoord[3];


    //read depth and co on this coordinate.
    float4 sensor =
            readInterpolated(u,v,
                             sensorInput,
                             sensorWidth,sensorHeight,0.1);//threshold =0.1
    if(blockIdx.x==1 && false){//this seems to be ok
        printf("the position on the sensor iiis %f %f\n"
               "and its value: %f %f %f %f\n",
               u,v,
               sensor.x,sensor.y,sensor.z,sensor.w);
    }

    //extract the depth from the readout data
    float d = posCamFrame[2];
    float d_s = sensor.x;
    if(d>(d_s+0.05) || d<(d_s-0.05)){
        //TODO: make these thresholds more dependant on the noise. (and or the normal)
        //if the depth is bigger than the depth of the sensor + some threshold
        //we do not continue due to being occluded.
        return false;

    }


    //adapt texture coordinate to atlas
    float2 texAtlasCoord = make_float2(
                vertex_k.texP.x*descriptor.sourceN.width+descriptor.sourceN.x,
                vertex_k.texP.y*descriptor.sourceN.height+descriptor.sourceN.y);

    //read out surface data.
    float4 surface_k=
            readBilinearGLStyle(texAtlasCoord.x,texAtlasCoord.y,
                                descriptor.sourceGeometry,
                                descriptor.sourceSize.width,descriptor.sourceSize.height);



    if(blockIdx.x==1 &&false){//somehow the first element on the surface is NAN
        printf("the position on the surface iiis %f %f\n"
               "and its value: %f %f %f %f\n",
               vertex_k.texP.x,vertex_k.texP.y,
               surface_k.x,surface_k.y,surface_k.z,surface_k.w);
    }
    float dev_s=d-d_s;

    //the minimum possible standard deviation
    float s_m = min(sensor.z,surface_k.z);
    float s_k = surface_k.y;
    float s_s = sensor.y;

    //standard deviation to the lower bound of standard deviation
    float s_s_ = s_s-s_m;
    float s_k_ = s_k-s_m;

    //float s_k1_ = s_k_*s_s_/(s_k_+s_s_);

    //the deviation of the current estimate from the surface!!!
    //float dev_k1=(dev_s/s_s+dev_k/s_k_)*s_k1_;
    //reduced:
    float dev_k1=dev_s*s_k_/(s_k_+s_s_);
    if(isnan(dev_k1) || isinf(dev_k1)){
        return false;
    }
    //now update the surface with the vertex data
    vertex_k.p = vertex_k.p + dev_k1*front;


    if(blockIdx.x==1 &&false){//somehow the first element on the surface is NAN
        printf("dev_k1 = %f\n"
               "and the direction to the camera: %f %f %f\n",
               vertex_k.texP.x,vertex_k.texP.y,
               front.x,front.y,front.z);
    }

    return true;


}
*/

#endif

#endif
