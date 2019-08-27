#include "intermediateDepthModel.h"

#include <cublas.h>
#include <cuda.h>
#include "cuda/reproject.h"

using namespace Eigen;
using namespace std;
using namespace cv;



IntermediateMap::IntermediateMap(int sensorWidth, int sensorHeight, Eigen::Vector4f depthIntrinsics)
{

    this->width=sensorWidth;
    this->height=sensorHeight;
    cudaMalloc(&inDepth,sizeof(uint16_t)*width*height);
    cudaMalloc(&outDepth16,sizeof(uint16_t)*width*height);
    cudaMalloc(&outDepth32,sizeof(uint32_t)*width*height);
    this->dIntrinsics = depthIntrinsics;

}

IntermediateMap::~IntermediateMap()
{
    cudaFree(inDepth);
    cudaFree(outDepth16);
    cudaFree(outDepth32);
}



void IntermediateMap::setDepthMap(cv::Mat depth16, Eigen::Matrix4f poseAtCapture)
{
    this->camPoseAtCapture = poseAtCapture;

    cudaMemcpy(inDepth,depth16.data,sizeof(uint16_t)*width*height,cudaMemcpyHostToDevice);
}

cv::Mat IntermediateMap::renderDepthMap(Eigen::Matrix4f pose)
{
    Eigen::Matrix4f poseTransform = pose.inverse()*camPoseAtCapture;

    cudaMemset(outDepth32,255,sizeof(uint32_t)*width*height);
    reproject(inDepth,
              outDepth16,outDepth32,
              width,height,
              poseTransform,
              dIntrinsics);
    Mat result(height,width,CV_16UC1);
    cudaMemcpy(result.data,outDepth16,sizeof(uint16_t)*width*height,cudaMemcpyDeviceToHost);
    return result;
}
