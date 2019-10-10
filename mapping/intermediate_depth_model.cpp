#include "intermediate_depth_model.h"

#include <cublas.h>
#include <cuda.h>
#include <cuda/reproject.h>

using namespace std;
using namespace Eigen;

IntermediateMap::IntermediateMap(int sensor_width, int sensor_height, 
                                 Vector4f depth_intrinsics) 
		: width_(sensor_width),
		  height_(sensor_height),
		  d_intrinsics_(depth_intrinsics) {

	cudaMalloc(&in_depth_,     sizeof(uint16_t) * width_ * height_);
	cudaMalloc(&out_depth_16_, sizeof(uint16_t) * width_ * height_);
	cudaMalloc(&out_depth_32_, sizeof(uint32_t) * width_ * height_);
}

IntermediateMap::~IntermediateMap() {
	cudaFree(in_depth_);
	cudaFree(out_depth_16_);
	cudaFree(out_depth_32_);
}

void IntermediateMap::setDepthMap(cv::Mat depth_16, Matrix4f pose_at_capture) {
	cam_pose_at_capture_ = pose_at_capture;
	cudaMemcpy(in_depth_, depth_16.data, sizeof(uint16_t) * width_ * height_, 
	           cudaMemcpyHostToDevice);
}

cv::Mat IntermediateMap::renderDepthMap(Matrix4f pose) {
	Matrix4f pose_transform = pose.inverse() * cam_pose_at_capture_;

	cudaMemset(out_depth_32_, 255, sizeof(uint32_t) * width_ * height_);
	reproject(in_depth_, out_depth_16_, out_depth_32_,  width_, height_,
	          pose_transform, d_intrinsics_);
	cv::Mat result(height_, width_, CV_16UC1);
	cudaMemcpy(result.data, out_depth_16_, sizeof(uint16_t) * width_ * height_,
	           cudaMemcpyDeviceToHost);

	return result;
}
