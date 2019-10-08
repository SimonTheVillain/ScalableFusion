#ifndef FILE_INTERMEDIATE_DEPTH_MODEL_H
#define FILE_INTERMEDIATE_DEPTH_MODEL_H

#include <opencv2/core.hpp>
#include <Eigen/Eigen>

using namespace Eigen;

/*
 * TODO: create an intermediate model of stuff that isn't put into the reconstruction
 * already. this should be used for rendering. and mainly camera tracking
 */

class IntermediateMap {
public:

	IntermediateMap(int width,int height, Vector4f d_intrinsics);
	~IntermediateMap();

	//maybe also add color
	void setDepthMap(cv::Mat depth16,Matrix4f pose_at_capture);

	cv::Mat renderDepthMap(Matrix4f pose);

private:

	int width_;
	int height_;

	// both buffers reside on gpu
	uint16_t *in_depth_;
	uint32_t *out_depth_32_;
	uint16_t *out_depth_16_;

	Vector4f d_intrinsics_;

	Matrix4f cam_pose_at_capture_;

};

#endif
