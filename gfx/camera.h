#ifndef FILE_CAMERA_H
#define FILE_CAMERA_H

#include <eigen3/Eigen/Eigen>
#include <opencv2/core.hpp>

class Camera {
public:

	static Eigen::Matrix4f lookFromAt(Eigen::Vector4f eye, Eigen::Vector4f center, 
	                                  Eigen::Vector4f up);
	static Eigen::Matrix4f projection(float fov_y, float aspect_ratio, 
	                                  float z_far=100.0f, float z_near=0.01f);

	static Eigen::Vector4f calcCamPosFromExtrinsic(Eigen::Matrix4f cam);

	static Eigen::Matrix4f genProjMatrix(Eigen::Vector4f intrinsics);
	static Eigen::Matrix4f genScaledProjMatrix(Eigen::Vector4f intrinsics,
	                                           cv::Size2i res);

};

#endif
