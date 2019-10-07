#ifndef FILE_CAMERA_H
#define FILE_CAMERA_H

#include <eigen3/Eigen/Eigen>
#include <opencv2/core.hpp>

using namespace Eigen;

class Camera {
public:

	static Matrix4f lookFromAt(Vector4f eye, Vector4f center,  Vector4f up);
	static Matrix4f projection(float fov_y, float aspect_ratio,  
	                           float z_far = 100.0f, float z_near = 0.01f);

	static Vector4f calcCamPosFromExtrinsic(Matrix4f cam);

	static Matrix4f genProjMatrix(Vector4f intrinsics);
	static Matrix4f genScaledProjMatrix(Vector4f intrinsics, cv::Size2i res);

};

#endif
