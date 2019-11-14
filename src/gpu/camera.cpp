#include <gpu/camera.h>

using namespace std;
using namespace Eigen;

// http://stackoverflow.com/questions/349050/calculating-a-lookat-matrix
// but i really do not want to think for myself
Matrix4f Camera::lookFromAt(Vector4f eye, Vector4f at, Vector4f up) {
	assert(0);//this is untested but might work

	Vector3f dir = (at - eye).block<3, 1>(0, 0).normalized();

	Quaternionf q1 = Quaternionf::FromTwoVectors(Vector3f(0, 0, -1), dir);

	Vector3f intermediate_up = q1 * Vector3f(0, 1, 0);
	Vector3f up_on_img_plane = up.block<3, 1>(0, 0) - dir * up.block<3, 1>(0, 0).dot(dir);
	
	Quaternionf q2  = Quaternionf::FromTwoVectors(intermediate_up, up_on_img_plane);
	Quaternionf q   = q2 * q1;
	Matrix3f    rot = q.normalized().toRotationMatrix();

	Matrix4f look_at = Matrix4f::Identity();
	look_at.block<3, 3>(0, 0) = rot;
	look_at.block<3, 1>(0, 3) = eye.block<3, 1>(0, 0);

	return look_at;
}

//http://www.songho.ca/opengl/gl_projectionmatrix.html
Matrix4f Camera::projection(float fov_y, float aspect_ratio, float z_far, 
                            float z_near) {
	float y_scale = cos(fov_y * 0.5f) / sin(fov_y * 0.5f);
	float x_scale = y_scale / aspect_ratio;
	float z_scale = -(z_far + z_near) / (z_far - z_near);

	Matrix4f proj;
	proj << x_scale,       0,       0,                                       0,
	              0, y_scale,       0,                                       0,
	              0,       0, z_scale, (-2 * z_near * z_far) / (z_far - z_near),
	              0,       0,      -1,                                       0;

	return proj;
}

Vector4f Camera::calcCamPosFromExtrinsic(Matrix4f cam) {
	Matrix3f r   = cam.block<3, 3>(0, 0);
	Vector3f t   = cam.block<3, 1>(0, 3);
	Vector3f pos = -r.transpose() * t;

	return Vector4f(pos[0], pos[1], pos[2], 1);
}

Matrix4f Camera::genProjMatrix(Vector4f intrinsics) {
	float fx = intrinsics[0];
	float fy = intrinsics[1];
	float cx = intrinsics[2];
	float cy = intrinsics[3];

	Matrix4f proj;
	proj << fx,  0, cx,  0,
	         0, fy, cy,  0,
	         0,  0,  0, -1,
	         0,  0,  1,  0;

	return proj;
}

Matrix4f Camera::genScaledProjMatrix(Vector4f intrinsics, cv::Size2i res) {
	float fx = intrinsics[0];
	float fy = intrinsics[1];
	float cx = intrinsics[2];
	float cy = intrinsics[3];

	Matrix4f proj1;
	proj1 << fx,  0, cx,  0,
	          0, fy, cy,  0,
	          0,  0,  0, -1,
	          0,  0,  1,  0;

	float w    = res.width;
	float h    = res.height;
	float zmin = 0.1f;
	float zmax = 30.0f;
	float b    = 2.0f / (1.0f / zmin - 1.0f / zmax);
	float a    = b / zmax + 1.0f;

	Matrix4f proj2; // the other one to scale everything to the normalized coordinates
	proj2 << 2.0f / w,        0, 0, -1.0f + 1.0f / w,
	                0, 2.0f / h, 0, -1.0f + 1.0f / h,
	                0,        0, b,                a,
	                0,        0, 0,                1;

	return proj2 * proj1;
}
