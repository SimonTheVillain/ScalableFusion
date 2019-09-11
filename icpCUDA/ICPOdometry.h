#ifndef FILE_ICPODOMETRY_H_
#define FILE_ICPODOMETRY_H_

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "Cuda/internal.h"

using namespace std;

class ICPOdometry {
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ICPOdometry(int width, int height, float cx, float cy, float fx, float fy,
	            float distThresh = 0.10f, 
	            float angleThresh = sinf(20.f * 3.14159254f / 180.f));

	virtual ~ICPOdometry();

	void initICP(unsigned short * depth, const float depthCutoff = 20.0f);

	void initICPModel(unsigned short * depth, const float depthCutoff = 20.0f);

	void getIncrementalTransformation(Sophus::SE3d & T_prev_curr, int threads, 
	                                  int blocks);

	float last_error;
	float last_inliers;

private:

	static const int NUM_PYRS_ = 3;

	vector<DeviceArray2D<unsigned short>> depth_tmp_;

	vector<DeviceArray2D<float>> vmaps_prev_;
	vector<DeviceArray2D<float>> nmaps_prev_;
	vector<DeviceArray2D<float>> vmaps_curr_;
	vector<DeviceArray2D<float>> nmaps_curr_;

	Intr intr_;

	DeviceArray<Eigen::Matrix<float, 29, 1, Eigen::DontAlign>> sum_data_;
	DeviceArray<Eigen::Matrix<float, 29, 1, Eigen::DontAlign>> out_data_;


	vector<int> iterations_;

	float dist_thresh_;
	float angle_thresh_;

	const int width_;
	const int height_;
};

#endif
