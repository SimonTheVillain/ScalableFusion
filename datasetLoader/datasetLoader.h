#ifndef FILE_DATASET_LOADER_H
#define FILE_DATASET_LOADER_H

#include <vector>
#include <chrono>

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <radical/radiometric_response.h>
#include <radical/vignetting_response.h>

using namespace std;
using namespace Eigen;

class Stream {
public:

	virtual bool hasGroundTruth() = 0;
	virtual bool isRunning() = 0;

	virtual void readNewSetOfImages() = 0;

	virtual cv::Mat  getDepthFrame() = 0;
	virtual Vector4f getDepthIntrinsics() = 0;
	virtual Matrix4f getDepthPose() = 0;

	virtual cv::Mat  getRgbFrame() = 0;
	virtual Vector4f getRgbIntrinsics() = 0;
	virtual Matrix4f getRgbPose() = 0;

	virtual Matrix4f getDepth2RgbRegistration() = 0;

	// TODO: add pose as quaternion + translation

};

class TumDataset : public Stream {
public:

	TumDataset(string folder, bool realtime = false, bool use_pose = false, 
	           bool use_high_res = true, int skip_n_frames = 0,
	           float depth_scale = 1.0f, float trajectory_GT_scale = 1.0f, 
	           bool invert_GT_trajectory = false);

	virtual ~TumDataset();

	bool hasGroundTruth();
	bool hasHighRes();	
	bool isRunning();

	void readNewSetOfImages();

	cv::Mat  getDepthFrame();
	Vector4f getDepthIntrinsics();
	Matrix4f getDepthPose();

	float    getRgbExposure();
	cv::Mat  getRgbFrame();
	Vector4f getRgbIntrinsics();
	Matrix4f getRgbPose();

	Matrix4f getDepth2RgbRegistration();

	float replay_speed;

	int skip_count;

private:

	//TODO: store the trajectory in these groundtruth files:
	struct TrajectoryPoint_{
		double timestamp;
		Matrix4f position;
	};

	vector<TrajectoryPoint_> trajectory_;

	int frame_index_;

	bool has_poses_;
	bool has_high_res_;
	bool read_depth_;
	bool read_rgb_;
	bool running_;

	string         folder_path_;
	vector<string> rgb_files_;
	vector<string> depth_files_;

	cv::Mat current_depth_;
	cv::Mat current_rgb_;

	Vector4f depth_intrinsics_;
	Vector4f rgb_intrinsics_;
	Matrix4f depth_2_rgb_;
	float    scale_depth_;

	//TODO: set this accordingly
	double                           current_timestamp_;
	vector<double>                   timestamps_;
	chrono::system_clock::time_point last_frame_readout_;

	vector<float> exposure_times_;
	float         rgb_exposure_time_;

	cv::Mat rgb_undistort_1_,
	        rgb_undistort_2_,
	        depth_undistort_1_,
	        depth_undistort_2_;

	radical::RadiometricResponse *radiometric_response_;
	radical::VignettingResponse  *vignetting_response_;

	cv::Vec3f white_fix_;

};

#endif
