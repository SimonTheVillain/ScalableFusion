#ifndef FILE_DATASET_LOADER_H
#define FILE_DATASET_LOADER_H

#include <vector>
#include <chrono>

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <radical/radiometric_response.h>
#include <radical/vignetting_response.h>

class Stream {
public:

	virtual bool isRunning() = 0;
	virtual cv::Mat getDepthFrame() = 0;
	virtual cv::Mat getRGBFrame() = 0;
	virtual Eigen::Vector4f getDepthIntrinsics() = 0;
	virtual Eigen::Vector4f getRgbIntrinsics() = 0;
	virtual Eigen::Matrix4f getDepth2RgbRegistration() = 0;
	virtual bool hasGroundTruth() = 0;
	virtual Eigen::Matrix4f getDepthPose() = 0;
	virtual Eigen::Matrix4f getRgbPose() = 0;
	virtual void readNewSetOfImages() = 0;
	//TODO: add pose as quaternion + translation

};

class TumDataset : public Stream {
public:

	TumDataset(std::string folder, bool realtime = false, bool use_pose = false, 
	           bool use_high_res = true, int skip_n_frames = 0,
	           float depth_scale = 1.0f, float trajectory_GT_scale = 1.0f, 
	           bool invert_GT_trajectory = false);

	virtual ~TumDataset();

	bool isRunning();

	void readNewSetOfImages();
	cv::Mat getDepthFrame();
	cv::Mat getRGBFrame();
	float getRGBExposure();
	bool hasHighRes();
	Eigen::Vector4f getDepthIntrinsics();
	Eigen::Vector4f getRgbIntrinsics();
	Eigen::Matrix4f getDepth2RgbRegistration();
	bool hasGroundTruth();
	Eigen::Matrix4f getDepthPose();
	Eigen::Matrix4f getRgbPose();

	float replay_speed;
	bool realtime;//it anyway is not implemented yet.
	int skip_count;

private:

	//TODO: store the trajectory in these groundtruth files:
	struct TrajectoryPoint {
		double timestamp;
		Eigen::Matrix4f position;
	};

	void loadIntrinsics();

	int frameshift_RGB_;//seemingly this is not an issue for the new setup
	bool read_depth_;
	bool read_RGB_;
	bool running_;
	std::string folder_path_;
	cv::Mat current_depth_;
	cv::Mat current_RGB_;

	Eigen::Vector4f depth_intrinsics_;
	Eigen::Vector4f rgb_intrinsics_;
	Eigen::Matrix4f depth_2_RGB_;
	float scale_depth_;

	//TODO: set this accordingly
	double current_timestamp_;

	std::vector<TrajectoryPoint> trajectory_;


	std::chrono::system_clock::time_point last_frame_readout_;


	bool has_poses_;
	bool has_high_res_;

	//cv::Mat rgbFundamental, depthFundamental;
	cv::Mat rgb_undistort_1_,
	        rgb_undistort_2_,
	        depth_undistort_1_,
	        depth_undistort_2_;



	radical::RadiometricResponse* radiometric_response_;
	radical::VignettingResponse* vignetting_response_;

	int frame_index_;
	std::vector<double> timestamps_;
	std::vector<std::string> rgb_files_;
	std::vector<std::string> depth_files_;
	std::vector<float> exposure_times_;
	float rgb_exposure_time_;
	cv::Vec3f white_fix_;

};

/*
class GeorgDataset : public Stream{
//TODO: all of this
};
*/

#endif
