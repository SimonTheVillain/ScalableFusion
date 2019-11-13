//! Tool for accessing video sources.
/*! 
 *  Provides the tools for accessing various video sources like
 *  cameras and datasets.
 *
 * \file    source.h
 * \author  Nikolaus Wagner
 * \date    13. 11. 2019
 *
 */

#ifndef FILE_SOURCE_H
#define FILE_SOURCE_H

#include <vector>

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <radical/radiometric_response.h>
#include <radical/vignetting_response.h>

using namespace std;
using namespace Eigen;

//! Container for odometry information
/*!
 *  This container stores all the necessary odometry information, as well
 *  as utility functions for accessing them.
 */
struct Odometry {
	//! Extract the position for the current pose
	/*!
	 *  \return The 3x1 position vector
	 */
	inline Vector3f pos() {
		return pose.block<3, 1>(0, 3);
	}
	//! Extract the rotation for the current pose
	/*!
	 *  \return The 3x3 rotation matrix
	 */
	inline Matrix3f rot() {
		return pose.block<3, 3>(0, 0);
	}

	Matrix4f pose; //!< The current pose
};

//! Everything for doing basic video processing
namespace video {

//! Container for camera intrinsics
/*!
 *  This container can hold all the intrinsic parameter for a camera, as well
 *  as utility functions for accessing them.
 */
struct Intrinsics {

	inline void operator=(const Vector4f intrinsics) {
		fx = intrinsics[0];
		fy = intrinsics[1];
		cx = intrinsics[2];
		cy = intrinsics[3];
	}

	//! Convert the camera intrinsics to a 3x3 matrix
	/*!
	 *  \return The 3x3 matrix
	 */
	inline Matrix3f mat3() {
		Matrix3f mat;
		mat << fx,  0, cx,
		        0, fy, cy,
		        0,  0,  1;
		return mat;
	}
	//! Convert the camera intrinsics to a 4x4 matrix
	/*!
	 *  \return The 4x4 matrix
	 */
	inline Matrix4f mat4() {
		Matrix4f mat;
		mat << fx,  0, cx, 0,
		        0, fy, cy, 0,
		        0,  0,  1, 0,
		        0,  0,  0, 1;
		return mat;
	}

	float fx; //!< Focal length of the camera in the x-dimension
	float fy; //!< Focal length of the camera in the y-dimension
	float cx; //!< Principal point offset of the camera in the x-dimension
	float cy; //!< Principal point offset of the camera in the y-dimension
};

//! Container for frame data
/*! 
 *  This container can hold all possible data for a single frame as provided 
 *  by a video source of choice.
 */
struct Frame {
	cv::Mat rgb;         //!< The RGB image 
	cv::Mat depth;       //!< The depth image
	float exposure_time; //!< The exposure time of the RGB image
	double timestamp;    //!< The timestamp of the frame
};

//! Abstract base class for video sources
/*! 
 *  This is the base for all types of video sources. It implements everything
 *  used by datasets as well as cameras.
 */
class Source {
public:

	//! Read the next frame provided by the video source
	/*! 
	 *  This function will try to read a new frame from the video source, process
	 *  it and store it in #frame.
	 *
	 *  \return Whether or not reading the frame was successful
	 */
	bool readFrame();

	//! Whether or not the video source is currently providing frames
	inline bool isRunning() const {
		return is_running_;
	}
	//! Whether or not the video source provides RGB data
	inline bool providesRgb() const {
		return provides_rgb_;
	}
	//! Whether or not the video source provides depth data
	inline bool providesDepth() const {
		return provides_depth_;
	}
	//! Whether or not the video source provides exposure information
	inline bool providesExposure() const {
		return provides_exposure_;
	}
	//! Whether or not the video source provides exposure control
	inline bool providesExposureCtrl() const {
		return provides_exposure_ctrl_;
	}
	//! Whether or not the video source provides odometry information
	inline bool providesOdom() const {
		return provides_odom_;
	}

	Frame frame; //!< The most recent image data
	Odometry odom; //!< The most recent odometry data

protected:

	Source(bool provides_rgb = false, bool provides_depth = false, 
	       bool provides_exposure = false, bool provides_exposure_ctrl = false,
	       bool provides_odom = false) 
			: is_running_(false),
			  provides_rgb_(provides_rgb),
			  provides_depth_(provides_depth),
			  provides_exposure_(provides_exposure),
			  provides_exposure_ctrl_(provides_exposure_ctrl),
			  provides_odom_(provides_odom),
			  depth_to_rgb_(Matrix4f::Identity()),
			  radiometric_response_(nullptr),
			  vignetting_response_(nullptr) {

	}

	~Source() {
		delete radiometric_response_;
		delete vignetting_response_;
	}

	//! Read the RGB data for a frame
	/*!
	 *  This function will try to read the RGB data for the frame currently 
	 *  provided by the video source and store it to #frame.
	 *  The actual implementation of this function needs to be done in the
	 *  source-specific child classes.
	 *
	 *  \return Whether or not reading the RGB data was successful
	 */
	virtual bool readRgb_() = 0;	
	//! Read the depth data for a frame
	/*!
	 *  This function will try to read the depth data for the frame currently 
	 *  provided by the video source and store it to #frame.
	 *  The actual implementation of this function needs to be done in the
	 *  source-specific child classes.
	 *
	 *  \return Whether or not reading the depth data was successful
	 */
	virtual bool readDepth_() = 0;
	//! Read the exposure data for a frame
	/*!
	 *  This function will try to read the exposure data for the frame currently 
	 *  provided by the video source and store it to #frame.
	 *  The actual implementation of this function needs to be done in the
	 *  source-specific child classes.
	 *
	 *  \return Whether or not reading the exposure data was successful
	 */
	virtual bool readExposure_() = 0;
	//! Read the odometry data for a frame
	/*!
	 *  This function will try to read the odometry data for the frame currently 
	 *  provided by the video source and store it to #odom.
	 *  The actual implementation of this function needs to be done in the
	 *  source-specific child classes.
	 *
	 *  \return Whether or not reading the odometry data was successful
	 */
	virtual bool readOdom_() = 0;

	bool is_running_;             //!< Whether the video source is currently providing frames
	bool provides_rgb_;           //!< Whether the video source provides RGB data
	bool provides_depth_;         //!< Whether the video source provides depth data
	bool provides_exposure_;      //!< Whether the video source provides exposure information
	bool provides_exposure_ctrl_; //!< Whether the video source provides exposure control
	bool provides_odom_;          //!< Whether the video source provides odometry information

	Intrinsics intrinsics_rgb_;   //!< The intrinsics of the RGB camera
	Intrinsics intrinsics_depth_; //!< The intrinsics of the depth camera

	Matrix4f depth_to_rgb_; //!< The geometric transformation matrix from depth to rgb camera

	cv::Mat undistort_map_1;   //!< Image correction map 1/2 for rgb-frames from camera-calibration
	cv::Mat undistort_map_2;   //!< Image correction map 2/2 for rgb-frames from camera-calibration

	radical::RadiometricResponse *radiometric_response_; //!< Radiometric correction tool for rgb-frames from [radical](https://github.com/taketwo/radical)
	radical::VignettingResponse  *vignetting_response_;  //!< Vignetting correction tool for rgb-frames from [radical](https://github.com/taketwo/radical)
};

//! Abstract base class for all types of cameras
/*!
 *  This class provides the common base for all types of cameras. Specific
 *  read-/control-functions and configurations need to be implemented in the 
 *  camera-specific, derived subclasses.
 */
class Camera : public Source {
public:

	//! Set the desired exposure that the camera shall provide
	virtual bool setExposure(float exposure) = 0;

protected:

};

//! Abstract base class for all types of video datasets
/*!
 *  This class provides the common base for all types of datasets.
 *  Specific read-functions and configurations need to be implemented in the
 *  dataset-specific, derived subclasses.
 */
class Dataset : public Source {
public:

	//! Dataset-specific extension of Source::readFrame()
	bool readFrame();

protected:

	Dataset(string source_dir = "", bool provides_rgb = false, 
	        bool provides_depth = false, bool provides_exposure = false, 
	        bool provides_exposure_ctrl = false, bool provides_odom = false)
			: Source(provides_rgb, provides_depth, provides_exposure, 
			         provides_exposure_ctrl, provides_odom),
			  source_dir_(source_dir),
			  frame_idx_(0) {
	}

	//! Implementation of Source::readRgb_() 
	bool readRgb_() {
		frame.rgb = cv::imread(filenames_rgb_[frame_idx_]);
		return (frame.rgb.data == nullptr) ? false : true;
	}
	//! Implementation of Source::readDepth_()
	bool readDepth_() {
		frame.depth = cv::imread(filenames_depth_[frame_idx_]);
		return (frame.depth.data == nullptr) ? false : true;
	}
	//! Implementation of Source::readExposure_()
	bool readExposure_() {
		frame.exposure_time = exposure_times_[frame_idx_];
		return true;
	}
	//! Implementation of Source::readOdom_()
	bool readOdom_() {
		odom.pose = poses_[frame_idx_];
		return true;
	}

	string source_dir_; //!< The storage location of the dataset

	vector<string>   filenames_rgb_;   //!< The paths to the files containing the RGB images
	vector<string>   filenames_depth_; //!< The paths to the files containing the depth images
	vector<double>   timestamps_;      //!< The timestamps of the individual frames
	vector<float>    exposure_times_;  //!< The exposure times of the RGB frames
	vector<Matrix4f> poses_;           //!< The camera poses corresponding to the individual frames 

	unsigned int frame_idx_; //!< The index of the next frame that will be read
};

//! Specification for the TU München Datasets
/*!
 *  This class provides the proper construction and initialization for the
 *  TUM-datasets, so that they can be accessed with the common Dataset interface.
 */
class TumDataset : public Dataset {
public:
	TumDataset(string source_dir, bool is_high_res);

protected:
	bool is_high_res_; //!< Whether the dataset provides High-Res-images
};

//! Specification for the TU München - alike Datasets
/*!
 *  This class is an extension to the TumDataset, which behaves the same way
 *  but with different in- & extrinsics
 */
class TuwDataset : public TumDataset {
public: 
	TuwDataset(string source_dir, bool is_high_res);
};

} // namespace video

#endif // FILE_SOURCE_H