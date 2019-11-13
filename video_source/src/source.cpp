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

#include <source.h>

#include <fstream>

#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

bool video::Source::readFrame() {
	if(!is_running_) {
		return false;
	}
	bool success = true;
	if(provides_rgb_) {
		success &= readRgb_();
		// Improve RGB image
		if(radiometric_response_ != nullptr && vignetting_response_ != nullptr) {
			cv::Mat irradiance, radiance;
			radiometric_response_->inverseMap(frame.rgb, irradiance);
			vignetting_response_->remove(irradiance, radiance);
			radiance *= 1.08;
			for(int i = 0; i < radiance.size().area(); i++) {
				cv::Vec3f v = radiance.at<cv::Vec3f>(i);
				radiance.at<cv::Vec3f>(i) = v;
			}
			radiometric_response_->directMap(radiance, frame.rgb);
		}
		if(!undistort_map_1.empty()) {
			cv::remap(frame.rgb, frame.rgb, undistort_map_1, undistort_map_2, 
			          cv::INTER_LINEAR);
		}
	}
	if(provides_depth_) {
		success &= readDepth_();
	}
	if(provides_exposure_) {
		success &= readExposure_();
	}
	if(provides_odom_) {
		success &= readOdom_();
	}
	return success;
}

bool video::Dataset::readFrame() {
	if(!is_running_) {
		return false;
	}
	// Do generic reading operations
	if(!Source::readFrame()) {
		return false;
	}
	// Do dataset specific reading operations
	frame.timestamp = timestamps_[frame_idx_];
	frame_idx_++;
	// Check if dataset contains any more data
	if(frame_idx_ >= filenames_rgb_.size()) {
		is_running_ = false;
	}
	return true;
}

video::TumDataset::TumDataset(string source_dir, bool is_high_res)
		: Dataset(source_dir, true, true, false, false, false),
		  is_high_res_(is_high_res) {

	// Read RGB-Depth-associations from file:
	// - Open file with assocations and read line by line
	ifstream file_associations;
	is_high_res_ ? file_associations.open(source_dir + "/associations_ids.txt") :
	               file_associations.open(source_dir + "/associations.txt");
	if(!file_associations.is_open()) {
		cout << "Could not open dataset (no associations file found)" << endl;
		assert(0);
	}

	string line;
	while(getline(file_associations, line)) {
		istringstream line_ss(line);
		string word;

		// - Read first timestamp
		getline(line_ss, word, ' ');
		double timestamp = atof(word.c_str());
		timestamps_.push_back(timestamp);

		// - Read location of depth frame
		getline(line_ss, word, ' ');
		string depth_file = source_dir + "/" + word;
		filenames_depth_.push_back(depth_file);

		// - Skip second timestamp
		getline(line_ss, word, ' ');

		// - Read location of RGB frame
		getline(line_ss, word, ' ');
		string rgb_file = source_dir + "/" + word;
		filenames_rgb_.push_back(rgb_file);
	}

	if(!filenames_rgb_.empty()) {
		is_running_ = true;
	}	else {
		cout << "Could not read frame associations" << endl;
		assert(0);
	}

	// Try reading odometry from file
	// - Buffer all poses and extract the correct one for each frame later
	vector<pair<Matrix4f, double>> poses_buff;
	ifstream file_odometry(source_dir_ + "/groundtruth.txt"); 
	if(file_odometry.is_open()) {
		string line;
		while(getline(file_odometry, line)) {
			// - Skip header
			if(line[0] != '#') {
				pair<Matrix4f, double> pose_buff;
				float x, y, z;
				float qx, qy, qz, qw;
				sscanf(line.c_str(), "%lf %f %f %f %f %f %f %f", 
				       &pose_buff.second, &x, &y, &z, &qx, &qy, &qz, &qw);

				Affine3f transform(Translation3f(x, y, z));
				Matrix4f t = transform.matrix();

				// - Create homogenous rotation matrix
				Quaternionf q(qw, qx, qy, qz);
				Matrix4f r = Matrix4f::Identity();
				r.block<3, 3>(0, 0) = q.toRotationMatrix();

				pose_buff.first  = t * r;

				poses_buff.push_back(pose_buff);
			}
		}
	}
	// - If dataset provides odometry, extract the one single best fitting pose 
	//   for each frame given the timestamps of frames and poses
	if(!poses_buff.empty()) {
		provides_odom_ = true;
		poses_ = vector<Matrix4f>(filenames_rgb_.size());
		for(int i = 0; i < poses_.size(); i++) {
			double delta_time_min = 1000;
			for(int j = 0; j < poses_buff.size(); j++) {
				if(fabs(timestamps_[i] - poses_buff[j].second) < delta_time_min) {
					poses_[i] = poses_buff[j].first;
					delta_time_min = fabs(timestamps_[i] - poses_buff[j].second);
				}
			}
		}
	}

	// TUM intrinsics
	intrinsics_rgb_   = Vector4f(535.4, 539.2, 320.1, 247.6);
	intrinsics_depth_ = intrinsics_rgb_;

}

video::TuwDataset::TuwDataset(string source_dir, bool is_high_res) 
		: TumDataset(source_dir, is_high_res) {

	ifstream file_exposure;
	is_high_res_ ? file_exposure.open(source_dir_ + "/rgb_ids_exposure.txt") :
	               file_exposure.open(source_dir_ + "/rgb_exposure.txt");
	if(!file_exposure.is_open()) {
		cout << "Could not open open exposure file" << endl;
		assert(0);
	}
	string line;
	while(getline(file_exposure, line)) {
		exposure_times_.push_back(atof(line.c_str()));
		// TODO: Sort exposure times and assign to correct frames
	}

	if(is_high_res_) {
		cv::Size size;
		cv::Mat M;
		cv::Mat D;
		cv::FileStorage file_calib(source_dir + "/../calib_result_ids.yml", 
		                           cv::FileStorage::READ);
		if(!file_calib.isOpened()) {
			assert(0);
		}
		file_calib["camera_matrix"]           >> M;
		file_calib["distortion_coefficients"] >> D;
		file_calib["image_width"]             >> size.width;
		file_calib["image_height"]            >> size.height;

		cv::initUndistortRectifyMap(M, D, cv::Mat(), M, size, CV_16SC2, 
		                            undistort_map_1, undistort_map_2);
		intrinsics_rgb_ = Vector4f(M.at<double>(0, 0), 
		                           M.at<double>(1, 1),
		                           M.at<double>(0, 2), 
		                           M.at<double>(1, 2));
	} else {
		intrinsics_rgb_ = Vector4f(530, 530, 320, 240);
	}
	intrinsics_depth_ = Vector4f(568, 568, 320, 240); // the structure sensor
	//intrinsics_depth_ = Vector4f(570, 570, 320, 240); // xtion

	if(is_high_res) {
		// TRACK 16 - 19 should work with these settings:
		// Tweaking of the calibration because the camera rack is not rigid
		Matrix3f rot = (AngleAxisf(0.010 * M_PI, Vector3f::UnitX()) *
		                AngleAxisf(0.002 * M_PI, Vector3f::UnitY()) *
		                AngleAxisf(0.000 * M_PI, Vector3f::UnitZ())
		                ).normalized().toRotationMatrix();
		Matrix4f rot41 = Matrix4f::Identity();
		rot41.block<3, 3>(0, 0) = rot;

		cv::FileStorage file_extrinsics(source_dir_ + "/../extrinsics.yml", 
		                                cv::FileStorage::READ);

		cv::Mat R, T;
		file_extrinsics["R"] >> R;
		file_extrinsics["T"] >> T;
		R.convertTo(R, CV_32FC1);
		T.convertTo(T, CV_32FC1);
		Matrix3f eR(reinterpret_cast<float*>(R.data));
		Vector3f eT(reinterpret_cast<float*>(T.data));

		Matrix4f rot4 = Matrix4f::Identity();
		rot4.block<3, 3>(0, 0) = eR;
		rot4.block<3, 1>(0, 3) = eT;

		depth_to_rgb_ = rot41 * rot4;

	} else {
		depth_to_rgb_ = Matrix4f::Identity();
		depth_to_rgb_(0, 3) = 0.026f; // Standard xtion baseline
	}

	// Try reading radiometric and vignetting correctors
	if(is_high_res_) {
		radiometric_response_ = new radical::RadiometricResponse(source_dir_ + "/../rgb_ids.crf");
		vignetting_response_  = new radical::VignettingResponse(source_dir_ + "/../rgb_ids.vgn");
	} else {
		radiometric_response_ = new radical::RadiometricResponse(source_dir_ + "/../rgb.crf");
		vignetting_response_  = new radical::VignettingResponse(source_dir_ + "/../rgb.vgn");
	}

}