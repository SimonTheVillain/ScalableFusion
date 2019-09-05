#include "DatasetLoader.h"

#include <eigen3/Eigen/Geometry>
#include <thread>

#include <fstream>

using namespace Eigen;
using namespace std;
using namespace cv;

TumDataset::TumDataset(std::string folder, bool realtime, bool use_pose,
                       bool use_high_res,int skip_n_frames, float depth_scale,
                       float trajectory_GT_scale,bool invert_GT_trajectory) 
	: folder_path_(folder),
	  frame_index_(skip_n_frames),
	  frameshift_RGB_(0),
	  has_high_res_(false),
	  has_poses_(false),
	  radiometric_response_(nullptr),
	  read_depth_(false),
	  read_RGB_(false),
	  replay_speed(0),
	  running_(false),
	  scale_depth_(depth_scale),
	  skip_count(20),
	  vignetting_response_(nullptr) {

	//TODO: try to implement try catch pattern here, maybe it is useful

	ifstream frame_list;
	string file_name_2_open;
	if(use_high_res) {
		file_name_2_open = folder + "/associations_ids.txt";
		frame_list.open(file_name_2_open);
		if(frame_list.is_open()) {
			has_high_res_ = true;
		}
	}
	if(!has_high_res_) {
		file_name_2_open = folder + "/associations.txt";
		frame_list.open(file_name_2_open);
		if(!frame_list.is_open()) {
			cout << "Couldn't open dataset (no associations.txt file found)" << endl;
			assert(0);//
		}
	}


	FileStorage fs_white_fix_;
	if(has_high_res_) {
		fs_white_fix_.open(folder + "/white_fix__ids.yml", cv::FileStorage::READ);
	} else {
		fs_white_fix_.open(folder + "/white_fix_.yml", cv::FileStorage::READ);
	}
	if(fs_white_fix_.isOpened()) {
		fs_white_fix_["r_gain"] >> white_fix_[2];
		fs_white_fix_["g_gain"] >> white_fix_[1];
		fs_white_fix_["b_gain"] >> white_fix_[0];
	} else {
		white_fix_ = cv::Vec3f(1, 1, 1);
	}


	string file_locations;
	while(std::getline(frame_list, file_locations)) {
		stringstream just_for_the_timestamp(file_locations);
		double timestamp;
		just_for_the_timestamp >> timestamp;
		timestamps_.push_back(timestamp);

		istringstream line(file_locations);
		std::getline(line, file_locations, ' ');

		std::getline(line, file_locations, ' ');
		std::string file = folder_path_ + "/" + file_locations;
		depth_files_.push_back(file);


		std::getline(line,file_locations, ' ');
		std::getline(line,file_locations, ' ');
		file = folder_path_ + "/" + file_locations;
		rgb_files_.push_back(file);
	}

	if(rgb_files_.size() > 0) {
		running_ = true;
	}

	if(frameshift_RGB_ > 0) {
		rgb_files_.erase(rgb_files_.begin(), rgb_files_.begin() + frameshift_RGB_);
		depth_files_.erase(depth_files_.end() - frameshift_RGB_, depth_files_.end());
	}
	if(frameshift_RGB_ < 0) {
		depth_files_.erase(depth_files_.begin(), depth_files_.begin() - frameshift_RGB_);
		rgb_files_.erase(rgb_files_.end() + frameshift_RGB_, rgb_files_.end());
	}

	if(isRunning()) {
		//readNewSetOfImages();
	} else {
		cout << "could not open " << file_name_2_open << endl;
	}

	//Load groundtruth
	if(folder.find("georg") != string::npos) {
		assert(0); //THIS NEVER WORKED
		float scale = 1.0f * 0.580382; // scale the translation

		string trajectory_filename = folder + "/poses.csv";
		ifstream trajectory_file;
		trajectory_file.open(trajectory_filename);
		if(trajectory_file.is_open()) {
			has_poses_ = use_pose;
			string line;
			while(getline(trajectory_file, line)) {
				if(line[0] != '#') {

					std::replace(line.begin(), line.end(), ',', ' ');
					TrajectoryPoint p;
					stringstream stream(line);
					stream >> p.timestamp;
					float rx, ry, rz;
					float x, y, z;
					stream >> rx >> ry >> rz >> x >> y >> z;
					/*
					 * euler:
					Eigen::AngleAxisf rollAngle(rx, Eigen::Vector3f::UnitX());
					Eigen::AngleAxisf pitchAngle(ry, Eigen::Vector3f::UnitY());
					Eigen::AngleAxisf yawAngle(rz, Eigen::Vector3f::UnitZ());

					Eigen::Matrix3f r =
							 (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
							 */

					Eigen::Vector3f rot_vec(rx, ry, rz);
					Eigen::AngleAxisf raar(rot_vec.norm(), rot_vec.normalized());
					Eigen::Matrix3f r = raar.toRotationMatrix();

					Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
					pose.block<3, 3>(0, 0) = r;//.inverse();
					pose(0, 3) = x * scale;
					pose(1, 3) = y * scale;
					pose(2, 3) = z * scale;
					//cout << pose << endl;
					if(invert_GT_trajectory) {
						pose.block<3, 1>(0, 3) = - pose.block<3, 3>(0, 0).inverse() * pose.block<3, 1>(0, 3);
						pose.block<3, 1>(0, 3) = Vector3f(0, 0, 0);
						p.position = pose;
					} else {
						p.position = pose.inverse();
					}
					trajectory_.push_back(p);
				}
			}
		}


	} else {
		//Actually this is the only valid path
		string trajectory_filename = folder + "/groundtruth.txt";
		ifstream trajectory_file;
		trajectory_file.open(trajectory_filename);
		if(trajectory_file.is_open()) {
			has_poses_ = use_pose;

			/*TrajectoryPoint tp;
			tp.timesyamp=1;
			tp.position = Eigen::Matrix4f::Identity();
			trajectory_.push_back(tp);
			*/
			string line;
			while(getline(trajectory_file, line)) {
				if(line[0] != '#') {
					//cout << line << endl;
					TrajectoryPoint p;
					float x, y, z;
					float qx, qy, qz, qw;
					sscanf(line.c_str(), "%lf %f %f %f %f %f %f %f", &p.timestamp, &x, &y, &z, &qx, &qy, &qz, &qw);

					x *= trajectory_GT_scale;
					y *= trajectory_GT_scale;
					z *= trajectory_GT_scale;
					//read line to parameters and convert
					Eigen::Affine3f transform(Translation3f(x, y, z));
					Matrix4f t = transform.matrix();


					Quaternionf quaternion(qw, qx, qy, qz);
					quaternion.toRotationMatrix();
					Matrix4f r = Matrix4f::Identity();

					r.topLeftCorner<3,3>() = quaternion.toRotationMatrix();

					p.position = t * r;
					if(invert_GT_trajectory) {
						//original!
						//Matrix4f mat = p.position.inverse();
						//p.position = mat;
						//new attempt!!


						Matrix4f mat = p.position;
						p.position.block<3, 3>(0, 0) = mat.block<3, 3>(0, 0).inverse();
						p.position.block<3, 1>(0, 3) = mat.block<3, 1>(0, 3);

					}

					trajectory_.push_back(p);

				}
			}
		}
	}
	
	//tum intrinsics:
	rgb_intrinsics_ = Eigen::Vector4f(535.4, 539.2, 320.1, 247.6);
	depth_intrinsics_ = rgb_intrinsics_;
	depth_2_RGB_ = Eigen::Matrix4f::Identity();


	if(folder.find("tumalike") != string::npos) {
		rgb_intrinsics_ = Eigen::Vector4f(537.562, 537.278, 313.73, 243.601);
		depth_intrinsics_ = Eigen::Vector4f(563.937, 587.847, 328.987, 225.661);
		if(folder.find("6") != string::npos ||
		   folder.find("7") != string::npos ||
		   folder.find("8") != string::npos ||
		   folder.find("9") != string::npos ||
		   folder.find("8") != string::npos ||
		   folder.find("3") != string::npos ||
		   folder.find("5") != string::npos ||
		   folder.find("1") != string::npos) {//the printer corner

			rgb_intrinsics_ = Eigen::Vector4f(565, 575, 315, 220);
			depth_intrinsics_ = Eigen::Vector4f(563.937, 587.847, 328.987, 225.661);

			//create the deviation between
			Matrix4f rel_depth_to_color = Matrix4f::Identity();
			rel_depth_to_color(0,2) = -0.026f; //i think the color camera is 2.6cm left of the depth camera
			Matrix3f rot = (AngleAxisf(-0.05 * 0.0, Vector3f::UnitX()) *
			                AngleAxisf( 0.0 * M_PI, Vector3f::UnitY()) *
			                AngleAxisf( 0.0 * M_PI, Vector3f::UnitZ())).normalized().toRotationMatrix();//somehow when doing this none
						  
			Matrix4f rot4 = Matrix4f::Identity();
			rot4.block<3, 3>(0, 0) = rot;
			cout << rot << endl;
			depth_2_RGB_ = rot4 * rel_depth_to_color;

			//basicly whats in the elastic fusion initialization
			rgb_intrinsics_ = Eigen::Vector4f(528, 528, 320, 240); //why is this wrong
			depth_intrinsics_ = Eigen::Vector4f(528, 528, 320, 240);
			depth_2_RGB_ = Matrix4f::Identity();
		}
	}
	/*
	if(folder.find("virt") != string::npos){
		rgb_intrinsics_ = Eigen::Vector4f(481.2,-480.0,308.258 , 243.525);
		depth_intrinsics_ = rgb_intrinsics_;
		scale_depth_=1;
	}
	if(folder.find("georg") != string::npos){
		//setup the intrinsics according to georgs dataset.
		scale_depth_=5;
		rgb_intrinsics_=Eigen::Vector4f(528.658 , 528.225,313.73,243.601);
		depth_intrinsics_ = Eigen::Vector4f(564.55 , 562.179,319.282 , 248.212);
	}

	if(folder.find("ir_test")){
		rgb_intrinsics_=Eigen::Vector4f(528.658 , 528.225,313.73,243.601);
		depth_intrinsics_=Eigen::Vector4f(528.658 , 528.225,313.73,243.601);
		depth_2_RGB_ = Matrix4f::Identity();
		//assert(0);

	}*/
	//check if the exposure file exists.
	ifstream exposure_file;
	if(has_high_res_) {
		exposure_file.open(folder + "/rgb_ids_exposure.txt");
	} else {
		exposure_file.open(folder + "/rgb_exposure.txt");
	}
	//(folder + "/rgb_ids_exposure.txt");
	if(exposure_file.is_open()) {

		std::string line;
		while(getline(exposure_file, line)) {
			std::string::size_type sz;     // alias of size_t
			float time = std::stof(line, &sz);
			exposure_times_.push_back(time);
		}

		cout << "these exposure times are all wrong and need to be assigned to the correct frame" << endl;

		//assert(0);//this doesn't work this way the exposure times need to be attached to the frames
//        assert(0);
		//cout << "oh! seemingly we have exposure control and one of these new supwerdatasets" << endl;
		cv::Size size;
		cv::Mat M;
		cv::Mat D;
		cv::Mat Mnew;
		if(has_high_res_) {
			cv::FileStorage intrinsics_id_storage(folder + "/../calib_result_ids.yml", cv::FileStorage::READ);
			if(!intrinsics_id_storage.isOpened()) {
				assert(0);
			}
			intrinsics_id_storage["camera_matrix"] >> M;
			intrinsics_id_storage["distortion_coefficients"] >> D;
			intrinsics_id_storage["image_width"] >> size.width;
			intrinsics_id_storage["image_height"] >> size.height;

			Mnew = cv::getOptimalNewCameraMatrix(M, D, size, 1, size);
			Mnew = M;//DEBUG
			rgb_intrinsics_ = Eigen::Vector4f(Mnew.at<double>(0,0), 
			                                  Mnew.at<double>(1,1),
			                                  Mnew.at<double>(0,2), 
			                                  Mnew.at<double>(1,2));
			cv::initUndistortRectifyMap(M, D, cv::Mat(), Mnew, size, CV_16SC2, 
			                            rgb_undistort_1_, rgb_undistort_2_);

			float focalScale = 1.0f;//25f;
			rgb_intrinsics_[0] *= focalScale;
			rgb_intrinsics_[1] *= focalScale;

		} else {
			//do standard xtion stuff
			rgb_intrinsics_ = Eigen::Vector4f(530, 530, 320, 240);
		}


		/*
		cv::FileStorage intrinsicsIrStorage(folder + "/../newCalib/cam_params_ir.yml",cv::FileStorage::READ);
		if(!intrinsicsIrStorage.isOpened()){
			assert(0);
		}
		intrinsicsIrStorage["camera_matrix"] >> M;
		intrinsicsIrStorage["distortion_coefficients"] >> D;
		intrinsicsIrStorage["image_width"] >> size.width;
		intrinsicsIrStorage["image_height"] >> size.height;

		Mnew = cv::getOptimalNewCameraMatrix(M,D,size,1,size);


		depth_intrinsics_ =
				Eigen::Vector4f(Mnew.at<double>(0,0), Mnew.at<double>(1,1),
						Mnew.at<double>(0,2), Mnew.at<double>(1,2));

		cv::initUndistortRectifyMap(M,D,cv::Mat(),Mnew,size,CV_16SC2,depth_undistort_1_,depth_undistort_2_);
		*/

		//the standard values
		depth_intrinsics_ = Eigen::Vector4f(568, 568, 320, 240);//the structure sensor
		depth_intrinsics_ = Eigen::Vector4f(570, 570, 320, 240);//xtion






		//lets try to create a rotation and translation matrix:
		//create the deviation betweenx
		/*
		Matrix4f relDepthToColor = Matrix4f::Identity();
		relDepthToColor(0,3) = -0.045f;
		relDepthToColor(1,3) = -0.005f;
		relDepthToColor(2,3) = 0.0196f;
		//i think the color camera is 2.6cm left of the depth camera
		Matrix3f rot=(AngleAxisf(0.005*M_PI,Vector3f::UnitX()) *
					  AngleAxisf(0.0010*M_PI,  Vector3f::UnitY()) *
					  AngleAxisf(0.0*M_PI,  Vector3f::UnitZ())//somehow when doing this none
		).normalized().toRotationMatrix();//why normalized
		Matrix4f rot4=Matrix4f::Identity();
		rot4.block<3,3>(0,0) = rot;//actually transpose would work as well
		cout<< rot << endl;
		//depth_2_RGB_ = rot4*relDepthToColor;
		cout << relDepthToColor << endl;
		cout << depth_2_RGB_ << endl;
		*/



		cv::Mat R, T, Rf, Tf;

		if(has_high_res_) {
			Matrix4f rot4 = Matrix4f::Identity();


			//TRACK 16 - 19 should work with these settings:
			//Tweaking of the calibration because the camera rack is not rigid
			Matrix3f rot=(AngleAxisf(0.010 * M_PI, Vector3f::UnitX()) *
			              AngleAxisf(0.002 * M_PI, Vector3f::UnitY()) *
			              AngleAxisf(  0.0 * M_PI, Vector3f::UnitZ())).normalized().toRotationMatrix();
			Matrix4f rot41 = Matrix4f::Identity();
			rot41.block<3, 3>(0, 0) = rot;


			cv::FileStorage fs(folder + "/../extrinsics.yml", cv::FileStorage::READ);

			fs["R"] >> R;
			fs["T"] >> T;
			R.convertTo(Rf, CV_32FC1);
			T.convertTo(Tf, CV_32FC1);
			Eigen::Matrix3f eR(reinterpret_cast<float*>(Rf.data));
			Eigen::Vector3f eT(reinterpret_cast<float*>(Tf.data));
			//cout << eR << endl;
			//cout << eT << endl;


			rot4.block<3, 3>(0, 0) = eR;//.inverse();
			rot4.block<3, 1>(0, 3) = eT;


			depth_2_RGB_ =  rot41 * rot4; //rot41*

			//cout << depth_2_RGB_ << endl;

		} else {
			depth_2_RGB_ = Matrix4f::Identity();
			depth_2_RGB_(0, 3) = 0.026f;//standard xtion baseline
		}

		if(has_high_res_) {
			radiometric_response_ = new radical::RadiometricResponse(folder + "/../rgb_ids.crf");
			vignetting_response_ = new radical::VignettingResponse(folder + "/../rgb_ids.vgn");
		} else {
			radiometric_response_ = new radical::RadiometricResponse(folder + "/../rgb.crf");
			vignetting_response_ = new radical::VignettingResponse(folder + "/../rgb.vgn");
		}

	}
	if(folder.find("icl") != string::npos) {
		//scale_depth_=1.0f/5.0f;
	}

}

TumDataset::~TumDataset() {
	if(radiometric_response_ != nullptr) {
		delete radiometric_response_;
		delete vignetting_response_;
	}
}

void TumDataset::readNewSetOfImages() {
	//cout << "reading new Images" << endl;
	std::string file_locations;


	//return;//debug
	if(frame_index_ != 0) {
		frame_index_ += skip_count;
	}
	if(frame_index_ < rgb_files_.size()) {
		//std::string fileName2Open=folder_path_ + "/" + fileLocations;
		//cout << "reading file " << fileName2Open << endl;
		current_depth_ = imread(depth_files_[frame_index_], cv::IMREAD_UNCHANGED);
		if(exposure_times_.size() > 0) {
			rgb_exposure_time_ = exposure_times_[frame_index_];
		}
		current_timestamp_ = timestamps_[frame_index_];

		//cv::Mat flipped;
		//cv::flip(current_depth_,flipped,1);
		//cv::imwrite(fileName2Open,flipped);
		current_depth_ = current_depth_ * scale_depth_;
		//current_depth_ = current_depth_*0.1f;//DEBUG for the artificial dataset
		//DEBUG:
		 //because we colledted data in 100um mode





		//fileName2Open=folder_path_ + "/" + fileLocations;
		current_RGB_ = cv::imread(rgb_files_[frame_index_]);
		/*if(current_RGB_.type() == CV_8UC1){
			cv::Mat gray2RGB;
			current_RGB_.convertTo(gray2RGB,CV_8UC3);
			current_RGB_ = gray2RGB;
			assert(0);
		}*/

		//cout << "DEBUG intrinsics:" << endl << rgb_intrinsics_ << endl << depth_intrinsics_ << endl;
		//cout << "DEBUG matrix " << endl << depth_2_RGB_ << endl;


		if(!depth_undistort_1_.empty()) {
			//undistort the images
			cv::Mat current_depth_undistorted;
			cv::remap(current_depth_, current_depth_undistorted, depth_undistort_1_, depth_undistort_2_, INTER_NEAREST);
			//current_depth_ = current_depth_Undistorted;//DEBUG: deactivate this
		}



		if(radiometric_response_ != nullptr && vignetting_response_!=nullptr) {
			cv::Mat irradiance, radiance;  // temporary storage
			//cv::Mat rgbVignettingCorrected;       // output image with vignette removed
			radiometric_response_->inverseMap(current_RGB_, irradiance);
			vignetting_response_->remove(irradiance, radiance);
			radiance = radiance * 0.9;
			radiance = radiance * 1.2;
			for (int i = 0; i < radiance.size().area(); i++) {
				cv::Vec3f v = radiance.at<cv::Vec3f>(i);
				v[0] *= white_fix_[0];
				v[1] *= white_fix_[1];
				v[2] *= white_fix_[2];
				radiance.at<cv::Vec3f>(i) = v;
			}
			radiometric_response_->directMap(radiance, current_RGB_);


		}

		if(!rgb_undistort_1_.empty()) {

			cv::Mat current_rgb_undistorted;
			cv::remap(current_RGB_, current_rgb_undistorted, rgb_undistort_1_, 
			          rgb_undistort_2_, INTER_LINEAR);
			current_RGB_ = current_rgb_undistorted; 
			if(scale_depth_ != 1) {
				assert(0); //TODO: this scalefactor thingy really needs cleanup
			}
		}
		//as in the last datasets i collected
		//TODO: unify this over all datasets i use!

		//no scaling for the tumalike datasets

		//the highres datasets need scaling though:
		//current_depth_ = current_depth_*0.5f;// * 5; //scale by 5 in the high resolution dataset //between bla and 16

		frame_index_++;

	} else {
		//frameList.close();
		running_ = false;
	}



	//when we are done before our time we wait....
	chrono::system_clock::time_point now = chrono::system_clock::now();

	if(replay_speed != 0) {
		chrono::system_clock::duration frame_time =
		 chrono::microseconds((int)(33333.0f * (1.0f / replay_speed)));//1.0f/(30.0f*replaySpeed));
		chrono::system_clock::duration duration = now - last_frame_readout_;

		chrono::system_clock::duration zero = chrono::microseconds(0);

		chrono::system_clock::duration remaining_time =
			std::max(frame_time - duration, zero);


		//chrono::system_clock::duration remainingTime_ = duration-frameTime;

		/*
		cout << "target time in ms" <<
				std::chrono::duration_cast<std::chrono::milliseconds>(frameTime).count() << endl;
		cout << "used time in ms" <<
				std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << endl;
		cout << "time to wait" <<
				std::chrono::duration_cast<std::chrono::milliseconds>(remainingTime).count() << endl;
		auto tick = std::chrono::system_clock::now();
		*/
		this_thread::sleep_for(remaining_time);//remainingTime);
		/*auto delta = std::chrono::system_clock::now()- tick;
		cout << "time waited" <<
				std::chrono::duration_cast<std::chrono::milliseconds>(delta).count() << endl;
		*/
	}
	last_frame_readout_ = chrono::system_clock::now();
}


bool TumDataset::isRunning() {
	return running_;
}

cv::Mat TumDataset::getDepthFrame() {
	read_depth_ = true;
	if(read_depth_ && read_RGB_) {
		read_depth_ = read_RGB_ = false;
		//readNewSetOfImages();
	}
	return current_depth_;
}

cv::Mat TumDataset::getRGBFrame() {
	read_RGB_ = true;
	if(read_depth_ && read_RGB_) {
		read_depth_ = read_RGB_ = false;
		//readNewSetOfImages();
	}
	return current_RGB_;
}

Eigen::Vector4f TumDataset::getDepthIntrinsics() {
	return depth_intrinsics_;
}

Eigen::Vector4f TumDataset::getRgbIntrinsics() {
	return rgb_intrinsics_;
}

Eigen::Matrix4f TumDataset::getDepth2RgbRegistration() {
	return depth_2_RGB_;
}

bool TumDataset::hasGroundTruth() {
	return has_poses_;
}

Eigen::Matrix4f TumDataset::getDepthPose() {
	Matrix4f transform;//maybe not the right name
	double delta_time_min = 1000;
	for(const TumDataset::TrajectoryPoint& p : trajectory_) {
		if(fabs(current_timestamp_ - p.timestamp) < delta_time_min) {
			delta_time_min = fabs(current_timestamp_ - p.timestamp);
			transform = p.position;
		}
	}
	return transform;
}

Matrix4f TumDataset::getRgbPose() {
	Matrix4f transform;//maybe not the right name
	double delta_time_min = 1000;
	for(const TumDataset::TrajectoryPoint& p : trajectory_) {
		if(fabs(current_timestamp_ - p.timestamp) < delta_time_min) {
			delta_time_min = fabs(current_timestamp_ - p.timestamp);
			transform = p.position;
		}
	}
	return transform;
}

bool TumDataset::hasHighRes() {
	return has_high_res_;
}

float TumDataset::getRGBExposure() {
	return rgb_exposure_time_;
}
