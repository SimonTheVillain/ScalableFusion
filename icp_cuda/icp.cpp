#include <iomanip>
#include <fstream>
#include <chrono>

#include <pangolin/image/image_io.h>

#include "icp_odometry.h"

using namespace std;

ifstream as_file;
string directory;

void tokenize(const string &str, vector<string> &tokens, 
              string delimiters = " ") {
	tokens.clear();

	string::size_type last_pos = str.find_first_not_of(delimiters, 0);
	string::size_type pos      = str.find_first_of(delimiters, last_pos);

	while(string::npos != pos || string::npos != last_pos) {
		tokens.push_back(str.substr(last_pos, pos - last_pos));
		last_pos = str.find_first_not_of(delimiters, pos);
		pos      = str.find_first_of(delimiters, last_pos);
	}
}

uint64_t loadDepth(pangolin::Image<unsigned short> &depth) {
	string         current_line;
	vector<string> tokens;
	vector<string> time_tokens;

	do {
		getline(as_file, current_line);
		tokenize(current_line, tokens);
	} while(tokens.size() > 2);

	if(tokens.size() == 0)
		return 0;

	string depth_loc = directory;
	depth_loc.append(tokens[1]);

	pangolin::TypedImage depth_raw = 
		pangolin::LoadImage(depth_loc, pangolin::ImageFileTypePng);

	pangolin::Image<unsigned short> depth_raw_16(
		(unsigned short*)depth_raw.ptr, depth_raw.w,
		depth_raw.h, depth_raw.w * sizeof(unsigned short));

	tokenize(tokens[0], time_tokens, ".");

	string time_string = time_tokens[0];
	time_string.append(time_tokens[1]);

	uint64_t time;
	istringstream(time_string) >> time;

	for(unsigned int i = 0; i < 480; i++) {
		for(unsigned int j = 0; j < 640; j++) {
			depth.RowPtr(i)[j] = depth_raw_16(j, i) / 5;
		}
	}

	depth_raw.Dealloc();

	return time;
}

void outputFreiburg(const string filename, const uint64_t &timestamp, 
                    const Eigen::Matrix4f &currentPose) {
	ofstream file;
	file.open(filename.c_str(), fstream::app);

	stringstream strs;

	strs << setprecision(6) << fixed << (double)timestamp / 1000000.0 << " ";

	Eigen::Vector3f trans = currentPose.topRightCorner(3, 1);
	Eigen::Matrix3f rot   = currentPose.topLeftCorner(3, 3);

	file << strs.str() << trans(0) << " " << trans(1) << " " << trans(2) << " ";

	Eigen::Quaternionf current_cam_rot(rot);

	file << current_cam_rot.x() << " " <<
	        current_cam_rot.y() << " " <<
	        current_cam_rot.z() << " " <<
	        current_cam_rot.w() << "\n";

	file.close();
}

uint64_t getCurrTime() {
	return 
		chrono::duration_cast<chrono::microseconds>(
				chrono::high_resolution_clock::now().time_since_epoch()).count();
}

int main(int argc, char *argv[]) {
	assert((argc == 2 || argc == 3) && 
	       "Please supply the depth.txt dir as the first argument");

	directory.append(argv[1]);

	if(directory.at(directory.size() - 1) != '/') {
		directory.append("/");
	}

	string association_file = directory;
	association_file.append("depth.txt");

	as_file.open(association_file.c_str());

	pangolin::ManagedImage<unsigned short> first_data(640, 480);
	pangolin::ManagedImage<unsigned short> second_data(640, 480);

	pangolin::Image<unsigned short> first_raw(first_data.w, first_data.h, 
	                                          first_data.pitch, 
	                                          (unsigned short*) first_data.ptr);
	pangolin::Image<unsigned short> second_raw(second_data.w, second_data.h, 
	                                           second_data.pitch, 
	                                           (unsigned short*) second_data.ptr);

	ICPOdometry icp_odom(640, 480, 319.5, 239.5, 528, 528);

	assert(!as_file.eof() && as_file.is_open());

	loadDepth(first_raw);
	uint64_t timestamp = loadDepth(second_raw);

	Sophus::SE3d T_wc_prev;
	Sophus::SE3d T_wc_curr;

	ofstream file;
	file.open("output.poses", fstream::out);
	file.close();

	cudaDeviceProp prop;

	cudaGetDeviceProperties(&prop, 0);

	string dev(prop.name);

	cout << dev << endl;

	float mean = numeric_limits<float>::max();
	int count = 0;

	int threads = 224;
	int blocks = 96;

	int best_threads = threads;
	int best_blocks = blocks;
	float best = mean;

	if(argc == 3) {
		string search_arg(argv[2]);

		if(search_arg.compare("-v") == 0) {
			cout << "Searching for the best thread/block configuration "
			        "for your GPU..." << endl;
			cout << "Best: " << best_threads << " threads, " 
			     << best_blocks << " blocks (" << best << "ms)";
			cout.flush();

			float counter = 0;

			for(threads = 16; threads <= 512; threads += 16) {
				for(blocks = 16; blocks <= 512; blocks += 16) {
					mean = 0.0f;
					count = 0;

					for(int i = 0; i < 5; i++) {
						icp_odom.initICPModel(first_raw.ptr);
						icp_odom.initICP(second_raw.ptr);

						uint64_t tick = getCurrTime();

						T_wc_prev = T_wc_curr;

						Sophus::SE3d T_prev_curr = T_wc_prev.inverse() * T_wc_curr;

						icp_odom.getIncrementalTransformation(T_prev_curr, threads, blocks);

						T_wc_curr = T_wc_prev * T_prev_curr;

						uint64_t tock = getCurrTime();

						mean = (float(count) * mean + (tock - tick) / 1000.0f) / 
						       float(count + 1);
						count++;
					}

					counter++;

					if(mean < best) {
						best = mean;
						best_threads = threads;
						best_blocks = blocks;
					}

					cout << "\rBest: " << best_threads << " threads, " <<
					        best_blocks << " blocks (" << best << "ms), " <<
					        int((counter / 1024.f) * 100.f) << "%    ";
					cout.flush();
				}
			}
			cout << endl;
		}
	}

	threads = best_threads;
	blocks = best_blocks;
	cout << "best: threads " << threads << " blocks " << blocks << endl;

	mean = 0.0f;
	count = 0;

	T_wc_prev = Sophus::SE3d();
	T_wc_curr = Sophus::SE3d();

	while(!as_file.eof()) {
		icp_odom.initICPModel(first_raw.ptr);
		icp_odom.initICP(second_raw.ptr);

		uint64_t tick = getCurrTime();

		T_wc_prev = T_wc_curr;

		Sophus::SE3d T_prev_curr = T_wc_prev.inverse() * T_wc_curr;

		icp_odom.getIncrementalTransformation(T_prev_curr, threads, blocks);

		T_wc_curr = T_wc_prev * T_prev_curr;

		uint64_t tock = getCurrTime();

		mean = (float(count) * mean + (tock - tick) / 1000.0f) / float(count + 1);
		count++;

		cout << setprecision(4) << fixed << "\rICP: " << mean << "ms";
		cout.flush();

		swap(first_raw, second_raw);

		outputFreiburg("output.poses", timestamp, T_wc_curr.cast<float>().matrix());

		timestamp = loadDepth(second_raw);
	}

	cout << endl;

	cout << "ICP speed: " << int(1000.f / mean) << "Hz" << endl;

	return 0;
}