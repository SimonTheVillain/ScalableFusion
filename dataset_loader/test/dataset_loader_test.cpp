 #include "../dataset_loader.h"

using namespace std;

int main(int argc, const char* argv[]) {

	string folder("./rgbd_dataset_freiburg3_teddy");
	bool realtime = true;
	bool use_pose = true;
	bool use_high_res = false;
	int skip_n_frames = 0;
	float depth_scale = 1;
	float trajectory_GT_scale = 1;
	bool invert_GT_trajectory = false;
	TumDataset dataset(folder); 

	return 0;
}