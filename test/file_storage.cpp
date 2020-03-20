//
// Created by simon on 20.03.20.
//

#include <iostream>
#include <string>
#include <opencv2/core/persistence.hpp>

using namespace std;
// Depth cam intrinsics [596.6278076171875, 0.0, 311.98663330078125,
//                       0.0, 596.6278076171875, 236.76168823242188,
//                       0.0, 0.0, 1.0]

int main()
{
	string path ="/home/simon/datasets/tum_alike/jb/intrinsics.yml";

	cv::FileStorage fs(path,cv::FileStorage::WRITE);

	//[615.400634765625, 0.0, 312.87567138671875, 0.0, 615.0452880859375, 250.85874938964844, 0.0, 0.0, 1.0]
	//fx, fy, cx, cy
	cv::Vec4d rgb(615.400634765625, 615.0452880859375, 312.87567138671875, 250.85874938964844);

	//[596.6278076171875, 0.0, 311.98663330078125, 0.0, 596.6278076171875, 236.76168823242188, 0.0, 0.0, 1.0]
	//fx, fy, cx, cy
	cv::Vec4d depth(596.6278076171875, 596.6278076171875, 311.98663330078125, 236.76168823242188);
	fs << "rgb" << rgb;
	fs << "depth" << depth;
	fs.release();



	return 0;
}
