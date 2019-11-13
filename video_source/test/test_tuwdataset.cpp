#include <source.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void show(video::Source *source) {
	cv::imshow("Display", source->frame.rgb);
}

int main(void) {
	video::TuwDataset source("/home/niko/Downloads/reconstruction_dataset/track_19", false);

	cv::namedWindow("Display", cv::WINDOW_AUTOSIZE);
	while(source.readFrame()) {
		show(&source);
		cv::waitKey(0);
	}

	return 0;
}