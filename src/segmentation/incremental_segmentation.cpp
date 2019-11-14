#include <segmentation/incremental_segmentation.h>

using namespace std;
using namespace cv;

cv::Mat EdithSegmentation::generateNewLabels(cv::Mat *depth, cv::Mat *normals,
                                             cv::Mat *color, 
                                             cv::Mat *existing_labels) {
	Mat placeholder(depth->cols, depth->rows, CV_32SC4);
	placeholder = 100000;
	return placeholder;
}
