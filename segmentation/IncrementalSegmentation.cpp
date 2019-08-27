//
// Created by simon on 11/21/18.
//

#include "IncrementalSegmentation.h"

using namespace std;
using namespace cv;


cv::Mat EdithSegmentation::generateNewLabels(cv::Mat *depth,
        cv::Mat *normals,
        cv::Mat *color,
        cv::Mat *existingLabels) {
    Mat placeholder(depth->cols,depth->rows,CV_32SC4);
    placeholder = 100000;
    return placeholder;
}

