#ifndef FILE_INCREMENTALSEGMENTATION_H
#define FILE_INCREMENTALSEGMENTATION_H

#include <opencv2/core.hpp>

class IncrementalSegmentation {
public:

	virtual void initInThread() = 0;
	virtual cv::Mat generateNewLabels(cv::Mat *depth, cv::Mat *normals,
	                                  cv::Mat *color, 
	                                  cv::Mat *existing_labels) = 0;

};

class EdithSegmentation : public IncrementalSegmentation {
public:

    void initInThread() { }

    cv::Mat generateNewLabels(cv::Mat *depth, cv::Mat *normals, cv::Mat *color, 
                              cv::Mat *existing_labels);

};

#endif