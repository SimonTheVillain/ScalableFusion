//
// Created by simon on 11/21/18.
//

#ifndef SUPERMAPPING_INCREMENTALSEGMENTATION_H
#define SUPERMAPPING_INCREMENTALSEGMENTATION_H



#include <opencv2/core.hpp>

class IncrementalSegmentation{
public:

    virtual void initInThread() = 0;
    virtual cv::Mat generateNewLabels(cv::Mat *depth,
            cv::Mat *normals,
            cv::Mat *color,
            cv::Mat *existingLabels) = 0;
};


class EdithSegmentation : public IncrementalSegmentation{
public:


    void initInThread(){

    }

    cv::Mat generateNewLabels(cv::Mat *depth, cv::Mat *normals, cv::Mat *color, cv::Mat *existingLabels);
};








#endif //SUPERMAPPING_INCREMENTALSEGMENTATION_H
