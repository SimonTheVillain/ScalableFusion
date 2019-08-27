#ifndef FILE_XTION_MODEL_H
#define FILE_XTION_MODEL_H
#include <opencv2/core.hpp>



class XtionModel{
    //TODO: delete! it seems like we are not using this right now!
public:
    static cv::Mat getOccludedAreas(cv::Mat depthMap);


    static cv::Mat getTooSteepAreas(cv::Mat renderedNormals);

    static cv::Mat getNansToDelete(cv::Mat occluded, cv::Mat tooSteep);

};

#endif
