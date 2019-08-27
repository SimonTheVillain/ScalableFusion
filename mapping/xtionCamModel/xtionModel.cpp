#include "xtionModel.h"
cv::Mat XtionModel::getOccludedAreas(cv::Mat depthMap)
{
    int width = depthMap.cols;
    int height = depthMap.rows;
    for(size_t i = 0;i<height;i++){
        for(size_t j=0;j<width;j++){

        }
    }

}

cv::Mat XtionModel::getTooSteepAreas(cv::Mat renderedNormals)
{

}

cv::Mat XtionModel::getNansToDelete(cv::Mat occluded, cv::Mat tooSteep)
{

}
