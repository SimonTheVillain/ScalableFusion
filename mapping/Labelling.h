//
// Created by simon on 8/2/19.
//

#ifndef SUPERMAPPING_LABELLING_H
#define SUPERMAPPING_LABELLING_H

#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>


class ActiveSet;
class MeshReconstruction;
namespace gfx{
    class GpuTex2D;
}
//there is the same class in the gpu namespace

class Labelling {
public:
    MeshReconstruction *meshReconstruction;
    void ProjectLabels(std::shared_ptr<ActiveSet> activeSet,
                       cv::Mat &labels,
                       std::shared_ptr<gfx::GpuTex2D> dStdTex,
                       Eigen::Matrix4f pose);//,
    //Eigen::Matrix4f proj);



    void ApplyLabels(std::shared_ptr<ActiveSet> activeSet,
                             cv::Mat labels, Eigen::Matrix4f pose);
};


#endif //SUPERMAPPING_LABELLING_H
