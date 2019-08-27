//
// Created by simon on 8/2/19.
//

#ifndef SUPERMAPPING_GEOMETRYUPDATE_H
#define SUPERMAPPING_GEOMETRYUPDATE_H

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <memory>
#include "Meshing.h"
#include "Stitching.h"

class MeshReconstruction;
class ActiveSet;
namespace gfx{
    class GpuTex2D;
}

class GeometryUpdate {
public:
    MeshReconstruction *meshReconstruction;
    Meshing meshing;
    Stitching stitching;

    void Setup(MeshReconstruction* reconstruction){
        meshReconstruction = reconstruction;
        stitching.Setup(reconstruction);
        meshing.Setup(reconstruction);
    }

    void Extend(std::shared_ptr<ActiveSet> activeSetOfFormerlyVisiblePatches,
                std::shared_ptr<gfx::GpuTex2D> dStdTex,
                cv::Mat& dStdMat,
                Eigen::Matrix4f depthPoseIn,
                std::shared_ptr<gfx::GpuTex2D> rgbTex,
                Eigen::Matrix4f colorPoseIn);

    //TODO: this!!!!
    void Update(std::shared_ptr<gfx::GpuTex2D> dStdTex,
                Eigen::Matrix4f depthPoseIn,
                std::shared_ptr<ActiveSet> &activeSet);




};


#endif //SUPERMAPPING_GEOMETRYUPDATE_H
