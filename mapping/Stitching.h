//
// Created by simon on 8/2/19.
//

#ifndef SUPERMAPPING_STITCHING_H
#define SUPERMAPPING_STITCHING_H

#include <vector>
#include <opencv2/core.hpp>
#include "../base/meshStructure.h"

class MeshReconstruction;

class Stitching {
public:
    MeshReconstruction* meshReconstruction;

    void Setup(MeshReconstruction* reconstruction){
        meshReconstruction = reconstruction;
    }

    std::vector<std::vector<Edge>> borderList;

    void rasterBorderGeometry(std::vector<std::vector<Edge>> &borders,
                              Eigen::Matrix4f view, Eigen::Matrix4f proj, cv::Mat geometry);


    void rasterLineGeometry(Eigen::Matrix4f _view, Eigen::Matrix4f proj, Edge* edge, cv:: Mat geometry, cv::Mat debug);


    void genBorderList(std::vector<std::shared_ptr<MeshPatch>> &patches,
                       std::vector<std::vector<Edge>> &borderList,
                       Eigen::Matrix4f debugProj_pose);
    void reloadBorderGeometry(std::vector<std::vector<Edge>> &borderList);
    //TODO: also download the geometry of such list
    void freeBorderList(std::vector<std::vector<Edge>> &borderList);

    void stitchOnBorders(std::vector<std::vector<Edge> > &borders, Eigen::Matrix4f view, Eigen::Matrix4f proj,
                                    cv::Mat stdProj, cv::Mat geomProjM, cv::Mat newGeomM, cv::Mat newStd,
                                    cv::Mat debugColorCodedNewSegmentation, cv::Mat newSegPM,
                                    cv::Mat newPtIndM,
                                    std::vector<std::weak_ptr<GeometryBase>> &debugListNewEdges);


};


#endif //SUPERMAPPING_STITCHING_H
