//
// Created by simon on 8/2/19.
//

#ifndef SUPERMAPPING_MESHING_H
#define SUPERMAPPING_MESHING_H

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <memory>
#include "meshStructure.h"

class MeshReconstruction;

class Meshing {
public:
    MeshReconstruction* meshReconstruction;
    //MeshReconstruction *meshReconstruction;
    void Setup(MeshReconstruction* meshReconstruction){
        this->meshReconstruction = meshReconstruction;
    }

    //TODO: instead of matrices and positions, call it with point references
    TriangleReference addTriangle(const VertexReference &pr1,
                     const VertexReference &pr2,
                     const VertexReference &pr3,
                     const Triangle::Neighbour &n1,
                     const Triangle::Neighbour &n2,
                     const Triangle::Neighbour &n3,
                     int &rotated);

    void MeshIt(cv::Mat points, cv::Mat meshPointers,
                cv::Mat vertexIndices,
                cv::Mat sensorStd,
                float maxDepthStep,//deprecate this
                Eigen::Matrix4f depthPose);


    //TODO: move fillNovelPatchesWithTexIndices here! (because it is a terrible function name
    void GenTexIndices(std::vector<std::shared_ptr<MeshPatch> > &patches);


};


#endif //SUPERMAPPING_MESHING_H
