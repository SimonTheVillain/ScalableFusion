#ifndef FILE_MESHING_H
#define FILE_MESHING_H

#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include "meshStructure.h"

using namespace std;

class MeshReconstruction;

class Meshing {
public:

	void setup(MeshReconstruction* mesh_reconstruction) {
		this->mesh_reconstruction = mesh_reconstruction;
	}

	//TODO: instead of matrices and positions, call it with point references
	TriangleReference addTriangle(const VertexReference &pr1, 
	                              const VertexReference &pr2,
	                              const VertexReference &pr3,
	                              const Triangle::Neighbour &n1,
	                              const Triangle::Neighbour &n2,
	                              const Triangle::Neighbour &n3,
	                              int &rotated);

	void meshIt(cv::Mat points, cv::Mat mesh_pointers, cv::Mat vertex_indices,
	            cv::Mat sensor_std, float max_depth_step, //deprecate this
	            Eigen::Matrix4f depth_pose);


	// TODO: move fillNovelPatchesWithTexIndices here! (because it is a terrible function name
	void genTexIndices(vector<shared_ptr<MeshPatch> > &patches);

	MeshReconstruction *mesh_reconstruction;

};

#endif // FILE_MESHING_H
