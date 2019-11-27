#ifndef FILE_MESHER_H
#define FILE_MESHER_H

#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <base/mesh_structure.h>

using namespace std;

class MeshReconstruction;

class Mesher {
public:
	struct TriangleReference{
		Meshlet* ptr = nullptr;
		int pos = -1;
		void invalidate(){
			ptr = nullptr;
			pos = -1;
		}
		Triangle* get(){
			if(ptr==nullptr){
				return nullptr;
			}
			return & (ptr->triangles[pos]);
		}
	};

	struct Neighbour{
		int pos = -1;
		TriangleReference ref;
		void invalidate(){
			pos = -1;
			ref.invalidate();
		}

	};

	void setup(MeshReconstruction* mesh_reconstruction) {
		this->mesh_reconstruction = mesh_reconstruction;
	}

	//TODO: instead of matrices and positions, call it with point references
	TriangleReference addTriangle(Vertex* pr1,
	                              Vertex* pr2,
	                              Vertex* pr3,
	                              Neighbour &n1,
	                              Neighbour &n2,
	                              Neighbour &n3,
	                              int &rotated);

	void meshIt(cv::Mat points, cv::Mat mesh_pointers, cv::Mat vertex_indices,
	            cv::Mat sensor_std, float max_depth_step, //deprecate this
	            Eigen::Matrix4f depth_pose);


	// TODO: move fillNovelPatchesWithTexIndices here! (because it is a terrible function name
	void genTexIndices(vector<shared_ptr<Meshlet> > &patches);

	MeshReconstruction *mesh_reconstruction;

};

#endif // FILE_MESHER_H
