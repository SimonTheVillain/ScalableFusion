#ifndef FILE_GEOMETRY_UPDATE_H
#define FILE_GEOMETRY_UPDATE_H

#include <memory>
#include <iostream>
#include <unordered_map>

#include <Eigen/Eigen>
#include <opencv2/core.hpp>

#include <meshing.h>
#include <stitching.h>

using namespace std;
using namespace Eigen;

class MeshReconstruction;
class ActiveSet;

namespace gfx {

class GpuTex2D;

} // namespace gfx

class GeometryUpdate {
public:

	void setup(MeshReconstruction *reconstruction) {
		mesh_reconstruction = reconstruction;
		stitching.setup(reconstruction);
		meshing.setup(reconstruction);
	}

	void extend(shared_ptr<ActiveSet> active_set_of_formerly_visible_patches,
	            shared_ptr<gfx::GpuTex2D> d_std_tex,
	            cv::Mat &d_std_mat, Matrix4f depth_pose_in,
	            shared_ptr<gfx::GpuTex2D> rgb_tex,
	            Matrix4f color_pose_in);

	//TODO: this!!!!
	void update(shared_ptr<gfx::GpuTex2D> d_std_tex,
	            Matrix4f depth_pose_in,
	            shared_ptr<ActiveSet> &active_set);
	
	MeshReconstruction *mesh_reconstruction;
	Meshing meshing;
	Stitching stitching;

};

#endif // FILE_GEOMETRY_UPDATE_H
