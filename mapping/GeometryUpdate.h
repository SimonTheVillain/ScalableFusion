#ifndef FILE_GEOMETRYUPDATE_H
#define FILE_GEOMETRYUPDATE_H

#include <memory>

#include <Eigen/Eigen>
#include <opencv2/core.hpp>

#include "Meshing.h"
#include "Stitching.h"

class MeshReconstruction;
class ActiveSet;

namespace gfx {

class GpuTex2D;

} // namespace gfx

class GeometryUpdate {
public:

	void setup(MeshReconstruction* reconstruction) {
		mesh_reconstruction = reconstruction;
		stitching.Setup(reconstruction);
		meshing.setup(reconstruction);
	}

	void extend(std::shared_ptr<ActiveSet> active_set_of_formerly_visible_patches,
	            std::shared_ptr<gfx::GpuTex2D> d_std_tex,
	            cv::Mat &d_std_mat, Eigen::Matrix4f depth_pose_in,
	            std::shared_ptr<gfx::GpuTex2D> rgb_tex,
	            Eigen::Matrix4f color_pose_in);

	//TODO: this!!!!
	void update(std::shared_ptr<gfx::GpuTex2D> d_std_tex,
	            Eigen::Matrix4f depth_pose_in,
	            std::shared_ptr<ActiveSet> &active_set);
	
	MeshReconstruction *mesh_reconstruction;
	Meshing meshing;
	Stitching stitching;

};

#endif
