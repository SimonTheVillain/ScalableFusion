#ifndef FILE_GEOMETRY_UPDATER_H
#define FILE_GEOMETRY_UPDATER_H

#include <memory>
#include <iostream>
#include <unordered_map>

#include <Eigen/Eigen>
#include <opencv2/core.hpp>

#include <mesher.h>
#include <mesh_stitcher.h>
#include <utils/gpu_norm_seg.h>

using namespace std;
using namespace Eigen;

class MeshReconstruction;
class ActiveSet;
class TextureUpdater;
class SchedulerBase;

namespace gfx {

class GpuTex2D;

} // namespace gfx

class GeometryUpdater {
public:
	void setup(MeshReconstruction *reconstruction) {
		mesh_reconstruction = reconstruction;
		stitching.setup(reconstruction);
		meshing.setup(reconstruction);
	}

	shared_ptr<ActiveSet> extend(
			SchedulerBase* scheduler,
			MeshReconstruction* reconstruction,
			InformationRenderer *information_renderer,
			TextureUpdater* texture_updater,
			LowDetailRenderer* low_detail_renderer,
			GpuStorage* gpu_storage,
			shared_ptr<ActiveSet> active_set_of_formerly_visible_patches,
			shared_ptr<gfx::GpuTex2D> d_std_tex,
			cv::Mat &d_std_mat, Matrix4f depth_pose_in,
			shared_ptr<gfx::GpuTex2D> rgb_tex,
			Matrix4f color_pose_in);

	//TODO: this!!!!
	shared_ptr<ActiveSet> update(
				GpuStorage* gpu_storage,
				vector<shared_ptr<Meshlet>> requested_meshlets,
				shared_ptr<ActiveSet> preexisting_set, // the active set(if existing) containing all requested meshlets
				SchedulerBase* scheduler,//to generate a new active set with
				shared_ptr<gfx::GpuTex2D> d_std_tex,
	            Matrix4f depth_pose_in,
	            Matrix4f depth_proj);
	
	MeshReconstruction *mesh_reconstruction;
	Mesher meshing;
	MeshStitcher stitching;

	//TODO: integrate this here!
	GpuNormSeg gpu_pre_seg_;

	GeometryUpdater(	GarbageCollector* garbage_collector,
						int width,int height) :
							gpu_pre_seg_(garbage_collector,width,height) {}

};

#endif // FILE_GEOMETRY_UPDATE_H
