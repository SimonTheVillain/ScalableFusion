#ifndef FILE_LABELLING_CUDA_H
#define FILE_LABELLING_CUDA_H

//whatever will be going on in here
#include <vector>

#include <opencv2/core.hpp>
#include <Eigen/Eigen>

#include "gpu_mesh_structure.h"

using namespace std;
using namespace Eigen;

namespace gpu {

	class Labelling {
	public:

		struct SegProjTask {
			int subchannel;
			cudaSurfaceObject_t lookup_surf;
			cv::Point2i lookup;
			cudaSurfaceObject_t dest_surf;
			cv::Rect2i destination;//we might need more than this
			size_t vertex_dest_start_ind;
		};

		struct InitializeTask {
			cudaSurfaceObject_t dest_surf;
			cv::Rect2i dest_rect;
		};

		static void labelSurfaces(vector<SegProjTask> tasks,
		                          const cudaSurfaceObject_t labelling,
		                          //the depth and geometry at the current view
		                          const cudaSurfaceObject_t geom_buffer,
		                          cv::Size2i resolution,
		                          Matrix4f _pose,
		                          Matrix4f proj_pose,
		                          GpuVertex *vertices,
		                          Vector2f *tex_pos,
		                          GpuTriangle* triangles,
		                          GpuPatchInfo* patch_infos);

		template<class T>
		static void initializeSurfaces(vector<InitializeTask> tasks, T value);

	};
} // namespace gpu

#endif // FILE_LABELLING_CUDA_H
