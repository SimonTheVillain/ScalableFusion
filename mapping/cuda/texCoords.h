#ifndef FILE_TEX_COORDS_H
#define FILE_TEX_COORDS_H

#include <vector>
#include <memory>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "gpuMeshStructure.h"

using namespace std;
using namespace Eigen;

//better do this by triangles since we would have to select all the
class TexCoordGen {
public:

	struct Task {
		GpuTriangle *triangles;
		uint32_t triangle_count;
		Vector2f *coords;
		float scale_x;
		float scale_y;
		float offset_x;
		float offset_y;
	};

	struct BoundTask {
		GpuTriangle *triangles;
		uint32_t triangle_count;
		uint32_t target_ind;
		int debug_type = 0;
	};

	static void genTexCoords(vector<Task> tasks, Matrix4f proj_pose,
	                         GpuPatchInfo *patch_infos, GpuVertex *gpu_vertices);

	static vector<cv::Rect2f> getPotentialTexCoordBounds(
			vector<BoundTask> tasks, Matrix4f proj_pose, int result_count, 
			GpuPatchInfo *patch_infos, GpuVertex *vertices);

};

#endif


