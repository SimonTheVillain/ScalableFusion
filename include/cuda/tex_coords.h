#ifndef FILE_TEX_COORDS_H
#define FILE_TEX_COORDS_H

#include <vector>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "gpu_mesh_structure.h"

using namespace std;
using namespace Eigen;

//better do this by triangles since we would have to select all the
class TexCoordGen {
public:

	struct Task {
		GpuVertex *vertices;
		uint32_t vertex_count;
		Vector2f *coords;
		float scale_x;
		float scale_y;
		float offset_x;
		float offset_y;
	};

	struct BoundTask {
		GpuVertex* vertices;
		uint32_t vertex_count;
	};

	static void genTexCoords(vector<Task> tasks, Matrix4f proj_pose);

	static vector<cv::Rect2f> getPotentialTexCoordBounds(
			vector<BoundTask> tasks, Matrix4f proj_pose, int result_count);

	static vector<cv::Rect2f> getTexCoordBounds(
			vector<BoundTask> tasks, Matrix4f proj_pose);

};

#endif // FILE_TEX_COORDS_H


