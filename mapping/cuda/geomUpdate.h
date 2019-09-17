#ifndef FILE_GEOM_UPDATE
#define FILE_GEOM_UPDATE

#include <vector>
#include <memory>

#include <cuda.h>
#include <cublas.h>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>

#include "gpuMeshStructure.h"
#include "gpuErrchk.h"

using namespace std;
using namespace Eigen;

struct Vertex;
class MeshPatch;

namespace gpu {

	struct TexAtlasSubSurface {
		cudaSurfaceObject_t surface;
		cv::Rect2i rect;
	};

	struct UpdateDescriptor {
		cv::Rect2i destination;
		cv::Rect2f destination_n;
		cv::Size2i destination_size;
		cudaSurfaceObject_t destination_geometry;

		cv::Rect2i source;
		cv::Rect2f source_n;
		cv::Size2i source_size;
		cudaSurfaceObject_t source_geometry; // const in a sense of we wont overwrite its pixel
												//we might have to change that to be able to build
												//the structure
		//Pointers or indices to triangles are missing for this part.
		
		cudaSurfaceObject_t destination_references;

		cv::Point2i reference_offset;

		int vertex_source_start_ind;
		int vertex_destination_start_ind;
		int vertex_count;

		int patch_info_slot;
		int triangle_slot;
		int triangle_count;

		bool update_texture = false;//when all the neighbours are updloaded we also update the texture
	};

	class GeometryUpdate {
	public:

		struct CalcCenterTask {
			GpuVertex *vertices;
			uint32_t count;
		};

		static void calcCenterAndRadius(vector<shared_ptr<MeshPatch>> &patches);

	private:

		static void calcCenterAndRadiusKernelCall_(dim3 grid, dim3 block,
		                                           size_t bytes,
		                                           CalcCenterTask *gpuTasks,
		                                           Vector4f *results);
	};

	class GeometryValidityChecks {
	public:

		struct VertexTask {
			uint32_t start_source;
			uint32_t start_dest;
			uint32_t size;
		};

		struct TriangleTask {
			cudaSurfaceObject_t lookup_tex;
			cv::Rect2i rect;
		};

		static void checkVertexValidity(const cudaSurfaceObject_t sensor,
		                                int width, int height, Matrix4f pose, 
		                                Matrix4f proj_pose, 
		                                vector<VertexTask> tasks,
		                                GpuVertex *vertices);

		static void checkTriangleValidity(vector<TriangleTask> tasks,
		                                  const cudaSurfaceObject_t sensor,
		                                  int width, int height,
		                                  Matrix4f pose, // because we want the vertex position relative to the camera
		                                  Matrix4f proj_pose, //to get the position of the point on the image.
		                                  GpuVertex *vertices,
		                                  Vector2f *tex_pos,
		                                  GpuTriangle *triangles,
		                                  GpuPatchInfo *patch_infos);
	};

	//TODO: get rid of this!!!!!
	int updateGeometry(const cudaSurfaceObject_t geometry_input, //the sensor input adapted by standard deviations
	                   int width, int height, //sensor resolution
	                   const vector<UpdateDescriptor> &descriptors,
	                   Vector4f cam_pos,
	                   Matrix4f pose, // because we want the vertex position relative to the camera
	                   Matrix4f proj_pose, //to get the position of the point on the image.
	                   GpuVertex *vertices, Vector2f *tex_pos,
	                   GpuTriangle* triangles, GpuPatchInfo* patch_infos);

} // namespace gpu

#endif