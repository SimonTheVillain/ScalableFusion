#ifndef FILE_LOW_DETAIL_RENDERER_H
#define FILE_LOW_DETAIL_RENDERER_H

#include <memory>
#include <vector>
#include <mutex>

#include <cublas.h>
#include <Eigen/Eigen>

#include <gfx/gl_utils.h>
#include <gfx/shader.h>
#include <cuda/gpu_mesh_structure.h>

using namespace std;
using namespace Eigen;

class MeshPatch;
struct DoubleStitch;
struct TripleStitch;
class ActiveSet;

template<typename T>
struct GlCudaBuffer {

	GlCudaBuffer(size_t size);

	~GlCudaBuffer();

	GLuint gl_name;
	//todo: cuda resource
	size_t nr_elements;
	cudaGraphicsResource_t cuda_resource;
	T *cuda_ptr;
};

struct CoarseTriangle {

	CoarseTriangle(shared_ptr<MeshPatch> p1,
	               shared_ptr<MeshPatch> p2,
	               shared_ptr<MeshPatch> p3);

	~CoarseTriangle();

	bool isValid();

	bool isConnectingSame3Patches(shared_ptr<MeshPatch> p1,
	                              shared_ptr<MeshPatch> p2,
	                              shared_ptr<MeshPatch> p3);

	bool flipToFacePos(Vector3f pos);

	weak_ptr<MeshPatch> patches[3];
	weak_ptr<DoubleStitch> double_stitches[3];
	weak_ptr<TripleStitch> triple_stitches[3];
};

/**
 * @brief The LowDetailPoint class
 */
class LowDetailPoint {
public:

	bool isNeighbourWith(shared_ptr<LowDetailPoint> point);

	void addCoarseTriangle(shared_ptr<CoarseTriangle> coarse_triangle);

	shared_ptr<CoarseTriangle> getCoarseTriangleWith(shared_ptr<MeshPatch> p1,
	                                                 shared_ptr<MeshPatch> p2,
	                                                 shared_ptr<MeshPatch> p3);

	void cleanupCoarseTriangles();

	mutex coarse_triangle_mutex;
	vector<shared_ptr<CoarseTriangle>> triangle_within_neighbours;

	int index_within_coarse = -1;

	Vector4f average_color;
};

/**
 * @brief The LowDetailRenderer class
 * This class should encapsule the rendering of patches which are not loaded onto the gpu
 * each of the patch will only be represented by only one vertex within a mesh
 * Hopefully this will not introduce too many mutex locks so that everything becomest choppy and stuff.
 */
class LowDetailRenderer {
public:

	LowDetailRenderer() { };

	~LowDetailRenderer() { };

	void initInGlContext();

	void addPatches(vector<shared_ptr<MeshPatch>> &patches_in,
	                Vector3f cam_pos);

	void updateColorForPatches(vector<shared_ptr<MeshPatch>> &patches_in);

	//TODO: split this up:
	void renderExceptForActiveSets(vector<shared_ptr<ActiveSet>> &sets, 
	                               Matrix4f proj, 
	                               Matrix4f cam_pose);

	//into the following:
	void updateMaskForActiveSets(vector<shared_ptr<ActiveSet>> &sets);

	//render the color data
	void renderColor( Matrix4f proj, Matrix4f cam_pose);

	//render the geometry for when we click onto the surface:
	void renderGeometry(Matrix4f proj, Matrix4f cam_pose);

	void updateAllPatches();

	void downloadCurrentGeometry(vector<GpuCoarseVertex> &vertices, 
	                             vector<int> &indices);

	vector<weak_ptr<MeshPatch>> patches;
	vector<weak_ptr<CoarseTriangle>> coarse_triangles;

private:

	static weak_ptr<gfx::GLSLProgram> s_shader_;
	shared_ptr<gfx::GLSLProgram> shader_;
	static weak_ptr<gfx::GLSLProgram> s_geometry_shader_;
	shared_ptr<gfx::GLSLProgram> geometry_shader_;

	shared_ptr<gfx::GLSLProgram> debug_shader_;

	mutex replacing_buffers_;

	GLuint VAO_ = 0;

	mutex modifying_buffers_;
	shared_ptr<GlCudaBuffer<int>> index_buffer_;
	shared_ptr<GlCudaBuffer<GpuCoarseVertex>> vertex_buffer_;
	shared_ptr<GlCudaBuffer<int>> visibility_buffer_;
	int nr_indices_;

	bool new_buffers_ = false;

	vector<int> invisible_in_last_frame_;
};

#endif // FILE_LOW_DETAIL_RENDERER_H
