#ifndef FILE_RENDERABLE_MODEL_H
#define FILE_RENDERABLE_MODEL_H

#include <memory>

#include <Eigen/Eigen>

#include <gfx/shader.h>

using namespace std;
using namespace Eigen;

//TODO: rename this!!!
class RenderVerySimpleModel {
public:
	struct Vertex {
		Vertex(Vector4f pos, Vector4f color) 
				: p(pos),
				  c(color) {
		}
		Vector4f p;
		Vector4f c;
	};

	RenderVerySimpleModel();

	~RenderVerySimpleModel();

	void setMesh(vector<Vector4f,aligned_allocator<Vector4f>> &vertices, vector<unsigned int> indices);

	void render(Matrix4f &cam_proj);

	GLuint primitiveType = GL_LINES;

	Matrix4f pose  = Matrix4f::Identity();
	Vector4f color = Vector4f(1, 0, 0, 1);

private:

	GLuint vertex_buffer_;
	GLuint element_buffer_;
	int count_;

	GLuint VAO_;
	static weak_ptr<gfx::GLSLProgram> s_flat_program_;
	shared_ptr<gfx::GLSLProgram> unlit_program_;
};

class CameraFrustrumRenderableModel : public RenderVerySimpleModel {
public:
	CameraFrustrumRenderableModel(Vector4f color, Vector4f intrinsics, 
	                              Vector2f resolution, float near_clipping_plane, 
	                              float far_clipping_plane);

	~CameraFrustrumRenderableModel() { }
};

class WireSphereModel : public RenderVerySimpleModel {
public:

	WireSphereModel(Vector4f color, Vector4f pos, float radius);

	void setPosition(Vector3f position);

	void setRadius(float radius);

//TODO: implement this!
private:

	Vector3f color_;
	Vector3f pos_;
	Vector3f scale_;

	void updatePose_();

};

#endif // FILE_RENDERABLE_MODEL_H
