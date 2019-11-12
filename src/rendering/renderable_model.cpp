#include <rendering/renderable_model.h>

#include <iostream>

using namespace std;
using namespace Eigen;

const string unlit_frag =
#include "shader/unlit.frag"
;

const string unlit_vert =
#include "shader/unlit.vert"
;

weak_ptr<gfx::GLSLProgram> RenderVerySimpleModel::s_flat_program_;

RenderVerySimpleModel::RenderVerySimpleModel() 
		: vertex_buffer_(0),
		  element_buffer_(0),
		  VAO_ (0) {
	if(s_flat_program_.use_count() == 0) {
		unlit_program_ = shared_ptr<gfx::GLSLProgram>(new gfx::GLSLProgram());
		unlit_program_->compileShader(unlit_frag,
		                              gfx::GLSLShader::GLSLShaderType::FRAGMENT,
		                              "unlit.frag" );
		unlit_program_->compileShader(unlit_vert,
		                              gfx::GLSLShader::GLSLShaderType::VERTEX,
		                              "unlit.vert" );
		unlit_program_->link();
		s_flat_program_ = unlit_program_;

	} else {
		unlit_program_ = s_flat_program_.lock();
	}
}

RenderVerySimpleModel::~RenderVerySimpleModel() {
	if(VAO_ != 0) {
		cout << "TODO: delete all these stupid buffers" << endl;
	}
}

void RenderVerySimpleModel::setMesh(vector<Vector4f,aligned_allocator<Vector4f>> &vertices,
                                    vector<unsigned int> indices) {
	unlit_program_->use();
	glGenVertexArrays(1, &VAO_);
	glBindVertexArray(VAO_);

	glGenBuffers(1, &vertex_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vector4f),
	             &vertices[0], GL_STATIC_DRAW);

	glVertexAttribPointer(0,                // attribute
	                      4,                // size
	                      GL_FLOAT,         // type
	                      GL_FALSE,         // normalized?
	                      sizeof(Vector4f), // stride (0 should work as well)
	                      (void*) 0         // array buffer offset
	);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &element_buffer_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element_buffer_);

	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * indices.size(),
	             &indices[0], GL_STATIC_DRAW);
	count_ = indices.size();
}

void RenderVerySimpleModel::render(Matrix4f &cam_proj) {
	unlit_program_->use();
	glBindVertexArray(VAO_);
	Matrix4f proj_cam_model = cam_proj * pose;
	glUniformMatrix4fv(0, 1, false, (GLfloat*) &proj_cam_model);
	glUniform4fv(1, 1, (GLfloat*) &color[0]);

	gfx::GLUtils::checkForOpenGLError(
			"[RenderVerySimpleModel::render] Right before rendering "
			"some very simple geometry.");

	glDrawElements(primitiveType, count_, GL_UNSIGNED_INT, (void*) 0);
	gfx::GLUtils::checkForOpenGLError(
			"[RenderVerySimpleModel::render] Right at rendering "
			"some very simple geometry.");
}

CameraFrustrumRenderableModel::CameraFrustrumRenderableModel(
		Vector4f color, Vector4f intrinsics, Vector2f resolution, 
		float near_clipping_plane, float far_clipping_plane) {

	float fx = intrinsics[0];
	float fy = intrinsics[1];
	float cx = intrinsics[2];
	float cy = intrinsics[3];

	float rx=resolution[0];
	float ry=resolution[1];

	Vector4f p[4] = {Vector4f(        cx / fx,         cy / fy, 1, 0), 
	                 Vector4f(-(rx - cx) / fx,         cy / fy, 1, 0),
	                 Vector4f(        cx / fx, -(ry - cy) / fy, 1, 0),
	                 Vector4f(-(rx - cx) / fx, -(ry - cy) / fy, 1, 0)};

	Vector4f homo(0, 0, 0, 1);
	vector<Vector4f,aligned_allocator<Vector4f>> vertices = {p[0] * near_clipping_plane + homo,
	                             p[1] * near_clipping_plane + homo,
	                             p[2] * near_clipping_plane + homo,
	                             p[3] * near_clipping_plane + homo,
	                             p[0] * far_clipping_plane  + homo,
	                             p[1] * far_clipping_plane  + homo,
	                             p[2] * far_clipping_plane  + homo,
	                             p[3] * far_clipping_plane  + homo};
	vector<unsigned int> indices = {0, 1, 2, 3, 0, 2, 3, 1,
	                                0, 4, 1, 5, 2, 6, 3, 7,
	                                4, 5, 6, 7, 4, 6, 7, 5};
	setMesh(vertices, indices);
}

WireSphereModel::WireSphereModel(Vector4f color, Vector4f pos, float radius) {
	int vertex_count = 100;
	vector<Vector4f,aligned_allocator<Vector4f>> vertices(vertex_count * 3);
	vector<unsigned int> indices(vertex_count * 3 * 2);

	for(int i = 0; i < vertex_count; i++) {
		float s = sin(M_PI * 2.0 * float(i) / float(vertex_count));
		float c = cos(M_PI * 2.0 * float(i) / float(vertex_count));
		vertices[i + vertex_count * 0] = Vector4f(s, c, 0, 1);
		vertices[i + vertex_count * 1] = Vector4f(s, 0, c, 1);
		vertices[i + vertex_count * 2] = Vector4f(0, s, c, 1);

		indices[(i + vertex_count * 0) * 2 + 0] = vertex_count * 0 + i;
		indices[(i + vertex_count * 0) * 2 + 1] = vertex_count * 0 + i + 1;
		indices[(i + vertex_count * 1) * 2 + 0] = vertex_count * 1 + i;
		indices[(i + vertex_count * 1) * 2 + 1] = vertex_count * 1 + i + 1;
		indices[(i + vertex_count * 2) * 2 + 0] = vertex_count * 2 + i;
		indices[(i + vertex_count * 2) * 2 + 1] = vertex_count * 2 + i + 1;
		if(i == (vertex_count - 1)) {
			indices[(i + vertex_count * 0) * 2 + 1] = vertex_count * 0 + 0 + 0;
			indices[(i + vertex_count * 1) * 2 + 1] = vertex_count * 1 + 0 + 0;
			indices[(i + vertex_count * 2) * 2 + 1] = vertex_count * 2 + 0 + 0;
		}
	}
	setMesh(vertices, indices);
	pose = Matrix4f::Identity();
}

void WireSphereModel::updatePose_() {
	pose = Matrix4f::Identity();
	pose.block<3, 1>(0, 3) = pos_;
	pose(0, 0) = scale_[0];
	pose(1, 1) = scale_[1];
	pose(2, 2) = scale_[2];
}

void WireSphereModel::setPosition(Vector3f position) {
	pos_ = position;
	updatePose_();
}

void WireSphereModel::setRadius(float radius) {
	scale_[0] = scale_[1] = scale_[2] = radius;
	updatePose_();
}
