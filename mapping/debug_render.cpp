#include "debug_render.h"

#include <gfx/gl_utils.h>
#include <base/mesh_structure.h>

using namespace std;
using namespace Eigen;

const string debug_frag =
#include <rendering/shader/debugGeometry.frag>
;

const string debug_vert =
#include <rendering/shader/datastructure.glsl>
#include <rendering/shader/debugGeometry.vert>
;

RenderDebugInfo *that_one_debug_rendering_thingy;

RenderDebugInfo::RenderDebugInfo()
		: count_(0),
		  force_dst_geom(false),
		  rendered_patch(nullptr) {
	shader_ = make_shared<gfx::GLSLProgram>();
	shader_->compileShader(debug_frag, gfx::GLSLShader::GLSLShaderType::FRAGMENT,
	                       "debugGeometry.frag");
	shader_->compileShader(debug_vert, gfx::GLSLShader::GLSLShaderType::VERTEX,
	                       "debugGeometry.vert");
	shader_->link();
}

void RenderDebugInfo::render(Matrix4f proj, Matrix4f cam_pose) {
	if(rendered_patch == nullptr) {
		//return;
	}
	if(patches.empty()) {
		return;
	}
	shader_->use();

	gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] Binding triangleBuffer");
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, vertex_buffer);

	//bind texture coordinates
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, tex_pos_buffer);

	gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] Binding texPosBuffer");

	//the triangle buffer
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, triangle_buffer);

	//the patch information
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, info_buffer);

	glUniformMatrix4fv(0, 1, false, (GLfloat*) &cam_pose);
	glUniformMatrix4fv(1, 1, false, (GLfloat*) &proj);

	mutex_.lock();

	for(int i = 0; i < patches.size(); i++) {
		MeshPatch* p = patches[i].patch;
		glUniform4f(2, patches[i].r, patches[i].g, patches[i].b, 1);
		glUniform1i(3, -1);
		if(p != nullptr){
			int start = p->gpu.lock()->triangles->getStartingIndex();
			int nr = p->gpu.lock()->triangles->getSize();
			if(i != 0 && force_dst_geom) {
				glUniform1i(3, p->gpu.lock()->vertices_dest->getStartingIndex());
			}
			glDrawArrays(GL_TRIANGLES, start * 3, nr * 3);
			glDrawArrays(GL_POINTS, start * 3, nr * 3);

			if(i != 0)
				continue;

			for(shared_ptr<DoubleStitch> stitch : p->double_stitches) {
				shared_ptr<TriangleBufConnector> triangles = 
					stitch->triangles_gpu.lock();
				if(triangles == nullptr)
					continue; //doesn't need to be

				start = triangles->getStartingIndex();
				nr = triangles->getSize();
				glDrawArrays(GL_TRIANGLES, start * 3, nr * 3);

				glDrawArrays(GL_POINTS, start * 3, nr * 3);
				gfx::GLUtils::checkForOpenGLError(
					"[RenderMapPresentation::render] Binding texPosBuffer");
			}

			for(shared_ptr<TripleStitch> stitch : p->triple_stitches){
				shared_ptr<TriangleBufConnector> triangles = 
					stitch->triangles_gpu.lock();
				if(triangles == nullptr)
					continue; // doesn't need to be

				start = triangles->getStartingIndex();
				nr = triangles->getSize();
				glDrawArrays(GL_TRIANGLES, start * 3, nr * 3);

				glDrawArrays(GL_POINTS, start * 3, nr * 3);
				gfx::GLUtils::checkForOpenGLError(
					"[RenderMapPresentation::render] Binding texPosBuffer");
			}
		}
	}

	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapPresentation::render] Binding texPosBuffer");
	mutex_.unlock();

	glFinish();
}

void RenderDebugInfo::setIndexCount(int start_vertex, int vertex_count) {
	mutex_.lock();
	start_index_ = start_vertex;
	count_ = vertex_count;
	mutex_.unlock();
}

void RenderDebugInfo::setPatch(MeshPatch *patch) {
	rendered_patch = patch;
}

void RenderDebugInfo::addPatch(MeshPatch *patch, float r, float g, float b) {
	ShowPatch task = {r, g, b, 0, patch};
	patches.push_back(task);
}
