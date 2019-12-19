#include <rendering/presentation_renderer.h>

#include <gpu/gl_utils.h>
#include <mesh_reconstruction.h>
#include <worker.h>
#include <gpu/garbage_collector.h>
#include <gpu/active_set.h>
#include <cuda/gpu_errchk.h>

using namespace std;
using namespace Eigen;

//c++ 11 string literals as measure to include shader files
//http://stackoverflow.com/questions/20443560/how-to-practically-ship-glsl-shaders-with-your-c-software
const string presentation_frag =
#include "shader/presentation.frag"
;
const string presentation_vert =
#include "shader/datastructure.glsl"

#include "shader/presentation.vert"
;

const string presentation_debug_frag =
#include "shader/presentation_debug.frag"
;
const string presentation_debug_vert =
#include "shader/presentation_debug.vert"
;

weak_ptr<gfx::GLSLProgram> PresentationRenderer::s_rgb_program_;

PresentationRenderer::PresentationRenderer(int width, int height)
		: show_wireframe(true),
		  color_mode(0),
		  shading_mode(0),
		  rendering_active_set_update_worker_(nullptr) {
}

PresentationRenderer::~PresentationRenderer() {

}

void PresentationRenderer::initInContext(int width, int height,
										 MeshReconstruction *reconstruction) {
	//map_    = map;
	width_  = width;
	height_ = height;
	initInContext(reconstruction);
}

void PresentationRenderer::initInContext(MeshReconstruction* reconstruction) {
	gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::initInContext]");

	//render depth shader
	if(s_rgb_program_.use_count()) {//if the shader exists already copy the reference
		rgb_program_ = s_rgb_program_.lock();

	} else {//otherwise create a new shader

		rgb_program_ = shared_ptr<gfx::GLSLProgram>(new gfx::GLSLProgram());
		rgb_program_->compileShader(presentation_frag,
		                            gfx::GLSLShader::GLSLShaderType::FRAGMENT,
		                            "presentation.frag" );
		rgb_program_->compileShader(presentation_vert,
		                            gfx::GLSLShader::GLSLShaderType::VERTEX,
		                            "presentation.vert" );
		rgb_program_->link();
		s_rgb_program_ = rgb_program_;
	}
	glGenVertexArrays(1, &VAO_);
	glBindVertexArray(VAO_);

	debug_program_ = shared_ptr<gfx::GLSLProgram>(new gfx::GLSLProgram());
	debug_program_->compileShader(presentation_debug_frag,
	                              gfx::GLSLShader::GLSLShaderType::FRAGMENT,
	                              "presentation_debug.frag" );
	debug_program_->compileShader(presentation_debug_vert,
	                              gfx::GLSLShader::GLSLShaderType::VERTEX,
	                              "presentation_debug.vert" );
	debug_program_->link();
	glGenVertexArrays(1, &debug_VAO_);
	glBindVertexArray(debug_VAO_);
}

void PresentationRenderer::render(GpuStorage* gpu_storage, ActiveSet *active_set, Matrix4f projection,
								  Matrix4f pose) {
	gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] before doing anything.");

	if(active_set == nullptr) {
		return;
	}
	if(active_set->meshlets.size() == 0) {
		return;
	}

	//end of debug

	rgb_program_->use();

	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapPresentation::render] Binding shader.");
	glBindVertexArray(VAO_);

	//after setting up the uniforms we set up all the possible
	glUniformMatrix4fv(0, 1, false, (GLfloat*) &pose);
	glUniformMatrix4fv(1, 1, false, (GLfloat*) &projection);

	glUniform1i(4, (int) show_wireframe);
	glUniform1i(5, (int) color_mode);
	glUniform1i(6, (int) shading_mode);
	glUniform1i(8, active_set->headers->getStartingIndex());

	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapPresentation::render] Setting up uniforms.");
	//the vertex buffer
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0,
	                 gpu_storage->vertex_buffer->getGlName());

	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapPresentation::render] Binding vertexBuffer");
	//bind texture coordinates
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1,
	                 gpu_storage->tex_pos_buffer->getGlName());
	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapPresentation::render] Binding texPosBuffer");

	//the triangle buffer
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2,
	                 gpu_storage->triangle_buffer->getGlName());
	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapPresentation::render] Binding triangleBuffer");

	//the patch information
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3,
	                 gpu_storage->patch_info_buffer->getGlName());

	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapPresentation::render] Binding buffers");

	//TODO: this is baaaad, how to get rid of this lock?
	//this blocks everything.
	//somehow we should prevent this from locking the whole list while rendering
	//if we mutex every single patch while rendering?
	//    -> impossible, the tex updates come all at once. and would be blocked
	// we need a non blocking memory system without stalling everything
	// one problem still is that the gpu memory could be changing while the
	//objects are being rendered.
	//mutexing all objects at once is bad tough
	//WAIT AND SEE

	//this is not too bad tough since rendering takes only 15ms at most (on the old gpu)
	//on the new one its way better.
	//activeSet->vectorUpdateMutex.lock();

	glUniform4f(7, 0, 0, 0, 1); // set color for meshes to black!
	//TODO: really render stuff!!!!! (with our new approach)
	//active_set->drawPatches();

	int count = active_set->meshlets.size();
	vector<GLint> firsts(count,0);
	vector<GLsizei> counts(count);
	for(int i=0;i<count;i++){
		MeshletGPU &meshlet = active_set->meshlets[i];
		//firsts[i] = meshlet.triangles->getStartingIndex()*3;
		counts[i] = meshlet.triangles->getSize()*3;

		//TODO: this would be cheaper to do it per texture in the GPU_STORAGE
		//make standard deviation texture resident
		meshlet.std_tex.tex->getTex()->makeResidentInThisThread();
		for(size_t i = 0;i<meshlet.textures.size();i++){
			// same for color textures
			meshlet.textures[i]->tex->getTex()->makeResidentInThisThread();
		}
	}
	//cout <<"PRESENTATION_RENDERER::RENDER why is there only the first vertex of each meshlet rendered? " << endl;
	glMultiDrawArrays(GL_TRIANGLES,&firsts[0],&counts[0], count); //GL_TRIANGLES
	//THERE IS A BUG WITH NVIDIA that prevents gl_VertexID from incrementing (it is initialized with the right start point though)


	/*
	//TODO: check out new driver version and see if this is fixed!
	for(int i=0;i<count;i++){
		glUniform1i(8, active_set->headers->getStartingIndex() + i);
		glDrawArrays(GL_TRIANGLES,firsts[i],counts[i]);

	}
	 */




	glFinish();//i know this isn't ideal for performance but we need to make sure that the data usage is
	// contained within this function so it won't be modified outside

	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapPresentation::render] After doing everything.");
}
