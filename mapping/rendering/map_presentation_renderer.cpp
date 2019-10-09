#include "map_presentation_renderer.h"

#include <gfx/gl_utils.h>
#include <mesh_reconstruction.h>
#include <worker.h>
#include <gfx/garbage_collector.h>
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
#include "shader/presentationDebug.frag"
;
const string presentation_debug_vert =
#include "shader/presentationDebug.vert"
;

weak_ptr<gfx::GLSLProgram> MapPresentationRenderer::s_rgb_program_;

MapPresentationRenderer::MapPresentationRenderer(int width, int height)
		: show_wireframe(true),
		  color_mode(0),
		  shading_mode(0),
		  rendering_active_set_update_worker_(nullptr) {
}

MapPresentationRenderer::~MapPresentationRenderer() {

}

void MapPresentationRenderer::initInContext(int width, int height, 
                                            MeshReconstruction *map) {
	map_ = map;
	width_ = width;
	height_ = height;
	initInContext();
}


void MapPresentationRenderer::initInContext() {
	gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::initInContext]");

	//render depth shader
	if(s_rgb_program_.use_count()){//if the shader exists already copy the reference
		rgb_program_ = s_rgb_program_.lock();

	}else{//otherwise create a new shader

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
	                              "presentationDebug.frag" );
	debug_program_->compileShader(presentation_debug_vert,
	                              gfx::GLSLShader::GLSLShaderType::VERTEX,
	                              "presentationDebug.vert" );
	debug_program_->link();
	glGenVertexArrays(1, &debug_VAO_);
	glBindVertexArray(debug_VAO_);
}

void MapPresentationRenderer::render(ActiveSet *active_set, Matrix4f projection, 
                                     Matrix4f pose) {
	gfx::GLUtils::checkForOpenGLError("[RenderMapPresentation::render] before doing anything.");

	if(active_set == nullptr) {
		return;
	}
	if(active_set->retained_double_stitches.size() == 0) {
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

	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapPresentation::render] Setting up uniforms.");
	//the vertex buffer
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0,
	                 active_set->gpu_geom_storage->vertex_buffer->getGlName());

	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapPresentation::render] Binding vertexBuffer");
	//bind texture coordinates
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1,
	                 active_set->gpu_geom_storage->tex_pos_buffer->getGlName());
	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapPresentation::render] Binding texPosBuffer");

	//the triangle buffer
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2,
	                 active_set->gpu_geom_storage->triangle_buffer->getGlName());
	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapPresentation::render] Binding triangleBuffer");

	//the patch information
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3,
	                 active_set->gpu_geom_storage->patch_info_buffer->getGlName());

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

	glUniform4f(7, 0, 0, 0, 1);
	active_set->drawPatches();

	glUniform4f(7, 0.7f, 0, 0, 1);
	active_set->drawDoubleStitches();

	glUniform4f(7, 0, 0, 0.7f, 1);
	active_set->drawTripleStitches();

	glFinish();//i know this isn't ideal for performance but we need to make sure that the data usage is
	// contained within this function so it won't be modified outside

	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapPresentation::render] After doing everything.");
}

void MapPresentationRenderer::renderInWindow(Matrix4f view, Matrix4f proj,
                                             bool render_visible_from_cam,
                                             GLFWwindow *root_context) {
	//return;//TODO: remove this debug measure
	//disgusting debug attempt
	bool disable_render_of_user_camera=false; //false means we render stuff seen by the user camera

	//get the visible ones
	if(render_visible_from_cam && !disable_render_of_user_camera) {
		Vector4f intrinsics(700, 700, 320, 240); //to see more of everything use
		Vector2f res(640, 480);
		//since in opengl the camera faces towards -z we have to flip it
		//for our visibility thingy
		Matrix4f flipz = Matrix4f::Identity();
		flipz(2, 2)= -1;
		Matrix4f cam_pose = (flipz * view).inverse();

		//we are doing this in a secondary thread
		//lets create a worker thread if there is none
		if(rendering_active_set_update_worker_ == nullptr &&
		   !disable_render_of_user_camera) {//DEBUG
			GLFWwindow **context = new GLFWwindow*;
			auto initializer = [&](GLFWwindow *parent_context, 
			                       GLFWwindow **new_context) {
				//init opengl context here
				glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
				glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
				glfwWindowHint(GLFW_VISIBLE, 0);
				*new_context = glfwCreateWindow(640, 480, "HOPE U NO VISIBLE",
				                                nullptr, parent_context);
				glfwMakeContextCurrent(*new_context);
				glewExperimental = GL_TRUE;
				glewInit();
				glGetError();
				//TODO: it feels weird calling this method within this class
				map_->information_renderer.initInContext(640, 480, map_);
			};
			auto cleaner = [&](GLFWwindow **context) {
				map_->fbo_storage_.forceGarbageCollect();
				map_->garbage_collector_->forceCollect();
				glFinish();
				glfwDestroyWindow(*context);
				delete context;
			};
			function<void ()> init_function = bind(initializer, root_context, context);
			function<void ()> cleanup_function = bind(cleaner, context);
			rendering_active_set_update_worker_ =
					new Worker(init_function, "UpSetRend", cleanup_function);
		}
		auto task = [&](Matrix4f inv_cam_pose, Vector4f intrinsics,
		                Vector2f res, float view_distance) {
			vector<shared_ptr<MeshPatch>> visible_patches =
					map_->octree_.getObjects(inv_cam_pose, intrinsics, res, view_distance);

			cudaDeviceSynchronize();
			gpuErrchk(cudaPeekAtLastError());
			shared_ptr<ActiveSet> active_set =
					map_->gpu_geom_storage_.makeActiveSet(visible_patches, map_);

			//cleanup VBO and VAOs that are deleted but only used within this thread
			map_->cleanupGlStoragesThisThread_();
			map_->garbage_collector_->collect();
			glFinish();
			cudaDeviceSynchronize();
			gpuErrchk(cudaPeekAtLastError());
			//This might be obsolete when the makeActiveSet is changed to append all
			//the references
			//is this really needed?
			//activeSet->securePatchTextures();
			//store the active set
			map_->active_set_rendering_mutex_.lock();
			map_->active_set_rendering_ = active_set;
			map_->active_set_rendering_mutex_.unlock();
		};

		//the update worker is not queuing the update tasks.
		rendering_active_set_update_worker_->setNextTask(bind(task, cam_pose, 
		                                                      intrinsics,
		                                                      res, 1.0f));

	} else {
		map_->active_set_rendering_mutex_.lock();
		map_->active_set_rendering_ = nullptr;
		map_->active_set_rendering_mutex_.unlock();
	}

	//at last we render the active set
	map_->active_set_update_mutex.lock();
	shared_ptr<ActiveSet> active_set_work = map_->active_set_update;
	map_->active_set_update_mutex.unlock();

	map_->active_set_rendering_mutex_.lock();
	shared_ptr<ActiveSet> active_set_display;
	if(!disable_render_of_user_camera) {
		active_set_display = map_->active_set_rendering_;
	}
	map_->active_set_rendering_mutex_.unlock();

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	//set of patches excluded for the low detail render
	vector<shared_ptr<ActiveSet>> sets;
	if(render_visible_from_cam){
		render(active_set_work.get(), proj, view);
		sets.push_back(active_set_work);
	}

	if(active_set_display != nullptr && render_visible_from_cam) {
		//store it in the list to exlude it from the low detail render
		sets.push_back(active_set_display);
		//also render it
		render(active_set_display.get(), proj, view);
	}

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	map_->low_detail_renderer.renderExceptForActiveSets(sets, proj, view);
}