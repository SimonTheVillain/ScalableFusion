#include <rendering/information_renderer.h>

#include <gpu/gl_utils.h>
#include <mesh_reconstruction.h>
#include <gpu/active_set.h>

using namespace std;
using namespace Eigen;

const string information_frag =
#include "shader/information.frag"//todo: update this file with stuff from renderDepth.frag
;
const string information_vert =
#include "shader/datastructure.glsl"

#include "shader/information.vert"
;

const string coordinates_frag =
#include "shader/coordinates.frag"
;

const string coordinates_vert =
#include "shader/datastructure.glsl"

#include "shader/coordinates.vert"
;

const string triangle_ref_depth_frag =
#include "shader/triangle_ref_depth.frag"
;


const string triangle_ref_depth_vert =
#include "shader/datastructure.glsl"

#include "shader/triangle_ref_depth.vert"
;

const string unified_info_frag =
#include "shader/unified_info.frag"
;

const string unified_info_vert =
#include "shader/datastructure.glsl"

#include "shader/unified_info.vert"
;

mutex InformationRenderer::shader_mutex_;
weak_ptr<gfx::GLSLProgram> InformationRenderer::s_depth_program_;
weak_ptr<gfx::GLSLProgram> InformationRenderer::s_triangle_reference_program_;
weak_ptr<gfx::GLSLProgram> InformationRenderer::s_triangle_ref_depth_program_;

InformationRenderer::InformationRenderer(int width, int height)
		: width_(width), 
		  height_(height) { 
}

InformationRenderer::~InformationRenderer() {
}

void InformationRenderer::initInContext(int width, int height,
										GarbageCollector *garbage_collector) {
	//map_    = map;
	width_  = width;
	height_ = height;
	initInContext(garbage_collector);
}

void InformationRenderer::initInContext(GarbageCollector* garbage_collector) {
	//assert(0); //make this multithreading capable: (it depends on the VAO i think)
	//but also on the texture..... both of them need to be thread specific
	gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] start");

	shader_mutex_.lock();
	//render depth shader
	if(s_depth_program_.use_count()) {//if the shader exists already copy the reference
		depth_program = s_depth_program_.lock();

	} else {//otherwise create a new shader
		depth_program = shared_ptr<gfx::GLSLProgram>(new gfx::GLSLProgram());
		depth_program->compileShader(information_frag,
		                             gfx::GLSLShader::GLSLShaderType::FRAGMENT,
		                             "information.frag" );
		depth_program->compileShader(information_vert, 
		                             gfx::GLSLShader::GLSLShaderType::VERTEX,
		                             "information.vert" );
		depth_program->link();
		s_depth_program_ = depth_program;
	}
	shader_mutex_.unlock();

	gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] after setting up information shader");

	/**
	 * This shader is supposed to create texture coordinates and indices of the triangles  for a texture.
	 * the indices for the triangles as well as the indices
	 */

	shader_mutex_.lock();
	if(s_triangle_reference_program_.use_count()) {
		triangle_reference_program_ = s_triangle_reference_program_.lock();
	} else {
		triangle_reference_program_ = shared_ptr<gfx::GLSLProgram>(new gfx::GLSLProgram());
		triangle_reference_program_->compileShader(
				coordinates_frag,
				gfx::GLSLShader::GLSLShaderType::FRAGMENT,
				"coordinates.frag" );
		triangle_reference_program_->compileShader(
				coordinates_vert,
				gfx::GLSLShader::GLSLShaderType::VERTEX,
				"coordinates.vert" );
		triangle_reference_program_->link();
		s_triangle_reference_program_ = triangle_reference_program_;
	}
	shader_mutex_.unlock();

	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapInformations::initInContext] after setting up coordinates shader");

	shader_mutex_.lock();

	if(s_triangle_ref_depth_program_.use_count()) {
		triangle_ref_depth_prog_ = s_triangle_ref_depth_program_.lock();
	} else {
		//create the shader i was looking for
		triangle_ref_depth_prog_ = shared_ptr<gfx::GLSLProgram>(new gfx::GLSLProgram());
		triangle_ref_depth_prog_->compileShader(
				triangle_ref_depth_frag,
				gfx::GLSLShader::GLSLShaderType::FRAGMENT,
				"triangle_ref_depth.frag" );
		triangle_ref_depth_prog_->compileShader(
				triangle_ref_depth_vert,
				gfx::GLSLShader::GLSLShaderType::VERTEX,
				"triangle_ref_depth.vert" );
		triangle_ref_depth_prog_->link();
		s_triangle_ref_depth_program_ = triangle_ref_depth_prog_;
	}
	shader_mutex_.unlock();

	//TODO: get rid of these overflowing error checks
	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapInformations::initInContext] after setting up triangle Ref shader");

	unified_info_prog_ = make_shared<gfx::GLSLProgram>();

	assert(gfx::GLUtils::checkForOpenGLError(
			"[RenderMapInformations::initInContext] before setting up unified info shader") == GL_NO_ERROR);

	unified_info_prog_->compileShader(unified_info_frag,
	                                  gfx::GLSLShader::GLSLShaderType::FRAGMENT,
	                                  "unified_info.frag");

	assert(gfx::GLUtils::checkForOpenGLError(
			"[RenderMapInformations::initInContext] during setting up unified info shader") == GL_NO_ERROR);

	unified_info_prog_->compileShader(unified_info_vert,
	                                  gfx::GLSLShader::GLSLShaderType::VERTEX,
	                                  "unified_info.vert");

	assert(gfx::GLUtils::checkForOpenGLError(
			"[RenderMapInformations::initInContext] after setting up unified info shader") == GL_NO_ERROR);

	unified_info_prog_->link();

	assert(gfx::GLUtils::checkForOpenGLError(
			"[RenderMapInformations::initInContext] after linking up unified info shader")==GL_NO_ERROR);

	//after creating the shader which should only be done once we
	//create the struct
	//**************************************NEW*******************************
	thread::id id = this_thread::get_id();

	//if this thread does not have the texture and stuff (its very certain it has not)
	//we create all the resources
	if(per_thread_gl_objects_.count(id) == 0) {
		PerThread_ pt;

		//now do everything that also is done below:
		{
			gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::initInContext] before setting up framebuffers");
			//FBO creation:
			glGenFramebuffers(1, &pt.depth_FBO);
			glBindFramebuffer(GL_FRAMEBUFFER, pt.depth_FBO);

			//texture creation
			//depth:
			pt.depth_texture = make_shared<gfx::GpuTex2D>(garbage_collector,
			                                              GL_RGBA32F,GL_RGBA,
			                                              GL_FLOAT,
			                                              width_, height_, false);
			pt.depth_texture->name = "perThread depth texture";
			//debug:
			pt.std_texture = make_shared<gfx::GpuTex2D>(garbage_collector,
			                                            GL_RGBA32F,
			                                            GL_RGBA,
			                                            GL_FLOAT,
			                                            width_, height_, false);
			pt.std_texture->name = "perThread std Texture";

			glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
			                     pt.depth_texture->getGlName(), 0);

			glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, 
			                     pt.std_texture->getGlName(), 0);

			gfx::GLUtils::checkForOpenGLError(
					"[RenderMapInformations::initInContext] setting up depth buffer");

			//setting up the depth buffer
			glGenTextures(1, &pt.depth_buffer_tex);
			glBindTexture(GL_TEXTURE_2D, pt.depth_buffer_tex);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, width_, height_, 0,
			             GL_DEPTH_COMPONENT, GL_FLOAT, 0);
			glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
			                     pt.depth_buffer_tex, 0);
			//TODO: (or to think of)binding the depth buffer to cuda
			//http://stackoverflow.com/questions/32611002/opengl-depth-buffer-to-cuda
			//https://devtalk.nvidia.com/default/topic/877969/opengl-z-buffer-to-cuda/

			gfx::GLUtils::checkForOpenGLError(
					"[RenderMapInformations::initInContext] setting up z buffer");

			//TODO: it might not work if the
			GLenum draw_buffers[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
			glDrawBuffers(2, draw_buffers); // "1" is the size of DrawBuffers

			gfx::GLUtils::checkOpenGLFramebufferStatus(
					"Initializing Render map Informations");
			gfx::GLUtils::checkForOpenGLError(
					"[RenderMapInformations::initInContext] setting up framebuffer");

			//todo add depth buffer...
			glBindFramebuffer(GL_FRAMEBUFFER, 0);//unbind the framebuffer
			gfx::GLUtils::checkForOpenGLError(
					"[RenderMapInformations::initInContext] while setting up framebuffer");

			//create the VAO just because it is necessary for rendering:
			glGenVertexArrays(1, &pt.depth_VAO);
			glBindVertexArray(pt.depth_VAO);
		}

		//Create textures for the combined/unified rendering stuff.
		{
			//TODO: some of these textures are not needed or have wrong resolution
			glGenFramebuffers(1, &pt.combined_FBO);
			glBindFramebuffer(GL_FRAMEBUFFER, pt.combined_FBO);

			pt.z_texture = make_shared<gfx::GpuTex2D>(garbage_collector,
			                                          GL_R32F, GL_RED, GL_FLOAT,
			                                 	        width_, height_, false);
			pt.z_texture->name = "per thread zTexture";
			pt.color_texture = make_shared<gfx::GpuTex2D>(garbage_collector,
			                                              GL_RGBA32F, GL_RGBA, 
			                                              GL_FLOAT, 
			                                              width_, height_, false);
			pt.color_texture->name = "per thread colorTexture";
			pt.normal_texture = make_shared<gfx::GpuTex2D>(garbage_collector,
			                                              GL_RGBA32F, GL_RGBA, 
			                                              GL_FLOAT, width_, height_, 
			                                              false);
			pt.normal_texture->name = "per thread normalTexture";
			//make this an int32.... this is going to be hard enough
			pt.label_texture = make_shared<gfx::GpuTex2D>(garbage_collector,
			                                              GL_RGBA32F, GL_RGBA, 
			                                              GL_FLOAT, width_, height_,
			                                              false);
			pt.label_texture->name = "per thread label texture";
			glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
			                     pt.z_texture->getGlName(), 0);
			glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, 
			                     pt.color_texture->getGlName(), 0);
			glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, 
			                     pt.normal_texture->getGlName(), 0);
			glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, 
			                     pt.label_texture->getGlName(), 0);

			glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, 
			                     pt.depth_buffer_tex, 0);
			GLenum draw_buffers[4] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1,
			                          GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3};
			glDrawBuffers(4, draw_buffers); // "1" is the size of DrawBuffers

			glGenVertexArrays(1, &pt.combined_VAO);
			glBindVertexArray(pt.combined_VAO);
		}

		per_thread_gl_objects_[id] = pt;
	}
	return;
}

shared_ptr<gfx::GpuTex2D> InformationRenderer::getDepthTexture() {
	return per_thread_gl_objects_[this_thread::get_id()].depth_texture;
}

shared_ptr<gfx::GpuTex2D> InformationRenderer::getStdTexture() {
	return per_thread_gl_objects_[this_thread::get_id()].std_texture;
}

void InformationRenderer::renderDepth(ActiveSet *active_set,
									  GpuStorage* gpu_storage,
									  Matrix4f projection, Matrix4f pose) {

	PerThread_ pt = per_thread_gl_objects_[this_thread::get_id()];
	//activate VBO
	glBindFramebuffer(GL_FRAMEBUFFER, pt.depth_FBO);

	//there propably is a fbo bound somewhere already

	glViewport(0, 0, width_, height_);
	glClearColor(NAN, NAN, NAN, NAN);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	depth_program->use();

	//after setting up the uniforms we set up all the possible
	Matrix4f pose_tmp = pose.inverse();
	Matrix4f projection_tmp = projection.inverse();
	glUniformMatrix4fv(0, 1, false, (GLfloat*) &pose_tmp);
	glUniformMatrix4fv(1, 1, false, (GLfloat*) &projection);
	glUniformMatrix4fv(4, 1, false, (GLfloat*) &projection_tmp);//for as soon as i added this...

	if(active_set){
		if(active_set->meshlets.size() == 0) {
			//the active set is empty
			glFinish();
			return;
		}
	}else{
		//in case the active set is invalid
		glFinish();
		return;
	}
	//the vertex buffer
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 
	                 gpu_storage->vertex_buffer->getGlName());

	//bind texture coordinates
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1,
	                 gpu_storage->tex_pos_buffer->getGlName());

	//the triangle buffer
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2,
					 gpu_storage->triangle_buffer->getGlName());

	//the patch information
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3,
					 gpu_storage->patch_info_buffer->getGlName());

	cout << "LOOK AT WHAT THE PRESENTATION RENDERER DID AND REPEAT EXACTLY THAT" << endl;
	assert(0); // the rendering method is not operational right now!!!!! drawMulti stuff is needed
	//active_set->drawEverything();
	gfx::GLUtils::checkForOpenGLError("[InformationRenderer::renderDepth] "
	                                  "after issuing all render commands");

	glFinish();
}

void InformationRenderer::bindRenderTriangleReferenceProgram(GpuStorage* gpu_storage) {
	triangle_reference_program_->use();
	gfx::GLUtils::checkForOpenGLError(
			"[InformationRenderer::bindRenderTriangleReferenceProgram] "
			"Initializing the triangle reference program..");

	//the vertex buffer
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0,
					 gpu_storage->vertex_buffer->getGlName());

	//bind texture coordinates
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1,
					 gpu_storage->tex_pos_buffer->getGlName());

	//the triangle buffer
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2,
					 gpu_storage->triangle_buffer->getGlName());

	//the patch information
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3,
					 gpu_storage->patch_info_buffer->getGlName());

	gfx::GLUtils::checkForOpenGLError(
			"[InformationRenderer::bindRenderTriangleReferenceProgram] "
			"Binding the buffers.");
}


void InformationRenderer::renderReference(	MeshletGPU* meshlet_gpu,
											shared_ptr<TexCoordBufConnector> coords,
											shared_ptr<TexAtlasPatch> ref_tex  ){
	//what is supposed to be rendered here is the 3 (local) vertex indices +
	// 3x 8 bit barycentric coordinates packed into the last float

	cv::Rect2i r = ref_tex->getRect();
	glBindFramebuffer(GL_FRAMEBUFFER,ref_tex->getFBO());
	gfx::GLUtils::checkOpenGLFramebufferStatus(
			"ScaleableMap::finalizeGeomTexOfNewPatches");
	glEnable(GL_SCISSOR_TEST);
	glScissor(r.x, r.y, r.width, r.height);
	glViewport(r.x, r.y, r.width, r.height);

	//reinterpret cast
	int minus1i = -1;
	float minus1 = *((float*) (&minus1i));//convert the -1 to a float in a bitwise fashion

	glClearColor(minus1, 1, 0, 0);//DEBUG: paint it green
	glClear(GL_COLOR_BUFFER_BIT);
	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapInformations::renderTriangleReferencesForPatch]");

	//render here!
	int start_ind_tris = meshlet_gpu->triangles->getStartingIndex();
	int start_ind_tex_coords = coords->getStartingIndex();
	int size = meshlet_gpu->triangles->getSize();
	glUniform1i(0,start_ind_tris);
	glUniform1i(1,start_ind_tex_coords);
	glDrawArrays(GL_TRIANGLES, 0, size * 3);

	glDisable(GL_SCISSOR_TEST);



}
void InformationRenderer::renderTriangleReferencesForPatch(
		ActiveSet *active_set,GpuStorage* gpu_storage, shared_ptr<Meshlet> &patch,
		shared_ptr<MeshTexture> &target_texture) {


	assert(0); //todo: reinsert this logic... right now it is not feasible
	/*
	shared_ptr<MeshTextureGpuHandle> gpu_tex = target_texture->gpu.lock();
	if(gpu_tex == nullptr) {
		cout << "[InformationRenderer::renderTriangleReferencesForPatch] "
		        "There is no texture on the gpu" << endl;
		assert(0);
	}
	//clear the list of dependencies since we are setting up a new one
	gpu_tex->ref_tex_dependencies.clear();
	cv::Rect2i r = target_texture->getLookupRect();
	glBindFramebuffer(GL_FRAMEBUFFER, gpu_tex->ref_tex->getFBO());

	gfx::GLUtils::checkOpenGLFramebufferStatus(
			"ScaleableMap::finalizeGeomTexOfNewPatches");
	glEnable(GL_SCISSOR_TEST);
	glScissor(r.x, r.y, r.width, r.height);
	glViewport(r.x, r.y, r.width, r.height);

	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapInformations::renderTriangleReferencesForPatch]");
	int minus1i = -1;
	float minus1 = *((float*) (&minus1i));//convert the -1 to a float in a bitwise fashion
	//(reinterpret cast)

	glClearColor(minus1, 1, 0, 0);//DEBUG: paint it green
	glClear(GL_COLOR_BUFFER_BIT);
	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapInformations::renderTriangleReferencesForPatch]");
	//This todo still applies, especially since we needed to do a scissor thingy to
	//get this done correctly
	if(patch->isPartOfActiveSetWithNeighbours(active_set)) {//this should not not be the case
		//render the patch.
		shared_ptr<MeshletGpuHandle> patch_gpu = patch->gpu.lock();

		if(patch_gpu == nullptr) {
			cout << "[InformationRenderer::renderTriangleReferencesForPatch]"
			        " This should not happen when this is invoked for a patch"
			        " which is part of an active set" << endl;
			assert(0);
			return;
		}

		size_t slot = patch_gpu->triangles->getStartingIndex();
		size_t size = patch_gpu->triangles->getSize();
		//it does not seem like we are running out of empty elements

		gfx::GLUtils::checkForOpenGLError(
				"[RenderMapInformations::renderTriangleReferencesForPatch]");
		glDrawArrays(GL_TRIANGLES, slot * 3, size * 3);

		gfx::GLUtils::checkForOpenGLError(
				"[RenderMapInformations::renderTriangleReferencesForPatch]");

		//Add the dependency of this central patch
		MeshTextureGpuHandle::Dependency dependency;
		dependency.triangle_position_on_gpu = patch_gpu->triangles->getStartingIndex();
		dependency.triangles_version = patch->triangles_version;
		dependency.geometry = patch;
		gpu_tex->ref_tex_dependencies.push_back(dependency);

		patch->double_stitch_mutex.lock();
		for(size_t i = 0; i < patch->double_stitches.size(); i++) {
			DoubleStitch &stitch = *patch->double_stitches[i].get();
			if(stitch.patches[0].lock() == patch) {
				//only render when patch is main patch
				shared_ptr<TriangleBufConnector> stitch_gpu = stitch.triangles_gpu.lock();
				if(stitch_gpu == nullptr) {
					cout << "[InformationRenderer::renderTriangleReferencesForPatch]"
					        " This should not happen when this is invoked for a patch"
					        " which is part of an active set" << endl;
					assert(0);
					return;
				}

				slot = stitch_gpu->getStartingIndex();
				size = stitch_gpu->getSize();
				glDrawArrays(GL_TRIANGLES, slot * 3, size * 3);

				gfx::GLUtils::checkForOpenGLError(
						"[RenderMapInformations::renderTriangleReferencesForPatch]");
				//Add the dependency of this stitch
				dependency.triangle_position_on_gpu = stitch_gpu->getStartingIndex();
				dependency.triangles_version = stitch.triangles_version;
				dependency.geometry = patch->double_stitches[i];
				gpu_tex->ref_tex_dependencies.push_back(dependency);
			}
		}

		patch->double_stitch_mutex.unlock();

		patch->triple_stitch_mutex.lock();

		for(size_t i = 0; i < patch->triple_stitches.size(); i++) {
			TripleStitch &stitch = *patch->triple_stitches[i].get();
			if(stitch.patches[0].lock() != patch) {
				continue;
			}
			shared_ptr<TriangleBufConnector> stitch_gpu = stitch.triangles_gpu.lock();
			if(stitch_gpu == nullptr) {
				cout << "[InformationRenderer::renderTriangleReferencesForPatch]"
				        " This should not happen when this is invoked for a patch"
				        " which is part of an active set" << endl;
				assert(0);
				return;
			}
			slot = stitch_gpu->getStartingIndex();
			size = stitch_gpu->getSize();
			glDrawArrays(GL_TRIANGLES, slot * 3, size * 3);

			gfx::GLUtils::checkForOpenGLError(
					"[RenderMapInformations::renderTriangleReferencesForPatch]");

			//Add the dependency of this stitch
			dependency.triangle_position_on_gpu = stitch_gpu->getStartingIndex();
			dependency.triangles_version = stitch.triangles_version;
			dependency.geometry = patch->triple_stitches[i];
			gpu_tex->ref_tex_dependencies.push_back(dependency);
		}

		patch->triple_stitch_mutex.unlock();

	} else {
		cout << "[InformationRenderer::renderTriangleReferencesForPatch] "
		        "Can't render lookup texture since stitches and neighbouring "
		        "points are not part of active set!" << endl;
		assert(0);
	}

	gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::renderTriangleReferencesForPatch] After rendering");

	glDisable(GL_SCISSOR_TEST);
	 */
}

void InformationRenderer::renderTriangleReferencesAndDepth(
		ActiveSet *active_set,
		GpuStorage* gpu_storage,
		Matrix4f projection,
		Matrix4f pose) {
	gfx::GLUtils::checkForOpenGLError("[RenderMapInformations::renderTriangleReferencesAndDepth] Before rendering");

	triangle_ref_depth_prog_->use();

	//after setting up the uniforms we set up all the possible
	glUniformMatrix4fv(0, 1, false, (GLfloat*) &pose);
	glUniformMatrix4fv(1, 1, false, (GLfloat*) &projection);

	gfx::GLUtils::checkForOpenGLError(
			"[ RenderMapPresentation::render] Right at rendering patches");

	//the vertex buffer
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 
	                 gpu_storage->vertex_buffer->getGlName());

	//bind texture coordinates
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1,
	                 gpu_storage->tex_pos_buffer->getGlName());

	//the triangle buffer
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2,
	                 gpu_storage->triangle_buffer->getGlName());

	//the patch information
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3,
	                 gpu_storage->patch_info_buffer->getGlName());

	assert(0);//here we need to reinsert a ton of logic
	//active_set->drawEverything();
	gfx::GLUtils::checkForOpenGLError(
			"[InformationRenderer::renderTriangleReferencesAndDepth] Right after issuing render commands.");

	glFinish();
}

void InformationRenderer::render(	ActiveSet *active_set,
									GpuStorage* gpu_storage,
									Matrix4f projection,
								 	Matrix4f pose, cv::Mat *depth, cv::Mat *normals,
								 	cv::Mat *color, cv::Mat *labels) {

	PerThread_ pt = per_thread_gl_objects_[this_thread::get_id()];
	//activate VBO
	glBindFramebuffer(GL_FRAMEBUFFER, pt.combined_FBO);

	gfx::GLUtils::checkForOpenGLError(
			"[RenderMapInformations::render] Before rendering");

	unified_info_prog_->use();

	//TODO: bind the framebuffer!

	glClearColor(NAN, NAN, NAN, NAN);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	//set one certain buffer to something.....
	int clear[] = {-1, -1, -1, -1};
	glClearBufferiv(GL_COLOR, 2, clear);
	float clear2[] = {0, 0, 1.0f, 0}; // try it with float debugwise
	glClearBufferfv(GL_COLOR, 3, clear2);

	//TODO: setup all the other stuff!!!

	Matrix4f pose_inv = pose.inverse();

	glUniformMatrix4fv(0, 1, false, (GLfloat*) &pose_inv);
	glUniformMatrix4fv(1, 1, false, (GLfloat*) &projection);
	gfx::GLUtils::checkForOpenGLError(
		"[ RenderMapPresentation::render] Right at rendering patches");

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0,
	                 gpu_storage->vertex_buffer->getGlName());

	//bind texture coordinates
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1,
	                 gpu_storage->tex_pos_buffer->getGlName());

	//the triangle buffer
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2,
	                 gpu_storage->triangle_buffer->getGlName());

	//the patch information
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3,
	                 gpu_storage->patch_info_buffer->getGlName());

	gfx::GLUtils::checkForOpenGLError(
			"[InformationRenderer::render] Right before issuing render commands.");

	assert(0); // redo the draw calls for the new system
	//active_set->drawEverything();

	gfx::GLUtils::checkForOpenGLError("[InformationRenderer::render] Right after issuing render commands.");

	glFinish();
	pt.z_texture->downloadData(depth->data);
	pt.normal_texture->downloadData(normals->data);
	pt.color_texture->downloadData(color->data);
	pt.label_texture->downloadData(labels->data);
	glFinish();
}

Vector4f InformationRenderer::renderAndExtractInfo(
		MeshReconstruction* reconstruction,
		ActiveSet* active_set,
		GpuStorage* gpu_storage,
		Matrix4f view, Matrix4f proj, LowDetailRenderer* low_detail_renderer, bool render_visible_from_cam,
		GLFWwindow *root_context, int width, int height, int x, int y, 
		int *patch_ind, int *triangle_ind) {
	assert(0);//this needs a bigger overhaul
	/*
	gfx::GLUtils::checkForOpenGLError(
		"[InformationRenderer::renderInfo] Before rendering");

	reconstruction->active_set_update_mutex.lock();
	shared_ptr<ActiveSet> active_set1 = reconstruction->active_set_update;
	reconstruction->active_set_update_mutex.unlock();

	reconstruction->active_set_rendering_mutex_.lock();
	shared_ptr<ActiveSet> active_set2 = reconstruction->active_set_rendering_;
	reconstruction->active_set_rendering_mutex_.unlock();

	GLuint screen_FBO;
	glGetIntegerv(GL_FRAMEBUFFER_BINDING, (GLint*) &screen_FBO);
	//create a texture and fbo (+ depth texture) we render into
	GLuint FBO;
	glGenFramebuffers(1, &FBO);
	glBindFramebuffer(GL_FRAMEBUFFER, FBO);

	gfx::GLUtils::checkForOpenGLError(
			"[ScaleableMap::renderInfo] Setting up stuff");

	GLuint ref_tex;
	GLuint geom_tex;
	glGenTextures(1, &ref_tex);
	glBindTexture(GL_TEXTURE_2D, ref_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, 
	             GL_FLOAT, 0);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, ref_tex, 0);
	glGenTextures(1, &geom_tex);
	glBindTexture(GL_TEXTURE_2D, geom_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, 
	             GL_FLOAT, 0);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, geom_tex, 0);

	gfx::GLUtils::checkForOpenGLError(
			"[ScaleableMap::renderInfo] Setting up stuff");

	GLuint depth_buffer_tex;
	glGenTextures(1, &depth_buffer_tex);
	glBindTexture(GL_TEXTURE_2D, depth_buffer_tex);
	glTexImage2D(GL_TEXTURE_2D, 0,GL_DEPTH_COMPONENT32, width, height, 0,
	             GL_DEPTH_COMPONENT, GL_FLOAT, 0);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_buffer_tex, 
	                     0);

	gfx::GLUtils::checkForOpenGLError(
			"[ScaleableMap::renderInfo] Setting up stuff");
	//initialize with NAN

	//http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-14-render-to-texture/
	GLenum draw_buffers[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
	glDrawBuffers(2, draw_buffers); // "1" is the size of DrawBuffers

	gfx::GLUtils::checkForOpenGLError(
			"[ScaleableMap::renderInfo] Setting up stuff");

	glViewport(0, 0, width, height);
	glClearColor(NAN, NAN, NAN, NAN);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	gfx::GLUtils::checkForOpenGLError(
			"[ScaleableMap::renderInfo] Setting up stuff");

	if(render_visible_from_cam) {
		if(active_set1 != nullptr) {
			renderTriangleReferencesAndDepth(
					active_set1.get(), proj, view);
		}
		if(active_set2 != nullptr) {
			renderTriangleReferencesAndDepth(
					active_set2.get(), proj, view);
		}
	}
	//TODO: also render the low detail stuff... at least do it for the
	low_detail_renderer->renderGeometry(proj, view);

	glFinish();

	cv::Mat cpu_tri_ref(height, width, CV_32FC4);
	cv::Mat cpu_geom_ref(height, width, CV_32FC4);

	//download the data.
	glBindTexture(GL_TEXTURE_2D, ref_tex);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, cpu_tri_ref.data);
	glBindTexture(GL_TEXTURE_2D, geom_tex);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, cpu_geom_ref.data);

	//rebind the original screen framebuffer
	glBindFramebuffer(GL_FRAMEBUFFER, screen_FBO);

	//all of this is just a one time thingy:
	glDeleteTextures(1, &ref_tex);
	glDeleteTextures(1, &geom_tex);
	glDeleteFramebuffers(1, &FBO);

	//now read out the required data.... because we need it
	cout << "clicked on pixel" << x << " x " << y << endl;
	cout << "content: " << endl;
	Vector4f point = cpu_geom_ref.at<Vector4f>(y, x);
	cv::Vec4i ref = cpu_tri_ref.at<cv::Vec4i>(y, x);
	cout << "point " << point << endl;
	if(ref[3] != 10.0f) {
		cout << "patch index " <<  *((int*) &ref[0]) << endl;
		cout << "triangle index " <<  *((int*) &ref[1]) << endl;
	} else {
		cout << "not a valid triangle" << endl;
	}

	if(patch_ind != nullptr) {
		*patch_ind = ref[0];
	}
	if(triangle_ind != nullptr) {
		*triangle_ind = ref[1];
	}
	return point;
	 */
}