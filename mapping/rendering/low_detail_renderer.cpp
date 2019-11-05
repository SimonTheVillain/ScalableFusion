#include "low_detail_renderer.h"

#include <iostream>

#include <base/mesh_structure.h>
#include <cuda/gpu_errchk.h>
#include <cuda/coarse_update.h>
#include <gpu/active_set.h>

using namespace std;
using namespace Eigen;

const string low_detail_frag =
#include "shader/low_detail.frag"
;
const string low_detail_vert =
#include "shader/low_detail.vert"
;
const string low_detail_geom =
#include "shader/low_detail.geom"
;
const string low_detail_ref_depth_frag =
#include "shader/low_detail_ref_depth.frag"
;
const string low_detail_ref_depth_vert =
#include "shader/low_detail_ref_depth.vert"
;
const string low_detail_ref_depth_geom =
#include "shader/low_detail_ref_depth.geom"
;

//the purpose of these should be obvious!!!!!
const string debug_frag =
#include "shader/debug.frag"
;
const string debug_vert =
#include "shader/debug.vert"
;

weak_ptr<gfx::GLSLProgram> LowDetailRenderer::s_shader_;
weak_ptr<gfx::GLSLProgram> LowDetailRenderer::s_geometry_shader_;

void LowDetailRenderer::initInGlContext() {
	if(LowDetailRenderer::s_shader_.use_count()) {
		shader_ = LowDetailRenderer::s_shader_.lock();
	} else {
		shader_ = make_shared<gfx::GLSLProgram>();
		shader_->compileShader(low_detail_frag, 
		                       gfx::GLSLShader::GLSLShaderType::FRAGMENT,
		                       "low_detail.frag");
		shader_->compileShader(low_detail_geom, 
		                       gfx::GLSLShader::GLSLShaderType::GEOMETRY,
		                       "low_detail.geom");
		shader_->compileShader(low_detail_vert, 
		                       gfx::GLSLShader::GLSLShaderType::VERTEX,
		                       "low_detail.vert");
		shader_->link();
		s_shader_ = shader_;

		debug_shader_ = make_shared<gfx::GLSLProgram>();
		debug_shader_->compileShader(debug_frag, 
		                             gfx::GLSLShader::GLSLShaderType::FRAGMENT,
		                             "debug.frag");
		debug_shader_->compileShader(debug_vert, 
		                             gfx::GLSLShader::GLSLShaderType::VERTEX,
		                             "debug.vert");
		debug_shader_->link();
	}

	if(LowDetailRenderer::s_geometry_shader_.use_count()) {
		geometry_shader_ = LowDetailRenderer::s_geometry_shader_.lock();
	} else {
		geometry_shader_ = make_shared<gfx::GLSLProgram>();
		geometry_shader_->compileShader(low_detail_ref_depth_frag,
		                                gfx::GLSLShader::GLSLShaderType::FRAGMENT,
		                                "low_detail_ref_depth.frag");
		geometry_shader_->compileShader(low_detail_ref_depth_geom,
		                                gfx::GLSLShader::GLSLShaderType::GEOMETRY,
		                                "low_detail_ref_depth.geom");
		geometry_shader_->compileShader(low_detail_ref_depth_vert,
		                                gfx::GLSLShader::GLSLShaderType::VERTEX,
		                                "low_detail_ref_depth.vert");
		geometry_shader_->link();

		s_geometry_shader_ = geometry_shader_;
	}
}

void LowDetailRenderer::addPatches(vector<shared_ptr<MeshPatch> > &patches_in, 
                                   Vector3f cam_pos) {
	if(patches_in.size() == 0) {
		return;
	}
	//prevent the render thread from swapping the buffers
	new_buffers_ = false;

	vector<weak_ptr<MeshPatch>> new_patches;
	vector<weak_ptr<CoarseTriangle>> new_coarse_triangles;

	for(size_t i = 0; i < patches_in.size(); i++) {
		MeshPatch &patch = *(patches_in[i].get());
		if(patch.vertices.size() == 1) {
			cout << "this definitely should not be, "
			        "patches with single vertices (or zero triangles)" << endl;
		}

		//add this patch to our array
		int index = patches.size() + new_patches.size();
		new_patches.push_back(patches_in[i]);
		patch.index_within_coarse = index;
		//iterate neighbours
		set<shared_ptr<MeshPatch>> neighbours = patch.getNeighbours();

		//lets check triple stitches.... if they are consistent with the remaining data structure.
		patch.triple_stitch_mutex.lock();
		for(size_t j = 0; j < patch.triple_stitches.size(); j++) {
			shared_ptr<TripleStitch> triple_stitch = patch.triple_stitches[j];
			for(size_t k = 0; k < 3; k++) {
				bool found = false;
				for(size_t l = 0; l < triple_stitch->patches[k].lock()->triple_stitches.size(); l++) {
					if(triple_stitch->patches[k].lock()->triple_stitches[l] == triple_stitch) {
						found = true;
					}
				}
				if(found == false) {
					cout << "DEBUG: obviously the structure is fragmented" << endl;
					assert(0);
				}
			}
		}
		patch.triple_stitch_mutex.unlock();

		//This is a dirty workaround for something that should be doable in the next loop
		//to preventing lock at triangle creation we do not lock here
		//(even if we should)
		//so, even tough we do this here, why isn't it working????????
		for(size_t j = 0; j < patch.triple_stitches.size(); j++) {
			shared_ptr<TripleStitch> triple_stitch = patch.triple_stitches[j];
			continue;
			shared_ptr<CoarseTriangle> triangle =
					patch.getCoarseTriangleWith(triple_stitch->patches[0].lock(),
					                            triple_stitch->patches[1].lock(),
					                            triple_stitch->patches[2].lock());
			if(triangle != nullptr) {
				//seems like we already have a triangle connecting these three
				//continue;
			}

			triangle = make_shared<CoarseTriangle>(triple_stitch->patches[0].lock(),
			                                       triple_stitch->patches[1].lock(),
			                                       triple_stitch->patches[2].lock());
			triple_stitch->patches[0].lock()->addCoarseTriangle(triangle);
			triple_stitch->patches[1].lock()->addCoarseTriangle(triangle);
			triple_stitch->patches[2].lock()->addCoarseTriangle(triangle);

			new_coarse_triangles.push_back(triangle);
		}

		for(set<shared_ptr<MeshPatch>>::iterator it1 = neighbours.begin();
		    it1 != neighbours.end(); ++it1) {
			//find all the neighbours of the neighbours
			set<shared_ptr<MeshPatch>> n2eighbours = (*it1)->getNeighbours();
			for(set<shared_ptr<MeshPatch>>::iterator it2 = n2eighbours.begin();
			    it2 != n2eighbours.end(); ++it2) {
				//test if we have a fitting triangle by comparing with all the neighbouring
				//iterate again over the first set of neighbours (O(n^3))
				for(set<shared_ptr<MeshPatch>>::iterator it3 = neighbours.begin();
				    it3 != neighbours.end(); ++it3) {
					if((*it3) == (*it2)) {
						#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
						cout << "found a neighbour" << endl;
						#endif
						//found one triangle neighbourhood
						//the next step is to find one already existing triangle
						shared_ptr<CoarseTriangle> triangle =
								patch.getCoarseTriangleWith(patches_in[i], *it1, *it2);

						if(triangle != nullptr) {
							//seems like we already have a triangle connecting these three
							continue;
						}

						triangle = make_shared<CoarseTriangle>(patches_in[i], *it1, *it2);
						patches_in[i]->addCoarseTriangle(triangle);
						(*it1)->addCoarseTriangle(triangle);
						(*it2)->addCoarseTriangle(triangle);
						//also fill this triangle with everything thats needed

						new_coarse_triangles.push_back(triangle);
					}
				}
			}
		}
	}

	cout << " LowDetailRenderer::addPatches right after creating the triangle list" << endl;

	//now we update the buffer. (create a completely new one which admittedly is wasteful)
	//TODO: create mechanism that doesn't require us to completely update the whole buffer

	vector<GpuCoarseVertex> new_vertices(new_patches.size());
	vector<int> new_visible(new_patches.size());
	for(size_t i = 0; i < new_patches.size(); i++) {
		if(new_patches[i].expired()) {
			new_visible[i] = 0;
			continue;
		}
		shared_ptr<MeshPatch> patch = new_patches[i].lock();
		Vector3f p3 = patch->getPos();
		new_vertices[i].p = Vector4f(p3[0], p3[1], p3[2], 1.0f);
		new_vertices[i].n = Vector4f(NAN, NAN, NAN, NAN);//placeholder
		new_vertices[i].c = Vector4f(1, 0, 0, 1);//red placeholder

		new_visible[i] = 1;
	}

	vector<int> new_indices;
	for(size_t i = 0; i < new_coarse_triangles.size(); i++) {
		if(new_coarse_triangles[i].expired()) {
			continue;
		}
		shared_ptr<CoarseTriangle> triangle = new_coarse_triangles[i].lock();
		triangle->flipToFacePos(cam_pos);
		if(triangle->isValid()) {
			for(size_t j = 0; j < 3; j++) {
				//iterate over points and add all the points to the index buffers.
				new_indices.push_back(triangle->patches[j].lock()->index_within_coarse);
			}
		}
	}
	int old_nr_vertices = patches.size();
	int old_nr_indices = coarse_triangles.size() * 3;

	//append the new triangles and patches to the list
	patches.insert(patches.end(), new_patches.begin(), new_patches.end());
	coarse_triangles.insert(coarse_triangles.end(),
	                        new_coarse_triangles.begin(),
	                        new_coarse_triangles.end());

	modifying_buffers_.lock();

	shared_ptr<GlCudaBuffer<int>>             ind_buf  = index_buffer_;
	shared_ptr<GlCudaBuffer<GpuCoarseVertex>> vert_buf = vertex_buffer_;
	shared_ptr<GlCudaBuffer<int>>             vis_buf  = visibility_buffer_;
	int nr_inds;
	modifying_buffers_.unlock();

	bool new_vertex_buffer = false;
	bool new_index_buffer  = false;
	if(vert_buf == nullptr) {
		new_vertex_buffer = true;
		//create a whole new buffer
		vert_buf = make_shared<GlCudaBuffer<GpuCoarseVertex>>(patches.size() * 2);
		vis_buf = make_shared<GlCudaBuffer<int>>(patches.size() * 2);

	} else {
		if(vert_buf->nr_elements < patches.size()) {
			new_vertex_buffer = true;
			//create a whole new buffer and copy the old stuff in
			shared_ptr<GlCudaBuffer<GpuCoarseVertex>> old_vert_buf = vert_buf;
			vert_buf = make_shared<GlCudaBuffer<GpuCoarseVertex>>(patches.size() * 2);
			cudaMemcpy(vert_buf->cuda_ptr, old_vert_buf->cuda_ptr, 
			           old_vert_buf->nr_elements * sizeof(GpuCoarseVertex),
			           cudaMemcpyDeviceToDevice);

			shared_ptr<GlCudaBuffer<int>> old_vis_buf = vis_buf;
			vis_buf = make_shared<GlCudaBuffer<int>>(patches.size() * 2);
			cudaMemcpy(vis_buf->cuda_ptr, old_vis_buf->cuda_ptr,
			           old_vis_buf->nr_elements * sizeof(int),
			           cudaMemcpyDeviceToDevice);

			cudaDeviceSynchronize();
			gpuErrchk(cudaPeekAtLastError());
		}
	}

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	if(coarse_triangles.empty()) {
		//cout << "DEUBG: i am not sure if this does invalidate the whole structure" << endl;
		//return;
	}
	//upload novel data
	cudaMemcpy(&(vert_buf->cuda_ptr[old_nr_vertices]), &(new_vertices[0]),
	           new_vertices.size() * sizeof(GpuCoarseVertex),
	           cudaMemcpyHostToDevice);

	cudaMemcpy(&(vis_buf->cuda_ptr[old_nr_vertices]), &(new_visible[0]),
	           new_vertices.size() * sizeof(int),
	           cudaMemcpyHostToDevice);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	if(ind_buf == nullptr) {
		new_index_buffer = true;
		ind_buf = make_shared<GlCudaBuffer<int>>(coarse_triangles.size() * 3 * 2);
	} else {
		if(ind_buf->nr_elements < coarse_triangles.size() * 3) {//TODO: delete this *2
			new_index_buffer = true;
			shared_ptr<GlCudaBuffer<int>> old_ind_buf = ind_buf;
			//create a new buffer of bigger size:
			ind_buf = make_shared<GlCudaBuffer<int>>(coarse_triangles.size() * 3 * 2);

			//copy over the old data
			cudaMemcpy(ind_buf->cuda_ptr, old_ind_buf->cuda_ptr,
			           old_ind_buf->nr_elements * sizeof(int),
			           cudaMemcpyDeviceToDevice);
		}
	}
	//upload the new data
	cudaMemcpy(&(ind_buf->cuda_ptr[old_nr_indices]), &(new_indices[0]),
	           new_indices.size() * sizeof(int),
	           cudaMemcpyHostToDevice);
	nr_inds = coarse_triangles.size() * 3;

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	//setup the gpu to calculate the average colors of the vertices
	vector<CalcMeanColorDescriptor> descriptors;
	for(size_t i = 0; i < patches_in.size(); i++) {
		MeshPatch &patch = *(patches_in[i].get());
		if(patch.tex_patches.empty()) {
			//if there is no color for the texture we don't update it.
			continue;
		}
		MeshTexture &tex = *(patch.tex_patches[0].get());

		shared_ptr<MeshTextureGpuHandle> tex_gpu_handle = tex.gpu.lock();
		CalcMeanColorDescriptor desc;

		if(patch.tex_patches.size() == 0) {
			continue;
		}
		if(tex.gpu.lock() == nullptr) {
			continue;
		}
		cv::Rect2f rect = tex_gpu_handle->tex->getRect();
		desc.color      = tex_gpu_handle->tex->getCudaSurfaceObject();
		desc.scale      = 1.0f;
		desc.hdr        = false;
		desc.x          = rect.x;
		desc.y          = rect.y;
		desc.width      = rect.width;
		desc.height     = rect.height;
		desc.vert_ind   = patch.index_within_coarse;
		descriptors.push_back(desc);
	}

	//use the kernel to update the vertices on the gpu
	calcMeanColor(descriptors, vert_buf->cuda_ptr, 0);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	modifying_buffers_.lock();
	nr_indices_ = nr_inds;

	//set the buffer references for rendering
	if(new_vertex_buffer || new_index_buffer) {
		//but only do this if one of the buffers had to be resetted
		index_buffer_      = ind_buf;
		vertex_buffer_     = vert_buf;
		visibility_buffer_ = vis_buf;

		new_buffers_ = true;
	}
	modifying_buffers_.unlock();

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
}

void LowDetailRenderer::downloadCurrentGeometry(
		vector<GpuCoarseVertex> &vertices, vector<int> &indices) {

	modifying_buffers_.lock();

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	indices.resize(coarse_triangles.size() * 3);//indexBuffer->nrElements);

	cudaMemcpy(&(indices[0]),//dst
	           index_buffer_->cuda_ptr,//src
	           coarse_triangles.size() * sizeof(int) * 3,
	           cudaMemcpyDeviceToHost);

	//AAAAAAAAAAAH EVERYTHING HERE IS SUPERAWFUL!!!!!!!!
	int count = coarse_triangles.size();
	int count2 = indices.size();
	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	//TODO: it might be that some of these patches have gottin invalid
	//then we really need to change stuff
	int vertex_count = patches.size();
	vertices.resize(vertex_count);

	cudaMemcpy(&(vertices[0]),//dst
	           vertex_buffer_->cuda_ptr,//src
	           vertex_count * sizeof(GpuCoarseVertex),
	           cudaMemcpyDeviceToHost);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	modifying_buffers_.unlock();
}

void LowDetailRenderer::updateColorForPatches(
		vector<shared_ptr<MeshPatch>> &patches_in) {
	modifying_buffers_.lock();

	shared_ptr<GlCudaBuffer<GpuCoarseVertex>> vert_buf = vertex_buffer_;
	if(vert_buf == nullptr) {
		modifying_buffers_.unlock();
		return;
	}

	//prevent the render thread from swapping buffers
	bool new_buffers_intermediate = new_buffers_;
	new_buffers_ = false;

	//we should somehow prevent the whole thing from creating new buffers.
	//especially deleting

	//setup the gpu to calculate the average colors of the vertices
	vector<CalcMeanColorDescriptor> descriptors;
	for(size_t i = 0; i < patches_in.size(); i++) {
		MeshPatch &patch = *(patches_in[i].get());
		CalcMeanColorDescriptor desc;

		if(patch.tex_patches.size() == 0) {
			continue;
		}
		if(patch.index_within_coarse == -1) {
			continue;//don't do anything if the patch is not part
			//of the coarse representation yet
		}
		MeshTexture &tex = *(patch.tex_patches[0].get());
		shared_ptr<MeshTextureGpuHandle> tex_gpu_handle = tex.gpu.lock();
		if(tex_gpu_handle->tex == nullptr) {
			//continue;//actually this should not happen but in case it does we
			//want to prevent a crash
		}
		cv::Rect2f rect = tex_gpu_handle->tex->getRect();
		desc.color      = tex_gpu_handle->tex->getCudaSurfaceObject();
		desc.scale      = 1.0f;
		desc.hdr        = false;
		desc.x          = rect.x;
		desc.y          = rect.y;
		desc.width      = rect.width;
		desc.height     = rect.height;
		desc.vert_ind   = patch.index_within_coarse;
		descriptors.push_back(desc);
	}

	//use the kernel to update the vertices on the gpu
	calcMeanColor(descriptors, vert_buf->cuda_ptr, 0);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	new_buffers_ = new_buffers_intermediate;
	modifying_buffers_.unlock();
}

void LowDetailRenderer::renderExceptForActiveSets(
		vector<shared_ptr<ActiveSet> > &sets, Matrix4f proj, Matrix4f cam_pose) {
	gfx::GLUtils::checkForOpenGLError(
			"[LowDetailRenderer::renderExceptForActiveSets] At the beginning.");
	shader_->use();

	modifying_buffers_.lock();

	//prevent these buffers from beeing erased
	shared_ptr<GlCudaBuffer<int>>             ind_buf  = index_buffer_;
	shared_ptr<GlCudaBuffer<GpuCoarseVertex>> vert_buf = vertex_buffer_;
	shared_ptr<GlCudaBuffer<int>>             vis_buf  = visibility_buffer_;
	int nr_inds = nr_indices_;

	if(new_buffers_) {
		//copy the new buffers over the old ones
		//and also delete the old ones
		//todo: delete the old buffers

		//adapt (or create) the VAO
		if(VAO_ == 0) {
			glGenVertexArrays(1, &VAO_);
		}
		glBindVertexArray(VAO_);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ind_buf->gl_name);
		glBindBuffer(GL_ARRAY_BUFFER, vert_buf->gl_name);
		//pos
		glVertexAttribPointer(0,                               // attribute
		                      4,                               // size
		                      GL_FLOAT,                        // type
		                      GL_FALSE,                        // normalized?
		                      sizeof(GpuCoarseVertex),         // stride (0 should work as well)
		                      (void*) 0);                      // array buffer offset
		glEnableVertexAttribArray(0);
		//normal
		glVertexAttribPointer(1,                               // attribute
	                        4,                               // size
		                      GL_FLOAT,                        // type
		                      GL_FALSE,                        // normalized?
		                      sizeof(GpuCoarseVertex),         // stride (0 should work as well)
		                      (void*) (sizeof(Vector4f) * 1)); // array buffer offset
		glEnableVertexAttribArray(1);
		//color
		glVertexAttribPointer(2,                               // attribute
		                      4,                               // size
		                      GL_FLOAT,                        // type
		                      GL_FALSE,                        // normalized?
		                      sizeof(GpuCoarseVertex),         // stride (0 should work as well)
		                      (void*) (sizeof(Vector4f) * 2)); // array buffer offset
		glEnableVertexAttribArray(2);

		glBindBuffer(GL_ARRAY_BUFFER, vis_buf->gl_name);
		glVertexAttribPointer(3,                               // attribute
		                      1,                               // size
		                      GL_INT,                          // type
		                      GL_FALSE,                        // normalized?
		                      sizeof(int),                     // stride (0 should work as well)
		                      (void*) 0);                      // array buffer offset
		glEnableVertexAttribArray(3);

		new_buffers_ = false;
	}

	modifying_buffers_.unlock();
	if(VAO_ == 0) {
		return;
	}

	//make the triangles visible/invisible
	//size_t nrDisabledPatches=0;
	vector<int> disable_patches;
	int debug_max = 0;
	for(size_t i = 0; i < sets.size(); i++) {
		shared_ptr<ActiveSet> &aset = sets[i];
		if(aset == nullptr) {
			continue;
		}
		for(size_t j = 0; j < aset->retained_mesh_patches_cpu.size(); j++) {
			shared_ptr<MeshPatch> patch = aset->retained_mesh_patches_cpu[j];
			disable_patches.push_back(patch->index_within_coarse);
			if(debug_max<patch->index_within_coarse) {
				debug_max = patch->index_within_coarse;
			}
		}
	}

	for(size_t i = 0; i < invisible_in_last_frame_.size(); i++) {
		if(invisible_in_last_frame_[i] >= vis_buf->nr_elements) {
			assert(0);
		}
	}
	for(size_t i = 0; i < disable_patches.size(); i++) {
		if(disable_patches[i] >= vis_buf->nr_elements) {
			assert(0);
		}
	}
	coarseUpdateVisibility(invisible_in_last_frame_, disable_patches, 
	                       vis_buf->cuda_ptr);
	invisible_in_last_frame_ = disable_patches;

	//disable vertices included within this active set
	//(do this with a cuda kernel)

	glBindVertexArray(VAO_);

	Matrix4f mvp = proj * cam_pose;
	glUniformMatrix4fv(0, 1, false, (float*) &mvp);

	gfx::GLUtils::checkForOpenGLError(
			"[LowDetailRenderer::renderExceptForActiveSets] "
			"Error at setting up the low detail rendering.");

	//render the low poly version of the map
	glDrawElements(GL_TRIANGLES, nr_inds, GL_UNSIGNED_INT, (void*) 0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);//back from debug
	glFinish();//DEBUG: this should not be necessary.
	//check for opengl errors
	gfx::GLUtils::checkForOpenGLError(
			"[LowDetailRenderer::renderExceptForActiveSets] "
			"Error at rendering coarse reconstruction.");
}

void LowDetailRenderer::updateMaskForActiveSets(
		vector<shared_ptr<ActiveSet>> &sets) {
}

void LowDetailRenderer::renderColor(Matrix4f proj, Matrix4f cam_pose) {

	modifying_buffers_.lock();

	//prevent these buffers from being erased
	shared_ptr<GlCudaBuffer<int>>             ind_buf  = index_buffer_;
	shared_ptr<GlCudaBuffer<GpuCoarseVertex>> vert_buf = vertex_buffer_;
	shared_ptr<GlCudaBuffer<int>>             vis_buf  = visibility_buffer_;
	int nr_inds = nr_indices_;

	modifying_buffers_.unlock();
	//disable vertices included within this active set
	//(do this with a cuda kernel)

	if(VAO_ == 0) {
		return;
	}

	glBindVertexArray(VAO_);
	shader_->use();

	Matrix4f mvp = proj * cam_pose;
	glUniformMatrix4fv(0, 1, false, (float*) &mvp);

	gfx::GLUtils::checkForOpenGLError(
			"[LowDetailRenderer::renderExceptForActiveSets] "
			"Error at setting up the low detail rendering.");

	//render the low poly version of the map
	glDrawElements(GL_TRIANGLES, nr_inds, GL_UNSIGNED_INT, (void*) 0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);//back from debug
	glFinish();//DEBUG: this should not be necessary.
	//check for opengl errors
	gfx::GLUtils::checkForOpenGLError(
			"[LowDetailRenderer::renderColor] "
			"Error at rendering coarse reconstruction.");
}

void LowDetailRenderer::renderGeometry(Matrix4f proj, Matrix4f cam_pose) {
	modifying_buffers_.lock();


	//prevent these buffers from beeing erased
	shared_ptr<GlCudaBuffer<int>>             ind_buf  = index_buffer_;
	shared_ptr<GlCudaBuffer<GpuCoarseVertex>> vert_buf = vertex_buffer_;
	shared_ptr<GlCudaBuffer<int>>             vis_buf  = visibility_buffer_;
	int nr_inds = nr_indices_;

	modifying_buffers_.unlock();

	if(VAO_ == 0) {
		return;
	}
	glBindVertexArray(VAO_);
	geometry_shader_->use();

	Matrix4f mvp = proj * cam_pose;
	glUniformMatrix4fv(0, 1, false, (float*) &mvp);

	//render the low poly version of the map
	glDrawElements(GL_TRIANGLES, nr_inds, GL_UNSIGNED_INT, (void*) 0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);//back from debug

	glFinish();//DEBUG: this should not be necessary.
}

bool CoarseTriangle::isValid() {
	for(size_t i = 0; i < 3; i++) {
		if(patches[i].expired()) {
			return false;
		}
	}
	return true;
}

CoarseTriangle::CoarseTriangle(shared_ptr<MeshPatch> p1, 
                               shared_ptr<MeshPatch> p2, 
                               shared_ptr<MeshPatch> p3) {
	patches[0] = p1;
	patches[1] = p2;
	patches[2] = p3;

	double_stitches[0] = p1->getDoubleStitchTo(p2);
	double_stitches[1] = p1->getDoubleStitchTo(p3);
	double_stitches[2] = p2->getDoubleStitchTo(p3);

	triple_stitches[0] = p1->getTripleStitchTo(p2);
	triple_stitches[1] = p1->getTripleStitchTo(p3);
	triple_stitches[2] = p2->getTripleStitchTo(p3);
}

CoarseTriangle::~CoarseTriangle() {

}

bool CoarseTriangle::isConnectingSame3Patches(shared_ptr<MeshPatch> p1,
                                              shared_ptr<MeshPatch> p2,
                                              shared_ptr<MeshPatch> p3) {
	if(!isValid()) {
		return false;
	}
	int indices[] = {0, 1, 2};
	sort(indices, indices + 3);
	do {
		if(patches[indices[0]].lock() == p1 &&
		   patches[indices[1]].lock() == p2 &&
		   patches[indices[2]].lock() == p3) {
			return true;
		}
	} while(next_permutation(indices, indices + 3));
	return false;
}

bool CoarseTriangle::flipToFacePos(Vector3f pos) {
	Vector3f to_camera =  pos - patches[0].lock()->getPos();

	Vector3f v1 = patches[1].lock()->getPos() - patches[0].lock()->getPos();
	Vector3f v2 = patches[2].lock()->getPos() - patches[0].lock()->getPos();
	if(v1.cross(v2).dot(to_camera) < 0) {
		swap(patches[1], patches[2]);
		swap(double_stitches[1], double_stitches[2]);
		swap(triple_stitches[1], triple_stitches[2]);
	}
}

void LowDetailPoint::addCoarseTriangle(
		shared_ptr<CoarseTriangle> coarse_triangle) {
	coarse_triangle_mutex.lock();
	triangle_within_neighbours.push_back(coarse_triangle);
	coarse_triangle_mutex.unlock();
}

shared_ptr<CoarseTriangle> LowDetailPoint::getCoarseTriangleWith(
		shared_ptr<MeshPatch> p1, 
		shared_ptr<MeshPatch> p2, 
		shared_ptr<MeshPatch> p3) {

	coarse_triangle_mutex.lock();
	for(size_t i = 0; i < triangle_within_neighbours.size(); i++) {
		shared_ptr<CoarseTriangle> triangle = triangle_within_neighbours[i];
		if(triangle->isConnectingSame3Patches(p1, p2, p3)) {
			coarse_triangle_mutex.unlock();
			return triangle;
		}
	}
	coarse_triangle_mutex.unlock();
	return nullptr;
}

//TODO: think about when we want to call this function because at some point it might become necessary
void LowDetailPoint::cleanupCoarseTriangles() {
	coarse_triangle_mutex.lock();
	assert(0); //not implemented yet
	coarse_triangle_mutex.unlock();
}

template<typename T>
GlCudaBuffer<T>::GlCudaBuffer(size_t size) {
	nr_elements = size;
	glGenBuffers(1, &gl_name);
	glBindBuffer(GL_ARRAY_BUFFER, gl_name);
	glBufferData(GL_ARRAY_BUFFER, size * sizeof(T), 0, GL_DYNAMIC_DRAW);

	cudaError_t error_test;
	error_test = cudaGraphicsGLRegisterBuffer(&cuda_resource, gl_name,
	                                          cudaGraphicsMapFlagsNone);//TODO: test these
	gpuErrchk(error_test);
	error_test = cudaGraphicsMapResources(1, &cuda_resource);//stream 0
	gpuErrchk(error_test);
	size_t bytes;
	error_test = cudaGraphicsResourceGetMappedPointer((void**) &cuda_ptr, &bytes, 
	                                                  cuda_resource);
	gpuErrchk(error_test);
}

template<typename T>
GlCudaBuffer<T>::~GlCudaBuffer() {
	cudaGraphicsUnmapResources(1, &cuda_resource);
	cudaGraphicsUnregisterResource(cuda_resource);
	glDeleteBuffers(1, &gl_name);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
}
