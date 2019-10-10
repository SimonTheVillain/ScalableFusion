#include "texturing.h"

#include <cuda/tex_coords.h>
#include <cuda/tex_patch_init.h>
#include <cuda/std_tex_update.h>
#include <gpu/active_set.h>
#include <debug_render.h>
#include <mesh_reconstruction.h>
#include <gfx/camera.h>

using namespace std;
using namespace Eigen;

void Texturing::generateGeomTex(vector<shared_ptr<MeshPatch> > &new_patches,
                                Matrix4f pose, Matrix4f proj,
                                shared_ptr<gfx::GpuTex2D> geom_sensor_data,
                                shared_ptr<ActiveSet> active_set) {

	MeshReconstruction *mesh = mesh_reconstruction;
	//TODO: even though commented out this still holds true
	/*
	cout << "[ScaleableMap::generateGeomTexForNovelPatches] "
			"i fear tex coordinates are still missing for vertices "
			"without triangles! This is going to be a problem with the "
			"texture update" << endl;
	*/
	float scale = 2;

	Matrix4f pose_inv = pose.inverse();
	Matrix4f mvp = proj * pose_inv;

	//all of these patches have to be valid...
	vector<shared_ptr<MeshPatch>> valid_mesh_patches = new_patches;

	for(shared_ptr<MeshPatch> patch : new_patches) {
		for(shared_ptr<DoubleStitch> stitch : patch->double_stitches) {
			if(stitch->patches[1].lock()->gpu.lock() == nullptr) {
				assert(0);
			}
			if(stitch->patches[0].lock() != patch) {
				continue;
			}
			if(!stitch->isPartOfActiveSet(active_set.get())) {
				stitch->isPartOfActiveSet(active_set.get());
				assert(0);
			}
		}
		for(shared_ptr<TripleStitch> stitch : patch->triple_stitches) {
			for(int i = 1; i < 3; i++) {
				if(stitch->patches[i].lock()->gpu.lock() == nullptr) {
					assert(0);
				}
			}
			if(stitch->patches[0].lock() != patch) {
				continue;
			}
			if(!stitch->isPartOfActiveSet(active_set.get())) {
				stitch->isPartOfActiveSet(active_set.get());
				assert(0);
				// One explanation could be that we just do not care for loading the neighbouring patch
				// into memory even though we should.
			}
		}
	}
	vector<cv::Rect2f> bounds = mesh->genBoundsFromPatches(new_patches, pose,
	                                                       proj, active_set);

	for(size_t i = 0; i < bounds.size(); i++) {
		cv::Rect2f bound = bounds[i];
		if(max(bound.width, bound.height) > 1024) {
			cout << "maybe download everything related to these bounds. "
			        "we need to find out what is going on here" << endl;
			shared_ptr<MeshPatchGpuHandle> gpu = new_patches[i]->gpu.lock();
			int count = gpu->vertices_source->getSize();
			GpuVertex vertices[count];
			gpu->vertices_source->download(vertices);

			for(size_t j = 0; j < count; j++) {
				cout << vertices[j].p << endl;
			}

			cv::namedWindow("test test");
			that_one_debug_rendering_thingy->vertex_buffer = 
					mesh->gpu_geom_storage_.vertex_buffer->getGlName();
			that_one_debug_rendering_thingy->info_buffer = 
					mesh->gpu_geom_storage_.patch_info_buffer->getGlName();
			that_one_debug_rendering_thingy->triangle_buffer = 
					mesh->gpu_geom_storage_.triangle_buffer->getGlName();
			that_one_debug_rendering_thingy->tex_pos_buffer = 
					mesh->gpu_geom_storage_.tex_pos_buffer->getGlName();
			that_one_debug_rendering_thingy->addPatch(new_patches[i].get(), 1, 0, 0);

			shared_ptr<MeshPatch> debug_patch = new_patches[i];
			for(int i = 0; i < debug_patch->double_stitches.size(); i++) {
				if(debug_patch->double_stitches[i]->patches[0].lock() != debug_patch) {
					continue;
				}
				if(debug_patch->double_stitches[i]->patches[1].lock()->gpu.lock() == nullptr) {
					assert(0);
				}
				that_one_debug_rendering_thingy->addPatch(
						debug_patch->double_stitches[i]->patches[1].lock().get(), 0, 0, 1);
			}

			//render the neighbouring patches.
			while(true) {
				// After setting up the debug rendering for this, we wait so the user can take a look at it
				//cv::waitKey();
			}
			assert(0);
		}
	}

	if(bounds.size() != valid_mesh_patches.size()) {
		assert(0);
	}

	vector<TexCoordGen::Task> tex_gen_tasks;
	tex_gen_tasks.reserve(valid_mesh_patches.size());

	vector<shared_ptr<MeshPatch>> mesh_patches_with_novel_tex;
	mesh_patches_with_novel_tex.reserve(valid_mesh_patches.size());

	for(size_t i = 0; i < valid_mesh_patches.size(); i++) {
		shared_ptr<MeshPatch>              patch = valid_mesh_patches[i];
		shared_ptr<MeshPatchGpuHandle> gpu_patch = patch->gpu.lock();

		shared_ptr<MeshTextureGpuHandle> gpu_texture = gpu_patch->geom_tex;
		if(gpu_texture == nullptr) {
			//create the gpu resources if they are not existant
			int nr_coords = patch->geom_tex_patch->tex_coords.size();
			if(bounds[i].width * scale < 0) {
				//why are the bounds at this one negative?
				continue;//do not continue with creating a new texture here!
				assert(0);//the bounds failed
			}
			//this is where we get the size
			gpu_texture = make_shared<MeshTextureGpuHandle>(
					mesh->gpu_geom_storage_.tex_pos_buffer, nr_coords,
					mesh->tex_atlas_geom_lookup_.get(), mesh->tex_atlas_stds_.get(),
					int(bounds[i].width * scale), int(bounds[i].height * scale));
			gpu_patch->geom_tex = gpu_texture;
			patch->geom_tex_patch->gpu = gpu_texture;

			//mark the texture as the most current source for data
			//in case the container gets deleted.
			//patch->geomTexPatch->texCoordsGpu = gpuTexture->coords;
			patch->geom_tex_patch->gpu = gpu_texture;
			gpu_texture->gpu_data_changed = true;
		} else {
			assert(0);//debug: i don't think the gpu texture should be existing
			//already since this is part of the creation process of novel
			//patches
		}

		//allocate textures and coordinates if necessary.
		//+ create new tasks
		TexCoordGen::Task task;
		task.offset_x       = bounds[i].x - 0.5f / float(scale);
		task.offset_y       = bounds[i].y - 0.5f / float(scale);
		task.scale_x        = 1.0f / (bounds[i].width  + 1.0f / float(scale));
		task.scale_y        = 1.0f / (bounds[i].height + 1.0f / float(scale));
		task.coords         = gpu_texture->coords->getStartingPtr();
		task.triangle_count = gpu_patch->triangles->getSize();
		task.triangles      = gpu_patch->triangles->getStartingPtr();
		tex_gen_tasks.push_back(task);

		//now do all the stitches and so on:

		for(shared_ptr<DoubleStitch> stitch : patch->double_stitches) {
			if(stitch->patches[0].lock() != patch) {
				continue;
			}
			if(!stitch->isPartOfActiveSet(active_set.get())) {
				//TODO: we really need to figure out why this is happening
				//the stitch must have both neighbours being part of this
				stitch->isPartOfActiveSet(active_set.get());

				#ifndef IGNORE_SERIOUS_BUG_5
				assert(0);//This actually should not happen
				#endif // IGNORE_SERIOUS_BUG_5
				continue;
			}
			shared_ptr<TriangleBufConnector> gpu_stitch = stitch->triangles_gpu.lock();
			if(gpu_stitch == nullptr) {
				assert(0);
				continue;
			}

			task.triangle_count = gpu_stitch->getSize();
			task.triangles      = gpu_stitch->getStartingPtr();
			tex_gen_tasks.push_back(task);
		}

		//also triple stitches
		for(shared_ptr<TripleStitch> stitch : patch->triple_stitches) {
			if(stitch->patches[0].lock() != patch) {
				continue;
			}
			if(!stitch->isPartOfActiveSet(active_set.get())) {
				//TODO: we really need to figure out why this is happening
				assert(0); // again, i don't think this should happen
				continue;
			}
			shared_ptr<TriangleBufConnector> gpu_stitch = stitch->triangles_gpu.lock();
			if(gpu_stitch == nullptr) {
				assert(0);
				continue;
			}

			task.triangle_count = gpu_stitch->getSize();
			task.triangles      = gpu_stitch->getStartingPtr();
			tex_gen_tasks.push_back(task);
		}
	}

	//execute the tasks on the gpu:
	TexCoordGen::genTexCoords(
			tex_gen_tasks, mvp, 
			mesh->gpu_geom_storage_.patch_info_buffer->getCudaPtr(),
			mesh->gpu_geom_storage_.vertex_buffer->getCudaPtr());

	//we need headers pointing at the vertices
	active_set->reuploadHeaders();

	//this seems to have worked perfectly
	genLookupTexGeom(active_set.get(), valid_mesh_patches);

	projToGeomTex(active_set.get(), valid_mesh_patches, geom_sensor_data, pose, 
	              proj);

	//since the texture has data that needs to only exists on the gpu,
	//we setup the texture to be downloaded as soon
	for(shared_ptr<MeshPatch> patch : valid_mesh_patches) {
		shared_ptr<MeshTextureGpuHandle> tex = patch->gpu.lock()->geom_tex;
		if(tex == nullptr) {
			assert(0);
		}
		tex->gpu_data_changed = true;
		patch->geom_tex_patch->debug_is_uninitialized = false;
	}
	//TODO: now initialize that stuff

	//TODO: this actually should be capable of creating the ref textures
	//as well as the initial content of the textures.
	//pls!!!! do this here

	//and after doing this we can update the patch header

	active_set->reuploadHeaders();
}


void Texturing::projToGeomTex(ActiveSet* active_set, 
                              vector<shared_ptr<MeshPatch>> &new_patches,
                              shared_ptr<gfx::GpuTex2D> geom_sensor_data,
                              Matrix4f pose, Matrix4f proj) {

	//we create a list of commands for the gpu to follow to update the textures.
	vector<InitDescriptor> commands;

	for(size_t i = 0; i < new_patches.size(); i++) {
		MeshPatch *patch = new_patches[i].get();
		InitDescriptor command;
		shared_ptr<MeshTextureGpuHandle> geom_tex_gpu_handle = 
				patch->geom_tex_patch->gpu.lock();

		cv::Rect2i rect = geom_tex_gpu_handle->tex->getRect(); //This is different from get rect
		command.out_offset = cv::Point2i(rect.x, rect.y);
		rect = geom_tex_gpu_handle->ref_tex->getRect();
		command.ref_offset = cv::Point2i(rect.x, rect.y);
		command.width  = rect.width;
		command.height = rect.height;

		command.output = geom_tex_gpu_handle->tex->getCudaTextureObject();
		command.reference_texture = 
				geom_tex_gpu_handle->ref_tex->getCudaTextureObject();
		commands.push_back(command);
	}

	//run the kernel
	Matrix4f pose_projected = proj * pose.inverse();
	//TODO: get rid of this scaling?
	Matrix4f scale;
	float w = geom_sensor_data->getWidth();
	float h = geom_sensor_data->getHeight();
	scale << 1.0f / w,        0,    0,    0,
	                0, 1.0f / h,    0,    0,
	                0,        0, 1.0f,    0,
	                0,        0,    0, 1.0f;

	stdTexInit(
			geom_sensor_data->getCudaTextureObject(), commands, 
			scale * pose_projected,
			(GpuVertex*)    active_set->gpu_geom_storage->vertex_buffer->getCudaPtr(),
			(Vector2f*)     active_set->gpu_geom_storage->tex_pos_buffer->getCudaPtr(),
			(GpuTriangle*)  active_set->gpu_geom_storage->triangle_buffer->getCudaPtr(),
			(GpuPatchInfo*) active_set->gpu_geom_storage->patch_info_buffer->getCudaPtr());

	//TODO: this is not fully filling the textures. Get to the root of this issue

	return;
}

void Texturing::colorTexUpdate(shared_ptr<gfx::GpuTex2D> rgba_tex, 
                               Matrix4f color_pose_in, 
                               shared_ptr<ActiveSet> &active_set) {
	int width = rgba_tex->getWidth();
	int height = rgba_tex->getHeight();

	uint64_t tex_ptr = rgba_tex->getGlHandle();
	rgba_tex->makeResidentInThisThread();
	//2. Step should be the incorporation of new sensor data into the already existing map.
	Matrix4f proj = Camera::genProjMatrix(mesh_reconstruction->params.rgb_fxycxy);

	/**
	 * TODO:
	 * .) iterate over all visible patches
	 * .) and over the color textures attached to these patches
	 * .) check if the texture we are using is a better fit than the one already in place
	 * .) replace or add the texture.
	 *   *) calculate new tex coordinates
	 *   *) normalize them and create the cutouts from the texture
	 *
	 */

	vector<shared_ptr<MeshPatch>> visible_shared_patches;
	if(active_set != nullptr) {
		visible_shared_patches = active_set->retained_mesh_patches_cpu;
	}

	//todo: replace the depth pose with the rgb camera pose

	vector<shared_ptr<MeshPatch>> fully_loaded_visible_patches;
	for(shared_ptr<MeshPatch> patch : visible_shared_patches) {
		if(patch->isPartOfActiveSetWithNeighbours(active_set.get())) {
			fully_loaded_visible_patches.push_back(patch);
		}
	}

	applyColorData(fully_loaded_visible_patches, rgba_tex, color_pose_in, proj,
	               active_set);//the active set all the newly created textures will be attached to

	mesh_reconstruction->cleanupGlStoragesThisThread_();
}

void Texturing::applyColorData(vector<shared_ptr<MeshPatch>> &visible_patches,
                               shared_ptr<gfx::GpuTex2D> rgb_in,
                               Matrix4f &pose, Matrix4f &proj,
                               shared_ptr<ActiveSet> active_set) {

	MeshReconstruction *mesh = mesh_reconstruction;
	if(active_set == nullptr) {
		return;
	}

	vector<cv::Rect2f> bounds = mesh->genBoundsFromPatches(visible_patches, pose, 
	                                                       proj, active_set);

	Vector4f cam_pos4 = pose * Vector4f(0, 0, 0, 1);
	Vector3f cam_pos(cam_pos4[0], cam_pos4[1], cam_pos4[2]);

	int width        = rgb_in->getWidth();
	float width_inv  = 1.0f / float(width);
	int height       = rgb_in->getHeight();
	float height_inv = 1.0f / float(height);
	Matrix4f mvp = proj * pose.inverse();;

	//this retains the textures that are being replaced so nothing flickers
	//because textures are released before new ones are created.
	vector<shared_ptr<MeshTextureGpuHandle>> mesh_textures_being_replaced;

	vector<TexCoordGen::Task> tex_gen_tasks;

	vector<CopyDescriptor> copies;

	vector<shared_ptr<MeshPatch>> patches_with_color_updates;

	struct GpuCpuTexPair {
		shared_ptr<MeshTexture> cpu;
		shared_ptr<MeshTextureGpuHandle> gpu;
	};
	vector<GpuCpuTexPair> updated_textures;
	//TODO: use this for enabling RGB texture download

	for(size_t i = 0; i < visible_patches.size(); i++) {
		cv::Rect2f bound = bounds[i];
		if(bound.x < 0 || 
		   bound.y < 0 || 
		   bound.x + bound.width  > (width - 1) || 
		   bound.y + bound.height > (height - 1)) {
			//if patch does not have valid points something went wrong
			//most likely there are no triangles in said patch....
			continue;
		}
		shared_ptr<MeshPatch>              patch = visible_patches[i];
		shared_ptr<MeshPatchGpuHandle> gpu_patch = patch->gpu.lock();
		vector<shared_ptr<MeshTexture>> tex_patches_to_delete;
		patch->tex_patches_mutex.lock();

		//i don't trust the patch position
		float dist = (patch->getPos() - cam_pos).norm();

		bool create_this_texture = false;
		if(patch->tex_patches.size() != 0) {
			//vector with textures that need to be removed
			for(size_t j = 0; j < patch->tex_patches.size(); j++) {
				//iterate over all texture patches to see which of them need to be removed
				Vector4f cam_pos_at_capture4 = 
						patch->tex_patches[j]->cam_pose.inverse() * Vector4f(0, 0, 0, 1);
				Vector3f cam_pos_at_capture = 
						cam_pos_at_capture4.block<3, 1>(0, 0);

				float dist_at_capture = (patch->getPos() - cam_pos_at_capture).norm();
				if(dist_at_capture * mesh->params.max_depth_factor_thresh_for_tex_adding > 
				   dist) {
					//now the camera is so close that the new texture is of
					//way higher quality. It is time to remove the old texture
					create_this_texture = true;
					tex_patches_to_delete.push_back(patch->tex_patches[j]);
				}
			}

			//the first implementation only allows one texPatch for each patch.
			if(!create_this_texture) {
				//cout << "we do not add a new texture since there already is one" << endl;
				patch->tex_patches_mutex.unlock();
				continue;
			}
		} else {
			create_this_texture = true;
		}
		patch->tex_patches_mutex.unlock();

		if(create_this_texture) {

			int res_x = bounds[i].width + 1;
			int res_y = bounds[i].height + 1;

			shared_ptr<MeshTexture> mesh_tex =
					mesh->genMeshTexture(MeshTexture::Type::COLOR8);

			//set the pose at which this texture patch got captured
			mesh_tex->cam_pose = pose;
			//thats a good amount of
			int nr_coords = patch->gpu.lock()->geom_tex->coords->getSize();

			shared_ptr<MeshTextureGpuHandle> mesh_tex_gpu =
					mesh_tex->genGpuResource(nr_coords, cv::Size2i(res_x, res_y));

			//create a task for new texture coordinates
			TexCoordGen::Task task;
			task.offset_x       = bounds[i].x - 0.5f;
			task.offset_y       = bounds[i].y - 0.5f;
			task.scale_x        = 1.0f / (bounds[i].width + 1.0f);
			task.scale_y        = 1.0f / (bounds[i].height + 1.0f);
			task.coords         = mesh_tex_gpu->coords->getStartingPtr();
			task.triangle_count = gpu_patch->triangles->getSize();
			task.triangles      = gpu_patch->triangles->getStartingPtr();
			tex_gen_tasks.push_back(task);

			//oh and also do this for all the double stitches
			for(shared_ptr<DoubleStitch> stitch : patch->double_stitches) {
				if(stitch->patches[0].lock() != patch) {
					continue;
				}
				if(!stitch->isPartOfActiveSet(active_set.get())) {
					stitch->isPartOfActiveSet(active_set.get());
					assert(0);//This actually should not happen
					continue;
				}
				shared_ptr<TriangleBufConnector> gpu_stitch = 
						stitch->triangles_gpu.lock();
				if(gpu_stitch == nullptr) {
					assert(0);
					continue;
				}
				task.triangle_count = gpu_stitch->getSize();
				task.triangles      = gpu_stitch->getStartingPtr();
				tex_gen_tasks.push_back(task);
			}

			//also triple stitches
			for(shared_ptr<TripleStitch> stitch : patch->triple_stitches) {
				if(stitch->patches[0].lock() != patch) {
					continue;
				}
				if(!stitch->isPartOfActiveSet(active_set.get())) {
					assert(0); // again, i don't think this should happen
					continue;
				}
				shared_ptr<TriangleBufConnector> gpu_stitch = 
						stitch->triangles_gpu.lock();
				if(gpu_stitch == nullptr) {
					assert(0);
					continue;
				}
				task.triangle_count = gpu_stitch->getSize();
				task.triangles = gpu_stitch->getStartingPtr();
				tex_gen_tasks.push_back(task);
			}

			//and also for copying the texture where it belongs to:
			CopyDescriptor copy;
			copy.x              = bounds[i].x * width_inv;
			copy.y              = bounds[i].y * height_inv;
			copy.width          = bounds[i].width * width_inv;
			copy.height         = bounds[i].height * height_inv;
			copy.target_width   = res_x;
			copy.target_height  = res_y;
			cv::Rect2i out_rect = mesh_tex_gpu->tex->getRect();
			copy.target_x       = out_rect.x;
			copy.target_y       = out_rect.y;
			copy.output         = mesh_tex_gpu->tex->getCudaSurfaceObject();

			copies.push_back(copy);
			patches_with_color_updates.push_back(patch);

			//TODO: here!
			GpuCpuTexPair update;
			update.gpu = mesh_tex_gpu;
			update.cpu = mesh_tex;
			updated_textures.push_back(update);

			//TODO: this has to change when using multiple layers
			//issues!
			mesh_textures_being_replaced.push_back(gpu_patch->texs[0]);
			gpu_patch->texs[0] = mesh_tex_gpu;
			patch->removeTexPatches(tex_patches_to_delete);
			patch->addTexPatch(mesh_tex);
			mesh_tex->gpu = mesh_tex_gpu;
		}
	}

	TexCoordGen::genTexCoords(
			tex_gen_tasks, mvp, 
			mesh->gpu_geom_storage_.patch_info_buffer->getCudaPtr(),
			mesh->gpu_geom_storage_.vertex_buffer->getCudaPtr());

	copyToTinyPatches(rgb_in->getCudaTextureObject(), copies); //1.milliseconds for a full image (should be way less if the images are smaller)

	//updating the low detail map probably also requires a new version of the
	//header TODO: update headers on the fly and just set the headers to the new version in one (all blocking) kernel call.
	active_set->reuploadHeaders();

	//also update the low detail map:
	mesh->low_detail_renderer.updateColorForPatches(patches_with_color_updates);

	//TODO: download these textures!
	for(GpuCpuTexPair update : updated_textures) {
		update.gpu->gpu_data_changed = true;
	}

	return;
}

void Texturing::genLookupTexGeom(ActiveSet *active_set,
                                 vector<shared_ptr<MeshPatch>> &patches) {
	vector<shared_ptr<MeshTexture>> textures;
	for(size_t i = 0; i < patches.size(); i++) {
		textures.push_back(patches[i]->geom_tex_patch);
	}
	genLookupTex(active_set, patches, textures);
}

//TODO: maybe relocate this function into another class?
//also maybe directly operate on the patch
void Texturing::genLookupTex(ActiveSet *active_set,
                             vector<shared_ptr<MeshPatch>> &patches,
                             vector<shared_ptr<MeshTexture>> &textures,
                             bool dilate) {
	vector<DilationDescriptor> dilations;
	dilations.reserve(patches.size());
	mesh_reconstruction->information_renderer.bindRenderTriangleReferenceProgram();

	for(size_t i = 0; i < patches.size();i++) {
		shared_ptr<MeshPatch>              patch = patches[i];
		shared_ptr<MeshTexture>          texture = textures[i];
		shared_ptr<MeshTextureGpuHandle> gpu_tex = texture->gpu.lock();
		if(gpu_tex == nullptr) {
			cout << "[ScaleableMap::generateLookupTexGeom] "
			        "There is no texture on the gpu" << endl;
			continue;
		}

		cv::Rect2i r = texture->getLookupRect();
		//TODO: the scissor test probably is a little wasteful (a quad would be way better)

		//to solve this we might want to draw a quad
		mesh_reconstruction->information_renderer.renderTriangleReferencesForPatch(
				active_set, patches[i], texture);
		//how about imshowing the result

		if(dilate) {
			DilationDescriptor dilation;
			dilation.target = gpu_tex->ref_tex->getCudaSurfaceObject();
			dilation.width  = r.width;
			dilation.height = r.height;
			dilation.x      = r.x;
			dilation.y      = r.y;
			dilations.push_back(dilation);
		}
	}

	glFinish();//let the opengl stuff render before we download it.

	//At last we dilate the lookup of
	if(dilate) {
		dilateLookupTextures(dilations);
		cudaDeviceSynchronize();
	}
	for(size_t i = 0; i < patches.size(); i++) {
		patches[i]->geom_tex_patch->ref_tex_filled = true;
	}
}
