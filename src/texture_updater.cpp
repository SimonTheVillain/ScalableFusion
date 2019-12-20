#include "texture_updater.h"

#include <cuda/tex_coords.h>
#include <cuda/tex_patch_init.h>
#include <cuda/std_tex_update.h>
#include <gpu/active_set.h>
#include <debug_render.h>
#include <mesh_reconstruction.h>
#include <gpu/camera.h>
#include <gpu/gpu_mesh_structure.h>
#include <scheduler.h>

using namespace std;
using namespace Eigen;

void TextureUpdater::generateGeomTex(MeshReconstruction* reconstruction,
									 GpuStorage* gpu_storage,
									 vector<shared_ptr<Meshlet> > &meshlets,
									 Matrix4f pose, Matrix4f proj,
									 shared_ptr<gfx::GpuTex2D> geom_sensor_data,
									 shared_ptr<ActiveSet> active_set,
									 InformationRenderer* information_renderer) {


	//texture scale!
	float scale = 2;
	//cout << "TextureUpdater::generateGeomTex REIMPLEMENT THIS" << endl;
	//return;
	//assert(0);//TODO: reimplement this functionality
	{
		//MeshReconstruction *mesh = reconstruction;//TODO: get rid of!

		vector<MeshletGPU*> meshlets_gpu(meshlets.size());
		vector<TextureLayerGPU*> texture_layers_gpu(meshlets.size());
		//first we need to get bounds:
		vector<cv::Rect2f> bounds = calcTexBounds(active_set, meshlets, pose, proj);

		//then we generate texture coordinates:
		vector<TexCoordGen::Task> tex_gen_tasks(bounds.size());
		tex_gen_tasks.reserve(active_set->meshlets.size());
		for(size_t i=0; i < meshlets.size(); i++){
			cv::Rect2f &bound = bounds[i];
			TexCoordGen::Task &task = tex_gen_tasks[i];
			shared_ptr<Meshlet> &meshlet = meshlets[i];
			//allocate gpu_storage, generate the tasks and go for it!
			MeshletGPU *meshlet_gpu = active_set->getGpuMeshlet(meshlet);
			meshlets_gpu[i] = meshlet_gpu;
			texture_layers_gpu[i] = &meshlet_gpu->std_tex;



			//setup data containers for standard deviation texture
			meshlet_gpu->std_tex.coords =
					gpu_storage->tex_pos_buffer->getBlock(meshlet_gpu->vertices->getSize());
			meshlet_gpu->std_tex.version = 1; //initial version!
			if(meshlet->geom_tex_patch == nullptr){
				meshlet->geom_tex_patch = make_shared<MeshTexture>(MeshTexture::Type::STANDARD_DEVIATION);
			}
			meshlet_gpu->std_tex.token =
					make_unique<weak_ptr<MeshTexture>>(meshlet->geom_tex_patch);//TODO: this token
			cv::Size2i size(bound.width * scale,bound.height * scale);
			meshlet_gpu->std_tex.tex =
					gpu_storage->tex_atlas_stds_->getTexAtlasPatch(size);
			meshlet_gpu->geom_lookup_tex =
					gpu_storage->tex_atlas_geom_lookup_->getTexAtlasPatch(size);


			//setup the task for texture coordinate generation
			task.coords = meshlet_gpu->std_tex.coords->getStartingPtr();
			task.vertices = meshlet_gpu->vertices->getStartingPtr();
			task.vertex_count = meshlet_gpu->vertices->getSize();

			task.offset_x       = bound.x - 0.5f / float(scale);
			task.offset_y       = bound.y - 0.5f / float(scale);
			task.scale_x        = 1.0f / (bound.width  + 1.0f / float(scale));
			task.scale_y        = 1.0f / (bound.height + 1.0f / float(scale));



		}
		Matrix4f mvp = proj * pose.inverse();
		//TODO:this could actually be put into a future while we are rendering the lookup texture
		TexCoordGen::genTexCoords(tex_gen_tasks, mvp);

		//render the geom lookup texture

		//TODO: remove: it doesn't seem like the headers need to be setup just now!!!
		//active_set->setupHeaders();//the headers are needed for  the tex_coord positions

		genLookupTex(information_renderer,
					 gpu_storage,
					 meshlets_gpu, //for geometry
					 texture_layers_gpu,//the texture layer the lookup is created for
					 true);//dilate

		//std texture
		projToGeomTex(meshlets_gpu,geom_sensor_data,pose,proj);

		//finally update the header on the gpu
		active_set->setupHeaders();
	}

}

void TextureUpdater::projToGeomTex(vector<MeshletGPU*> meshlets,
								   shared_ptr<gfx::GpuTex2D> geom_sensor_data,
								   Matrix4f pose, Matrix4f proj){
	vector<InitDescriptor> commands(meshlets.size());
	for(size_t i=0;i<meshlets.size();i++){
		MeshletGPU* meshlet = meshlets[i];
		InitDescriptor & command = commands[i];
		cv::Rect2i rect = meshlet->std_tex.tex->getRect();

		command.vertices = meshlet->vertices->getStartingPtr();
		command.triangles = meshlet->triangles->getStartingPtr();
		command.out_offset = rect.tl();
		command.output = meshlet->std_tex.tex->getCudaSurfaceObject();
		command.width = rect.width;
		command.height = rect.height;

		rect = meshlet->geom_lookup_tex->getRect();
		command.reference_texture = meshlet->geom_lookup_tex->getCudaSurfaceObject();//SURFACE?
		command.ref_offset = rect.tl();
	}


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
			scale * pose_projected);
}

void TextureUpdater::projToGeomTex(ActiveSet* active_set,
								   vector<shared_ptr<Meshlet>> &new_patches,
								   shared_ptr<gfx::GpuTex2D> geom_sensor_data,
								   Matrix4f pose, Matrix4f proj) {
	assert(0);//TODO: reimplement this functionality
	/*
	//we create a list of commands for the gpu to follow to update the textures.
	vector<InitDescriptor> commands;

	for(size_t i = 0; i < new_patches.size(); i++) {
		Meshlet *patch = new_patches[i].get();
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
	 */
}

shared_ptr<ActiveSet> TextureUpdater::colorTexUpdate(
									GpuStorage* gpu_storage,
									vector<shared_ptr<Meshlet>> requested_meshlets,
									SchedulerBase* scheduler,
									shared_ptr<gfx::GpuTex2D> rgba_tex,
									Vector4f fxycxy, // intrinsics
									Matrix4f color_pose_in){ //extrinsics

	//TODO: this needs to:
	//1) check the most current active set
	//2) create a new one with space for the most current items
	//3) fill the new textures with data
	//4)return with an updated active set!
	vector<shared_ptr<ActiveSet>> active_sets = scheduler->getActiveSets();
	shared_ptr<ActiveSet> new_active_set =
			make_shared<ActiveSet>(gpu_storage,
								   requested_meshlets,
								   scheduler->getActiveSets());
	Matrix4f proj = Camera::genProjMatrix(fxycxy);
	applyColorData2(gpu_storage,requested_meshlets,rgba_tex,color_pose_in,proj,new_active_set);

	return new_active_set;
	/*

	int width = rgba_tex->getWidth();
	int height = rgba_tex->getHeight();

	uint64_t tex_ptr = rgba_tex->getGlHandle();
	rgba_tex->makeResidentInThisThread();
	//2. Step should be the incorporation of new sensor data into the already existing map.
	Matrix4f proj = Camera::genProjMatrix(reconstruction->params.rgb_fxycxy);


	vector<shared_ptr<Meshlet>> visible_shared_patches;
	if(active_set != nullptr) {
		visible_shared_patches = active_set->retained_mesh_patches_cpu;
	}

	//todo: replace the depth pose with the rgb camera pose

	vector<shared_ptr<Meshlet>> fully_loaded_visible_patches;
	for(shared_ptr<Meshlet> patch : visible_shared_patches) {
		if(patch->isPartOfActiveSetWithNeighbours(active_set.get())) {
			fully_loaded_visible_patches.push_back(patch);
		}
	}

	applyColorData(reconstruction,fully_loaded_visible_patches,low_detail_renderer, rgba_tex, color_pose_in, proj,
	               active_set);//the active set all the newly created textures will be attached to

	reconstruction->cleanupGlStoragesThisThread_();
	 */
}
void TextureUpdater::applyColorData2(GpuStorage* gpu_storage,
									vector<shared_ptr<Meshlet>> &meshlets,
									shared_ptr<gfx::GpuTex2D> rgb_in,
									Matrix4f &pose, Matrix4f &proj,
									shared_ptr<ActiveSet> active_set) {

	//TODO: try not to use the reconstruction itself in here!
	if(active_set == nullptr) {
		return;
	}


	Vector4f cam_pos4 = pose * Vector4f(0, 0, 0, 1);
	Vector3f cam_pos(cam_pos4[0], cam_pos4[1], cam_pos4[2]);


	vector<cv::Rect2f> bounds = calcTexBounds(active_set, meshlets, pose, proj);
	vector<TexCoordGen::Task> tex_coord_gen_tasks;
	vector<CopyDescriptor> tex_copy_tasks;


	int width        = rgb_in->getWidth();
	float width_inv  = 1.0f / float(width);
	int height       = rgb_in->getHeight();
	float height_inv = 1.0f / float(height);
	Matrix4f mvp = proj * pose.inverse();;


	for(size_t i=0;i<meshlets.size();i++){
		shared_ptr<Meshlet> meshlet = meshlets[i];
		cv::Rect2i bound = bounds[i];
		MeshletGPU* meshlet_gpu = active_set->getGpuMeshlet(meshlet);

		//so what this is supposed to do is to check if the textures are in bound....
		if(		bound.x < 0 ||  bound.y < 0 ||
		        bound.x + bound.width  > (width - 1) ||
				bound.y + bound.height > (height - 1) ) {
			//if patch does not have valid points something went wrong
			//most likely there are no triangles in said patch....
			continue;
		}

		//and then check if the texture is empty already...
		//and only then create a new texture
		if(meshlet->tex_patches.size() != 0)
			continue;

		//apparently this patch qualifies for
		tex_coord_gen_tasks.emplace_back();
		TexCoordGen::Task &task = tex_coord_gen_tasks.back();

		//create a texture on the cpu:
		meshlet->tex_patches.emplace_back();
		shared_ptr<MeshTexture> tex = meshlet->tex_patches[0];



		//create a texture on the gpu:
		meshlet_gpu->textures.emplace_back();
		shared_ptr<TextureLayerGPU> &tex_gpu = meshlet_gpu->textures[0];
		tex_gpu = make_shared<TextureLayerGPU>();
		tex_gpu->coords = gpu_storage->tex_pos_buffer->getBlock(meshlet_gpu->vertices->getSize());
		cv::Size2i size(bound.width + 0.5f,bound.height + 0.5f);
		tex_gpu->tex = gpu_storage->tex_atlas_rgb_8_bit_->getTexAtlasPatch(size);
		tex_gpu->token = make_unique<weak_ptr<MeshTexture>>(tex);


		task.vertices = meshlet_gpu->vertices->getStartingPtr();
		task.vertex_count = meshlet_gpu->vertices->getSize();
		task.offset_x = bound.x - 0.5f;
		task.offset_y = bound.y - 0.5f;
		task.coords = tex_gpu->coords->getStartingPtr();
		task.scale_x =  1.0f / (bounds[i].width + 1.0f);
		task.scale_y = 1.0f / (bounds[i].height + 1.0f);



		tex_copy_tasks.emplace_back();
		CopyDescriptor &copy_task = tex_copy_tasks.back();
		copy_task.x = bound.x * width_inv;
		copy_task.y = bound.y * height_inv;
		copy_task.width = bound.width * width_inv;
		copy_task.height = bound.height * height_inv;
		cv::Rect2i rect = tex_gpu->tex->getRect();
		copy_task.target_height = rect.height;//REALLY?
		copy_task.target_width = rect.width;//REALLY?
		copy_task.target_x = rect.x;
		copy_task.target_y = rect.y;
		copy_task.target = tex_gpu->tex->getCudaSurfaceObject();


	}


	//generate texture coordinates
	TexCoordGen::genTexCoords(tex_coord_gen_tasks, mvp);

	//copy over the texture patches from the incoming rgb image
	copyToTexPatches(rgb_in->getCudaTextureObject(), tex_copy_tasks); //1.milliseconds for a full image (should be way less if the images are smaller)


	//copy over the textures!

	active_set->setupHeaders();

	return;
	//debugging:
	for(size_t i=0;i<meshlets.size();i++){
		shared_ptr<Meshlet> meshlet = meshlets[i];
		cv::Rect2i bound = bounds[i];
		MeshletGPU* meshlet_gpu = active_set->getGpuMeshlet(meshlet);
		if(meshlet_gpu->textures.size()){
			cv::Rect2i r = meshlet_gpu->textures[0]->tex->getRect();
			cv::Mat data(r.height,r.width,CV_8UC4);
			meshlet_gpu->textures[0]->tex->downloadData(data.data);
			cv::imshow("test",data);
			cv::waitKey();
		}
	}

}
void TextureUpdater::applyColorData(MeshReconstruction* reconstruction,
									vector<shared_ptr<Meshlet>> &visible_patches,
									LowDetailRenderer* low_detail_renderer,
									shared_ptr<gfx::GpuTex2D> rgb_in,
									Matrix4f &pose, Matrix4f &proj,
									shared_ptr<ActiveSet> active_set) {
	cout<< "TextureUpdater::applyColorData reimplement this" << endl;
	return;
	assert(0); //TODO: reimplement this functionality
	/*
	MeshReconstruction *mesh = reconstruction;
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

	vector<shared_ptr<Meshlet>> patches_with_color_updates;

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
		shared_ptr<Meshlet>              patch = visible_patches[i];
		shared_ptr<MeshletGpuHandle> gpu_patch = patch->gpu.lock();
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
	low_detail_renderer->updateColorForPatches(patches_with_color_updates);

	//TODO: download these textures!
	for(GpuCpuTexPair update : updated_textures) {
		update.gpu->gpu_data_changed = true;
	}

	*/
}

void TextureUpdater::genLookupTexGeom(MeshReconstruction* reconstruction,
									  ActiveSet *active_set,
									  vector<shared_ptr<Meshlet>> &patches,
									  InformationRenderer* information_renderer) {
	assert(0);
	vector<shared_ptr<MeshTexture>> textures;
	for(size_t i = 0; i < patches.size(); i++) {
		textures.push_back(patches[i]->geom_tex_patch);
	}
//	genLookupTex(reconstruction,active_set, patches, textures,information_renderer);
}

void TextureUpdater::genLookupTexGeom(	InformationRenderer* information_renderer,
										shared_ptr<ActiveSet> active_set,
										vector<shared_ptr<Meshlet>> &meshlets,
									 	 bool dilate) {
	assert(0);
	vector<shared_ptr<MeshTexture>> textures;
	for(size_t i = 0; i < meshlets.size(); i++) {
		textures.push_back(meshlets[i]->geom_tex_patch);
	}

}


void TextureUpdater::genLookupTex(	InformationRenderer* information_renderer,
									GpuStorage* gpu_storage,
								 	vector<MeshletGPU*> meshlets_gpu,
								  vector<TextureLayerGPU*> textures_gpu,
								  bool dilate) {
	vector<DilationDescriptor> dilations(meshlets_gpu.size());
	information_renderer->bindRenderTriangleReferenceProgram(gpu_storage);

	for(size_t i = 0; i < meshlets_gpu.size();i++) {
		if(meshlets_gpu[i]->geom_lookup_tex == nullptr) {
			cout << "[ScaleableMap::generateLookupTexGeom] "
					"There is no texture on the gpu" << endl;
			continue;
		}

		cv::Rect2i r = meshlets_gpu[i]->geom_lookup_tex->getRect();
		//TODO: the scissor test probably is a little wasteful (a quad would be way better)

		//to solve this we might want to draw a quad
		information_renderer->renderReference(meshlets_gpu[i], textures_gpu[i]->coords,
				meshlets_gpu[i]->geom_lookup_tex);
		//how about imshowing the result

		if(dilate) {
			DilationDescriptor &dilation = dilations[i];
			dilation.target = meshlets_gpu[i]->geom_lookup_tex->getCudaSurfaceObject();
			dilation.width  = r.width;
			dilation.height = r.height;
			dilation.x      = r.x;
			dilation.y      = r.y;
		}
	}

	glFinish();//let the opengl stuff render before we download it.

	//At last we dilate the lookup of
	if(dilate) {
		dilateLookupTextures(dilations);
		cudaDeviceSynchronize();
	}
	for(size_t i = 0; i < meshlets_gpu.size(); i++) {
		//meshlets[i]->geom_tex_patch->ref_tex_filled = true;
	}


}


vector<cv::Rect2f> TextureUpdater::calcTexBounds(	shared_ptr<ActiveSet> active_set,
									 vector<shared_ptr<Meshlet>> &meshlets,
									 Eigen::Matrix4f pose,
									 Eigen::Matrix4f proj){

	Matrix4f mvp = proj * pose.inverse();
	vector<TexCoordGen::BoundTask> bound_tasks(meshlets.size());
	for(int i=0;i<meshlets.size();i++){
		shared_ptr<Meshlet> &meshlet = meshlets[i];
		MeshletGPU* meshlet_gpu = active_set->getGpuMeshlet(meshlet);
		assert(meshlet_gpu);//debug measure

		TexCoordGen::BoundTask &task = bound_tasks[i];
		task.vertices = meshlet_gpu->vertices->getStartingPtr();
		task.vertex_count = meshlet_gpu->vertices->getSize();

	}


	return TexCoordGen::getTexCoordBounds(bound_tasks, mvp);

}
