#include "Labelling.h"

#include "camera.h"
#include "meshReconstruction.h"
#include "cuda/coalescedMemoryTransfer.h"
#include "cuda/labelling.h"
#include "cuda/coalescedMemoryTransfer.h"
#include "gpu/ActiveSet.h"

using namespace std;
using namespace Eigen;

typedef CoalescedGpuTransfer::SetTaskTemp<GpuTextureInfo> GpuTexInfoTask;
typedef CoalescedGpuTransfer::CpyTaskTemp<GpuTextureInfo> DEBUGTask;

void Labelling::applyLabels(shared_ptr<ActiveSet> active_set, cv::Mat labels,
                            Matrix4f pose) {
	assert(0);
}

void Labelling::projectLabels(shared_ptr<ActiveSet> active_set, cv::Mat &labels,
                              shared_ptr<gfx::GpuTex2D> d_std_tex, 
                              Matrix4f pose) {

	MeshReconstruction *mesh = mesh_reconstruction;
	//first we create a texture: gfx::GpuTex2D
	//internal formatGL_R32I;

	int width  = labels.cols;
	int height = labels.rows;

	// GL_R
	shared_ptr<gfx::GpuTex2D> label_tex = make_shared<gfx::GpuTex2D>(
			mesh->garbageCollector, GL_R32I, GL_RED_INTEGER, GL_INT, width, height,
			true, nullptr, GL_NEAREST);
	label_tex->uploadData(labels.data);

	vector<shared_ptr<MeshTextureGpuHandle>> secured_patches;

	// For proper labelling we need 3 kinds of tasks:
	// Copying over texture coordinates from the geometry texture to be able to
	// create the lookup textures
	vector<CoalescedGpuTransfer::TaskD2D> copy_tasks;

	// Creating lookup textures
	vector<shared_ptr<MeshPatch>> patches_update_lookup;
	vector<shared_ptr<MeshTexture>> textures_update_lookup;

	//and the actual labelling tasks:
	vector<gpu::Labelling::SegProjTask> labelling_tasks;
	vector<gpu::Labelling::InitializeTask> initialization_tasks;
	vector<shared_ptr<MeshTextureGpuHandle>> labelling_textures_debug;

	vector<GpuTexInfoTask> info_update_tasks;
	vector<DEBUGTask> debug_tasks;

	//the first two could actually be shared with the geometry texture.
	//TODO: do this at some point... make this info shareable

	//actually, we need to secure the labels in at least this active set.

	for(shared_ptr<MeshPatch> patch_cpu : active_set->retainedMeshPatchesCpu) {

		patch_cpu->label_tex_patch_mutex.lock();
		shared_ptr<MeshPatchGpuHandle> patch_gpu = patch_cpu->gpu.lock();
		if(patch_gpu == nullptr) {
			continue;
		}

		if(patch_cpu->label_tex_patch == nullptr) {
			//create a new texture, use the same resolution as the geometry texture
			shared_ptr<MeshTextureGpuHandle> geom_tex_gpu =
					patch_cpu->geom_tex_patch->gpu.lock();
			cv::Rect2i roi  = geom_tex_gpu->tex->getRect();
			cv::Size2i size = roi.size();

			size_t tex_coord_count = geom_tex_gpu->coords->getSize();

			cout << "[ScaleableMap::projectLabels] A lot more work needs to be "
			        "done here!" << endl;

			shared_ptr<MeshTexture> mesh_texture =
					mesh->genMeshTexture(MeshTexture::Type::INTEGER_LABELS);

			shared_ptr<MeshTextureGpuHandle> tex_gpu_handle =
					mesh_texture->genGpuResource(tex_coord_count, size);

			mesh_texture->gpu = tex_gpu_handle;//TODO: how to secure the gpuTexPatch?
			patch_cpu->label_tex_patch = mesh_texture;
			patch_gpu->setLabelTex(tex_gpu_handle);

			shared_ptr<MeshPatchGpuHandle> patch_gpu = patch_cpu->gpu.lock();

			if(patch_gpu != nullptr) {
				shared_ptr<MeshTextureGpuHandle> tex_patch_gpu = patch_gpu->label_tex;

				//TODO: set the label to -1
				//also don't create a new texture every time!!!!!
				gpu::Labelling::InitializeTask initialize_task;
				initialize_task.destSurf = tex_patch_gpu->tex->getCudaSurfaceObject();
				initialize_task.destRect = tex_patch_gpu->tex->getRect();
				initialization_tasks.push_back(initialize_task);

				patches_update_lookup.push_back(patch_cpu);
				textures_update_lookup.push_back(mesh_texture);

				CoalescedGpuTransfer::TaskD2D tex_coord_task;
				tex_coord_task.count = tex_coord_count;
				tex_coord_task.source_index = geom_tex_gpu->coords->getStartingIndex();
				tex_coord_task.destination_index = 
						tex_patch_gpu->coords->getStartingIndex();

				copy_tasks.push_back(tex_coord_task);

				GpuTexInfoTask info_task;
				info_task.dst =
						&(patch_gpu->patch_infos->getStartingPtr()->segmentation_texture);

				info_task.value = tex_patch_gpu->genTexInfo();
				GpuTextureInfo info = tex_patch_gpu->genTexInfo();
				info_update_tasks.push_back(info_task);

				DEBUGTask debug_task;

				debug_task.src =
						&(patch_gpu->patch_infos->getStartingPtr()->std_texture);
				debug_task.dst =
						&(patch_gpu->patch_infos->getStartingPtr()->segmentation_texture);
				debug_tasks.push_back(debug_task);
			} else {
				assert(0);//this means we lost the newly created tex patch
			}
		}

		shared_ptr<MeshTexture> tex_patch = patch_cpu->label_tex_patch;
		shared_ptr<MeshTextureGpuHandle> gpu_tex_patch = tex_patch->gpu.lock();
		if(gpu_tex_patch == nullptr) {
			assert(0);
		}
		gpu::Labelling::SegProjTask task;
		//fill up task
		task.subchannel = static_cast<int>(0);
		task.destSurf = gpu_tex_patch->tex->getCudaSurfaceObject();
		//wow! this is too long
		task.destination = gpu_tex_patch->tex->getRect();
		//position (top left)
		task.lookup = gpu_tex_patch->ref_tex->getRect().tl();
		task.lookupSurf = 
				gpu_tex_patch->ref_tex->getAtlasTex()->getTex2D()->getCudaSurfaceObject();

		task.vertexDestStartInd = patch_gpu->vertices_source->getStartingIndex();
		labelling_tasks.push_back(task);

		labelling_textures_debug.push_back(gpu_tex_patch);
		patch_cpu->label_tex_patch_mutex.unlock();
	}

	//first we need to copy over the texture coordinates from geometry to the
	//label patch
	CoalescedGpuTransfer::device2DeviceSameBuf(
			mesh->m_gpuGeomStorage.texPosBuffer->getCudaPtr(), copy_tasks);
	//now we do the lookup textures
	cout << "DEBUG: some of these textures are missing?, but why?" << endl;

	mesh->texturing.GenLookupTex(active_set.get(), patches_update_lookup,
	                             textures_update_lookup, false); // no dilation of lookup

	int minuseins = -1; // TODO: wth?
	Vector4f initial_value(*((float*) &minuseins), 0, 0, 0);
	gpu::Labelling::initializeSurfaces<Vector4f>(initialization_tasks, 
	                                             initial_value);
	cudaDeviceSynchronize();

	Matrix4f proj = Camera::genProjMatrix(mesh->params.depthfxycxy);

	Matrix4f pose_tmp = pose.inverse();
	Matrix4f proj_pose =  proj * pose_tmp;
	gpu::Labelling::labelSurfaces(
			labelling_tasks, label_tex->getCudaSurfaceObject(), 
			d_std_tex->getCudaSurfaceObject(), cv::Size2i(width, height), pose_tmp, 
			proj_pose, mesh->m_gpuGeomStorage.vertexBuffer->getCudaPtr(),
			mesh->m_gpuGeomStorage.texPosBuffer->getCudaPtr(),
			mesh->m_gpuGeomStorage.triangleBuffer->getCudaPtr(),
			mesh->m_gpuGeomStorage.patchInfoBuffer->getCudaPtr());

	//TODO: update descriptor on gpu
	CoalescedGpuTransfer::upload(info_update_tasks);

	return;
}
