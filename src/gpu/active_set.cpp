#include <gpu/active_set.h>

#include <tuple>
#include <unordered_map>

#include <Eigen/Core>

#include <gpu/gpu_storage.h>
#include <base/mesh_structure.h>
#include <cuda/gpu_errchk.h>
#include <cuda/float16_utils.h>
#include <mesh_reconstruction.h>

using namespace std;
using namespace Eigen;


//TODO: find a way to allocate textures and texture pointers in this thing:
//Meshlet textures could have a flag "do_not_upload" and a new version number to indicate a
//update that requires a newly allocated texture
ActiveSet::ActiveSet(GpuStorage *storage,
					 vector<shared_ptr<Meshlet>> meshlets_requested,
					 vector<shared_ptr<ActiveSet>> active_sets,
					 vector<bool> allocate_new_verts){
	vector<shared_ptr<Meshlet>> reupload;

	//the key is the index
	unordered_map<int,shared_ptr<Meshlet>> map_requested;
	for(const shared_ptr<Meshlet> &meshlet : meshlets_requested)
		map_requested[meshlet->id] = meshlet;

	//patches_set.insert(patches.begin(),patches.end());

	//key is index
	//unordered_map<int,MeshletGPU> most_current_meshlets;

	//unfortunately this does not hold the most current texture
	//unordered_map<int,shared_ptr<TextureLayerGPU>> most_current_textures;


	for(size_t i=0;i<meshlets_requested.size();i++){
		shared_ptr<Meshlet> meshlet = meshlets_requested[i];
		bool reallocate=false;
		if(!allocate_new_verts.empty()){
			reallocate = allocate_new_verts[i];
		}
		patch_inds[meshlet->id] = meshlets.size();
		meshlets.emplace_back();
		MeshletGPU &most_current = meshlets.back();
		most_current.id = meshlet->id;

		//check if the data already exists in one of the active sets
		for(auto active_set : active_sets){
			if(active_set == nullptr)
				continue;
			if(active_set->patch_inds.count(meshlet->id) == 0)
				continue;
			int id = meshlet->id;
			int ind = active_set->patch_inds[id];

			MeshletGPU &candidate = active_set->meshlets[ind];

			if(candidate.triangle_version == meshlet->triangles_version){
				//store the most current versions of texture + vertices
				most_current.triangle_version = candidate.triangle_version;
				most_current.triangles = candidate.triangles;
				if(candidate.vertex_version > most_current.vertex_version){
					most_current.vertex_version = candidate.vertex_version;
					most_current.vertices = candidate.vertices;

					// token taken from most current vertex
					most_current.vertex_token = move(candidate.vertex_token);

				}

				//TODO: geometry texture!!!



				//TODO: all the other textures
				for(size_t k=0;k<candidate.textures.size();k++){
					auto &candid_texture = candidate.textures[k];
					if(most_current.textures.size()-1 >= i){
						//when there is no old texture in this patch we just copy over
						most_current.textures[i] = candidate.textures[i];
					}else{
						//TODO: go by version number and such!
					}
				}

			}else if(candidate.triangle_version > meshlet->triangles_version){
				//the triangle version on the GPU should never be greater than on the CPU
				assert(0);//this really should not happen
			}
		}

		//reallocate does only really apply if there
		if(reallocate){
			if(most_current.vertex_version == -1)
				assert(0);
			//TODO: reallocate the new vertices
			most_current.vertex_version ++;
			most_current.vertices = storage->vertex_buffer->getBlock(most_current.vertices->getSize());
		}
		//check if we already have new data
		if(most_current.triangle_version != -1)
			continue;

		//also create token and such!
		if(reallocate){
			//reallocate only if there is no initial allocation
			assert(0);
		}


		most_current.triangle_version = meshlet->triangles_version;
		most_current.vertex_version = meshlet->vertices_version;
		uploadGeometry(storage,most_current,meshlet.get());
		
		most_current.vertex_token = make_unique<weak_ptr<Meshlet>>(meshlet);

		//TODO: geometry textures

		//TODO: textures
		for(size_t k=0;k<meshlet->tex_patches.size();k++){
			//upload new textures
		}


	}


	headers = storage->patch_info_buffer->getBlock(meshlets.size());
	setupHeaders();
	/*
	//todo: fill the most current maps
	for(auto active_set : active_sets){

		if(active_set == nullptr)
			continue;
		for(auto id_ind : active_set->patch_inds){
			int id = id_ind.first;
			int ind = id_ind.second;


			if(map_requested.count(id) == 0)
				break;// the meshlet in question is not requested to be in this active set


			MeshletGPU &most_current = most_current_meshlets[id];
			MeshletGPU &candidate = active_set->meshlets[ind];

			//if(candidate.triangle_version == meshlets)
			if(false){
				if(most_current_meshlets.count(id)){
					//check which one is the newest
					MeshletGPU &most_current = most_current_meshlets[id];
					MeshletGPU &candidate = active_set->meshlets[ind];
					//TODO: at this point it would be appropriate to test for triangle_version
					if(candidate.vertex_version > most_current.vertex_version){
						//most_current_meshlets[id] = candidate;
						most_current = candidate;
					}
				}else{
					MeshletGPU meshletGpu;
					meshlet
							most_current_meshlets[id] = active_set->meshlets[ind];
				}
			}


			//TODO: what to do about the textures?
			//maybe we create a GPUMeshlet that collects the newest elements of neighbouring meshlets
		}
	}
	//upload / convert list to ne
	for(auto meshlet : meshlets_requested){
		int index = 0;
		if(most_current_meshlets.count(meshlet->id)){
			//there is a meshlet on the GPU
			//TODO: check if it is sufficient and in case generate it
			patch_inds[meshlet->id] = meshlets.size();
			meshlets.push_back(most_current_meshlets[meshlet->id]);

		}else{
			//there is no meshlet on the GPU
			//at least allocate the memory for this meshlet!
			//upload...
			MeshletGPU meshletGpu;


		}
		patch_inds[meshlet->id] = index;
	}
	*/
	/*
	for(auto patch : patches){

		shared_ptr<MeshletGPU> most_current = nullptr;
		int most_current_triangle = -1;
		int most_current_vertices = -1;
		for(auto old_set : active_sets){
			if(old_set->patch_inds.count(patch->id)){
				int ind = old_set->patch_inds[patch->id];
				//if
			}
		}
	}
	for(auto patch:)
	 */
}


ActiveSet::~ActiveSet() {

}

void ActiveSet::setupHeaders(){
	cout << "TODO: (IMPLEMENT THIS) ActiveSet::setupHeaders" << endl;
	vector<GpuPatchInfo> infos(meshlets.size());
	for(size_t i=0;i<meshlets.size();i++){
		GpuPatchInfo & info = infos[i];
		auto & meshlet = meshlets[i];
		info.vertex_start_ind = meshlet.vertices->getStartingIndex();
		info.triangle_start_ind = meshlet.triangles->getStartingIndex();
		info.patch_id = meshlet.id;

		//TODO: textures
		info.tex_layers = meshlet.textures.size();
		for(size_t k=0;k<meshlet.textures.size();k++){

		}

		//TODO: geometry texture
		if(meshlet.std_tex.tex != nullptr){
			info.std_texture.size;
			info.std_texture._size;
			info.std_texture.pos;
			info.std_texture.tex_coord_start_ind;
			info.std_texture.gl_tex_pointer = meshlet.std_tex.tex->getGlHandle();

		}
	}

	headers->upload(&infos[0]);


}


void ActiveSet::upload(shared_ptr<VertexBufConnector> &buf, vector<Vertex> &vertices) {
	assert(0);//it is not as trivial as this makes it seem
	vector<GpuVertex> gpu_verts(vertices.size());
	for(size_t i=0;i<vertices.size();i++){
		gpu_verts[i] = vertices[i].genGpuVertex();
	}
	buf->upload(&gpu_verts[0]);
}

void ActiveSet::upload(shared_ptr<TriangleBufConnector> &buf, vector<Triangle> &triangles) {
	assert(0); // it is not exactly clear where these triangles are pointing
	vector<GpuTriangle> gpu_tris(triangles.size());
	for(size_t i=0;i<triangles.size();i++){
		//triangles[i].
	}

	buf->upload(&gpu_tris[0]);
}

void ActiveSet::uploadGeometry(GpuStorage *storage, MeshletGPU &meshlet_gpu, Meshlet* meshlet){

	vector<GpuTriangle> gpu_triangles(meshlet->triangles.size());
	unordered_map<Vertex*,int> additional_verts;
	int offset = meshlet->vertices.size();
	int debug_triangle_count = meshlet->triangles.size();
	int count = 0;
	for(size_t k=0;k<meshlet->triangles.size();k++){
		auto & triangle = meshlet->triangles[k];
		for(auto i : {0,1,2}){
			if(triangle.vertices[i]->meshlet == meshlet){
				//vertex within this meshlet:
				//get index
				size_t ind = 	(reinterpret_cast<size_t>(triangle.vertices[i]) -
								reinterpret_cast<size_t>(&meshlet->vertices[0]))/sizeof(Vertex);
				gpu_triangles[k].indices[i] = ind;//TODO: check if this really works
			}else if(additional_verts.count(triangle.vertices[i])){
				//the vertex already is part of the additional vertices
				gpu_triangles[k].indices[i] = additional_verts[triangle.vertices[i]] + offset;
			} else{
				//the vertex has to be added to the additional vertices
				gpu_triangles[k].indices[i] = count + offset;
				assert(triangle.vertices[i]->p[3] == 1.0f);
				additional_verts[triangle.vertices[i]] = count;
				count ++;
			}
		}
	}

	vector<GpuVertex> gpu_verts(meshlet->vertices.size() + additional_verts.size());

	for(size_t i=0;i<meshlet->vertices.size();i++){
		gpu_verts[i] = meshlet->vertices[i].genGpuVertex();
	}
	for(auto pair : additional_verts){
		gpu_verts[pair.second + offset] = pair.first->genGpuVertex();

		assert(gpu_verts[pair.second + offset].p[3] == 1.0f);//debug remove
	}
	//debug measure
	int debug_count=0;
	for(auto vert : gpu_verts){
		cout << vert.p<< endl;
		debug_count++;
		assert(!isnan(vert.p[3]));
		int debug_size = meshlet->vertices.size();
		assert(vert.p[3] == 1.0f);
	}

	//reserve and upload vertices
	meshlet_gpu.vertices = storage->vertex_buffer->getBlock(gpu_verts.size());
	meshlet_gpu.vertices->upload(&gpu_verts[0]);

	//reserve and upload triangles
	meshlet_gpu.triangles = storage->triangle_buffer->getBlock(gpu_triangles.size());
	meshlet_gpu.triangles->upload(&gpu_triangles[0]);


	//TODO: tasks to transcribe the neighbouring vertices to the ones we have here


}
/*
ActiveSet::ActiveSet(GpuStorage *storage,
					 vector<shared_ptr<Meshlet>> patches,
					 MeshReconstruction *map,
					 LowDetailRenderer* low_detail_renderer,
					 TextureUpdater* texture_updater,
					 InformationRenderer* information_renderer,
					 bool initial,
					 bool debug1) {
	//TODO: for all the patches that needed to be reuploaded we also upload the
	//textures!

	gpu_geom_storage = storage;

	vector<shared_ptr<Meshlet>> new_mesh_patches_cpu;
	vector<shared_ptr<MeshletGpuHandle>> new_mesh_patches_gpu;
	vector<shared_ptr<TriangleBufConnector>> mesh_stitches_gpu;

	//combined download for triangles and vertices
	vector<CoalescedGpuTransfer::Task> coalesced_vertex_tasks;
	vector<GpuVertex> coalesced_vertices;

	vector<CoalescedGpuTransfer::Task> coalesced_triangle_tasks;
	vector<GpuTriangle> coalesced_triangles;

	//connect patches and gpu patches in a map until we are finally connecting
	unordered_map<Meshlet*, shared_ptr<MeshletGpuHandle>> patch_map;

	//also combined download for the header?

	//manually upload all the new patches:
	for(size_t i = 0; i < patches.size(); i++) {
		//create all the necessary buffers and also upload the triangles
		Meshlet *patch = patches[i].get();
		shared_ptr<MeshletGpuHandle> gpu_patch = patch->gpu.lock();

		//if the gpuPatch exists we should just secure it and will not need to download
		if(gpu_patch == nullptr) {
			//if there is no gpuPatch we create a new one.
			gpu_patch = make_shared<MeshletGpuHandle>(storage,
			                                            patch->vertices.size(),
			                                            patch->triangles.size());

			//TODO: get rid of the vertices in download thingy

			//patch->gpu = gpuPatch; //TODO: maybe only do this after downloading all of the patches is commenced
			new_mesh_patches_gpu.push_back(gpu_patch);
			new_mesh_patches_cpu.push_back(patches[i]);

			//TODO:
			//triggering this update not only when the gpu resources are
			//initially created
			CoalescedGpuTransfer::Task task;
			task.count  = patch->vertices.size();
			task.start  = coalesced_vertices.size();
			task.target = gpu_patch->vertices_source->getStartingPtr();
			coalesced_vertex_tasks.push_back(task);

			//create the necessary vertices
			for(size_t j = 0; j < patch->vertices.size(); j++) {
				coalesced_vertices.push_back(patch->vertices[j].genGpuVertex());
			}
		}

		if(gpu_patch == nullptr) {
			//at this point we really should have a gpu resource
			assert(0);
		}
		//Triangles are done further down
		//retainedMeshPatchesCpu.push_back(patches[i]);
		retained_mesh_patches.push_back(gpu_patch);
		patch->addActiveSet(this);
		patch_map[patch] = gpu_patch;

	}
	retained_mesh_patches_cpu = patches;
	CoalescedGpuTransfer::upload(coalesced_vertices,coalesced_vertex_tasks);

	//check if really all of the gpu patches are valid
	for(size_t i = 0; i < retained_mesh_patches.size(); i++) {
		if(retained_mesh_patches[i] == nullptr) {
			assert(0);
		}
	}

	//only upload triangles for patches we uploaded the vertices
	//TODO: also for triangles with chenged position of the header and similar issues
	for(size_t i = 0; i < new_mesh_patches_cpu.size(); i++) {
		Meshlet *patch = new_mesh_patches_cpu[i].get();
		shared_ptr<MeshletGpuHandle> gpu_patch = new_mesh_patches_gpu[i];

		CoalescedGpuTransfer::Task task;
		task.count = patch->triangles.size();
		task.start = coalesced_triangles.size();
		task.target = gpu_patch->triangles->getStartingPtr();
		coalesced_triangle_tasks.push_back(task);
		for(size_t j = 0; j < patch->triangles.size(); j++) {
			Triangle &triangle = patch->triangles[j];
			GpuTriangle gpu_triangle;
			shared_ptr<MeshletGpuHandle> gpu_this_pt = patch_map[triangle.points[0].getPatch()];
			for(size_t k = 0; k < 3; k++) {
				VertexReference pr = triangle.points[k];
				shared_ptr<MeshletGpuHandle> gpu_this_pt_debug =
						patch_map[triangle.points[k].getPatch()];
				//TODO: do something else to fix this
				//obviously this fails if we don't set gpu references
				gpu_triangle.patch_info_inds[k] =
						gpu_this_pt->patch_infos->getStartingIndex();
				gpu_triangle.indices[k] = pr.getIndex();
				gpu_triangle.tex_indices[k] = triangle.tex_indices[k];
			}
			coalesced_triangles.push_back(gpu_triangle);
		}
	}

	//setup lists with unique stitches of each type
	set<shared_ptr<DoubleStitch>> double_stitches;
	set<shared_ptr<TripleStitch>> triple_stitches;
	for(size_t i = 0; i < patches.size(); i++) {
		Meshlet *patch = patches[i].get();

		//now add the triangles for stitches to the set:
		for(size_t j = 0; j < patch->double_stitches.size(); j++) {
			shared_ptr<DoubleStitch> stitch = patch->double_stitches[j];
			Meshlet *debug_patch1 = stitch->patches[0].lock().get();
			Meshlet *debug_patch2 = stitch->patches[1].lock().get();
			if(stitch == nullptr) {
				continue;
			}
			//check if this patch is this stitches main patch
			if(stitch->patches[0].lock().get() != patch) {
				continue;
			}
			if(!stitch->isPartOfActiveSet(this)) {
				continue;
			}

			shared_ptr<TriangleBufConnector> gpu = stitch->triangles_gpu.lock();
			//TODO: also check if all the dependencies are met!!!!
			if(gpu == nullptr) {
				//in case the stitch is lacking a gpu representation:
				//create a new one
				gpu = storage->triangle_buffer->getBlock(stitch->triangles.size());
				stitch->triangles_gpu = gpu;
				//create a upload task:
				CoalescedGpuTransfer::Task task;
				task.count = stitch->triangles.size();
				task.start = coalesced_triangles.size();
				task.target = gpu->getStartingPtr();
				coalesced_triangle_tasks.push_back(task);
				for(size_t j = 0; j < stitch->triangles.size(); j++) {
					Triangle &triangle = stitch->triangles[j];
					GpuTriangle gpu_triangle;
					for(size_t k = 0; k < 3; k++) {
						VertexReference pr = triangle.points[k];
						shared_ptr<MeshletGpuHandle> gpu_this_pt =
								patch_map[pr.getPatch()];

						#ifdef DEBUG
						if(gpu_this_pt == nullptr) {
							//no triangle should have any invalid reference
							assert(0);
						}
						#endif // DEBUG

						gpu_triangle.patch_info_inds[k] =
								gpu_this_pt->patch_infos->getStartingIndex();
						gpu_triangle.indices[k] = pr.getIndex();
						gpu_triangle.tex_indices[k] = triangle.tex_indices[k];
					}
					coalesced_triangles.push_back(gpu_triangle);
				}
			}
			mesh_stitches_gpu.push_back(gpu);
			double_stitches.insert(stitch);
		}
		for(size_t j = 0; j < patch->triple_stitches.size(); j++) {
			shared_ptr<TripleStitch> stitch = patch->triple_stitches[j];
			if(stitch == nullptr) {
				continue;
			}
			//check if this patch is this stitches main patch
			if(stitch->patches[0].lock().get() != patch) {
				continue;
			}
			if(!stitch->isPartOfActiveSet(this)) {
				continue;
			}
			triple_stitches.insert(stitch);
		}
	}

	//Uploading one coalesced set of triangles and also upload singular triangles when needed
	CoalescedGpuTransfer::Task coalesced_task;
	coalesced_task.start = coalesced_triangles.size();
	for(auto trip_stitch : triple_stitches) {
		//no need to check the main patch and stuff since we already did this

		shared_ptr<TripleStitch> stitch = trip_stitch;
		CoalescedGpuTransfer::Task task;
		task.start = coalesced_triangles.size();

		if(stitch->triangles.size() == 0) {
			assert(0); //a empty stitch should not exist.
		}

		for(size_t j = 0; j < stitch->triangles.size(); j++) {
			Triangle &triangle = stitch->triangles[j];
			GpuTriangle gpu_triangle;
			for(size_t k = 0; k < 3; k++) {
				VertexReference pr = triangle.points[k];
				shared_ptr<MeshletGpuHandle> gpu_this_pt =
						patch_map[pr.getPatch()];
				gpu_triangle.patch_info_inds[k] =
						gpu_this_pt->patch_infos->getStartingIndex();
				gpu_triangle.indices[k] = pr.getIndex();
				gpu_triangle.tex_indices[k] = triangle.tex_indices[k];
			}
			coalesced_triangles.push_back(gpu_triangle);
		}
		//only create a new buffer for the triple stitch if the content changes (TODO)
		//or when there is nothing uploaded yet
		shared_ptr<TriangleBufConnector> gpu = stitch->triangles_gpu.lock();
		if(gpu == nullptr) {
			task.count = coalesced_triangles.size() - task.start;
			shared_ptr<TriangleBufConnector> gpu = 
					storage->triangle_buffer->getBlock(task.count);
			stitch->triangles_gpu = gpu;
			retained_triple_stitches.push_back(gpu);
			task.target = gpu->getStartingPtr();
			coalesced_triangle_tasks.push_back(task);
		} else {
			retained_triple_stitches.push_back(gpu);
		}
	}
	coalesced_task.count = coalesced_triangles.size() - coalesced_task.start;
	//now create a new buffer!
	if(coalesced_task.count != 0) {
		shared_ptr<TriangleBufConnector> gpu = 
				storage->triangle_buffer->getBlock(coalesced_task.count);
		coalesced_task.target = gpu->getStartingPtr();
		coalesced_triangle_tasks.push_back(coalesced_task);

		retained_triple_stitches_coalesced = gpu;
	}

	CoalescedGpuTransfer::upload(coalesced_triangles, coalesced_triangle_tasks);

	//retain the collected stitches in this active set
	retained_double_stitches = mesh_stitches_gpu;

	//debug
	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	//TODO: checkout what this is doing!!!
	vector<CoalescedGpuTransfer::Task> coalesced_tex_coord_tasks;
	vector<Vector2f> coalesced_tex_coords;

	uploadTexAndCoords_(new_mesh_patches_cpu, new_mesh_patches_gpu, map, initial);

	//after all the data is downloaded we "atomically" set the references to the gpu
	for(size_t i = 0; i < new_mesh_patches_cpu.size(); i++) {
		new_mesh_patches_cpu[i]->gpu = new_mesh_patches_gpu[i];
	}

	//uploading the header so we can update the reference textures
	reuploadHeaders();
	checkAndUpdateRefTextures_(retained_mesh_patches_cpu, map,texture_updater,information_renderer);
	//update the headers to show the new reference textures
	reuploadHeaders();
	//TODO: debug stupid bugs

	for(int i = 0; i < new_mesh_patches_cpu.size(); i++) {
		shared_ptr<Meshlet> patch = new_mesh_patches_cpu[i];
		for(int j = 0; j < patch->tex_patches.size(); j++) {
			if(patch->tex_patches[j]->gpu.lock() == nullptr) {
				assert(0);//this should not be here
			}
		}
	}
	for(int i = 0; i < patches.size(); i++) {
		shared_ptr<Meshlet> patch = patches[i];
		for(int j = 0; j < patch->tex_patches.size(); j++) {
			if(patch->tex_patches[j]->gpu.lock() == nullptr) {
				assert(0);//this should not be here
			}
		}
	}
}

ActiveSet::~ActiveSet() {

	//TODO: time this! make the download enforcable to prevent it from happening during time critical tasks.

	vector<shared_ptr<Meshlet>> patches_to_be_downloaded;
	patches_to_be_downloaded.reserve(retained_mesh_patches_cpu.size());
	for(shared_ptr<Meshlet> patch : retained_mesh_patches_cpu) {
		//remove this active set from all patches.
		if(patch->removeActiveSet(this)) {
			if(patch->gpu.lock() == nullptr) {
				assert(0); //the patch was owned by at least this active set. it should have a gpu representation
			}
			patches_to_be_downloaded.push_back(patch);
		}
	}

	//TODO: download the textures and vertices (texCoords?) to the according cpu structures
	vector<CoalescedGpuTransfer::DirectTask> coalesced_download_tasks;
	vector<tuple<shared_ptr<Meshlet>, vector<GpuVertex>>> downloaded_vertices;
	vector<tuple<shared_ptr<MeshTexture>, vector<Vector2f>>> downloaded_tex_coords;
	//vector<tuple<shared_ptr<MeshTexture>, vector<Vector2f>>> downloaded_tex_coords;
	//for the beginning lets do the texture transfer uncoalesced
	for(shared_ptr<Meshlet> patch : patches_to_be_downloaded) {
		//TODO: add a retain count to the gpu handle
		//TODO: not so fast there is a active set list for each Meshlet (remove these todos after implementation)
		shared_ptr<MeshletGpuHandle> patch_gpu = patch->gpu.lock();

		//check if the gpu ever changed anything with the vertices
		if(patch_gpu->gpu_vertices_changed) {
			//and if add to the coalesced download
			CoalescedGpuTransfer::DirectTask task;
			int count = patch_gpu->vertices_dest->getSize();
			task.src = patch_gpu->vertices_dest->getStartingPtr();
			task.byte_count = sizeof(GpuVertex) * count;
			vector<GpuVertex> vertices(count);
			task.dst = &vertices[0];
			tuple<shared_ptr<Meshlet>, vector<GpuVertex>> t =
					make_tuple(patch, move(vertices));
			downloaded_vertices.push_back(move(t));
			coalesced_download_tasks.push_back(task);
		}

		//DOWNLOAD TO INTERMEDIATE VECTORS WHICH THEN SHOULD BE ATOMICLY STD::MOVE D
		//https://stackoverflow.com/questions/43243977/assigning-vectors-without-copying-them
		{
			shared_ptr<MeshTextureGpuHandle> tex_patch_gpu = patch->geom_tex_patch->gpu.lock();
			shared_ptr<MeshTexture> tex_patch = patch->geom_tex_patch;
			if(tex_patch == nullptr) {
				assert(0); //the geometry texture should exist in any case!!!!!!!!
			}
			if(tex_patch_gpu == nullptr) {
				assert(0);//the geometry texture should exist in any case.
			}

			if(tex_patch_gpu->gpu_data_changed) {
				cv::Rect2i roi = tex_patch_gpu->tex->getRect();
				cv::Mat mat(roi.height, roi.width, CV_32FC4);
				//downloading process
				castF16SurfaceToF32Buffer(tex_patch_gpu->tex->getCudaSurfaceObject(),
				                          roi.x,roi.y,roi.width,roi.height,
				                          (float*) mat.data, 4);

				tex_patch->mat = mat;
				tex_patch_gpu->gpu_data_changed = false;//in case somebody really safes this gpu patch last minute
			}
			CoalescedGpuTransfer::DirectTask task;
			int count = tex_patch_gpu->coords->getSize();
			task.src = tex_patch_gpu->coords->getStartingPtr();
			task.byte_count = sizeof(Vector2f) * count;
			vector<Vector2f> target(count);
			//vector<Vector2f> target(count);
			task.dst = static_cast<void*>(&target[0]);
			coalesced_download_tasks.push_back(task);
			downloaded_tex_coords.push_back(make_tuple(tex_patch, move(target)));
		}

		//download color textures
		for(size_t i = 0; i < patch->tex_patches.size(); i++) {
			shared_ptr<MeshTexture> tex_patch = patch->tex_patches[i];
			shared_ptr<MeshTextureGpuHandle> tex_patch_gpu = tex_patch->gpu.lock();
			if(tex_patch == nullptr) {
				assert(0); //the texture should exist if it is in this list
			}
			if(tex_patch_gpu == nullptr) {
				assert(0);//the gpu texture should exist if it is in this list
			}


			if(tex_patch_gpu->gpu_data_changed) {
				cv::Rect2i roi = tex_patch_gpu->tex->getRect();
				cv::Mat mat(roi.height,roi.width,CV_8UC4);
				//downloading process
				tex_patch_gpu->tex->downloadData(mat.data);
				tex_patch->mat = mat;
				tex_patch_gpu->gpu_data_changed = false;//in case somebody really safes this gpu patch last minute
			}
			CoalescedGpuTransfer::DirectTask task;
			int count = tex_patch_gpu->coords->getSize();
			task.src = tex_patch_gpu->coords->getStartingPtr();
			task.byte_count = sizeof(Vector2f) * count;
			vector<Vector2f> target(count);
			task.dst = static_cast<void*>(&target[0]);
			coalesced_download_tasks.push_back(task);
			downloaded_tex_coords.push_back(make_tuple(tex_patch, move(target)));
		}

		//TODO: download label textures

		//OBVIOUSLY ALWAYS CHECK IF THERE IS A DOWNLOADWORTHY UPDATE

		//set the retain count to zero

		//if necessary setup a coalesced memory transfer for the vertices

		//same for the texture

		//same for texCoords

	}
	//execute the download
	CoalescedGpuTransfer::download(coalesced_download_tasks);

	//copy the buffer over to the according elements
	//vertices
	for(size_t i = 0; i < downloaded_vertices.size(); i++) {
		shared_ptr<Meshlet> patch = get<0>(downloaded_vertices[i]);
		vector<GpuVertex> &verts_gpu = get<1>(downloaded_vertices[i]);
		assert(verts_gpu.size() == patch->vertices.size());
		//TODO: secure this with a mutex!!!!
		for(size_t j = 0; j < patch->vertices.size(); j++) {
			patch->vertices[j].n = verts_gpu[j].n;
			patch->vertices[j].p = verts_gpu[j].p;
		}
		//TODO: maybe mutex here

		//TODO: secure this with a mutex!!!!
	}
	for(auto t : downloaded_tex_coords) {
		//TODO: maybe we also want to use mutexes here and stuff
		get<0>(t)->tex_coords = move(get<1>(t));
	}
}


//ideally this method only got patches with newly generated gpuPatches but probably it doesn't have any gpuTextures
void ActiveSet::uploadTexAndCoords_(
		vector<shared_ptr<Meshlet>> &patches,
		vector<shared_ptr<MeshletGpuHandle>> &patches_gpu,
		const MeshReconstruction *map, bool initial) {

	vector<CoalescedGpuTransfer::Task> coalesced_tex_coord_tasks;
	vector<Vector2f> coalesced_tex_coords;

	for(size_t k = 0; k < patches.size(); k++) {
		shared_ptr<Meshlet> patch = patches[k];
		shared_ptr<MeshletGpuHandle> patch_gpu = patches_gpu[k];
		if(patch_gpu == nullptr) {
			assert(0); //the gpuPatch should exist at this point
		}

		shared_ptr<MeshTexture> tex_patch = patch->geom_tex_patch; //TODO: here was a crash... why is texPatch zero
		if(!tex_patch->mat.empty()) {//if it is empty there is nothing to do here!!!!
			//but if there is content upload it to the gpu
			//TODO: ideally this should be 1/2 lines
			if(patch_gpu->geom_tex != nullptr) {
				assert(0);//as all of these patches are reuploads this geomTex should not exist
			}

			int width = tex_patch->mat.cols;
			int height = tex_patch->mat.rows;

			shared_ptr<MeshTextureGpuHandle> tex_patch_gpu =
					make_shared<MeshTextureGpuHandle>(gpu_geom_storage->tex_pos_buffer,
					                                  tex_patch->tex_coords.size(),
					                                  map->tex_atlas_geom_lookup_.get(),
					                                  map->tex_atlas_stds_.get(),//TODO: the references are supposed to be filled at "CheckAndUpdateRefTextures"
					                                  width, height);

			//now do the uploading
			cv::Rect2i roi = tex_patch_gpu->tex->getRect();

			//upload the texture:
			//maybe we should also hold the texture with a mutex
			cudaSurfaceObject_t surface =
					tex_patch_gpu->tex->getCudaSurfaceObject();
			//TODO: this definitely is something that we could concatenate
			castF32BufferToF16Surface(surface, roi.x, roi.y, roi.width, roi.height,
			                          (float*) tex_patch->mat.data, 4);

			//create the task for the tex coord upload

			if(tex_patch->tex_coords.size() == 0) {
				assert(0);//either the tex coords would reside on gpu (then we wouldn't reach this code.
				//or they are on cpu (in which case we wouldn't reach this assert.
			}
			CoalescedGpuTransfer::Task task;
			task.count  = tex_patch->tex_coords.size();
			task.target = tex_patch_gpu->coords->getStartingPtr();
			task.start  = coalesced_tex_coords.size();
			coalesced_tex_coord_tasks.push_back(task);
			coalesced_tex_coords.insert(coalesced_tex_coords.end(),
			                            tex_patch->tex_coords.begin(),
			                            tex_patch->tex_coords.end());

			//store/secure the texture
			patch_gpu->geom_tex = tex_patch_gpu;
			tex_patch->gpu = tex_patch_gpu;

		} else {
			if(!initial) {
				assert(0);//the mesh patch has to have either geometry data on gpu or on cpu
			}
		}

		for(size_t i = 0; i < patch->tex_patches.size(); i++) {
			tex_patch = patch->tex_patches[i];
			if(!tex_patch->mat.empty()) {
				shared_ptr<MeshTextureGpuHandle> tex_patch_gpu = patch_gpu->texs[i];
				if(tex_patch_gpu != nullptr) {
					assert(0); // we should only be doing this if the gpu texture is zero
				}
				if(tex_patch->mat.empty()) {
					assert(0);//this really should not be empty
				}
				int width  = tex_patch->mat.cols;
				int height = tex_patch->mat.rows;
				tex_patch_gpu = 
						make_shared<MeshTextureGpuHandle>(gpu_geom_storage->tex_pos_buffer,
						                                  tex_patch->tex_coords.size(),
						                                  nullptr,
						                                  map->tex_atlas_rgb_8_bit_.get(),//TODO: where to get these from?
						                                  width, height);

				tex_patch->mat_mutex.lock();
				patch->tex_patches[i]->gpu = tex_patch_gpu;
				tex_patch_gpu->tex->uploadData(tex_patch->mat.data);
				tex_patch->mat_mutex.unlock();

				//tex coordinate upload
				CoalescedGpuTransfer::Task task;
				task.count  = tex_patch->tex_coords.size();
				task.start  = coalesced_tex_coords.size();
				task.target = tex_patch_gpu->coords->getStartingPtr();
				coalesced_tex_coord_tasks.push_back(task);
				coalesced_tex_coords.insert(coalesced_tex_coords.end(),
				                            tex_patch->tex_coords.begin(),
				                            tex_patch->tex_coords.end());

				patch_gpu->texs[i] = tex_patch_gpu;
				tex_patch->gpu = tex_patch_gpu;

			} else {
				assert(0); // if the texture is not filled it should not exist in this list
			}
		}

		tex_patch = patch->label_tex_patch;
		if(tex_patch != nullptr) {
			//also do it for the label texture
			if(!tex_patch->mat.empty()) {
				//TODO:
			} else {
				//it would be ok if there is no label data
			}
		}
	}

	//concatenated upload of tex coords
	CoalescedGpuTransfer::upload(coalesced_tex_coords, coalesced_tex_coord_tasks);
}



void ActiveSet::checkAndUpdateRefTextures_(
		const vector<shared_ptr<Meshlet>> &patches,
		MeshReconstruction *reconstruction,
		TextureUpdater *texture_updater,
		InformationRenderer* information_renderer) {
	vector<shared_ptr<Meshlet>> dated_patches;
	vector<shared_ptr<MeshTexture>> dated_textures;

	for(size_t i = 0; i < patches.size(); i++) {
		shared_ptr<Meshlet> patch = patches[i];
		if(!patch->isPartOfActiveSetWithNeighbours(this)) {
			continue;
		}
		shared_ptr<MeshletGpuHandle> patch_gpu = patch->gpu.lock();
		shared_ptr<MeshTextureGpuHandle> tex_patch_gpu = patch_gpu->geom_tex;
		if(tex_patch_gpu != nullptr) {
			if(!tex_patch_gpu->checkRefTexDependencies()) {
				dated_patches.push_back(patch);
				dated_textures.push_back(patch->geom_tex_patch);
			}
		}
		//TODO: also update refTextures for the labelTextures if available
	}
	texture_updater->genLookupTex(reconstruction, this, dated_patches, dated_textures, information_renderer, true);//true: dilate the resulting textures
}

void ActiveSet::drawDoubleStitches() {
	for(size_t i = 0; i < retained_double_stitches.size(); i++) {
		TriangleBufConnector &current_stitch = *retained_double_stitches[i];
		int slot  = static_cast<int>(current_stitch.getStartingIndex());
		int count = static_cast<int>(current_stitch.getSize());
		glDrawArrays(GL_TRIANGLES, slot * 3, count * 3);
	}
}

void ActiveSet::drawTripleStitches() {
	if(retained_triple_stitches_coalesced == nullptr) {
		return;
	}
	TriangleBufConnector &current_stitch = *retained_triple_stitches_coalesced;
	int slot  = current_stitch.getStartingIndex();
	int count = current_stitch.getSize();
	glDrawArrays(GL_TRIANGLES, slot * 3, count * 3);
}

void ActiveSet::drawPatches() {
	//first iterate over all the mesh patches:
	for(size_t i = 0; i < retained_mesh_patches.size(); i++) {
		MeshletGpuHandle &current_patch = *retained_mesh_patches[i];
		//first check that all the textures are resident
		if(current_patch.geom_tex != nullptr) {
			current_patch.geom_tex->tex->getTex()->makeResidentInThisThread();
			current_patch.geom_tex->ref_tex->getTex()->makeResidentInThisThread();
		}
		for(size_t j = 0; j < current_patch.tex_count; j++) {
			if(current_patch.texs[j] != nullptr) {
				current_patch.texs[j]->tex->getTex()->makeResidentInThisThread();
			}
		}
		//now do the real rendering:
		//TODO: maybe put this code into the active set class.
		int slot  = current_patch.triangles->getStartingIndex();
		int count = current_patch.triangles->getSize();
		glDrawArrays(GL_TRIANGLES, slot * 3, count * 3);
		//TODO: assemble list for draw arrays instanced in base arrays
	}
}

void ActiveSet::drawEverything() {
	drawPatches();
	drawDoubleStitches();
	drawTripleStitches();
}

void ActiveSet::reuploadHeaders() {
	vector<GpuPatchInfo>  coalesced_infos;
	vector<GpuPatchInfo*> coalesced_info_pos;
	for(shared_ptr<Meshlet> patch : retained_mesh_patches_cpu) {
		GpuPatchInfo info;
		shared_ptr<MeshletGpuHandle> gpu = patch->gpu.lock();
		info.patch_id = patch->id;
		info.debug1 = patch->debug1;

		//put on the texturing information:
		if(gpu->geom_tex != nullptr) {
			info.std_texture = gpu->geom_tex->genTexInfo();

			info.std_texture.gl_ref_tex_ptr_DEBUG = 
					gpu->geom_tex->ref_tex->getGlHandle();
			cv::Rect2i roi = gpu->geom_tex->ref_tex->getRect();
			info.std_texture.ref_tex_pos_DEBUG = 
					Vector2f(roi.x, roi.y) * (1.0f / 1024.0f);
		} else {
			info.std_texture.tex_coord_start_ind = 0;
			info.std_texture.gl_tex_pointer = 0;
		}

		info.tex_layers = 0;
		patch->tex_patches_mutex.lock();
		for(size_t i = 0; i < patch->tex_patches.size(); i++) {
			shared_ptr<MeshTextureGpuHandle> gpu_tex_patch =
					patch->tex_patches[i]->gpu.lock();
			if(gpu_tex_patch == nullptr) {
				//this really should not happen.... so why is this?
				//TODO: place assert here and check
				continue;
			}
			info.texture_infos[info.tex_layers] = gpu_tex_patch->genTexInfo();
			info.tex_layers++;
		}
		patch->tex_patches_mutex.unlock();
		//TODO. this texture upload

		//the labelling textures:
		patch->label_tex_patch_mutex.lock();
		if(patch->label_tex_patch != nullptr) {
			shared_ptr<MeshTextureGpuHandle> gpu_tex_patch =
					patch->label_tex_patch->gpu.lock();
			if(gpu_tex_patch == nullptr) {
				//this really should not happen.... so why is this?
				//TODO: place assert here and check
				assert(0);
				continue;
			}
			info.segmentation_texture = gpu_tex_patch->genTexInfo();//tex->genTexInfo();
			info.segmentation_tex_valid = true;
		}
		patch->label_tex_patch_mutex.unlock();

		info.vertex_source_start_ind = gpu->vertices_source->getStartingIndex();
		info.vertex_destination_start_ind = gpu->vertices_dest->getStartingIndex();
		info.triangle_start_ind = gpu->triangles->getStartingIndex();

		coalesced_infos.push_back(info);
		coalesced_info_pos.push_back(gpu->patch_infos->getStartingPtr());
	}
	CoalescedGpuTransfer::upload(coalesced_infos, coalesced_info_pos);
}

void ActiveSet::checkForCompleteGeometry() {
	for(size_t i = 0; i < retained_mesh_patches.size(); i++) {
		if(retained_mesh_patches[i] == nullptr) {
			assert(0);
		}
		if(retained_mesh_patches_cpu[i]->geom_tex_patch == nullptr) {
			assert(0); // something is very fishy <°)))<
		}
		if(retained_mesh_patches[i]->geom_tex == nullptr) {
			//assert(0);
		}
	}
}
 */