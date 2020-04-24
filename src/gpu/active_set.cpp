#include <gpu/active_set.h>

#include <tuple>
#include <unordered_map>

#include <Eigen/Core>

#include <gpu/gpu_storage.h>
#include <base/mesh_structure.h>
#include <cuda/gpu_errchk.h>
#include <cuda/float16_utils.h>
#include <mesh_reconstruction.h>
#include <cuda/geom_update.h>

using namespace std;
using namespace Eigen;

std::mutex ActiveSet::mutex;

//TODO: find a way to allocate textures and texture pointers in this thing:
//Meshlet textures could have a flag "do_not_upload" and a new version number to indicate a
//update that requires a newly allocated texture
ActiveSet::ActiveSet(GpuStorage *storage,
					 vector<shared_ptr<Meshlet>> meshlets_requested,
					 vector<shared_ptr<ActiveSet>> active_sets,
					 vector<bool> allocate_new_verts){

	std::lock_guard<std::mutex> guard(mutex);

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
		meshlet_inds[meshlet->id] = meshlets.size();
		meshlets.emplace_back();
		MeshletGPU &most_current = meshlets.back();
		most_current.id = meshlet->id;

		//check if the data already exists in one of the active sets
		for(auto active_set : active_sets){
			if(active_set == nullptr)
				continue;
			if(active_set->meshlet_inds.count(meshlet->id) == 0)
				continue;
			int id = meshlet->id;
			int ind = active_set->meshlet_inds[id];

			MeshletGPU &candidate = active_set->meshlets[ind];

			if(candidate.triangle_version == meshlet->triangles_version){
				//store the most current versions of texture + vertices
				most_current.triangle_version = candidate.triangle_version;
				most_current.triangles = candidate.triangles;
				if(candidate.vertex_version >= most_current.vertex_version){
					most_current.vertex_version = candidate.vertex_version;
					most_current.vertices = candidate.vertices;

					// token taken from most current vertex
					if(most_current.vertex_token == nullptr)
						most_current.vertex_token = move(candidate.vertex_token);

					//lookup and geometry texture
					{
						most_current.geom_lookup_tex = candidate.geom_lookup_tex;
						auto &candid_texture = candidate.std_tex;
						if(candid_texture.tex_version >= most_current.std_tex.tex_version){
							most_current.std_tex.tex_coord_version = candid_texture.tex_coord_version;
							most_current.std_tex.coords = candid_texture.coords;

							most_current.std_tex.tex_version = candid_texture.tex_version;
							most_current.std_tex.tex = candid_texture.tex;
							if(most_current.std_tex.token == nullptr)
								most_current.std_tex.token = move(candid_texture.token);
						}

					}

				}


				//TODO: all the other textures
				for(size_t k=0;k<candidate.textures.size();k++){
					auto &candid_texture = candidate.textures[k];
					if(most_current.textures.size() > k){
						//TODO: check version number and such!
						most_current.textures[k] = candidate.textures[k];
					}else{
						//TODO: check version number and such!
						most_current.textures.push_back(candidate.textures[k]);
					}
				}

			}else if(candidate.triangle_version > meshlet->triangles_version){
				//the triangle version on the GPU should never be greater than on the CPU
				assert(0);//this really should not happen
			}
		}

		//reallocate does only really apply if there has been data before. if there has not been. we want
		if(reallocate){
			if(most_current.vertex_version == -1){
				assert(0);

			}
			//TODO: reallocate the new vertices
			most_current.vertex_version ++;
			most_current.vertices = storage->vertex_buffer->getBlock(most_current.vertices->getSize());

			if(most_current.std_tex.tex_version == -1)
				assert(0);
			most_current.std_tex.tex_version ++;
			cv::Size2i size = most_current.std_tex.tex->getRect().size();
			most_current.std_tex.tex = storage->tex_atlas_stds_->getTexAtlasPatch(size);

		}

		most_current.debug = 0;


		if(most_current.vertex_token == nullptr){
			//create a vertex token if it got lost somehow
			most_current.vertex_token = make_unique<weak_ptr<Meshlet>>(meshlet);
		}
		if(most_current.std_tex.token == nullptr){
			//create a new token if it got lost somehow
			most_current.std_tex.token = make_unique<weak_ptr<MeshTexture>>(meshlet->geom_tex_patch);
		}
		for(size_t k=0;k<meshlet->tex_patches.size();k++){
			//TODO: check if token for the textures exist or got lost
			//recreate token if they got lost somehow

		}
		for(size_t k = 0; k < most_current.textures.size();k++){
			if(most_current.textures[k] == nullptr){
				assert(0);
			}
			if(most_current.textures[k]->token == nullptr){
				assert(0); // should not happen. at this point we already want some
			}
		}

		//upload geometry data if there was none on the GPU
		if(most_current.triangle_version == -1){
			//no geometry found on the gpu: upload from cpu

			//also create token and such!
			if(reallocate){
				//reallocate only if there is no initial allocation
				assert(0);
			}


			most_current.triangle_version = meshlet->triangles_version;
			most_current.vertex_version = meshlet->vertices_version;
			uploadGeometry(storage,most_current,meshlet.get());

			most_current.vertex_token = make_unique<weak_ptr<Meshlet>>(meshlet);


			if(meshlet->triangles.size()){
				assert(most_current.triangles != nullptr);
			}
		}

		//upload geometry texture if there was none on the GPU
		if(most_current.std_tex.tex == nullptr){
			most_current.debug = 1;
			if(meshlet->geom_tex_patch != nullptr){
				most_current.debug = 2;
				if(!meshlet->geom_tex_patch->mat.empty()){
					most_current.debug = 3;
					most_current.std_tex.create(meshlet->geom_tex_patch,storage->tex_atlas_stds_,storage->tex_pos_buffer);
				}
			}
		}

		//upload color textures if there was none on the GPU
		if(most_current.textures.size() == 0){

			for(size_t k=0;k<meshlet->tex_patches.size();k++){
				//upload new textures
				shared_ptr<MeshTexture> &tex_cpu = meshlet->tex_patches[k];


				most_current.textures.emplace_back();
				shared_ptr<TextureLayerGPU> &tex_gpu = most_current.textures[k];
				tex_gpu = make_shared<TextureLayerGPU>();
				tex_gpu->create(tex_cpu,storage->tex_atlas_rgb_8_bit_,storage->tex_pos_buffer);

			}

		}



		if(most_current.vertex_token == nullptr){
			cout << " all this fuzz and still no valid token?" << endl;
			assert(0);
			//create a vertex token if it got lost somehow
			//	most_current.vertex_token = make_unique<weak_ptr<Meshlet>>(meshlet);
		}
	}


	headers = storage->patch_info_buffer->getBlock(meshlets.size());
	setupHeaders();


	setupTranscribeStitchesTasks(meshlets_requested);

}


ActiveSet::~ActiveSet() {
	mutex.lock();
	//calling all the destructors that will be downloading the data to CPU
	meshlets.clear();


	mutex.unlock();
}

void ActiveSet::setupHeaders(bool debug){
	//cout << "TODO: (IMPLEMENT THIS) ActiveSet::setupHeaders" << endl;
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
			info.texture_infos[k] = meshlet.textures[k]->genGpuTextureInfo();
		}

		//TODO: geometry texture
		if(meshlet.std_tex.tex != nullptr){
			info.std_texture = meshlet.std_tex.genGpuTextureInfo();
			meshlet.std_tex.tex->getTex()->makeResidentInThisThread();
		}else{
			if(debug){
				cout << "[ActiveSet::setupHeaders] there is no std_tex yet" << endl;
				//this should be ok as long as this only happens within an extend step.
				assert(0);
			}
		}
	}

	headers->upload(&infos[0]);


}

void ActiveSet::setupTranscribeStitchesTasks(vector<shared_ptr<Meshlet>> &	meshlets_requested){

	//TODO: setting this up every update might be overly expensive, so reuse as often as possible


	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
	//vector<gpu::GeometryUpdate::TranscribeStitchTask> transcribe_tasks;
	//unordered_map<Meshlet,int> meshlets_to_ind;
	//vector<GpuVertex*> vertices_ptr_gpu(meshlets.size());



	for(int i=0;i<meshlets.size();i++){

		auto &meshlet_gpu = meshlets[i];
		auto &meshlet = meshlets_requested[i];

		if(!containsNeighbours(meshlet))
			continue;

		//mapping from (neighbouring) meshlets (CPU) to index in the list of neighbours
		unordered_map<Meshlet*,int> meshlets_to_ind;

		//pointer to each neighbours vertices on gpu
		vector<GpuVertex*> vertices_ptr_gpu(meshlet->neighbours.size());
		for(size_t j=0;j<meshlet->neighbours.size();j++){
			weak_ptr<Meshlet> nb_weak = meshlet->neighbours[j];
			shared_ptr<Meshlet> nb = nb_weak.lock();
			MeshletGPU* nb_gpu = getGpuMeshlet(nb);
			if(nb == nullptr)
				assert(0);

			meshlets_to_ind[nb.get()] = j;
			vertices_ptr_gpu[j] = nb_gpu->vertices->getStartingPtr();
		}

		//map from CPU vertex pointers to their indices within the given meshlets (only if they are neighbours though)
		unordered_map<Vertex*,int> vertex_indices;
		Meshlet* meshlet_ptr = meshlet.get();
		for(int j=0;j<meshlet->triangles.size();j++){
			Triangle &tri = meshlet->triangles[j];
			for(int k : {0, 1, 2}){
				if(tri.vertices[k]->meshlet != meshlet_ptr){
					//we need to add this vertex to the map of neighbouring vertices
					vertex_indices[tri.vertices[k]] = tri.local_indices[k];
				}
			}
		}
		int task_count = vertex_indices.size();

		//TODO: 2020 Simon! Find out whats going on here!
		//TODO: DELETE AFTER FIXED! invalid next size (fast) happening here! (at the destructor)
		vector<MeshletGPU::TranscribeBorderVertTask> tasks(task_count);

		int count = 0;
		for(auto vert : vertex_indices){ //iterate over all vertices that are not local to this patch
			tasks[count].ind_local = vert.second;
			//calculate index by subtracting pointers (ptr(vertex) - ptr(first vert of according meshlet)
			int ind_in_neighbour = vert.first - &vert.first->meshlet->vertices[0];
			//cout << ind_in_neighbour << endl;
			tasks[count].ind_in_neighbour = ind_in_neighbour;//check if these indices make sense
			tasks[count].ind_neighbour = 0;

			//check if the neighbour even exists in our list of neighbours
			if(meshlets_to_ind.count(vert.first->meshlet)){
				tasks[count].ind_neighbour = meshlets_to_ind[vert.first->meshlet];
			}else{
				cout << "DEBUG:!!!! That vertex is probably part of an invalid meshlet" << endl;
				assert(0);
			}
			count ++;
		}


		//setting up transcribe tasks on a per gpu meshlet basis
		int byte_count = sizeof(GpuVertex*) * vertices_ptr_gpu.size();
		cudaMalloc(&meshlet_gpu.gpu_neighbour_vertices,byte_count);
		cudaMemcpy(meshlet_gpu.gpu_neighbour_vertices,&vertices_ptr_gpu[0],byte_count,cudaMemcpyHostToDevice);

		cudaDeviceSynchronize();
		gpuErrchk(cudaPeekAtLastError());

		byte_count = sizeof(MeshletGPU::TranscribeBorderVertTask) * vertex_indices.size();
		cudaMalloc(&meshlet_gpu.gpu_vert_transcribe_tasks,byte_count);
		cudaMemcpy(meshlet_gpu.gpu_vert_transcribe_tasks,&tasks[0],byte_count,cudaMemcpyHostToDevice);


		cudaDeviceSynchronize();
		gpuErrchk(cudaPeekAtLastError());

		meshlet_gpu.gpu_vert_transcribe_task_count = vertex_indices.size();


	}


	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

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
	int max_local_vertex_index = 0;
	for(auto & tri : meshlet->triangles)
		for(size_t i : {0 ,1 ,2})
			max_local_vertex_index =
					std::max(max_local_vertex_index,tri.local_indices[i]);

	int debug_vertex_count = meshlet->vertices.size();
#ifdef DEBUG_PICKY
	//this essentially means that some of the vertices are unconnected within this patch.
	assert(max_local_vertex_index + 1 >= debug_vertex_count);
#endif
	vector<Vertex*> vertices(std::max((size_t)max_local_vertex_index+1,meshlet->vertices.size()));
	vector<GpuVertex> gpu_vertices(vertices.size());
	for(size_t i=0;i<meshlet->vertices.size();i++)
		vertices[i] = &meshlet->vertices[i]; // this is necessary since some of the vertices do not have triangles
											 // should we clean the mesh from these vertices?

	for(auto & tri : meshlet->triangles)
		for(size_t i : {0,1,2})
			vertices[ tri.local_indices[i] ] = tri.vertices[i];

	for(size_t i=0;i<vertices.size();i++)
		gpu_vertices[i] = vertices[i]->genGpuVertex();

	for(size_t i=0;i<meshlet->triangles.size();i++)
		for(size_t k : {0,1,2})
			gpu_triangles[i].indices[k] = meshlet->triangles[i].local_indices[k];

	//debug measure
	int debug_count=0;
	for(auto vert : gpu_vertices){
		//cout << vert.p<< endl;
		debug_count++;
		assert(!isnan(vert.p[3]));
		int debug_size = meshlet->vertices.size();
		assert(vert.p[3] == 1.0f);
	}

	//reserve and upload vertices
	meshlet_gpu.vertices = storage->vertex_buffer->getBlock(gpu_vertices.size());
	meshlet_gpu.vertices->upload(&gpu_vertices[0]);

	//reserve and upload triangles
	meshlet_gpu.triangles = storage->triangle_buffer->getBlock(gpu_triangles.size());
	meshlet_gpu.triangles->upload(&gpu_triangles[0]);

	//TODO: tasks to transcribe the neighbouring vertices to the ones we have here


}
MeshletGPU* ActiveSet::getGpuMeshlet(shared_ptr<Meshlet> meshlet) {
	if(meshlet_inds.count(meshlet->id)){
		int ind = meshlet_inds[meshlet->id];
		return &meshlets[ind];
	}
	return nullptr;

}

bool ActiveSet::containsNeighbours(shared_ptr<Meshlet> meshlet) {
	for(auto nb : meshlet->neighbours){
		shared_ptr<Meshlet> nb_shared = nb.lock();
		if(nb_shared == nullptr){
			assert(0);
		}
		if(meshlet_inds.count(nb_shared->id) == 0){
			return false; // found neighbour not contained in this active set
		}
	}
	return true;
}

bool ActiveSet::hasAllGeometry(){
	for(MeshletGPU &meshlet : meshlets){
		if(meshlet.std_tex.tex == nullptr)
			return false;
		if(meshlet.vertices == nullptr)
			return false;
		if(meshlet.triangles == nullptr)
			return false;
	}
	return true;
}

void ActiveSet::assertAllGeometry(){
	for(MeshletGPU &meshlet : meshlets){
		if(meshlet.std_tex.tex == nullptr)
			assert(0);
		if(meshlet.vertices == nullptr)
			assert(0);
		if(meshlet.triangles == nullptr)
			assert(0);
	}
}