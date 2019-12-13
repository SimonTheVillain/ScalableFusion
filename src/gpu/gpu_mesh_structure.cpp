//
// Created by simon on 06.12.19.
//

#include <gpu/gpu_mesh_structure.h>
#include <base/mesh_structure.h>


GpuTextureInfo TextureLayerGPU::genGpuTextureInfo(){
	GpuTextureInfo info;
	info = tex->genTexInfo(coords->getStartingIndex());
	return info;
}

MeshletGPU::~MeshletGPU(){

	if(vertex_token == nullptr)
		return;

	shared_ptr<Meshlet> meshlet = vertex_token->lock();
	if(meshlet == nullptr)
		return; //the meshlet we would have stored back the data doesn't exist anymore

	meshlet->vertices_mutex.lock(); //TODO: maybe make this a shared mutex so the vertices can be read nonetheless

	if(meshlet->triangles_version > triangle_version){
		return;//the meshlet we were to save back is more up to date than we would like
	}else if(meshlet->triangles_version < triangle_version){
		assert(0);//this would mean somebody updated the mesh on the GPU (should not happen)
	}

	if(meshlet->vertices_version > vertex_version)
		assert(0); // this would mean somebody updated vertex positions on the CPU (should not happen)

	vector<GpuVertex> gpu_verts(meshlet->vertices.size());
	vertices->download(&gpu_verts[0],0,meshlet->vertices.size());//only download the vertices of interest

	for(int i=0;i<meshlet->vertices.size();i++){
		meshlet->vertices[i].p = gpu_verts[i].p;
		meshlet->vertices[i].n = gpu_verts[i].n;
	}
	meshlet->vertices_version = vertex_version;
	meshlet->vertices_mutex.unlock();


	//TODO: remove everything needed to transcribe neighbouring vertex positions to this patch!!!


}