//
// Created by simon on 06.12.19.
//

#include <gpu/gpu_mesh_structure.h>


GpuTextureInfo TextureLayerGPU::genGpuTextureInfo(){
	GpuTextureInfo info;
	assert(0);
	return info;
}

MeshletGPU::~MeshletGPU(){

	if(vertex_token == nullptr)
		return;

	assert(0);//TODO: check if we should download data!!!
}