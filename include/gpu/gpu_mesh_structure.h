//
// Created by simon on 15.11.19.
//

#ifndef SCALABLE_FUSION_GPU_MESH_STRUCTURE_H
#define SCALABLE_FUSION_GPU_MESH_STRUCTURE_H
/** TODO:
 * put everything in here that manages gpu resources
 */
#include <memory>
#include "gpu_buffer.h"


class Meshlet;
class MeshTexture;
class GpuStorage;
class TexAtlasPatch;

/*
struct VertexToken{
	weak_ptr<Meshlet> token;
};

struct TextureToken{
	weak_ptr<MeshTexture> token;
};
*/

class TextureLayerGPU{
public:

	TextureLayerGPU(){}

	TextureLayerGPU(TextureLayerGPU &&o) :
			coords(move(o.coords)),
			tex(move(o.tex)),
			version(move(o.version)),
			token(move(o.token)){

	}


	~TextureLayerGPU(){}

	shared_ptr<TexCoordBufConnector> coords;
	shared_ptr<TexAtlasPatch> tex;
	int version;
	unique_ptr<weak_ptr<MeshTexture>> token;

	//TODO:this might also need a version of traingles...
	//also the triangle version of its neighbours

	GpuTextureInfo genGpuTextureInfo();

};


class MeshletGPU{
public:
	MeshletGPU(){}
	//MeshletGPU(GpuStorage* gpu_storage);
	//copy constructor
	MeshletGPU(MeshletGPU &o);
	MeshletGPU(MeshletGPU &&o) :
				id(o.id),
				triangles(move(o.triangles)),
				triangle_version(o.triangle_version),
				vertices(move(o.vertices)),
				vertex_version(o.vertex_version),
				vertex_token(move(o.vertex_token)),
				textures(move(o.textures)),
				std_tex(move(o.std_tex)),
				gpu_vert_transcribe_task_count(move(o.gpu_vert_transcribe_task_count)),
				gpu_vert_transcribe_tasks(move(o.gpu_vert_transcribe_tasks)),
				gpu_neighbour_start_ind(move(o.gpu_neighbour_start_ind)),
				geom_lookup_tex(move(o.geom_lookup_tex))
				{

	}

	//destructor
	//given a vertex token the data is downloaded and put back to the according meshlet
	~MeshletGPU();

	int id = -1;
	shared_ptr<TriangleBufConnector> triangles;
	int triangle_version = -1;

	shared_ptr<VertexBufConnector> vertices;
	int vertex_version = -1;
	unique_ptr<weak_ptr<Meshlet>> vertex_token;


	//TODO: triangle version of neighbours might be needed
	vector<shared_ptr<TextureLayerGPU>> textures;
	TextureLayerGPU std_tex;

	//TODO: add all the mechanisms necessary to ensure the consistency between these tasks and the result
	//the geometry lookup part is tricky! (thanks past simon! this is really helping!)
	//these are some scatter commands.
	//these commands stay valid as long as the neighbouring triangles/vertices do not change
	//(there should be an actual mechanism checking this)
	// (no added or removed triangles)
	struct TranscribeBorderVertTask{
		int ind_local; //index of vertex in this meshlet
		int ind_in_neighbour; //index of the "same" vertex in the neighbouring meshlet
		int ind_neighbour;//references to gpu_neighbour_start_ind
	};
	int gpu_vert_transcribe_task_count = -1;
	TranscribeBorderVertTask *gpu_vert_transcribe_tasks = nullptr;

	//the starting indices for the triangle buffer. They have to be updated every frame.
	int *gpu_neighbour_start_ind = nullptr;


	shared_ptr<TexAtlasPatch> geom_lookup_tex = nullptr;





};
/*
class VerticesGPU{
public:
	int version;
	VertexBufConnector vertices;
	unique_ptr<weak_ptr<Meshlet>> token;
};*/
#endif //SCALABLE_FUSION_GPU_MESH_STRUCTURE_H
