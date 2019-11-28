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

	TextureLayerGPU();


	~TextureLayerGPU();

	TexCoordBufConnector coords;
	shared_ptr<TexAtlasPatch> tex;
	int version;
	unique_ptr<weak_ptr<MeshTexture>> token;

	//TODO:this might also need a version of traingles...
	//also the triangle version of its neighbours

};


class MeshletGPU{
public:

	MeshletGPU(GpuStorage* gpu_storage);
	//copy constructor
	MeshletGPU(MeshletGPU &o);

	//destructor
	//given a vertex token the data is downloaded and put back to the according meshlet
	~MeshletGPU();

	TriangleBufConnector triangles;
	int triangle_version;

	VertexBufConnector vertices;
	int vertex_version;
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
	int gpu_vert_transcribe_task_count;
	TranscribeBorderVertTask *gpu_vert_transcribe_tasks;

	//the starting indices for the triangle buffer. They have to be updated every frame.
	int *gpu_neighbour_start_ind;


	shared_ptr<TexAtlasPatch> geom_lookup_tex;



};
/*
class VerticesGPU{
public:
	int version;
	VertexBufConnector vertices;
	unique_ptr<weak_ptr<Meshlet>> token;
};*/
#endif //SCALABLE_FUSION_GPU_MESH_STRUCTURE_H
