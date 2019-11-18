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


class MeshPatch;
class MeshTexture;

/*
struct VertexToken{
	weak_ptr<MeshPatch> token;
};

struct TextureToken{
	weak_ptr<MeshTexture> token;
};
*/

class TextureLayerGPU{
public:
	TexCoordBufConnector coords;
	shared_ptr<TexAtlasPatch> tex;
	int version;
	unique_ptr<weak_ptr<MeshTexture>> token;

	//TODO:this might also need a version of traingles...
	//also the triangle version of its neighbours

};


class TexturedMeshGPU{
public:
	TriangleBufConnector triangles;
	int triangle_version;

	VertexBufConnector vertices;
	int vertex_version;
	unique_ptr<weak_ptr<MeshPatch>> vertex_token;


	//TODO: triangle version of neighbours might be needed
	vector<shared_ptr<TextureLayerGPU>> textures;
	TextureLayerGPU std_tex;


	//the geometry lookup part is tricky!
	struct TranscribeBorderVertTask{
		int ind_local;
		int ind_in_neighbour;
		int ind_neighbour;//references to gpu_neighbour_start_ind
	};
	int gpu_vert_transcribe_task_count;
	TranscribeBorderVertTask *gpu_vert_transcribe_tasks;
	int *gpu_neighbour_start_ind;
	shared_ptr<TexAtlasPatch> geom_lookup_tex;

};
/*
class VerticesGPU{
public:
	int version;
	VertexBufConnector vertices;
	unique_ptr<weak_ptr<MeshPatch>> token;
};*/
#endif //SCALABLE_FUSION_GPU_MESH_STRUCTURE_H
