#ifndef FILE_ACTIVE_SET_H
#define FILE_ACTIVE_SET_H

#include <memory>
#include <vector>
//#include <thread>

#include <cuda/coalesced_memory_transfer.h>
#include <cuda/geom_update.h>
#include "tex_atlas.h"
#include "gpu_mesh_structure.h"

using namespace std;

class InformationRenderer;
class PresentationRenderer;

class MeshReconstruction;
class Meshlet;
class Vertex;
class Triangle;
class MeshletGpuHandle;
class MeshTextureGpuHandle;
class GpuStorage;
class LowDetailRenderer;
class TextureUpdater;
class GpuTriangle;

template <typename T>
class GpuBufferConnector;
typedef GpuBufferConnector<GpuTriangle> TriangleBufConnector;

class ActiveSet {
	friend GpuStorage;
	friend InformationRenderer;
	friend PresentationRenderer;
	friend Meshlet;

public:
    static std::vector<ActiveSet*> debug_all_active_sets;
	static std::mutex mutex;
	~ActiveSet();

	string name;
	/*
	GpuStorage *gpu_geom_storage;

	vector<shared_ptr<Meshlet>> retained_mesh_patches_cpu;

	vector<shared_ptr<MeshletGpuHandle>> retained_mesh_patches;

	vector<shared_ptr<TriangleBufConnector>> retained_double_stitches;
	vector<shared_ptr<TriangleBufConnector>> retained_triple_stitches;//TODO: implement this (almost just for ref textures)
	shared_ptr<TriangleBufConnector> retained_triple_stitches_coalesced;
	*/
	//TODO: is it better retaining it here compared to retaining it in the actual gpumesh structure?
	//vector<shared_ptr<MeshTextureGpuHandle>> retainedMeshTextureGpuHandles;


	/*
	ActiveSet(GpuStorage *storage, vector<shared_ptr<Meshlet>> patches,
			  MeshReconstruction *map,
			  LowDetailRenderer* low_detail_renderer,
			  TextureUpdater* texture_updater,
			  InformationRenderer* information_renderer,
			  bool initial,//TODO: also get rid of these initial frames
	          bool debug1 = false);
	*/

	//initial setup of active set (TODO: how to handle textures and resources that do not exist upfront?)
	/*
	ActiveSet(	GpuStorage *storage,
				vector<shared_ptr<Meshlet>> patches);
	*/

	//setup of active set while also retaining data from existing active sets
	ActiveSet(	GpuStorage *storage,
				vector<shared_ptr<Meshlet>> meshlets_requested,
				vector<shared_ptr<ActiveSet>> active_sets,
				vector<bool> allocate_new_verts = {},
				bool debug_ignore_missing_geom_tex = false);




	void setupHeaders(bool debug = false);
	void setupTranscribeStitchesTasks(vector<shared_ptr<Meshlet>> &	meshlets_requested);
	GLuint getHeaderBuffer();


	//gpu::GeometryUpdate::TranscribeStitchTask* gpu_transcribe_tasks = nullptr;
	//size_t gpu_transcribe_task_count = 0;
	vector<MeshletGPU> meshlets;
	shared_ptr<PatchInfoBufConnector> headers;



	//vector<gpu::GeometryUpdate::TranscribeStitchTask> transcribe_tasks;
	//the key is the patch id (maybe share patch id with stitch ids)
	//value is the index in the according vectors
	unordered_map<int,int> meshlet_inds; // to p
	//separate textures


	MeshletGPU* getGpuMeshlet(shared_ptr<Meshlet> meshlet);
    MeshletGPU* getGpuMeshlet(const Meshlet* meshlet);

	void uploadTexAndCoords_(vector<shared_ptr<Meshlet>> &patches,
	                         vector<shared_ptr<MeshletGpuHandle>> &patches_gpu,
	                         const MeshReconstruction* map, bool initial = false);


	void checkAndUpdateRefTextures_(const vector<shared_ptr<Meshlet>> &patches,
									MeshReconstruction *reconstruction,
									TextureUpdater *texture_updater,
									InformationRenderer* information_renderer);



	void upload(shared_ptr<VertexBufConnector> &buf, vector<Vertex> &vertices);
	void upload(shared_ptr<TriangleBufConnector> &buf, vector<Triangle> &triangles);

	void uploadGeometry(GpuStorage *storage, MeshletGPU &meshlet_gpu, Meshlet* meshlet);

	bool containsNeighbours(shared_ptr<Meshlet> meshlet);


	bool hasAllGeometry();

	void assertAllGeometry();


};

#endif // FILE_ACTIVE_SET_H
