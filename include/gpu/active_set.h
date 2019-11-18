#ifndef FILE_ACTIVE_SET_H
#define FILE_ACTIVE_SET_H

#include <memory>
#include <vector>

#include <cuda/coalesced_memory_transfer.h>
#include "tex_atlas.h"
#include "gpu_mesh_structure.h"

using namespace std;

class InformationRenderer;
class PresentationRenderer;

class MeshReconstruction;
class MeshPatch;
class MeshPatchGpuHandle;
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
	friend MeshPatch;

public:

	~ActiveSet();

	void drawDoubleStitches();
	void drawTripleStitches();
	void drawPatches();
	void drawEverything();

	void reuploadHeaders();

	void checkForCompleteGeometry();

	string name;

	GpuStorage *gpu_geom_storage;

	vector<shared_ptr<MeshPatch>> retained_mesh_patches_cpu;

	vector<shared_ptr<MeshPatchGpuHandle>> retained_mesh_patches;

	vector<shared_ptr<TriangleBufConnector>> retained_double_stitches;
	vector<shared_ptr<TriangleBufConnector>> retained_triple_stitches;//TODO: implement this (almost just for ref textures)
	shared_ptr<TriangleBufConnector> retained_triple_stitches_coalesced;

	//TODO: is it better retaining it here compared to retaining it in the actual gpumesh structure?
	//vector<shared_ptr<MeshTextureGpuHandle>> retainedMeshTextureGpuHandles;
	GLuint glBufferHeader;
private:

	ActiveSet(GpuStorage *storage, vector<shared_ptr<MeshPatch>> patches,
			  MeshReconstruction *map,
			  LowDetailRenderer* low_detail_renderer,
			  TextureUpdater* texture_updater,
			  InformationRenderer* information_renderer,
			  bool initial,//TODO: also get rid of these initial frames
	          bool debug1 = false);


	//initial setup of active set (TODO: how to handle textures and resources that do not exist upfront?)
	/*
	ActiveSet(	GpuStorage *storage,
				vector<shared_ptr<MeshPatch>> patches);
	*/

	//setup of active set while also retaining data from existing active sets
	ActiveSet(	GpuStorage *storage,
				vector<shared_ptr<MeshPatch>> patches,
				vector<shared_ptr<ActiveSet>> activeSets,
				vector<bool> allocate_new_verts = {});


	void setupHeaders();
	GLuint getHeaderBuffer();




	vector<shared_ptr<TexturedMeshGPU>> patches;
	//the key is the patch id (maybe share patch id with stitch ids)
	//value is the index in the according vectors
	unordered_map<int,int> patchInds;
	unordered_map<int,int> verticesInds;
	//separate textures


	void uploadTexAndCoords_(vector<shared_ptr<MeshPatch>> &patches,
	                         vector<shared_ptr<MeshPatchGpuHandle>> &patches_gpu,
	                         const MeshReconstruction* map, bool initial = false);


	void checkAndUpdateRefTextures_(const vector<shared_ptr<MeshPatch>> &patches,
									MeshReconstruction *reconstruction,
									TextureUpdater *texture_updater,
									InformationRenderer* information_renderer);

};

#endif // FILE_ACTIVE_SET_H
