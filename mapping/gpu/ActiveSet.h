#ifndef FILE_ACTIVE_SET_H
#define FILE_ACTIVE_SET_H

#include <memory>
#include <vector>

#include <coalescedMemoryTransfer.h>

using namespace std;

class MapInformationRenderer;
class MapPresentationRenderer;

class MeshReconstruction;
class MeshPatch;
class MeshPatchGpuHandle;
class MeshTextureGpuHandle;
class GpuGeomStorage;

class GpuTriangle;
template <typename T>
class GpuBufferConnector;
typedef GpuBufferConnector<GpuTriangle> TriangleBufConnector;

class ActiveSet {
	friend GpuGeomStorage;
	friend MapInformationRenderer;
	friend MapPresentationRenderer;
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

	GpuGeomStorage *gpu_geom_storage;

	vector<shared_ptr<MeshPatch>> retained_mesh_patches_cpu;

	vector<shared_ptr<MeshPatchGpuHandle>> retained_mesh_patches;

	vector<shared_ptr<TriangleBufConnector>> retained_double_stitches;
	vector<shared_ptr<TriangleBufConnector>> retained_triple_stitches;//TODO: implement this (almost just for ref textures)
	shared_ptr<TriangleBufConnector> retained_triple_stitches_coalesced;

	//TODO: is it better retaining it here compared to retaining it in the actual gpumesh structure?
	//vector<shared_ptr<MeshTextureGpuHandle>> retainedMeshTextureGpuHandles;

private:

	ActiveSet(GpuGeomStorage *storage, vector<shared_ptr<MeshPatch>> patches,
	          MeshReconstruction *map,
	          bool initial,//TODO: also get rid of these initial frames
	          bool debug1 = false);

	//TODO: these:
	void uploadTexAndCoords_(vector<shared_ptr<MeshPatch>> &patches,
	                         vector<shared_ptr<MeshPatchGpuHandle>> &patches_gpu,
	                         const MeshReconstruction* map, bool initial = false);

	//void UploadTex

	void uploadTexAndCoords_(
			MeshPatch *patch, MeshPatchGpuHandle *patch_gpu, //lets check if these are necessary
			vector<CoalescedGpuTransfer::Task> &coalesced_tex_coord_tasks);

	//TODO: these, but this seems not to be elegant
	void checkAndAppendTriangles_(
			const vector<shared_ptr<MeshPatch>> &patches_to_check,
			vector<shared_ptr<MeshPatch>> &append_to);
	void uploadTriangles_(vector<shared_ptr<MeshPatch>> &patches);
	//TODO: these
	void checkAndUpdateRefTextures_(const vector<shared_ptr<MeshPatch>> &patches,
	                                MeshReconstruction *map);

	//TODO: propably the same for download
};

#endif
