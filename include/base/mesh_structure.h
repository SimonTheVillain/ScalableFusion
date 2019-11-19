#ifndef FILE_MESH_STRUCTURE_H
#define FILE_MESH_STRUCTURE_H

#include <vector>
#include <mutex>
#include <iostream>
#include <set>

#include <eigen3/Eigen/Core> ///TODO: Change this!!!!! this is baaaad!!!

#include "texture_structure.h"
#include "stack_vector.h"
#include <gpu/gpu_storage.h>
#include <utils/octree.h>
#include <rendering/low_detail_renderer.h>

using namespace std;
using namespace Eigen;

/**
 * REMEMBER: This structure has to be done in a way that even when we remove one point, the
 * remaining structure should not autodelete. I don't know what the mechanism of deleting
 * points should be....????
 * The use of shared_ptr is costly. especially for points that might be accessed often like
 * the most primitive elements. (points, triangles, edges)
 * THIS IS CRITICAL!!!!! It would increase the performance a lot if we reduce the use of smart pointers!!!
 */


class TexAtlas;

class MeshReconstruction;
class MeshTextureGpuHandle;

struct Triangle;
struct Edge;

class DeformationNode;

/**
 * @brief The GeometryBase class
 * This is supposed to be a base class to stitches, patches and so on.
 * Because of heritage the access to the overloaded elements might be slow. (vtable)
 * Therefore changing this to "curiously recurring template pattern" might be useful
 */
class GeometryBase {
public:

	//TODO: check if this is even necessary
	//TODO: maybe do not do this as virtual method to get higher performance
	/**
	 * @brief isGpuResidencyRequired
	 * @return
	 * this returns true if this structures residency on the gpu is required for rendering
	 */
	virtual bool isGpuResidencyRequired() = 0;

	virtual bool isPartOfActiveSet(const ActiveSet *set) = 0;

	//TODO: maybe do this with respect to the active set!
	virtual	shared_ptr<TriangleBufConnector> getMostCurrentGpuTriangles() = 0;

	void replacePatchInTriangleReferences(shared_ptr<MeshPatch> from,
	                                      shared_ptr<MeshPatch> to,
	                                      int vertex_offset);

	void setNewMainPatchForTriangles(shared_ptr<MeshPatch> main_patch);

	void deregisterTriangles();

	/**
	 * @brief edges
	 * a list of edges this geometry is part of. One edge can be part of multiple stitches/patches.
	 * TODO: whenever an edge is deleted, the according pointer should be deleted or invalidated
	 * (handling this was way easier with sharde/weak_ptrs)
	 * Edges could therefore either have a list of references to these pointers or:
	 * Edges could also have a affiliation to their triangle and we do not store this vector
	 */
	//vector<Edge*> edges;///tl;dr: this list of pointers is deprecated

	virtual weak_ptr<GeometryBase> getWeakBaseSelf() = 0;

	bool debugIsValid() {
		return (debug_hash == 1123);
	}

	int debug_hash = 1123;

	/**
	 * @brief triangles
	 * a list of references to triangles that are within this mesh or stitch.
	 * A triangle can only be part of one of these elements. (it is only referenced in one
	 * of these lists.)
	 * TODO: whenever a triangle is deleted the according pointer should be deleted or invalidated
	 * (handling this was way easier with shared/weak_ptrs)
	 * Triangles should have a reference back to this vector (and their position within it).
	 * TODO: think about the triangle as a pointer. is it really used by multiple elements and does it really have to be a pointer
	 */
	vector<Triangle> triangles;

	int triangles_version = -1;

	/**
	 * @brief workInProgress
	 * a mutex to lock the geometry as soon as it is beeing worked on
	 */
	mutex work_in_progress;///TODO: Maybe rename this to cpuWorkInProgress. or meshingInProgress
};

/**
 * this is the class that handles how many
 * Patches, DoubleStitches or TripleStitches are used by a certain thread right now.
 */
class ActiveSet;

struct TriangleReference {

	TriangleReference() { }

	TriangleReference(GeometryBase *geometry_base, int ind)
			: container(geometry_base),
			  index(ind) { };

	bool valid() {
		return (container != nullptr && index != -1);
	}

	/** \deprecated */
	bool equalTo(const TriangleReference &other) {
		return (container == other.container && index == other.index);
	}

	bool operator==(const TriangleReference &other) {
		return (container == other.container && index == other.index);
	}

	bool operator!=(const TriangleReference &other) {
		return (!(*this == other));
	}

	Triangle *get() {
		assert(valid());
		return &container->triangles[index];
	};
	//should this be  a weak ptr?
	GeometryBase *container = nullptr;

	int index = -1;

};
class Vertex;
struct VertexReference {
	MeshPatch *patch_ = nullptr;

	int index_ = -1;
public:

	MeshPatch* getPatch() const {
		return patch_;
	}

	int getIndex() const {
		return index_;
	}

	// TODO: create a proper operator you fool
	/** \deprecated */
	bool isEqualTo(VertexReference &other) {
		return (patch_ == other.patch_ && index_ == other.index_);
	}

	bool operator==(const VertexReference &other) {
		return (patch_ == other.patch_ && index_ == other.index_);
	}

	bool operator<(const VertexReference &other) {
		if(patch_ < other.patch_) {
			return true;
		} else if(patch_ > other.patch_) {
			return false;
		} else {
			return index_ < other.index_;
		}
	}

	Vertex* get() const;/* {
		return &patch_->vertices[index_];
	};*/

	void set(MeshPatch *p, int i) {
		if(i < -1 || i > 10000) {
			assert(0); // Something again is very wrong DEBUG
		}
		if(p == 0) {
			//assert(0);
		}
		index_ = i;
		patch_ = p;
	}
	VertexReference(){};

	//copy constructor
	VertexReference(const VertexReference &o):
			patch_(o.patch_),
			index_(o.index_){ }
	VertexReference& operator=(const VertexReference &o)
	{
		patch_ = o.patch_;
		index_ = o.index_;
		return *this;
	}
	//move constructor that invalidates the original reference
	VertexReference(VertexReference &&o){
		patch_ = o.patch_;
		index_ = o.index_;
		o.set(nullptr,-1);//invalidate the original reference
	}

};

// TODO: Maybe let this vertex structure also replace the gpuvertex
class Vertex {
public:

	struct VertexInTriangle {
		TriangleReference triangle;
		int ind_in_triangle = -1;
	};
private:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Vector4f p;
	Vector3f n; // TODO: start using this at some point

	int32_t tex_ind_in_main_patch;

	vector<VertexReference> mocked_by;
	VertexReference mocking;
	vector<VertexInTriangle> triangles;
public:

	void removeMockedBy(Vertex *vert){
		for(int i=0;i<mocked_by.size();i++){
			if(mocked_by[i].get() == vert){
				mocked_by[i] = mocked_by[mocked_by.size()-1];
				mocked_by.pop_back();
				return;
			}
		}
		assert(0);
	}


	//StackVector<VertexInTriangle, 16> triangles; //this will not really give any speed benefits (a few milliseconds at most)


	Vertex() 
			: tex_ind_in_main_patch(-1) { 
	}

	Vertex(	Vertex && o) :
			mocked_by(std::move(o.mocked_by)),
			mocking(std::move(mocking)),
			p(o.p),
			n(o.n),
			triangles(std::move(o.triangles)) {}

	Vertex(GpuVertex gpu_vertex) {
		p = gpu_vertex.p;
		n = gpu_vertex.n;
		tex_ind_in_main_patch = gpu_vertex.tex_ind_in_main_patch;
	}

	~Vertex(){
#ifdef DEBUG
		assert(isMocking() != mocked_by.size()>0);
#endif
		if(isMocking()){
			mocking.get()->removeMockedBy(this);
		}
		if(mocked_by.size()>0){
			mocked_by[0].get()->p = p;
			mocked_by[0].get()->n = n;
			mocked_by[0].get()->triangles = std::move(triangles);
			for(int i=1;i<mocked_by.size();i++){
				mocked_by[i].get()->mocking = mocked_by[0];
			}
		}
	}

	void removeTriangle(Triangle* triangle) {
		for(size_t i = 0; i < triangles.size(); i++) {
			if(triangles[i].triangle.get() == triangle) {
				triangles[i] = triangles.back();
				triangles.pop_back();
				return;
			}
		}
		// Ending up here means that the triangle probably was not properly registered
		assert(0);
	}

	void removeTriangle(int pos) {
		triangles[pos] = triangles.back();
		triangles.pop_back();
	}

	bool isMocking(){
		return mocking.getPatch() != nullptr;
	}

	Vector4f getP(){
		if(isMocking()){
			return mocking.get()->getP();
		}
		return p;
	}
	Vector3f getN(){
		if(isMocking()){
			return mocking.get()->getN();
		}
		return n;
	}

	void setP(Vector4f p){
		if(isMocking()){
			mocking.get()->p = p;
			return;
		}
		this->p = p;
	}
	void setN(){
		if(isMocking()){
			mocking.get()->n = n;
			return;
		}
		this->n = n;
	}

	//TODO: make the constructor such that this method doesn't need to be called
	GpuVertex genGpuVertex() {
		if(isMocking()){
			return mocking.get()->genGpuVertex();
		}
		GpuVertex vert;
		vert.p = p;
		vert.n = n;
		vert.tex_ind_in_main_patch = static_cast<int16_t>(tex_ind_in_main_patch);
		return vert;
	}

	/**
	 * Checks if the vertex is completely encompassed by triangles.
	 * @return
	 */
	bool encompassed();






};

class MeshPatchGpuHandle {
public:

	MeshPatchGpuHandle(GpuStorage* gpu_geom_storage, int nr_vertices,
					   int nr_triangles);

	~MeshPatchGpuHandle();

	bool setLabelTex(shared_ptr<MeshTextureGpuHandle> tex);

	bool addWeightedLabelTex(shared_ptr<MeshTextureGpuHandle> tex);

	// TODO: we have to find a way to submit this to the according cleanup thread
	// transmitting the verticesSource to the recycler
	// the recycler should also modified to handle multiple threads all of them
	// working a queue of vertices
	void swapSrcDst() {
		swap(vertices_source, vertices_dest);
	}

	mutex mesh_patch_gpu_mutex;

	shared_ptr<PatchInfoBufConnector> patch_infos;
	shared_ptr<VertexBufConnector>    vertices_source;
	shared_ptr<VertexBufConnector>    vertices_dest;
	shared_ptr<TriangleBufConnector>  triangles;

	bool gpu_vertices_changed;

	weak_ptr<MeshPatch> download_to_when_finished;

	//TODO: think about what to do on the texture side of things
	//the plan is that this mesh patch gpu handle keeps grip of
	//the patch texture handles
	shared_ptr<MeshTextureGpuHandle> geom_tex;

	shared_ptr<MeshTextureGpuHandle> texs[GPU_MAX_TEX_PER_PATCH];

	shared_ptr<MeshTextureGpuHandle> label_tex;
	bool label_tex_valid;

	shared_ptr<MeshTextureGpuHandle> weighted_label_texs[3];
	size_t weighted_label_tex_count;

	//todo replace this with 0 and keep it up to date
	const size_t tex_count = GPU_MAX_TEX_PER_PATCH;

	// TODO: think about how to use the triple stitches
	// there is a solution... look at the implementation ( which is not done yet)
};

/**
 * @brief The MeshPatch class
 * Stores a patch, all of its points, its connections to other patches,
 * to textures and if/where the data is stored
 * TODO: as soon as we use some raw pointers we have to create a copy constructor
 * this is due to the way the (simon does not know anymore)
 * Very big TODO: a patch will need a system to be worked on and being rendered from
 * different threads.
 */
class MeshPatch : public GeometryBase,
                  public OctreeMember<MeshPatch>, //The name of this pattern is CRTP https://stackoverflow.com/questions/4030224/whats-the-use-of-the-derived-class-as-a-template-parameter
                  public LowDetailPoint {
public:

	MeshPatch(Octree<MeshPatch> *octree);

	~MeshPatch();

	weak_ptr<GeometryBase> getWeakBaseSelf() {
		return static_pointer_cast<GeometryBase>(weak_self.lock());
	}

	//iterates over all the geometry and updates the octree accordingly
	void updateCenterPoint();
	void updateSphereRadius();
	void updatePrincipalPlaneAndCenter();

	//TODO: add the functionality to set the according flags for the gpu synchronization
	void addTexPatch(shared_ptr<MeshTexture> tex_patch);
	void removeTexPatch(shared_ptr<MeshTexture> tex_patch);
	void removeTexPatches(vector<shared_ptr<MeshTexture>> tex_patches);


	/*
	int addActiveSet(ActiveSet* active_set);
	int removeActiveSet(ActiveSet *active_set);

	shared_ptr<DoubleStitch> getDoubleStitchWith(MeshPatch *other_patch);
	void addStitchReference(shared_ptr<DoubleStitch> stitch);
	void removeStitchReference(shared_ptr<DoubleStitch> stitch);


	shared_ptr<TripleStitch> getTripleStitchWith(MeshPatch *other_patch_1, 
	                                             MeshPatch *other_patch_2);
	void addStitchReference(shared_ptr<TripleStitch> stitch);
	void removeStitchReference(shared_ptr<TripleStitch> stitch);

	set<shared_ptr<MeshPatch>> getNeighbours();

	shared_ptr<DoubleStitch> getDoubleStitchTo(shared_ptr<MeshPatch> patch);
	shared_ptr<TripleStitch> getTripleStitchTo(shared_ptr<MeshPatch> patch);

	bool isGeometryFullyAllocated();
	 */

	mutex neighbour_mutex;
	vector<weak_ptr<MeshPatch>> neighbours;
	void addNeighbour(weak_ptr<MeshPatch> nb){
		neighbour_mutex.lock();
		neighbours.push_back(nb);
		neighbour_mutex.unlock();
	}
	void removeNeighbour(MeshPatch *neighbour);


	/**
	 * @brief cleanupStitches
	 * When, after a merge of patches, two or more different stitches stitch the same patches
	 * we combine these stitches. TODO: check if this is right
	 */
	void cleanupStitches();


	shared_ptr<DeformationNode> deformation_node;

	bool label_tex_patch_used = false;

	//TODO: follow the example of the standard labelling right above
	//(but do it for semantic labels)
	//http://www.boost.org/doc/libs/1_55_0/doc/html/lockfree.html (could also be an solution)	
	//TODO: this
	mutex label_tex_patch_mutex;
	shared_ptr<MeshTexture> label_tex_patch;

	mutex sem_label_tex_patch_mutex;
	vector<shared_ptr<MeshTexture>> sem_label_tex_patches;

	mutex double_stitch_mutex;
	vector<shared_ptr<DoubleStitch>> double_stitches;

	mutex triple_stitch_mutex;
	vector<shared_ptr<TripleStitch>> triple_stitches;

	/**
	* @brief geom_tex_patch
	* This texture patch stores the information of standard deviation and minimum observed distance
	* it is supposed to be updated every frame therefore we want to utilize 2 textures 1 is texture the
	* other one is the render target. This might be paralellizable. (hopefully)
	* Solution for updating standard deviation:
	* https://math.stackexchange.com/questions/198336/how-to-calculate-standard-deviation-with-streaming-inputs
	*/

	/*
	* TODO 4: let the texutures and the FBOs (VAOs? but i don't think) only be weak_ptr by
	* the patches. The active set then has the correct references and triggers the deletion if necessary.
	* For the FBOs the destruction of the ActiveSet puts each FBO to the "destruction lists"
	* of its according thread. These lists should be treated in regular intervals.
	* (maybe even put the fbos onto the shelves and reuse them)
	*/
	mutex geom_tex_patch_mutex;
	shared_ptr<MeshTexture> geom_tex_patch;



	/**
	 * @brief tex_patches
	 * TODO: right now the only real texture is in texPatches[0]
	 * In future i want one texturepatch to contain multiple layers (why then separating this from a MeshPatch?)
	 * One layer should be(unique name) the variance and median of the geometry
	 */
	mutex tex_patches_mutex;
	vector<shared_ptr<MeshTexture>> tex_patches;// we are really doing this without a shared ptr?

	mutex vertices_mutex;
	/**
	 * @brief vertices
	 * This is the storage for the geometry itself.
	 * (almost ready to be loaded to gpu)
	 */
	 int vertices_version = -1;
	vector<Vertex> vertices;

	/**
	 * @brief id
	 * an unique identifyer for these patches
	 */
	int id = -1;

	bool debug = false;
	int debug1 = 0;

	weak_ptr<MeshPatch> weak_self;

	/**
	 * @brief center
	 * The center of this patch and a bounding sphere.
	 * TODO: not set yet
	 */
	//Vector4f center;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Vector4f principal_plane;


};




/**
 * @brief The Triangle struct
 * stores references to 3 geometrical points and 3 texture coordinates.
 * It also stores pointers to up to three edges
 * TODO: maybe instead of having edges the triangles should store their up to 3 neighbours directly
 * Edges are only then necessary when it comes to stitching, therefore they should only be generated when needed.
 */
struct Triangle {



	struct EdgeReference {

		bool valid() {
			return (border_ind != -1 && ind != -1);
		}

		void invalidate() {
			border_ind = -1;
			ind        = -1;
		}

		Edge* get(vector<vector<Edge>> &list) {
			assert(valid());
			return &list[border_ind][ind];
		}

		int border_ind = -1;
		int ind        = -1;
	};

	struct Neighbour {

		bool valid() {
			return ref.valid();
		}

		void invalidate() {
			pos = -1;
			ref = TriangleReference();
		}

		//position in neighbour
		int pos = -1;
		TriangleReference ref;
		bool debug = false;
	};

	bool registered = false;
	bool debug_is_stitch = false;
	int  debug_nr = -1;

	/**
	 * These are the indices to the texture coordinates to the texture defined by the patch used in the first point
	 */
	uint32_t tex_indices[3] = {0, 0, 0};

	EdgeReference   edges[3];
	//VertexReference points[3];
	int 			points[3] = {-1, -1, -1};
	Neighbour       neighbours[3];

	Triangle() { }

	~Triangle() {
	}

	//TODO: fix and reinsert these (used a lot in meshing and stitching)
	/*
	bool containsPoint(VertexReference& p) {
		for(uint8_t i = 0; i < 3; i++) {
			if(p.isEqualTo(points[i])) {
				return true;
			}
		}
		return false;
	}

	int getPointIndex(VertexReference& p) {
		for(uint8_t i = 0; i < 3; i++) {
			if(p.isEqualTo(points[i])) {
				return i;
			}
		}
		return -1;
	}

	bool addNeighbour(TriangleReference triangle, VertexReference p1,
	                  VertexReference p2, int pos) { //position within the target triangle
		if(((p1 == points[0]) && (p2 == points[1])) ||
		   ((p1 == points[1]) && (p2 == points[0]))) {
			if(neighbours[0].valid()) {
				assert(0);//already has neighbour... adding a new one is forbidden
				//return false;
			}
			neighbours[0].ref = triangle;
			neighbours[0].pos = pos;

		} else if(((p1 == points[1]) && (p2 == points[2])) ||
		          ((p1 == points[2]) && (p2 == points[1]))) {
			if(neighbours[1].valid()) {
				assert(0);
			}
			neighbours[1].ref = triangle;
			neighbours[1].pos = pos;

		} else if(((p1 == points[0]) && (p2 == points[2])) ||
		          ((p1 == points[2]) && (p2 == points[0]))) {
			if(neighbours[2].valid()) {
				assert(0);
			}
			neighbours[2].ref = triangle;
			neighbours[2].pos = pos;

		} else {
			//none of the edges is fitting to add a
			//triangle as neighbour.... (the points don't match)
			//Calling this method should never occur?
			assert(0);
		}
		return true;
	}

	int getPosInTriangle(VertexReference p1, VertexReference p2) {
		if(((p1 == points[0]) && (p2 == points[1])) ||
		   ((p1 == points[1]) && (p2 == points[0]))) {
			return 0;
		}
		if(((p1 == points[2]) && (p2 == points[1])) ||
		   ((p1 == points[1]) && (p2 == points[2]))) {
			return 1;
		}
		if(((p1 == points[0]) && (p2 == points[2])) ||
		   ((p1 == points[2]) && (p2 == points[0]))) {
			return 2;
		}
		assert(0);
		return -1;
	}
	*/

	/**
	 * @brief registerToPoints
	 * Registers a freshly created triangle to its neighbouring points (so the point variables have to be set already
	 * @param triangleToRegister
	 */
	 //TODO: remove this debug
	 /*
	static void registerTriangle(TriangleReference &triangle_to_register, 
	                             bool search_edges = true, bool debug = false);
	*/
	void cycle(int cnt) {
		assert(!registered);
		assert(!neighbours[0].valid());
		assert(!neighbours[1].valid());
		assert(!neighbours[2].valid());
		if(cnt == 1) {
			int p = points[0];
			points[0] = points[1];
			points[1] = points[2];
			points[2] = p;
		} else if(cnt == 2) {
			int p = points[0];
			points[0] = points[2];
			points[2] = points[1];
			points[1] = p;

		}
	}

	//TODO: reinsert these (used a lot in meshing and stitching)
	/*
	void replacePatchReference(MeshPatch *from, MeshPatch *to, int offset);

	void setNewMainPatch(MeshPatch *patch);

	VertexReference getThirdPoint(VertexReference p1, VertexReference p2);
	*/


	//TODO: find out if this needs a self reference
	/*
	TriangleReference selfReference;
	TriangleReference &ref(){
		assert(selfReference.valid());
		return selfReference;
	}*/
};

struct Stitch : public GeometryBase {
	//TODO: if that stitch stays empty remove the class
	/*
	shared_ptr<TriangleBufConnector> getMostCurrentGpuTriangles() {
		return triangles_gpu.lock();
	}*/

	//TODO: this could also just be a weak ptr to the buffer itself
	//weak_ptr<TriangleBufConnector> triangles_gpu;
};

/**
 * @brief The DoubleStitch struct
 * Geometry to stitch only two geometry texments together
 */
 /*
struct DoubleStitch : public Stitch {

	DoubleStitch();

	~DoubleStitch();

	bool isEqualTo(int id1, int id2);

	weak_ptr<GeometryBase> getWeakBaseSelf() {
		return static_pointer_cast<GeometryBase>(weak_self.lock());
	}

	// Remove self from all the patches
	void removeFromPatches(shared_ptr<MeshPatch> except_for_patch = nullptr);

	shared_ptr<MeshPatch> getOtherPatch(shared_ptr<MeshPatch> const &patch);

	bool isGpuResidencyRequired();

	bool isPartOfActiveSet(const ActiveSet* active_set);

	void replacePatchReference(shared_ptr<MeshPatch> from,
	                           shared_ptr<MeshPatch> to);
	bool isDegenerated();

	bool isConnectingPatch(shared_ptr<MeshPatch> patch);

	bool connectsSamePatches(shared_ptr<DoubleStitch> other);

	weak_ptr<DoubleStitch> weak_self;

	//the two patches the double stitch is connecting.
	//the main patch(the one whose texture we are using) is patch[0]
	weak_ptr<MeshPatch> patches[2];

	bool debug_to_delete1 = false;
	bool debug_to_delete2 = false;
	int  debug_to_delete3 = 0;
};
  */

/**
 * @brief The TripleStitch struct
 * A structure that stitches triplets of patches together whenever they touch.
 * TO THINK: Due to a triple stitch most commonly only consisting out of one triangle we want this
 * structure to be even sleeker. Maybe for rendering we should create one buffer for only triple stitches
 * There may even be rules like to not to consider three patches beeing connected when it is only done by
 * a triple stitch.
 * Of course a triple stitch should be removed if one of the patches or points gets deleted.
 */
 /*
struct TripleStitch: public Stitch {

	TripleStitch();

	~TripleStitch();

	bool isGpuResidencyRequired();

	bool isPartOfActiveSet(const ActiveSet *active_set);

	bool isEqualTo(int id1, int id2, int id3);

	weak_ptr<GeometryBase> getWeakBaseSelf() {
		return static_pointer_cast<GeometryBase>(weak_self.lock());
	}

	//remove self from all the patches
	void removeFromPatches(shared_ptr<MeshPatch> except_for_patch = nullptr);

	shared_ptr<MeshPatch> getAnyOtherPatch(shared_ptr<MeshPatch> patch);
	//
	bool getOtherPatches(shared_ptr<MeshPatch> patch, 
	                     shared_ptr<MeshPatch> patches_out[2]);


	void replacePatchReference(shared_ptr<MeshPatch> from,
	                           shared_ptr<MeshPatch> to,
	                           int offset);

	bool isCovertDoubleStitch();

	bool isConnectingPatch(shared_ptr<MeshPatch> patch);

	///TODO: check if this weak self is needed
	weak_ptr<TripleStitch> weak_self;

	///TODO (not implemented yet)
	//int positionWithinSlot=-1;//since a triple stitch will most certainly only contain one triangle it is necessary to have
	weak_ptr<MeshPatch> patches[3];
};
  */

/**
 * @brief The Edge struct
 * Stores the connection between two points at a border (therefore there is only one triangle
 */
struct Edge {

	Edge() { }

	Edge(TriangleReference &tri, int index_in_triangle) {
		triangle = tri;
		pos = index_in_triangle;
	}

	void registerInTriangle(int border_ind, int ind) {
		is_registered = true;
		triangle.get()->edges[pos].border_ind = border_ind;
		triangle.get()->edges[pos].ind = ind;
	}

	void unregister() {
		assert(is_registered);
		triangle.get()->edges[pos].invalidate();//invalidate the reference to this edge
		is_registered=false;
	}

	//borderlist is used in case a edge is already registered in the triangle
	bool getOtherEdge(int endpoint, Edge &result, 
	                  vector<vector<Edge>> &border_list) {

		Triangle* current_triangle = triangle.get();
		TriangleReference current_triangle_ref = triangle;
		int current_edge_ind_in_triangle = pos;

		int ttl = 10000;//TODO: put that back to 100.... we will not have than 16 or so triangles for a point
		while(ttl--) {

			//go to the next edge on this triangle:
			current_edge_ind_in_triangle += ((endpoint * 2) - 1);
			if(current_edge_ind_in_triangle == -1) {
				current_edge_ind_in_triangle = 2;
			}
			if(current_edge_ind_in_triangle == 3) {
				current_edge_ind_in_triangle = 0;
			}
			//if the current edge is free we will return it
			if(!current_triangle->neighbours[current_edge_ind_in_triangle].valid()) {
				if(current_triangle->edges[current_edge_ind_in_triangle].valid()) {
					result = *current_triangle->edges[current_edge_ind_in_triangle].get(border_list);
					assert(result.triangle.valid());
					return true;
				}

				Edge res;
				res.triangle = current_triangle_ref;
				res.pos = current_edge_ind_in_triangle;
				result = res;
				return true;
			}

			if(current_edge_ind_in_triangle > 2 || current_edge_ind_in_triangle < 0) {//debug efforts
				assert(0);
			}
			if(current_triangle_ref.index < 0 || current_triangle_ref.index > 10000) {
				cout << current_triangle_ref.container->triangles.size() << endl;
				assert(0);
			}
			// Get to the next triangle
			current_triangle_ref =
					current_triangle->neighbours[current_edge_ind_in_triangle].ref;
			if(!current_triangle_ref.container->debugIsValid()) {
				getOtherEdge(endpoint, result, border_list); //debug.... this should not happen
				assert(0);
			}
			if(current_triangle_ref.index < 0 || current_triangle_ref.index > 10000) {
				getOtherEdge(endpoint, result, border_list);//debug.... this should not happen
				cout << current_triangle_ref.container->triangles.size() << endl;
				assert(0);
			}
			current_edge_ind_in_triangle =
					current_triangle->neighbours[current_edge_ind_in_triangle].pos;
			if(current_edge_ind_in_triangle > 2 || current_edge_ind_in_triangle < 0) {//debug efforts
				assert(0); //(simon) current triangle seems off
			}

			current_triangle = current_triangle_ref.get();
		}

		getOtherEdge(endpoint, result, border_list);//debug... this should never happen
		assert(0);//more than 100 triangles on this node means something is off!!!
		return false;
	}

	bool isOpen() {
		Triangle *tri = triangle.get();
		return !tri->neighbours[pos].valid();
	}

	bool getOtherEdge(int endpoint, Edge &result) {
		assert(0);
		return false;
	}

	//TODO:fix and reinsert (used in stitching and meshing)
	/*
	VertexReference points(int i) {
		assert(i == 0 || i == 1);
		int ind = pos + i;
		if(ind == 3) {
			ind = 0;
		}
		return triangle.get()->points[ind];
	}

	VertexReference oppositePoint() {
		int ind = pos - 1;
		if(ind == -1) {
			ind = 2;
		}
		return triangle.get()->points[ind];
	}
	*/

	/** \deprecated */
	bool equalTo(const Edge &other) {
		return ((triangle == other.triangle) && (pos == other.pos));
	}

	bool operator==(const Edge &other) {
		return ((triangle == other.triangle) && (pos == other.pos));
	}

	// TODO: i don't think this is necessary
	// When already used for stitching we do not do it from there
	bool already_used_for_stitch = false;

	TriangleReference triangle;
	int pos;//edge index within this triangle

	bool is_registered=false;

	int debug=-1;

	//this is weird

	bool outermost_connections_made[2] = {false, false};
	VertexReference outermost_connections[2];
	Vector2f outermost_connection_fragment_pos[2];
};

#endif // FILE_MESH_STRUCTURE_H