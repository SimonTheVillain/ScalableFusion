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


	void replacePatchInTriangleReferences(shared_ptr<Meshlet> from,
	                                      shared_ptr<Meshlet> to,
	                                      int vertex_offset);

	void setNewMainPatchForTriangles(shared_ptr<Meshlet> main_patch);

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

class Vertex;

// TODO: Maybe let this vertex structure also replace the gpuvertex
class Vertex {
public:

	struct VertexInTriangle {
		Triangle* triangle;
		int ind_in_triangle = -1;
		//TODO: move constructor to move the vertex and its new pointer
	};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Vector4f p;
	Vector3f n;

	Vector4f color;

	Meshlet* meshlet = nullptr;



	//int32_t tex_ind_in_main_patch = -1;

	vector<VertexInTriangle> triangles;
public:



	//StackVector<VertexInTriangle, 16> triangles; //this will not really give any speed benefits (a few milliseconds at most)


	Vertex() {
		//cout << "vertex constructor " << endl;
	}

	Vertex( const Vertex& o){
		p = o.p;
		n = o.n;
		meshlet = o.meshlet;
		color = o.color;
		//the other elements are better not copied since they are highly dependant on the
		//vertex retaining its position in memory
	}
	//move constructor
	Vertex(	Vertex && o) noexcept;

	Vertex(GpuVertex gpu_vertex) {
		p = gpu_vertex.p;
		n = gpu_vertex.n;
		meshlet = nullptr;

		color = gpu_vertex.color;
		//tex_ind_in_main_patch = gpu_vertex.tex_ind_in_main_patch;
	}

	~Vertex(){
		//well, this is a complicated questions: removing a vertex would also mean to delete a few triangles
		if(triangles.size()>0){
			assert(0);// at this stage we don't remove valid vertices
		}
		for(int i=0;i<triangles.size();i++){
			//at least remove the reference to this triangle
			//triangles[i].triangle->
		}
	}

	void removeTriangle(Triangle* triangle) {
		for(size_t i = 0; i < triangles.size(); i++) {
			if(triangles[i].triangle == triangle) {
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

	void replaceTriangle(Triangle* from, Triangle* to) {
		for(size_t i=0;i<triangles.size();i++){
			if(triangles[i].triangle == from){
				triangles[i].triangle = to;
				return;
			}
		}
	}
	void insertTriangle(Triangle* in,int ind_in_triangle){
		triangles.emplace_back();
		triangles.back().ind_in_triangle = ind_in_triangle;
		triangles.back().triangle = in;
		//VertexInTriangle vit;
		//vit.ind_in_triangle = indexInTriangle;
		//vit.triangle = in;
		//triangles.push_back(vit);
	}


	Vector4f getP(){

		return p;
	}
	Vector3f getN(){

		return n;
	}

	void setP(Vector4f p){

		this->p = p;
	}
	void setN(){
		this->n = n;
	}

	//TODO: make the constructor such that this method doesn't need to be called
	GpuVertex genGpuVertex() {
		GpuVertex vert;
		vert.p = p;
		vert.n = n;
		vert.color = color;
		//vert.tex_ind_in_main_patch = static_cast<int16_t>(tex_ind_in_main_patch);
		return vert;
	}

	/**
	 * Checks if the vertex is completely encompassed by triangles.
	 * @return
	 */
	bool encompassed();






};

class MeshletGpuHandle {
public:

	MeshletGpuHandle(GpuStorage* gpu_geom_storage, int nr_vertices,
					 int nr_triangles);

	~MeshletGpuHandle();

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

	weak_ptr<Meshlet> download_to_when_finished;

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
class Meshlet : public GeometryBase,
				public octree::Object,//OctreeMember<Meshlet>, //The name of this pattern is CRTP https://stackoverflow.com/questions/4030224/whats-the-use-of-the-derived-class-as-a-template-parameter
				public LowDetailPoint {
public:

	//TODO: get rid of octree in this constructor
	Meshlet(int id, octree::Octree* octree);//Octree<Meshlet> *octree);


	~Meshlet();

	weak_ptr<GeometryBase> getWeakBaseSelf() {
		return static_pointer_cast<GeometryBase>(weak_self.lock());
	}

	//TODO: make this obsolete with https://en.cppreference.com/w/cpp/memory/enable_shared_from_this
	weak_ptr<Meshlet> getWeakSelf() {
		return weak_self; //TODO: replace with shared_from_this
	}

	//iterates over all the geometry and updates the octree accordingly
	void updateCenterPoint();
	void updateSphereRadius();
	void updatePrincipalPlaneAndCenter();

	//TODO: add the functionality to set the according flags for the gpu synchronization
	void addTexPatch(shared_ptr<MeshTexture> tex_patch);
	void removeTexPatch(shared_ptr<MeshTexture> tex_patch);
	void removeTexPatches(vector<shared_ptr<MeshTexture>> tex_patches);

	mutex neighbour_mutex;
	vector<weak_ptr<Meshlet>> neighbours;
	void addNeighbour(weak_ptr<Meshlet> nb){
		neighbour_mutex.lock();
		neighbours.push_back(nb);
		neighbour_mutex.unlock();
	}
	void removeNeighbour(Meshlet *neighbour){
		neighbour_mutex.lock();
		for(int i=0;i<neighbours.size();i++){
			if(neighbours[i].lock().get() == neighbour){
				neighbours[i] = neighbours.back();
				neighbours.pop_back();
				break;
			}
		}
		neighbour_mutex.unlock();
	}

	bool isNeighbourWith(Meshlet* other){
		neighbour_mutex.lock();
		bool result = false;
		for(auto neighbour : neighbours){
			if(neighbour.lock().get() == other){
				result = true;
				break;
			}
		}
		neighbour_mutex.unlock();
		return result;
	}

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
	/*
	mutex label_tex_patch_mutex;
	shared_ptr<MeshTexture> label_tex_patch;

	mutex sem_label_tex_patch_mutex;
	vector<shared_ptr<MeshTexture>> sem_label_tex_patches;
	*/

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

	weak_ptr<Meshlet> weak_self;

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
			return ptr != nullptr;
		}

		void invalidate() {
			pos = -1;
			ptr = nullptr;
		}


		//position in neighbour
		int pos = -1;
		Triangle* ptr = nullptr;
		bool debug = false;

		//TODO: write proper move constructor
		/*
		Neighbour(Neighbour &&o){
			//this move constructor would not work without
			if(o.pos != -1){
				//the old reference to this edge needs to be updated
				//o.ptr->neighbours[o.pos].ptr = this; // this is not even the same type
				//the index (pos) should not change except for we would cylce trough within one triangle or something
			}
			this->pos = o.pos;
			this->ptr = o.ptr;

			//invaldiate the old copy source
			o.pos = -1;
			o.ptr = nullptr;

		}*/
		Neighbour(){}
		~Neighbour(){
			if(pos != -1){
				//invalidate the old reference to this triangle
				ptr->neighbours[pos].ptr = nullptr;
				ptr->neighbours[pos].pos = -1;
			}
		}


	};

	bool registered = false;
	bool debug_is_stitch = false;//
	int  debug_nr = -1;

	/**
	 * These are the indices to the texture coordinates to the texture defined by the patch used in the first point
	 */
	//uint32_t tex_indices[3] = {0, 0, 0};

	//TODO: this should replace the tex indices
	//these should also be used for
	int32_t local_indices[3] = {-1, -1, -1};

	//EdgeReference   edges[3];
	Edge* edges[3] = {nullptr, nullptr, nullptr};
	//VertexReference points[3];
	Vertex* 		vertices[3] = {nullptr, nullptr, nullptr};
	Neighbour       neighbours[3];

	Triangle() { }

	//move constructor
	Triangle(Triangle && o) noexcept ;

private:
	Triangle(Triangle &o){
		assert(0); // we don't really like copies of triangles (only moves, references and pointers)
	}
public:

	Meshlet* getMeshlet(){
		return vertices[0]->meshlet;
	}
	~Triangle() {
		for(auto vert : vertices){
			if(vert!=nullptr)
				vert->removeTriangle(this);
		}
		for(auto nb : neighbours){
			if(nb.ptr != nullptr){
				nb.ptr->neighbours[nb.pos].invalidate();
			}
		}

		//TODO: edges!
	}

	bool containsPoint(Vertex* vert){
		for(Vertex* vertex : vertices){
			if(vertex == vert){
				return true;
			}
		}
		return false;
	}

	int getPointIndex(Vertex* vert){
		for(size_t i : {0, 1, 2}){
			if(vertices[i] == vert){
				return i;
			}
		}
		return -1;
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
	 void registerSelf(){
	 	for(int i : {0, 1, 2}){
	 		if(neighbours[i].valid()){
	 			int ind = neighbours[i].pos;
	 			neighbours[i].ptr->neighbours[ind].ptr = this;
	 			neighbours[i].ptr->neighbours[ind].pos = i;

	 		}

	 		//vertices should always be valid
			//add triangle to vertices
			vertices[i]->insertTriangle(this,i);
	 	}
	 }
	void cycle(int cnt) {
		assert(!registered);
		assert(!neighbours[0].valid());
		assert(!neighbours[1].valid());
		assert(!neighbours[2].valid());
		if(cnt == 1) {
			Vertex* p = vertices[0];
			vertices[0] = vertices[1];
			vertices[1] = vertices[2];
			vertices[2] = p;
		} else if(cnt == 2) {
			Vertex* p = vertices[0];
			vertices[0] = vertices[2];
			vertices[2] = vertices[1];
			vertices[1] = p;

		}
	}

	//TODO: reinsert these (used a lot in meshing and stitching)
	/*
	void replacePatchReference(Meshlet *from, Meshlet *to, int offset);

	void setNewMainPatch(Meshlet *patch);

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
/*
struct Stitch : public GeometryBase {
	//TODO: if that stitch stays empty remove the class

	//shared_ptr<TriangleBufConnector> getMostCurrentGpuTriangles() {
	//	return triangles_gpu.lock();
	//}

	//TODO: this could also just be a weak ptr to the buffer itself
	//weak_ptr<TriangleBufConnector> triangles_gpu;
};
*/
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
	void removeFromPatches(shared_ptr<Meshlet> except_for_patch = nullptr);

	shared_ptr<Meshlet> getOtherPatch(shared_ptr<Meshlet> const &patch);

	bool isGpuResidencyRequired();

	bool isPartOfActiveSet(const ActiveSet* active_set);

	void replacePatchReference(shared_ptr<Meshlet> from,
	                           shared_ptr<Meshlet> to);
	bool isDegenerated();

	bool isConnectingPatch(shared_ptr<Meshlet> patch);

	bool connectsSamePatches(shared_ptr<DoubleStitch> other);

	weak_ptr<DoubleStitch> weak_self;

	//the two patches the double stitch is connecting.
	//the main patch(the one whose texture we are using) is patch[0]
	weak_ptr<Meshlet> patches[2];

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
	void removeFromPatches(shared_ptr<Meshlet> except_for_patch = nullptr);

	shared_ptr<Meshlet> getAnyOtherPatch(shared_ptr<Meshlet> patch);
	//
	bool getOtherPatches(shared_ptr<Meshlet> patch,
	                     shared_ptr<Meshlet> patches_out[2]);


	void replacePatchReference(shared_ptr<Meshlet> from,
	                           shared_ptr<Meshlet> to,
	                           int offset);

	bool isCovertDoubleStitch();

	bool isConnectingPatch(shared_ptr<Meshlet> patch);

	///TODO: check if this weak self is needed
	weak_ptr<TripleStitch> weak_self;

	///TODO (not implemented yet)
	//int positionWithinSlot=-1;//since a triple stitch will most certainly only contain one triangle it is necessary to have
	weak_ptr<Meshlet> patches[3];
};
  */



 //TODO: remove if it stays unused
 struct UnregisteredEdge{
	 UnregisteredEdge() {}

	 //Keep in mind that as long as this unregistered
	 Meshlet* meshlet = nullptr;
	 int triangle_ind = -1;
	 int pos = -1;

 };

/**
 * @brief The Edge struct
 * Stores the connection between two points at a border (therefore there is only one triangle
 */
struct Edge {

	Edge() { }

	/*
	Edge(Meshlet* meshlet, int triangle_ind_meshlet, int ind_in_triangle){
		this->meshlet = meshlet;
		this->triangle_ind_meshlet = triangle_ind_meshlet;
		this->pos = ind_in_triangle;
	}*/

	//move constructor
	Edge(Edge && o){
		this->triangle = o.triangle;
		this->pos = o.pos;
		this->is_registered = o.is_registered;
		//update the reference of the triangles to this edge
		if(this->triangle != nullptr && this->is_registered){
			this->triangle->edges[this->pos] = this;
		}

		o.triangle = nullptr;
		o.pos = -1;
		o.is_registered = false;
	}
	//move asign:
	Edge& operator=(const Edge && o){
		this->triangle = o.triangle;
		this->pos = o.pos;
		this->is_registered = o.is_registered;
		//update the reference of the triangles to this edge
		if(this->triangle != nullptr && this->is_registered){
			this->triangle->edges[this->pos] = this;
		}

	}
private:
	Edge(Edge & o){
		assert(0); //We don't really want copies of an edge. References, pointers / move only.
	}
public:


	/*
	Triangle* triangle() const{
		return &meshlet->triangles[triangle_ind_meshlet];
	}
	 */

	Edge(Triangle* tri, int index_in_triangle) {
		triangle = tri;
		pos = index_in_triangle;
	}


	/*
	void registerInTriangle(int border_ind, int ind) {
		is_registered = true;
		triangle->edges[pos].border_ind = border_ind;
		triangle->edges[pos].ind = ind;
	}*/
	void registerInTriangle(){
		triangle->edges[pos] = this;
		is_registered = true;
	}

	/*
	void unregister() {
		assert(is_registered);
		triangle->edges[pos].invalidate();//invalidate the reference to this edge
		is_registered=false;
	}*/

	void unregister(){
		assert(is_registered);
		triangle->edges[pos] = nullptr;
		is_registered=false;
	}

	//borderlist is used in case a edge is already registered in the triangle
	bool getOrAttachNextEdge(int endpoint, Edge* &result, //bool debug = false){
	                  vector<Edge> &edge_list, bool debug = false) {

		//Meshlet* current_meshlet = meshlet;
		//int current_triangle_ind = triangle_ind_meshlet;
		int current_edge_ind_in_triangle = pos;

		Triangle* current_triangle = triangle;


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
				if(current_triangle->edges[current_edge_ind_in_triangle] != nullptr) {

					// if the edge already exists in the list:
					result = current_triangle->edges[current_edge_ind_in_triangle];
					return true;
				}else{
					// if we need to create a new edge:
					edge_list.emplace_back(current_triangle, current_edge_ind_in_triangle);
					result = &edge_list.back();
					return true;
				}
			}

			Triangle* old_triangle = current_triangle;
			current_triangle = current_triangle->neighbours[current_edge_ind_in_triangle].ptr;
			current_edge_ind_in_triangle =
					old_triangle->neighbours[current_edge_ind_in_triangle].pos;
			if(debug){
				cout << current_triangle << " at " << current_edge_ind_in_triangle	 << endl;
			}
		}

		getOrAttachNextEdge(endpoint, result, edge_list, true);//debug... this should never happen
		//getOrAttachNextEdge(endpoint, result, border_list,true);//debug... this should never happen
		assert(0);//more than 100 triangles on this node means something is off!!!
		return false;
	}

	bool isOpen() {
		return !triangle->neighbours[pos].valid();
	}

	bool getOtherEdge(int endpoint, Edge &result) {
		assert(0);
		return false;
	}

	//TODO:fix and reinsert (used in stitching and meshing)

	Vertex* vertices(int i) {
		assert(i == 0 || i == 1);
		int ind = pos + i;
		if(ind == 3) {
			ind = 0;
		}
		return triangle->vertices[ind];
	}
	Vertex* oppositePoint(){
		int ind = pos - 1;
		if(ind == -1) {
			ind = 2;
		}
		return triangle->vertices[ind];
	}
	/*

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

	Triangle* triangle;
	//Meshlet* meshlet;
	//int triangle_ind_meshlet;
	int pos;//edge index within this triangle

	bool is_registered=false;

	int debug=-1;

	//this is weird

	bool outermost_connections_made[2] = {false, false};
	Vertex* outermost_connections[2];
	Vector2f outermost_connection_fragment_pos[2];
};



//this is down here so it can be inlined
inline Vertex::Vertex ( Vertex && o) noexcept{
	meshlet = o.meshlet;
	p = o.p;
	n = o.n;
	color = o.color;
//	tex_ind_in_main_patch  = o.tex_ind_in_main_patch;
	triangles = std::move(o.triangles);
	for(auto triangle : triangles){
		//set old references to this vertex to the new position
		triangle.triangle->vertices[triangle.ind_in_triangle] = this;
	}
}


inline Triangle::Triangle(Triangle && o) noexcept{

	for(auto i : {0,1,2}){
		Neighbour &nb = neighbours[i];
		nb = o.neighbours[i];
		//update the neighbours reference pointing at this
		if(nb.ptr){
			nb.ptr->neighbours[nb.pos].ptr = this;
			o.neighbours[i].invalidate();//invalidating the move source
		}
		debug_nr = o.debug_nr;


		assert(o.vertices[i]->p[3] == 1.0f);
		//update the vertices and the pointer from them
		vertices[i] = o.vertices[i];
		o.vertices[i] = nullptr;//invalidate at source (maybe not necessary)
		assert(vertices[i]);
		vertices[i]->replaceTriangle(&o,this);


		//Update the references of edges to the triangles.
		edges[i] = o.edges[i];
		if(edges[i]){
			edges[i]->triangle = this;
		}
		o.edges[i] = nullptr; // invalidate at source (maybe not necessary)

	}
}

#endif // FILE_MESH_STRUCTURE_H