#include <base/mesh_structure.h>

#include <iostream>

#include <utils/principal_plane.h>
#include <gpu/tex_atlas.h>
#include <graph/deformation_node.h>

using namespace std;
using namespace Eigen;

///TODO: replace this with something that returns the pointer not the shared_ptr
MeshPatch::MeshPatch(octree::Octree *octree) {
	//octree->add(shared_from_this());

	//TODO: get this from a constant... essentially it should speed up the process
	triangles.reserve(800);
	vertices.reserve(800);

	deformation_node = make_shared<DeformationNode>(this);
}

MeshPatch::~MeshPatch() {
	//cout << "[MeshPatch::~MeshPatch] DEBUG: destructor got called." << endl;
	//cout << "THE TRIANGLES ARE NOT DELETED!!!!!" << endl;
}

bool MeshPatch::isCreationProcessOngoing() {
	return (geom_tex_patch == nullptr);
}

void MeshPatch::updateCenterPoint() {
	Vector4d c = Vector4d(0, 0, 0, 0);
	for(size_t i = 0; i < vertices.size(); i++) {
		c += vertices[i].p.cast<double>();
	}
	setSphere((c * (1.0 / double(vertices.size()))).block<3, 1>(0, 0).cast<float>(),
	          radius());
}

void MeshPatch::updateSphereRadius() {
	float r = 0;
	Vector3f p = center();
	Vector4f p_center(p[0], p[1], p[2], 0);
	for(size_t i = 0; i < vertices.size(); i++) {
		Vector4f diff = p_center - vertices[i].p;
		float dist = Vector3f(diff[0], diff[1], diff[2]).norm();
		r = max(r, dist);
	}
	setSphere(center(), r);
}

void MeshPatch::updatePrincipalPlaneAndCenter() {
	PrincipalPlaneAccumulator accumulator;
	for(size_t i = 0; i < vertices.size(); i++) {
		accumulator.addPoint(vertices[i].p);
		if(isnan(vertices[i].p[0])) {
			cout << "[MeshPatch::updatePrincipalPlaneAndCenter] why is this nan?" << endl;
		}
	}
	principal_plane = accumulator.plane();
	Vector4f p_center = accumulator.centerPoint();
	setSphere(Vector3f(p_center[0], p_center[1], p_center[2]), radius());
	if(isnan(p_center[0])) {
		cout << "[MeshPatch::updatePrincipalPlaneAndCenter] why is this nan?" << endl;
	}
}

void MeshPatch::addTexPatch(shared_ptr<MeshTexture> tex_patch) {
	//todo: set according flag
	//cout << "[MeshPatch::addTexPatch]we added a new texture for this patch, a (re)upload should be imminent" << endl;
	cpu_tex_patch_ahead = true;
	tex_patches.push_back(tex_patch);
}

void MeshPatch::removeTexPatch(shared_ptr<MeshTexture> tex_patch) {
	//todo: set according flag
	cpu_tex_patch_ahead = true;
	tex_patches.erase(remove(tex_patches.begin(), tex_patches.end(), tex_patch), 
	                  tex_patches.end());
}

void MeshPatch::removeTexPatches(vector<shared_ptr<MeshTexture>> tex_patches) {
	for(auto tex_patch : tex_patches) {
		removeTexPatch(tex_patch);
	}
}

bool MeshPatch::isGpuResidencyRequired() {
	return (!gpu_required_by_sets.empty());
}

int MeshPatch::addActiveSet(ActiveSet *active_set) {
	int count;
	gpu_required_by_sets_mutex.lock();
	count = gpu_required_by_sets.size();
	gpu_required_by_sets.push_back(active_set);
	gpu_required_by_sets_mutex.unlock();
	return count;
}

//returns the amount of active set bound before removing this one!!!!!
int MeshPatch::removeActiveSet(ActiveSet *active_set) {
	int count;
	gpu_required_by_sets_mutex.lock();
	count = gpu_required_by_sets.size();
	//the erase deletes the elements at the end. but i stil do not know why this is necessary
	gpu_required_by_sets.erase(remove(gpu_required_by_sets.begin(), 
	                                  gpu_required_by_sets.end(), active_set), 
	                           gpu_required_by_sets.end());
	gpu_required_by_sets_mutex.unlock();
	return count;
}

bool MeshPatch::isPartOfActiveSet(const ActiveSet *active_set) {
	for(size_t i = 0; i < gpu_required_by_sets.size(); i++) {
		if(gpu_required_by_sets[i] == active_set) {
			return true;
		}
	}
	return false;
}

bool MeshPatch::isPartOfActiveSetWithNeighbours(const ActiveSet *active_set) {
	if(!isPartOfActiveSet(active_set)) {
		return false;
	}

	double_stitch_mutex.lock();
	for(shared_ptr<DoubleStitch> stitch : double_stitches) {
		if(stitch->patches[0].lock().get() != this) {
			continue;
		}
		if(!stitch->isPartOfActiveSet(active_set)) {
			//this means not all the neighbours of this stitch are loaded to the gpu
			double_stitch_mutex.unlock();
			return false;
		}
	}
	double_stitch_mutex.unlock();

	triple_stitch_mutex.lock();
	for(shared_ptr<TripleStitch> stitch : triple_stitches) {
		if(stitch->patches[0].lock().get() != this) {
			continue;
		}
		if(!stitch->isPartOfActiveSet(active_set)) {
			//this means not all the neighbours of this are loaded to the gpu
			triple_stitch_mutex.unlock();
			return false;
		}
	}
	triple_stitch_mutex.unlock();
	return true;
}

bool MeshPatch::isValidOnGpu() {
	cout << "[MeshPatch::isValidOnGpu] this check probably is useless! don't use it / get rid of it" << endl;
	//todo implement more checks
	return (gpu.lock() != nullptr);
}

shared_ptr<DoubleStitch> MeshPatch::getDoubleStitchWith(MeshPatch* other_patch) {
	//cout << "[MeshPatch::getDoubleStitchWith]replace this with the other function not working with IDs" << endl;
	for(int i = 0; i < double_stitches.size(); i++) {
		bool invalid_stitch = false;
		if(double_stitches[i]->patches[0].use_count()) {
			if(double_stitches[i]->patches[0].lock().get() == other_patch) {
				return double_stitches[i];
			}
		} else {
			invalid_stitch = true;
		}
		if(double_stitches[i]->patches[1].use_count()) {
			if(double_stitches[i]->patches[1].lock().get() == other_patch) {
				return double_stitches[i];
			}
		} else {
			invalid_stitch=true;
		}
		//DEBUG output..... please delete when this is not seen anywhere
		if(invalid_stitch) {
			cout << "this is a invalid stitch" << endl;
		}
	}

	shared_ptr<DoubleStitch> empty;
	return empty;
}

void MeshPatch::addStitchReference(shared_ptr<DoubleStitch> stitch) {
	double_stitch_mutex.lock();
	double_stitches.push_back(stitch);
	double_stitch_mutex.unlock();
}

shared_ptr<TripleStitch> MeshPatch::getTripleStitchWith(
		MeshPatch *other_patch1,
		MeshPatch *other_patch2) {
	MeshPatch* id1 = this;
	MeshPatch* id2 = other_patch1;
	MeshPatch* id3 = other_patch2;
	//all permutations:
	const MeshPatch *perm[][3] = {{id1, id2, id3},
	                              {id1, id3, id2},
	                              {id2, id1, id3},
	                              {id2, id3, id1},
	                              {id3, id1, id2},
	                              {id3, id2, id1}};
	//cout << "dead lock directly here" << endl;
	triple_stitch_mutex.lock();
	for(int i = 0; i < triple_stitches.size(); i++) {
		shared_ptr<TripleStitch> stitch = triple_stitches[i];
		for(int j = 0; j < 6; j++) {
			if(stitch->patches[0].lock().get() == perm[j][0] &&
			   stitch->patches[1].lock().get() == perm[j][1] &&
			   stitch->patches[2].lock().get() == perm[j][2]) {
				triple_stitch_mutex.unlock();
				return stitch;
			}
		}
	}
	triple_stitch_mutex.unlock();
	shared_ptr<TripleStitch> empty;
	return empty;
}

void MeshPatch::addStitchReference(shared_ptr<TripleStitch> stitch) {
	triple_stitch_mutex.lock();
	triple_stitches.push_back(stitch);
	triple_stitch_mutex.unlock();
}

void MeshPatch::removeStitchReference(shared_ptr<DoubleStitch> stitch) {
	double_stitch_mutex.lock();
	double_stitches.erase(
			remove(double_stitches.begin(),double_stitches.end(),stitch),
			double_stitches.end());
	double_stitch_mutex.unlock();
}

void MeshPatch::removeStitchReference(shared_ptr<TripleStitch> stitch) {
	triple_stitch_mutex.lock();
	triple_stitches.erase(
			remove(triple_stitches.begin(),triple_stitches.end(),stitch),
			triple_stitches.end());
	triple_stitch_mutex.unlock();
}

set<shared_ptr<MeshPatch> > MeshPatch::getNeighbours() {
	set<shared_ptr<MeshPatch>> neighbours;

	double_stitch_mutex.lock();
	for(auto double_stitch : double_stitches) {
		neighbours.insert(double_stitch->getOtherPatch(weak_self.lock()));
	}
	double_stitch_mutex.unlock();

	triple_stitch_mutex.lock();
	for(auto triple_stitch : triple_stitches) {
		//continue; //does this make a diference?
		shared_ptr<MeshPatch> other_patches[2];
		triple_stitch->getOtherPatches(weak_self.lock(), other_patches);
		if(other_patches[0].get() != this) {
			neighbours.insert(other_patches[0]);
		} else {
			cout << "something is weird" << endl;
			assert(0);
		}
		if(other_patches[1].get() != this) {
			neighbours.insert(other_patches[1]);
		} else {
			cout << "something is weird" << endl;
			assert(0);
		}
		if(other_patches[0].get() == other_patches[1].get()) {
			cout << "this should not happen, even though it might"
			        " be caused when patches are combined and the triple"
			        "stitch degenerates" << endl;
			assert(0);
		}
	}
	triple_stitch_mutex.unlock();

	return neighbours;
}

shared_ptr<DoubleStitch> MeshPatch::getDoubleStitchTo(shared_ptr<MeshPatch> patch) {
	double_stitch_mutex.lock();
	for(auto double_stitch : double_stitches) {
		if(double_stitch->isConnectingPatch(patch)) {
			double_stitch_mutex.unlock();
			return double_stitch;
		}
	}
	double_stitch_mutex.unlock();
	return nullptr;
}

shared_ptr<TripleStitch> MeshPatch::getTripleStitchTo(shared_ptr<MeshPatch> patch) {
	triple_stitch_mutex.lock();
	for(auto triple_stitch : triple_stitches) {
		if(triple_stitch->isConnectingPatch(patch)) {
			triple_stitch_mutex.unlock();
			return triple_stitch;
		}
	}
	triple_stitch_mutex.unlock();
	return nullptr;
}

bool MeshPatch::isGeometryFullyAllocated() {
	assert(0);
	return (gpu.lock() != nullptr);
}

TripleStitch::TripleStitch() {
	//mostly this will only be one triangle
	triangles.reserve(4);
}

TripleStitch::~TripleStitch() {
	#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
	cout << "[TripleStitch::~TripleStitch] DEBUG: deletion of triple stitch. " << endl;
	#endif
	//cout << "THE TRIANGLES ARE NOT DELETED!!!!!" << endl;
}

bool TripleStitch::isGpuResidencyRequired() {
	//test if one of the required patches is not required anymore, otherwise keep this triangle on the gpu
	return (patches[0].lock()->isGpuResidencyRequired() &&
	        patches[1].lock()->isGpuResidencyRequired() &&
	        patches[2].lock()->isGpuResidencyRequired());
}

bool TripleStitch::isPartOfActiveSet(const ActiveSet *active_set) {
	//test if one of the required patches is not required anymore, otherwise keep this triangle on the gpu
	return (patches[0].lock()->isPartOfActiveSet(active_set) &&
	        patches[1].lock()->isPartOfActiveSet(active_set) &&
	        patches[2].lock()->isPartOfActiveSet(active_set));
}

void TripleStitch::removeFromPatches(shared_ptr<MeshPatch> except_for_patch) {
	for(size_t i = 0; i < 3; i++) {
		shared_ptr<MeshPatch> locked = patches[i].lock();
		if(except_for_patch == locked || locked.use_count() == 0) {
			continue;
		} else {
			locked->removeStitchReference(weak_self.lock());
		}
	}
}

shared_ptr<MeshPatch> TripleStitch::getAnyOtherPatch(
		shared_ptr<MeshPatch> patch) {
	if(patch == patches[0].lock()) {
		return patches[1].lock();
	}
	if(patch == patches[1].lock()) {
		return patches[0].lock();
	}
	if(patch == patches[2].lock()) {
		return patches[0].lock();
	}
	return nullptr;
}

bool TripleStitch::getOtherPatches(shared_ptr<MeshPatch> patch, 
                                   shared_ptr<MeshPatch> patches_out[]) {
	if(patch == patches[0].lock()) {
		patches_out[0] = patches[1].lock();
		patches_out[1] = patches[2].lock();
		return true;
	}
	if(patch == patches[1].lock()) {
		patches_out[0] = patches[0].lock();
		patches_out[1] = patches[2].lock();
		return true;
	}
	if(patch == patches[2].lock()) {
		patches_out[0] = patches[0].lock();
		patches_out[1] = patches[1].lock();
		return true;
	}
	return false;
}

void TripleStitch::replacePatchReference(shared_ptr<MeshPatch> from, 
                                         shared_ptr<MeshPatch> to, int offset) {
	for(size_t i = 0; i < 3; i++) {
		if(patches[i].lock() == from) {
			patches[i] = to;
		}
	}
	replacePatchInTriangleReferences(from, to, offset);
}

bool TripleStitch::isCovertDoubleStitch() {
	return ((patches[0].lock() == patches[1].lock()) ||
	        (patches[1].lock() == patches[2].lock()) ||
	        (patches[0].lock() == patches[2].lock()));
}

bool TripleStitch::isConnectingPatch(shared_ptr<MeshPatch> patch) {
	for(size_t i = 0; i < 3; i++) {
		//assert(0); //here (at the weak pointer) we had a memory exception
		if(patches[i].lock() == patch) {
			return true;
		}
	}
	return false;
}

DoubleStitch::DoubleStitch() {
	triangles.reserve(100);
}

DoubleStitch::~DoubleStitch() {
	#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
	cout << "[DoubleStitch::~DoubleStitch] DEBUG: deletion of double stitch." << endl;
	#endif
}

void DoubleStitch::removeFromPatches(shared_ptr<MeshPatch> except_for_patch) {
	for(size_t i = 0; i < 2; i++) {
		shared_ptr<MeshPatch> locked = patches[i].lock();
		if(except_for_patch == locked || locked.use_count() == 0) {
			continue;
		}
		locked->removeStitchReference(weak_self.lock());
	}
}

shared_ptr<MeshPatch> DoubleStitch::getOtherPatch(
		shared_ptr<MeshPatch> const &patch) {
	if(patch == patches[0].lock()) {
		return patches[1].lock();
	}
	if(patch == patches[1].lock()) {
		return patches[0].lock();
	}
	return nullptr;
}

bool DoubleStitch::isGpuResidencyRequired() {
	//test if one of the required patches is not required anymore, otherwise keep this triangle on the gpu
	return (patches[0].lock()->isGpuResidencyRequired() &&
	        patches[1].lock()->isGpuResidencyRequired());
}

bool DoubleStitch::isPartOfActiveSet(const ActiveSet *active_set) {
	//test if one of the required patches is not required anymore, otherwise keep this triangle on the gpu
	return (patches[0].lock()->isPartOfActiveSet(active_set) &&
	        patches[1].lock()->isPartOfActiveSet(active_set));
}

void DoubleStitch::replacePatchReference(shared_ptr<MeshPatch> from, 
                                         shared_ptr<MeshPatch> to) {
	for(size_t i = 0; i < 2; i++) {
		if(patches[i].lock() == from) {
			patches[i] = to;
		}
	}
}

bool DoubleStitch::isDegenerated() {
	return (patches[0].lock() == patches[1].lock());
}

bool DoubleStitch::isConnectingPatch(shared_ptr<MeshPatch> patch) {
	if(patches[0].lock() == patch) {
		return true;
	}
	if(patches[1].lock() == patch) {
		return true;
	}
	return false;
}

bool DoubleStitch::connectsSamePatches(shared_ptr<DoubleStitch> other) {
	if(other->patches[0].lock() == patches[0].lock() &&
	   other->patches[1].lock() == patches[1].lock()) {
		return true;
	}
	if(other->patches[1].lock() == patches[0].lock() &&
	   other->patches[0].lock() == patches[1].lock()) {
		return true;
	}
	return false;
}

void GeometryBase::replacePatchInTriangleReferences(shared_ptr<MeshPatch> from,
                                                    shared_ptr<MeshPatch> to,
                                                    int vertex_offset) {
	//do i even know what this is about?
	for(size_t i = 0; i < triangles.size(); i++) {
		triangles[i].replacePatchReference(from.get(), to.get(), vertex_offset);
	}
}

void GeometryBase::setNewMainPatchForTriangles(
		shared_ptr<MeshPatch> main_patch) {
	for(size_t i = 0; i < triangles.size(); i++) {
		triangles[i].setNewMainPatch(main_patch.get());
	}
}

void GeometryBase::deregisterTriangles() {
	for(size_t i = 0; i < triangles.size(); i++) {
		for(size_t k = 0; k < 3; k++) {
			//TODO: remove this comment and the "removeTriangle" line if it turns out it really is not needed
			//Actually this should not be necessary: (might even be the cause for crashes)
			triangles[i].points[k].get()->removeTriangle(&triangles[i]);

			//a bit expensive but usually less than 8 iterations until triangle is found
			if(triangles[i].neighbours[k].valid()) {
				Triangle *tri1 = triangles[i].neighbours[k].ref.get();
				Triangle *tri2 =
						triangles[i].neighbours[k].ref.get()->neighbours[triangles[i].neighbours[k].pos].ref.get();
				assert(
						triangles[i].neighbours[k].ref.get()->neighbours[triangles[i].neighbours[k].pos].ref.get() ==
						&triangles[i]);
				assert(
						triangles[i].neighbours[k].ref.get()->neighbours[triangles[i].neighbours[k].pos].valid());
				triangles[i].neighbours[k].ref.get()->neighbours[triangles[i].neighbours[k].pos].invalidate();
				triangles[i].neighbours[k].invalidate();
			}
		}
	}
}

void Triangle::replacePatchReference(MeshPatch *from, MeshPatch *to, 
                                     int offset) {
	for(size_t i = 0; i < 3; i++) {
		if(points[i].getPatch() == from) {
			points[i].set(to, points[i].getIndex() + offset);
		}
	}
}

void Triangle::setNewMainPatch(MeshPatch *patch) {
	for(size_t i = 1; i < 3; i++) {
		if(points[i].getPatch() == patch) {
			swap(points[0], points[i]);
			return;
		}
	}
}

VertexReference Triangle::getThirdPoint(VertexReference p1, 
                                        VertexReference p2) {
	VertexReference result;
	bool debug = false;
	for(size_t i = 0; i < 3; i++) {
		if(points[i].getIndex() == p1.getIndex() &&
		   points[i].getPatch() == p1.getPatch()) {
			continue;
		}
		if(points[i].getIndex() == p2.getIndex() &&
		   points[i].getPatch() == p2.getPatch()) {
			continue;
		}
		debug = true;
		result = points[i];
	}
	if(debug == false) {
		assert(0);
	}
	return result;
}

bool Vertex::encompassed() {
	if(triangles.size() < 3) {
		return false;
	}

	TriangleReference tri_ref;
	Triangle *first_tri = triangles[0].triangle.get();
	Triangle *tri = first_tri;

	int ind = triangles[0].ind_in_triangle;
	int debugTTL = 100000;
	while(debugTTL--) {
		ind--;
		if(ind == -1) {
			ind = 2;
		}
		if(tri->neighbours[ind].valid()) {
			//get the next triangle
			tri_ref = tri->neighbours[ind].ref;
			ind = tri->neighbours[ind].pos;
			tri = tri_ref.get();
			if(tri == first_tri) {
				//we came back to the first triangle
				return true;
			}
		} else {
			return false;
		}
	}
	encompassed();//debug just so we can GDB into it
	assert(0);
}

vector<shared_ptr<VertexBufConnector>> debug_retain_vertices;
vector<shared_ptr<TriangleBufConnector>> debug_retain_triangle_buf;
MeshPatchGpuHandle::MeshPatchGpuHandle(GpuGeomStorage *gpu_geom_storage,
                                       int nr_vertices, int nr_triangles) 
		: vertices_dest(gpu_geom_storage->vertex_buffer->getBlock(nr_vertices)),
		  vertices_source(gpu_geom_storage->vertex_buffer->getBlock(nr_vertices)),
		  patch_infos(gpu_geom_storage->patch_info_buffer->getBlock(1)),
		  triangles(gpu_geom_storage->triangle_buffer->getBlock(nr_triangles)),
		  gpu_vertices_changed(false),
		  label_tex_valid(false),
		  weighted_label_tex_count(0) {
}

MeshPatchGpuHandle::~MeshPatchGpuHandle() {
	shared_ptr<MeshPatch> download_to = download_to_when_finished.lock();
}

bool MeshPatchGpuHandle::setLabelTex(shared_ptr<MeshTextureGpuHandle> tex) {
	label_tex = tex;
	label_tex_valid = true;
}

bool MeshPatchGpuHandle::addWeightedLabelTex(
		shared_ptr<MeshTextureGpuHandle> tex) {
	size_t max_lab_cnt = sizeof(weighted_label_texs) / sizeof(tex);
	if(weighted_label_tex_count >= max_lab_cnt) {
		return false;
	} else {
		weighted_label_texs[weighted_label_tex_count] = tex;
		weighted_label_tex_count++;
		return true;
	}
	return false;
}

void Triangle::registerTriangle(TriangleReference &triangle_to_register,
                                bool search_edges, bool debug) {
	//assert(0);//Depending on if the triangle structure is available this method could be simplified
	Triangle *triangle = triangle_to_register.get();
	triangle->registered = true;
	Vertex &p1 = *(triangle->points[0].get());
	Vertex &p2 = *(triangle->points[1].get());
	Vertex &p3 = *(triangle->points[2].get());

	if(!search_edges) {
		if(debug) {
			//check if the proposed edge neighbours really make sense
			//lets debug here:
			Neighbour neighbours_test[3];
			for(uint32_t i = 0; i < p1.triangles.size(); i++) {
				if(p1.triangles[i].triangle.get()->containsPoint(triangle_to_register.get()->points[1])) {
					TriangleReference &other_triangle_ref = p1.triangles[i].triangle;
					Triangle *other_triangle = other_triangle_ref.get();
					int k = other_triangle->getPosInTriangle(triangle_to_register.get()->points[0], 
					                                         triangle_to_register.get()->points[1]);
					neighbours_test[0].ref = other_triangle_ref;
					neighbours_test[0].pos = k;
				}
				if(p1.triangles[i].triangle.get()->containsPoint(triangle_to_register.get()->points[2])) {
					TriangleReference &other_triangle_ref = p1.triangles[i].triangle;
					Triangle *other_triangle = other_triangle_ref.get();
					//TODO: it is ineficient to first check if it exists and afterwards retrieve it
					int k = other_triangle->getPosInTriangle(triangle_to_register.get()->points[0],
					                                         triangle_to_register.get()->points[2]);
					neighbours_test[2].ref = other_triangle_ref;
					neighbours_test[2].pos = k;
				}
			}

			for(uint32_t i=0; i < p2.triangles.size(); i++) {
				if(!p2.triangles[i].triangle.container->debugIsValid()) {
					assert(0);
				}
				if(p2.triangles[i].triangle.get()->containsPoint(triangle_to_register.get()->points[2])) {
					TriangleReference &other_triangle_ref = p2.triangles[i].triangle;
					Triangle *other_triangle = other_triangle_ref.get();
					//TODO: it is ineficient to first check if it exists and afterwards retrieve it
					int k = other_triangle->getPosInTriangle(triangle_to_register.get()->points[1],
					                                         triangle_to_register.get()->points[2]);
					neighbours_test[1].ref = other_triangle_ref;
					neighbours_test[1].pos = k;
				}
			}

			for(uint8_t i = 0; i < 3; i++) {
				if((triangle->neighbours[i].ref != neighbours_test[i].ref) ||
				   (triangle->neighbours[i].pos != neighbours_test[i].pos)) {
					assert(0);
				}
			}
		}

		for(uint8_t i = 0; i < 3; i++) {
			if(triangle->neighbours[i].valid()) {
				Triangle *other = triangle->neighbours[i].ref.get();
				other->neighbours[triangle->neighbours[i].pos].ref = triangle_to_register;
				other->neighbours[triangle->neighbours[i].pos].pos = i;
				//DEBUG: get rid of this!
				if(other->neighbours[triangle->neighbours[i].pos].ref.get() != triangle) {
					assert(0); //this seems invalid then!!!!!!"!!!!!!!!!°
				}
			}
		}

		//register points!
		Vertex::VertexInTriangle vit;
		vit.triangle = triangle_to_register;
		vit.ind_in_triangle = 0;
		p1.triangles.pushBack(vit);

		vit.ind_in_triangle = 1;
		p2.triangles.pushBack(vit);

		vit.ind_in_triangle = 2;
		p3.triangles.pushBack(vit);
		return;
	} else {
		for(uint8_t i = 0; i < 3; i++) {
			triangle->neighbours[i].invalidate();
		}
	}

	//Add triangle to neighbour list af adjacent triangles
	int debug_count=0;
	for(uint32_t i = 0; i < p1.triangles.size(); i++) {
		if(p1.triangles[i].triangle.get()->containsPoint(triangle_to_register.get()->points[1])) {
			TriangleReference &other_triangle_ref = p1.triangles[i].triangle;
			Triangle *other_triangle = other_triangle_ref.get();
			//TODO: it is ineficient to first check if it exists and afterwards retrieve it
			int k = other_triangle->getPosInTriangle(triangle_to_register.get()->points[0],
			                                         triangle_to_register.get()->points[1]);
			if(other_triangle->neighbours[k].valid()) {
				//it seems that we want to add another triangle to an edge that already has two
				Triangle *debug_triangle = other_triangle->neighbours[k].ref.get();
				assert(0);// when registering a triangle we hope that it already is valid
			}
			//get where the triangle belongs
			triangle->neighbours[0].ref = other_triangle_ref;
			triangle->neighbours[0].pos = k;
			triangle->neighbours[0].debug = debug;
			other_triangle->neighbours[k].ref = triangle_to_register;
			other_triangle->neighbours[k].pos = 0;
			other_triangle->neighbours[k].debug = debug;

			debug_count++;
		}
		if(p1.triangles[i].triangle.get()->containsPoint(triangle_to_register.get()->points[2])) {
			TriangleReference &other_triangle_ref = p1.triangles[i].triangle;
			Triangle *other_triangle = other_triangle_ref.get();
			//TODO: it is ineficient to first check if it exists and afterwards retrieve it
			int k = other_triangle->getPosInTriangle(triangle_to_register.get()->points[0],
			                                         triangle_to_register.get()->points[2]);
			if(other_triangle->neighbours[k].valid()) {
				//it seems that we want to add another triangle to an edge that already has two
				Triangle* debug_triangle = other_triangle->neighbours[k].ref.get();
				assert(0);// when registering a triangle we hope that it already is valid
			}
			//get where the triangle belongs
			triangle->neighbours[2].ref = other_triangle_ref;
			triangle->neighbours[2].pos = k;
			triangle->neighbours[2].debug = debug;
			other_triangle->neighbours[k].ref = triangle_to_register;
			other_triangle->neighbours[k].pos = 2;
			other_triangle->neighbours[k].debug = debug;

			debug_count++;
		}
	}

	for(uint32_t i = 0; i < p2.triangles.size(); i++) {
		if(!p2.triangles[i].triangle.container->debugIsValid()) {
			assert(0);
		}
		if(p2.triangles[i].triangle.get()->containsPoint(triangle_to_register.get()->points[2])) {
			TriangleReference &other_triangle_ref = p2.triangles[i].triangle;
			Triangle *other_triangle = other_triangle_ref.get();
			//TODO: it is ineficient to first check if it exists and afterwards retrieve it
			int k = other_triangle->getPosInTriangle(triangle_to_register.get()->points[1],
			                                         triangle_to_register.get()->points[2]);
			if(other_triangle->neighbours[k].valid()) {
				//it seems that we want to add another triangle to an edge that already has two
				Triangle* debugTriangle = other_triangle->neighbours[k].ref.get();
				assert(0);// when registering a triangle we hope that it already is valid
			}
			//get where the triangle belongs
			triangle->neighbours[1].ref = other_triangle_ref;
			triangle->neighbours[1].pos = k;
			triangle->neighbours[1].debug = debug;
			other_triangle->neighbours[k].ref = triangle_to_register;
			other_triangle->neighbours[k].pos = 1;
			other_triangle->neighbours[k].debug = debug;

			debug_count++;
		}
	}

	if(debug_count > 3) {
		// This means that we likely have two triangles with the same edge points
		assert(0);
	}

	//last step is to add the triangles to the according points
	Vertex::VertexInTriangle vit;
	vit.triangle = triangle_to_register;
	vit.ind_in_triangle = 0;
	p1.triangles.pushBack(vit);

	vit.ind_in_triangle = 1;
	p2.triangles.pushBack(vit);

	vit.ind_in_triangle = 2;
	p3.triangles.pushBack(vit);




	//check if neighbouring triangles are oriented consistently
	Triangle* tri = triangle;
	for(uint8_t i = 0; i < 3; i++) {
		if(tri->neighbours[i].valid()) {
			Triangle *nb = tri->neighbours[i].ref.get();
			Vertex *v11 = tri->points[i].get();
			int ind2 = (i == 2) ? 0 : i + 1;
			Vertex *v12 = tri->points[ind2].get();

			int ind1 = tri->neighbours[i].pos;
			Vertex *v21 = nb->points[ind1].get();
			ind2 = (ind1 == 2) ? 0 : ind1 + 1;
			Vertex *v22 = nb->points[ind2].get();
			if(v11 != v22 || v12 != v21) {
				assert(0);
			}
		}
	}
}