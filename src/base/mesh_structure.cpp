#include <base/mesh_structure.h>

#include <iostream>

#include <utils/principal_plane.h>
#include <gpu/tex_atlas.h>
#include <graph/deformation_node.h>

using namespace std;
using namespace Eigen;

///TODO: replace this with something that returns the pointer not the shared_ptr
Meshlet::Meshlet(int id, octree::Octree *octree) :
				id(id){
	//TODO: get rid of the octree here? or figure out what to do about it
	//setOctree(octree);

	//TODO: get this from a constant... essentially it should speed up the process
	//triangles.reserve(500);
	//vertices.reserve(900);

	deformation_node = make_shared<DeformationNode>(this);
}

Meshlet::~Meshlet() {
	triangles.clear();
	vertices.clear();
	//cout << "[Meshlet::MeshletDEBUG: destructor got called." << endl;
	//cout << "THE TRIANGLES ARE NOT DELETED!!!!!" << endl;
}


void Meshlet::updateCenterPoint() {
	Vector4d c = Vector4d(0, 0, 0, 0);
	for(size_t i = 0; i < vertices.size(); i++) {
		c += vertices[i].getP().cast<double>();
	}
	setSphere((c * (1.0 / double(vertices.size()))).block<3, 1>(0, 0).cast<float>(),
	          radius());
}

void Meshlet::updateSphereRadius() {
	float r = 0;
	Vector3f p = center();
	Vector4f p_center(p[0], p[1], p[2], 0);
	for(size_t i = 0; i < vertices.size(); i++) {
		Vector4f diff = p_center - vertices[i].getP();
		float dist = Vector3f(diff[0], diff[1], diff[2]).norm();
		r = max(r, dist);
	}
	setSphere(center(), r);
}

void Meshlet::updatePrincipalPlaneAndCenter() {
	PrincipalPlaneAccumulator accumulator;
	for(size_t i = 0; i < vertices.size(); i++) {
		accumulator.addPoint(vertices[i].getP());
		if(isnan(vertices[i].getP()[0])) {
			cout << "[Meshlet::updatePrincipalPlaneAndCenter] why is this nan?" << endl;
		}
	}
	principal_plane = accumulator.plane();
	Vector4f center = accumulator.centerPoint();
	setSphere(Vector3f(center[0], center[1], center[2]),radius());
	if(isnan(center[0])) {
		cout << "[Meshlet::updatePrincipalPlaneAndCenter] why is this nan?" << endl;
	}
}

void Meshlet::addTexPatch(shared_ptr<MeshTexture> tex_patch) {
	tex_patches.push_back(tex_patch);
}

void Meshlet::removeTexPatch(shared_ptr<MeshTexture> tex_patch) {
	tex_patches.erase(remove(tex_patches.begin(), tex_patches.end(), tex_patch), 
	                  tex_patches.end());
}

void Meshlet::removeTexPatches(vector<shared_ptr<MeshTexture>> tex_patches) {
	for(auto tex_patch : tex_patches) {
		removeTexPatch(tex_patch);
	}
}


bool Vertex::encompassed(){
	if(triangles.size() <2){
		return false;
	}
	int running = 1000;
	Triangle* tri = triangles[0].triangle;
	int ind = triangles[0].ind_in_triangle;
	while(running--){
		ind--;
		if(ind==-1){
			ind=2;
		}
		if(!tri->neighbours[ind].valid()){
			return false;
		}
		int ind_new = tri->neighbours[ind].pos;
		tri = tri->neighbours[ind].ptr;
		ind = ind_new;
		if(tri == triangles[0].triangle){
			return true;
		}

	}
	assert(0);
	return false;
}
