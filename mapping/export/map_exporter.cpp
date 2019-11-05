#include "map_exporter.h"

#include <unordered_map>
#include <fstream>

#include <assimp/Exporter.hpp>
#include <assimp/scene.h>           // Output data structure
#include <assimp/version.h>
#include <Eigen/Core>

#include <graph/deformation_node.h>
#include <mesh_reconstruction.h>

using namespace std;
using namespace Eigen;

void MapExporter::exportMap(MeshReconstruction *map, string path,
							unsigned int properties) {
}

//only HighRes or LowRes is valid
void MapExporter::exportMesh(MeshReconstruction *map, string path,
							 MapExporter::Properties_ properties) {
}

void MapExporter::storeCoarse(MeshReconstruction *map, LowDetailRenderer* low_detail_renderer, string file_path) {
	vector<GpuCoarseVertex> vertices;
	vector<int> indices;
	low_detail_renderer->downloadCurrentGeometry(vertices, indices);

	aiScene scene;

	scene.mRootNode = new aiNode();

	scene.mMaterials = new aiMaterial * [1];
	scene.mMaterials[0] = nullptr;
	scene.mNumMaterials = 1;
	scene.mMaterials[0] = new aiMaterial();

	scene.mMeshes = new aiMesh * [1];
	scene.mMeshes[0] = nullptr;
	scene.mNumMeshes = 1;
	scene.mMeshes[0] = new aiMesh();
	scene.mMeshes[0]->mMaterialIndex = 0;

	scene.mRootNode->mMeshes = new unsigned int[1];
	scene.mRootNode->mMeshes[0] = 0;
	scene.mRootNode->mNumMeshes = 1;

	auto p_mesh = scene.mMeshes[0];
	//lets just try a quad
	p_mesh->mVertices = new aiVector3D[vertices.size()];
	p_mesh->mNumVertices = vertices.size();
	p_mesh->mTextureCoords[0] = new aiVector3D[vertices.size()];
	p_mesh->mNumUVComponents[0] = vertices.size();

	for(size_t i = 0; i < vertices.size(); i++) {
		aiVector3D vec;
		vec.x = vertices[i].p[0];
		vec.y = vertices[i].p[1];
		vec.z = vertices[i].p[2];
		p_mesh->mVertices[i] = vec;
		vec.x = vec.x * 0.5f + 0.5f;
		vec.y = vec.y * 0.5f + 0.5f;
		vec.z = vec.z * 0.5f + 0.5f;
		p_mesh->mTextureCoords[0][i] = vec;
	}

	p_mesh->mFaces = new aiFace[indices.size() / 3];
	p_mesh->mNumFaces = indices.size() / 3;

	for(size_t i = 0; i < indices.size() / 3; i++) {
		aiFace face;
		face.mIndices    = new unsigned int[3];
		face.mNumIndices = 3;
		face.mIndices[0] = indices[i * 3 + 0];
		face.mIndices[1] = indices[i * 3 + 1];
		face.mIndices[2] = indices[i * 3 + 2];
		p_mesh->mFaces[i] = face;
	}

	Assimp::Exporter exporter;
	aiReturn result = exporter.Export(&scene, "ply", file_path.c_str());
	if(result == aiReturn_SUCCESS) {
		cout << "file stored in " << file_path << endl;
	} else if(result == aiReturn_OUTOFMEMORY) {
		cout << "storing file " << file_path << 
		        " failed due to running out of memory" << endl;
	} else if(result == aiReturn_FAILURE) {
		cout << "storing file " << file_path << " failed" << 
		        exporter.GetErrorString() << endl;
	}
}

void MapExporter::storeFine(MeshReconstruction *map, string file_path) {

	//todo: iterate over all patches download their data from gpu to cpu

	set<shared_ptr<DoubleStitch>> double_stitches;
	set<shared_ptr<TripleStitch>> triple_stitches;

	set<Triangle*> triangles;//don't think it will be needed
	vector<Vertex> vertices;
	vector<unsigned int> indices;
	std::map<int, unsigned int> start_indices;

	size_t triangle_count = 0;
	size_t vertex_count = 0;
	map->patches_mutex_.lock();
	for(pair<int, shared_ptr<MeshPatch>> id_patch : map->patches_) {//of course auto would be valid here as well
		shared_ptr<MeshPatch> patch = id_patch.second;
		start_indices[id_patch.first] = vertex_count;
		triangle_count += patch->triangles.size();
		vertex_count += patch->vertices.size();
		for(size_t i = 0; i < patch->double_stitches.size(); i++) {
			double_stitches.insert(patch->double_stitches[i]);
		}
		for(size_t i = 0; i < patch->triple_stitches.size(); i++) {
			triple_stitches.insert(patch->triple_stitches[i]);
		}
		//download the most current vertices and so on
	}
	for(auto double_stitch : double_stitches) {
		triangle_count += double_stitch->triangles.size();
	}
	for(auto triple_stitch : triple_stitches) {
		triangle_count += triple_stitch->triangles.size();
	}
	//how to store

	map->patches_mutex_.unlock();

	aiScene scene;

	scene.mRootNode = new aiNode();

	scene.mMaterials = new aiMaterial * [1];
	scene.mMaterials[0] = nullptr;
	scene.mNumMaterials = 1;
	scene.mMaterials[0] = new aiMaterial();

	scene.mMeshes = new aiMesh * [1];
	scene.mMeshes[0] = nullptr;
	scene.mNumMeshes = 1;
	scene.mMeshes[0] = new aiMesh();
	scene.mMeshes[0]->mMaterialIndex = 0;

	scene.mRootNode->mMeshes = new unsigned int[1];
	scene.mRootNode->mMeshes[0] = 0;
	scene.mRootNode->mNumMeshes = 1;
	auto p_mesh = scene.mMeshes[0];
	//lets just try a quad
	p_mesh->mVertices = new aiVector3D[vertex_count];
	p_mesh->mNumVertices = vertex_count;
	p_mesh->mTextureCoords[0] = new aiVector3D[vertex_count];

	p_mesh->mFaces = new aiFace[triangle_count];
	p_mesh->mNumFaces = triangle_count;

	size_t j = 0;
	size_t k = 0;
	auto appendTrianglesAsFaces = [&k, &start_indices, &p_mesh] (
			vector<Triangle> triangles) {
		for(Triangle &triangle : triangles) {
			aiFace face;
			face.mIndices = new unsigned int[3];
			face.mNumIndices = 3;
			for(size_t l = 0; l < 3; l++) {
				unsigned int offset = start_indices[triangle.points[l].getPatch()->id];
				unsigned int index = triangle.points[l].getIndex() + offset;
				face.mIndices[l] = index;
			}
			p_mesh->mFaces[k]=face;
			k++;
		}
	};

	for(pair<int, shared_ptr<MeshPatch>> id_patch : map->patches_) {
		shared_ptr<MeshPatch> patch = id_patch.second;
		for(int i = 0 ; i < patch->vertices.size(); i++) {
			aiVector3D vec;
			vec.x = patch->vertices[i].p[0];
			vec.y = patch->vertices[i].p[1];
			vec.z = patch->vertices[i].p[2];
			p_mesh->mVertices[j] = vec;
			vec.x = vec.x * 0.5f + 0.5f;
			vec.y = vec.y * 0.5f + 0.5f;
			vec.z = vec.z * 0.5f + 0.5f;
			p_mesh->mTextureCoords[0][j] = vec;
			j++;
		}
		appendTrianglesAsFaces(patch->triangles);
	}

	auto appendFaces = [&k, &start_indices, &p_mesh, &appendTrianglesAsFaces] (
			set<shared_ptr<GeometryBase>> &geoms) {
		for(auto geom : geoms) {
			appendTrianglesAsFaces(geom->triangles);
		}
	};
	shared_ptr<GeometryBase> base = *(double_stitches.begin());
	appendFaces(reinterpret_cast<set<shared_ptr<GeometryBase>>&>(double_stitches));
	appendFaces(reinterpret_cast<set<shared_ptr<GeometryBase>>&>(triple_stitches));

	Assimp::Exporter exporter;
	aiReturn result = exporter.Export(&scene, "ply", file_path.c_str());
	if(result == aiReturn_SUCCESS) {
		cout << "file stored in " << file_path << endl;
	}else if(result == aiReturn_OUTOFMEMORY) {
		cout << "storing file " << file_path << 
		        " failed due to running out of memory" << endl;
	}else if(result == aiReturn_FAILURE) {
		cout << "storing file " << file_path << " failed" << 
		        exporter.GetErrorString() << endl;
	}
}

void MapExporter::exportMapTest(string file_path) {
	cout << "Assimp " << aiGetVersionMajor() << "." << aiGetVersionMinor() << endl;

	int format_count = aiGetExportFormatCount();
	const aiExportFormatDesc *chosen_format;
	for(int i = 0; i < format_count; i++) {
		const aiExportFormatDesc *desc = aiGetExportFormatDescription(i);
		cout << desc->description << endl;
		cout << desc->fileExtension << endl;
		cout << desc->id << endl;
	}

	cout << "Testing the storage of models" << endl;
	//https://github.com/assimp/assimp/issues/203
	aiScene scene;

	scene.mRootNode = new aiNode();

	scene.mMaterials = new aiMaterial * [1];
	scene.mMaterials[0] = nullptr;
	scene.mNumMaterials = 1;
	scene.mMaterials[ 0 ] = new aiMaterial();

	scene.mMeshes = new aiMesh * [1];
	scene.mMeshes[0] = nullptr;
	scene.mNumMeshes = 1;
	scene.mMeshes[0] = new aiMesh();
	scene.mMeshes[0]->mMaterialIndex = 0;

	scene.mRootNode->mMeshes = new unsigned int[1];
	scene.mRootNode->mMeshes[0] = 0;
	scene.mRootNode->mNumMeshes = 1;

	auto p_mesh = scene.mMeshes[0];
	//lets just try a quad
	p_mesh->mVertices = new aiVector3D[4];
	p_mesh->mNumVertices = 4;
	p_mesh->mTextureCoords[0] = new aiVector3D[4];

	p_mesh->mNumUVComponents[0] = 4;

	Vector3f points[] = {Vector3f(-1, -1, 0),
	                     Vector3f( 1, -1, 0),
	                     Vector3f( 1,  1, 0),
	                     Vector3f(-1,  1, 0)};
	for(size_t i = 0; i < 4; i++) {
		aiVector3D vec;
		vec.x = points[i][0];
		vec.y = points[i][1];
		vec.z = points[i][2];
		p_mesh->mVertices[i] = vec;
		vec.x = vec.x * 0.5f + 0.5f;
		vec.y = vec.y * 0.5f + 0.5f;
		vec.z = vec.z * 0.5f + 0.5f;
		p_mesh->mTextureCoords[0][i] = vec;
	}

	p_mesh->mFaces = new aiFace[2];
	p_mesh->mNumFaces = 2;

	aiFace face;
	face.mIndices = new unsigned int[3];
	face.mNumIndices = 3;
	face.mIndices[0] = 0;
	face.mIndices[1] = 1;
	face.mIndices[2] = 2;
	p_mesh->mFaces[0] = face;

	//we need to reserve a new piece of memory for the next face!
	face.mIndices = new unsigned int[3];
	face.mIndices[0] = 0;
	face.mIndices[1] = 2;
	face.mIndices[2] = 3;
	p_mesh->mFaces[1] = face;
	Assimp::Exporter exporter;
	exporter.Export(&scene, "ply", file_path.c_str());
	cout << "file stored in " << file_path << endl;
}

void MapExporter::storeGraph(MeshReconstruction *map, string file_path) {
	unordered_map<MeshPatch*, int> index_map;
	unordered_set<shared_ptr<DoubleStitch>> unique_stitches;
	int k = 0;
	for(pair<int, shared_ptr<MeshPatch>> id_patch : map->patches_) {//of course auto would be valid here as well
		shared_ptr<MeshPatch> patch = id_patch.second;
		index_map[patch.get()] = k;
		k++;
		for(size_t i = 0; i < patch->double_stitches.size(); i++) {
			unique_stitches.insert(patch->double_stitches[i]);
		}
	}

	ofstream file;
	file.open(file_path);
	file << index_map.size() << endl;
	file << unique_stitches.size() << endl;
	for(auto patch : map->patches_) {
		Vector3f center = patch.second->getPos();
		file << center[0] << " " << center[1] << " " << center[2]  << endl;
	}
	for(auto stitch: unique_stitches) {
		int index1 = index_map[stitch->patches[0].lock().get()];
		int index2 = index_map[stitch->patches[1].lock().get()];
		file << index1 << " " << index2 << endl;
	}
	//magic in between
	file.close();
}

void MapExporter::storeDeformationGraph(MeshReconstruction *map,
										string file_path) {

	unordered_map<MeshPatch*, unordered_set<MeshPatch*>> unique_edges;
	unordered_map<MeshPatch*, int> index_map;
	int k = 0;
	for(pair<int, shared_ptr<MeshPatch>> id_patch : map->patches_) {//of course auto would be valid here as well
		shared_ptr<MeshPatch> patch = id_patch.second;
		index_map[patch.get()] = k;
		k++;
		for(int i = 0; i < 4; i++) {
			for(pair<float, DeformationNode::WeakNodeDist> neighbour : 
			    patch->deformation_node->neighbours[i]) {
				MeshPatch *p1 = patch.get();
				if(neighbour.second.node.lock() != nullptr) {
					MeshPatch *p2 = neighbour.second.node.lock()->patch;
					if(p1 == p2) {
						assert(0);//this should not happen!!!! really!!!
						continue;
					}
					if(p1 > p2) {
						swap(p1, p2);
					}
					if(unique_edges.count(p1) == 0) {
						unordered_set<MeshPatch*> set;
						unique_edges[p1] = set;
					}
					unique_edges[p1].insert(p2);
				}
			}
		}
	}

	int size = 0;
	for(auto one : unique_edges) {
		for(auto two : one.second) {
			size ++;
		}
	}
	ofstream file;
	file.open(file_path);
	file << index_map.size() << endl;
	file << size << endl;
	for(auto patch : map->patches_) {
		Vector3f center = patch.second->getPos();
		file << center[0] << " " << center[1] << " " << center[2]  << endl;
	}

	for(auto one : unique_edges) {
		for(auto two : one.second) {
			file << index_map[one.first] << " " << index_map[two] << endl;
		}
	}
	//magic in between
	file.close();
}
