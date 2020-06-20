#include <export/map_exporter.h>

#include <unordered_map>
#include <fstream>

#include <assimp/Exporter.hpp>
#include <assimp/scene.h>           // Output data structure
#include <assimp/version.h>
#include <Eigen/Core>

#include <graph/deformation_node.h>
#include <mesh_reconstruction.h>

#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace Eigen;

/*
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
*/
void MapExporter::storeFine(MeshReconstruction *map, string file_path, bool fix_ply_meshlab, bool swap_rb) {


	set<Triangle*> triangles;//don't think it will be needed
	vector<Vertex> vertices;
	vector<unsigned int> indices;
	std::map<int, unsigned int> start_indices;

	size_t triangle_count = 0;
	size_t vertex_count = 0;
	map->meshlet_mutex_.lock();
	std::unordered_map<Vertex*,int> vertex_indices;
	for(pair<int, shared_ptr<Meshlet>> id_patch : map->meshlets_) {//of course auto would be valid here as well
		shared_ptr<Meshlet> meshlet = id_patch.second;
		for(Triangle &triangle : meshlet->triangles){
			triangle_count++;
			for(int i : {0, 1, 2}){
				Vertex* vert = triangle.vertices[i];
				if(!vertex_indices.count(vert)){
					vertex_indices[vert] = vertex_count;
					vertex_count++;
				}

			}
		}
	}

	map->meshlet_mutex_.unlock();

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
	p_mesh->mColors[0] = new aiColor4D[vertex_count];

	p_mesh->mFaces = new aiFace[triangle_count];
	p_mesh->mNumFaces = triangle_count;


	size_t face_iter = 0;
	for(pair<int, shared_ptr<Meshlet>> id_patch : map->meshlets_) {
		shared_ptr<Meshlet> meshlet = id_patch.second;

		for(Triangle &triangle : meshlet->triangles){
			aiFace face;
			face.mIndices = new unsigned int[3];
			face.mNumIndices = 3;
			for(size_t i : {0, 1, 2}){
				face.mIndices[i] =
						vertex_indices[triangle.vertices[i]];
			}
			p_mesh->mFaces[face_iter] = face;
			face_iter++;
		}
	}
	for(pair<Vertex*, int> vert_ind : vertex_indices){
		Vertex* vert = vert_ind.first;
		int ind = vert_ind.second;
		aiVector3D vec;
		vec.x = vert->p[0];
		vec.y = vert->p[1];
		vec.z = vert->p[2];
		p_mesh->mVertices[ind] = vec;
		vec.x = vec.x * 0.5f + 0.5f;
		vec.y = vec.y * 0.5f + 0.5f;
		vec.z = vec.z * 0.5f + 0.5f;
		p_mesh->mTextureCoords[0][ind] = vec;
		aiColor4D c;
		c.r = vert->color[0];
		c.g = vert->color[1];
		c.b = vert->color[2];
		if(swap_rb){
		    std::swap(c.r, c.b);
		}
		c.a = vert->color[3];
		p_mesh->mColors[0][ind] = c;

	}


	if(!fix_ply_meshlab){
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
	}else{
		export_scene_ply_meshlab(&scene, file_path);
	}
}

void MapExporter::export_scene_ply_meshlab(aiScene *scene, string path) {
	ofstream file(path, std::ios::out);

	if(file.is_open())
	{
		file << "ply \n"
				"format ascii 1.0\n"
				"comment Only for you Edith! I hope you aknowledge my effort of writing the dirties ply exporter ever!"
				<< endl;
		file << "element vertex " << scene->mMeshes[0]->mNumVertices << endl;
		file << "property float x\n"
				"property float y\n"
				"property float z\n"
				"property float s\n"
				"property float t\n"
				"property uchar red\n"
				"property uchar green\n"
				"property uchar blue\n"
				"property uchar alpha" << endl;
		file << "element face " << scene->mMeshes[0]->mNumFaces << endl;
		file << "property list uchar int vertex_index \n end_header" << endl;

		for(int i=0;i<scene->mMeshes[0]->mNumVertices;i++){
			int r, g, b, a;
			r = 255 * scene->mMeshes[0]->mColors[0][i].r;
			g = 255 * scene->mMeshes[0]->mColors[0][i].g;
			b = 255 * scene->mMeshes[0]->mColors[0][i].b;
			a = 255 * scene->mMeshes[0]->mColors[0][i].a;
			file << scene->mMeshes[0]->mVertices[i].x << " " <<
					scene->mMeshes[0]->mVertices[i].y << " " <<
					scene->mMeshes[0]->mVertices[i].z << " " <<
					scene->mMeshes[0]->mTextureCoords[0][i].x << " " <<
					scene->mMeshes[0]->mTextureCoords[0][i].y << " " <<
					r << " " << g << " " << b << " " << a << endl;
		}

		for(int i=0;i<scene->mMeshes[0]->mNumFaces;i++){
			file << "3 " << scene->mMeshes[0]->mFaces[i].mIndices[0] << " " <<
							scene->mMeshes[0]->mFaces[i].mIndices[1] << " " <<
							scene->mMeshes[0]->mFaces[i].mIndices[2] << endl;

		}
	}
	else cerr<<"Unable to open file";


}
/*
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
	unordered_map<Meshlet*, int> index_map;
	unordered_set<shared_ptr<DoubleStitch>> unique_stitches;
	int k = 0;
	for(pair<int, shared_ptr<Meshlet>> id_patch : map->meshlets_) {//of course auto would be valid here as well
		shared_ptr<Meshlet> patch = id_patch.second;
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
	for(auto patch : map->meshlets_) {
		Vector3f center = patch.second->center();
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

	unordered_map<Meshlet*, unordered_set<Meshlet*>> unique_edges;
	unordered_map<Meshlet*, int> index_map;
	int k = 0;
	for(pair<int, shared_ptr<Meshlet>> id_patch : map->meshlets_) {//of course auto would be valid here as well
		shared_ptr<Meshlet> patch = id_patch.second;
		index_map[patch.get()] = k;
		k++;
		for(int i = 0; i < 4; i++) {
			for(pair<float, DeformationNode::WeakNodeDist> neighbour : 
			    patch->deformation_node->neighbours[i]) {
				Meshlet *p1 = patch.get();
				if(neighbour.second.node.lock() != nullptr) {
					Meshlet *p2 = neighbour.second.node.lock()->patch;
					if(p1 == p2) {
						assert(0);//this should not happen!!!! really!!!
						continue;
					}
					if(p1 > p2) {
						swap(p1, p2);
					}
					if(unique_edges.count(p1) == 0) {
						unordered_set<Meshlet*> set;
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
	for(auto patch : map->meshlets_) {
		Vector3f center = patch.second->center();
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
*/