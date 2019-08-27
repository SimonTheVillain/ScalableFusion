#include "exportMap.h"
#include "../meshReconstruction.h"

#include <Eigen/Core>

#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>
#include <assimp/version.h>

#include <map>//TODO: remove since it should be included already
#include <unordered_map>
#include <fstream>

#include "../graph/DeformationNode.h"

using namespace std;
using namespace Eigen;



void Exporter::ExportMap(MeshReconstruction *map,std::string path,unsigned int properties){

    //lets start with the low detail stuff:
    //map->lowDetailRenderer.coarseTriangles

}

//only HighRes or LowRes is valid
void Exporter::ExportMesh(MeshReconstruction *map,std::string path,Exporter::Properties properties){


}



void Exporter::storeCoarse(MeshReconstruction* map,std::string filePath){
    std::vector<GpuCoarseVertex> vertices;
    std::vector<int> indices;
    map->lowDetailRenderer.downloadCurrentGeometry(vertices,indices);


    aiScene scene;

    scene.mRootNode = new aiNode();

    scene.mMaterials = new aiMaterial*[ 1 ];
    scene.mMaterials[ 0 ] = nullptr;
    scene.mNumMaterials = 1;

    scene.mMaterials[ 0 ] = new aiMaterial();


    scene.mMeshes = new aiMesh*[ 1 ];
    scene.mMeshes[ 0 ] = nullptr;
    scene.mNumMeshes = 1;


    scene.mMeshes[ 0 ] = new aiMesh();
    scene.mMeshes[ 0 ]->mMaterialIndex = 0;



    scene.mRootNode->mMeshes = new unsigned int[ 1 ];
    scene.mRootNode->mMeshes[ 0 ] = 0;
    scene.mRootNode->mNumMeshes = 1;



    auto pMesh = scene.mMeshes[ 0 ];
    //lets just try a quad
    pMesh->mVertices = new aiVector3D[vertices.size()];
    pMesh->mNumVertices = vertices.size();
    pMesh->mTextureCoords[0] = new aiVector3D[vertices.size()];


    pMesh->mNumUVComponents[0] = vertices.size();

    for(size_t i=0;i<vertices.size();i++){
        aiVector3D vec;
        vec.x = vertices[i].p[0];
        vec.y = vertices[i].p[1];
        vec.z = vertices[i].p[2];
        pMesh->mVertices[i] = vec;
        vec.x =vec.x*0.5f+0.5f;
        vec.y =vec.y*0.5f+0.5f;
        vec.z =vec.z*0.5f+0.5f;
        pMesh->mTextureCoords[0][i]=vec;
    }


    pMesh->mFaces = new aiFace[ indices.size()/3 ];
    pMesh->mNumFaces = indices.size()/3 ;


    for(size_t i=0;i<indices.size()/3;i++){

        aiFace face;
        face.mIndices = new unsigned int[ 3 ];
        face.mNumIndices = 3;
        face.mIndices[0]=indices[i*3 + 0];
        face.mIndices[1]=indices[i*3 + 1];
        face.mIndices[2]=indices[i*3 + 2];
        pMesh->mFaces[i]=face;
    }

    Assimp::Exporter exporter;
    aiReturn result = exporter.Export(&scene,"ply",filePath.c_str());
    if(result == aiReturn_SUCCESS){
        cout << "file stored in " << filePath << endl;
    }else if(result == aiReturn_OUTOFMEMORY){
        cout << "storing file " << filePath << " failed due to running out of memory" << endl;
    }else if(result == aiReturn_FAILURE){
        cout << "storing file " << filePath << " failed" << exporter.GetErrorString() << endl;
    }




}
void Exporter::storeFine(MeshReconstruction* map,std::string filePath){

    //todo: iterate over all patches download their data from gpu to cpu

    set<shared_ptr<DoubleStitch>> doubleStitches;
    set<shared_ptr<TripleStitch>> tripleStitches;

    set<Triangle*> triangles;//don't think it will be needed
    vector<Vertex> vertices;
    vector<unsigned int> indices;
    std::map<int,unsigned int> startIndices;


    size_t triangleCount = 0;
    size_t vertexCount = 0;
    map->m_patchesMutex.lock();
    for(pair<int,shared_ptr<MeshPatch>> idPatch : map->m_patches){//of course auto would be valid here as well
        shared_ptr<MeshPatch> patch = idPatch.second;
        startIndices[idPatch.first] = vertexCount;
        triangleCount += patch->triangles.size();
        vertexCount += patch->vertices.size();
        for(size_t i = 0; i < patch->doubleStitches.size();i++){
            doubleStitches.insert(patch->doubleStitches[i]);
        }
        for(size_t i = 0; i < patch->tripleStitches.size();i++){
            tripleStitches.insert(patch->tripleStitches[i]);
        }


        //download the most current vertices and so on

    }
    for(auto doubleStitch : doubleStitches){
        triangleCount += doubleStitch->triangles.size();
    }
    for(auto tripleStitch : tripleStitches){
        triangleCount += tripleStitch->triangles.size();
    }
    //how to store



    map->m_patchesMutex.unlock();


    aiScene scene;

    scene.mRootNode = new aiNode();

    scene.mMaterials = new aiMaterial*[ 1 ];
    scene.mMaterials[ 0 ] = nullptr;
    scene.mNumMaterials = 1;

    scene.mMaterials[ 0 ] = new aiMaterial();


    scene.mMeshes = new aiMesh*[ 1 ];
    scene.mMeshes[ 0 ] = nullptr;
    scene.mNumMeshes = 1;


    scene.mMeshes[ 0 ] = new aiMesh();
    scene.mMeshes[ 0 ]->mMaterialIndex = 0;



    scene.mRootNode->mMeshes = new unsigned int[ 1 ];
    scene.mRootNode->mMeshes[ 0 ] = 0;
    scene.mRootNode->mNumMeshes = 1;


    auto pMesh = scene.mMeshes[ 0 ];
    //lets just try a quad
    pMesh->mVertices = new aiVector3D[vertexCount];
    pMesh->mNumVertices = vertexCount;
    pMesh->mTextureCoords[0] = new aiVector3D[vertexCount];




    pMesh->mFaces = new aiFace[ triangleCount ];
    pMesh->mNumFaces = triangleCount;




    size_t j=0;
    size_t k=0;
    auto appendTrianglesAsFaces = [&k,&startIndices,&pMesh] (vector<Triangle> triangles){
        for(Triangle &triangle : triangles){
            aiFace face;
            face.mIndices = new unsigned int[ 3 ];
            face.mNumIndices = 3;
            for(size_t l = 0;l<3;l++){
                unsigned int offset = startIndices[triangle.points[l].getPatch()->id];
                unsigned int index = triangle.points[l].getIndex() + offset;
                face.mIndices[l] = index;
            }
            pMesh->mFaces[k]=face;
            k++;
        }
    };


    for(pair<int,shared_ptr<MeshPatch>> idPatch : map->m_patches){
        shared_ptr<MeshPatch> patch = idPatch.second;

        for(int i = 0 ; i < patch->vertices.size();i++){

            aiVector3D vec;
            vec.x = patch->vertices[i].p[0];
            vec.y = patch->vertices[i].p[1];
            vec.z = patch->vertices[i].p[2];
            pMesh->mVertices[j] = vec;
            vec.x =vec.x*0.5f+0.5f;
            vec.y =vec.y*0.5f+0.5f;
            vec.z =vec.z*0.5f+0.5f;
            pMesh->mTextureCoords[0][j]=vec;
            j++;
        }

        appendTrianglesAsFaces(patch->triangles);
    }
    auto appendFaces = [&k,&startIndices,&pMesh,&appendTrianglesAsFaces] (set<shared_ptr<GeometryBase>> &geoms) {
        for(auto geom : geoms){
            appendTrianglesAsFaces(geom->triangles);
        }
    };
    std::shared_ptr<GeometryBase> base = *(doubleStitches.begin());
    appendFaces(reinterpret_cast<set<shared_ptr<GeometryBase>>&>(doubleStitches));
    appendFaces(reinterpret_cast<set<shared_ptr<GeometryBase>>&>(tripleStitches));



    Assimp::Exporter exporter;
    aiReturn result = exporter.Export(&scene,"ply",filePath.c_str());
    if(result == aiReturn_SUCCESS){
        cout << "file stored in " << filePath << endl;
    }else if(result == aiReturn_OUTOFMEMORY){
        cout << "storing file " << filePath << " failed due to running out of memory" << endl;
    }else if(result == aiReturn_FAILURE) {
        cout << "storing file " << filePath << " failed" << exporter.GetErrorString() << endl;
    }



}

void Exporter::ExportMapTest(std::string filePath)
{
    cout << "Assimp " << aiGetVersionMajor() << "." << aiGetVersionMinor() << endl;

    int formatCount = aiGetExportFormatCount();
    const aiExportFormatDesc* chosenFormat;
    for(int i=0;i<formatCount;i++){
        const aiExportFormatDesc * desc = aiGetExportFormatDescription(i);
        cout << desc->description << endl;
        cout << desc->fileExtension << endl;
        cout << desc->id << endl;
    }

    cout << "Testing the storage of models" << endl;
    //https://github.com/assimp/assimp/issues/203
    aiScene scene;

    scene.mRootNode = new aiNode();

    scene.mMaterials = new aiMaterial*[ 1 ];
    scene.mMaterials[ 0 ] = nullptr;
    scene.mNumMaterials = 1;

    scene.mMaterials[ 0 ] = new aiMaterial();


    scene.mMeshes = new aiMesh*[ 1 ];
    scene.mMeshes[ 0 ] = nullptr;
    scene.mNumMeshes = 1;

    scene.mMeshes[ 0 ] = new aiMesh();
    scene.mMeshes[ 0 ]->mMaterialIndex = 0;


    scene.mRootNode->mMeshes = new unsigned int[ 1 ];
    scene.mRootNode->mMeshes[ 0 ] = 0;
    scene.mRootNode->mNumMeshes = 1;


    auto pMesh = scene.mMeshes[ 0 ];
    //lets just try a quad
    pMesh->mVertices = new aiVector3D[4];
    pMesh->mNumVertices = 4;
    pMesh->mTextureCoords[0] = new aiVector3D[4];

    pMesh->mNumUVComponents[0] = 4;

    Vector3f points[] = {
            Vector3f(-1,-1,0),
            Vector3f(1,-1,0),
            Vector3f(1,1,0),
            Vector3f(-1,1,0)
    };
    for(size_t i=0;i<4;i++){
        aiVector3D vec;
        vec.x = points[i][0];
        vec.y = points[i][1];
        vec.z = points[i][2];
        pMesh->mVertices[i] = vec;
        vec.x =vec.x*0.5f+0.5f;
        vec.y =vec.y*0.5f+0.5f;
        vec.z =vec.z*0.5f+0.5f;
        pMesh->mTextureCoords[0][i]=vec;
    }

    pMesh->mFaces = new aiFace[ 2 ];
    pMesh->mNumFaces = 2;

    aiFace face;
    face.mIndices = new unsigned int[ 3 ];
    face.mNumIndices = 3;
    face.mIndices[0]=0;
    face.mIndices[1]=1;
    face.mIndices[2]=2;
    pMesh->mFaces[0]=face;

    //we need to reserve a new piece of memory for the next face!
    face.mIndices = new unsigned int[ 3 ];
    face.mIndices[0]=0;
    face.mIndices[1]=2;
    face.mIndices[2]=3;
    pMesh->mFaces[1]=face;
    Assimp::Exporter exporter;
    exporter.Export(&scene,"ply",filePath.c_str());
    cout << "file stored in " << filePath << endl;


}

void Exporter::storeGraph(MeshReconstruction *map, std::string filePath) {
    unordered_map<MeshPatch*,int> indexMap;
    unordered_set<shared_ptr<DoubleStitch>> uniqueStitches;
    int k = 0;
    for(pair<int,shared_ptr<MeshPatch>> idPatch : map->m_patches){//of course auto would be valid here as well
        shared_ptr<MeshPatch> patch = idPatch.second;
        indexMap[patch.get()] = k;
        k++;
        for(size_t i=0;i<patch->doubleStitches.size();i++){
            uniqueStitches.insert(patch->doubleStitches[i]);
        }

    }

    ofstream file;
    file.open(filePath);
    file << indexMap.size() << endl;
    file << uniqueStitches.size() << endl;
    for(auto patch : map->m_patches){
        Vector3f center = patch.second->getPos();
        file << center[0] << " " << center[1] << " " << center[2]  << endl;
    }

    for(auto stitch: uniqueStitches){
        int index1 = indexMap[stitch->patches[0].lock().get()];
        int index2 = indexMap[stitch->patches[1].lock().get()];
        file << index1 << " " << index2 << endl;
    }
    //magic in between
    file.close();

}

void Exporter::storeDeformationGraph(MeshReconstruction *map, std::string filePath) {
    struct edge{
        MeshPatch* patch1;
        MeshPatch* patch2;
    };
    unordered_map<MeshPatch*,unordered_set<MeshPatch*>> uniqueEdges;
    unordered_map<MeshPatch*,int> indexMap;
    int k = 0;
    for(pair<int,shared_ptr<MeshPatch>> idPatch : map->m_patches){//of course auto would be valid here as well
        shared_ptr<MeshPatch> patch = idPatch.second;
        indexMap[patch.get()] = k;
        k++;
        for(int i=0;i<4;i++){
            for( pair<float,DeformationNode::WeakNodeDist> neighbour : patch->deformationNode->neighbours[i]){
                MeshPatch* p1 = patch.get();
                if(neighbour.second.node.lock()!=nullptr){
                    MeshPatch* p2 = neighbour.second.node.lock()->patch;
                    if(p1 == p2){
                        assert(0);//this should not happen!!!! really!!!
                        continue;
                    }
                    if(p1>p2){
                        swap(p1,p2);
                    }
                    if(uniqueEdges.count(p1) == 0){
                        unordered_set<MeshPatch*> set;
                        uniqueEdges[p1] = set;
                    }
                    uniqueEdges[p1].insert(p2);


                }
            }

        }


    }

    int size = 0;
    for(auto one : uniqueEdges){
        for(auto two : one.second){
            size ++;
        }
    }
    ofstream file;
    file.open(filePath);
    file << indexMap.size() << endl;
    file << size << endl;
    for(auto patch : map->m_patches){
        Vector3f center = patch.second->getPos();
        file << center[0] << " " << center[1] << " " << center[2]  << endl;
    }

    for(auto one : uniqueEdges){
        for(auto two : one.second){
            file << indexMap[one.first] << " " << indexMap[two] << endl;
        }
    }
    //magic in between
    file.close();

}