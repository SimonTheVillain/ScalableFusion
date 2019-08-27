#ifndef FILE_MESH_STRUCTURE
#define FILE_MESH_STRUCTURE

#include "textureStructure.h"
#include <eigen3/Eigen/Core> ///TODO: Change this!!!!! this is baaaad!!!
#include <list>
#include <vector>
#include <mutex>
//#include <texture.h>

#include <gpuTex.h>

#include <iostream>
#include <memory>
#include <set>
#include "stackVector.h"

//i included this for deleting stuff
#include <algorithm>
#include <gpuGeomStorage.h>
#include "octree.h"

#include "lowDetailMapRenderer.h"
#include "gpuBuffer.h"

//#include "../graph/DeformationNode.h"

//using namespace std;
//using namespace Eigen;


/**
 * REMEMBER: This structure has to be done in a way that even when we remove one point, the
 * remaining structure should not autodelete. I don't know what the mechanism of deleting
 * points should be....????
 * The use of shared_ptr is costly. especially for points that might be accessed often like
 * the most primitive elements. (points, triangles, edges)
 * THIS IS CRITICAL!!!!! It would increase the performance a lot if we reduce the use of smart pointers!!!
 */


struct DoubleStitch;
struct TripleStitch;

class TexAtlas;

class MeshReconstruction;

struct Triangle;
struct Edge;
class DeformationNode;





/**
 * this is the class that handles how many
 * Patches, DoubleStitches or TripleStitches are used by a certain thread right now.
 */
class ActiveSet;



struct TriangleReference{
    GeometryBase* container = nullptr;
    int index = -1;

    bool valid(){
        if(container == nullptr || index == -1){
            return false;
        }
        return true;
    }

    bool equalTo(const TriangleReference& other){
        return container == other.container && index == other.index;
    }
    Triangle* get();
    TriangleReference(){

    }
    TriangleReference(GeometryBase* geometryBase,int ind): container(geometryBase),index(ind)
    {

    }
    ~TriangleReference(){

    }
};

//TODO: Maybe let this vertex structure also replace the gpuvertex
struct Vertex{
    Eigen::Vector4f p;
    Eigen::Vector3f n;//TODO: start using this at some point
    int32_t texIndInMainPatch = -1;
    GpuVertex genGpuVertex(){
        GpuVertex vert;
        vert.p=p;
        vert.n=n;
        vert.texIndInMainPatch = static_cast<int16_t>(texIndInMainPatch);
        return vert;
    }
    Vertex(){}
    Vertex(GpuVertex gpuVertex){
        p = gpuVertex.p;
        n = gpuVertex.n;
        texIndInMainPatch = gpuVertex.texIndInMainPatch;
    }

    //https://en.cppreference.com/w/cpp/language/operators
    //meeh copy and move assignment
    /*
    Vertex& operator=(const GpuVertex& gpuVertex){
        p = gpuVertex.p;
        n = gpuVertex.n;
        texIndInMainPatch = gpuVertex.texIndInMainPatch;
        return *this;
    }
     */



    struct VertexInTriangle{
        TriangleReference triangle;
        int indInTriangle = -1;
        //bool debugIsStitch = false;
    };

    //std::vector<VertexInTriangle> triangles;
    stack_vector<VertexInTriangle,16> triangles; //this will not really give any speed benefits (a few milliseconds at most)

    void removeTriangle(Triangle* triangle){
        for(size_t i=0;i<triangles.size();i++){
            if(triangles[i].triangle.get() == triangle){
                triangles[i] = triangles.back();
                triangles.pop_back();
                return;
            }
        }
        //ending up here means that the triangle probably was not properly registered
        assert(0);
    }
    void removeTriangle(int pos){
        triangles[pos] = triangles.back();
        triangles.pop_back();
    }


    /**
     * Checks if the vertex is completely encompassed by triangles.
     * @return
     */
    bool encompassed();
};


/**
 * @brief The PointReference struct
 * stores a reference to a vertex.
 * By having the pointer to the mesh one could as well read out the information about
 * the specific vertex information. (the vertex information also contains references to
 * all the connected primitives)
 */
struct VertexReference{
private:
    MeshPatch* patch=nullptr;
    int index=-1;
public:

    void set(MeshPatch *p,int i){
        if(i<-1 || i>10000){
            assert(0);//something again is very wrong DEBUG
        }
        if(p==0){
            //assert(0);
        }
        index=i;
        patch=p;
    }
    MeshPatch* getPatch() const{

        return patch;
    }
    int getIndex() const{
        return index;
    }


    //TODO: create a proper operator you fool
    bool isEqualTo(VertexReference &theOther){
        if(patch==theOther.patch && index==theOther.index)
            return true;
        return false;
    }


    bool operator< (const VertexReference& a)const {
        if(patch<a.patch){
            return true;
        }else{
            if(patch>a.patch){
                return false;
            }else{
                return index<a.index;
            }
        }
    }



    Vertex* get() const;
};





/**
 * @brief The Triangle struct
 * stores references to 3 geometrical points and 3 texture coordinates.
 * It also stores pointers to up to three edges
 * TODO: maybe instead of having edges the triangles should store their up to 3 neighbours directly
 * Edges are only then necessary when it comes to stitching, therefore they should only be generated when needed.
 */
struct Triangle{
    bool registered = false;
    bool debugIsStitch = false;
    int debugNr=-1;
    //TODO: find out if this needs a self reference
    /*
    TriangleReference selfReference;
    TriangleReference &ref(){
        assert(selfReference.valid());
        return selfReference;
    }*/

    VertexReference points[3];

    bool containsPoint(VertexReference& p){
        for(uint32_t i : {0,1,2}){
            if(p.isEqualTo(points[i])){
                return true;
            }
        }
        return false;
    }
    int getPointIndex(VertexReference& p){

        for(uint32_t i : {0,1,2}){
            if(p.isEqualTo(points[i])){
                return i;
            }

        }
        return -1;
    }

    struct Neighbour{
        TriangleReference ref;
        bool valid(){
            return ref.valid();
        }
        //position in neighbour
        int pos = -1;

        bool debug = false;
        void invalidate(){
            pos = -1;
            ref = TriangleReference();
            //ref.container = nullptr;
            //ref.index = -1;
        }
    };

    Neighbour neighbours[3];


    struct EdgeReference{
        int borderInd = -1;
        int ind = -1;
        bool valid(){
            return borderInd!=-1 && ind!=-1;
        }
        void invalidate(){
            borderInd = -1;
            ind = -1;
        }
        Edge* get(std::vector<std::vector<Edge>> &list){
            assert(valid());
            return &list[borderInd][ind];
        }
    };
    EdgeReference edges[3];




    bool addNeighbour(TriangleReference triangle,VertexReference p1,VertexReference p2,
                      int pos){//position within the target triangle
        if((points[0].isEqualTo(p1) && points[1].isEqualTo(p2)) ||
           (points[1].isEqualTo(p1) && points[0].isEqualTo(p2))){
            if(neighbours[0].valid()){
                assert(0);//already has neighbour... adding a new one is forbidden
                //return false;
            }
            neighbours[0].ref=triangle;
            neighbours[0].pos = pos;

        }else if((points[1].isEqualTo(p1) && points[2].isEqualTo(p2)) ||
                 (points[2].isEqualTo(p1) && points[1].isEqualTo(p2))){
            if(neighbours[1].valid()){
                assert(0);
            }

            neighbours[1].ref=triangle;
            neighbours[1].pos = pos;

        }else if((points[0].isEqualTo(p1) && points[2].isEqualTo(p2)) ||
                 (points[2].isEqualTo(p1) && points[0].isEqualTo(p2))){
            if(neighbours[2].valid()){
                assert(0);
            }
            neighbours[2].ref=triangle;
            neighbours[2].pos = pos;
        }else{
            //none of the edges is fitting to add a
            //triangle as neighbour.... (the points don't match)
            //Calling this method should never occur?
            assert(0);
        }
        return true;
    }



    int getPosInTriangle(VertexReference p1,VertexReference p2){
        if(p1.isEqualTo(points[0]) && p2.isEqualTo(points[1]) ||
           p1.isEqualTo(points[1]) && p2.isEqualTo(points[0])){
            return 0;
        }
        if(p1.isEqualTo(points[2]) && p2.isEqualTo(points[1]) ||
           p1.isEqualTo(points[1]) && p2.isEqualTo(points[2])){
            return 1;
        }
        if(p1.isEqualTo(points[0]) && p2.isEqualTo(points[2]) ||
           p1.isEqualTo(points[2]) && p2.isEqualTo(points[0])){
            return 2;
        }
        assert(0);
        return -1;
    }

    /**
     * @brief registerToPoints
     * Registers a freshly created triangle to its neighbouring points (so the point variables have to be set already
     * @param triangleToRegister
     */
     //TODO: remove this debug
    static void registerTriangle(TriangleReference &triangleToRegister,bool searchEdges = true, bool debug = false);


    void cycle(int cnt){
        assert(!registered);
        assert(!neighbours[0].valid());
        assert(!neighbours[1].valid());
        assert(!neighbours[2].valid());
        if(cnt == 1){
            VertexReference p = points[0];
            points[0] = points[1];
            points[1] = points[2];
            points[2] = p;
        }else if(cnt == 2){
            VertexReference p = points[0];
            points[0] = points[2];
            points[2] = points[1];
            points[1] = p;

        }
    }


    void replacePatchReference(MeshPatch* from, MeshPatch* to,int offset);
    void setNewMainPatch(MeshPatch* patch);




    VertexReference getThirdPoint(VertexReference p1,VertexReference p2);
    /**
     * These are the indices to the texture coordinates to the texture defined by the patch used in the first point
     */
    uint32_t texIndices[3]={0,0,0};



    Triangle(){

    }
    ~Triangle(){
        //TODO: when destroying a triangle we want to unregister it from a neighbour.
        /***************************************************************
         * But there is more to it:
         * when a whole patch is deleted only the stitching triangles should
         * unregister themselves from the neighbour....
         * everything else would be a waste
         **************************************************************/

        //This is not ideal when deleting a patch but better than nothing
        //for deleting whole patches and stitches we need to be more clever
        return;
        /*
        //it seems to cause issues when the vector reallocates and calls destructors when moving the old data
        //move and copy constructors mgiht be needed
        for(int i=0; i<3;i++){
            if(neighbours[i].valid()){
                neighbours[i].ref.get()->neighbours[neighbours[i].pos].invalidate();
            }
            //also removing triangle from vertex
            if(registered){
                points[i].get()->removeTriangle(this);
            }
        }
         */

    }

};




/**
 * @brief The GeometryBase class
 * This is supposed to be a base class to stitches, patches and so on.
 * Because of heritage the access to the overloaded elements might be slow. (vtable)
 * Therefore changing this to "curiously recurring template pattern" might be useful
 */
class GeometryBase{
public:
    int debugHash = 1123;
    bool debugIsValid(){
        return debugHash == 1123;
    }


    int trianglesVersion = -1;

    bool cpuTrianglesAhead = false;//TODO: maybe unneeded



    //TODO: check if this is even necessary
    //TODO: maybe do not do this as virtual method to get higher performance
    /**
     * @brief isGpuResidencyRequired
     * @return
     * this returns true if this structures residency on the gpu is required for rendering
     */
    virtual bool isGpuResidencyRequired() = 0;
    virtual bool isPartOfActiveSet(const ActiveSet* set) = 0;



    //TODO: maybe do this with respect to the active set!
    virtual
    std::shared_ptr<TriangleBufConnector> getMostCurrentGpuTriangles() = 0;

    /**
     * @brief workInProgress
     * a mutex to lock the geometry as soon as it is beeing worked on
     */
    std::mutex workInProgress;///TODO: Maybe rename this to cpuWorkInProgress. or meshingInProgress

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
    std::vector<Triangle> triangles;
    void replacePatchInTriangleReferences(std::shared_ptr<MeshPatch> from,
                                          std::shared_ptr<MeshPatch> to,
                                          int vertexOffset);

    void setNewMainPatchForTriangles(std::shared_ptr<MeshPatch> mainPatch);


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

    virtual std::weak_ptr<GeometryBase> getWeakBaseSelf() = 0;

    virtual ~GeometryBase(){ }


};





class MeshPatchGpuHandle{
private:
public:
    std::mutex mutex;

    std::shared_ptr<PatchInfoBufConnector> patchInfos;
    std::shared_ptr<VertexBufConnector> verticesSource;
    std::shared_ptr<VertexBufConnector> verticesDest;
    std::shared_ptr<TriangleBufConnector> triangles;

    //bool readyForRendering = false;//needed?
    bool gpuVerticesChanged=false;

    std::weak_ptr<MeshPatch> downloadToWhenFinished;
    //TODO: we have to find a way to submit this to the according cleanup thread
    //transmitting the verticesSource to the recycler
    //the recycler should also modified to handle multiple threads all of them
    //working a queue of vertices
    void swapSrcDst(){
        std::swap(verticesSource,verticesDest);
    }

    MeshPatchGpuHandle(GpuGeomStorage* gpuGeomStorage,
                       int nrVertices,
                       int nrTriangles);

    ~MeshPatchGpuHandle();


    //TODO: think about what to do on the texture side of things
    //the plan is that this mesh patch gpu handle keeps grip of
    //the patch texture handles
    std::shared_ptr<MeshTextureGpuHandle> geomTex;

    std::shared_ptr<MeshTextureGpuHandle> texs[GPU_MAX_TEX_PER_PATCH];


    std::shared_ptr<MeshTextureGpuHandle> labelTex;
    bool labelTexValid = false;
    bool setLabelTex(std::shared_ptr<MeshTextureGpuHandle> tex);

    std::shared_ptr<MeshTextureGpuHandle> weightedLabelTexs[3];
    size_t weightedLabelTexCount = 0;
    bool addWeightedLabelTex(std::shared_ptr<MeshTextureGpuHandle> tex);

    //todo replace this with 0 and keep it up to date
    const size_t texCount = GPU_MAX_TEX_PER_PATCH;





    //TODO: think about how to use the triple stitches
    //there is a solution... look at the implementation ( which is not done yet)
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
                  public LowDetailPoint{//<MeshPatch>{// : public std::enable_shared_from_this<MeshPatch>{
private:

public:


    MeshPatch(Octree<MeshPatch> *octree);
    ~MeshPatch();
    /**
     * @brief id
     * an unique identifyer for these patches
     */
    int id=-1;


    bool debug=false;
    int debug1=0;

    std::weak_ptr<MeshPatch> weakSelf;
    std::weak_ptr<GeometryBase> getWeakBaseSelf(){return std::static_pointer_cast<GeometryBase>(weakSelf.lock());};


    bool isCreationProcessOngoing();
    /**
     * @brief center
     * The center of this patch and a bounding sphere.
     * TODO: not set yet
     */
    //Eigen::Vector4f center;
    Eigen::Vector4f principalPlane;
    //float boundingSphereRadius;

    //iterates over all the geometry and updates the octree accordingly
    void updateCenterPoint();
    void updateSphereRadius();
    void updatePrincipalPlaneAndCenter();



    std::weak_ptr<MeshPatchGpuHandle> gpu;

    //TODO: probably get rid of these:
#ifdef MOST_CURRENT
    std::weak_ptr<VertexBufConnector> mostCurrentVertices;
    std::weak_ptr<TriangleBufConnector> mostCurrentTriangles;
#endif
    //int nrTexCoords=-1;

    ///TODO: Use this variable to see when and if there has been changes to the textures
    /// this has to be set to true whenever the process of setting up a texture is finished.
    //TODO: get rid of this
    bool cpuTexPatchAhead=false;
    //TODO: get rid of this
    bool cpuInfoAhead =false;

    //TODO: get rid of this
    bool cpuVerticesAhead=false;
    //bool gpuVerticesAhead=false;

    std::mutex verticesMutex;

    /**
     * @brief vertices
     * This is the storage for the geometry itself.
     * (almost ready to be loaded to gpu)
     */
    std::vector<Vertex> vertices;


    /**
     * @brief texPatches
     * TODO: right now the only real texture is in texPatches[0]
     * In future i want one texturepatch to contain multiple layers (why then separating this from a MeshPatch?)
     * One layer should be(unique name) the variance and median of the geometry
     */
    std::mutex texPatchesMutex;
    std::vector<std::shared_ptr<MeshTexture>> texPatches;// we are really doing this without a shared ptr?

    //TODO: add the functionality to set the according flags for the gpu synchronization
    void addTexPatch(std::shared_ptr<MeshTexture> texPatch);
    void removeTexPatch(std::shared_ptr<MeshTexture> texPatch);
    void removeTexPatches(std::vector<std::shared_ptr<MeshTexture>> texPatches);

    /**
     * @brief gpuResidencyRequired
     * @param required
     * define if the gpuResidency is required or not:
     * true means that the patch has to be on the gpu for the next render requests.
     * this might only be needed for the GeometryBase
     * //TODO: this is sort of not working right now!
     * REDO THIS ACCORDING TO SOLUTION SELECTED IN gpuGeomStorage.h
     */
    //bool gpuResidencyRequired=false;
    //TODO: maybe put this into a base class as part of gpuGeomstorage.h
    //this would help to split up the code accordingly
    std::mutex gpuRequiredBySetsMutex;//TODO: implement this
    std::vector<ActiveSet*> gpuRequiredBySets;
    bool isGpuResidencyRequired();
    int addActiveSet(ActiveSet* activeSet);
    int removeActiveSet(ActiveSet *activeSet);
    bool isPartOfActiveSet(const ActiveSet* activeSet);
    bool isPartOfActiveSetWithNeighbours(const ActiveSet* activeSet);



    //TODO: this is redundant ( the same as allEssentialStitchesPartOfActiveSet)
    //bool allEssentialStitchesPartOfActiveSet(ActiveSet* set);
    /**
     * @brief isValidOnGpu
     * checks if it is fully loaded onto the gpu or if something failed
     * @return
     */
    bool isValidOnGpu();


    /**
    * @brief geomTexPatch
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
    std::mutex geomTexPatchMutex;
    std::shared_ptr<MeshTexture> geomTexPatch;
    /*
    void createGeomTex(std::shared_ptr<TexAtlas> refTexAtlas,
                       std::shared_ptr<TexAtlas> dataAtlas);
    */



    //TODO: this
    std::mutex labelTexPatchMutex;
    std::shared_ptr<MeshTexture> labelTexPatch;
    bool labelTexPatchUsed = false;
    //size_t labelTexPatchUsedChannelCount=0;
    //int labelTexPatchMaxChannels=8;//this should be a property of the map



    //TODO: follow the example of the standard labelling right above
    //(but do it for semantic labels)
    std::mutex semLabelTexPatchMutex;
    std::vector<std::shared_ptr<MeshTexture>> semLabelTexPatches;


    std::mutex doubleStitchMutex;
    //http://www.boost.org/doc/libs/1_55_0/doc/html/lockfree.html (could also be an solution)
    std::vector<std::shared_ptr<DoubleStitch>> doubleStitches;

    std::shared_ptr<DoubleStitch> getDoubleStitchWith(MeshPatch *otherPatch);
    void addStitchReference(std::shared_ptr<DoubleStitch> stitch);
    void removeStitchReference(std::shared_ptr<DoubleStitch> stitch);

    std::mutex tripleStitchMutex;
    std::vector<std::shared_ptr<TripleStitch>> tripleStitches;
    std::shared_ptr<TripleStitch> getTripleStitchWith(MeshPatch* otherPatch1,MeshPatch* otherPatch2);
    void addStitchReference(std::shared_ptr<TripleStitch> stitch);
    void removeStitchReference(std::shared_ptr<TripleStitch> stitch);


    std::set<std::shared_ptr<MeshPatch>> getNeighbours();
    std::set<std::shared_ptr<MeshPatch>> getNeighboursDebug();

    //bool allNeighboursPartOfActiveSet(ActiveSet* activeSet);

    //bool allEssentialStitchesPartOfActiveSet(ActiveSet* activeSet);



    std::shared_ptr<DoubleStitch> getDoubleStitchTo(std::shared_ptr<MeshPatch> patch);
    std::shared_ptr<TripleStitch> getTripleStitchTo(std::shared_ptr<MeshPatch> patch);



    bool isGeometryFullyAllocated();


    /**
     * @brief cleanupStitches
     * When, after a merge of patches, two or more different stitches stitch the same patches
     * we combine these stitches. TODO: check if this is right
     */
    void cleanupStitches();





    std::shared_ptr<TriangleBufConnector> getMostCurrentGpuTriangles(){
        return gpu.lock()->triangles;
    }

    std::shared_ptr<DeformationNode> deformationNode;

};


struct Stitch : public GeometryBase{
    //TODO: this could also just be a weak ptr to the buffer itself
    std::weak_ptr<TriangleBufConnector> trianglesGpu;
    std::shared_ptr<TriangleBufConnector> getMostCurrentGpuTriangles(){
        return trianglesGpu.lock();
    }
};
/**
 * @brief The DoubleStitch struct
 * Geometry to stitch only two geometry texments together
 */
struct DoubleStitch : public Stitch{

    DoubleStitch();
    ~DoubleStitch();

    //the two patches the double stitch is connecting.
    //the main patch(the one whose texture we are using) is patch[0]
    std::weak_ptr<MeshPatch> patches[2];
    bool isEqualTo(int id1,int id2);


//    std::weak_ptr<StitchGpuHandle> gpu;


    std::weak_ptr<DoubleStitch> weakSelf;
    std::weak_ptr<GeometryBase> getWeakBaseSelf(){
        return std::static_pointer_cast<GeometryBase>(weakSelf.lock());
    }



    //remove self from all the patches
    void removeFromPatches(std::shared_ptr<MeshPatch> exceptForPatch=nullptr);

    std::shared_ptr<MeshPatch> getOtherPatch(std::shared_ptr<MeshPatch> const &patch);


    bool isGpuResidencyRequired();
    bool isPartOfActiveSet(const ActiveSet* activeSet);

    bool debugToDelete1=false;
    bool debugToDelete2=false;
    int debugToDelete3=0;

/*
    void replacePatchReference(std::shared_ptr<MeshPatch> from,
                               std::shared_ptr<MeshPatch> to,
                               int offset);
                               */
    void replacePatchReference(std::shared_ptr<MeshPatch> from,
                               std::shared_ptr<MeshPatch> to);
    bool isDegenerated();

    bool isConnectingPatch(std::shared_ptr<MeshPatch> patch);

    bool connectsSamePatches(std::shared_ptr<DoubleStitch> other);

};





/**f
 * @brief The TripleStitch struct
 * A structure that stitches triplets of patches together whenever they touch.
 * TO THINK: Due to a triple stitch most commonly only consisting out of one triangle we want this
 * structure to be even sleeker. Maybe for rendering we should create one buffer for only triple stitches
 * There may even be rules like to not to consider three patches beeing connected when it is only done by
 * a triple stitch.
 * Of course a triple stitch should be removed if one of the patches or points gets deleted.
 */
struct TripleStitch: public Stitch{

    TripleStitch();
    ~TripleStitch();
    ///TODO (not implemented yet)
    //int positionWithinSlot=-1;//since a triple stitch will most certainly only contain one triangle it is necessary to have
    std::weak_ptr<MeshPatch> patches[3];
    bool isGpuResidencyRequired();
    bool isPartOfActiveSet(const ActiveSet* activeSet);
    bool isEqualTo(int id1,int id2,int id3);



    ///TODO: check if this weak self is needed
    std::weak_ptr<TripleStitch> weakSelf;
    std::weak_ptr<GeometryBase> getWeakBaseSelf(){return std::static_pointer_cast<GeometryBase>(weakSelf.lock());};


    //remove self from all the patches
    void removeFromPatches(std::shared_ptr<MeshPatch> exceptForPatch=nullptr);

    std::shared_ptr<MeshPatch> getAnyOtherPatch(std::shared_ptr<MeshPatch> patch);
    //
    bool getOtherPatches(std::shared_ptr<MeshPatch> patch, std::shared_ptr<MeshPatch> patchesOut[2]);


    //TODO: remove these as we now have gpu triangle references stored separately

    /*
    //and replace it with this: (TODO btw)
    std::mutex triangleRefMutex;
    std::vector<TriangleReference> triangleRefs;
    void addTriangleRef(TriangleReference ref);
    std::shared_ptr<StitchGpuHandle> getAnyValidTriangleRef(TriangleReference* triangleRef);

*/
    void replacePatchReference(std::shared_ptr<MeshPatch> from,
                               std::shared_ptr<MeshPatch> to,
                               int offset);


    bool isCovertDoubleStitch();

    bool isConnectingPatch(std::shared_ptr<MeshPatch> patch);
};








/**
 * @brief The Edge struct
 * Stores the connection between two points at a border (therefore there is only one triangle
 */
struct Edge{


    //TODO: given an triangle and the ind in the triangle this point reference should be redundant
    //Triang* triangle;
    TriangleReference triangle;//ideally replace this with something that is not a pointer
    int pos;//edge index within this triangle

    bool isRegistered=false;

    int debug=-1;


    //this is weird

    bool outermostConnectionsMade[2] = {false,false};
    VertexReference outermostConnections[2];
    Eigen::Vector2f outermostConnectionFragmentPos[2];


    //the index is either 0 or 1
    //inline Edge GetNeighbouringEdge(int indInEdge);

    void registerInTriangle(int borderInd,int ind){
        isRegistered = true;
        triangle.get()->edges[pos].borderInd = borderInd;
        triangle.get()->edges[pos].ind = ind;
    }

    void unregister(){
        assert(isRegistered);
        triangle.get()->edges[pos].invalidate();//invalidate the reference to this edge
        isRegistered=false;

    }





    VertexReference points(int i);
    VertexReference oppositePoint();


    //borderlist is used in case a edge is already registered in the triangle
    bool getOtherEdge(int endpoint,Edge& result,std::vector<std::vector<Edge>> &borderList){

        Vertex *debugP = points(1).get();

        Triangle* currentTriangle = triangle.get();
        TriangleReference currentTriangleRef = triangle;
        int currentEdgeIndInTriangle = pos;

        int ttl = 10000;//TODO: put that back to 100.... we will not have than 16 or so triangles for a point
        while(ttl--){

            //go to the next edge on this triangle:
            currentEdgeIndInTriangle += endpoint * 2 - 1;
            if(currentEdgeIndInTriangle == -1){
                currentEdgeIndInTriangle = 2;
            }
            if(currentEdgeIndInTriangle == 3){
                currentEdgeIndInTriangle = 0;
            }
            //if the current edge is free we will return it
            if(!currentTriangle->neighbours[currentEdgeIndInTriangle].valid()){
                if(currentTriangle->edges[currentEdgeIndInTriangle].valid()){
                    result = *currentTriangle->edges[currentEdgeIndInTriangle].get(borderList);
                    assert(result.triangle.valid());
                    //std::cout<<"existing edge" << std::endl;
                    return true;
                }

                Edge res;
                res.triangle = currentTriangleRef;
                res.pos = currentEdgeIndInTriangle;
                result = res;
                //std::cout<<"creating new edge" << std::endl;
                return true;
            }


            if(currentEdgeIndInTriangle >2 || currentEdgeIndInTriangle <0){//debug efforts
                assert(0);
            }
            if(currentTriangleRef.index < 0 || currentTriangleRef.index > 10000){
                std::cout << currentTriangleRef.container->triangles.size() << std::endl;
                assert(0);
            }
            //get to the next triangle
            currentTriangleRef =
                    currentTriangle->neighbours[currentEdgeIndInTriangle].ref;
            if(!currentTriangleRef.container->debugIsValid()){
                getOtherEdge(endpoint,result,borderList);//debug.... this should not happen
                assert(0);
            }
            if(currentTriangleRef.index < 0 || currentTriangleRef.index > 10000){
                getOtherEdge(endpoint,result,borderList);//debug.... this should not happen
                std::cout << currentTriangleRef.container->triangles.size() << std::endl;
                assert(0);
            }
            currentEdgeIndInTriangle =
                    currentTriangle->neighbours[currentEdgeIndInTriangle].pos;
            if(currentEdgeIndInTriangle >2 || currentEdgeIndInTriangle <0){//debug efforts
                assert(0); //(simon) current triangle seems off
            }

            currentTriangle = currentTriangleRef.get();
        }
        getOtherEdge(endpoint,result,borderList);//debug... this should never happen
        assert(0);//more than 100 triangles on this node means something is off!!!
        return false;


    }

    bool isOpen(){
        Triangle* tri = triangle.get();
        return !tri->neighbours[pos].valid();
    }

    bool getOtherEdge(int endpoint,Edge& result){
        assert(0);
    }
    bool equalTo(const Edge& other){
        return triangle.equalTo(other.triangle) && pos == other.pos;
    }





    //TODO: i don't think this is necessary
    // When already used for stitching we do not do it from there
    bool alreadyUsedForStitch=false;


    Edge(){
    }
    Edge(TriangleReference &tri,int indexInTriangle){
        triangle = tri;
        pos = indexInTriangle;
    }
    ~Edge(){

    }

};
const Eigen::Vector2i fourConnNeighborOffsets[]{Eigen::Vector2i(0,-1),//up
            Eigen::Vector2i(1,0),//right
            Eigen::Vector2i(0,1),//down
            Eigen::Vector2i(-1,0)//left
};





inline VertexReference Edge::points(int i){
    assert(i == 0 || i == 1);
    int ind = pos+i;
    if(ind ==3){
        ind = 0;
    }
    return triangle.get()->points[ind];
}

//the .
inline VertexReference Edge::oppositePoint(){
    int ind = pos-1;
    if(ind == -1){
        ind = 2;
    }
    return triangle.get()->points[ind];
}



/******************IMPLEMENTATION FOR PointReference******************/

inline Vertex* VertexReference::get() const{
    return &patch->vertices[index];
}



/*****************IMPLEMENTATION FOR TraingleReference****************/
inline Triangle* TriangleReference::get(){
    assert(valid());
    return &container->triangles[index];
}


#endif


