#ifndef FILE_GPU_GEOM_STORAGE_H
#define FILE_GPU_GEOM_STORAGE_H

///TODO: think about a ringbuffer idea for this purpose.
/// Propably a ringbuffer is not the right thing.
/// or think about this:
/// https://www.ibm.com/developerworks/aix/tutorials/au-memorymanager/index.html
/// essentially we create four (unfortunately we can't typecast within the shader)
/// buffers: Information, Vertex, TexCoord, Triangle
/// it would be space efficient though if we put everything into the same buffer
/// if the objects have multiples of their size, we could put multiple o

#include <vector>
#include <memory>
#include <mutex>
#include <queue>
#include <GL/glew.h>
#include <Eigen/Eigen>

//all the cuda and cuda interop stuff
#include <cuda.h>
#include <cuda_gl_interop.h>

#include "../cuda/gpuMeshStructure.h"

#include "gpuBuffer.h"

class MeshReconstruction;
class GeometryBase;
class MeshPatch;
class MeshTexture;
struct DoubleStitch;
struct TripleStitch;
class UnsecuredTriangleCollection;
class MeshTexture;

class GpuGeomStorage;
class MapInformationRenderer;
class MapPresentationRenderer;

class MeshPatchGpuHandle;

class GpuSlottedBuffer;


class TexAtlasPatch;
class FBOConnector;
class Scheduler;

class ActiveSet;
class GeometryUpdate;

/**
 * @brief The GpuGeomStorage class
 * TODO: think about how data could be loaded onto and downloaded from from different threads.
 * (Propably you could easiliy add mutexes that block everything but that is not the nicest way)
 *
 * for the structures defined in scaleableMap it is as follows:
 * Most ideally we know that only the capture thread modifies data on the gpu and the remaining structure.
 * the render thread only needs to read it.
 *
 * For the GpuGeomStorage it is different:
 * We would have to use MUTEXes for the arrays we iterate over, or we use the mutex for every geometry structure!!!!!
 * It is the easiest solution and the only
 * one that secures that we do not duplicate data on the gpu.
 */
/**
 * @brief The GpuGeomStorage class
 */
class GpuGeomStorage{
    friend ActiveSet;
    friend MeshReconstruction;

    friend MapInformationRenderer;
    friend MapPresentationRenderer;


    friend Scheduler;
    friend MeshTexture;
    friend GeometryUpdate;
private:
    uint64_t deleteDebugTexReference;



    MeshReconstruction* map;

public:
    ///TODO: these references to MeshPatches etc. should be shared_ptr.
    /// This would prevent them from being destroyed prematurely but still having references
    /**
     * @brief pointers
     * Have pointers, names and references to all the necessary gpu resources here at this place
     * They are supposed to be connected to cuda with: cudaGraphicsGLRegisterBuffer
     * TOTHINK: we have to protect these pointers from beeing used too early.
     * e.g. as they are not totally uploaded yet but the pointer is set. the renderer might want to render from that.
     * POSSIBLE SOLUTION: protect these arrays with the pointers with mutexes but add "uploadingInProgress" booleans
     * to tell the reading threads that there is something going on.
     * TOTHINK2: a lot of these elements could be deleted on the cpu side.
     * having weak_ptrs in this list would definitely help managing deleting of objects.
     * (the destructor would not have to remove pointers from this list)
     */

    GpuBuffer<GpuVertex>* vertexBuffer = nullptr;
    GpuBuffer<Eigen::Vector2f>* texPosBuffer = nullptr;
    GpuBuffer<GpuTriangle>* triangleBuffer = nullptr;
    GpuBuffer<GpuPatchInfo>* patchInfoBuffer = nullptr;
    GpuBuffer<GLint>* patchInfoIndex = nullptr;
private:

    //TODO: add another buffer with references from texture points to vertices!!!!!!


    //TODO: get rid of these and spearate the reservation of slots from
    //the upload of slots

    std::shared_ptr<TexCoordBufConnector> uploadMeshTexCoords(std::shared_ptr<MeshTexture> coords);
    std::shared_ptr<TriangleBufConnector> uploadTriangles(GeometryBase* baseElement);
    void uploadMeshPatch(MeshPatch* patch, ActiveSet* activeSet);
    void unloadMeshPatch(MeshPatch* patch);
    //std::shared_ptr<BufferSlotConnector> uploadDoubleStitch(DoubleStitch* stitch);
    void unloadDoubleStitch(DoubleStitch* stitch);

    /**
     * @brief uploadTripleStitch
     * again triple stitches, they should be collected and put into one buffer, but only when needed.
     */
    void uploadTripleStitches(std::vector<TripleStitch*> tripleStitches);


public:

    void init(MeshReconstruction* scaleableMap){
        this->map = scaleableMap;
    }

    std::chrono::duration<double> timeSpentUploadingVertices;
    int uploadCallsVertices=0;
    std::chrono::duration<double> timeSpentUploadingTriangles;
    int uploadCallsTriangles=0;
    std::chrono::duration<double> timeSpentUploadingPatchInfos;
    int uploadCallsPatchInfos=0;
    std::chrono::duration<double> timeSpentUploadingTexPos;
    int uploadCallsTexPos=0;
    void resetTimers();

    bool debugOutputs=false;
    uint32_t debug = 3;
    //TODO: we should get this from the scaleableMap parameter structure
    //the assumption is that the average patch has 400 triangles
    uint32_t m_maxNrPatches=1024*5 * 2  * debug;//5k patches is reasonable
    //*2 for debug because we are fragmenting too much.
    //debug because we are fragmenting too much

    uint32_t m_maxNrVertices = 800*m_maxNrPatches ;

    uint32_t m_maxNrTexCoordinates = 1000*m_maxNrPatches;

    uint32_t m_maxNrTriangles = 800*m_maxNrPatches;
    uint32_t m_maxNrtrianglesInCollection = 800;

    uint32_t m_maxNrLoadedPatchInfos = m_maxNrPatches;//TODO: implement this


    void initialize();

    ~GpuGeomStorage();


    std::shared_ptr<ActiveSet> makeActiveSet(std::vector<std::shared_ptr<MeshPatch> > patches =
                                                {}, MeshReconstruction *map = nullptr,
                                                bool initial = false, //TODO: get rid of the debug and initial parameter
                                                bool debug1 = false);//defaults to an empty element


};

#endif
