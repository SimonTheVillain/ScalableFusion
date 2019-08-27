#include "gpuGeomStorage.h"
#include <Eigen/Eigen>


#include <memory>
#include <set>
//#include <GL/glew.h>
//#include <GL/gl.h>

#include "../base/meshStructure.h"
#include "../base/textureStructure.h"
#include "../meshReconstruction.h"
#include "../cuda/gpuErrchk.h"
#include "../cuda/float16_utils.h"
#include "../cuda/texPatchInit.h"
#include "../cuda/stdTexUpdate.h"
#include "../cuda/coalescedMemoryTransfer.h"
#include "../gpu/ActiveSet.h"


using namespace Eigen;
using namespace std;
using namespace cv;



/**
 * We need a new way of handling these slots,
 * Idea: there should be a queue of free slots from which we get free ones.
 * To ensure that they stay in gpu memory we create a new structure (BufferSlotConnector)
 * the last reference to BufferSlotConnector reinserts the utilized slots into the queue.
 * The patches and so on only store a weak_ptr the real shared_ptr should be stored within the ActiveSet Objects
 *
 */




void copy(cudaTextureObject_t texture,cv::cuda::GpuMat &to);




void GpuGeomStorage::resetTimers()
{
    timeSpentUploadingVertices = std::chrono::duration<double>::zero();
    uploadCallsVertices=0;
    timeSpentUploadingTexPos = std::chrono::duration<double>::zero();
    uploadCallsTexPos=0;
    timeSpentUploadingTriangles = std::chrono::duration<double>::zero();
    uploadCallsTriangles=0;
    timeSpentUploadingPatchInfos = std::chrono::duration<double>::zero();
    uploadCallsPatchInfos=0;
}

void GpuGeomStorage::unloadMeshPatch(MeshPatch *patch)
{
    cout << "the mechanism for unloading a mesh patch is not implemented yet" <<  endl;
    assert(0);
}



void GpuGeomStorage::unloadDoubleStitch(DoubleStitch *stitch)
{

}

/**
 * @brief GpuGeomStorage::uploadTripleStitches
 * @param tripleStitches
 * Triple stitches are shit.....
 * We collect the triple stitches to put them collectively into triangle blocks. Whenever one of the triangles within one slot
 * becomes invalid, the slot should be refilled with the remaining valid triangles.
 * TODO: implement this! or maybe not? Probably not!!! (don't know anymore.... think about it
 */
void GpuGeomStorage::uploadTripleStitches(std::vector<TripleStitch *> tripleStitches)
{
    cout << "[GpuGeomStorage::uploadTripleStitches] for using this unimplemented method i sentence you to crash" << endl;
    assert(0);
}

void GpuGeomStorage::initialize()
{
    //TODO: delete these 2 lines from here,
    //this 1) is not necessary with only one gpu
    //and 2) it should happen way earlier in the code
    cudaSetDevice(0);
    //cudaGLSetGLDevice(0);


    //create the buffers that contain slots for our data (vertices, triangles, references to textures
    // and texture coordinates)
    
    vertexBuffer =
        new GpuBuffer<GpuVertex>(m_maxNrVertices);
    texPosBuffer =
        new GpuBuffer<Eigen::Vector2f>(m_maxNrTexCoordinates);
    triangleBuffer =
        new GpuBuffer<GpuTriangle>(m_maxNrTriangles);
    patchInfoBuffer =
        new GpuBuffer<GpuPatchInfo>(m_maxNrLoadedPatchInfos);
    patchInfoIndex =
            new GpuBuffer<GLint>(m_maxNrLoadedPatchInfos,GL_ATOMIC_COUNTER_BUFFER);//,false,GL_ATOMIC_COUNTER_BUFFER);



}

GpuGeomStorage::~GpuGeomStorage()
{
    if(vertexBuffer != nullptr){
        delete vertexBuffer;
        delete texPosBuffer;
        delete triangleBuffer;
        delete patchInfoBuffer;
        delete patchInfoIndex;
    }
}

std::shared_ptr<ActiveSet> GpuGeomStorage::makeActiveSet(std::vector<std::shared_ptr<MeshPatch> > patches,
        MeshReconstruction* map,bool initial,bool debug1)
{

    shared_ptr<ActiveSet> activeSet =
            shared_ptr<ActiveSet>(new ActiveSet(this,patches,map,
                    initial,
                    debug1));//instead of just patches



    return activeSet;
}
