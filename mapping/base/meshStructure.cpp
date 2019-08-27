#include "meshStructure.h"
#include "../utils/principalPlane.h"
#include "../gpu/texAtlas.h"
#include "../gpu/gpuGeomStorage.h"
#include <iostream>
#include "../gpu/gpuBuffer.h"

#include "../graph/DeformationNode.h"


using namespace std;
using namespace Eigen;

///TODO: replace this with something that returns the pointer not the shared_ptr
MeshPatch::MeshPatch(Octree<MeshPatch> *octree)
{
    //this->map = map;
    setOctree(octree);

    //TODO: get this from a constant... essentially it should speed up the process
    triangles.reserve(800);
    vertices.reserve(800);

    deformationNode = make_shared<DeformationNode>(this);
}

MeshPatch::~MeshPatch()
{
    //cout << "[MeshPatch::~MeshPatch] DEBUG: destructor got called." << endl;
    //cout << "THE TRIANGLES ARE NOT DELETED!!!!!" << endl;
}

bool MeshPatch::isCreationProcessOngoing()
{
    if(geomTexPatch==nullptr){
        return true;
    }else{
        return false;
    }
}

void MeshPatch::updateCenterPoint()
{
    Vector4d c=Vector4d(0,0,0,0);
    for(size_t i=0;i<vertices.size();i++){
        c+=vertices[i].p.cast<double>();
    }
    setPos((c*(1.0/double(vertices.size()))).block<3,1>(0,0).cast<float>());
    //pos = Vector3f(c[0],c[1],c[2]);

}

void MeshPatch::updateSphereRadius()
{
    float r = 0;
    Vector3f p = getPos();
    Vector4f center(p[0],p[1],p[2],0);
    for(size_t i=0;i<vertices.size();i++){
        Vector4f diff = center - vertices[i].p;
        float dist = Vector3f(diff[0],diff[1],diff[2]).norm();
        r = max(r,dist);
    }
    setRadius(r);
}

void MeshPatch::updatePrincipalPlaneAndCenter()
{

    PrincipalPlaneAccumulator accumulator;
    for(size_t i=0;i<vertices.size();i++){
        accumulator.addPoint(vertices[i].p);
        if(isnan(vertices[i].p[0])){
            cout << "[MeshPatch::updatePrincipalPlaneAndCenter] why is this nan?" << endl;
        }
    }
    principalPlane = accumulator.plane();
    Vector4f center = accumulator.centerPoint();
    setPos(Vector3f(center[0],center[1],center[2]));
    if(isnan(center[0])){
        cout << "[MeshPatch::updatePrincipalPlaneAndCenter] why is this nan?" << endl;
    }
}

void MeshPatch::addTexPatch(std::shared_ptr<MeshTexture> texPatch)
{
    //todo: set according flag
    //cout << "[MeshPatch::addTexPatch]we added a new texture for this patch, a (re)upload should be imminent" << endl;
    cpuTexPatchAhead=true;
    texPatches.push_back(texPatch);
}

void MeshPatch::removeTexPatch(std::shared_ptr<MeshTexture> texPatch)
{
    //todo: set according flag
    cpuTexPatchAhead=true;
    texPatches.erase(std::remove(texPatches.begin(),texPatches.end(),texPatch),texPatches.end());
}

void MeshPatch::removeTexPatches(std::vector<std::shared_ptr<MeshTexture> > texPatches)
{
    for(size_t i=0;i<texPatches.size();i++){
        removeTexPatch(texPatches[i]);
    }
}

bool MeshPatch::isGpuResidencyRequired()
{
    if(gpuRequiredBySets.size()>0){
        return true;
    }else{
        return false;
    }
}

int MeshPatch::addActiveSet(ActiveSet *activeSet)
{
    int count;
    gpuRequiredBySetsMutex.lock();
    count = gpuRequiredBySets.size();
    gpuRequiredBySets.push_back(activeSet);
    gpuRequiredBySetsMutex.unlock();
    return count;
}


//returns the amount of active set bound before removing this one!!!!!
int MeshPatch::removeActiveSet(ActiveSet* activeSet)
{
    int count;
    gpuRequiredBySetsMutex.lock();
    count = gpuRequiredBySets.size();
    //the erase deletes the elements at the end. but i stil do not know why this is necessary
    gpuRequiredBySets.erase(std::remove(gpuRequiredBySets.begin(), gpuRequiredBySets.end(), activeSet), gpuRequiredBySets.end());
    gpuRequiredBySetsMutex.unlock();
    return count;
}


bool MeshPatch::isPartOfActiveSet(const ActiveSet *activeSet)
{
    for(size_t i=0;i<gpuRequiredBySets.size();i++){
        if(gpuRequiredBySets[i]==activeSet){
            return true;
        }
    }
    return false;
}
bool MeshPatch::isPartOfActiveSetWithNeighbours(const ActiveSet *activeSet) {
    if(!isPartOfActiveSet(activeSet)){
        return false;
    }

    doubleStitchMutex.lock();
    for(shared_ptr<DoubleStitch> stitch : doubleStitches){
        if(stitch->patches[0].lock().get() != this){
            continue;
        }
        if(!stitch->isPartOfActiveSet(activeSet)){
            //this means not all the neighbours of this stitch are loaded to the gpu
            doubleStitchMutex.unlock();
            return false;
        }
    }
    doubleStitchMutex.unlock();


    tripleStitchMutex.lock();
    for(shared_ptr<TripleStitch> stitch : tripleStitches){
        if(stitch->patches[0].lock().get() != this){
            continue;
        }
        if(!stitch->isPartOfActiveSet(activeSet)){
            //this means not all the neighbours of this are loaded to the gpu
            tripleStitchMutex.unlock();
            return false;
        }
    }
    tripleStitchMutex.unlock();
    return true;
}

/*
bool MeshPatch::allEssentialStitchesPartOfActiveSet(ActiveSet *set)
{
    cout << "[allEssentialStitchesPartOfActiveSet] use isPartOfActiveSetWithNeighbours instead" << endl;
    if(!isPartOfActiveSet(set)){
        return false;
    }
    doubleStitchMutex.lock();
    for(shared_ptr<DoubleStitch> stitch : doubleStitches){
        if(stitch->patches[0].lock().get() != this){
            continue;
        }
        if(!stitch->isPartOfActiveSet(set)){
            //this means not all the neighbours of this stitch are loaded to the gpu
            doubleStitchMutex.unlock();
            return false;
        }
    }
    doubleStitchMutex.unlock();


    tripleStitchMutex.lock();
    for(shared_ptr<TripleStitch> stitch : tripleStitches){
        if(stitch->patches[0].lock().get() != this){
            continue;
        }
        if(!stitch->isPartOfActiveSet(set)){
            //this means not all the neighbours of this are loaded to the gpu
            tripleStitchMutex.unlock();
            return false;
        }
    }
    tripleStitchMutex.unlock();

    return true;
}
 */

bool MeshPatch::isValidOnGpu()
{
    cout << "[MeshPatch::isValidOnGpu] this check probably is useless! don't use it / get rid of it" << endl;
    //todo implement more checks
    if(gpu.lock()!= nullptr){
        return true;
    }
    return false;
}

shared_ptr<DoubleStitch> MeshPatch::getDoubleStitchWith(MeshPatch* otherPatch)
{
    //std::cout << "[MeshPatch::getDoubleStitchWith]replace this with the other function not working with IDs" << std::endl;
    for(int i=0;i<doubleStitches.size();i++){
        bool invalidStitch=false;
        if(doubleStitches[i]->patches[0].use_count()){
            if(doubleStitches[i]->patches[0].lock().get()==otherPatch){
                return doubleStitches[i];
            }
        }else{
            invalidStitch=true;
        }
        if(doubleStitches[i]->patches[1].use_count()){
            if(doubleStitches[i]->patches[1].lock().get()==otherPatch){
                return doubleStitches[i];
            }
        }else{
            invalidStitch=true;
        }
        //DEBUG output..... please delete when this is not seen anywhere
        if(invalidStitch){
            std::cout << "this is a invalid stitch" << std::endl;
        }

    }

    shared_ptr<DoubleStitch> empty;
    return empty;
}

void MeshPatch::addStitchReference(std::shared_ptr<DoubleStitch> stitch)
{
    doubleStitchMutex.lock();
    doubleStitches.push_back(stitch);
    doubleStitchMutex.unlock();
}



shared_ptr<TripleStitch> MeshPatch::getTripleStitchWith(MeshPatch* otherPatch1,MeshPatch* otherPatch2)
{
    MeshPatch* id1=this;
    MeshPatch* id2=otherPatch1;
    MeshPatch* id3=otherPatch2;
    //all permutations:
    const MeshPatch* perm[][3] = {{id1,id2,id3},
                           {id1,id3,id2},
                           {id2,id1,id3},
                           {id2,id3,id1},
                           {id3,id1,id2},
                           {id3,id2,id1}};
    //cout << "dead lock directly here" << endl;
    tripleStitchMutex.lock();
    for(int i=0;i<tripleStitches.size();i++){
        shared_ptr<TripleStitch> stitch = tripleStitches[i];
        for(int j=0;j<6;j++){
            if(     stitch->patches[0].lock().get()==perm[j][0] &&
                    stitch->patches[1].lock().get()==perm[j][1] &&
                    stitch->patches[2].lock().get()==perm[j][2]   ){
                tripleStitchMutex.unlock();
                return stitch;
            }
        }

    }
    tripleStitchMutex.unlock();
    shared_ptr<TripleStitch> empty;
    return empty;
}

void MeshPatch::addStitchReference(std::shared_ptr<TripleStitch> stitch)
{
    tripleStitchMutex.lock();
    tripleStitches.push_back(stitch);
    tripleStitchMutex.unlock();
}

void MeshPatch::removeStitchReference(std::shared_ptr<DoubleStitch> stitch)
{
    doubleStitchMutex.lock();
    doubleStitches.erase(
                remove(doubleStitches.begin(),doubleStitches.end(),stitch),
                doubleStitches.end());
    doubleStitchMutex.unlock();
}

void MeshPatch::removeStitchReference(std::shared_ptr<TripleStitch> stitch)
{
    tripleStitchMutex.lock();
    tripleStitches.erase(
                remove(tripleStitches.begin(),tripleStitches.end(),stitch),
                tripleStitches.end());
    tripleStitchMutex.unlock();
}

std::set<std::shared_ptr<MeshPatch> > MeshPatch::getNeighbours()
{
    set<shared_ptr<MeshPatch>> neighbours;
    doubleStitchMutex.lock();
    for(size_t i=0;i<doubleStitches.size();i++){
        neighbours.insert(doubleStitches[i]->getOtherPatch(weakSelf.lock()));
    }
    doubleStitchMutex.unlock();
    tripleStitchMutex.lock();
    for(size_t i=0;i<tripleStitches.size();i++){
        //continue; //does this make a diference?
        shared_ptr<MeshPatch> otherPatches[2];
        tripleStitches[i]->getOtherPatches(weakSelf.lock(),otherPatches);
        if(otherPatches[0].get()!=this){
            neighbours.insert(otherPatches[0]);
        }else{
            cout << "something is weird" << endl;
            assert(0);
        }
        if(otherPatches[1].get()!=this){
            neighbours.insert(otherPatches[1]);
        }else{
            cout << "something is weird" << endl;
            assert(0);
        }
        if(otherPatches[0].get()==otherPatches[1].get()){
            cout << "this should not happen, even though it might"
                    " be caused when patches are combined and the triple"
                    "stitch degenerates" << endl;
            assert(0);
        }
    }
    tripleStitchMutex.unlock();
    return neighbours;
}

std::set<std::shared_ptr<MeshPatch> > MeshPatch::getNeighboursDebug()
{
    set<shared_ptr<MeshPatch>> neighbours;
    doubleStitchMutex.lock();
    for(size_t i=0;i<doubleStitches.size();i++){
        neighbours.insert(doubleStitches[i]->getOtherPatch(weakSelf.lock()));
    }
    doubleStitchMutex.unlock();
    tripleStitchMutex.lock();
    for(size_t i=0;i<tripleStitches.size();i++){
        continue; //does this make a diference?
        shared_ptr<MeshPatch> otherPatches[2];
        tripleStitches[i]->getOtherPatches(weakSelf.lock(),otherPatches);
        if(otherPatches[0].get()!=this){
            neighbours.insert(otherPatches[0]);
        }else{
            cout << "something is weird" << endl;
            assert(0);
        }
        if(otherPatches[1].get()!=this){
            neighbours.insert(otherPatches[1]);
        }else{
            cout << "something is weird" << endl;
            assert(0);
        }
        if(otherPatches[0].get()==otherPatches[1].get()){
            cout << "this should not happen, even though it might"
                    " be caused when patches are combined and the triple"
                    "stitch degenerates" << endl;
            assert(0);
        }
    }
    tripleStitchMutex.unlock();
    return neighbours;
}

/*
bool MeshPatch::allNeighboursPartOfActiveSet(ActiveSet *activeSet)
{
    set<shared_ptr<MeshPatch>> neighbours = getNeighbours();
    for(auto neighbour : neighbours){
        if(!neighbour->isPartOfActiveSet(activeSet)){
            return false;
        }
    }
    return true;
}
 */

/*
bool MeshPatch::allEssentialStitchesPartOfActiveSet(ActiveSet *activeSet)
{
    if(!isPartOfActiveSet(activeSet)){
        return false;
    }
    doubleStitchMutex.lock();
    for(shared_ptr<DoubleStitch> stitch : doubleStitches){
        if(stitch->patches[0].lock().get() != this){
            continue;
        }
        if(!stitch->isPartOfActiveSet(activeSet)){
            //this means not all the neighbours of this are loaded to the gpu
            doubleStitchMutex.unlock();
            return false;
        }
    }
    doubleStitchMutex.unlock();


    tripleStitchMutex.lock();
    for(shared_ptr<TripleStitch> stitch : tripleStitches){
        if(stitch->patches[0].lock().get() != this){
            continue;
        }
        if(!stitch->isPartOfActiveSet(activeSet)){
            //this means not all the neighbours of this are loaded to the gpu
            tripleStitchMutex.unlock();
            return false;
        }
    }
    tripleStitchMutex.unlock();

    return true;
}
*/
std::shared_ptr<DoubleStitch> MeshPatch::getDoubleStitchTo(std::shared_ptr<MeshPatch> patch)
{
    doubleStitchMutex.lock();
    for(size_t i=0;i<doubleStitches.size();i++){
        if(doubleStitches[i]->isConnectingPatch(patch)){
            doubleStitchMutex.unlock();
            return doubleStitches[i];
        }
    }
    doubleStitchMutex.unlock();
    return nullptr;
}

std::shared_ptr<TripleStitch> MeshPatch::getTripleStitchTo(std::shared_ptr<MeshPatch> patch)
{
    tripleStitchMutex.lock();
    for(size_t i=0;i<tripleStitches.size();i++){
        if(tripleStitches[i]->isConnectingPatch(patch)){
            tripleStitchMutex.unlock();
            return tripleStitches[i];
        }
    }
    tripleStitchMutex.unlock();
    return nullptr;
}

bool MeshPatch::isGeometryFullyAllocated()
{
    assert(0);
    return gpu.lock() != nullptr;
}


TripleStitch::TripleStitch()
{
    //mostly this will only be one triangle
    triangles.reserve(4);
}

TripleStitch::~TripleStitch()
{
#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
    cout << "[TripleStitch::~TripleStitch] DEBUG: deletion of triple stitch. " << endl;
#endif
    //cout << "THE TRIANGLES ARE NOT DELETED!!!!!" << endl;
}

bool TripleStitch::isGpuResidencyRequired()
{
    //test if one of the required patches is not required anymore, otherwise keep this triangle on the gpu
    if(     patches[0].lock()->isGpuResidencyRequired() &&
            patches[1].lock()->isGpuResidencyRequired() &&
            patches[2].lock()->isGpuResidencyRequired()){
        return true;
    }
    return false;
}

bool TripleStitch::isPartOfActiveSet(const ActiveSet *activeSet)
{
    //test if one of the required patches is not required anymore, otherwise keep this triangle on the gpu
    if(     patches[0].lock()->isPartOfActiveSet(activeSet) &&
            patches[1].lock()->isPartOfActiveSet(activeSet) &&
            patches[2].lock()->isPartOfActiveSet(activeSet)){
        return true;
    }
    return false;
}

void TripleStitch::removeFromPatches(std::shared_ptr<MeshPatch> exceptForPatch)
{
    for(size_t i=0;i<3;i++){
        std::shared_ptr<MeshPatch> locked = patches[i].lock();
        if(exceptForPatch==locked || locked.use_count()==0){
            continue;
        }
        locked->removeStitchReference(weakSelf.lock());
    }
}

std::shared_ptr<MeshPatch> TripleStitch::getAnyOtherPatch(std::shared_ptr<MeshPatch> patch)
{

    if(patch==patches[0].lock()){
        return patches[1].lock();
    }
    if(patch==patches[1].lock()){
        return patches[0].lock();
    }
    if(patch==patches[2].lock()){
        return patches[0].lock();
    }
    return nullptr;
}

bool TripleStitch::getOtherPatches(std::shared_ptr<MeshPatch> patch, std::shared_ptr<MeshPatch> patchesOut[])
{
    if(patch==patches[0].lock()){
        patchesOut[0] = patches[1].lock();
        patchesOut[1] = patches[2].lock();
        return true;
    }
    if(patch==patches[1].lock()){
        patchesOut[0] = patches[0].lock();
        patchesOut[1] = patches[2].lock();
        return true;
    }
    if(patch==patches[2].lock()){
        patchesOut[0] = patches[0].lock();
        patchesOut[1] = patches[1].lock();
        return true;
    }
    return false;
}
/*
void TripleStitch::addTriangleRef(TriangleReference ref)
{
    triangleRefMutex.lock();
    triangleRefs.push_back(ref);
    triangleRefMutex.unlock();
}

std::shared_ptr<StitchGpuHandle> TripleStitch::getAnyValidTriangleRef(TriangleReference *triangleRef)
{
    //THIS IS UNTESTED!!!!

    std::shared_ptr<StitchGpuHandle> handle;
    triangleRefMutex.lock();
    for(size_t i = 0; i< triangleRefs.size(); i++){
        if(!triangleRefs[i].isValid()){
            triangleRefs[i]=triangleRefs[triangleRefs.size()-1];
            triangleRefs.pop_back();
            i--;
            continue;
        }
        *triangleRef = triangleRefs[i];
        handle = triangleRef->triangles.lock();
        break;
    }
    triangleRefMutex.unlock();
    return handle;
}
 */
/*
void TripleStitch::addTriangleBlockRef(UnsecuredTriangleBlockReference blockRef)
{
    triangleBlockMutex.lock();
    triangleBlocks.push_back(blockRef);
    triangleBlockMutex.unlock();
}

std::shared_ptr<UnsecuredTriangleCollection> TripleStitch::getAnyTriangleBlock(int &pos, int &length)
{
    triangleBlockMutex.lock();
    shared_ptr<UnsecuredTriangleCollection> triangleCollection = nullptr;
    vector<size_t> toErase;
    for(size_t i = 0; i < triangleBlocks.size();i++){
        if(triangleBlocks[i].triangleCollection.expired()){
            //add it to the list of elements that should e removed
            toErase.push_back(i);
        }else{
            if(!triangleCollection){
                triangleCollection = triangleBlocks[i].triangleCollection.lock();
                pos = triangleBlocks[i].posInCollection;
                length = triangleBlocks[i].size;

            }
        }
    }
    //https://stackoverflow.com/questions/3487717/erasing-multiple-objects-from-a-stdvector
    for(size_t i = 0; i<toErase.size();i++){
        triangleBlocks[toErase[i]] = triangleBlocks.back();
        triangleBlocks.pop_back();
    }
    triangleBlockMutex.unlock();
    return triangleCollection;
}
*/

void TripleStitch::replacePatchReference(std::shared_ptr<MeshPatch> from, std::shared_ptr<MeshPatch> to, int offset)
{
    for(size_t i=0;i<3;i++){
        if(patches[i].lock() == from){
            patches[i] = to;
        }
    }
    replacePatchInTriangleReferences(from,to,offset);
}

bool TripleStitch::isCovertDoubleStitch()
{
    return  patches[0].lock()==patches[1].lock() ||
            patches[1].lock()==patches[2].lock() ||
            patches[0].lock()==patches[2].lock();
}

bool TripleStitch::isConnectingPatch(std::shared_ptr<MeshPatch> patch)
{
    for(size_t i=0;i<3;i++){
        //assert(0); //here (at the weak pointer) we had a memory exception
        if(patches[i].lock() == patch){
            return true;
        }
    }
    return false;
}

DoubleStitch::DoubleStitch()
{
    triangles.reserve(100);
}

DoubleStitch::~DoubleStitch() {
#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
    cout << "[DoubleStitch::~DoubleStitch] DEBUG: deletion of double stitch." << endl;
#endif

}

void DoubleStitch::removeFromPatches(shared_ptr<MeshPatch> exceptForPatch)
{
    for(size_t i=0;i<2;i++){
        std::shared_ptr<MeshPatch> locked = patches[i].lock();
        if(exceptForPatch==locked || locked.use_count()==0){
            continue;
        }
        locked->removeStitchReference(weakSelf.lock());
    }
}

std::shared_ptr<MeshPatch> DoubleStitch::getOtherPatch(std::shared_ptr<MeshPatch> const &patch)
{
    if(patch==patches[0].lock()){
        return patches[1].lock();
    }
    if(patch==patches[1].lock()){
        return patches[0].lock();
    }
    return nullptr;
}

bool DoubleStitch::isGpuResidencyRequired(){

    //test if one of the required patches is not required anymore, otherwise keep this triangle on the gpu
    if(     patches[0].lock()->isGpuResidencyRequired() &&
            patches[1].lock()->isGpuResidencyRequired()){
        return true;
    }
    return false;
}
bool DoubleStitch::isPartOfActiveSet(const ActiveSet *activeSet){
    //test if one of the required patches is not required anymore, otherwise keep this triangle on the gpu
    if(     patches[0].lock()->isPartOfActiveSet(activeSet) &&
            patches[1].lock()->isPartOfActiveSet(activeSet)){
        return true;
    }
    return false;
}

void DoubleStitch::replacePatchReference(std::shared_ptr<MeshPatch> from, std::shared_ptr<MeshPatch> to)
{
    for(size_t i=0;i<2;i++){
        if(patches[i].lock() == from){
            patches[i] = to;
        }
    }
    //replacePatchInTriangleReferences(from,to,offset);
}

bool DoubleStitch::isDegenerated()
{
    return patches[0].lock()==patches[1].lock();
}

bool DoubleStitch::isConnectingPatch(std::shared_ptr<MeshPatch> patch)
{
    if(patches[0].lock()==patch){
        return true;
    }
    if(patches[1].lock()==patch){
        return true;
    }
    return false;
}

bool DoubleStitch::connectsSamePatches(std::shared_ptr<DoubleStitch> other)
{
    if(other->patches[0].lock()==patches[0].lock() &&
            other->patches[1].lock() == patches[1].lock()){
        return true;
    }
    if(other->patches[1].lock() == patches[0].lock() &&
            other->patches[0].lock() == patches[1].lock()){
        return true;
    }

    return false;
}

void GeometryBase::replacePatchInTriangleReferences(    std::shared_ptr<MeshPatch> from,
                                                        std::shared_ptr<MeshPatch> to,
                                                        int vertexOffset)
{
    //do i even know what this is about?
    for(size_t i=0;i<triangles.size();i++){
        triangles[i].replacePatchReference(from.get(),to.get(),vertexOffset);
    }
}

void GeometryBase::setNewMainPatchForTriangles(std::shared_ptr<MeshPatch> mainPatch)
{
    for(size_t i=0;i<triangles.size();i++){
        triangles[i].setNewMainPatch(mainPatch.get());
    }
}

void GeometryBase::deregisterTriangles() {
    for(size_t i=0;i<triangles.size();i++){
        for(size_t k=0;k<3;k++){
            //TODO: remove this comment and the "removeTriangle" line if it turns out it really is not needed
            //Actually this should not be necessary: (might even be the cause for crashes)
            triangles[i].points[k].get()->removeTriangle(&triangles[i]);

            //a bit expensive but usually less than 8 iterations till triangle is found
            if(triangles[i].neighbours[k].valid()){
                Triangle * tri1=triangles[i].neighbours[k].ref.get();
                Triangle* tri2 =
                        triangles[i].neighbours[k].ref.get()->neighbours[triangles[i].neighbours[k].pos].ref.get();
                assert(triangles[i].neighbours[k].ref.get()->neighbours[triangles[i].neighbours[k].pos].ref.get() ==
                               &triangles[i]);
                assert(triangles[i].neighbours[k].ref.get()->neighbours[triangles[i].neighbours[k].pos].valid());
                triangles[i].neighbours[k].ref.get()->neighbours[triangles[i].neighbours[k].pos].invalidate();
                triangles[i].neighbours[k].invalidate();
                //cout << "invalidate the second neighbourhood field" << endl;
            }
        }
    }
    //triangles.clear();
}

void Triangle::replacePatchReference(MeshPatch *from, MeshPatch *to, int offset)
{
    for(size_t i=0;i<3;i++){
        if(points[i].getPatch()==from){
            points[i].set(to,points[i].getIndex()+offset);
        }
    }
}

void Triangle::setNewMainPatch(MeshPatch *patch)
{
    for(size_t i=1;i<3;i++){
        if(points[i].getPatch() == patch){
            std::swap(points[0],points[i]);
            return;
        }
    }
}

VertexReference Triangle::getThirdPoint(VertexReference p1, VertexReference p2)
{
    VertexReference result;
    bool debug=false;
    for(size_t i=0;i<3;i++){
        if(points[i].getIndex() == p1.getIndex() &&
                points[i].getPatch() == p1.getPatch()){
            continue;
        }
        if(points[i].getIndex() == p2.getIndex() &&
                points[i].getPatch() == p2.getPatch()){
            continue;
        }



        debug=true;
        result = points[i];
    }
    if(debug==false){
        assert(0);
    }
    return result;
}

/*
Edge Edge::GetNeighbouringEdge(int indInEdge) {
    assert(triangle.valid());
    //first check on the triangle itself:
    int indNextEdge = indInTriangle + indInEdge *2 -1;
    if(indNextEdge == 3){
        indNextEdge = 0;
    }else if(indNextEdge==-1){
        indNextEdge = 2;
    }

    if(!triangle.get()->neighbours[indNextEdge].valid()){
        if(!triangle.get()->edges[indNextEdge].valid()){
            Edge newEdge(triangle,indInTriangle + indInEdge);
            //newEdge.triangle = triangle;
            //newEdge.indInTriangle = indInTriangle + indInEdge;
            return newEdge;
        }else{
            return *(triangle.get()->edges[indNextEdge].get());
        }
    }

    Edge edgeToReturn;
    int foundValidEdges=0;
    int indPoint = indInTriangle + indInEdge;
    Vertex *vert = triangle.get()->points[indPoint].get();
    for(size_t i=0;i<vert->triangles.size();i++){
        //TODO: find edge here!
        int pointIndInTriangle = vert->triangles[i].indInTriangle;
        //indices of neighbouring edges
        int is[2] = {pointIndInTriangle, pointIndInTriangle + 1};
        if(is[1]==3){//going circles around a triangle
            is[1] = 0;
        }
        for(size_t k=0;k<2;k++){
            //check for edges
            if(!vert->triangles[i].triangle.get()->neighbours[is[k]].valid()){
                if(vert->triangles[i].triangle.get()->edges[is[k]].valid()){
                    edgeToReturn = *(vert->triangles[i].triangle.get()->edges[is[k]].get());
                    foundValidEdges ++;
                }else{
                    edgeToReturn.triangle = vert->triangles[i].triangle;
                    edgeToReturn.indInTriangle = is[k];

                    foundValidEdges ++;
                }
            }
        }

    }
    if(foundValidEdges!=1){
        assert(0);
    }
    return edgeToReturn;

}
 */

bool Vertex::encompassed(){
    if(triangles.size()<3){
        return false;
    }
    //VertexInTriangle triRef = triangles[0];
    TriangleReference triRef;
    Triangle *firstTri = triangles[0].triangle.get();
    Triangle *tri = firstTri;

    int ind = triangles[0].indInTriangle;
    int debugTTL=100000;
    while(debugTTL--){
        ind--;
        if(ind==-1){
            ind = 2;
        }
        if(tri->neighbours[ind].valid()){
            //get the next triangle
            triRef = tri->neighbours[ind].ref;
            ind= tri->neighbours[ind].pos;
            tri = triRef.get();
            if(tri == firstTri){
                //we came back to the first triangle
                return true;
            }

        }else{
            return false;
        }

    }
    encompassed();//debug just so we can GDB into it
    assert(0);
}
std::vector<shared_ptr<VertexBufConnector>> debugRetainVertices;
std::vector<shared_ptr<TriangleBufConnector>> debugRetainTriangleBuf;
MeshPatchGpuHandle::MeshPatchGpuHandle(GpuGeomStorage* gpuGeomStorage,
                                       int nrVertices, int nrTriangles)
{
    verticesDest = gpuGeomStorage->vertexBuffer->getBlock(nrVertices);
    verticesSource = gpuGeomStorage->vertexBuffer->getBlock(nrVertices);
    patchInfos = gpuGeomStorage->patchInfoBuffer->getBlock(1);
    triangles = gpuGeomStorage->triangleBuffer->getBlock(nrTriangles);

    //debugRetainTriangleBuf.push_back(triangles);
    //debugRetainVertices.push_back(verticesSource);
}

MeshPatchGpuHandle::~MeshPatchGpuHandle()
{
    std::shared_ptr<MeshPatch> downloadTo = downloadToWhenFinished.lock();
    /*
    if(recycler!=nullptr &&
            downloadTo!=nullptr &&
            gpuVerticesChanged){
        //cout << "we need to download the source vertices" << endl;
        downloadTo->mostCurrentVertices = verticesSource;
        downloadTo->mostCurrentTriangles = triangles;

        auto downloadFunc = [](std::shared_ptr<MeshPatch> patch,
                std::shared_ptr<VertexBufConnector> vertices,
                std::shared_ptr<TriangleBufConnector> retainPatches){
            patch->vertices.reserve(vertices->getSize());
            //The assumption is that the gpu vertex and the cpu vertex are the
            //same
            vertices->download((GpuVertex*)&(patch->vertices[0]));
            retainPatches.reset();


        };

        //TODO bind the function and forward it to the recycler
        auto task = std::bind(downloadFunc,downloadTo,verticesSource,triangles);
        recycler->addRecycleTask(task);
    }else{
        //cout << "[MeshPatchGpuHandle::~MeshPatchGpuHandle] recycler is not set" << endl;
    }
     */
    //TODO: do something similar for textures and texture coordinates
}

bool MeshPatchGpuHandle::setLabelTex(std::shared_ptr<MeshTextureGpuHandle> tex)
{
    labelTex = tex;
    labelTexValid = true;
}

bool MeshPatchGpuHandle::addWeightedLabelTex(std::shared_ptr<MeshTextureGpuHandle> tex)
{
    size_t maxLabCnt = sizeof(weightedLabelTexs)/sizeof(tex);
    if(weightedLabelTexCount >= maxLabCnt){
        return false;
    }else{
        weightedLabelTexs[weightedLabelTexCount] = tex;
        weightedLabelTexCount++;
        return true;
    }

    return false;
}

void Triangle::registerTriangle(TriangleReference &triangleToRegister,bool searchEdges,bool debug){
    //assert(0);//Depending on if the triangle structure is available this method could be simplified
    //searchEdges = true;
    Triangle *triangle = triangleToRegister.get();
    triangle->registered = true;
    Vertex& p1=*(triangle->points[0].get());
    Vertex& p2=*(triangle->points[1].get());
    Vertex& p3=*(triangle->points[2].get());

    if(!searchEdges){
        if(debug){
            //check if the proposed edge neighbours really make sense
            //lets debug here:
            Neighbour neighboursTest[3];
            for(uint32_t i=0;i<p1.triangles.size();i++){
                if(p1.triangles[i].triangle.get()->containsPoint(triangleToRegister.get()->points[1])) {
                    TriangleReference &otherTriangleRef = p1.triangles[i].triangle;
                    Triangle *otherTriangle = otherTriangleRef.get();
                    int k = otherTriangle->getPosInTriangle(triangleToRegister.get()->points[0],triangleToRegister.get()->points[1]);
                    neighboursTest[0].ref = otherTriangleRef;
                    neighboursTest[0].pos = k;
                }
                if(p1.triangles[i].triangle.get()->containsPoint(triangleToRegister.get()->points[2])) {
                    TriangleReference &otherTriangleRef = p1.triangles[i].triangle;
                    Triangle *otherTriangle = otherTriangleRef.get();
                    //TODO: it is ineficient to first check if it exists and afterwards retrieve it
                    int k = otherTriangle->getPosInTriangle(triangleToRegister.get()->points[0],
                                                            triangleToRegister.get()->points[2]);
                    neighboursTest[2].ref = otherTriangleRef;
                    neighboursTest[2].pos = k;
                }

            }


            for(uint32_t i=0;i<p2.triangles.size();i++) {
                if (!p2.triangles[i].triangle.container->debugIsValid()) {
                    assert(0);
                }
                if (p2.triangles[i].triangle.get()->containsPoint(triangleToRegister.get()->points[2])) {
                    TriangleReference &otherTriangleRef = p2.triangles[i].triangle;
                    Triangle *otherTriangle = otherTriangleRef.get();
                    //TODO: it is ineficient to first check if it exists and afterwards retrieve it
                    int k =
                            otherTriangle->getPosInTriangle(triangleToRegister.get()->points[1],
                                                            triangleToRegister.get()->points[2]);
                    neighboursTest[1].ref = otherTriangleRef;
                    neighboursTest[1].pos = k;
                }
            }



            for(int i : {0,1,2}){
                if(!triangle->neighbours[i].ref.equalTo(neighboursTest[i].ref) ||
                  triangle->neighbours[i].pos != neighboursTest[i].pos){
                    assert(0);
                }
            }
        }
        for(uint32_t i : {0,1,2}){
            if(triangle->neighbours[i].valid()){
                Triangle* other = triangle->neighbours[i].ref.get();
                other->neighbours[triangle->neighbours[i].pos].ref = triangleToRegister;
                other->neighbours[triangle->neighbours[i].pos].pos = i;
                //DEBUG: get rid of this!
                if(other->neighbours[triangle->neighbours[i].pos].ref.get() != triangle){
                    assert(0);//this seems invalid then!!!!!!"!!!!!!!!!°
                }

            }
        }
        //register points!
        Vertex::VertexInTriangle vit;
        vit.triangle = triangleToRegister;
        vit.indInTriangle = 0;
        p1.triangles.push_back(vit);

        vit.indInTriangle = 1;
        p2.triangles.push_back(vit);

        vit.indInTriangle = 2;
        p3.triangles.push_back(vit);
        return;
    }else{
        for(int i : {0,1,2}){
            triangle->neighbours[i].invalidate();
        }
    }

    //Add triangle to neighbour list af adjacent triangles
    int debugCount=0;
    for(uint32_t i=0;i<p1.triangles.size();i++){
        if(p1.triangles[i].triangle.get()->containsPoint(triangleToRegister.get()->points[1])){
            TriangleReference &otherTriangleRef = p1.triangles[i].triangle;
            Triangle *otherTriangle = otherTriangleRef.get();
            //TODO: it is ineficient to first check if it exists and afterwards retrieve it
            int k = otherTriangle->getPosInTriangle(triangleToRegister.get()->points[0],triangleToRegister.get()->points[1]);
            if(otherTriangle->neighbours[k].valid()){
                //it seems that we want to add another triangle to an edge that already has two
                Triangle* debugTriangle = otherTriangle->neighbours[k].ref.get();
                assert(0);// when registering a triangle we hope that it already is valid
            }
            //get where the triangle belongs
            triangle->neighbours[0].ref = otherTriangleRef;
            triangle->neighbours[0].pos = k;
            triangle->neighbours[0].debug = debug;
            otherTriangle->neighbours[k].ref = triangleToRegister;
            otherTriangle->neighbours[k].pos = 0;
            otherTriangle->neighbours[k].debug = debug;


            debugCount++;
        }
        if(p1.triangles[i].triangle.get()->containsPoint(triangleToRegister.get()->points[2])){
            TriangleReference &otherTriangleRef = p1.triangles[i].triangle;
            Triangle *otherTriangle = otherTriangleRef.get();
            //TODO: it is ineficient to first check if it exists and afterwards retrieve it
            int k = otherTriangle->getPosInTriangle(triangleToRegister.get()->points[0],triangleToRegister.get()->points[2]);
            if(otherTriangle->neighbours[k].valid()){
                //it seems that we want to add another triangle to an edge that already has two
                Triangle* debugTriangle = otherTriangle->neighbours[k].ref.get();
                assert(0);// when registering a triangle we hope that it already is valid
            }
            //get where the triangle belongs
            triangle->neighbours[2].ref = otherTriangleRef;
            triangle->neighbours[2].pos = k;
            triangle->neighbours[2].debug = debug;
            otherTriangle->neighbours[k].ref = triangleToRegister;
            otherTriangle->neighbours[k].pos = 2;
            otherTriangle->neighbours[k].debug = debug;

            debugCount++;
        }
    }

    for(uint32_t i=0;i<p2.triangles.size();i++){
        if(!p2.triangles[i].triangle.container->debugIsValid()){
            assert(0);
        }
        if(p2.triangles[i].triangle.get()->containsPoint(triangleToRegister.get()->points[2])){
            TriangleReference &otherTriangleRef = p2.triangles[i].triangle;
            Triangle *otherTriangle = otherTriangleRef.get();
            //TODO: it is ineficient to first check if it exists and afterwards retrieve it
            int k =
                    otherTriangle->getPosInTriangle(triangleToRegister.get()->points[1],triangleToRegister.get()->points[2]);
            if(otherTriangle->neighbours[k].valid()){
                //it seems that we want to add another triangle to an edge that already has two
                Triangle* debugTriangle = otherTriangle->neighbours[k].ref.get();
                assert(0);// when registering a triangle we hope that it already is valid
            }
            //get where the triangle belongs
            triangle->neighbours[1].ref = otherTriangleRef;
            triangle->neighbours[1].pos = k;
            triangle->neighbours[1].debug = debug;
            otherTriangle->neighbours[k].ref = triangleToRegister;
            otherTriangle->neighbours[k].pos = 1;
            otherTriangle->neighbours[k].debug = debug;

            debugCount++;
        }
    }
    if(debugCount>3){
        //This means that we likely have two triangles with the same edge points
        assert(0);
    }

    //last step is to add the triangles to the according points
    Vertex::VertexInTriangle vit;
    vit.triangle = triangleToRegister;
    vit.indInTriangle = 0;
    p1.triangles.push_back(vit);

    vit.indInTriangle = 1;
    p2.triangles.push_back(vit);

    vit.indInTriangle = 2;
    p3.triangles.push_back(vit);




    //check if neighbouring triangles are oriented consistently
    Triangle* tri = triangle;
    for(int i : {0,1,2}){
        if(tri->neighbours[i].valid()){
            Triangle* nb = tri->neighbours[i].ref.get();
            Vertex* v11 = tri->points[i].get();
            int ind2 = i==2 ? 0 : i+1;
            Vertex* v12 = tri->points[ind2].get();

            int ind1 = tri->neighbours[i].pos;
            Vertex* v21 = nb->points[ind1].get();
            ind2 = ind1==2 ? 0 : ind1+1;
            Vertex* v22 = nb->points[ind2].get();
            if(v11 != v22 || v12 != v21){
                assert(0);
            }




        }
    }


}

/*
StitchGpuHandle::StitchGpuHandle(TriangleBuffer *triangleBuffer, int nrTriangles)
{
    count = nrTriangles;
    triangles = triangleBuffer->getBlock(nrTriangles);
}

StitchGpuHandle::~StitchGpuHandle()
{

    //cout << "[StitchGpuHandle::~StitchGpuHandle] "
    //        "Start a proper download task" << endl;

    //nothing that really needs to be downloaded!!! except for triangles
    //at some later point
}
*/
