#include "meshStructure.h"
#include <limits>
#include "../gpu/gpuGeomStorage.h"
#include "../gpu/gpuBuffer.h"
#include "../cuda/float16_utils.h"
#include "../cuda/gpuErrchk.h"
#include "../meshReconstruction.h"
#include "textureStructure.h"


#include <chrono>

using namespace cv;
using namespace std;
using namespace Eigen;


MeshTexture::MeshTexture(std::shared_ptr<TexAtlas> referenceAtlas, std::shared_ptr<TexAtlas> dataAtlas){
    //this->refAtlas = referenceAtlas;
    //this->dataAtlas = dataAtlas;
}

MeshTexture::MeshTexture(MeshTexture::Type type, MeshReconstruction *map)
{
    this->map = map;
    this->type = type;
}

MeshTexture::~MeshTexture(){
    /*if(glFramebufferToPixReference){
        glDeleteFramebuffers(1,&glFramebufferToPixReference);
    }*/
}

std::shared_ptr<MeshTextureGpuHandle> MeshTexture::genGpuResource(size_t nrCoords,
        Size2i size)
{

    TexAtlas* refAtlas = map->texAtlasGeomLookup.get();
    TexAtlas* dataAtlas=nullptr;
    switch(type){
    case MeshTexture::Type::color:
        assert(0);
        //dataAtlas = map->texAtlasRgb16F.get();
        break;
    case MeshTexture::Type::color8:
        dataAtlas = map->texAtlasRgb8Bit.get();
        //TODO: add this to exarregate the impact of that texture leak
        refAtlas = nullptr;//TODO: remove this when we finally have a better way of updating the color texture
        break;
    case MeshTexture::Type::integerLabels:
        dataAtlas = map->texAtlasSegLabels.get();
        break;
    case MeshTexture::Type::weightedIntegerLabels:
        dataAtlas = map->texAtlasSemSegLabelsWeights.get();
        break;
    case MeshTexture::Type::standardDeviation:
        dataAtlas = map->texAtlasStds.get();
        break;
    }

    TexCoordBuffer* coordBuf = map->m_gpuGeomStorage.texPosBuffer;

    std::shared_ptr<MeshTextureGpuHandle> meshTexGpu(new MeshTextureGpuHandle(
                                                         coordBuf,
                                                         nrCoords,
                                                         refAtlas,
                                                         dataAtlas,
                                                         size.width,
                                                         size.height,
                                                         nullptr,
                                                         nullptr
                                                         ));
    //cout << "[MeshTexture] todo" << endl;
    //Explanation: When textures or tex coordinates are currently being downloaded
    //for this module they should be safed from being scrapped and then reapplied
    //the MeshTexture class is just the right place to handle this!!!!!!
    //TODO!!!
    //take a look at the creation of active sets!
    //assert(0);//sourceTexBeingDownloaded & texCoordsbeeingDownloaded needs to be added
    return meshTexGpu;

}

/*
void MeshTexture::createPixReferencesFramebuffer()
{
    //use the proper function in
}
*/

bool MeshTexture::isGpuResidencyRequired()
{
    if(parentPatch!=NULL){
        return parentPatch->isGpuResidencyRequired();
    }else{
        cout << "[MeshTexture::isGpuResidencyRequired] The parentPatch unfortunately is not set, therefore this function fails" << endl;
    }
    return false;
}

/*
std::shared_ptr<TexAtlasPatch> MeshTexture::getSourceTexPatch(){
    std::shared_ptr<MeshTextureGpuHandle> data = gpu.lock();
    return data->sourceTex;
    //return gpu.lock()->sourceTexPatch.lock();
}

std::shared_ptr<TexAtlasPatch> MeshTexture::getDestinationTexPatch(){
    std::shared_ptr<MeshTextureGpuHandle> data = gpu.lock();
    return data->destTex;
}

std::shared_ptr<TexAtlasPatch> MeshTexture::getLookupTexPatch(){
    std::shared_ptr<MeshTextureGpuHandle> data = gpu.lock();
    return data->refTex;
}

cv::Rect2f MeshTexture::getSourceRect()
{
    std::shared_ptr<MeshTextureGpuHandle> data = gpu.lock();
    return data->sourceTex->getRectOfInterest();
}

cv::Rect2f MeshTexture::getDestinationRect()
{
    std::shared_ptr<MeshTextureGpuHandle> data = gpu.lock();
    return data->destTex->getRectOfInterest();
}
*/
cv::Rect2i MeshTexture::getLookupRect(){
    //TODO: get rid of this, Since it is locking a weak pointer this would create a false sense of safety
    std::shared_ptr<MeshTextureGpuHandle> data = gpu.lock();
    return data->refTex->getRect();
}


cv::Rect2f MeshTexture::getBounds(const std::vector<Vector2f> &list)
{
    if(list.size()==0){
        cout << "[MeshTexture::getBounds] this is not correct yet." << endl;
        return cv::Rect2f(0,0,0,0);//this is a very dirty workaround
    }
    Vector2f min=Vector2f(numeric_limits<float>::max(),numeric_limits<float>::max());
    Vector2f max=Vector2f(numeric_limits<float>::min(),numeric_limits<float>::min());
    for(size_t i=0;i<list.size();i++){
        const Vector2f &coord = list[i];
        for(int k=0;k<2;k++){
            if(coord[k]<min[k]){
                min[k]=coord[k];
            }
            if(coord[k]>max[k]){
                max[k]=coord[k];
            }
        }

    }
    cv::Rect2f rect;
    rect.height=max[1]-min[1];
    rect.width=max[0]-min[0];
    rect.x=min[0];
    rect.y=min[1];

    return rect;
}


cv::Rect2f MeshTexture::getBounds()
{
    return getBounds(texCoords);
}

void MeshTexture::scaleAndShiftTexCoordsIntoRect(const cv::Rect2f rect, const std::vector<Vector2f> &in, std::vector<Vector2f> &out)
{
    //rect.x-=0.5f;
    //rect.y-=0.5f;
    float _width=1.0f/rect.width;
    float _height=1.0f/rect.height;
    for(size_t i=0;i<in.size();i++){
        const Vector2f &coordIn = in[i];
        Vector2f &coordOut = out[i];
        coordOut[0]=(coordIn[0] - rect.x)*_width;
        coordOut[1]=(coordIn[1] - rect.y)*_height;
    }
}

void MeshTexture::scaleAndShiftTexCoordsIntoRect(cv::Rect2f rect)
{
    scaleAndShiftTexCoordsIntoRect(rect,texCoords,texCoords);

}



MeshTextureGpuHandle::MeshTextureGpuHandle(TexCoordBuffer *texCoordBuf,
                                           int nrTexCoords,
                                           TexAtlas *refAtlas,
                                           TexAtlas *dataAtlas,
                                           int width, int height,
                                           shared_ptr<TexAtlasPatch> sourceTexBeingDownloaded,
                                           shared_ptr<TexCoordBufConnector> texCoordsBeingDownloaded)
{

    //debug
    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );

    cv::Size2i size = cv::Size2i(width,height);//size of interest
    if(texCoordsBeingDownloaded==nullptr){
        coords = texCoordBuf->getBlock(nrTexCoords);
    }else{
        coords = texCoordsBeingDownloaded;
    }
    //create a reference texture
    if(refAtlas!=nullptr){
        this->refTex = refAtlas->getTexAtlasPatch(size);
    }


    if(dataAtlas!=nullptr){
        //if there is a texture on the gpu that is already being downloaded...
        //use that texture instead of creating a new one
        if(sourceTexBeingDownloaded == nullptr){
            this->tex = dataAtlas->getTexAtlasPatch(size);
        }else{
            this->tex = sourceTexBeingDownloaded;
        }

    }



    //debug
    cudaDeviceSynchronize();
    gpuErrchk( cudaPeekAtLastError() );



}

MeshTextureGpuHandle::~MeshTextureGpuHandle()
{
    //std::shared_ptr<MeshTexture> downloadTo = downloadToWhenFinished.lock();
    //instead of doing the download in a separate recycling task lets do it where it happens.
    //specifically whenever the active set changes!!!!!
    //we might want to aggregate this download

/*    if(recycler!=nullptr &&
            downloadTo!=nullptr &&
            gpuDataChanged){
        //downloadTo->TextureMostCurrent = sourceTex;


        auto downloadFunc = [](std::shared_ptr<MeshTexture> patch,
                std::shared_ptr<TexCoordBufConnector> coords,
                std::shared_ptr<TexAtlasPatch> texture){    //The assumption is that the gpu vertex and the cpu vertex are the
            //same


            if(coords==nullptr){
                assert(0);
            }
            vector<Eigen::Vector2f> coordData(coords->getSize());
            coords->download(&(coordData[0]));

            //now do the texture:
            cv::Rect2i roi = texture->getRectOfInterest();
            GLuint intFormat = texture->getTex()->getGlInternalFormat();
            uint32_t cvType = texture->getAtlasTex()->getCvType();


            //cout << "DEBUG: roi " << roi << endl;
            if(cvType!=CV_32FC4 &&
                    cvType!=CV_8UC4){
                assert(0);
            }
            cv::Mat data(roi.height,roi.width,cvType);
            if(intFormat == GL_RGBA16F){
                if(cvType != CV_32FC4){
                    assert(0);
                }


                //maybe we should also hold the texture with a mutex
                cudaSurfaceObject_t surface =
                        texture->getTex()->getCudaSurfaceObject();
                castF16SurfaceToF32Buffer(surface,
                                          roi.x,roi.y,
                                          roi.width,roi.height,
                                          (float*)data.data,4);

                cudaDeviceSynchronize();

            }else{

                //data.create(roi.height,roi.width,cvFormat);
                if(cvType!=CV_8UC4){
                    assert(0);
                }
                //texture->
                texture->downloadData(data.data);
                cudaDeviceSynchronize();
            }


            patch->cpuPayloadMutex.lock();
            patch->cpuPayload = data;
            patch->texCoords = coordData;
            patch->cpuPayloadMutex.unlock();
        };

        if(downloadTo->texCoordsMostCurrent.lock() == nullptr){
            cout << "can't download the resource. "
                    "Something i myself do not really get is wrong" << endl;
            return;
            assert(0);
        }
        if(downloadTo->TextureMostCurrent.lock() == nullptr){
            cout << "can't download the resource. "
                    "Something i myself do not really get is wrong" << endl;
            return;
            assert(0);
        }

        //ORIGINAL
        auto task = std::bind(downloadFunc,downloadTo,
                              downloadTo->texCoordsMostCurrent.lock(),//these should be valid
                              downloadTo->TextureMostCurrent.lock());
        recycler->addRecycleTask(task);
        //DEBUG:
        //downloadFunc(downloadTo,coords,sourceTex);
    }
    */
}



bool MeshTextureGpuHandle::checkRefTexDependencies() {
    //TODO: maybe check the refTex dependencies relative to a active set. !?
    if(refTexDependencies.size()==0){
        return false;//this list being zero points towards a uninitialized refTexture
    }
    for(size_t j=0; j<refTexDependencies.size();j++){
        Dependency dependency = refTexDependencies[j];
        shared_ptr<GeometryBase> dependenceGeom = dependency.geometry.lock();
        if(dependenceGeom->getMostCurrentGpuTriangles() == nullptr){
            return false;//there is no geometry on the gpu for this patch

        }
        //TODO: also check if the dependencies of the triangles themselves are met.


        //if the triangles are of the wrong/older version than we need
        if(dependenceGeom->triangles_version != dependency.trianglesVersion){
            return false;
        }

        //TODO: get these triangles in respect to the active set!?
        shared_ptr<TriangleBufConnector> dependenceTris = dependenceGeom->getMostCurrentGpuTriangles();
        //check if the triangles are at the correct index referred to by the refTex
        if(dependenceTris->getStartingIndex() != dependency.trianglePositionOnGpu){
            return false;
        }
    }
    return true;
}
/*
GpuTextureInfo MeshTextureGpuHandle::genSrcTexInfo()
{

    GpuTextureInfo info =
            sourceTex->genTexInfo(coords->getStartingIndex());

    if(refTex != nullptr){
        info.glRefTexPtrDEBUG =
                refTex->getAtlasTex()->getTex2D()->getGlHandle();
        cv::Rect2i pos =
                refTex->getPosition();
        info.refTexPosDEBUG = Vector2f(pos.x,pos.y)*(1.0f/1024.0f);
    }

    return info;

}

void MeshTextureGpuHandle::swapSrcDst()
{
    std::swap(sourceTex,destTex);
}
*/
GpuTextureInfo MeshTextureGpuHandle::genTexInfo()
{

    GpuTextureInfo info =
            tex->genTexInfo(coords->getStartingIndex());

    if(refTex != nullptr){
        info.glRefTexPtrDEBUG =
                refTex->getAtlasTex()->getTex2D()->getGlHandle();
        cv::Rect2i pos =
                refTex->getPosition();
        info.refTexPosDEBUG = Vector2f(pos.x,pos.y)*(1.0f/1024.0f);
    }

    return info;

}
/*
cudaSurfaceObject_t MeshTextureGpuHandle::getCudaSurfaceObject() {
    return 0;//this->texAtlasPatch;
}

cudaSurfaceObject_t MeshTextureGpuHandle::getCudaTextureObject() {
    return 0;
}
*/