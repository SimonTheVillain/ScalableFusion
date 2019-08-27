#include "texAtlas.h"
#include <glUtils.h>
#include <GarbageCollector.h>

using namespace std;
using namespace cv;
using namespace Eigen;

TexAtlas::TexAtlas(GarbageCollector* garbageCollector,
        GLuint intType, GLuint type, GLuint format, int cvType, int res,
                   ThreadSafeFBOStorage *fbos)
{
    this->garbageCollector = garbageCollector;
    this->intType=intType;
    this->type=type;
    this->format=format;
    this->cvType=cvType;
    this->maxRes=res;


    textures = new SafeVector[(int)ceil(log2(float(res)))+1]();
    fboStorage = fbos;
}

TexAtlas::~TexAtlas()
{
    delete[] textures;
}

std::shared_ptr<TexAtlasPatch> TexAtlas::getTexAtlasPatch(Size2i size)
{

    float maxDim = std::max(size.width,size.height);
    if(maxDim>maxRes){
        //throw an exception!!!
        cout << "Texture exceeds maximum allowed size in this atlas structure" << endl;
        assert(0);
        throw;
    }
    int s = ceil(log2(float(maxDim)));
#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
    cout << "DEBUG: requested " << size <<", delivered " <<pow(2,s) << endl;
    cout << (int)ceil(log2(float(1024))) << endl;
#endif
    int targetSize = pow(2,s);
    shared_ptr<TexAtlasPatch> foundTexPatch = nullptr;
    SafeVector &texThisSize = textures[s];
    texThisSize.texMutex.lock();
    texThisSize.tex.erase(std::remove_if(texThisSize.tex.begin(),texThisSize.tex.end(),
            [](weak_ptr<TexAtlasTex> unlocked){return unlocked.lock() == nullptr;}),texThisSize.tex.end());
    for(size_t i=0;i<texThisSize.tex.size();i++){
        //check the existing textures if there is some place left to store data onto
        //

        //if the TexAtlasTex is valid we want to secure it in case somebody wants to delete it,
        //this is done by just letting a shared_ptr point to it
        shared_ptr<TexAtlasTex> locked = texThisSize.tex[i].lock();


        if(locked == nullptr){
            assert(0);//we filtered this before (the algorithm below would jump over the next possible entry
            //if the texture has expired because no texture patch was pointing at it anymore
            //we delete the dead reference in our list
            texThisSize.tex.erase(texThisSize.tex.begin()+i);//TODO: we are not fully iterating anymore
            continue;
            /*
            if((texThisSize.tex.begin()+i-1) ==
                    texThisSize.tex.end()){
                continue;//it seems like this was the last element and it was invalid
            }*/
        }


        foundTexPatch = locked->getFreePatch(locked);
        if(foundTexPatch!=nullptr){
            foundTexPatch->size = size;
            texThisSize.texMutex.unlock();
            return foundTexPatch;
        }//else just continue searching

    }
    if(foundTexPatch==nullptr){
#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
        cout << "DEBUG: since there is no free texture left we create a new texture" << endl;
 #endif
        //we obviously don't have any free patches left in this texture
        //so we create a new one
        shared_ptr<TexAtlasTex> tex =
                shared_ptr<TexAtlasTex>(
                    new TexAtlasTex(garbageCollector,intType,type,format,cvType,targetSize,maxRes,fboStorage));
        foundTexPatch = tex->getFreePatch(tex);
        texThisSize.tex.push_back(tex);
#ifdef GL_MEMORY_LEAK_WORKAROUND
        texRetainer.push_back(tex);
#endif


    }
    foundTexPatch->size = size;
    texThisSize.texMutex.unlock();
    return foundTexPatch;


}

int TexAtlas::countPatches()
{

    int count=0;
    size_t sizes = ceil(log2(maxRes));
    for(size_t i = 0;i<sizes;i++){
        SafeVector &texOfSize = textures[i];
        texOfSize.texMutex.lock();
        for(size_t j=0;j<texOfSize.tex.size();j++){
            shared_ptr<TexAtlasTex> tex = texOfSize.tex[j].lock();
            if(tex==nullptr){
                continue;
            }
            count += tex->countPatches();
        }
        texOfSize.texMutex.unlock();
    }
    return count;
}

int TexAtlas::countTex()
{
    int count=0;
    size_t sizes = ceil(log2(maxRes));
    for(size_t i = 0;i<sizes;i++){
        SafeVector &texOfSize = textures[i];
        texOfSize.texMutex.lock();
        for(size_t i=0;i<texOfSize.tex.size();i++){
            if(!texOfSize.tex[i].expired()){
                count ++;
            }
        }
        texOfSize.texMutex.unlock();
    }
    return count;
}

Rect2i TexAtlasTex::posFromIndex(int i)
{
    int tilesPerSide=tex->getWidth()/tileSize;
    int y=(i/tilesPerSide)*tileSize;
    int x=(i%tilesPerSide)*tileSize;
    return cv::Rect2i(x,y,tileSize,tileSize);
}

std::shared_ptr<TexAtlasPatch> TexAtlasTex::getFreePatch(std::shared_ptr<TexAtlasTex> self)
{
    occupantsMutex.lock();
    if(freeSlots.empty()){
        occupantsMutex.unlock();
        return nullptr;
    }
    int i = freeSlots.top();
    freeSlots.pop();
    /*if(occupants[i].use_count()!=0){
        cout << "[TexAtlasTex::getFreePatch] this definitely should be free! "
                "something went wrong!!! terribly" << endl;
        assert(0);
        occupantsMutex.unlock();
        return nullptr;
    }*/

    Rect2i r = posFromIndex(i);
    shared_ptr<TexAtlasPatch> patch =
            shared_ptr<TexAtlasPatch>(
                new TexAtlasPatch(self,
                                  r,
                                  i));
    //occupants[i]=patch;
    occupantsMutex.unlock();
    return patch;

}

void TexAtlasTex::freeTexSlot(int inSlot)
{
    occupantsMutex.lock();
    freeSlots.push(inSlot);
    occupantsMutex.unlock();

}

TexAtlasTex::TexAtlasTex(GarbageCollector* garbageCollector,GLuint intType, GLuint type, GLuint format,
                         int cvType, int res, int fullRes,
                         ThreadSafeFBOStorage *fboStorage)
{
    debug=false;
    debugThreadIdTexCreatedIn = std::this_thread::get_id();

    tex = make_shared<gfx::GpuTex2D>(garbageCollector,intType,format,type,
                                   fullRes,fullRes,
                                   true,
                                   nullptr);
    glFinish();//force the texture to really be created... (i ho
    debug=true;
    this->cvType = cvType;
    tileSize=res;
    int s = fullRes/res;
    //occupants.resize(s*s);
    for(size_t i=0;i<s*s;i++){
        freeSlots.push(i);
    }
    fbo = fboStorage->createGlObject();

}

TexAtlasTex::~TexAtlasTex()
{
    if(fbo != nullptr){
        delete fbo;
    }
}

void TexAtlasTex::showImage(string text)
{
    showImage(text,Rect2i(0,0,tileSize,tileSize));
}

void TexAtlasTex::showImage(string text, Rect2i cutOut)
{
    Mat image(cutOut.height,cutOut.width,cvType);//TODO: the opencv format is just a placeholder
    tex->downloadData(image.data,
                      cutOut.x,cutOut.y,
                      cutOut.width,cutOut.height);
    imshow(text,image);
}

GLuint TexAtlasTex::getFBO()
{
    if(!fbo->existsInThisThread()){
        //cout << "debug variable set?" << debug << endl;
        //making sure the texture really is created.... shit. i really hate this
        glBindFramebuffer(GL_FRAMEBUFFER,fbo->get());
        glBindTexture(GL_TEXTURE_2D,tex->getGlName());//not necessary
        glFramebufferTexture(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,tex->getGlName(),0);
        GLenum drawBuffers[1]={GL_COLOR_ATTACHMENT0};
        glDrawBuffers(1,drawBuffers);

        GLenum glErr=glCheckFramebufferStatus( GL_FRAMEBUFFER);
        if(glErr != GL_FRAMEBUFFER_COMPLETE){
            cout << "debug variable set?" << debug << endl;
            glBindTexture(GL_TEXTURE_2D,tex->getGlName());
            GLint result;
            glGetTexParameteriv( GL_TEXTURE_2D, GL_TEXTURE_RESIDENT, &result);
            cout << "is texture resident" << result << endl;
            //assert(0);//popably it is
            //do even more debug stuff:
            GLuint desperateFBO;
            glGenFramebuffers(1,&desperateFBO);
            glBindFramebuffer(GL_FRAMEBUFFER,desperateFBO);
            glFramebufferTexture(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,tex->getGlName(),0);
            GLenum drawBuffers[1]={GL_COLOR_ATTACHMENT0};
            glDrawBuffers(1,drawBuffers);
            //if after this, the error still occurs it probably is on the
            //texture and not on the framebuffer itself.
            gfx::GLUtils::checkOpenGLFramebufferStatus("TexAtlasTex::getFBO");
            assert(0);


        }
        std::thread::id id = std::this_thread::get_id();
        /*cout << "this thread " << id <<
                "the tex creation thread " << debugThreadIdTexCreatedIn << endl;
        */
        gfx::GLUtils::checkOpenGLFramebufferStatus("TexAtlasTex::getFBO");
        if(id!=debugThreadIdTexCreatedIn){
            cout << "this is notable! at least if this does not crash!," << endl <<
                    "It means that FBO and textures can be created "
                    "in different threads" << endl;
        }
        //GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT (WHY IS IT THIS ERROR MESSAGE???)
        //GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT //on occasion
        glBindFramebuffer(GL_FRAMEBUFFER,0);//TODO: do we really need this?

    }
    return fbo->get();
    /*
    if(FBO==0){

        //create a FBO
        glGenFramebuffers(1,&FBO);
        glBindFramebuffer(GL_FRAMEBUFFER,FBO);
        glFramebufferTexture(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,tex->getGlName(),0);
        GLenum drawBuffers[1]={GL_COLOR_ATTACHMENT0};
        glDrawBuffers(1,drawBuffers);
        glBindFramebuffer(GL_FRAMEBUFFER,0);//TODO: do we really need this?
    }
    return FBO;
    */
}

int TexAtlasTex::countPatches()
{
    int width = tex->getWidth()/tileSize;
    return width*width-freeSlots.size();
}


TexAtlasPatch::TexAtlasPatch(std::shared_ptr<TexAtlasTex> &tex, Rect2i &pos,int index)
{
    this->tex=tex;
    this->pos=pos;
    this->indexInAtlasTex = index;
}

TexAtlasPatch::~TexAtlasPatch()
{

    tex->freeTexSlot(indexInAtlasTex);
}

std::shared_ptr<gfx::GpuTex2D> TexAtlasPatch::getTex()
{
    return tex->tex;
}

Rect2i TexAtlasPatch::getPosition()
{
    return pos;
}

GLuint TexAtlasPatch::getFBO()
{
    return tex->getFBO();
}

void TexAtlasPatch::setViewport()
{
    setViewport(Size2i(pos.width,pos.height));
}

void TexAtlasPatch::setViewport(Size2i size)
{
    glViewport(pos.x,pos.y,size.width,size.height);
}

bool TexAtlasPatch::downloadData(void *data, Rect2i rect)
{
    int width = tex->tex->getWidth();
    int height = tex->tex->getWidth();

    if(rect.x+rect.width >= width || rect.y+rect.height >= height){
        return false;
    }


    tex->tex->downloadData(data,
                           rect.x,rect.y,
                           rect.width,rect.height);

    return true;


}

bool TexAtlasPatch::uploadData(void *data, Rect2i rect)
{
    int width = tex->tex->getWidth();
    int height = tex->tex->getWidth();

    if(rect.x+rect.width >= width || rect.y+rect.height >= height){
        return false;
    }

    tex->tex->uploadData(data,
                           rect.x,rect.y,
                           rect.width,rect.height);

    return true;

}

void TexAtlasPatch::showImage(string text)
{
    showImage(text,Size2i(pos.width,pos.height));
}

void TexAtlasPatch::showImage(string text, Size2i size)
{
    tex->showImage(text,Rect2i(pos.x,pos.y,size.width,size.height));
}

GpuTextureInfo TexAtlasPatch::genTexInfo(Size2i size, int texCoordStartingIndex)
{
    int width = tex->tex->getWidth();
    int height = tex->tex->getHeight();

    GpuTextureInfo texInfo;
    texInfo.texCoordStartInd = texCoordStartingIndex;
    texInfo.glTexPointer = tex->tex->getGlHandle();
    texInfo.pos = Vector2f(float(pos.x)/float(width),float(pos.y)/float(height));
    texInfo.size = Vector2f(float(size.width)/float(width),float(size.height)/float(height));
    texInfo._size = Vector2f(1.0f/texInfo.size[0],1.0f/texInfo.size[1]);
    return texInfo;
}

GpuTextureInfo TexAtlasPatch::genTexInfo(int texCoordStartingIndex)
{
    Rect2i roi = getRect();
    return genTexInfo(roi.size(),texCoordStartingIndex);
}

cudaSurfaceObject_t TexAtlasPatch::getCudaSurfaceObject() {
    return getTex()->getCudaSurfaceObject();
}

cudaTextureObject_t TexAtlasPatch::getCudaTextureObject() {
    return getTex()->getCudaTextureObject();
}
uint64_t TexAtlasPatch::getGlHandle(){
    return getTex()->getGlHandle();
}

