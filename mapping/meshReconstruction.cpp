#include "meshReconstruction.h"



#include "utils/gpuNormSeg.h"

#include <iostream>

#include <gpuTex.h>
#include <shader.h>

#include "cuda/coalescedMemoryTransfer.h"
#include "cuda/xtionCameraModel.h"
#include "cuda/texCoords.h"
#include "cuda/texPatchInit.h"
#include "cuda/stdTexUpdate.h"
#include "cuda/geomUpdate.h"
#include "debugRender.h"
#include "gpu/ActiveSet.h"

#include "StitchingUtils.h"

#include "camera.h"
#include "worker.h"

using namespace cv;
using namespace std;
using namespace Eigen;
/*
#include "../icpCUDA/ICPOdometry.h"//this is not supposed to be the way how to include a library
//TODO: add icp to this project
//#include <ICPOdometry.h>
*/



void MeshReconstruction::initInGlLogicContext()
{


    if(initializingLogic.exchange(true)){
        //Apparently the logic already is beeing initialized somewhere
        //therefore we wait for this to complete:
        while(!m_glLogicInitialized){
            std::unique_lock<std::mutex> lk(m_conditionVariableMutex);
            m_conditionVariable.wait(lk);
        }

        //if completed we initialize the informationRenderer in this thread
        m_informationRenderer.initInContext();

        //since all the remaining stuff is alredy initialized wo do only the stuff
        //that is needs thread dependant initialization
        return;
    }


    //after the buffers are created we can initialize the rendering stuff:

    //initializing the render logic of some other stuff

    //cout << "right before allocating all the gpu storage" << endl;
    //getchar();

    ///New  New functionality that gets rid of all the preveous initialization stuff
    m_gpuGeomStorage.initialize();

    //this requires the geometry storage to be initialized.
    m_informationRenderer.initInContext(params.depthRes.width,
                                        params.depthRes.height,this);





    //Generating the active sets for rendering and capturing
    activeSetRendering;// = m_gpuGeomStorage.makeActiveSet();

    activeSetUpdate;// = m_gpuGeomStorage.makeActiveSet();




    texAtlasGeomLookup =
            make_shared<TexAtlas>(garbageCollector,GL_RGBA32F,GL_FLOAT,GL_RGBA,CV_32FC4,
                                  1024,&fboStorage);
    texAtlasStds =
            make_shared<TexAtlas>(garbageCollector,GL_RGBA16F,GL_FLOAT,GL_RGBA,CV_32FC4, //CV_16UC4
                                  1024,&fboStorage);
    texAtlasStds =
            make_shared<TexAtlas>(garbageCollector,GL_RGBA16F,GL_FLOAT,GL_RGBA,CV_32FC4,
                                  1024,&fboStorage);

    texAtlasRgb8Bit =
            make_shared<TexAtlas>(garbageCollector,GL_RGBA,GL_UNSIGNED_BYTE,GL_RGBA,CV_8UC4,
                                  1024,&fboStorage);



    //use integers to prevent every attempt of interpolation
    /*texAtlasSegLabels =
            make_shared<TexAtlas>(GL_RGBA32I,GL_INT,GL_RGBA_INTEGER,CV_32SC4,
                                  1024,&fboStorage);
                                  */
    //DEBUG: Don't use integers
    cout << "TODO: SETUP THE TEXTURES TO BE NON_INTERPOLATED" << endl;
    texAtlasSegLabels =
            make_shared<TexAtlas>(garbageCollector,GL_RGBA32F,GL_FLOAT,GL_RGBA,CV_32FC4,
                                  1024,&fboStorage);


    //maybe in opengl it still is better ineterpreting these as floats
    //and doing a reinterpret cast
    texAtlasSemSegLabelsWeights =
            make_shared<TexAtlas>(garbageCollector,GL_RGBA32I,GL_INT,GL_RGBA_INTEGER,CV_32SC4,
                                  1024,&fboStorage);

    GLuint test;
    glGenTextures(1,&test);
    glBindTexture(GL_TEXTURE_2D,test);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32I, 1024, 1024, 0, GL_RGBA_INTEGER,
                 GL_INT, nullptr);

    gfx::GLUtils::checkForOpenGLError("Error while creating glTexture");


    //afterwards send message to other thread:
    m_glLogicInitialized=true;
    m_conditionVariable.notify_all();
}


/*
void MeshReconstruction::reloadBorderGeometry(std::vector<std::vector<Edge>> &borderList){
    assert(0);

    //i very much fear what happens when something here changes
    set<Vertex*> vertices;
    set<VertexReference> pointRefs;
    for(size_t i=0;i<borderList.size();i++){
        for(size_t j=0;j<borderList[i].size();j++){
            //all points of related triangles
            for(size_t k=0;k<3;k++){
                VertexReference p = borderList[i][j].triangle.get()->points[k];
            }
        }
    }


    GpuVertex* gpuVertBuf = m_gpuGeomStorage.vertexBuffer->getCudaPtr();
    vector<GpuVertex> downloadedData(pointRefs.size());

    vector<GpuVertex*> gpuVerts;
    vector<VertexReference> pts;
    for(auto p : pointRefs){
        shared_ptr<MeshPatchGpuHandle> gpu = p.getPatch()->gpu.lock();
        if(gpu==nullptr){
#ifndef IGNORE_SERIOUS_BUG_2
            assert(0);
#endif
            //all the vertices fetched here should be loaded to the gpu and
            //and therefore also have vertices and a vertex buffer slot
            continue;
        }
        pts.push_back(p);
        int index =
                gpu->verticesSource->getStartingIndex() +
                p.getIndex();
        gpuVerts.push_back(&(gpuVertBuf[index]));
    }

    downloadVertices(gpuVerts,
                     &(downloadedData[0]));
    //spread the data to the vertices to do the mapping magic
    for(size_t i = 0;i<pts.size();i++){

        pts[i].get()->p = downloadedData[i].p;
        //verts[i]->p=downloadedData[i].p;
        //p->get
    }


    cout << "[ScaleableMap::reloadBorderGeometry] its over!!" << endl;
}
*/

void MeshReconstruction::cleanupGlStoragesThisThread()
{
    fboStorage.garbageCollect();
}

bool MeshReconstruction::removePatch(std::shared_ptr<MeshPatch> patch)
{
    patch->double_stitch_mutex.lock();
    for(size_t i=0;i<patch->double_stitches.size();i++){
       patch->double_stitches[i]->removeFromPatches(patch);
    }
    patch->double_stitch_mutex.unlock();

    patch->triple_stitch_mutex.lock();
    for(size_t i=0;i<patch->triple_stitches.size();i++){
        patch->triple_stitches[i]->removeFromPatches(patch);
    }
    patch->triple_stitch_mutex.unlock();

    m_patchesMutex.lock();
    for(auto it=m_patches.begin();it!=m_patches.end();++it){
        if(it->second==patch){
            m_patches.erase(it);
            m_patchesMutex.unlock();
            return true;
        }
    }
    m_patchesMutex.unlock();
    return false;

}

Mat MeshReconstruction::generateColorCodedTexture(Mat segmentation)
{

    cv::Mat colorMap(1,64*48,CV_8UC4);
    colorMap.at<cv::Vec4b>(0)=cv::Vec4b(0,0,0,0);
    colorMap.at<cv::Vec4b>(1)=cv::Vec4b(0,0,200,0);
    colorMap.at<cv::Vec4b>(2)=cv::Vec4b(0,200,0,0);
    colorMap.at<cv::Vec4b>(3)=cv::Vec4b(200,0,0,0);
    colorMap.at<cv::Vec4b>(4)=cv::Vec4b(0,200,200,0);
    colorMap.at<cv::Vec4b>(5)=cv::Vec4b(250,0,0,0);
    colorMap.at<cv::Vec4b>(6)=cv::Vec4b(200,200,200,0);
    colorMap.at<cv::Vec4b>(7)=cv::Vec4b(0,0,100,0);
    colorMap.at<cv::Vec4b>(8)=cv::Vec4b(0,100,0,0);
    colorMap.at<cv::Vec4b>(9)=cv::Vec4b(100,0,0,0);
    colorMap.at<cv::Vec4b>(10)=cv::Vec4b(0,100,100,0);
    colorMap.at<cv::Vec4b>(11)=cv::Vec4b(100,100,0,0);
    colorMap.at<cv::Vec4b>(12)=cv::Vec4b(100,100,100,0);
    int cols=0;
    int rows=0;
    for (int n=13;n<colorMap.cols;n++){
        colorMap.at<cv::Vec4b>(n)=cv::Vec4b(n/10*50,((n%10)/5)*50,(n%5)*50,0);
    }

    //TODO: take cols and rows from the segmentation Mat
    cols=segmentation.cols;
    rows=segmentation.rows;

    cv::Mat coloredImage(rows,cols,CV_8UC4);
    for(int i=0;i<rows;i++){
        for(int j=0;j<cols;j++){
            asm("#begin asm test");//later use this to check if code is properly vectorized
            EIGEN_ASM_COMMENT("begin");
            coloredImage.at<cv::Vec4b>(i,j)=colorMap.at<cv::Vec4b>(0,segmentation.at<uint64_t>(i,j)%(64*48));
            asm("#end asm test");
        }
    }


    return coloredImage;
}


Matrix4f MeshReconstruction::genDepthProjMat(){
    float fx=params.depthfxycxy[0];
    float fy=params.depthfxycxy[1];
    float cx=params.depthfxycxy[2];
    float cy=params.depthfxycxy[3];
    Matrix4f proj=Camera::projection(M_PI*0.5f,640.0f/480.0f);

    Matrix4f proj1;//one to do what has to be done anyway
    proj1 << fx, 0,  cx, 0,
            0,  fy, cy, 0,
            0,  0,  0,  -1,
            0,  0, 1,  0;


    float w=params.depthRes.width;
    float h=params.depthRes.height;
    float zmin=0.1f;
    float zmax=30.0f;
    float b=2.0f/(1.0f/zmin-1.0f/zmax);
    float a=b/zmax+1.0f;
    Matrix4f proj2;//the other one to scale everything to the normalized coordinates
    proj2 << 2.0f/(w),    0,  0,  -1.0f+1.0f/w,
            0,     2.0f/(h), 0,  -1.0f+1.0f/h,
            0,          0,  b,  a,
            0,          0,  0,  1;
    proj=proj2*proj1;
    return proj;
}
void MeshReconstruction::setActiveSetUpdate(std::shared_ptr<ActiveSet> set)
{
    activeSetUpdateMutex.lock();
    activeSetUpdate = set;
    activeSetUpdateMutex.unlock();
    //recycler->recycleSet(oldActiveSet,set,true);
}


MeshReconstruction::MeshReconstruction(GLFWwindow* context, GarbageCollector* garbageCollector,bool threaded,int depthWidth, int depthHeight,int rgbWidth,int rgbHeight)
{
    //TODO: name these setup functions consistently
    geometryUpdate.setup(this);
    texturing.meshReconstruction = this;
    //meshing.Setup(this);



    this->garbageCollector = garbageCollector;
    m_gpuGeomStorage.init(this);

    params.depthRes.width = depthWidth;
    params.depthRes.height = depthHeight;
    params.rgbRes.width = rgbWidth;
    params.rgbRes.height = rgbHeight;

    //recycler = make_shared<Recycler>(context,threaded);

    /*
    recycler = new Recycler(context,
                            threaded);
    */
    initializingLogic = false;

}


//we assume that the opengl context still exists when running the destructor
// (otherwise the opengl stuff would not behave nicely)
MeshReconstruction::~MeshReconstruction()
{
    //delete recycler;

    if(renderingActiveSetUpdateWorker != nullptr){
        //renderingActiveSetUpdateWorker->endThread();//TODO: remove this! you should not need to call this manually
        delete renderingActiveSetUpdateWorker;
    }

    //TODO: check for the textures.... are they deleted as well?
}

std::shared_ptr<MeshPatch> MeshReconstruction::genMeshPatch()
{
    m_currentMaxPatchID++;
    std::shared_ptr<MeshPatch> meshPatch(new MeshPatch(&this->octree));
            //std::make_shared<MeshPatch>(&this->octree);
    meshPatch->id=m_currentMaxPatchID;
    meshPatch->weak_self=meshPatch;
    m_patches[m_currentMaxPatchID] = meshPatch;
    return meshPatch;
}

std::shared_ptr<MeshTexture> MeshReconstruction::genMeshTexture(
        MeshTexture::Type contentType)
{

    return std::make_shared<MeshTexture>(contentType,this);
}

void MeshReconstruction::setRGBIntrinsics(Vector4f fxycxy){
    params.rgbfxycxy = fxycxy;
}

void MeshReconstruction::setDepthIntrinsics(Vector4f fxycxy){
    params.depthfxycxy = fxycxy;
}


/*
void ScaleableMap::addRGBFrame(Mat rgb)
{
    m_rgb=rgb;
}

void ScaleableMap::addDepthFrame(Mat depth)
{
    m_depth=depth;
}
*/

/*
Mat MeshReconstruction::projectDepth(Mat depth)
{
    std::cout << "IS THIS EVEN USED?????" << std::endl;
    cv::Mat points(depth.rows,depth.cols,CV_32FC4);
    float fx=params.depthfxycxy[0];
    float fy=params.depthfxycxy[1];
    float cx=params.depthfxycxy[2];
    float cy=params.depthfxycxy[3];
    for(int i=0;i<depth.rows;i++){
        for(int j=0;j<depth.cols;j++){
            float z=(1.0f/5000.0f)*(float)depth.at<uint16_t>(i,j);//these 5000.0f are coming from the TUM dataset
            points.at<Vector4f>(i,j)=Vector4f(((float)j-cx)*z/fx,((float)i-cy)*z/fy,z,1.0f);
            if(z==0 || z > params.maxDistance){
                points.at<Vector4f>(i,j)=Vector4f(NAN,NAN,NAN,NAN);
            }
        }
    }
    return points;
}
 */

bool MeshReconstruction::hasGeometry()
{
    return m_patches.size()>0;
}




void MeshReconstruction::erase()
{
    //The scheduler should be closed at this point

    vector<weak_ptr<MeshPatch>> patches;//collect weak pointers to that stuff to see what of this is still existent
    vector<shared_ptr<MeshPatch>> sharedPatches;//collect weak pointers to that stuff to see what of this is still existent
    vector<weak_ptr<MeshPatchGpuHandle>> gpuPatches;
    vector<weak_ptr<MeshTextureGpuHandle>> gpuTextures;
    int debug =0 ;
    for(auto patch :m_patches){
        debug++;
        //octree.removeObject(patch.second);//lets improve the octree system for this not to be necessary
        patches.push_back(patch.second);
        gpuPatches.push_back(patch.second->gpu);
        //patch.second.reset();
        sharedPatches.push_back(patch.second);
        shared_ptr<MeshPatchGpuHandle> gpuPatch = patch.second->gpu.lock();
        if(gpuPatch != nullptr){
            gpuTextures.push_back(gpuPatch->geom_tex);
        }
    }
    //m_patches.begin()->second.reset();


    m_patches.clear();
    activeSetUpdate.reset();
    activeSetRendering.reset();
    activeSetExpand.reset();


    for(int i=0;i<debug;i++){
        sharedPatches.pop_back();
    }

    //DEBUG:
    //all of this should be zero: TODO FIND OUT WHY, parse through all the texatlas thingis
    //TODO: follow down the octree ( objects don't seem to be deleted accordingly)
    //and then find who is holding references to the gpu structures

    //check how many textures are left now:
    int texCount =
            texAtlasStds->countTex() +
            texAtlasGeomLookup->countTex() +
            texAtlasRgb8Bit->countTex() +
            texAtlasSegLabels->countTex();
    cout << "texCount overall: " << texCount <<
         " stds " << texAtlasStds->countTex() << " lookup " <<
         texAtlasGeomLookup->countTex() << " rgb " << texAtlasRgb8Bit->countTex() <<   endl;

    int patchCount =
            texAtlasStds->countPatches() +
            texAtlasGeomLookup->countPatches() +
            texAtlasRgb8Bit->countPatches() +
            texAtlasSegLabels->countPatches();
    cout << "patchCount overall: " << patchCount <<
         " stds " << texAtlasStds->countPatches() << " lookup " <<
        texAtlasGeomLookup->countPatches() << " rgb " << texAtlasRgb8Bit->countPatches() <<   endl;

    for(int i=0;i<patches.size();i++){
        //cout << patches[i].expired() << endl;
        assert(patches[i].expired());
    }
    int activeGpuPatches=0;
    for(int i=0;i<gpuPatches.size();i++){
        activeGpuPatches += gpuPatches[i].use_count();
        //cout << gpuPatches[i].use_count() << endl; // all zero
    }
    int activeGpuTextures = 0;
    for(int i=0;i<gpuTextures.size();i++){
        activeGpuTextures += gpuTextures[i].use_count();
    }
    vector<gfx::GpuTex2D*> textures = gfx::GpuTex2D::getTexList();

    //TODO: mechanism to delete all fbos that are used in thread
    cout << "FBOs active " << getFboCountDebug() << endl; // should be zero
    cout << "GpuPatches active" << activeGpuPatches << endl;
    cout << "GpuTextures active" << activeGpuTextures << endl;
    cout << "overall tex count" << gfx::GpuTex2D::getTexCount() << endl;
    glFinish();//wait until all the commands are executed.
    texAtlasStds->countPatches();
    texAtlasStds->countTex();//Why is this bigger 0 in this case????????
    cout << "end of this analysis" << endl;
}



std::vector<std::shared_ptr<MeshPatch>> MeshReconstruction::GetAllPatches(){
    std::vector<std::shared_ptr<MeshPatch>> patches;
    for(auto patch :m_patches){
        patches.push_back(patch.second);
    }
    return patches;
}


/*
 * the Purpose of this whole file is to stick the geometry of two different frames together.
 * Therefore we plan to project the old geometry boarder into the new frame.
 * There we look for bordering pixel which are containing new geometry.
 * In this way we find the closest new geometry and are able to do proper stitching.
 *
 * Things we need:
 * Getting a already existing stitch (otherwise creating a new one
 *
 */


//http://stackoverflow.com/questions/24441631/how-exactly-does-opengl-do-perspectively-correct-linear-interpolation
//maybe do this stupid line algorithm with a templated pixel set bla
/*
void MeshReconstruction::debugBresenham(Vector2f pix0,Vector2f pix1,Mat image){

    struct Functor{
        Mat &image;

        Functor(Mat& debImage) : image(debImage){}
        void operator() (int x,int y,float f){
            int &width=image.cols;
            int &height=image.rows;
            if(x<width && y<height && x>=0 && y>=0){
                image.at<cv::Vec4b>(y,x)=cv::Vec4b(0,255,0,255);
            }
        }
    };
    Functor f(image);
    //f.image=image;

    bresenham(pix0,pix1,f);
    return;

}
*/


/*
void ScaleableMap::stitchOnEdge(Edge* edge){
    VertexInformation* pi=edge->points[0].getInfo();
    VertexInformation* pi1=edge->points[1].getInfo();
    for(int i=0;i<pi1->edges.size();i++){

    }

}
*/


TriangleReference MeshReconstruction::addTriangle(VertexReference pr1,
                                                  VertexReference pr2,
                                                  VertexReference pr3){
    //same method as below but without the debug
    assert(0);
}
//Creating a new triangle just from vertex references.
//used in the inter frame stitching process
int debugGlobalStitchTriangleCtr = 0;
TripleStitch *debugQuenstionableTriplestitch = nullptr;

TriangleReference MeshReconstruction::addTriangle(VertexReference pr1,
                                                  VertexReference pr2,
                                                  VertexReference pr3,
                                                  std::vector<std::weak_ptr<GeometryBase>> &debugNewStitches){
    //std::vector<std::shared_ptr<MeshPatch>> debugMeshpatches
    //std::vector<Edge*> debugEdges){



    //TODO: remove this debug outputy
    //cout << "This method is untested when it comes to its ability to register triangles" << endl;
    Triangle triangle;
    triangle.points[0]=pr1;
    triangle.points[1]=pr2;
    triangle.points[2]=pr3;
    triangle.debug_is_stitch = true;
    triangle.debug_nr = debugGlobalStitchTriangleCtr;
    if(triangle.debug_nr == 11757){
        cout << "bad bad triangle! debug here!" << endl;
    }
    debugGlobalStitchTriangleCtr ++;

    if( pr1.getPatch()==pr2.getPatch() && pr1.getPatch()==pr3.getPatch() ){
        cout << "ERROR [scaleableMapStitching:createnewTriangle] the points of these triangles "
                "should never be from the same patch." << endl;
        //this method is usually called during stitching so we don't expect this to be called for trianlges of this source
        assert(0);
        TriangleReference tr;
        tr.index = triangle.points[0].getPatch()->triangles.size();
        tr.container = triangle.points[0].getPatch();
        triangle.points[0].getPatch()->triangles.push_back(triangle);
        Triangle::registerTriangle(tr,true);
        triangle.points[0].getPatch()->cpu_tex_patch_ahead=true;
        return tr;
    }


    bool doubleStitch=false;
    bool tripleStitch=false;
    if(     pr1.getPatch()==pr2.getPatch() && pr1.getPatch()!=pr3.getPatch() ||//p3 is different
            pr1.getPatch()!=pr2.getPatch() && pr1.getPatch()==pr3.getPatch() ||//p2 is different
            pr1.getPatch()!=pr2.getPatch() && pr2.getPatch()==pr3.getPatch()){
        doubleStitch=true;
    }else{
        //TODO: test if it is a triple stitch
        if(pr1.getPatch()!=pr2.getPatch() &&
           pr1.getPatch()!=pr3.getPatch() &&
           pr2.getPatch()!=pr3.getPatch()){
            tripleStitch=true;
        }else{
            //in case it is neither a dual nor a triple stitch don't do anything
            assert(0);
            // return triangle;
        }
    }


    //now register the new triangle to its neighbours (even tough its done further up?)
    //Triangle::registerTriangle(triangle);

    if( doubleStitch){//p1 is different
        //find a fitting double stitch or create one
        TriangleReference tr;
        MeshPatch* patch2=nullptr;
        if(pr1.getPatch()!=pr2.getPatch()){
            patch2 =pr2.getPatch();
        }else if(pr1.getPatch()!=pr3.getPatch()){
            patch2=pr3.getPatch();
        }

        shared_ptr<DoubleStitch> stitch = pr1.getPatch()->getDoubleStitchWith(patch2);
        //add triangle to the according stitch
        if(!stitch){
            stitch = make_shared<DoubleStitch>();
            stitch->weak_self = stitch;
            stitch->patches[0]=pr1.getPatch()->weak_self;
            stitch->patches[1]=patch2->weak_self;

            pr1.getPatch()->addStitchReference(stitch);
            patch2->addStitchReference(stitch);
            debugNewStitches.push_back(stitch);

        }else{
            if(stitch->patches[0].lock().get() != triangle.points[0].getPatch()){
                cout << "JUST DON'T LET THIS HAPPEN TOO OFTEN" << endl;
                //assert(0);// the triangle is referring to the same main patch as the stitch

                if(stitch->patches[0].lock().get() == triangle.points[1].getPatch()){
                    triangle.cycle(1);
                }else if(stitch->patches[0].lock().get() == triangle.points[2].getPatch()){
                    triangle.cycle(2);
                }
                //TODO: rotate either once or twice but better get rid of this before
            }
        }

        stitch->debug_to_delete3=1;
        tr.index =stitch->triangles.size();
        tr.container = stitch.get();
        stitch->triangles.push_back(triangle);
        Triangle::registerTriangle(tr,true);
        stitch->cpu_triangles_ahead = true;

        return tr;
    }

    if(tripleStitch){
        TriangleReference tr;
        //all 3 points are from a different point
        //find a fitting triple stitch
        shared_ptr<TripleStitch> stitch =
                pr1.getPatch()->getTripleStitchWith(pr2.getPatch(),pr3.getPatch());
        if(!stitch){
            stitch = make_shared<TripleStitch>();
            stitch->weak_self=stitch;
            stitch->patches[0]=pr1.getPatch()->weak_self;
            stitch->patches[1]=pr2.getPatch()->weak_self;
            stitch->patches[2]=pr3.getPatch()->weak_self;

            pr1.getPatch()->addStitchReference(stitch);
            pr2.getPatch()->addStitchReference(stitch);
            pr3.getPatch()->addStitchReference(stitch);
            debugNewStitches.push_back(stitch);

        }else{
            MeshPatch* mainPatch = stitch->patches[0].lock().get();
            if(mainPatch != triangle.points[0].getPatch()){
                //TODO: rotate the triangle points so it fits again:
                if(mainPatch == triangle.points[1].getPatch()){
                    triangle.cycle(1);
                }else if(mainPatch == triangle.points[2].getPatch()){
                    triangle.cycle(2);
                }
                //assert(0);// the triangle is *not* referring to the same main patch as the stitch
            }

            if(mainPatch != triangle.points[0].getPatch()){
                assert(0);// the triangle is *not* referring to the same main patch as the stitch
            }
        }

        //add triangle to the according stitch
        tr.index =stitch->triangles.size();
        tr.container = stitch.get();
        stitch->triangles.push_back(triangle);
        Triangle::registerTriangle(tr,true);
        stitch->cpu_triangles_ahead = true;

        return tr;

    }


}


float pointDist(Vector2f p1,Vector2f p2){
    Vector2f dist = p1-p2;
    return dist.norm();
}



float cross(Vector2f v1,Vector2f v2){
    return v1[0]*v2[1]-v1[1]*v2[0];
}

bool isOnRightSideOfLine(Vector2f pointInQuestion,Vector2f p1,Vector2f p2,Vector2f pointOnWrongSide){
    assert(0);
    //we compare this with the cross product
    Vector2f v1=p2-p1;
    Vector2f vw=pointOnWrongSide-p1;
    Vector2f vq=pointInQuestion-p1;
    float crossWrong=cross(v1,vw);
    float crossInQuestion=cross(v1,vq);
    bool sameAsWrong=std::signbit(crossWrong)==std::signbit(crossInQuestion);
    return !sameAsWrong;

}





Mat MeshReconstruction::generateDepthFromView(int width,int height,Matrix4f pose){
    //assert(0);
    //int width=m_depth.cols;
    //int height=m_depth.rows;


    //because i don't know where else to put it: add a texture here:
    //gfx::GpuTex2D texture3(GL_RGBA,GL_RGBA,GL_UNSIGNED_BYTE,640,480,false,test.data);
    /*Mat rgba;
    cv::cvtColor(m_rgb,rgba,CV_BGR2RGBA);
    //gfx::GpuTex2D texture2(G_RGBA,GL_RGBA,GL_UNSIGNED_BYTE,rgba.cols,rgba.rows,false,rgba.data);
    std::shared_ptr<gfx::GpuTex2D> texture = std::make_shared<gfx::GpuTex2D>(GL_RGBA,GL_RGBA,GL_UNSIGNED_BYTE,rgba.cols,rgba.rows,
                                                                             true,rgba.data);
   */
    //The first step would be to do ICP, from this we would already get the projected points.
    //but right now we do not have anyof these steps
    //, therefore we project the points into space. on the gpu


    //2. Step should be the incorporation of new sensor data into the already existing map.
    float fx=params.depthfxycxy[0];
    float fy=params.depthfxycxy[1];
    float cx=params.depthfxycxy[2];
    float cy=params.depthfxycxy[3];
    Matrix4f proj=Camera::projection(M_PI*0.5f,640.0f/480.0f);

    Matrix4f proj1;//one to do what has to be done anyway
    proj1 << fx, 0,  cx, 0,
            0,  fy, cy, 0,
            0,  0,  0,  -1,
            0,  0, 1,  0;


    float w=params.depthRes.width;
    float h=params.depthRes.height;
    float zmin=0.1f;
    float zmax=30.0f;
    float b=2.0f/(1.0f/zmin-1.0f/zmax);
    float a=b/zmax+1.0f;
    Matrix4f proj2;//the other one to scale everything to the normalized coordinates
    proj2 << 2.0f/(w),    0,  0,  -1.0f+1.0f/w,
            0,     2.0f/(h), 0,  -1.0f+1.0f/h,
            0,          0,  b,  a,
            0,          0,  0,  1;
    proj=proj2*proj1;


    activeSetUpdateMutex.lock();
    shared_ptr<ActiveSet> activeSet = activeSetUpdate;
    activeSetUpdateMutex.unlock();
    //TODO: maybe update the active set
    m_informationRenderer.renderDepth(activeSet.get(),proj,pose);
    cv::Mat exGeom(height,width,CV_32FC4);//
    m_informationRenderer.getDepthTexture()->downloadData(exGeom.data);
    cv::Mat intDepth(height,width,CV_16UC1);//store the geometry back to a depthmap
    for(int i=0;i<height*width;i++){
        intDepth.at<unsigned short>(i)=exGeom.at<Eigen::Vector4f>(i)[2]*1000.0f;
    }
    return intDepth;
}





void MeshReconstruction::debugCheckTriangleNeighbourConsistency(std::vector<std::shared_ptr<MeshPatch>> patches){
    //assert(0);//TODO: remove assert
    //checking if triangle neighbourhoods are consistent and if the containers the triangles reside are valid.
    auto checkTriangle = [](Triangle& tri){
        //cout << "debug: checking triangle" << endl;
        if(!tri.points[0].getPatch()->debugIsValid()){
            assert(0);
        }

        for(int k : {0,1,2}){
            if(tri.neighbours[k].valid()){
                if(!tri.neighbours[k].ref.container->debugIsValid()){
                    assert(0);
                }
                Triangle *other = tri.neighbours[k].ref.get();
                if(other->neighbours[tri.neighbours[k].pos].ref.get() != &tri){
                    assert(0);//this seems invalid then!!!!!!"!!!!!!!!!°
                }
                //TODO: do this container check also for the stitching triangles
            }
        }

    };
    for(shared_ptr<MeshPatch> patch : patches){
        for(auto vertex : patch->vertices){
            for(int i=0;i<vertex.triangles.size();i++){
                if(!vertex.triangles[i].triangle.container->debugIsValid()){
                    assert(0);
                }
            }
        }
        for(size_t i=0; i< patch->triangles.size();i++){
            Triangle &tri = patch->triangles[i];
            checkTriangle(tri);
        }
        for(shared_ptr<DoubleStitch> stitch : patch->double_stitches){
            for(size_t i=0; i< stitch->triangles.size();i++){
                Triangle &tri = stitch->triangles[i];
                checkTriangle(tri);
            }
        }

        for(shared_ptr<TripleStitch> stitch : patch->triple_stitches){
            for(size_t i=0; i< stitch->triangles.size();i++){
                Triangle &tri = stitch->triangles[i];
                checkTriangle(tri);
            }
        }
    }
}

void MeshReconstruction::debugCheckTriangleEdgesUnregistered(std::vector<std::shared_ptr<MeshPatch>> patches) {
    assert(0);//TODO: remove this assert
    for(auto patch : patches){
        for(int i=0;i<patch->triangles.size();i++){
            for(int j : {0,1,2}){
                if(patch->triangles[i].edges[j].valid()){
                    assert(0);
                }
            }
        }
        for(auto stitch : patch->double_stitches){

            for(int i=0;i<stitch->triangles.size();i++){
                for(auto & edge : stitch->triangles[i].edges){
                    if(edge.valid()){
                        assert(0);
                    }
                }
            }
        }

        for(auto stitch : patch->triple_stitches){

            for(auto & triangle : stitch->triangles){
                for(auto & edge : triangle.edges){
                    if(edge.valid()){
                        assert(0);
                    }
                }
            }
        }
    }
}



//let this stay within the mesh
std::vector<Rect2f> MeshReconstruction::genBoundsFromPatches( std::vector<std::shared_ptr<MeshPatch> > &patches,
                                                              Matrix4f pose,
                                                              Matrix4f proj,
                                                              std::shared_ptr<ActiveSet> activeSet)
{
    Matrix4f _pose=pose.inverse();
    Matrix4f mvp = proj*_pose;

    //TexCoordGen::getPotentialTexCoordBounds(newPatches,mvp);
    vector<TexCoordGen::BoundTask> boundTasks;
    vector<shared_ptr<MeshPatch>> validMeshPatches;
    int validCount=0;
    for(shared_ptr<MeshPatch> patch : patches){
        shared_ptr<MeshPatchGpuHandle> gpuPatch = patch->gpu.lock();

        //check if patch is on gpu. Which it should be!
        if(gpuPatch==nullptr){
            assert(0);
            continue;
        }
        bool valid = true;
        //vector<TexCoordGen::BoundTask> intermediateTasks;
        TexCoordGen::BoundTask task;

        task.target_ind = validCount;


        //now iterate all stitches for neighbours
        for(shared_ptr<DoubleStitch> stitch : patch->double_stitches){
            if(stitch->patches[0].lock() != patch){
                continue;
            }
            if(!stitch->isPartOfActiveSet(activeSet.get())){
                stitch->isPartOfActiveSet(activeSet.get());
#ifndef IGNORE_SERIOUS_BUG_4
                assert(0);//This actually should not happen
                //no it acutally can happen. it also should mean that the
                //according rect would be invalid
#endif
                valid=false;
                break;
            }
            shared_ptr<TriangleBufConnector> gpuStitch = stitch->triangles_gpu.lock();
            if(gpuStitch == nullptr){
                assert(0);
                valid = false;
                break;
            }
            task.triangle_count = gpuStitch->getSize();
            task.triangles = gpuStitch->getStartingPtr();
            task.debug_type = 1;
            boundTasks.push_back(task);

        }
        if(!valid){
            continue;
        }
        for(shared_ptr<TripleStitch> stitch : patch->triple_stitches){
            if(stitch->patches[0].lock() != patch){
                continue;
            }
            if(!stitch->isPartOfActiveSet(activeSet.get())){
#ifndef IGNORE_SERIOUS_BUG_4
                assert(0); // actually this can happen and it means we should
                //not update the geometry here.
#endif
                valid=false;
                break;
            }
            shared_ptr<TriangleBufConnector> gpuStitch = stitch->triangles_gpu.lock();
            if(gpuStitch == nullptr){
                assert(0);
                valid =false;
                break;
                continue;
            }
            task.triangle_count = gpuStitch->getSize();
            task.triangles = gpuStitch->getStartingPtr();
            task.debug_type = 2;

            boundTasks.push_back(task);
        }
        if(!valid){
            continue;
        }


        task.triangle_count = gpuPatch->triangles->getSize();
        task.triangles = gpuPatch->triangles->getStartingPtr();
        task.debug_type = 0;
        if(gpuPatch->triangles->getSize() != 0){
            boundTasks.push_back(task);
        }else{
            //if the patch itself does not have triangles we do not submit this task
        }
        validMeshPatches.push_back(patch);
        validCount ++;
    }


    const vector<Rect2f> &vector = TexCoordGen::getPotentialTexCoordBounds(boundTasks, mvp,
                                                                           validMeshPatches.size(),
                                                                           m_gpuGeomStorage.patchInfoBuffer->getCudaPtr(),
                                                                           m_gpuGeomStorage.vertexBuffer->getCudaPtr());
    std::vector<cv::Rect2f> bounds = vector;
    return bounds;
}



void MeshReconstruction::clearInvalidGeometry(std::shared_ptr<ActiveSet> set, Mat depth, Matrix4f depthPose)
{
    if(set==nullptr){
        return;
    }

    int width=depth.cols;
    int height=depth.rows;

    Mat depthf;
    //TODO: maybe we want to have a different
    depth.convertTo(depthf,CV_32FC1,1.0f/5000.0f);

    std::shared_ptr<gfx::GpuTex2D> depthTex = std::make_shared<gfx::GpuTex2D>(garbageCollector,GL_R32F,GL_RED,GL_FLOAT,width,height,
                                                                              true,(void*)depthf.data);


    std::shared_ptr<gfx::GpuTex2D> dStdMaxStdMap = std::make_shared<gfx::GpuTex2D>(garbageCollector,GL_RGBA32F,GL_RGBA,GL_FLOAT,width,height,
                                                                                   true,(void*)0);

    //TODO: don't do this every time we need it but only once!!!
    generateXtionConfidenceImage(depthTex->getCudaSurfaceObject(),dStdMaxStdMap->getCudaSurfaceObject(),width,height);


    float fx=params.depthfxycxy[0];
    float fy=params.depthfxycxy[1];
    float cx=params.depthfxycxy[2];
    float cy=params.depthfxycxy[3];

    Matrix4f proj;//one to do what has to be done anyway
    proj << fx, 0,  cx, 0,
            0,  fy, cy, 0,
            0,  0,  0,  -1,
            0,  0, 1,  0;
    Matrix4f pose = depthPose;
    Matrix4f _pose = pose.inverse();
    Matrix4f proj_pose= proj*_pose;


    vector<shared_ptr<MeshPatch>> patches = set->retained_mesh_patches_cpu;

    vector<gpu::GeometryValidityChecks::VertexTask> tasks;
    for(shared_ptr<MeshPatch> patch : patches){
        shared_ptr<MeshPatchGpuHandle> gpuPatch = patch->gpu.lock();
        if(gpuPatch == nullptr){
            continue;
        }
        gpu::GeometryValidityChecks::VertexTask task;


        task.start_source = gpuPatch->vertices_source->getStartingIndex();
        task.start_dest = gpuPatch->vertices_dest->getStartingIndex();
        task.size = gpuPatch->vertices_source->getSize();
        tasks.push_back(task);
    }
    gpu::GeometryValidityChecks::checkVertexValidity(dStdMaxStdMap->getCudaSurfaceObject(),
                                                     width,height,
                                                     _pose,
                                                     proj_pose,
                                                     tasks,
                                                     m_gpuGeomStorage.vertexBuffer->getCudaPtr());//vertices on gpu
}





std::shared_ptr<ActiveSet> MeshReconstruction::genActiveSetFromPose(Matrix4f depthPose)
{

    auto start = chrono::high_resolution_clock::now();


    vector<shared_ptr<MeshPatch>> visibleSharedPatches =
            octree.getObjects(  depthPose,params.depthfxycxy,
                                Vector2f(params.depthRes.width,
                                         params.depthRes.height),
                                getMaxDistance(),//6.0f,
                                0.0f);//don't dilate the frustum in this case

    for(shared_ptr<MeshPatch> patch : visibleSharedPatches){
        //TODO: test patch
        if(!patch->isPartOfActiveSetWithNeighbours(activeSetUpdate.get())){
            //continue;
            //assert(0);//all of the new ones should be loaded
        }
        if(patch->gpu.lock() == nullptr){
            continue;
            //assert(0);
        }
        if(patch->geom_tex_patch->gpu.lock() ==nullptr){
            //assert(0);//whyyyyy. the geometry textures should be secured at
            //this point
            cv::waitKey(1);
        }
    }

    set<shared_ptr<MeshPatch>> patchesIncludingNeighbours;
    for(shared_ptr<MeshPatch> patch : visibleSharedPatches){
        patchesIncludingNeighbours.insert(patch);
        for(shared_ptr<DoubleStitch> stitch : patch->double_stitches) {
            if (stitch->patches[0].lock() != patch) {
                //continue;
            }

            shared_ptr<MeshPatch> neighbour = stitch->patches[1].lock();
            if(neighbour->isInsertedInOctree()){
                patchesIncludingNeighbours.insert(neighbour);
            }
        }
        for(shared_ptr<TripleStitch> stitch : patch->triple_stitches){
            if(stitch->patches[0].lock() != patch){
                // continue;
            }
            for(size_t i=1;i<3;i++){
                shared_ptr<MeshPatch> neighbour = stitch->patches[i].lock();
                if(neighbour->isInsertedInOctree()){
                    patchesIncludingNeighbours.insert(neighbour);
                }
            }
        }

    }


    std::vector<shared_ptr<MeshPatch>> patchesWithNeighbours(patchesIncludingNeighbours.begin(),
                                                             patchesIncludingNeighbours.end());

    shared_ptr<ActiveSet> newActiveSet = m_gpuGeomStorage.makeActiveSet(patchesWithNeighbours,
                                                                        this,true);

    //if it fails down here even though it doesn't fail up there....
    //you are doing something wrong
    for(shared_ptr<MeshPatch> patch : visibleSharedPatches){
        //TODO: test patch
        if(!patch->isPartOfActiveSetWithNeighbours(newActiveSet.get())){
            continue;
            //assert(0);//all of the new ones should be loaded
        }
        if(patch->gpu.lock() == nullptr){
            assert(0);
        }
        if(patch->geom_tex_patch->gpu.lock() ==nullptr){
            continue;
            setActiveSetUpdate(newActiveSet);
            cv::waitKey();
            //assert(0);//whyyyyy. the geometry textures should be secured at
            //this point
        }
    }
    //TODO: maybe do some uploads:

    newActiveSet->checkForCompleteGeometry();


    return newActiveSet;
}




void MeshReconstruction::initInGLRenderingContext()
{
    gfx::GLUtils::checkForOpenGLError("[ScaleableMap::initInGLRenderingContex] before");

    //to think of: We could ignore any synchronization and begin rendering only if the buffers are created
    //
    while(!m_glLogicInitialized){
        std::unique_lock<std::mutex> lk(m_conditionVariableMutex);
        m_conditionVariable.wait(lk);
    }



    m_renderPresentation.initInContext(640,480,this);

    gpuPreSeg =
            make_shared<GpuNormSeg>(
                    garbageCollector,
                    params.depthRes.width, params.depthRes.height);

    gfx::GLUtils::checkForOpenGLError("[ScaleableMap::initInGLRenderingContex] Initialization of presentation object.");


    lowDetailRenderer.initInGlContext();

    gfx::GLUtils::checkForOpenGLError("[ScaleableMap::initInGLRenderingContex] end");



}

