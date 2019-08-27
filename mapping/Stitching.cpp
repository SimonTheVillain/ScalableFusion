//
// Created by simon on 8/2/19.
//

#include "Stitching.h"
#include "StitchingUtils.h"
#include <Eigen/Eigen>
//#include <initializer_list>
#include "meshReconstruction.h"
#include "cuda/coalescedMemoryTransfer.h"

using namespace std;
using namespace Eigen;
using namespace cv;

//#define SHOW_WINDOW_STITCHING
void Stitching::rasterBorderGeometry(std::vector<std::vector<Edge>> &borders,
                          Eigen::Matrix4f view, Eigen::Matrix4f proj, cv::Mat geometry){
    Matrix4f _view=view.inverse();
    Mat debug = geometry.clone();//TODO: get rid of this
    for(size_t i=0;i<borders.size();i++) {
        for (size_t j = 0; j < borders[i].size(); j++) {
            Edge *edge = &borders[i][j];
            rasterLineGeometry(_view, proj, edge, geometry, debug);
        }
    }
}


void Stitching::rasterLineGeometry(Eigen::Matrix4f _view, Eigen::Matrix4f proj, Edge* edge, cv:: Mat geometry, cv::Mat debug){

    //first of all calculate the two points in screen space
    Matrix4f p_v=proj*_view;
    int width=geometry.cols;
    int height=geometry.rows;

    //do a proper interpolation between depth values:
    //http://stackoverflow.com/questions/24441631/how-exactly-does-opengl-do-perspectively-correct-linear-interpolation
    VertexReference pi = edge->points(1);
    Vector4f P0=pi.get()->p;
    Vector4f projected0=p_v*P0;
    Vector2f pix0=Vector2f( projected0[0]/projected0[3],
                            projected0[1]/projected0[3]);
    float w0=projected0[3];
    Vector4f point0 =_view*P0;//transform the point into camera space

    pi = edge->points(0);
    Vector4f P1=pi.get()->p;
    Vector4f projected1=p_v*P1;
    Vector2f pix1=Vector2f( projected1[0]/projected1[3],
                            projected1[1]/projected1[3]);

    float w1=projected1[3];
    Vector4f point1 =_view*P1;

    struct Functor{
        cv::Mat &geometry;
        Vector4f &point0;
        Vector4f &point1;
        float &w0,&w1;
        cv::Mat &debug;
        Functor(Mat &geometry,Vector4f &point0,Vector4f &point1,float &w0,float &w1,Mat &debug) :
                geometry(geometry), point0(point0), point1(point1), w0(w0), w1(w1),debug(debug){}
        void operator() (int x,int y,float t){
            int width=geometry.cols;
            int height=geometry.rows;
            if(x<width && y<height && x>=0 && y>=0){
                Vector4f fragPos=interpolatePerspectific(point0,point1,w0,w1,t);

                float depth=fragPos[2];
                float currentDepth=geometry.at<Vector4f>(y,x)[2];
                if(currentDepth>depth || isnan(currentDepth)){
                    if(isnan(fragPos[0])){
                        cout << "this should never occur" << endl;
                        assert(0);
                    }
                    geometry.at<Vector4f>(y,x)=fragPos;//placeholder unfortunately
                    debug.at<Vector4f>(y,x) = Vector4f(1,1,1,1);
                }else{

                    debug.at<Vector4f>(y,x) = Vector4f(1,1,1,1);
                }
            }
        }
    };
    if(pix0.norm()>10000 ||
       pix1.norm()>10000){
        //this is a poor fix to ignore points which get projected far
        //outside of the view plane
        return;
    }
    Functor f(geometry,
              point0,point1,
              w0,w1,
              debug);
    bresenham(pix0,pix1,f);

}


void Stitching::genBorderList(std::vector<std::shared_ptr<MeshPatch>> &patches,
                   std::vector<std::vector<Edge>> &borderList,
                   Eigen::Matrix4f debugProj_pose){
    int debugi=0;
    int debugj=0;
    float s = 2.0f;
    cv::Mat debugMat(480*(s+1),640*(s+1),CV_8UC3);
    debugMat.setTo(0);
    //cv::imshow("edges",debugMat);
    //cv::waitKey(1);

    unordered_set<shared_ptr<MeshPatch>> patchSet(patches.begin(),patches.end());
    unordered_set<GeometryBase*> baseSet;

    //TODO: remove
    //DEBUG: to enxure readability we will create a vector
    vector<MeshPatch*> debugPatchesOrdered;
    for(int i=0;i<patches.size();i++){
        debugPatchesOrdered.push_back(patches[i].get());
    }
    sort(debugPatchesOrdered.begin(), debugPatchesOrdered.end(),
         [](const MeshPatch*  a, const MeshPatch*  b)
         {
             return a->id > b->id;
         });

    //iterate over all patches and triangles
    for(shared_ptr<MeshPatch> patch : patches){
        //iterate over all triangles belonging to this patch and its stitches
        //in case of an empty unused edge we follow it in both directions
        baseSet.insert(patch.get());
        patch->doubleStitchMutex.lock();
        for(shared_ptr<DoubleStitch> &stitch : patch->doubleStitches){

            if( patchSet.count(stitch->patches[0].lock()) &&
                patchSet.count(stitch->patches[1].lock())){
                baseSet.insert(stitch.get());
            }
        }
        patch->doubleStitchMutex.unlock();

        patch->tripleStitchMutex.lock();
        for(shared_ptr<TripleStitch> stitch : patch->tripleStitches){
            if( patchSet.count(stitch->patches[0].lock()) &&
                patchSet.count(stitch->patches[1].lock()) &&
                patchSet.count(stitch->patches[2].lock())) {
                baseSet.insert(stitch.get());
            }

        }
        patch->tripleStitchMutex.unlock();

    }

    cv::Point2i pcv1old;
    cv::Point2i pcv2old;
    auto renderEdge = [&](Edge& edge){
        return;
        if(borderList.size() != 49){
            return;
        }
        Vector4f p1 = edge.points(0).get()->p;
        p1 = debugProj_pose * p1;
        p1 = p1 * (1.0f/p1[3]);
        Vector4f p2 = edge.points(1).get()->p;
        p2 = debugProj_pose * p2;
        p2 = p2 * (1.0f/p2[3]);
        //Now draw a line via OpenCV
        cv::Point2i pcv1(p1[0]*s*3.0f,p1[1]*s*3.0f);
        cv::Point2i pcv2(p2[0]*s*3.0f,p2[1]*s*3.0f);
        //cout << p1 << endl;
        //pcv1.x = pcv1.y = 0;
        //pcv2.x = pcv2.y= 1000;
        cv::line(debugMat,pcv1old,pcv2old,cv::Scalar(255,0,0));
        cv::line(debugMat,pcv1,pcv2,cv::Scalar(0,0,255));
        pcv1old = pcv1;
        pcv2old = pcv2;
        cv::imshow("edges",debugMat);
        cv::waitKey();
    };

    auto addBorderStrip = [&](Edge initialEdge){
        //cout << "STARTING NEW BORDER" << endl;

        //get other edge obviously does not cut it
        //for(int i=0;i<2;i++){
        Edge currentEdge = initialEdge;
        renderEdge(currentEdge);
        //stack_vector<Vertex::VertexInTriangle,16> &triangles = initialEdge.points(i).get()->triangles;
        bool followingBorder=true;
        bool ranInCircle= false;
        int debug=0;
        vector<Edge> forward = borderList.back();
        vector<Edge> backward;
        Edge debugLastEdge =currentEdge;
        while(followingBorder){
            debug++;
            currentEdge.getOtherEdge(0,currentEdge,borderList);
            renderEdge(currentEdge); //debug

            if(baseSet.count(currentEdge.triangle.container) != 1){
                followingBorder = false;//if the new edge is attached to a patch outside we abort.
                continue;
            }
            if(currentEdge.isRegistered){
                followingBorder=false; //we ran in circle
                if(!currentEdge.equalTo(initialEdge)){
                    //one edge should never hit anoter edge within the same
                    for(int i=0;i<borderList.back().size();i++){
                        if(currentEdge.equalTo(borderList.back()[i])){
                            assert(0);//intersecting the current border
                        }
                    }
                    debugLastEdge.getOtherEdge(0,currentEdge,borderList);
                    assert(0);//only the initial triangle should be registered
                }else{
                    ranInCircle = true;
                }
            }else{
                //TODO: remove registration
                //register the new edge:
                currentEdge.registerInTriangle(borderList.size()-1,borderList.back().size());
                currentEdge.debug=borderList.size()-1;
                borderList.back().push_back(currentEdge);//Edge edge(triRef,0);
                debugj++;
                forward.push_back(currentEdge);
                if(currentEdge.equalTo(initialEdge)){
                    assert(0);//the initial edge should be registered so we should never be in this branch
                }

            }
            debugLastEdge=currentEdge;
        }
        int debugBla = 0;
        if(!ranInCircle){
            //if we didn't follow the border in a full loop we start again from the initial triangle
            followingBorder = true;
            currentEdge = initialEdge;
            while(followingBorder){
                currentEdge.getOtherEdge(1,currentEdge,borderList);//other direction
                renderEdge(currentEdge);
                if(baseSet.count(currentEdge.triangle.container) !=1){
                    followingBorder = false;//if the new edge is attached to a patch outside we abort.
                    continue;
                }
                if(currentEdge.isRegistered){

                    if(!currentEdge.equalTo(initialEdge)){
                        for(int i=0;i<borderList.size();i++){
                            for(int j=0;j<borderList[i].size();j++){
                                renderEdge(borderList[i][j]);

                            }
                        }
                        for(int i=0;i<forward.size();i++){
                            renderEdge(forward[i]);
                        }
                        for(int i=0;i<backward.size();i++){
                            renderEdge(backward[i]);
                        }
                        cv::waitKey(1);
                        int debugSize = borderList.size();
                        Triangle* currentTriangle = currentEdge.triangle.get();
                        Triangle* lastTriangle = debugLastEdge.triangle.get();
                        Edge newEdge;
                        debugLastEdge.getOtherEdge(1,newEdge,borderList);

                        Edge newEdge2;
                        debugLastEdge.getOtherEdge(1,newEdge2,borderList);


                        assert(0);// this should not happen!!!!!
                    }else{
                        assert(0);
                    }
                }else{
                    //register the new edge:
                    currentEdge.registerInTriangle(borderList.size()-1,borderList.back().size());
                    currentEdge.debug=borderList.size()-1;
                    borderList.back().push_back(currentEdge);//Edge edge(triRef,0);
                    backward.push_back(currentEdge);
                    if(debugBla==150){
                        //cout << "this is where the bullshit happens" << endl;
                    }
                    debugBla++;
                    debugj++;
                    if(currentEdge.equalTo(initialEdge)){
                        assert(0);//the
                    }

                }
            }
            int debugBackwardSize = backward.size();
            int debugForwardSize= borderList.back().size();
            std::reverse(backward.begin(),backward.end());
            backward.insert(backward.end(),forward.begin(),forward.end());
            //but why is this????
            /*
            if(backward[1610].triangle.get() == backward[2188].triangle.get() &&
                backward[1610].pos == backward[2188].pos){
                assert(0);//why are they the same? they should not be!!!!
            }
             */
            borderList.back() = std::move(backward);
        }
        //changing the registration for the edges
        for(size_t i=0;i<borderList.back().size();i++){
            borderList.back()[i].registerInTriangle(borderList.size()-1,i);
        }

        //}

    };

    int debugCount = 0;
    //return borderList;
    for(GeometryBase* geomBase : debugPatchesOrdered){//baseSet){
        debugCount++;
        for(size_t j=0;j<geomBase->triangles.size();j++){
            //geomBase->triangles[j]->appendBorderEdges(edges);
            Triangle &triangle = geomBase->triangles[j];
            TriangleReference triRef(geomBase,j);
            if(!triangle.neighbours[0].valid()){
                //TODO: also check for the according neighbour not to have a valid edge
                //Edge* edge = new Edge(triangle->points[0],triangle->points[1],triangle,triangle->points[2]);
                if(!triangle.edges[0].valid()){
                    //register the new edge:
                    triangle.edges[0].borderInd = borderList.size();
                    borderList.emplace_back();//create a new list of edges (read border) at its end
                    triangle.edges[0].ind = borderList.back().size();//obviously
                    borderList.back().emplace_back(triRef,0 );//Edge edge(triRef,0);
                    borderList.back().back().isRegistered=true;
                    borderList.back().back().debug = -borderList.size() +1;
                    Edge &edge = borderList.back().back();

                    debugj=0;
                    addBorderStrip(edge);//recursively append this edge!!!!
                    debugi++;
                }
            }
            if(!triangle.neighbours[1].valid()){
                if(!triangle.edges[1].valid()){
                    //register the new edge:
                    triangle.edges[1].borderInd = borderList.size();
                    borderList.emplace_back();//create a new list of edges (read border) at its end
                    triangle.edges[1].ind = borderList.back().size();
                    borderList.back().emplace_back(triRef,1);//Edge edge(triRef,0);
                    borderList.back().back().isRegistered=true;
                    borderList.back().back().debug = -borderList.size() +1;
                    Edge &edge = borderList.back().back();

                    debugj=0;
                    addBorderStrip(edge);//recursively append this edge!!!!
                    debugi++;
                }
                //TODO: also check for the according neighbour not to have a valid edge
                //assert(0);
                //Edge* edge = new Edge(triangle->points[0],triangle->points[2],triangle,triangle->points[1]);
                //borderList.emplace_back();//create a new list of edges at its end
                //addBorderStrip(edge);
            }
            if(!triangle.neighbours[2].valid()){
                if(!triangle.edges[2].valid()){
                    //register the new edge:
                    triangle.edges[2].borderInd = borderList.size();
                    borderList.emplace_back();//create a new list of edges (read border) at its end
                    triangle.edges[2].ind = borderList.back().size();
                    borderList.back().emplace_back(triRef,2);//Edge edge(triRef,0);
                    borderList.back().back().isRegistered=true;
                    borderList.back().back().debug = -borderList.size() +1;
                    Edge &edge = borderList.back().back();

                    debugj=0;
                    addBorderStrip(edge);//recursively append this edge!!!!
                    debugi++;
                }
                //TODO: also check for the according neighbour not to have a valid edge
                //assert(0);
                //Edge* edge = new Edge(triangle->points[1],triangle->points[2],triangle,triangle->points[0]);
                //borderList.emplace_back();//create a new list of edges at its end
                //addBorderStrip(edge);
            }
        }
    }

}
void Stitching::reloadBorderGeometry(std::vector<std::vector<Edge>> &borderList){
    //i very much fear what happens when something here changes

    MeshReconstruction* mesh = meshReconstruction;

    set<Vertex*> vertices;
    set<VertexReference> pointRefs;
    for(size_t i=0;i<borderList.size();i++){
        for(size_t j=0;j<borderList[i].size();j++){
            //all points of related triangles
            for(size_t k=0;k<3;k++){
                VertexReference p = borderList[i][j].triangle.get()->points[k];
            }
            //only edge points:
            /*
            for(size_t k=0;k<2;k++){
                VertexReference p = borderList[i][j].points(k);
                pointRefs.insert(p);//this will definitely fail
            }
             */

        }
    }


    GpuVertex* gpuVertBuf = mesh->m_gpuGeomStorage.vertexBuffer->getCudaPtr();
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

        /*cout << "original: " << pts[i].getVertex()->p << endl <<
                "updated:" << downloadedData[i].p << endl;
        */
        pts[i].get()->p = downloadedData[i].p;
        //verts[i]->p=downloadedData[i].p;
        //p->get
    }


    cout << "[Stitching::reloadBorderGeometry] its over!!" << endl;
}
//TODO: also download the geometry of such list
void Stitching::freeBorderList(std::vector<std::vector<Edge>> &borderList){
    for(size_t i=0;i<borderList.size();i++){
        for(size_t j=0;j<borderList[i].size();j++){
            Edge &edge = borderList[i][j];
            edge.unregister();
        }
    }

    borderList.clear();
}


void Stitching::stitchOnBorders(std::vector<std::vector<Edge> > &borders, Eigen::Matrix4f view, Eigen::Matrix4f proj,
                                         cv::Mat stdProj, cv::Mat geomProjM, cv::Mat newGeomM, cv::Mat newStd,
                                         cv::Mat debugColorCodedNewSegmentation, cv::Mat newSegPM,
                                         cv::Mat newPtIndM,
                                         std::vector<std::weak_ptr<GeometryBase>> &debugListNewEdges){
    MeshReconstruction *map = meshReconstruction;

    Matrix4f _view=view.inverse();
    Matrix4f p_v=proj*_view;
    int width=geomProjM.cols;
    int height=geomProjM.rows;

    Vector2i debugLastPix(0,0);
    auto isCW = [] (Vector2f p1,Vector2f p2, Vector2f p3) {
        Vector2f v1 = p2-p1;
        Vector2f v2 = p3-p1;
        return v1[0]*v2[1]-v1[1]*v2[0] > 0;
    };
    auto isConnected = [] (VertexReference p1,VertexReference p2){
        Vertex* v1 = p1.get();
        for(int i=0;i<v1->triangles.size();i++){
            //iterating over all the triangles connected to vertex 1
            Triangle* tri = v1->triangles[i].triangle.get();
            for(int k=0;k<3;k++){
                if(tri->points[k].isEqualTo(p2)){
                    return true;
                }
            }
        }
        return false;
    };

    auto isOpenEdge = [] (VertexReference p1,VertexReference p2){
        Vertex* v1 = p1.get();
        for(int i=0;i<v1->triangles.size();i++){
            //iterating over all the triangles connected to vertex 1
            Triangle* tri = v1->triangles[i].triangle.get();
            for(int k=0;k<3;k++){
                if(tri->points[k].isEqualTo(p2)){
                    //found a triangle lets calculate the index the potential edge
                    int ind = std::min(k,v1->triangles[i].indInTriangle);
                    int inds[2] = {ind,
                                   std::max(k,v1->triangles[i].indInTriangle)};//is there a +1 missing?
                    if(inds[0] == 0 && inds[1] == 2){
                        ind = 2;
                    }
                    if(tri->neighbours[ind].valid()){
                        return false;
                    }else{
                        //the triangle does not have any neighbours between points one and two:
                        return true;
                    }

                }
            }
        }
#ifdef SHOW_TEXT_STITCHING
        cout << "DEBUG: these two points are not connected by any triangle at all." << endl;
#endif
        return false;
    };
    //TODO implement and reduce overall file size

    auto isConnectedClosedEdge = [] (VertexReference p1,VertexReference p2){
        return false;
    };

    auto project3f = [&](Vector4f p){
        Vector3f result;

        Vector4f point =_view*p;
        Vector4f projected=p_v*p;
        float w=projected[3];
        Vector2f pix = Vector2f(   projected[0]/w,
                                   projected[1]/w);
        result =  Vector3f(pix[0],pix[1],point[2]);
        return result;
    };
    auto project2i = [&](Vector4f p){
        Vector3f a = project3f(p);

        Vector2i result(round(a[0]),round(a[1]));//TODO: improve rounding to nearest
        return result;
    };
    auto pos2pix2i = [&](Vector3f p){
        Vector2i result(round(p[0]),round(p[1]));//TODO: improve rounding to nearest
        return result;
    };
    auto getNextOpenEdge = [] (VertexReference p,Edge &resultingUnregisteredEdge){
        if(p.get()->encompassed()){
            assert(0);
        }
        //To prevent this from outputting something of the existing geometry, this has to be called before stitching
        Vertex* v = p.get();
        for(int i=0;i<v->triangles.size();i++){
            int ind = v->triangles[i].indInTriangle;
            //ind--; // decrement if we want to go to the last edge
            if(ind == -1){
                ind = 2;
            }
            Triangle* triangle = v->triangles[i].triangle.get();
            if(!triangle->neighbours[ind].valid()){
                Edge e;
                e.triangle = v->triangles[i].triangle;
                e.pos = ind;
                resultingUnregisteredEdge = e;
                return true;
            }
        }
        return false;
    };
    //return;


    auto isTrianglePossible = [&] (std::array<VertexReference, 3> p,
                                   std::array<bool, 3> test = {true,true,true},
                                   std::array<bool, 3> testOrientation = {true,true,true}){

        for(size_t i : {0,1,2}){
            size_t i2 = i == 2 ? 0 : i+1;
            if(!test[i]){
                continue;
            }
            if(isConnected(p[i],p[i2])){

                if(!isOpenEdge(p[i],p[i2])){
                    return false;
                }
                if(!testOrientation[i]){
                    continue;
                }
                Vertex* thisVert = p[i].get();
                for(int l = 0; l< thisVert->triangles.size();l++){
                    if(thisVert->triangles[l].triangle.get()->containsPoint(p[i2])){
                        int ind1 = thisVert->triangles[l].indInTriangle;
                        int ind2 =  thisVert->triangles[l].triangle.get()->getPointIndex(p[i2]);
                        if(ind1!= 0){
                            //cout << "this is astonishing" << endl;
                        }
                        if((ind1>ind2 || (ind1==0 && ind2==2))){//this query is just plain wrong
                            //return false;
                        }
                        if( (ind2==1 && ind1==0) ||
                            (ind2==2 && ind1==1) ||
                            (ind2==0 && ind1==2)){ // (ind2-ind1) == 1 || (ind2-ind1) == -2
                            return false;
                        }
                        break;//break the current loop
                    }
                }

            }
        }
        return true;
    };
    cv::Mat debug(height,width,CV_8UC4);//please don't really use this
    debug.setTo(cv::Scalar(16,16,16,255));

    int debugTriangleCount=0;

    for(int i = 0; i <borders.size();i++){

        //depending on if the last edge had a triangle or not it can be pretty clear if we want to create a new triangle
        // or not
        bool veryFirstPoint = true;
        VertexReference firstPr = borders[i][0].points(1);
        VertexReference lastPr;
        Vector2f lastPix;
        bool triangleCreatedLastEdge = false;

        //VertexReference debugLastPr1;
#ifdef SHOW_TEXT_STITCHING
        cout << "running along new border" << endl;
#endif

        //TODO: implement this sewing algorithm that follows edges on both sides of the seam!!!!
        bool sewingMode = false;
        VertexReference currentSewingPr;
        Vertex* currentSewingP;
        VertexReference lastSewingPr;
        Vertex* lastSewingP;
        Edge currentSewingEdge;
        Vector2i currentSewingPix(-1,-1);
        Vector2i lastPixi(-1,-1);

        auto checkAndStartSewing = [&](VertexReference startPr){
            Edge otherEdge;
            if(getNextOpenEdge(startPr,otherEdge)) {
                VertexReference vr = otherEdge.points(1);
                Vector2i pix = project2i(otherEdge.points(1).get()->p);
                //TODO: check if the next point really belongs to the newly added geometry
                MeshPatch *patch = newSegPM.at<MeshPatch *>(pix[1], pix[0]);
                if (patch == vr.getPatch()) {
                    //if the other side of the stitch is not just a single vertex
#ifdef SHOW_TEXT_STITCHING
                    cout << "starting sewing process" << endl;
#endif
                    //Starting sewing process!!!!!
                    sewingMode = true;
                    currentSewingEdge = otherEdge;
                    lastSewingPr = currentSewingEdge.points(0);
                    lastSewingP = lastSewingPr.get();
                    currentSewingPr = currentSewingEdge.points(1);
                    currentSewingP = currentSewingPr.get();
                    currentSewingPix = project2i(currentSewingP->p);
                }
            }
        };

        for(int j=0;j<borders[i].size();j++){
#ifdef SHOW_TEXT_STITCHING
            cout << "starting new edge" << endl;
#endif

            Edge &edge = borders[i][j];

            if(edge.triangle.get()->edges[edge.pos].get(borders) != &edge){
                assert(0);
            }
            if(edge.alreadyUsedForStitch){
                assert(0);//shouldn't every edge be touched only once?
            }
            //Triangle *tri = edge.triangle.get();

            VertexReference pr0 = edge.points(1);//these are mixed because we screwed up the order at one point
            Vector4f P0=pr0.get()->p;
            Vector4f point0 =_view*P0;//transform the point into camera space
            Vector4f projected0=p_v*P0;
            float w0=projected0[3];
            Vector2f pix0 = Vector2f(   projected0[0]/w0,
                                        projected0[1]/w0);
            Vector2i pix0i=Vector2i(std::round(pix0[0]),std::round(pix0[1]));

            VertexReference pr1 = edge.points(0);//these are mixed because we screwed up the order at one point
            Vector4f P1=pr1.get()->p;
            Vector4f point1 =_view*P1;
            Vector4f projected1=p_v*P1;
            float w1=projected1[3];
            Vector2f pix1 = Vector2f(   projected1[0]/w1,
                                        projected1[1]/w1);
            Vector2i pix1i=Vector2i(std::round(pix1[0]),std::round(pix1[1]));

            VertexReference pr2 = edge.oppositePoint();//we need the third point of the triangle here!
            Vector4f P2=pr2.get()->p;
            Vector4f point2 =_view*P2;
            Vector4f projected2=p_v*P2;
            float w2=projected2[3];
            Vector2f pix2 = Vector2f(   projected2[0]/w2,
                                        projected2[1]/w2);
            Vector2i pix2i=Vector2i(std::round(pix2[0]),std::round(pix2[1]));


            //TODO: remove this! This is here to avoid the issues that appear when we might close a loop
            if(j==borders[i].size()-1){
                if(firstPr.isEqualTo(pr1)){
                    //assert(0);//this is still triggered
                    //continue;
                }
            }



            Vector2f firstPix;

            // skip triangle if it is not facing camera. (backfacing triangle)
            /*
            if(!isCW(pix0,pix1,pix2)){
                //assert(0);//there is valid reasons this could happen
                //we skip this edge and go to the next one...
                triangleCreatedLastEdge = false;//we have to do this here due to our bad programming.
                //(continue in middle of a block)
                continue;
            }
             */

            bool veryFirstEdgeMade=false;//TODO: does this belong one level up?
            Edge veryFirstEdge;
            bool edgeMade=false;
            Edge lastEdge;


            bool firstPoint = true;
            int nrTrianglesThisEdge=0;



            auto func = [&] (int x,int y, float t) {
#ifdef SHOW_TEXT_STITCHING
                cout << "another pixel (" << x << ", " << y << ")" << endl;
#endif
                //TODO: guarantee that lastPix always is set!!!!!!
                if(sewingMode){

                    Vector4f fragPos=interpolatePerspectific(point0,point1,w0,w1,t);
                    float depthInterpolated = fragPos[2];
                    bool sewingEdgeMade = false;

                    if(x==lastPixi[0] && y == lastPixi[1]){
#ifdef SHOW_TEXT_STITCHING
                        cout << "this pixel was last pixel" << endl;
#endif
                        //we want at least one pixel to change
                        //if this is the first pixel of this edge then we want to give triangle creation another chance:
                        if(!edgeMade){

                            if(!isTrianglePossible({lastSewingPr,pr0,pr1})){
                                sewingMode= false;
                                return;
                            }
                            map->addTriangle(lastSewingPr, pr0,pr1,debugListNewEdges);
                            nrTrianglesThisEdge++;
#ifdef SHOW_TEXT_STITCHING
                            cout << "e" << endl;
#endif
                            lastPr = lastSewingPr;
                            edgeMade=true;
                            sewingEdgeMade=true;

                        }

                        //maybe we also need other stuff
                        //at least we skip this pixel
                        return;
                    }
                    if(abs(x-currentSewingPix[0]) >1 || abs(y-currentSewingPix[1]) > 1){
                        //if the sewing pix is not within the a one pixel radius
                        //we should wait for another bresenham iteration?



                        //TODO: also check for distance > threshold!!!!!!
                        sewingMode = false;
#ifdef SHOW_TEXT_STITCHING
                        cout << "Exiting sewing mode since pixel are deviating too much" << endl;
#endif
                        //TODO: should there be a way to continue sewing for a little while?


                        Vector2i p1 = project2i(lastSewingP->p);
                        Vector2i p2 = project2i(currentSewingP->p);
#ifdef SHOW_WINDOW_STITCHING
                        debugColorCodedNewSegmentation.at<cv::Vec4b>(p1[1],p1[0]) = cv::Vec4b(0,0,255,0);
                        debugColorCodedNewSegmentation.at<cv::Vec4b>(p2[1],p2[0]) = cv::Vec4b(255,0,255,0);
                        cv::imshow("stitch",debugColorCodedNewSegmentation);
                        cv::waitKey(1);
#endif
                        //but this means we should run the logic coming afterwards.
                    }else{
                        //TODO:create one or two triangles depending on if the current edge already has one
                        if(edgeMade){
                            //the problem is that edges on both sides do not have the same length! Two cases:
                            //1) too slow: each step we will try to add another triangle to the stitch by progressing on the oppositing edge
                            //2) too fast: wait till the bresenham progresses one pixel
                            //but when to break up?

                            bool more=true;
                            while(more){

                                if(currentSewingEdge.points(1).get()->encompassed()){

                                    more = false;
                                    sewingMode=false;
                                    continue;
                                }
                                Edge otherEdge;
                                currentSewingEdge.getOtherEdge(1,otherEdge,borders);

                                VertexReference vr = otherEdge.points(1);
                                Vertex* v = vr.get();
                                Vector2i pix = project2i(v->p);
                                //check if the pixel of the next edge point really belongs to new geometry:
                                MeshPatch* patch = newSegPM.at<MeshPatch*>(pix[1],pix[0]);
                                if(patch != vr.getPatch()){
#ifdef SHOW_TEXT_STITCHING
                                    cout << "quitting sewing because the next pixel would lie on wrong side" << endl;
#endif
                                    sewingMode=false;
                                    more = false;
                                    continue;
                                }


                                float newDepth = newGeomM.at<Vec4f>(pix[1],pix[0])[2]; //getting depth
                                float depthThreshold = 0.05f;//TODO: change this to something depth (standard deviation dependant)
                                if(std::abs(newDepth-depthInterpolated) > depthThreshold){
                                    sewingMode=false;
                                    more = false;
                                    continue;
                                }

                                if(!isTrianglePossible({currentSewingPr,lastSewingPr,pr1})){
                                    more = false;
                                    sewingMode = false;
                                    continue;

                                }
                                map->addTriangle(currentSewingPr,lastSewingPr,pr1,debugListNewEdges  );
                                nrTrianglesThisEdge++;
                                lastPr = currentSewingPr;
#ifdef SHOW_TEXT_STITCHING
                                cout << "f2" << endl;
#endif

                                edgeMade = true;

                                currentSewingEdge = otherEdge;
                                lastSewingPr = currentSewingPr;
                                lastSewingP = currentSewingP;
                                currentSewingPr = currentSewingEdge.points(1);
                                currentSewingP = currentSewingPr.get();
                                currentSewingPix = project2i(currentSewingP->p);
                                if(abs(pix[0]-x) > 1 || abs(pix[1]-y) > 1 ){
                                    //boo yeah! create another triangle!!!
#ifdef SHOW_TEXT_STITCHING
                                    cout << "the next pixel (" << pix[0] << ", " << pix[1] << ") is too far away. wait for the next bresenham step" << endl;
#endif
                                    more = false;
                                }else{


                                }
                            }
                        }else{
                            //multiple new triangles:
                            //pr0,pr1,lastSewingPr

                            if(!isTrianglePossible({lastSewingPr,pr0,pr1})){

                                sewingMode=false;
                                return;
                            }
                            map->addTriangle(lastSewingPr,pr0,pr1,debugListNewEdges);
                            nrTrianglesThisEdge++;
                            lastPr = lastSewingPr;
#ifdef SHOW_TEXT_STITCHING
                            cout << "g1" << endl;
#endif
                            Vector2i p1 = project2i(lastSewingP->p);
                            Vector2i p2 = project2i(currentSewingP->p);
                            //debugColorCodedNewSegmentation.at<cv::Vec4b>(p1[1],p1[0]) = cv::Vec4b(0,0,255,0);
                            //debugColorCodedNewSegmentation.at<cv::Vec4b>(p2[1],p2[0]) = cv::Vec4b(255,0,255,0);
                            //cv::imshow("stitch",debugColorCodedNewSegmentation);
                            //cv::waitKey(1);

                            //pr1,lastSewingPr,currentSewingPr


                            bool more=true;
                            while(more){


                                if(!isTrianglePossible({currentSewingPr,lastSewingPr,pr1})){
                                    sewingMode=false;
                                    return;

                                }
                                map->addTriangle(currentSewingPr,lastSewingPr,pr1,debugListNewEdges);
                                lastPr = currentSewingPr;
#ifdef SHOW_TEXT_STITCHING
                                cout << "g2" << endl;
#endif
                                edgeMade=true;
                                if(currentSewingP->encompassed()){
                                    //if(!currentSewingEdge.isOpen()){
                                    more=false;
                                    sewingMode=false;
                                    continue;
                                }
                                Edge otherEdge;
                                Vertex *debugP = currentSewingEdge.points(1).get();
                                currentSewingEdge.getOtherEdge(1,otherEdge,borders);
                                VertexReference vr = otherEdge.points(1);
                                Vertex* v = vr.get();
                                Vector2i pix = project2i(v->p);
                                if(isConnected(pr1,vr)){
                                    if(!isOpenEdge(pr1,vr)){
                                        more=false;
                                        sewingMode=false;
                                        continue;
                                    }
                                }
                                MeshPatch* patch = newSegPM.at<MeshPatch*>(pix[1],pix[0]);
                                if(patch != vr.getPatch()){
#ifdef SHOW_TEXT_STITCHING
                                    cout << "quitting sewing because the next pixel would lie on wrong side" << endl;
#endif
                                    sewingMode=false;
                                    more = false;
                                    continue;
                                }

                                //TODO: this could potentially happen at the end of a loop
                                if(isConnected(currentSewingPr,pr1)){
                                    if(!isOpenEdge(currentSewingPr,pr1)){
#ifdef SHOW_TEXT_STITCHING
                                        cout << "quitting sewing because of pr1 and currentSewingPr are closed" << endl;
#endif
                                        more = false;
                                        sewingMode=false;
                                        continue;
                                    }
                                }

                                float newDepth = newGeomM.at<Vec4f>(pix[1],pix[0])[2]; //getting depth
                                float depthThreshold = 0.05f;//TODO: change this to something depth (standard deviation dependant)
                                if(std::abs(newDepth-depthInterpolated) > depthThreshold){
#ifdef SHOW_TEXT_STITCHING
                                    cout << "quitting sewing because of bigger depth step" << endl;
#endif
                                    sewingMode=false;
                                    more = false;
                                    continue;
                                }



                                currentSewingEdge = otherEdge;
                                lastSewingPr = currentSewingPr;
                                lastSewingP = currentSewingP;
                                currentSewingPr = currentSewingEdge.points(1);
                                currentSewingP = currentSewingPr.get();
                                currentSewingPix = project2i(currentSewingP->p);
                                if(abs(pix[0]-x) > 1 || abs(pix[1]-y) > 1 ){
                                    //boo yeah! create another triangle!!!
#ifdef SHOW_TEXT_STITCHING
                                    cout << "the next pixel (" << pix[0] << ", " << pix[1] << ") is too far away. wait for the next bresenham step" << endl;
#endif
                                    more = false;
                                }else{
                                }
                            }
                            if(currentSewingP->encompassed()){
                                sewingMode = false;
                                more=false;
                                return;
                            }
                            //get next edge
                            edge.alreadyUsedForStitch = true;
                            Edge otherEdge;
                            currentSewingEdge.getOtherEdge(1,otherEdge,borders);
                            currentSewingEdge = otherEdge;
                            lastSewingPr = currentSewingPr;
                            lastSewingP = currentSewingP;
                            currentSewingPr = currentSewingEdge.points(1);
                            currentSewingP = currentSewingPr.get();
                            currentSewingPix = project2i(currentSewingP->p);
                            edgeMade=true;
                        }
                    }
                    //TODO: check if z-distance is within thresholds
                }
                //when in sewing mode we don't need to check for pixelwise neighbourhood:
                if(sewingMode){
                    return;
                }
                //cout << x << " " << y << endl;
                //TODO: think why this block is necessary (and document it)
                if(!veryFirstPoint && firstPoint){
                    firstPoint = false;
                    //return;//make this whole block useless
                }
                veryFirstPoint = false;

                if(x<0 || y<0 || x>=width || y>=height){
                    return;
                }
#ifdef SHOW_WINDOW_STITCHING
                debugColorCodedNewSegmentation.at<cv::Vec4b>(y,x) = cv::Vec4b(255,255,255,0);
                cv::imshow("stitch",debugColorCodedNewSegmentation);
                cv::waitKey(1);
#endif

                Vector2i neighbours[]={
                        Vector2i(x-1,y), Vector2i(x+1,y), Vector2i(x,y-1), Vector2i(x,y+1), //4 connectivity
                        Vector2i(x-1,y-1), Vector2i(x+1,y-1), Vector2i(x-1,y+1), Vector2i(x+1,y+1)//8 connectivity.
                };

                Vector4f geomProj = geomProjM.at<Eigen::Vector4f>(y,x);


                float depthSensor = newGeomM.at<Vec4f>(y,x)[2];
                float depthProjected = geomProj[2];
                Vector4f fragPos=interpolatePerspectific(point0,point1,w0,w1,t);
                float depthInterpolated = fragPos[2];

                //sort the neighoburing points depending on if there has been a triangle in this patch
                if(!edgeMade){
                    Vector2f end = pix1;
                    auto compare = [end](Vector2i l,Vector2i r){
                        float dist1 = (Vector2f(l[0],l[1])-end).norm();
                        float dist2 = (Vector2f(r[0],r[1])-end).norm();
                        return dist1 > dist2;
                    };
                    int n = sizeof(neighbours)/sizeof(neighbours[0]);
                    sort(neighbours,neighbours+n,compare);

                }else{

                    Vector2f end = pix0;
                    auto compare = [end](Vector2i l,Vector2i r){
                        float dist1 = (Vector2f(l[0],l[1])-end).norm();
                        float dist2 = (Vector2f(r[0],r[1])-end).norm();
                        return dist1 < dist2;
                    };
                    int n = sizeof(neighbours)/sizeof(neighbours[0]);
                    sort(neighbours,neighbours+n,compare);
                }



                for(Vector2i & neighbour : neighbours) {
                    //iterate over all neighbours
                    int xn = neighbour[0];
                    int yn = neighbour[1];
                    if(xn<0 || yn<0 || xn>=width || yn>=height){
                        continue;
                    }

                    //check if the pixel exists within the newly added points
                    MeshPatch* patch = newSegPM.at<MeshPatch*>(yn,xn);
                    int index = newPtIndM.at<int>(yn,xn);
                    if(patch == nullptr){
                        continue;
                    }

                    //check if depth threshold is not exceeded
                    float newDepth = newGeomM.at<Vec4f>(yn,xn)[2]; //getting depth
                    float depthThreshold = 0.05f;//TODO: change this to something depth (standard deviation dependant)
                    if(std::abs(newDepth-depthInterpolated) > depthThreshold){
                        //don't create new geometry
                        debug.at<Vec4b>(yn,xn) = Vec4b(255,0,0,255);
                        continue;
                    }

                    //check if we are at the right side of the old triangle
                    Vector2f newPix(xn,yn);
                    if(isOnSameSide(pix0,pix1,pix2,
                                    newPix)){
                        continue; //no triangle here

                    }
                    //if there already is a stitch we also want to check if we are on the right side of the stitching triangle
                    if(edgeMade){
                        if(isOnSameSide(pix1,lastPix,pix0,newPix)){
                            continue;
                        }
                    }

                    //check if the new pixel is equal to the very first connected one!
                    if(veryFirstEdgeMade){
                        if(newPix[0] == firstPix[0] && newPix[1] == firstPix[1]){
                            //actually this might mean that we want to close up the stitch
                            continue;//TODO: we might even want to end the process of stitching here entirely
                        }
                    }


                    VertexReference thisPr;
                    thisPr.set(patch,index);

                    //Check if this point is already completely encapsulated by triangles (stitching would then be illegal)
                    if(thisPr.get()->encompassed()){

                        continue;
                    }


                    debugColorCodedNewSegmentation.at<cv::Vec4b>(yn,xn) = cv::Vec4b(0,0,255,0);

                    //create new triangle
                    //TriangleReference tr;
                    //TODO: also check the orientation of the newly generated triangles
                    debugTriangleCount++;

                    if(!edgeMade){
                        if(!triangleCreatedLastEdge){//a

                            if(!isTrianglePossible({thisPr,pr0,pr1})){
                                continue;
                            }
                            checkAndStartSewing(thisPr);

                            map->addTriangle(thisPr,pr0,pr1,debugListNewEdges);
#ifdef SHOW_TEXT_STITCHING
                            cout << "a" << endl;
#endif

                            if(!veryFirstEdgeMade){
                                veryFirstEdgeMade = true;
                                firstPr = thisPr;
                                firstPix = newPix;
                            }
                            //return because we want to go in sewing mode
                            if(sewingMode){
                                edgeMade=true;

                                lastPr = thisPr;
                                lastPix = newPix;
                                return;
                            }

                        }else{
                            //No triangle connected to this edge yet.
                            //create a triangle that incorporates the last made edge
                            if(thisPr.isEqualTo(lastPr)){//b
                                //continue;

                                if(!isTrianglePossible({thisPr,pr0,pr1})){
                                    continue;
                                }
                                checkAndStartSewing(thisPr);

                                map->addTriangle(thisPr,pr0,pr1,debugListNewEdges);
#ifdef SHOW_TEXT_STITCHING
                                cout << "b" << endl;
#endif

                            }else{//c

                                if(!isTrianglePossible({lastPr,pr0,pr1})){
                                    continue;
                                }

                                Edge otherEdge;
                                if(getNextOpenEdge(thisPr,otherEdge)){
                                    VertexReference vr =otherEdge.points(1);
                                    Vector2i pix = project2i(otherEdge.points(1).get()->p);
                                    //TODO: check if the next point really belongs to the newly added geometry
                                    MeshPatch* patch = newSegPM.at<MeshPatch*>(pix[1],pix[0]);
                                    if(patch == vr.getPatch()){
                                        //if the other side of the stitch is not just a single vertex
#ifdef SHOW_TEXT_STITCHING
                                        cout << "starting sewing process" << endl;
#endif
                                        //Starting sewing process!!!!!
                                        sewingMode = true;
                                        currentSewingEdge = otherEdge;
                                        lastSewingPr = currentSewingEdge.points(0);
                                        lastSewingP = lastSewingPr.get();
                                        currentSewingPr = currentSewingEdge.points(1);
                                        currentSewingP = currentSewingPr.get();
                                        currentSewingPix = project2i(currentSewingP->p);
                                    }

                                }

                                map->addTriangle(lastPr,pr0,pr1,debugListNewEdges);//TODO: reinsert this
#ifdef SHOW_TEXT_STITCHING
                                cout << "c" << endl;
#endif
                                //TODO: reinsert this!!!!!

                                nrTrianglesThisEdge++; // just so veryFirstEdgeMade will be set
                                //lastPr = thisPr; //should actually stay the same!!!!!
                                //lastPix = newPix;
                                edgeMade=true;

                                continue;

                                bool needToConnectHole = false;


                                //TODO: whenever we have a lastPr and a thisPr to create a triangle we have to check if
                                //the orientation is correct

                                if(!isTrianglePossible({thisPr,lastPr,pr1})){
                                    continue;
                                }
                                map->addTriangle(thisPr,lastPr,pr1,debugListNewEdges);
#ifdef SHOW_TEXT_STITCHING
                                cout << "c" << endl;
#endif
                                if(needToConnectHole){
                                    //TODO: check if we can do another triangle:
                                    cout << "fill hole!!!!" << endl;
                                }



                            }

                        }
                        lastPr = thisPr;
                        lastPix = newPix;
                        edgeMade = true;

                    }else{//d
                        //we continue stitching if this edge already has one stitch.
                        //TODO: remove this:
                        //continue;//debug
                        bool needToConnectHole = false;

                        if(thisPr.isEqualTo(lastPr)){
                            continue;
                        }

                        //TODO: whenever we have a lastPr and a thisPr to create a triangle we have to check if
                        //the orientation is correct
                        //don't be confused this is only running if you delete the continue further up
                        checkAndStartSewing(thisPr);


                        if(!isTrianglePossible({thisPr,lastPr,pr1})){
                            continue;
                        }
                        if(!isConnected(lastPr,thisPr)){
                            needToConnectHole= true;
                            //TODO: check if the hole is one or two hops long.
                        }
                        map->addTriangle(thisPr,lastPr,pr1,debugListNewEdges);//third point is from last edge
#ifdef SHOW_TEXT_STITCHING
                        cout << "d" << endl;
#endif

                        if(needToConnectHole){
#ifdef SHOW_TEXT_STITCHING
                            cout << "holefix" << endl;
#endif
                        }
                        lastPr = thisPr;
                        lastPix = newPix;
                        if(!edgeMade){
                            assert(0);
                        }
                    }
                    nrTrianglesThisEdge++;

                }

                debugColorCodedNewSegmentation.at<cv::Vec4b>(debugLastPix[1],debugLastPix[0]) =
                        cv::Vec4b(0,255,0,0);
                debugLastPix = Vector2i(x,y);


            };


            auto func1 = [&] (int x,int y, float t) {
                //same as func1 but with proper setup
                func(x,y,t);

                lastPixi[0] = x;
                lastPixi[1] = y;
            };
            //the bresenham algorithm always needs to start with pixel 0 and go to pixel 1
            bresenham(pix0,pix1,func1);


            triangleCreatedLastEdge = nrTrianglesThisEdge>0;

            edge.alreadyUsedForStitch =true;
        }
    }
}




