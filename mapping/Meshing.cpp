//
// Created by simon on 8/2/19.
//

#include "Meshing.h"
#include "meshReconstruction.h"
#include "cuda/xtionCameraModel.h"

using namespace cv;
using namespace Eigen;
using namespace std;


int debugTriangleCount = 0;
inline TriangleReference Meshing::addTriangle(const VertexReference &pr1,
                                              const VertexReference &pr2,
                                              const VertexReference &pr3,
                                 const Triangle::Neighbour &nr1,
                                 const Triangle::Neighbour &nr2,
                                 const Triangle::Neighbour &nr3,
                                 int &rotated) {

    //return TriangleReference();
    rotated = 0;
    MeshPatch *patch1=pr1.getPatch();
    MeshPatch *patch2=pr2.getPatch();
    MeshPatch *patch3=pr3.getPatch();
    if(patch1==0 || patch2 ==0  || patch3 == 0){
        assert(0); //maybe we don't want to have a assert here
        return TriangleReference();//can't add this triangle to the
    }
    //here we should not ask if all the indices are the same...
    //first case is every vertex is from the same patch
    if(patch1 == patch2 && patch1 == patch3){
        //if(segments.at<int>(p1[0],p1[1]) > 0 ){
        MeshPatch* patch = patch1;
        patch->workInProgress.lock();
        //patch->isGpuUpToDate=false;
        //patch->isCpuAhead = true;
        patch->cpuTrianglesAhead = true;

        //create a triangle:
        Triangle triangle;
        triangle.points[0] = pr1;
        triangle.points[1] = pr2;
        triangle.points[2] = pr3;
        triangle.neighbours[0] = nr1;
        triangle.neighbours[1] = nr2;
        triangle.neighbours[2] = nr3;
        triangle.debugNr = debugTriangleCount++;


        patch->cpuTrianglesAhead=true;

        TriangleReference triangleReference;
        triangleReference.container = patch;
        triangleReference.index = patch->triangles.size();
        patch->triangles.push_back(triangle);
        Triangle::registerTriangle(triangleReference,false);//TODO: check if registering the triangle worked
        //meshReconstruction->debugCheckTriangleNeighbourConsistency(meshReconstruction->GetAllPatches());
        patch->workInProgress.unlock();
        return triangleReference;
    }


    ///TODO: this triangle double and triple stitches need to be rplaced by something simpler.
    //triple stitch:
    if(patch1 != patch2 && patch2!=patch3 && patch1!=patch3){

        //first test if there is a triple stitch already
        //shared_ptr<TripleStitch> stitch =  patch1->getTripleStitchWith(patch2->id,patch3->id);
        shared_ptr<TripleStitch> stitch =  patch1->getTripleStitchWith(patch2,patch3);
        //temporary point references
        VertexReference p1 = pr1;
        VertexReference p2 = pr2;
        VertexReference p3 = pr3;
        Triangle::Neighbour n1 = nr1;
        Triangle::Neighbour n2 = nr2;
        Triangle::Neighbour n3 = nr3;

        //if not create one:
        if(!stitch){
            stitch = make_shared<TripleStitch>();
            stitch->weakSelf = stitch;
            stitch->patches[0]=patch1->weakSelf;
            stitch->patches[1]=patch2->weakSelf;
            stitch->patches[2]=patch3->weakSelf;

            patch1->addStitchReference(stitch);
            patch2->addStitchReference(stitch);
            patch3->addStitchReference(stitch);
        }else{

            //When a stitch was preexisting the order of points in the triangles has to be changed. to respect that
            MeshPatch* mainPatch = stitch->patches[0].lock().get();
            if(mainPatch == patch2){
                //we have to rotate the print references so that the first reference is pointing to segment 2
                VertexReference psw=p1;
                p1=p2;
                p2=p3;
                p3=psw;

                Triangle::Neighbour nw = n1;
                n1 = n2;
                n2 = n3;
                n3 = nw;
                rotated = 1;
            }
            if(mainPatch == patch3){
                VertexReference psw=p1;
                p1=p3;
                p3=p2;
                p2=psw;

                Triangle::Neighbour nw = n1;
                n1 = n3;
                n3 = n2;
                n2 = nw;
                rotated = 2;
            }
        }


        Triangle triangle;

        triangle.points[0] = p1;
        triangle.points[1] = p2;
        triangle.points[2] = p3;
        triangle.neighbours[0] = n1;
        triangle.neighbours[1] = n2;
        triangle.neighbours[2] = n3;
        triangle.debugNr = debugTriangleCount++;



        TriangleReference triangleReference;
        triangleReference.container = stitch.get();
        triangleReference.index = stitch->triangles.size();
        stitch->triangles.push_back(triangle);
        Triangle::registerTriangle(triangleReference, false);
        //we later on want to reupload the triangles to the gpu
        stitch->cpuTrianglesAhead=true;

        return triangleReference;
    }

    //double triangle stitch.
    //i know this is quite involved for one triangle:
    if(patch1 != patch2 || patch2 != patch3 || patch1 != patch3){
        MeshPatch* mainPatch = patch1;
        MeshPatch* secondaryPatch = nullptr;
        //find out the second segment
        if(patch1 == patch2 && patch2==patch3 && patch1==patch3){
            assert(0);//TODO: Make this optional so it only checks this in debug mode
        }
        if(patch1==patch2){
            secondaryPatch = patch3;
        }else{
            if(patch1==patch3){
                secondaryPatch = patch2;
            }else{
                if(patch2==patch3){
                    secondaryPatch = patch2;
                }
            }
        }
        assert(secondaryPatch!=mainPatch);
        assert(secondaryPatch!=nullptr);

        shared_ptr<DoubleStitch> stitch = mainPatch->getDoubleStitchWith(secondaryPatch);

        //temporary references to points
        VertexReference p1 = pr1;
        VertexReference p2 = pr2;
        VertexReference p3 = pr3;
        Triangle::Neighbour n1 = nr1;
        Triangle::Neighbour n2 = nr2;
        Triangle::Neighbour n3 = nr3;

        if(!stitch){
            stitch = make_shared<DoubleStitch>();
            stitch->weakSelf=stitch;
            stitch->patches[0]=mainPatch->weakSelf;
            stitch->patches[1]=secondaryPatch->weakSelf;

            patch1->doubleStitches.push_back(stitch);
            patch2->doubleStitches.push_back(stitch);
        }else{
            //When a stitch was preexisting the order of points in the triangles has to be changed. to respect that
            MeshPatch* primaryPatch = stitch->patches[0].lock().get();
            if(primaryPatch != patch1){
                if(primaryPatch == patch2){
                    //we have to rotate the print references so that the first reference is pointing to segment 2
                    VertexReference psw=p1;
                    p1=p2;
                    p2=p3;
                    p3=psw;

                    Triangle::Neighbour nw = n1;
                    n1 = n2;
                    n2 = n3;
                    n3 = nw;
                    rotated = 1;
                }else if(primaryPatch == patch3){
                    VertexReference psw=p1;
                    p1=p3;
                    p3=p2;
                    p2=psw;

                    Triangle::Neighbour nw = n1;
                    n1 = n3;
                    n3 = n2;
                    n2 = nw;
                    rotated = 2;
                }
            }
        }


        Triangle triangle;


        triangle.points[0] = p1;
        triangle.points[1] = p2;
        triangle.points[2] = p3;
        triangle.neighbours[0] = n1;
        triangle.neighbours[1] = n2;
        triangle.neighbours[2] = n3;
        triangle.debugNr = debugTriangleCount++;

        TriangleReference triangleReference;
        triangleReference.container = stitch.get();
        triangleReference.index = stitch->triangles.size();
        stitch->triangles.push_back(triangle);
        Triangle::registerTriangle(triangleReference,false);

        stitch->cpuTrianglesAhead=true;
        return triangleReference;
        //return triangle;
    }
}
void Meshing::MeshIt(cv::Mat points, cv::Mat meshPointers,
            cv::Mat vertexIndices,
            cv::Mat sensorStd,
            float maxDepthStep,//deprecate this
            Eigen::Matrix4f depthPose){
    int width=points.cols;
    int height=points.rows;

    Triangle::Neighbour nbUp[width];
    Triangle::Neighbour nbLeft;
    const Triangle::Neighbour empty;

    int r;
    //in a first step iterate over all points to store the indices
    for(int i=0;i<height;i++){
        for(int j=0;j<width;j++){
            MeshPatch* mesh = (MeshPatch*)meshPointers.at<uint64_t>(i,j);
            if(mesh){
                Vertex vertex;
                vertex.p = depthPose*points.at<Vector4f>(i,j);
                if(isnan(vertex.p[0])){
                    assert(0);
                    cout << "[ScaleableMap::meshIt] "
                            "Why is this nan?" << endl;
                }
                //cout <<"vertex" << endl <<  vertex.point << endl;
                //vertex.point=Vector4f(0,0,0,0);
                int index = mesh->vertices.size();
                vertexIndices.at<int>(i,j)=index;

                mesh->workInProgress.lock();
                mesh->vertices.push_back(vertex);
                mesh->workInProgress.unlock();
                //mesh->isGpuUpToDate=false;
                //mesh->isCpuAhead = true;
                mesh->cpuVerticesAhead = true;
                //mesh->cpuTrianglesAhead = true;
            }else{
                if(!isnan(points.at<Vector4f>(i,j)[0])){
                    //sometimes a point appears valid even though it shouldn't be ( correcting this)
                    points.at<Vector4f>(i,j) = Vector4f(NAN,NAN,NAN,NAN);
                }
            }
        }
    }
    //imshow("debug1",debug1);
    //waitKey(1);

    //now create the indices to mesh all this
    for(size_t i=0;i<height-1;i++){
        nbLeft.invalidate();
        for(size_t j=0;j<width-1;j++){
            Triangle::Neighbour tn;
            //two cases: upper left and lower right is closer to each other than upper right and lower left are
            // or the other way around.
            //in each case the possible triangles are different

            //this will throw a big compiler error tomorrow!!!!
            //        its going to be greeeeeeat!!!!!




            float zs[4]; //storage for depth values
            zs[0]=points.at<Vector4f>(i,j)[2];//Upper left
            zs[2]=points.at<Vector4f>(i+1,j+1)[2];//bottom right
            float distanceUlBr=fabs(zs[0]-zs[2]);
            zs[1]=points.at<Vector4f>(i,j+1)[2];//Upper right
            zs[3]=points.at<Vector4f>(i+1,j)[2];//bottom left
            float distanceUrBl=fabs(zs[1]-zs[3]);
            VertexReference pr[4];
            pr[0].set((MeshPatch*)meshPointers.at<uint64_t>(i,j),
                    vertexIndices.at<int>(i,j));
            pr[2].set((MeshPatch*)meshPointers.at<uint64_t>(i+1,j+1),
                      vertexIndices.at<int>(i+1,j+1));
            pr[1].set((MeshPatch*)meshPointers.at<uint64_t>(i,j+1),
                      vertexIndices.at<int>(i,j+1));
            pr[3].set((MeshPatch*)meshPointers.at<uint64_t>(i+1,j),
                      vertexIndices.at<int>(i+1,j));

            float threshold = maxDepthStep;


            int nanCount=0;
            int nanAt=-1;
            for(size_t k=0;k<4;k++){
                if(isnan(zs[k])){
                    nanCount++;
                    nanAt=k;
                }
            }
            if(nanCount==1){

                //The threshold has to be taken from a valid pixel
                // which is part of the triangle
                threshold = sensorStd.at<Vec4f>(i,j)[2];
                threshold = xtionStdToThresholdSeg(threshold);
                //with only one invalid triangle we can create one triangle
                bool created = false;
                switch(nanAt){
                    case 0:
                        //create a bottom right triangle
                        //but first check for depth values beeing within certain parameters

                        //The threshold has to be taken from a valid pixel
                        // which is part of the triangle
                        threshold = sensorStd.at<Vec4f>(i+1,j)[2];
                        threshold = xtionStdToThresholdSeg(threshold);
                        if(fabs(zs[1]-zs[2])<threshold &&
                           fabs(zs[1]-zs[3])<threshold){
                            tn.ref =
                                    addTriangle(pr[3],pr[2],pr[1],
                                            empty,empty,empty,r); // no neighbour in this case

                            tn.pos = (1 - r + 3) % 3;
                            nbLeft = tn;
                            tn.pos = (0 - r + 3) % 3;
                            nbUp[j] = tn;
                            created=true;
                            //cout << "a" << j  << endl;

                        }
                        break;
                    case 1:
                        //bottom left triangle
                        if(fabs(zs[0]-zs[3])<threshold &&
                           fabs(zs[3]-zs[2])<threshold){
                            tn.ref =
                                    addTriangle(pr[0],pr[3],pr[2],
                                                nbLeft,empty,empty,r);

                            tn.pos = (1 - r + 3) % 3;
                            nbUp[j] = tn;
                            nbLeft.invalidate();
                            created=true;
                            //cout << "b" << j  << endl;
                        }
                        break;
                    case 2:
                        //top left triangle
                        if(fabs(zs[0]-zs[1])<threshold &&
                           fabs(zs[0]-zs[2])<threshold){
                            tn.ref =
                                    addTriangle(pr[0],pr[3],pr[1],
                                                nbLeft,nbUp[j],empty,r);

                            nbUp[j].invalidate();
                            nbLeft.invalidate();
                            created=true;
                            //cout << "c" << j  << endl;
                        }
                        break;
                    case 3:
                        //top right triangle
                        if(fabs(zs[0]-zs[1])<threshold &&
                           fabs(zs[1]-zs[2])<threshold){
                            tn.ref =
                                    addTriangle(pr[0],pr[2],pr[1],
                                            empty,empty,nbUp[j],r);

                            tn.pos = (1 - r + 3) % 3;
                            nbLeft = tn;
                            nbUp[j].invalidate();
                            created=true;
                            //cout << "d" << j  << endl;
                        }
                        break;
                }
                if(!created){
                    //cout << "no fitting half" << j <<endl;
                    nbUp[j].invalidate();
                    nbLeft.invalidate();
                }

                continue;
            }
            if(nanCount>1){
                //cout << "overNAN" << j << endl;
                nbUp[j].invalidate();
                nbLeft.invalidate();
                continue;//with 2 or more invalid points we can't create a triangle
            }


            bool debugCreated = false;
            //if there is no invalid point we create the triangles depending on which
            if(distanceUlBr > distanceUrBl){// && !isnan(distanceUlBr)){//even excluding the NAN does not make this code faster

                //The threshold has to be taken from a valid pixel
                // which is part of the triangle
                threshold = sensorStd.at<Vec4f>(i+1,j)[2];
                threshold = xtionStdToThresholdSeg(threshold);
                if(distanceUrBl<threshold){

                    Triangle::Neighbour nbDiag;
                    if(fabs(zs[1]-zs[0])<threshold){
                        //top left triangle
                        nbDiag.ref =
                                addTriangle(pr[0],pr[3],pr[1],
                                        nbLeft,empty,nbUp[j],r);
                        debugCreated = true;

                        nbDiag.pos = (1 - r + 3) % 3;
                        //cout << "e" << j << endl;
                    }

                    if(fabs(zs[1]-zs[2])<threshold){
                        //bottom right triangle
                        tn.ref =
                                addTriangle(pr[3],pr[2],pr[1],
                                        empty,empty,nbDiag,r);
                        debugCreated = true;
                        tn.pos = (1 - r + 3) % 3;
                        nbLeft = tn;
                        tn.pos = (0 - r + 3) % 3;
                        nbUp[j] = tn;
                        //cout << "f" << j  << endl;

                    }else{
                        nbLeft.invalidate();
                        nbUp[j].invalidate();
                    }
                }else{
                    nbLeft.invalidate();
                    nbUp[j].invalidate();
                }

                if(!debugCreated){
                    //cout << "no Quad possible a" << j << endl;
                }
            }else{

                //The threshold has to be taken from a valid pixel
                // which is part of the triangle
                threshold = sensorStd.at<Vec4f>(i,j)[2];
                threshold = xtionStdToThresholdSeg(threshold);
                Triangle::Neighbour nbDiag;
                Triangle::Neighbour nbUpNew;
                Triangle::Neighbour nbLeftNew;
                if(distanceUlBr<threshold){
                    if(fabs(zs[0]-zs[3])<threshold){
                        // bottom left triangle
                        if(j==88 && i==11){
                            //cout << "Lets fix this" << endl;
                        }
                        tn.ref =
                                addTriangle(pr[0],pr[3],pr[2],
                                        nbLeft,empty,empty,r);
                        debugCreated = true;
                        tn.pos = (2 - r + 3) % 3;
                        nbDiag = tn;
                        tn.pos = (1 - r + 3) % 3;
                        nbUpNew = tn;
                        //cout << "g"  << j  << endl;

                    }

                    if(fabs(zs[0]-zs[1])<threshold){
                        //top right triangle
                        tn.ref =
                                addTriangle(pr[0],pr[2],pr[1],
                                        nbDiag,empty,nbUp[j],r);
                        debugCreated = true;
                        tn.pos = (1 - r + 3) % 3;
                        nbLeftNew = tn;
                        //cout << "h"  << j   << endl;

                    }
                }
                nbUp[j] = nbUpNew;
                nbLeft = nbLeftNew;
                if(!debugCreated){
                    //cout << "no Quad possible b" << j << endl;
                }
            }
        }
    }
}
struct VertexTexConn{
    Vertex* vert;
    //TODO: maybe also add a proper vertex reference
    //with patch and index. this would allow to have
    // a proper connection between vertices and triangles.
    //THINK: why where some of the vertices not updated???
    //only because they have solely been part of non-patch triangles
    //this unfortunately still might be the case sometimes (even tough
    //this occures less frequently now)
    //DECISION: create a buffer that connects vertices with their tex coordinates
    //ON GPU AND CPU!!!!!!!!!



    //VertexInformation* info = nullptr;
    //int texIndWithinMainPatch = -1;
    vector<uint32_t*> texInds;

    //TODO: set the texture index for the vertex if the verte
    //bool setTexIndex=false;
};
inline bool operator<(const VertexTexConn &lhs, const VertexTexConn &rhs){
    return lhs.vert < rhs.vert;
}
inline bool operator==(const VertexTexConn &lhs, const VertexTexConn &rhs){
    return lhs.vert == rhs.vert;
}

void Meshing::GenTexIndices(std::vector<std::shared_ptr<MeshPatch> > &patches) {


    for(size_t i=0;i<patches.size();i++){

        set<VertexTexConn> vertSet;
        auto appendTexCoords = [&vertSet] (Triangle &triangle){
            for(size_t i=0;i<3;i++){
                VertexTexConn vert;
                vert.vert = triangle.points[i].get();
                set<VertexTexConn>::iterator found = vertSet.find(vert);
                if(found != vertSet.end()){
                    //when one vertex already is within the set we just add the
                    //according pointer to the vertex index
                    VertexTexConn *texConn =
                            const_cast<VertexTexConn*>(&(*found));
                    //only use this cast and modify the elements when
                    //you do not modify anything that influences the
                    //operators defined on the set
                    texConn->texInds.push_back(&(triangle.texIndices[i]));
                }else{
                    //otherwise we create a new container and push back the
                    //right pointer to the texture index
                    vert.texInds.push_back(&(triangle.texIndices[i]));
                    vertSet.insert(vert);
                }
            }
        };
        shared_ptr<MeshPatch> &patch = patches[i];
        patch->geomTexPatch =
                meshReconstruction->genMeshTexture(MeshTexture::Type::standardDeviation);

        //to really accomodate for every vertex (even if it may not be visible)
        //we also add the vertices that do not have triangles
        for(size_t j=0;j<patch->vertices.size();j++){
            VertexTexConn vert;
            vert.vert = &patch->vertices[j];
            vertSet.insert(vert);
            //we do this for every vertex because not every vertex would be provided
            //with a texture by one of the triangles.
        }
        //now create the texture coordinates for the triangles
        //TODO:maybe also a mutex for the triangles? (and vertices)
        for(size_t j=0;j<patch->triangles.size();j++){
            appendTexCoords(patch->triangles[j]);
        }

        //check all the triangles that lie in double stitches
        patch->doubleStitchMutex.lock();
        for(size_t k=0;k<patch->doubleStitches.size();k++){
            shared_ptr<DoubleStitch> &stitch = patch->doubleStitches[k];
            if(stitch->patches[0].lock()!=patch){
                //the texture for this stitch is provided by another patch
                continue;
            }
            for(size_t j=0;j<stitch->triangles.size();j++){
                appendTexCoords(stitch->triangles[j]);
            }
        }
        patch->doubleStitchMutex.unlock();
        //and now check all the triangles that lie in triple stitches
        patch->tripleStitchMutex.lock();
        for(size_t k=0;k<patch->tripleStitches.size();k++){
            shared_ptr<TripleStitch> &stitch = patch->tripleStitches[k];
            if(stitch->patches[0].lock()!=patch){
                //obviously this patch is not what we are searching for
                continue;
            }
            for(size_t j=0;j<stitch->triangles.size();j++){
                appendTexCoords(stitch->triangles[j]);
            }
        }
        patch->tripleStitchMutex.unlock();
        patch->geomTexPatch->texCoords.resize(vertSet.size());
        size_t j=0;
        for(auto v : vertSet){
            for(size_t k=0;k<v.texInds.size();k++){
                *(v.texInds[k]) = j;
            }
            j++;
        }

        //patch->nrTexCoords = j;

        //now do the texture indices on the patch vertices
        for(size_t j=0;j<patch->triangles.size();j++){
            //TODO: get better tex coordinates
            for(size_t k=0;k<3;k++){
                if(patch->triangles[j].points[k].getPatch()==patch.get()){
                    patch->triangles[j].points[k].get()->texIndInMainPatch =
                            patch->triangles[j].texIndices[k];
                }
            }
        }
    }

    //assert(0); //TODO: orient on generateGeomTexForNovelPatches

}