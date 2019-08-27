#include "gpuNormSeg.h"
#include "../cuda/normEst.h"
#include <opencv2/opencv.hpp>

#include <vector>
#include <tuple>
#include <queue>
#include <chrono>

#include "../cuda/xtionCameraModel.h"

using namespace std;
using namespace Eigen;
using namespace cv;

GpuNormSeg::GpuNormSeg(GarbageCollector *garbageCollector,int width, int height)
{

    points = make_shared<gfx::GpuTex2D>(garbageCollector,GL_RGBA32F,GL_RGBA,GL_FLOAT,
                                        width,height,
                                        true,(void*)0);
    points->name = "gpu based segmentation points";

    normals = make_shared<gfx::GpuTex2D>(garbageCollector,GL_RGBA32F,GL_RGBA,GL_FLOAT,
                                        width,height,
                                        true,(void*)0);
    normals->name = "gpu based segmentation normals";

    gpuSegmentation = make_shared<gfx::GpuTex2D>(garbageCollector,GL_R32F,GL_RED,GL_FLOAT,
                                        width,height,
                                        true,(void*)0);
    gpuSegmentation->name = "gpu based segmentation gpuSegmentation";


}

GpuNormSeg::~GpuNormSeg()
{

}

void GpuNormSeg::calcNormals()
{
    int width = points->getWidth();
    int height = points->getHeight();

    cudaCalcNormals(dStdMaxStd->getCudaSurfaceObject(),
                    points->getCudaSurfaceObject(),
                    normals->getCudaSurfaceObject(),
                    width,height,0.05);//threshold of 1 cm for normal calculation...
    return;
    Mat debug(height,width,CV_32FC4);
    normals->downloadData((void*)debug.data);
#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
    imshow("estimatedNormals",debug*-1.0f);
    waitKey();
#endif
}

std::shared_ptr<gfx::GpuTex2D> GpuNormSeg::getGpuNormals()
{
    return normals;
}

void GpuNormSeg::calcPoints()
{
    int width = dStdMaxStd->getWidth();
    int height = dStdMaxStd->getHeight();

    cudaCalcPoints(dStdMaxStd->getCudaSurfaceObject(),
               points->getCudaSurfaceObject(),
               width,height,
               fxycxy);


}

std::shared_ptr<gfx::GpuTex2D> GpuNormSeg::getGpuPoints()
{
    return points;
}
cv::Mat generateColorCodedTexture(cv::Mat segmentation){
    cv::Mat colorMap(1,64*48,CV_8UC4);
    colorMap.at<cv::Vec4b>(0)=cv::Vec4b(51,51,0,0);
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

            coloredImage.at<cv::Vec4b>(i,j) =
                    colorMap.at<cv::Vec4b>(0,
                                           segmentation.at<uint32_t>(i,j)%(64*48));
            if(segmentation.at<int32_t>(i,j)==-1){
                coloredImage.at<cv::Vec4b>(i,j)=Vec4b(0,0,0,255);
            }
            asm("#end asm test");
        }
    }


    return coloredImage;
}

void GpuNormSeg::segment()
{
    float maxDist = this->maxDistance;
    //float maxDist=10;//10 meter max distance
    float geometryAssignThreshold=0.05;//5cm for instance
    int maxSegSize=400;//this should come from somewhere else but in the meantime this will do
    float maxDepthStep=0.05;//50cm max depth step
    float minCosAngle = 0.9;//determines how much the normals could be apart from each other on the edge
    int minNrPointsPerSegment=10;//5;//the minimum amount of points to really generate a point
    int maxSegmentExtent=30;
    int width = dStdMaxStd->getWidth();
    int height = dStdMaxStd->getHeight();

    Vector2i w[4] = {
        Vector2i(0,-1),
        Vector2i(0,1),
        Vector2i(-1,0),
        Vector2i(1,0)
    };

    Mat sensorStds(height,width,CV_32FC4);
    dStdMaxStd->downloadData((void*)sensorStds.data);
    Mat pts(height,width,CV_32FC4);//propably don't need these
    points->downloadData((void*)pts.data);
    Mat norms(height,width,CV_32FC4);
    normals->downloadData((void*)norms.data);

    Mat depth(height,width,CV_8UC1);
    for(size_t i = 0;i<width*height;i++){
        depth.at<uint8_t>(i) = sensorStds.at<cv::Vec4f>(i)[0]*(255/5);
    }

    Mat edges;
    cv::Canny(depth,edges,10,20);


    int dilation_size=3;
    Mat element = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2*dilation_size + 1,
                                               2*dilation_size+1 ),
                                         Point( dilation_size, dilation_size ) );


    Mat dilatedEdges;
    dilate( edges, dilatedEdges, element );

    Mat debug(height,width,CV_32FC4);
    points->downloadData((void*)debug.data);
#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
    imshow("projectedPoints",debug*0.1);
#endif
    normals->downloadData((void*)debug.data);
#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
    imshow("estimatedNormals",debug*-1.0f);
    waitKey();
#endif

    Mat mask(height,width,CV_8UC1);
    mask.setTo(0);
    for(size_t i=0;i < existingGeometry.rows; i++){
        for(size_t j=0;j< existingGeometry.cols;j++){
            //texPos.at<Vector2f>(i,j)=Vector2f(float(j)/float(width),float(i)/float(height));
            float z = sensorStds.at<Vector4f>(i,j)[0];//depth
            if(isnan(z) || z > maxDist){
                //mask.at<unsigned char>(i,j)=255;
                continue;
            }

            float exZ=existingGeometry.at<Vector4f>(i,j)[2];
            if(isnan(exZ)){
                //either there is no pixel for the existing geometry in this image
                mask.at<unsigned char>(i,j)=255;
            }else{
                //or the new pixel is in front of the existing geometry
                //(if it is behind it we might want to delete geometry

                float thresh = std::max(
                            existingStds.at<Vec4f>(i,j)[2], //standard deviation. TODO: WRONG DATA FORMAT!!!
                            sensorStds.at<Vec4f>(i,j)[2]);//something not working here
                thresh = xtionStdToThresholdSeg(thresh);

                if(z<exZ-thresh){
                    mask.at<unsigned char>(i,j)=255;
                }
            }
        }
    }

#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
    imshow("existing geometry",existingGeometry);
    imshow("segmentation mask",mask);
    imshow("stds",stds);
    waitKey();
#endif


    /*
    imshow("existing geometry",existingGeometry);
    imshow("segmentation mask",mask);
    imshow("stds",stds);
    imshow("depthItself",depth*0.1);
    imshow("edges",edges);
    imshow("dilatedEdges",dilatedEdges);
    //waitKey();
    */

    Mat seg(height,width,CV_32SC1);
    seg.setTo(Scalar(-1));

    //do the real segmentation

    //first element is count of the segment
    vector<int> segmentSize;
    vector<int> segmentSizeWithinBorder;
    vector<Vector2f> startingPointOfSegment;
    /*cout << "TODO: don't let the patch extend too far away from its seeding point"
         << endl;*/
    //assert(0);

    //the tuple in this queue is the position at wich to continue and the
    //maybe no tuple is needed at all
    queue<tuple<Vector2i, //pixel position
            int, //the segment index of the parent
            float,//the distance of the parent pixel
            Vector3f//the normal of the parent pixel
            >> q;
    //pixel position, segment id

    //w window ( the neighborhood for this segmentation method)

    for(size_t i=0;i<height;i++){
        for(size_t j=0;j<width;j++){
            Vector4f dStd = sensorStds.at<Vector4f>(i,j);
            if(isnan(dStd[0]) || !mask.at<uint8_t>(i,j)){
                //don't add if the pixel is invalid
                continue;
            }
            if(seg.at<int>(i,j)!=-1){
                //don't add the point if its already part of another element
                continue;
            }

            Vector4f n4 = norms.at<Vector4f>(i,j);
            Vector3f n = Vector3f(n4[0],n4[1],n4[2]);
            seg.at<int>(i,j)=segmentSize.size();
            segmentSize.push_back(1);
            if(dilatedEdges.at<uint8_t>(i,j) == 255){
                segmentSizeWithinBorder.push_back(1);
            }else{
                segmentSizeWithinBorder.push_back(0);
            }
            startingPointOfSegment.push_back(Vector2f(i,j));
            for(int k=0;k<4;k++){
                q.push(make_tuple(Vector2i(i,j)+w[k],
                                  segmentSize.size()-1,//the segment index
                                  dStd[0],//the depth
                                  n));//get the normal
            }
            while(!q.empty()){
                auto f = q.front();
                q.pop();//we get rid of this
                Vector2i p=get<0>(f);
                int segIndex = get<1>(f);
                float depthCenter = get<2>(f);
                Vector3f nc = get<3>(f);
                int segCount = segmentSize[segIndex];
                if(segCount>maxSegSize){
                    continue;//this segment already has enough elements...
                }
                if( (Vector2f(p[0],p[1]) -
                     startingPointOfSegment[segIndex]).norm() >
                        maxSegmentExtent ){
                    continue;
                }
                if(p[0]<0 || p[1]<0 || p[0]>=height || p[1]>=width){
                    continue; //out of bounds
                }
                //if this element already is part of a segment....
                //out of bou don't add it.
                if(seg.at<int>(p[0],p[1])!=-1){
                    continue;
                }
                Vector4f dStdp = sensorStds.at<Vector4f>(p[0],p[1]);
                if(isnan(dStdp[0]) || !mask.at<uint8_t>(p[0],p[1])){
                    continue;
                }

                //TODO: derive the maximum standard deviation from the current
                //depth map
                maxDepthStep = sensorStds.at<Vec4f>(p[0],p[1])[2];

                maxDepthStep =
                        xtionStdToThresholdSeg(
                            sensorStds.at<Vec4f>(p[0],p[1])[2]);
                //test if we have a discontinuity on the edge
                if(std::abs(dStdp[0]-depthCenter)>maxDepthStep){
                    continue;
                }
                //test if the normals are too far apart from each other
                Vector4f np4=norms.at<Vector4f>(p[0],p[1]);
                Vector3f np=Vector3f(np4[0],np4[1],np4[2]);

                float cosAlpha = nc.dot(np);
                //cout << cosAlpha << endl;
                if(cosAlpha<minCosAngle){
                    //continue; // the normals are not reliable enough yet
                }


                //make this element part of everything:
                seg.at<int>(p[0],p[1])=segIndex;
                segmentSize[segIndex]++;

                if(dilatedEdges.at<uint8_t>(p[0],p[1]) == 255){
                    segmentSizeWithinBorder[segIndex]++;
                }
                for(int k=0;k<4;k++){
                    //lets continue with all the neighbours
                    q.push(make_tuple(p+w[k],
                                      segmentSize.size()-1,
                                      dStdp[0],//depth of this point
                                      np));
                }
            }
        }
    }


    /*************************************************************/
    //PREPROCESSING to get rid of tiny holes in segmentation by connecting
    //too small objects to fitting bigger ones
    //Untested

    vector<int> newInds1(segmentSize.size()+1);
    for(size_t i=0;i<newInds1.size();i++){
        newInds1[i]=i-1;
    }
    for(size_t i=0;i<height;i++){
        for(size_t j=0;j<width;j++){
            int segInd=seg.at<int>(i,j);
            if(segInd==-1){
                continue;//invalid pixel anyway
            }
            float d = sensorStds.at<Vector4f>(i,j)[0];
            if(segmentSize[segInd]<minNrPointsPerSegment){
                //iterate over the neighbours if we can
                maxDepthStep =
                        xtionStdToThresholdSeg(sensorStds.at<Vec4f>(i,j)[2]);
                for(size_t k=0;k<4;k++){
                    Vector2i p=Vector2i(i,j) + w[k];
                    if(p[0]<0 || p[1]<0 || p[0]>=height || p[1]>=width){
                        continue; //out of bounds
                    }
                    int indOther = seg.at<int>(p[0],p[1]);
                    if(indOther==-1){
                        continue;
                    }
                    if(segmentSize[indOther]<minNrPointsPerSegment){
                        continue;
                    }

                    float ddebug=sensorStds.at<Vector4f>(p[0],p[1])[0];
                    if(std::abs(sensorStds.at<Vector4f>(p[0],p[1])[0]-d) <
                            maxDepthStep){
                        newInds1[segInd+1] = indOther;
                    }

                }
            }
        }
    }
    for(size_t i=0;i<width*height;i++){
        int segInd=seg.at<int>(i);
        seg.at<int>(i)=newInds1[segInd+1];
    }
    /**********************************************************/

    //imshow("pre filter segmentation",generateColorCodedTexture(seg));
    //destroy the elements with only one or less than.... lets say 3 pixel
    vector<int> newInds(segmentSize.size()+1);
    newInds[0]=-1;
    segCount=0;
    //cout << "TODO: add code to add tinier patches to their neighbours." << endl;
    for(size_t i=0;i<segmentSize.size();i++){
        //cout << "segment " << i << " had " << segments[i] << " pixel " << endl;

        //only keep a segment when less than 80% are within a edge region
        if( segmentSizeWithinBorder[i]*10 > segmentSize[i]*8 ){
            newInds[i+1]=-1;
            continue;
        }
        if( segmentSize[i]<minNrPointsPerSegment ){
            newInds[i+1]=-1;
            continue;
            //TODO: actually in this case we have to check
            //if there is at least one valid neighbour we could
            //connect it to.
        }
        newInds[i+1]=segCount;
        segCount++;
    }
    //replace the segments with too little pixel
    for(size_t i=0;i<width*height;i++){
        seg.at<int>(i)=newInds[seg.at<int>(i)+1];
    }
    segResult=seg;
#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
    imshow("segmentation",generateColorCodedTexture(seg));
    waitKey();
#endif

    //imshow("segmentation",generateColorCodedTexture(seg));
    //waitKey();


}
std::shared_ptr<gfx::GpuTex2D> GpuNormSeg::getGpuSegmentation()
{
    cout << "[GpuNormSeg::getGpuSegmentation] "
            "The segmentation is implemented on CPU not GPU" << endl;
    return gpuSegmentation;
}

Mat GpuNormSeg::getSegmentation()
{
    return segResult;
}

int GpuNormSeg::getSegCount()
{
    return segCount;
}

std::shared_ptr<gfx::GpuTex2D> GpuNormSeg::segment(
        std::shared_ptr<gfx::GpuTex2D> dStdMaxStd,
        cv::Mat existingDStdMaxStd,
        Mat existingGeometry)
{
    auto start = std::chrono::system_clock::now();
    this->existingGeometry = existingGeometry;
    this->dStdMaxStd = dStdMaxStd;
    this->existingStds = existingDStdMaxStd;
    calcPoints();
    calcNormals();
    segment();
    auto end = std::chrono::system_clock::now();
    auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        //present
    std::cout << "[GpuNormSeg::segment] time consumed by segmentation: " <<
                 elapsed.count() << "ms" << std::endl;

    return gpuSegmentation;

}
