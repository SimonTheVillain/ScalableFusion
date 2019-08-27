#include "DatasetLoader.h"

#include <eigen3/Eigen/Geometry>
#include <thread>


#include <radical/radiometric_response.h>
#include <radical/vignetting_response.h>

#include <fstream>

using namespace Eigen;
using namespace std;
using namespace cv;

void TumDataset::readNewSetOfImages()
{
    //cout << "reading new Images" << endl;
    std::string fileLocations;


    //return;//debug
    if(frameIndex != 0){
        frameIndex +=skipCount;
    }
    if(frameIndex < rgbFiles.size()){
        //std::string fileName2Open=folderPath + "/" + fileLocations;
        //cout << "reading file " << fileName2Open << endl;
        currentDepth = imread(depthFiles[frameIndex],cv::IMREAD_UNCHANGED);
        if(exposureTimes.size()>0){
            _rgbExposureTime = exposureTimes[frameIndex];
        }
        currentTimestamp = timestamps[frameIndex];

        //cv::Mat flipped;
        //cv::flip(currentDepth,flipped,1);
        //cv::imwrite(fileName2Open,flipped);
        currentDepth = currentDepth*scaleDepth;
        //currentDepth = currentDepth*0.1f;//DEBUG for the artificial dataset
        //DEBUG:
         //because we colledted data in 100um mode





        //fileName2Open=folderPath + "/" + fileLocations;
        currentRGB = cv::imread(rgbFiles[frameIndex]);
        /*if(currentRGB.type() == CV_8UC1){
            cv::Mat gray2RGB;
            currentRGB.convertTo(gray2RGB,CV_8UC3);
            currentRGB = gray2RGB;
            assert(0);
        }*/

        //cout << "DEBUG intrinsics:" << endl << rgbIntrinsics << endl << depthIntrinsics << endl;
        //cout << "DEBUG matrix " << endl << depth2RGB << endl;


        if(!depthUndistort1.empty()){
            //undistort the images
            cv::Mat currentDepthUndistorted;
            cv::remap(currentDepth, currentDepthUndistorted, depthUndistort1, depthUndistort2, CV_INTER_NN);
            //currentDepth = currentDepthUndistorted;//DEBUG: deactivate this
        }



        if(radiometricResponse!=nullptr &&
           vignettingResponse!=nullptr){
            cv::Mat irradiance, radiance;  // temporary storage
            //cv::Mat rgbVignettingCorrected;       // output image with vignette removed
            radiometricResponse->inverseMap(currentRGB, irradiance);
            vignettingResponse->remove(irradiance, radiance);
            radiance = radiance * 0.9;
            radiance = radiance * 1.2;
            for (int i = 0; i < radiance.size().area(); i++) {
                cv::Vec3f v = radiance.at<cv::Vec3f>(i);
                v[0] *= whiteFix[0];
                v[1] *= whiteFix[1];
                v[2] *= whiteFix[2];
                radiance.at<cv::Vec3f>(i) = v;
            }
            radiometricResponse->directMap(radiance, currentRGB);


        }

        if(!rgbUndistort1.empty()){



            cv::Mat currentRgbUndistorted;
            cv::remap(currentRGB,currentRgbUndistorted,rgbUndistort1,rgbUndistort2,CV_INTER_LINEAR);
            currentRGB = currentRgbUndistorted; 
            if(scaleDepth!=1){
                assert(0); //TODO: this scalefactor thingy really needs cleanup
            }
        }
        //as in the last datasets i collected
        //TODO: unify this over all datasets i use!

        //no scaling for the tumalike datasets

        //the highres datasets need scaling though:
        //currentDepth = currentDepth*0.5f;// * 5; //scale by 5 in the high resolution dataset //between bla and 16





        frameIndex++;

    }else{
        //frameList.close();
        running=false;
    }



    //when we are done before our time we wait....
    chrono::system_clock::time_point now =
            chrono::system_clock::now();

    if(replaySpeed!=0){
        chrono::system_clock::duration frameTime =
                chrono::microseconds((int)(33333.0f*(1.0f/replaySpeed)));//1.0f/(30.0f*replaySpeed));
        chrono::system_clock::duration duration =
                now-lastFrameReadout;

        chrono::system_clock::duration zero = chrono::microseconds(0);

        chrono::system_clock::duration remainingTime =
                std::max(frameTime-duration, zero);


        //chrono::system_clock::duration remainingTime_ = duration-frameTime;

        /*
        cout << "target time in ms" <<
                std::chrono::duration_cast<std::chrono::milliseconds>(frameTime).count() << endl;
        cout << "used time in ms" <<
                std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << endl;
        cout << "time to wait" <<
                std::chrono::duration_cast<std::chrono::milliseconds>(remainingTime).count() << endl;
        auto tick = std::chrono::system_clock::now();
        */
        this_thread::sleep_for(remainingTime);//remainingTime);
        /*auto delta = std::chrono::system_clock::now()- tick;
        cout << "time waited" <<
                std::chrono::duration_cast<std::chrono::milliseconds>(delta).count() << endl;
        */
    }
    lastFrameReadout = chrono::system_clock::now();

}



TumDataset::TumDataset(std::string folder, bool realtime, bool usePose,bool useHighRes,int skipNFrames,
        float depthScale,float trajectoryGTScale,bool invertGTTrajectory):
        scaleDepth(depthScale)
{

    hasPoses=false;
    //TODO: try to implement try catch pattern here, maybe it is useful
    readDepth=false;
    readRGB=false;
    folderPath=folder;

    frameIndex=skipNFrames;

    std::ifstream frameList;
    std::string fileName2Open;
    _hasHighRes = false;
    if(useHighRes){
        fileName2Open = folder + "/associations_ids.txt";
        frameList.open(fileName2Open);
        if(frameList.is_open()) {
            _hasHighRes = true;
        }
    }
    if(!_hasHighRes){
        fileName2Open = folder + "/associations.txt";
        frameList.open(fileName2Open);
        if(!frameList.is_open()){
            cout << "Couldn't open dataset (no associations.txt file found)" << endl;
            assert(0);//
        }
    }


    cv::FileStorage fs_whiteFix;
    if(_hasHighRes){
        fs_whiteFix.open(folder + "/white_fix_ids.yml",cv::FileStorage::READ);
    }else{
        fs_whiteFix.open(folder + "/white_fix.yml",cv::FileStorage::READ);
    }
    if(fs_whiteFix.isOpened()){
        fs_whiteFix["r_gain"] >> whiteFix[2];
        fs_whiteFix["g_gain"] >> whiteFix[1];
        fs_whiteFix["b_gain"] >> whiteFix[0];
    }else{
        whiteFix = cv::Vec3f(1,1,1);
    }


    std::string fileLocations;
    while(std::getline(frameList,fileLocations)){
        stringstream justForTheTimestamp(fileLocations);
        double timestamp;
        justForTheTimestamp >> timestamp;
        timestamps.push_back(timestamp);

        istringstream line(fileLocations);
        std::getline(line,fileLocations,' ');

        std::getline(line,fileLocations,' ');
        std::string file=folderPath + "/" + fileLocations;
        depthFiles.push_back(file);


        std::getline(line,fileLocations,' ');
        std::getline(line,fileLocations,' ');
        file=folderPath + "/" + fileLocations;
        rgbFiles.push_back(file);


    }
    if(rgbFiles.size()>0){
        running = true;
    }

    if(frameshiftRGB>0){
        rgbFiles.erase(rgbFiles.begin(),rgbFiles.begin()+frameshiftRGB);
        depthFiles.erase(depthFiles.end()-frameshiftRGB,depthFiles.end());
    }
    if(frameshiftRGB<0){
        depthFiles.erase(depthFiles.begin(),depthFiles.begin()-frameshiftRGB);
        rgbFiles.erase(rgbFiles.end()+frameshiftRGB,rgbFiles.end());
    }

    if(isRunning()){
        //readNewSetOfImages();
    }else{
        std::cout << "could not open " << fileName2Open << std::endl;
    }

    //Load groundtruth

    if(folder.find("georg") != string::npos){
        assert(0); //THIS NEVER WORKED
        float scale = 1.0f * 0.580382; // scale the translation

        string trajectoryFileName = folder + "/poses.csv";
        ifstream trajectoryFile;
        trajectoryFile.open(trajectoryFileName);
        if(trajectoryFile.is_open()){
            hasPoses=usePose;
            string line;
            while(getline(trajectoryFile,line)){
                if(line[0]!='#'){

                    std::replace( line.begin(), line.end(), ',', ' ');
                    TrajectoryPoint p;
                    stringstream stream(line);
                    stream >> p.timestamp;
                    float rx,ry,rz;
                    float x,y,z;
                    stream >> rx >> ry >> rz >> x >> y >> z;
                    /*
                     * euler:
                    Eigen::AngleAxisf rollAngle(rx, Eigen::Vector3f::UnitX());
                    Eigen::AngleAxisf pitchAngle(ry, Eigen::Vector3f::UnitY());
                    Eigen::AngleAxisf yawAngle(rz, Eigen::Vector3f::UnitZ());

                    Eigen::Matrix3f r =
                             (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
                             */

                    Eigen::Vector3f rotVec(rx,ry,rz);
                    Eigen::AngleAxisf raar(rotVec.norm(),rotVec.normalized());
                    Eigen::Matrix3f r=raar.toRotationMatrix();

                    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
                    pose.block<3,3>(0,0) = r;//.inverse();
                    pose(0,3) = x * scale;
                    pose(1,3) = y * scale;
                    pose(2,3) = z * scale;
                    //cout << pose << endl;
                    if(invertGTTrajectory){
                        pose.block<3,1>(0,3) = - pose.block<3,3>(0,0).inverse() * pose.block<3,1>(0,3);
                        pose.block<3,1>(0,3) = Vector3f(0,0,0);
                        p.position = pose;
                    }else{
                        p.position = pose.inverse();
                    }
                    trajectory.push_back(p);

                }
            }
        }


    }else{
        //Actually this is the only valid path
        string trajectoryFileName = folder + "/groundtruth.txt";
        ifstream trajectoryFile;
        trajectoryFile.open(trajectoryFileName);
        if(trajectoryFile.is_open()){
            hasPoses=usePose;

            /*TrajectoryPoint tp;
            tp.timesyamp=1;
            tp.position = Eigen::Matrix4f::Identity();
            trajectory.push_back(tp);
            */
            string line;
            while(getline(trajectoryFile,line)){
                if(line[0]!='#'){
                    //cout << line << endl;
                    TrajectoryPoint p;
                    float x,y,z;
                    float qx,qy,qz,qw;
                    sscanf(line.c_str(),"%lf %f %f %f %f %f %f %f",&p.timestamp,&x,&y,&z,&qx,&qy,&qz,&qw);

                    x *= trajectoryGTScale;
                    y *= trajectoryGTScale;
                    z *= trajectoryGTScale;
                    //read line to parameters and convert
                    Eigen::Affine3f transform(Translation3f(x,y,z));
                    Matrix4f t=transform.matrix();


                    Quaternionf quaternion(qw,qx,qy,qz);
                    quaternion.toRotationMatrix();
                    Matrix4f r=Matrix4f::Identity();

                    r.topLeftCorner<3,3>()=quaternion.toRotationMatrix();

                    p.position=t*r;
                    if(invertGTTrajectory){
                        //original!
                        //Matrix4f mat = p.position.inverse();
                        //p.position = mat;
                        //new attempt!!


                        Matrix4f mat = p.position;
                        p.position.block<3,3>(0,0) = mat.block<3,3>(0,0).inverse();
                        p.position.block<3,1>(0,3) = mat.block<3,1>(0,3);

                    }

                    trajectory.push_back(p);

                }
            }
        }
    }
    //tum intrinsics:
    rgbIntrinsics=Eigen::Vector4f(535.4,539.2,320.1,247.6);
    depthIntrinsics=rgbIntrinsics;
    depth2RGB = Eigen::Matrix4f::Identity();


    if(folder.find("tumalike") != string::npos){
        rgbIntrinsics=Eigen::Vector4f(537.562,537.278,313.73,243.601);
        depthIntrinsics = Eigen::Vector4f(563.937,587.847,328.987,225.661);
        if(folder.find("6") != string::npos ||
                folder.find("7") != string::npos ||
                folder.find("8") != string::npos||
                folder.find("9") != string::npos ||
                folder.find("8") != string::npos||
                folder.find("3") != string::npos||
                folder.find("5") != string::npos||
                folder.find("1") != string::npos){//the printer corner
            rgbIntrinsics=Eigen::Vector4f(565,575,315,220);
            depthIntrinsics = Eigen::Vector4f(563.937,587.847,328.987,225.661);

            //create the deviation between
            Matrix4f relDepthToColor = Matrix4f::Identity();
            relDepthToColor(0,2) =-0.026f; //i think the color camera is 2.6cm left of the depth camera
            Matrix3f rot=(AngleAxisf(-0.05*0.0,Vector3f::UnitX()) *
                    AngleAxisf(0.0*M_PI,  Vector3f::UnitY()) *
                          AngleAxisf(0.0*M_PI,  Vector3f::UnitZ())//somehow when doing this none
                          ).normalized().toRotationMatrix();
            Matrix4f rot4=Matrix4f::Identity();
            rot4.block<3,3>(0,0) = rot;
            cout<< rot << endl;
            depth2RGB = rot4*relDepthToColor;

            //basicly whats in the elastic fusion initialization
            rgbIntrinsics=Eigen::Vector4f(528, 528, 320, 240); //why is this wrong
            depthIntrinsics=Eigen::Vector4f(528, 528, 320, 240);
            depth2RGB = Matrix4f::Identity();

        }
    }
    /*
    if(folder.find("virt") != string::npos){
        rgbIntrinsics = Eigen::Vector4f(481.2,-480.0,308.258 , 243.525);
        depthIntrinsics = rgbIntrinsics;
        scaleDepth=1;
    }
    if(folder.find("georg") != string::npos){
        //setup the intrinsics according to georgs dataset.
        scaleDepth=5;
        rgbIntrinsics=Eigen::Vector4f(528.658 , 528.225,313.73,243.601);
        depthIntrinsics = Eigen::Vector4f(564.55 , 562.179,319.282 , 248.212);
    }

    if(folder.find("ir_test")){
        rgbIntrinsics=Eigen::Vector4f(528.658 , 528.225,313.73,243.601);
        depthIntrinsics=Eigen::Vector4f(528.658 , 528.225,313.73,243.601);
        depth2RGB = Matrix4f::Identity();
        //assert(0);

    }*/
    //check if the exposure file exists.
    ifstream exposureFile;
    if(_hasHighRes){
        exposureFile.open(folder + "/rgb_ids_exposure.txt");
    }else{
        exposureFile.open(folder + "/rgb_exposure.txt");
    }
    //(folder + "/rgb_ids_exposure.txt");
    if(exposureFile.is_open()){

        std::string line;
        while(getline(exposureFile,line)){

            std::string::size_type sz;     // alias of size_t
            float time = std::stof(line,&sz);
            exposureTimes.push_back(time);
        }

        cout << "these exposure times are all wrong and need to be assigned to the correct frame" << endl;

        //assert(0);//this doesn't work this way the exposure times need to be attached to the frames
//        assert(0);
        //cout << "oh! seemingly we have exposure control and one of these new supwerdatasets" << endl;
        cv::Size size;
        cv::Mat M;
        cv::Mat D;
        cv::Mat Mnew;
        if(_hasHighRes){
            cv::FileStorage intrinsicsIdsStorage(folder + "/../calib_result_ids.yml",cv::FileStorage::READ);
            if(!intrinsicsIdsStorage.isOpened()){
                assert(0);
            }
            intrinsicsIdsStorage["camera_matrix"] >> M;
            intrinsicsIdsStorage["distortion_coefficients"] >> D;
            intrinsicsIdsStorage["image_width"] >> size.width;
            intrinsicsIdsStorage["image_height"] >> size.height;

            Mnew = cv::getOptimalNewCameraMatrix(M,D,size,1,size);
            Mnew = M;//DEBUG
            rgbIntrinsics =
                    Eigen::Vector4f(Mnew.at<double>(0,0), Mnew.at<double>(1,1),
                                    Mnew.at<double>(0,2), Mnew.at<double>(1,2));
            cv::initUndistortRectifyMap(M,D,cv::Mat(),Mnew,size,CV_16SC2,rgbUndistort1,rgbUndistort2);



            float focalScale = 1.0f;//25f;
            rgbIntrinsics[0] *= focalScale;
            rgbIntrinsics[1] *= focalScale;

        }else{
            //do standard xtion stuff

            rgbIntrinsics =
                    Eigen::Vector4f(530,530,320,240);
        }


        /*
        cv::FileStorage intrinsicsIrStorage(folder + "/../newCalib/cam_params_ir.yml",cv::FileStorage::READ);
        if(!intrinsicsIrStorage.isOpened()){
            assert(0);
        }
        intrinsicsIrStorage["camera_matrix"] >> M;
        intrinsicsIrStorage["distortion_coefficients"] >> D;
        intrinsicsIrStorage["image_width"] >> size.width;
        intrinsicsIrStorage["image_height"] >> size.height;

        Mnew = cv::getOptimalNewCameraMatrix(M,D,size,1,size);


        depthIntrinsics =
                Eigen::Vector4f(Mnew.at<double>(0,0), Mnew.at<double>(1,1),
                        Mnew.at<double>(0,2), Mnew.at<double>(1,2));

        cv::initUndistortRectifyMap(M,D,cv::Mat(),Mnew,size,CV_16SC2,depthUndistort1,depthUndistort2);
        */

        //the standard values
        depthIntrinsics = Eigen::Vector4f(568, 568, 320, 240);//the structure sensor
        depthIntrinsics = Eigen::Vector4f(570, 570, 320, 240);//xtion






        //lets try to create a rotation and translation matrix:
        //create the deviation betweenx
        /*
        Matrix4f relDepthToColor = Matrix4f::Identity();
        relDepthToColor(0,3) = -0.045f;
        relDepthToColor(1,3) = -0.005f;
        relDepthToColor(2,3) = 0.0196f;
        //i think the color camera is 2.6cm left of the depth camera
        Matrix3f rot=(AngleAxisf(0.005*M_PI,Vector3f::UnitX()) *
                      AngleAxisf(0.0010*M_PI,  Vector3f::UnitY()) *
                      AngleAxisf(0.0*M_PI,  Vector3f::UnitZ())//somehow when doing this none
        ).normalized().toRotationMatrix();//why normalized
        Matrix4f rot4=Matrix4f::Identity();
        rot4.block<3,3>(0,0) = rot;//actually transpose would work as well
        cout<< rot << endl;
        //depth2RGB = rot4*relDepthToColor;
        cout << relDepthToColor << endl;
        cout << depth2RGB << endl;
        */



        cv::Mat R, T,Rf,Tf;

        if(_hasHighRes){
            Matrix4f rot4 = Matrix4f::Identity();


            //TRACK 16 - 19 should work with these settings:
            //Tweaking of the calibration because the camera rack is not rigid
            Matrix3f rot=(AngleAxisf(0.010*M_PI,Vector3f::UnitX()) *
                          AngleAxisf(0.002*M_PI,  Vector3f::UnitY()) *
                          AngleAxisf(0.0*M_PI,  Vector3f::UnitZ())).normalized().toRotationMatrix();
            Matrix4f rot41 = Matrix4f::Identity();
            rot41.block<3,3>(0,0) = rot;


            cv::FileStorage fs(folder + "/../extrinsics.yml",cv::FileStorage::READ);

            fs["R"] >> R;
            fs["T"] >> T;
            R.convertTo(Rf,CV_32FC1);
            T.convertTo(Tf,CV_32FC1);
            Eigen::Matrix3f eR(reinterpret_cast<float*>(Rf.data));
            Eigen::Vector3f eT(reinterpret_cast<float*>(Tf.data));
            //cout << eR << endl;
            //cout << eT << endl;


            rot4.block<3,3>(0,0) = eR;//.inverse();
            rot4.block<3,1>(0,3) = eT;


            depth2RGB =  rot41 * rot4; //rot41*

            //cout << depth2RGB << endl;

        }
        else{
            depth2RGB = Matrix4f::Identity();
            depth2RGB(0,3) = 0.026f;//standard xtion baseline
        }

        if(_hasHighRes){
            radiometricResponse = new radical::RadiometricResponse(folder +"/../rgb_ids.crf");
            vignettingResponse = new radical::VignettingResponse(folder + "/../rgb_ids.vgn");
        }else{
            radiometricResponse = new radical::RadiometricResponse(folder +"/../rgb.crf");
            vignettingResponse = new radical::VignettingResponse(folder + "/../rgb.vgn");
        }


    }
    if(folder.find("icl") != string::npos) {
        //scaleDepth=1.0f/5.0f;
    }




}

bool TumDataset::isRunning()
{
    return running;
}

cv::Mat TumDataset::getDepthFrame()
{
    readDepth=true;
    if(readDepth && readRGB){
        readDepth=readRGB=false;
        //readNewSetOfImages();
    }
    return currentDepth;
}

cv::Mat TumDataset::getRGBFrame()
{
    readRGB=true;
    if(readDepth && readRGB){
        readDepth=readRGB=false;
        //readNewSetOfImages();
    }
    return currentRGB;
}

Eigen::Vector4f TumDataset::getDepthIntrinsics()
{
    return depthIntrinsics;
}

Eigen::Vector4f TumDataset::getRgbIntrinsics()
{
    return rgbIntrinsics;
}

Eigen::Matrix4f TumDataset::getDepth2RgbRegistration()
{
    return depth2RGB;
}

bool TumDataset::hasGroundTruth()
{
    return hasPoses;
}

Eigen::Matrix4f TumDataset::getDepthPose()
{
    Matrix4f transform;//maybe not the right name
    double deltaTimeMin=1000;
    for(const TumDataset::TrajectoryPoint& p : trajectory){
        if(fabs(currentTimestamp-p.timestamp)<deltaTimeMin){
                deltaTimeMin=fabs(currentTimestamp-p.timestamp);
                transform=p.position;
            }
    }
    return transform;
}

Matrix4f TumDataset::getRgbPose()
{
    Matrix4f transform;//maybe not the right name
    double deltaTimeMin=1000;
    for(const TumDataset::TrajectoryPoint& p : trajectory){
        if(fabs(currentTimestamp-p.timestamp)<deltaTimeMin){
            deltaTimeMin=fabs(currentTimestamp-p.timestamp);
            transform=p.position;
        }
    }
    return transform;
}

TumDataset::~TumDataset() {
    if(radiometricResponse!=nullptr){
        delete radiometricResponse;
        delete vignettingResponse;
    }

}

bool TumDataset::hasHighRes() {
    return _hasHighRes;
}

float TumDataset::getRGBExposure() {
    return _rgbExposureTime;
}
