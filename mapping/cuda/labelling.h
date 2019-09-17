// TODO: reintroduce include guards, once the problem with the double
//       labelling.h has been fixed
//#ifndef FILE_LABELLING_H
//#define FILE_LABELLING_H

//whatever will be going on in here
#include <cuda.h>
#include <cublas.h>
#include <vector>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>

#include "gpuMeshStructure.h"
#include "gpuErrchk.h"

namespace gpu{
    class Labelling{
    public:
        struct SegProjTask{
            int subchannel;
            cudaSurfaceObject_t lookupSurf;
            cv::Point2i lookup;
            cudaSurfaceObject_t destSurf;
            cv::Rect2i destination;//we might need more than this
            size_t vertexDestStartInd;
        };

        static void labelSurfaces(std::vector<SegProjTask> tasks,
                                  const cudaSurfaceObject_t labelling,
                //the depth and geometry at the current view
                                  const cudaSurfaceObject_t geomBuffer,
                                  cv::Size2i resolution,
                                  Eigen::Matrix4f _pose,
                                  Eigen::Matrix4f proj_pose,
                                  GpuVertex *vertices,
                                  Eigen::Vector2f *texPos,
                                  GpuTriangle* triangles,
                                  GpuPatchInfo* patchInfos);

        struct InitializeTask{
            cudaSurfaceObject_t destSurf;
            cv::Rect2i destRect;
        };


        template<class T>
        static void initializeSurfaces(std::vector<InitializeTask> tasks,
                                       T value);






    };

}

//#endif

