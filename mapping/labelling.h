#ifndef FILE_LABELLING_H
#define FILE_LABELLING_H

#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class ActiveSet;
class MeshReconstruction;

namespace gfx {
	class GpuTex2D;
} // namespace gfx

//there is the same class in the gpu namespace
class Labelling {
public:
	
	void projectLabels(shared_ptr<ActiveSet> active_set, cv::Mat &labels,
	                   shared_ptr<gfx::GpuTex2D> d_std_tex, Matrix4f pose);

	void applyLabels(shared_ptr<ActiveSet> active_set, cv::Mat labels, 
	                 Matrix4f pose);

	MeshReconstruction *mesh_reconstruction;
};

#endif // FILE_LABELLING_H
