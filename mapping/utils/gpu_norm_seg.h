#ifndef FILE_GPU_NORM_SEG_H
#define FILE_GPU_NORM_SEG_H

#include <memory>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <gfx/gpu_tex.h>

using namespace std;
using namespace Eigen;

class GpuNormSeg {
public:
	
	GpuNormSeg(GarbageCollector* garbage_collector, int width, int height);

	void calcNormals();
	shared_ptr<gfx::GpuTex2D> getGpuNormals();

	void calcPoints(shared_ptr<gfx::GpuTex2D> d_std_max_std, Vector4f fxycxy);
	shared_ptr<gfx::GpuTex2D> getGpuPoints();

	void segment();
	shared_ptr<gfx::GpuTex2D> getGpuSegmentation();
	cv::Mat getSegmentation();
	int getSegCount();
	void calcPoints();
	shared_ptr<gfx::GpuTex2D> segment(shared_ptr<gfx::GpuTex2D> d_std_max_std,
	                                  cv::Mat existing_d_std_max_std,
	                                  cv::Mat existing_geometry);

	int max_nr_points_per_segment;
	int max_extent_per_segment;
	int min_nr_points_per_segment;
	float dist_threshold;
	float max_distance;
	Vector4f fxycxy;

private:
	shared_ptr<gfx::GpuTex2D> d_std_max_std_;
	shared_ptr<gfx::GpuTex2D> points_;
	shared_ptr<gfx::GpuTex2D> normals_;
	shared_ptr<gfx::GpuTex2D> gpu_segmentation_;
	cv::Mat existing_geometry_;
	cv::Mat sensor_stds_;
	cv::Mat existing_stds_;
	cv::Mat seg_result_;
	int seg_count_;

};

#endif // FILE_GPU_NORM_SEG_H
