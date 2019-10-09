#include "gpu_norm_seg.h"

#include <vector>
#include <tuple>
#include <queue>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <cuda/norm_est.h>
#include <cuda/xtion_camera_model.h>

using namespace std;
using namespace Eigen;

GpuNormSeg::GpuNormSeg(GarbageCollector *garbage_collector, int width, 
                       int height) 
		: max_nr_points_per_segment(800),
		  max_extent_per_segment(30),
		  min_nr_points_per_segment(10),
		  dist_threshold(0.01),
		  max_distance(3.5f),
		  seg_count_(-1) {

	points_ = make_shared<gfx::GpuTex2D>(garbage_collector, GL_RGBA32F, GL_RGBA,
	                                     GL_FLOAT, width, height, true, 
	                                     (void*) 0);
	points_->name = "gpu based segmentation points";

	normals_ = make_shared<gfx::GpuTex2D>(garbage_collector, GL_RGBA32F, GL_RGBA,
	                                      GL_FLOAT, width, height, true, 
	                                      (void*) 0);
	normals_->name = "gpu based segmentation normals";

	gpu_segmentation_ = make_shared<gfx::GpuTex2D>(garbage_collector, GL_R32F, 
	                                               GL_RED, GL_FLOAT, width,
	                                               height, true, (void*) 0);
	gpu_segmentation_->name = "gpu based segmentation gpuSegmentation";

}

void GpuNormSeg::calcNormals() {
	int width = points_->getWidth();
	int height = points_->getHeight();

	cudaCalcNormals(d_std_max_std_->getCudaSurfaceObject(),
	                points_->getCudaSurfaceObject(),
	                normals_->getCudaSurfaceObject(),
	                width, height, 0.05);//threshold of 1 cm for normal calculation...
	return;
}

shared_ptr<gfx::GpuTex2D> GpuNormSeg::getGpuNormals() {
	return normals_;
}

void GpuNormSeg::calcPoints() {
	int width = d_std_max_std_->getWidth();
	int height = d_std_max_std_->getHeight();

	cudaCalcPoints(d_std_max_std_->getCudaSurfaceObject(),
	               points_->getCudaSurfaceObject(),
	               width, height, fxycxy);
}

shared_ptr<gfx::GpuTex2D> GpuNormSeg::getGpuPoints() {
	return points_;
}

cv::Mat generateColorCodedTexture(cv::Mat segmentation) {
	cv::Mat color_map(1, 64*48, CV_8UC4);

	color_map.at<cv::Vec4b>(0)  = cv::Vec4b( 51,  51,   0, 0);
	color_map.at<cv::Vec4b>(1)  = cv::Vec4b(  0,   0, 200, 0);
	color_map.at<cv::Vec4b>(2)  = cv::Vec4b(  0, 200,   0, 0);
	color_map.at<cv::Vec4b>(3)  = cv::Vec4b(200,   0,   0, 0);
	color_map.at<cv::Vec4b>(4)  = cv::Vec4b(  0, 200, 200, 0);
	color_map.at<cv::Vec4b>(5)  = cv::Vec4b(250,   0,   0, 0);
	color_map.at<cv::Vec4b>(6)  = cv::Vec4b(200, 200, 200, 0);
	color_map.at<cv::Vec4b>(7)  = cv::Vec4b(  0,   0, 100, 0);
	color_map.at<cv::Vec4b>(8)  = cv::Vec4b(  0, 100,   0, 0);
	color_map.at<cv::Vec4b>(9)  = cv::Vec4b(100,   0,   0, 0);
	color_map.at<cv::Vec4b>(10) = cv::Vec4b(  0, 100, 100, 0);
	color_map.at<cv::Vec4b>(11) = cv::Vec4b(100, 100,   0, 0);
	color_map.at<cv::Vec4b>(12) = cv::Vec4b(100, 100, 100, 0);

	int cols = 0;
	int rows = 0;
	for(int n = 13; n < color_map.cols; n++) {
		color_map.at<cv::Vec4b>(n) = cv::Vec4b(n / 10 * 50,
		                                       ((n % 10) / 5) * 50, 
		                                       (n % 5) * 50, 
		                                       0);
	}

	//TODO: take cols and rows from the segmentation Mat
	cols = segmentation.cols;
	rows = segmentation.rows;

	cv::Mat colored_image(rows, cols, CV_8UC4);
	for(int i = 0; i < rows; i++) {
		for(int j = 0; j < cols; j++) {
			asm("#begin asm test");//later use this to check if code is properly vectorized
			EIGEN_ASM_COMMENT("begin");
			colored_image.at<cv::Vec4b>(i,j) = color_map.at<cv::Vec4b>(
					0, segmentation.at<uint32_t>(i, j) % (64 * 48));
			if(segmentation.at<int32_t>(i, j) == -1) {
				colored_image.at<cv::Vec4b>(i, j) = cv::Vec4b(0, 0, 0, 255);
			}
			asm("#end asm test");
		}
	}
	return colored_image;
}

void GpuNormSeg::segment() {
	float max_dist = max_distance;
	float geometry_assign_threshold = 0.05;//5cm for instance
	int max_seg_size = 400;//this should come from somewhere else but in the meantime this will do
	float max_depth_step = 0.05;//50cm max depth step
	float min_cos_angle = 0.9;//determines how much the normals could be apart from each other on the edge
	int min_nr_points_per_segment = 10;//5;//the minimum amount of points to really generate a point
	int max_segment_extent = 30;
	int width = d_std_max_std_->getWidth();
	int height = d_std_max_std_->getHeight();

	Vector2i w[4] = {Vector2i( 0, -1), Vector2i(0, 1), 
	                 Vector2i(-1,  0), Vector2i(1, 0)};

	cv::Mat sensor_stds(height, width, CV_32FC4);
	d_std_max_std_->downloadData((void*) sensor_stds.data);
	cv::Mat pts(height, width, CV_32FC4);//propably don't need these
	points_->downloadData((void*) pts.data);
	cv::Mat norms(height, width, CV_32FC4);
	normals_->downloadData((void*) norms.data);

	cv::Mat depth(height, width, CV_8UC1);
	for(size_t i = 0; i < width * height; i++) {
		depth.at<uint8_t>(i) = sensor_stds.at<cv::Vec4f>(i)[0] * (255 / 5);
	}
	cv::Mat edges;
	cv::Canny(depth, edges, 10, 20);

	int dilation_size = 3;
	cv::Mat element = cv::getStructuringElement(
			cv::MORPH_ELLIPSE, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
			cv::Point(dilation_size, dilation_size));

	cv::Mat dilated_edges;
	cv::dilate(edges, dilated_edges, element);

	cv::Mat debug(height, width, CV_32FC4);
	points_->downloadData((void*) debug.data);

	#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
	imshow("projectedPoints", debug * 0.1);
	#endif
	normals_->downloadData((void*) debug.data);
	#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
	imshow("estimatedNormals", debug * -1.0f);
	waitKey();
	#endif

	cv::Mat mask(height, width, CV_8UC1);
	mask.setTo(0);
	for(size_t i = 0; i < existing_geometry_.rows; i++) {
		for(size_t j = 0; j < existing_geometry_.cols; j++) {
			float z = sensor_stds_.at<Vector4f>(i, j)[0];//depth
			if(isnan(z) || z > max_dist) {
				continue;
			}

			float exZ = existing_geometry_.at<Vector4f>(i, j)[2];
			if(isnan(exZ)) {
				//either there is no pixel for the existing geometry in this image
				mask.at<unsigned char>(i, j) = 255;
			} else {
				//or the new pixel is in front of the existing geometry
				//(if it is behind it we might want to delete geometry

				float thresh = max(existing_stds_.at<cv::Vec4f>(i, j)[2], //standard deviation. TODO: WRONG DATA FORMAT!!!
				                   sensor_stds_.at<cv::Vec4f>(i, j)[2]);//something not working here
				thresh = xtionStdToThresholdSeg(thresh);

				if(z < (exZ - thresh)) {
					mask.at<unsigned char>(i, j) = 255;
				}
			}
		}
	}

	#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
	imshow("existing geometry", existing_geometry);
	imshow("segmentation mask", mask);
	imshow("stds", stds);
	waitKey();
	#endif

	cv::Mat seg(height, width, CV_32SC1);
	seg.setTo(cv::Scalar(-1));

	//do the real segmentation

	//first element is count of the segment
	vector<int> segment_size;
	vector<int> segment_size_within_border;
	vector<Vector2f> starting_point_of_segment;

	//the tuple in this queue is the position at wich to continue and the
	//maybe no tuple is needed at all
	queue<tuple<Vector2i, //pixel position
	            int, //the segment index of the parent
	            float,//the distance of the parent pixel
	            Vector3f//the normal of the parent pixel
	            >> q;
	//pixel position, segment id

	//w window ( the neighborhood for this segmentation method)

	for(size_t i = 0; i < height; i++) {
		for(size_t j = 0; j < width; j++) {
			Vector4f d_std = sensor_stds.at<Vector4f>(i, j);
			if(isnan(d_std[0]) || !mask.at<uint8_t>(i, j)) {
				//don't add if the pixel is invalid
				continue;
			}
			if(seg.at<int>(i, j) != -1) {
				//don't add the point if its already part of another element
				continue;
			}

			Vector4f n4 = norms.at<Vector4f>(i, j);
			Vector3f n = Vector3f(n4[0], n4[1], n4[2]);
			seg.at<int>(i, j) = segment_size.size();
			segment_size.push_back(1);
			if(dilated_edges.at<uint8_t>(i, j) == 255) {
				segment_size_within_border.push_back(1);
			} else {
				segment_size_within_border.push_back(0);
			}
			starting_point_of_segment.push_back(Vector2f(i, j));
			for(int k = 0; k < 4; k++) {
				q.push(make_tuple(Vector2i(i, j) + w[k],
				                  segment_size.size() - 1,//the segment index
				                  d_std[0],//the depth
				                  n));//get the normal
			}

			while(!q.empty()) {
				auto f = q.front();
				q.pop();//we get rid of this
				Vector2i p = get<0>(f);
				int seg_index = get<1>(f);
				float depth_center = get<2>(f);
				Vector3f nc = get<3>(f);
				int seg_count = segment_size[seg_index];
				if(seg_count > max_seg_size) {
					continue;//this segment already has enough elements...
				}
				if((Vector2f(p[0], p[1]) - starting_point_of_segment[seg_index]).norm() >
				   max_segment_extent) {
					continue;
				}
				if(p[0] < 0 || p[1] < 0 || p[0] >= height || p[1] >= width) {
					continue; //out of bounds
				}
				//if this element already is part of a segment....
				//out of bou don't add it.
				if(seg.at<int>(p[0], p[1]) != -1) {
					continue;
				}
				Vector4f d_stdp = sensor_stds.at<Vector4f>(p[0], p[1]);
				if(isnan(d_stdp[0]) || !mask.at<uint8_t>(p[0], p[1])) {
					continue;
				}

				//TODO: derive the maximum standard deviation from the current
				//depth map
				max_depth_step = sensor_stds.at<cv::Vec4f>(p[0], p[1])[2];

				max_depth_step = xtionStdToThresholdSeg(
						sensor_stds.at<cv::Vec4f>(p[0], p[1])[2]);
				//test if we have a discontinuity on the edge
				if(abs(d_stdp[0] - depth_center) > max_depth_step) {
					continue;
				}
				//test if the normals are too far apart from each other
				Vector4f np4 = norms.at<Vector4f>(p[0], p[1]);
				Vector3f np  = Vector3f(np4[0], np4[1], np4[2]);

				//make this element part of everything:
				seg.at<int>(p[0], p[1]) = seg_index;
				segment_size[seg_index]++;

				if(dilated_edges.at<uint8_t>(p[0], p[1]) == 255) {
					segment_size_within_border[seg_index]++;
				}

				for(int k = 0; k < 4; k++) {
					//lets continue with all the neighbours
					q.push(make_tuple(p + w[k], segment_size.size() - 1, d_stdp[0], np));
				}
			}
		}
	}

	/*************************************************************/
	//PREPROCESSING to get rid of tiny holes in segmentation by connecting
	//too small objects to fitting bigger ones
	//Untested

	vector<int> new_inds1(segment_size.size() + 1);
	for(size_t i = 0; i < new_inds1.size(); i++) {
		new_inds1[i] = i - 1;
	}
	for(size_t i = 0; i < height; i++) {
		for(size_t j = 0; j < width; j++) {
			int seg_ind = seg.at<int>(i, j);
			if(seg_ind == -1) {
				continue;//invalid pixel anyway
			}
			float d = sensor_stds.at<Vector4f>(i, j)[0];
			if(segment_size[seg_ind] < min_nr_points_per_segment) {
				//iterate over the neighbours if we can
				max_depth_step = xtionStdToThresholdSeg(sensor_stds.at<cv::Vec4f>(i, j)[2]);
				for(size_t k = 0; k < 4; k++) {
					Vector2i p = Vector2i(i, j) + w[k];
					if(p[0] < 0 || p[1] < 0 || p[0] >= height || p[1] >= width) {
						continue; //out of bounds
					}
					int ind_other = seg.at<int>(p[0], p[1]);
					if(ind_other == -1) {
						continue;
					}
					if(segment_size[ind_other] < min_nr_points_per_segment) {
						continue;
					}

					float ddebug = sensor_stds.at<Vector4f>(p[0], p[1])[0];
					if(abs(sensor_stds.at<Vector4f>(p[0], p[1])[0] - d) <
					   max_depth_step) {
						new_inds1[seg_ind + 1] = ind_other;
					}
				}
			}
		}
	}

	for(size_t i = 0; i < width * height; i++) {
		int seg_ind = seg.at<int>(i);
		seg.at<int>(i) = new_inds1[seg_ind + 1];
	}
	/**********************************************************/

	//imshow("pre filter segmentation",generateColorCodedTexture(seg));
	//destroy the elements with only one or less than.... lets say 3 pixel
	vector<int> new_inds(segment_size.size()+1);
	new_inds[0] = -1;
	seg_count_ = 0;
	for(size_t i = 0; i < segment_size.size(); i++) {
		//only keep a segment when less than 80% are within a edge region
		if(segment_size_within_border[i] * 10 > segment_size[i] * 8) {
			new_inds[i + 1] = -1;
			continue;
		}
		if(segment_size[i] < min_nr_points_per_segment){
			new_inds[i + 1] = -1;
			continue;
			//TODO: actually in this case we have to check
			//if there is at least one valid neighbour we could
			//connect it to.
		}
		new_inds[i + 1] = seg_count_;
		seg_count_++;
	}
	//replace the segments with too little pixel
	for(size_t i = 0; i < width * height; i++) {
		seg.at<int>(i) = new_inds[seg.at<int>(i) + 1];
	}
	seg_result_ = seg;
	#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
	imshow("segmentation", generateColorCodedTexture(seg));
	waitKey();
	#endif
}

shared_ptr<gfx::GpuTex2D> GpuNormSeg::getGpuSegmentation() {
	cout << "[GpuNormSeg::getGpuSegmentation] "
	        "The segmentation is implemented on CPU not GPU" << endl;
	return gpu_segmentation_;
}

cv::Mat GpuNormSeg::getSegmentation() {
	return seg_result_;
}

int GpuNormSeg::getSegCount() {
	return seg_count_;
}

shared_ptr<gfx::GpuTex2D> GpuNormSeg::segment(
		shared_ptr<gfx::GpuTex2D> d_std_max_std, cv::Mat existing_d_std_max_std,
		cv::Mat existing_geometry) {
	auto start = chrono::system_clock::now();
	existing_geometry_ = existing_geometry;
	d_std_max_std_ = d_std_max_std;
	existing_stds_ = existing_d_std_max_std;
	calcPoints();
	calcNormals();
	segment();
	auto end = chrono::system_clock::now();
	auto elapsed = chrono::duration_cast<chrono::milliseconds>(end - start);
	//present
	cout << "[GpuNormSeg::segment] time consumed by segmentation: " <<
	        elapsed.count() << "ms" << endl;

	return gpu_segmentation_;
}
