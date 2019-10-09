#include "ICPOdometry.h"

using namespace Eigen;

ICPOdometry::ICPOdometry(int width, int height, float cx, float cy, float fx, 
                         float fy, float dist_thresh, float angle_thresh)
		: last_error(0),
		  last_inliers(width * height),
		  dist_thresh_(dist_thresh),
		  angle_thresh_(angle_thresh),
		  width_(width),
		  height_(height),
		  intr_(Intr(fx, fy, cx, cy)) {

	sum_data_.create(MAX_THREADS);
	out_data_.create(1);

	iterations_.reserve(NUM_PYRS_);

	depth_tmp_.resize(NUM_PYRS_);

	vmaps_prev_.resize(NUM_PYRS_);
	nmaps_prev_.resize(NUM_PYRS_);
	vmaps_curr_.resize(NUM_PYRS_);
	nmaps_curr_.resize(NUM_PYRS_);

	for (int i = 0; i < NUM_PYRS_; ++i) {
		int pyr_rows = height >> i;
		int pyr_cols = width >> i;

		depth_tmp_[i].create(pyr_rows, pyr_cols);

		vmaps_prev_[i].create(pyr_rows * 3, pyr_cols);
		nmaps_prev_[i].create(pyr_rows * 3, pyr_cols);

		vmaps_curr_[i].create(pyr_rows * 3, pyr_cols);
		nmaps_curr_[i].create(pyr_rows * 3, pyr_cols);
	}
}

void ICPOdometry::initICP(unsigned short *depth, const float depth_cutoff) {
	depth_tmp_[0].upload(depth, sizeof(unsigned short) * width_, height_, width_);

	for(int i = 1; i < NUM_PYRS_; ++i) {
		pyrDown(depth_tmp_[i - 1], depth_tmp_[i]);
	}

	for(int i = 0; i < NUM_PYRS_; ++i)	{
		createVMap(intr_(i), depth_tmp_[i], vmaps_curr_[i], depth_cutoff);
		createNMap(vmaps_curr_[i], nmaps_curr_[i]);
	}

	cudaDeviceSynchronize();
}

void ICPOdometry::initICPModel(unsigned short * depth, 
                               const float depth_cutoff) {
	depth_tmp_[0].upload(depth, sizeof(unsigned short) * width_, height_, width_);

	for(int i = 1; i < NUM_PYRS_; ++i) {
		pyrDown(depth_tmp_[i - 1], depth_tmp_[i]);
	}

	for(int i = 0; i < NUM_PYRS_; ++i) {
		createVMap(intr_(i), depth_tmp_[i], vmaps_prev_[i], depth_cutoff);
		createNMap(vmaps_prev_[i], nmaps_prev_[i]);
	}

	cudaDeviceSynchronize();
}

void ICPOdometry::getIncrementalTransformation(Sophus::SE3d &T_prev_curr, 
                                               int threads, int blocks) {
	iterations_[0] = 10;
	iterations_[1] = 5;
	iterations_[2] = 4;

	for(int i = NUM_PYRS_ - 1; i >= 0; i--) {
		for(int j = 0; j < iterations_[i]; j++) {
			float residual_inliers[2];
			Matrix<float, 6, 6, RowMajor> A_icp;
			Matrix<float, 6, 1> b_icp;

			estimateStep(T_prev_curr.rotationMatrix().cast<float>().eval(),
			             T_prev_curr.translation().cast<float>().eval(),
			             vmaps_curr_[i], nmaps_curr_[i], intr_(i), vmaps_prev_[i],
			             nmaps_prev_[i], dist_thresh_,  angle_thresh_, sum_data_, 
			             out_data_, A_icp.data(), b_icp.data(), &residual_inliers[0], 
			             threads, blocks);

			last_error = sqrt(residual_inliers[0]) / residual_inliers[1];
			last_inliers = residual_inliers[1];

			const Matrix<double, 6, 1> update =
					A_icp.cast<double>().ldlt().solve(b_icp.cast<double>());

			T_prev_curr = Sophus::SE3d::exp(update) * T_prev_curr;
		}
	}
}
