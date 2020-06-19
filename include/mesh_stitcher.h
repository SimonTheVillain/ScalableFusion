#ifndef FILE_MESH_STITCHER_H
#define FILE_MESH_STITCHER_H

#include <vector>

#include <opencv2/core.hpp>

#include <base/mesh_structure.h>

using namespace std;
using namespace Eigen;

class MeshReconstruction;

class MeshStitcher {
public:

	void setup(MeshReconstruction *reconstruction) {
		mesh_reconstruction = reconstruction;
	}

	void rasterBorderGeometry(Matrix4f view,
	                          Matrix4f proj, cv::Mat geometry);

	void rasterLineGeometry(Matrix4f view, Matrix4f proj, Edge *edge, 
	                        cv::Mat geometry, cv::Mat debug);

	void genBorderList(vector<shared_ptr<Meshlet>> &patches,
	                   Matrix4f debug_proj_pose);

	void reloadBorderGeometry(shared_ptr<ActiveSet> active_set);

	//TODO: also download the geometry of such list
	void freeBorderList();

	void stitchOnBorders(Matrix4f view,
	                     Matrix4f proj, cv::Mat std_proj, cv::Mat geom_proj_m, 
	                     cv::Mat new_geom_m, cv::Mat new_std, 
	                     cv::Mat debug_color_coded_new_segmentation, 
	                     cv::Mat new_seg_p_m, cv::Mat new_pt_ind_m);

    void stitchOnBorders2(Matrix4f view,
                         Matrix4f proj, cv::Mat std_proj, cv::Mat geom_proj_m,
                         cv::Mat new_geom_m, cv::Mat new_std,
                         cv::Mat debug_color_coded_new_segmentation,
                         cv::Mat new_seg_p_m, cv::Mat new_pt_ind_m);


    void sewLocally(Vector2i center_pix, float center_pix_z, Vector2i last_new_pix,
            Vertex* vfar, Vertex* v0, Vertex* v1,
            Vector2f pfar, Vector2f p0, Vector2f p1,
            const cv::Mat &new_meshlets, const cv::Mat &new_vert_inds, const cv::Mat &new_geom,
            bool nbs_used[4],
            bool flip = false);

	vector<vector<Edge>> border_list;
	
	MeshReconstruction *mesh_reconstruction;

	void checkTriangleEdgeConsistency();



};

#endif // FILE_MESH_STITCHER_H
