#ifndef FILE_TEXTURE_UPDATER_H
#define FILE_TEXTURE_UPDATER_H

#include <memory>

#include <Eigen/Eigen>

#include <base/mesh_structure.h>

using namespace std;
using namespace Eigen;

class MeshReconstruction;

class TextureUpdater {
public:

	void generateGeomTex(MeshReconstruction* reconstruction,
						 vector<shared_ptr<Meshlet> > &new_patches,
						 Matrix4f pose, Matrix4f proj,
						 shared_ptr<gfx::GpuTex2D> geom_sensor_data,
						 shared_ptr<ActiveSet> active_set,
						 InformationRenderer* information_renderer);

	void projToGeomTex(ActiveSet *active_set, 
	                   vector<shared_ptr<Meshlet> > &new_patches,
	                   shared_ptr<gfx::GpuTex2D> geom_sensor_data,
	                   Matrix4f pose, Matrix4f proj);

	//TODO: put this to geometryUpdate else or split it up properly
	void vertGeomTexUpdate(shared_ptr<gfx::GpuTex2D> d_std_tex,
	                       Matrix4f depth_pose_in,
	                       shared_ptr<ActiveSet> &active_set);

	void colorTexUpdate(MeshReconstruction* reconstruction,
						shared_ptr<gfx::GpuTex2D> rgba_tex,
	                    LowDetailRenderer* low_detail_renderer,
	                    Matrix4f color_pose_in,
	                    shared_ptr<ActiveSet> &active_set);

	void applyColorData(MeshReconstruction* reconstruction,
						vector<shared_ptr<Meshlet>> &visible_patches,
						LowDetailRenderer* low_detail_renderer,
	                    shared_ptr<gfx::GpuTex2D> rgb_in,
	                    Matrix4f &pose, Matrix4f &proj, 
	                    shared_ptr<ActiveSet> active_set);

	void genLookupTexGeom(MeshReconstruction* reconstruction,
						  ActiveSet *active_set,
						  vector<shared_ptr<Meshlet> > &patches,
						  InformationRenderer* information_renderer);

	void genLookupTex(MeshReconstruction* reconstruction,
					  ActiveSet *active_set,
					  vector<shared_ptr<Meshlet> > &patches,
					  vector<shared_ptr<MeshTexture>> &textures,
					  InformationRenderer* information_renderer,
					  bool dilate = true);

	//MeshReconstruction *mesh_reconstruction;//TODO: remove this
};

#endif // FILE_TEXTURING_H
