#ifndef FILE_TEXTURING_H
#define FILE_TEXTURING_H

#include <memory>

#include <Eigen/Eigen>

#include "base/mesh_structure.h"

using namespace std;
using namespace Eigen;

class MeshReconstruction;

class Texturing {
public:

	void generateGeomTex(vector<shared_ptr<MeshPatch> > &new_patches,
	                     Matrix4f pose, Matrix4f proj,
	                     shared_ptr<gfx::GpuTex2D> geom_sensor_data,
	                     shared_ptr<ActiveSet> active_set);

	void projToGeomTex(ActiveSet *active_set, 
	                   vector<shared_ptr<MeshPatch> > &new_patches,
	                   shared_ptr<gfx::GpuTex2D> geom_sensor_data,
	                   Matrix4f pose, Matrix4f proj);

	//TODO: put this to geometryUpdate else or split it up properly
	void vertGeomTexUpdate(shared_ptr<gfx::GpuTex2D> d_std_tex,
	                       Matrix4f depth_pose_in,
	                       shared_ptr<ActiveSet> &active_set);

	void colorTexUpdate(shared_ptr<gfx::GpuTex2D> rgba_tex,
	                    Matrix4f color_pose_in,
	                    shared_ptr<ActiveSet> &active_set);

	void applyColorData(vector<shared_ptr<MeshPatch>> &visible_patches,
	                    shared_ptr<gfx::GpuTex2D> rgb_in,
	                    Matrix4f &pose, Matrix4f &proj, 
	                    shared_ptr<ActiveSet> active_set);

	void genLookupTexGeom(ActiveSet *active_set, 
	                      vector<shared_ptr<MeshPatch> > &patches);

	void genLookupTex(ActiveSet *active_set,
	                  vector<shared_ptr<MeshPatch> > &patches,
	                  vector<shared_ptr<MeshTexture>> &textures,
	                  bool dilate = true);

	MeshReconstruction *mesh_reconstruction;
};

#endif // FILE_TEXTURING_H
