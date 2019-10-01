#include "meshReconstruction.h"

#include <iostream>

#include <gpuTex.h>
#include <shader.h>
#include "cuda/coalescedMemoryTransfer.h"
#include "cuda/xtionCameraModel.h"
#include "cuda/texCoords.h"
#include "cuda/texPatchInit.h"
#include "cuda/stdTexUpdate.h"
#include "cuda/geomUpdate.h"
#include "debugRender.h"
#include "gpu/ActiveSet.h"
#include "utils/gpuNormSeg.h"
#include "StitchingUtils.h"
#include "camera.h"
#include "worker.h"

using namespace std;
using namespace Eigen;
/*
#include "../icpCUDA/ICPOdometry.h"//this is not supposed to be the way how to include a library
//TODO: add icp to this project
//#include <ICPOdometry.h>
*/

void MeshReconstruction::initInGlLogicContext() {

	if(initializing_logic.exchange(true)) {
		//Apparently the logic already is beeing initialized somewhere
		//therefore we wait for this to complete:
		while(!gl_logic_initialized_) {
			unique_lock<mutex> lk(condition_variable_mutex_);
			condition_variable_.wait(lk);
		}

		//if completed we initialize the informationRenderer in this thread
		information_renderer.initInContext();

		//since all the remaining stuff is alredy initialized wo do only the stuff
		//that is needs thread dependant initialization
		return;
	}

	//after the buffers are created we can initialize the rendering stuff:

	//initializing the render logic of some other stuff

	///New  New functionality that gets rid of all the preveous initialization stuff
	gpu_geom_storage_.initialize();

	//this requires the geometry storage to be initialized.
	information_renderer.initInContext(params.depth_res.width, 
	                                   params.depth_res.height, this);

	//Generating the active sets for rendering and capturing TODO?

	tex_atlas_geom_lookup_ = make_shared<TexAtlas>(garbage_collector_, GL_RGBA32F, 
	                                               GL_FLOAT,GL_RGBA,CV_32FC4,
	                                               1024, &fbo_storage_);
	tex_atlas_stds_ = make_shared<TexAtlas>(garbage_collector_, GL_RGBA16F, 
	                                        GL_FLOAT, GL_RGBA, CV_32FC4,
	                                        1024, &fbo_storage_);
	tex_atlas_rgb_8_bit_ = make_shared<TexAtlas>(garbage_collector_, GL_RGBA,
	                                             GL_UNSIGNED_BYTE, GL_RGBA, 
	                                             CV_8UC4, 1024, &fbo_storage_);

	cout << "TODO: SETUP THE TEXTURES TO BE NON_INTERPOLATED" << endl;
	tex_atlas_seg_labels_ = make_shared<TexAtlas>(garbage_collector_, GL_RGBA32F,
	                                              GL_FLOAT, GL_RGBA, CV_32FC4,
	                                              1024, &fbo_storage_);

	//maybe in opengl it still is better ineterpreting these as floats
	//and doing a reinterpret cast
	tex_atlas_sem_seg_labels_weights_ = make_shared<TexAtlas>(garbage_collector_,
	                                                          GL_RGBA32I,
	                                                          GL_INT,
	                                                          GL_RGBA_INTEGER,
	                                                          CV_32SC4, 1024,
	                                                          &fbo_storage_);

	GLuint test;
	glGenTextures(1, &test);
	glBindTexture(GL_TEXTURE_2D, test);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32I, 1024, 1024, 0, GL_RGBA_INTEGER,
	             GL_INT, nullptr);

	gfx::GLUtils::checkForOpenGLError("Error while creating glTexture");

	//afterwards send message to other thread:
	gl_logic_initialized_ = true;
	condition_variable_.notify_all();
}

void MeshReconstruction::cleanupGlStoragesThisThread_() {
	fbo_storage_.garbageCollect();
}

bool MeshReconstruction::removePatch(shared_ptr<MeshPatch> patch) {
	patch->double_stitch_mutex.lock();
	for(size_t i = 0; i < patch->double_stitches.size(); i++) {
	   patch->double_stitches[i]->removeFromPatches(patch);
	}
	patch->double_stitch_mutex.unlock();

	patch->triple_stitch_mutex.lock();
	for(size_t i = 0; i < patch->triple_stitches.size(); i++) {
		patch->triple_stitches[i]->removeFromPatches(patch);
	}
	patch->triple_stitch_mutex.unlock();

	patches_mutex_.lock();
	for(auto it = patches_.begin(); it != patches_.end(); ++it) {
		if(it->second == patch) {
			patches_.erase(it);
			patches_mutex_.unlock();
			return true;
		}
	}
	patches_mutex_.unlock();
	return false;
}

cv::Mat MeshReconstruction::generateColorCodedTexture_(cv::Mat segmentation) {

	cv::Mat color_map(1, 64 * 48, CV_8UC4);
	color_map.at<cv::Vec4b>(0)  = cv::Vec4b(  0,   0,   0, 0);
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
	for (int n = 13; n < color_map.cols; n++) {
		color_map.at<cv::Vec4b>(n) = cv::Vec4b(n / 10 * 50, ((n % 10) / 5) * 50, 
		                                       (n % 5) * 50, 0);
	}

	//TODO: take cols and rows from the segmentation Mat
	cols = segmentation.cols;
	rows = segmentation.rows;

	cv::Mat colored_image(rows, cols, CV_8UC4);
	for(int i = 0; i < rows; i++) {
		for(int j = 0; j < cols; j++) {
			asm("#begin asm test");//later use this to check if code is properly vectorized
			EIGEN_ASM_COMMENT("begin");
			colored_image.at<cv::Vec4b>(i, j) = 
				color_map.at<cv::Vec4b>(0, segmentation.at<uint64_t>(i, j) % (64 * 48));
			asm("#end asm test");
		}
	}
	return colored_image;
}


Matrix4f MeshReconstruction::genDepthProjMat() {
	float fx = params.depth_fxycxy[0];
	float fy = params.depth_fxycxy[1];
	float cx = params.depth_fxycxy[2];
	float cy = params.depth_fxycxy[3];

	Matrix4f proj1;//one to do what has to be done anyway
	proj1 << fx,  0, cx,  0,
	          0, fy, cy,  0,
	          0,  0,  0, -1,
	          0,  0,  1,  0;

	float w = params.depth_res.width;
	float h = params.depth_res.height;
	float zmin = 0.1f;
	float zmax = 30.0f;
	float b = 2.0f / (1.0f / zmin - 1.0f / zmax);
	float a = b / zmax + 1.0f;
	Matrix4f proj2; //the other one to scale everything to the normalized coordinates
	proj2 << 2.0f / w,        0, 0,  -1.0f + 1.0f / w,
	                0, 2.0f / h, 0,  -1.0f + 1.0f / h,
	                0,        0, b,                 a,
	                0,        0, 0,                 1;

	return proj2 * proj1;
}

void MeshReconstruction::setActiveSetUpdate_(shared_ptr<ActiveSet> set) {
	active_set_update_mutex.lock();
	active_set_update = set;
	active_set_update_mutex.unlock();
}

MeshReconstruction::MeshReconstruction(GLFWwindow *context, 
                                       GarbageCollector *garbage_collector,
                                       bool threaded, 
                                       int depth_width, int depth_height, 
                                       int rgb_width, int rgb_height) 
		: garbage_collector_(garbage_collector),
		  initializing_logic(false) {

	params.depth_res.width = depth_width;
	params.depth_res.height = depth_height;
	params.rgb_res.width = rgb_width;
	params.rgb_res.height = rgb_height;
	texturing.meshReconstruction = this;

	//TODO: name these setup functions consistently
	geometry_update.setup(this);
	gpu_geom_storage_.init(this);
}


//we assume that the opengl context still exists when running the destructor
// (otherwise the opengl stuff would not behave nicely)
MeshReconstruction::~MeshReconstruction() {
	if(rendering_active_set_update_worker_ != nullptr) {
		delete rendering_active_set_update_worker_;
	}
	//TODO: check for the textures.... are they deleted as well?
}

shared_ptr<MeshPatch> MeshReconstruction::genMeshPatch() {
	current_max_patch_id_++;
	shared_ptr<MeshPatch> mesh_patch(new MeshPatch(&octree_));
	mesh_patch->id = current_max_patch_id_;
	mesh_patch->weak_self = mesh_patch;
	patches_[current_max_patch_id_] = mesh_patch;
	return mesh_patch;
}

shared_ptr<MeshTexture> MeshReconstruction::genMeshTexture(
		MeshTexture::Type content_type) {
	return make_shared<MeshTexture>(content_type, this);
}

void MeshReconstruction::setRGBIntrinsics(Vector4f fxycxy) {
	params.rgb_fxycxy = fxycxy;
}

void MeshReconstruction::setDepthIntrinsics(Vector4f fxycxy) {
	params.depth_fxycxy = fxycxy;
}

bool MeshReconstruction::hasGeometry() {
	return patches_.size() > 0;
}

void MeshReconstruction::erase() {
	//The scheduler should be closed at this point

	vector<weak_ptr<MeshPatch>> patches;//collect weak pointers to that stuff to see what of this is still existent
	vector<shared_ptr<MeshPatch>> shared_patches;//collect weak pointers to that stuff to see what of this is still existent
	vector<weak_ptr<MeshPatchGpuHandle>> gpu_patches;
	vector<weak_ptr<MeshTextureGpuHandle>> gpu_textures;
	int debug = 0 ;
	for(auto patch : patches_) {
		debug++;
		patches.push_back(patch.second);
		gpu_patches.push_back(patch.second->gpu);
		shared_patches.push_back(patch.second);
		shared_ptr<MeshPatchGpuHandle> gpu_patch = patch.second->gpu.lock();
		if(gpu_patch != nullptr) {
			gpu_textures.push_back(gpu_patch->geom_tex);
		}
	}

	patches_.clear();
	active_set_update.reset();
	active_set_rendering_.reset();
	active_set_expand.reset();

	for(int i = 0; i < debug; i++) {
		shared_patches.pop_back();
	}

	//DEBUG:
	//all of this should be zero: TODO FIND OUT WHY, parse through all the texatlas thingis
	//TODO: follow down the octree ( objects don't seem to be deleted accordingly)
	//and then find who is holding references to the gpu structures

	//check how many textures are left now:
	int tex_count = tex_atlas_stds_->countTex() + 
	                tex_atlas_geom_lookup_->countTex() +
	                tex_atlas_rgb_8_bit_->countTex() +
	                tex_atlas_seg_labels_->countTex();
	cout << "texCount overall: " << tex_count << " stds " << 
	        tex_atlas_stds_->countTex() << " lookup " << 
	        tex_atlas_geom_lookup_->countTex() << " rgb " << 
	        tex_atlas_rgb_8_bit_->countTex() << endl;

	int patch_count = tex_atlas_stds_->countPatches() + 
	                  tex_atlas_geom_lookup_->countPatches() +
	                  tex_atlas_rgb_8_bit_->countPatches() +
	                  tex_atlas_seg_labels_->countPatches();
	cout << "patchCount overall: " << patch_count << " stds " << 
	        tex_atlas_stds_->countPatches() << " lookup " << 
	        tex_atlas_geom_lookup_->countPatches() << " rgb " << 
	        tex_atlas_rgb_8_bit_->countPatches() << endl;

	for(int i = 0; i < patches.size(); i++) {
		assert(patches[i].expired());
	}
	int active_gpu_patches = 0;
	for(int i = 0; i < gpu_patches.size(); i++) {
		active_gpu_patches += gpu_patches[i].use_count();
	}
	int active_gpu_textures = 0;
	for(int i = 0; i < gpu_textures.size(); i++) {
		active_gpu_textures += gpu_textures[i].use_count();
	}
	vector<gfx::GpuTex2D*> textures = gfx::GpuTex2D::getTexList();

	//TODO: mechanism to delete all fbos that are used in thread
	cout << "FBOs active " << getFboCountDebug_() << endl; // should be zero
	cout << "GpuPatches active" << active_gpu_patches << endl;
	cout << "GpuTextures active" << active_gpu_textures << endl;
	cout << "overall tex count" << gfx::GpuTex2D::getTexCount() << endl;
	glFinish();//wait until all the commands are executed.
	tex_atlas_stds_->countPatches();
	tex_atlas_stds_->countTex();//Why is this bigger 0 in this case????????
	cout << "end of this analysis" << endl;
}

vector<shared_ptr<MeshPatch>> MeshReconstruction::GetAllPatches() {
	vector<shared_ptr<MeshPatch>> patches;
	for(auto patch : patches_) {
		patches.push_back(patch.second);
	}
	return patches;
}

/*
 * the Purpose of this whole file is to stick the geometry of two different frames together.
 * Therefore we plan to project the old geometry boarder into the new frame.
 * There we look for bordering pixel which are containing new geometry.
 * In this way we find the closest new geometry and are able to do proper stitching.
 *
 * Things we need:
 * Getting a already existing stitch (otherwise creating a new one
 *
 */

TriangleReference MeshReconstruction::addTriangle_(VertexReference pr1,
                                                   VertexReference pr2,
                                                   VertexReference pr3) {
	//same method as below but without the debug
	assert(0);
}

//Creating a new triangle just from vertex references.
//used in the inter frame stitching process
int debug_global_stitch_triangle_ctr = 0;
TripleStitch *debug_quenstionable_triplestitch = nullptr;

TriangleReference MeshReconstruction::addTriangle_(
		VertexReference pr1, VertexReference pr2, VertexReference pr3,
		vector<weak_ptr<GeometryBase>> &debug_new_stitches) {

	//TODO: remove this debug output
	//cout << "This method is untested when it comes to its ability to register triangles" << endl;
	Triangle triangle;
	triangle.points[0] = pr1;
	triangle.points[1] = pr2;
	triangle.points[2] = pr3;
	triangle.debug_is_stitch = true;
	triangle.debug_nr = debug_global_stitch_triangle_ctr;
	if(triangle.debug_nr == 11757) {
		cout << "bad bad triangle! debug here!" << endl;
	}
	debug_global_stitch_triangle_ctr++;

	if(pr1.getPatch() == pr2.getPatch() && pr1.getPatch() == pr3.getPatch()) {
		cout << "ERROR [scaleableMapStitching:createnewTriangle] the points of"
		        " these triangles should never be from the same patch." << endl;
		//this method is usually called during stitching so we don't expect this to be called for trianlges of this source
		assert(0);
		TriangleReference tr;
		tr.index = triangle.points[0].getPatch()->triangles.size();
		tr.container = triangle.points[0].getPatch();
		triangle.points[0].getPatch()->triangles.push_back(triangle);
		Triangle::registerTriangle(tr,true);
		triangle.points[0].getPatch()->cpu_tex_patch_ahead=true;
		return tr;
	}

	bool double_stitch = false;
	bool triple_stitch = false;
	if(pr1.getPatch() == pr2.getPatch() && pr1.getPatch() != pr3.getPatch() ||//p3 is different
	   pr1.getPatch() != pr2.getPatch() && pr1.getPatch() == pr3.getPatch() ||//p2 is different
	   pr1.getPatch() != pr2.getPatch() && pr2.getPatch() == pr3.getPatch()) {
		double_stitch = true;

	} else {
		//TODO: test if it is a triple stitch
		if(pr1.getPatch() != pr2.getPatch() &&
		   pr1.getPatch() != pr3.getPatch() &&
		   pr2.getPatch() != pr3.getPatch()) {
			triple_stitch = true;

		} else {
			//in case it is neither a dual nor a triple stitch don't do anything
			assert(0);
		}
	}

	//now register the new triangle to its neighbours (even tough its done further up?)

	if(double_stitch) {//p1 is different
		//find a fitting double stitch or create one
		TriangleReference tr;
		MeshPatch *patch2 = nullptr;
		if(pr1.getPatch() != pr2.getPatch()) {
			patch2 = pr2.getPatch();
		} else if(pr1.getPatch() != pr3.getPatch()) {
			patch2 = pr3.getPatch();
		}

		shared_ptr<DoubleStitch> stitch = pr1.getPatch()->getDoubleStitchWith(patch2);
		//add triangle to the according stitch
		if(!stitch) {
			stitch = make_shared<DoubleStitch>();
			stitch->weak_self = stitch;
			stitch->patches[0] = pr1.getPatch()->weak_self;
			stitch->patches[1] = patch2->weak_self;

			pr1.getPatch()->addStitchReference(stitch);
			patch2->addStitchReference(stitch);
			debug_new_stitches.push_back(stitch);

		} else {
			if(stitch->patches[0].lock().get() != triangle.points[0].getPatch()) {
				cout << "JUST DON'T LET THIS HAPPEN TOO OFTEN" << endl;
				//assert(0);// the triangle is referring to the same main patch as the stitch

				if(stitch->patches[0].lock().get() == triangle.points[1].getPatch()) { 
					triangle.cycle(1);
				} else if(stitch->patches[0].lock().get() == triangle.points[2].getPatch()) {
					triangle.cycle(2);
				}
				//TODO: rotate either once or twice but better get rid of this before
			}
		}

		stitch->debug_to_delete3 = 1;
		tr.index = stitch->triangles.size();
		tr.container = stitch.get();
		stitch->triangles.push_back(triangle);
		Triangle::registerTriangle(tr, true);
		stitch->cpu_triangles_ahead = true;

		return tr;
	}

	if(triple_stitch) {
		TriangleReference tr;
		//all 3 points are from a different point
		//find a fitting triple stitch
		shared_ptr<TripleStitch> stitch =
				pr1.getPatch()->getTripleStitchWith(pr2.getPatch(), pr3.getPatch());
		if(!stitch) {
			stitch = make_shared<TripleStitch>();
			stitch->weak_self = stitch;
			stitch->patches[0] = pr1.getPatch()->weak_self;
			stitch->patches[1] = pr2.getPatch()->weak_self;
			stitch->patches[2] = pr3.getPatch()->weak_self;

			pr1.getPatch()->addStitchReference(stitch);
			pr2.getPatch()->addStitchReference(stitch);
			pr3.getPatch()->addStitchReference(stitch);
			debug_new_stitches.push_back(stitch);

		} else {
			MeshPatch *main_patch = stitch->patches[0].lock().get();
			if(main_patch != triangle.points[0].getPatch()) {
				//TODO: rotate the triangle points so it fits again:
				if(main_patch == triangle.points[1].getPatch()) {
					triangle.cycle(1);
				} else if(main_patch == triangle.points[2].getPatch()) {
					triangle.cycle(2);
				}
				//assert(0);// the triangle is *not* referring to the same main patch as the stitch
			}
			if(main_patch != triangle.points[0].getPatch()) {
				assert(0);// the triangle is *not* referring to the same main patch as the stitch
			}
		}

		//add triangle to the according stitch
		tr.index = stitch->triangles.size();
		tr.container = stitch.get();
		stitch->triangles.push_back(triangle);
		Triangle::registerTriangle(tr, true);
		stitch->cpu_triangles_ahead = true;

		return tr;
	}
}

float cross(Vector2f v1, Vector2f v2) {
	return v1[0] * v2[1] - v1[1] * v2[0];
}

bool isOnRightSideOfLine(Vector2f point_in_question, Vector2f p1, Vector2f p2,
                         Vector2f point_on_wrong_side) {
	assert(0);
	//we compare this with the cross product
	Vector2f v1 = p2 - p1;
	Vector2f vw = point_on_wrong_side - p1;
	Vector2f vq = point_in_question - p1;
	float cross_wrong = cross(v1, vw);
	float cross_in_question=cross(v1, vq);
	bool same_as_wrong = signbit(cross_wrong) == signbit(cross_in_question);
	return !same_as_wrong;
}

cv::Mat MeshReconstruction::generateDepthFromView(int width, int height, 
                                                  Matrix4f pose) {

	//because i don't know where else to put it: add a texture here:
	//gfx::GpuTex2D texture3(GL_RGBA,GL_RGBA,GL_UNSIGNED_BYTE,640,480,false,test.data);
	/*Mat rgba;
	cv::cvtColor(m_rgb,rgba,CV_BGR2RGBA);
	//gfx::GpuTex2D texture2(G_RGBA,GL_RGBA,GL_UNSIGNED_BYTE,rgba.cols,rgba.rows,false,rgba.data);
	shared_ptr<gfx::GpuTex2D> texture = make_shared<gfx::GpuTex2D>(GL_RGBA,GL_RGBA,GL_UNSIGNED_BYTE,rgba.cols,rgba.rows,
																			 true,rgba.data);
   */
	//The first step would be to do ICP, from this we would already get the projected points.
	//but right now we do not have anyof these steps
	//, therefore we project the points into space. on the gpu


	//2. Step should be the incorporation of new sensor data into the already existing map.
	float fx = params.depth_fxycxy[0];
	float fy = params.depth_fxycxy[1];
	float cx = params.depth_fxycxy[2];
	float cy = params.depth_fxycxy[3];

	Matrix4f proj1;//one to do what has to be done anyway
	proj1 << fx,  0, cx, 0,
	          0, fy, cy, 0,
	          0,  0,  0 -1,
	          0,  0,  1, 0;

	float w = params.depth_res.width;
	float h = params.depth_res.height;
	float zmin = 0.1f;
	float zmax = 30.0f;
	float b = 2.0f / (1.0f / zmin - 1.0f / zmax);
	float a = b / zmax + 1.0f;
	Matrix4f proj2;//the other one to scale everything to the normalized coordinates
	proj2 << 2.0f / w,        0, 0,  -1.0f + 1.0f / w,
	                0, 2.0f / h, 0,  -1.0f + 1.0f / h,
	                0,        0, b,                 a,
	                0,        0, 0,                 1;
	Matrix4f proj = proj2 * proj1;

	active_set_update_mutex.lock();
	shared_ptr<ActiveSet> active_set = active_set_update;
	active_set_update_mutex.unlock();
	//TODO: maybe update the active set
	information_renderer.renderDepth(active_set.get(), proj, pose);
	cv::Mat ex_geom(height, width, CV_32FC4);
	information_renderer.getDepthTexture()->downloadData(ex_geom.data);
	cv::Mat int_depth(height,width,CV_16UC1);//store the geometry back to a depthmap
	for(int i = 0; i < height * width; i++) {
		int_depth.at<unsigned short>(i) = ex_geom.at<Vector4f>(i)[2] * 1000.0f;
	}
	return int_depth;
}

//let this stay within the mesh
vector<cv::Rect2f> MeshReconstruction::genBoundsFromPatches(
		vector<shared_ptr<MeshPatch> > &patches, Matrix4f pose, Matrix4f proj,
		shared_ptr<ActiveSet> active_set) {

	Matrix4f mvp = proj * pose.inverse();

	vector<TexCoordGen::BoundTask> bound_tasks;
	vector<shared_ptr<MeshPatch>> valid_mesh_patches;
	int valid_count = 0;
	for(shared_ptr<MeshPatch> patch : patches) {
		shared_ptr<MeshPatchGpuHandle> gpu_patch = patch->gpu.lock();

		//check if patch is on gpu. Which it should be!
		if(gpu_patch == nullptr) {
			assert(0);
			continue;
		}
		bool valid = true;
		TexCoordGen::BoundTask task;

		task.target_ind = valid_count;

		//now iterate all stitches for neighbours
		for(shared_ptr<DoubleStitch> stitch : patch->double_stitches) {
			if(stitch->patches[0].lock() != patch) {
				continue;
			}
			if(!stitch->isPartOfActiveSet(active_set.get())) {
				stitch->isPartOfActiveSet(active_set.get());
				#ifndef IGNORE_SERIOUS_BUG_4
				assert(0);//This actually should not happen
				//no it acutally can happen. it also should mean that the
				//according rect would be invalid
				#endif
				valid = false;
				break;
			}
			shared_ptr<TriangleBufConnector> gpu_stitch = stitch->triangles_gpu.lock();
			if(gpu_stitch == nullptr) {
				assert(0);
				valid = false;
				break;
			}
			task.triangle_count = gpu_stitch->getSize();
			task.triangles = gpu_stitch->getStartingPtr();
			task.debug_type = 1;
			bound_tasks.push_back(task);

		}
		if(!valid) {
			continue;
		}
		for(shared_ptr<TripleStitch> stitch : patch->triple_stitches) {
			if(stitch->patches[0].lock() != patch) {
				continue;
			}
			if(!stitch->isPartOfActiveSet(active_set.get())) {
				#ifndef IGNORE_SERIOUS_BUG_4
				assert(0); // actually this can happen and it means we should
				//not update the geometry here.
				#endif
				valid = false;
				break;
			}
			shared_ptr<TriangleBufConnector> gpu_stitch = stitch->triangles_gpu.lock();
			if(gpu_stitch == nullptr){
				assert(0);
				valid = false;
				break;
				continue;
			}
			task.triangle_count = gpu_stitch->getSize();
			task.triangles = gpu_stitch->getStartingPtr();
			task.debug_type = 2;

			bound_tasks.push_back(task);
		}
		if(!valid) {
			continue;
		}

		task.triangle_count = gpu_patch->triangles->getSize();
		task.triangles = gpu_patch->triangles->getStartingPtr();
		task.debug_type = 0;
		if(gpu_patch->triangles->getSize() != 0) {
			bound_tasks.push_back(task);
		} else {
			//if the patch itself does not have triangles we do not submit this task
		}
		valid_mesh_patches.push_back(patch);
		valid_count++;
	}

	const vector<cv::Rect2f> &vector_tmp = 
			TexCoordGen::getPotentialTexCoordBounds(bound_tasks, mvp, 
			                                        valid_mesh_patches.size(),
			                                        gpu_geom_storage_.patch_info_buffer->getCudaPtr(),
			                                        gpu_geom_storage_.vertex_buffer->getCudaPtr());
	vector<cv::Rect2f> bounds = vector_tmp;
	return bounds;
}

void MeshReconstruction::clearInvalidGeometry(shared_ptr<ActiveSet> set, 
                                              cv::Mat depth, 
                                              Matrix4f depth_pose) {
	if(set == nullptr) {
		return;
	}

	int width = depth.cols;
	int height = depth.rows;

	cv::Mat depthf;
	//TODO: maybe we want to have a different
	depth.convertTo(depthf, CV_32FC1, 1.0f / 5000.0f);

	shared_ptr<gfx::GpuTex2D> depth_tex = 
			make_shared<gfx::GpuTex2D>(garbage_collector_, GL_R32F, GL_RED, GL_FLOAT,
			                           width, height, true, (void*) depthf.data);

	shared_ptr<gfx::GpuTex2D> d_std_max_std_map = 
			make_shared<gfx::GpuTex2D>(garbage_collector_, GL_RGBA32F, GL_RGBA,
			                           GL_FLOAT, width, height, true, (void*) 0);

	//TODO: don't do this every time we need it but only once!!!
	generateXtionConfidenceImage(depth_tex->getCudaSurfaceObject(),
	                             d_std_max_std_map->getCudaSurfaceObject(),
	                             width, height);

	float fx = params.depth_fxycxy[0];
	float fy = params.depth_fxycxy[1];
	float cx = params.depth_fxycxy[2];
	float cy = params.depth_fxycxy[3];

	Matrix4f proj;//one to do what has to be done anyway
	proj << fx,  0, cx,  0,
	         0, fy, cy,  0,
	         0,  0,  0, -1,
	         0,  0,  1,  0;
	Matrix4f pose = depth_pose;
	Matrix4f pose_inv = pose.inverse();
	Matrix4f proj_pose = proj * pose_inv;

	vector<shared_ptr<MeshPatch>> patches = set->retained_mesh_patches_cpu;

	vector<gpu::GeometryValidityChecks::VertexTask> tasks;
	for(shared_ptr<MeshPatch> patch : patches) {
		shared_ptr<MeshPatchGpuHandle> gpu_patch = patch->gpu.lock();
		if(gpu_patch == nullptr) {
			continue;
		}
		gpu::GeometryValidityChecks::VertexTask task;

		task.start_source = gpu_patch->vertices_source->getStartingIndex();
		task.start_dest = gpu_patch->vertices_dest->getStartingIndex();
		task.size = gpu_patch->vertices_source->getSize();
		tasks.push_back(task);
	}
	gpu::GeometryValidityChecks::checkVertexValidity(
			d_std_max_std_map->getCudaSurfaceObject(), width, height, pose_inv, 
			proj_pose, tasks, gpu_geom_storage_.vertex_buffer->getCudaPtr());//vertices on gpu
}

shared_ptr<ActiveSet> MeshReconstruction::genActiveSetFromPose(
		Matrix4f depth_pose) {

	vector<shared_ptr<MeshPatch>> visible_shared_patches = 
			octree_.getObjects(depth_pose, params.depth_fxycxy, 
			                   Vector2f(params.depth_res.width, 
			                            params.depth_res.height), 
			                   getMaxDistance(),//6.0f,
			                   0.0f);//don't dilate the frustum in this case

	for(shared_ptr<MeshPatch> patch : visible_shared_patches) {
		//TODO: test patch
		if(!patch->isPartOfActiveSetWithNeighbours(active_set_update.get())) {
			//continue;
			//assert(0);//all of the new ones should be loaded
		}
		if(patch->gpu.lock() == nullptr) {
			continue;
			//assert(0);
		}
		if(patch->geom_tex_patch->gpu.lock() == nullptr) {
			//assert(0);//whyyyyy. the geometry textures should be secured at
			//this point
			cv::waitKey(1);
		}
	}

	set<shared_ptr<MeshPatch>> patches_including_neighbours;
	for(shared_ptr<MeshPatch> patch : visible_shared_patches) {
		patches_including_neighbours.insert(patch);
		for(shared_ptr<DoubleStitch> stitch : patch->double_stitches) {
			if (stitch->patches[0].lock() != patch) {
				//continue;
			}
			shared_ptr<MeshPatch> neighbour = stitch->patches[1].lock();
			if(neighbour->isInsertedInOctree()) {
				patches_including_neighbours.insert(neighbour);
			}
		}
		for(shared_ptr<TripleStitch> stitch : patch->triple_stitches) {
			if(stitch->patches[0].lock() != patch) {
				// continue;
			}
			for(size_t i = 1; i < 3; i++) {
				shared_ptr<MeshPatch> neighbour = stitch->patches[i].lock();
				if(neighbour->isInsertedInOctree()) {
					patches_including_neighbours.insert(neighbour);
				}
			}
		}
	}

	vector<shared_ptr<MeshPatch>> patches_with_neighbours(
			patches_including_neighbours.begin(),
			patches_including_neighbours.end());

	shared_ptr<ActiveSet> new_active_set = 
			gpu_geom_storage_.makeActiveSet(patches_with_neighbours, this, true);

	//if it fails down here even though it doesn't fail up there....
	//you are doing something wrong
	for(shared_ptr<MeshPatch> patch : visible_shared_patches) {
		//TODO: test patch
		if(!patch->isPartOfActiveSetWithNeighbours(new_active_set.get())) {
			continue;
			//assert(0);//all of the new ones should be loaded
		}
		if(patch->gpu.lock() == nullptr) {
			assert(0);
		}
		if(patch->geom_tex_patch->gpu.lock() == nullptr) {
			continue;
			setActiveSetUpdate_(new_active_set);
			cv::waitKey();
			//assert(0);//whyyyyy. the geometry textures should be secured at
			//this point
		}
	}
	//TODO: maybe do some uploads:

	new_active_set->checkForCompleteGeometry();

	return new_active_set;
}

void MeshReconstruction::initInGLRenderingContext() {
	gfx::GLUtils::checkForOpenGLError("[ScaleableMap::initInGLRenderingContex] before");

	//to think of: We could ignore any synchronization and begin rendering only if the buffers are created
	//
	while(!gl_logic_initialized_) {
		unique_lock<mutex> lk(condition_variable_mutex_);
		condition_variable_.wait(lk);
	}



	render_presentation.initInContext(640, 480, this);

	gpu_pre_seg_ = make_shared<GpuNormSeg>(garbage_collector_, 
	                                       params.depth_res.width, 
	                                       params.depth_res.height);

	gfx::GLUtils::checkForOpenGLError(
			"[ScaleableMap::initInGLRenderingContex] Initialization of presentation object.");

	low_detail_renderer.initInGlContext();

	gfx::GLUtils::checkForOpenGLError("[ScaleableMap::initInGLRenderingContex] end");
}