#include "mesh_reconstruction.h"

#include <iostream>

#include <gpu/gpu_tex.h>
#include <cuda/xtion_camera_model.h>
#include <cuda/tex_coords.h>
#include <cuda/geom_update.h>
#include <gpu/active_set.h>
#include <utils/gpu_norm_seg.h>
#include <worker.h>

using namespace std;
using namespace Eigen;

bool MeshReconstruction::removePatch(shared_ptr<Meshlet> patch) {
	cout << "THIS IS IMPORTANT" << endl;

	//remove patch from
	for(auto neighbour_weak : patch->neighbours){
		auto neighbour = neighbour_weak.lock();
		neighbour->removeNeighbour(patch.get());
	}

	patch->neighbours.clear();
	meshlet_mutex_.lock();
	for(auto it = meshlets_.begin(); it != meshlets_.end(); ++it) {
		if(it->second == patch) {
			meshlets_.erase(it);
			meshlet_mutex_.unlock();
			return true;
		}
	}
	meshlet_mutex_.unlock();
	return false;
}

void MeshReconstruction::checkNeighbourhoodConsistency() {

	int k=0;
	for(auto p : meshlets_){
		auto patch = p.second;
		int l = 0;
		for(auto &tri : patch->triangles){
		    assert(tri.orientation_valid());
			for(int i : { 0, 1, 2}){
				Triangle* tri1 = &tri;
				if(tri.neighbours[i].ptr == nullptr)
					continue;
				Triangle* tri2 = tri.neighbours[i].ptr;
				Triangle* tri2_1 = tri.neighbours[i].ptr->neighbours[tri.neighbours[i].pos].ptr;
				assert(tri.neighbours[i].ptr->neighbours[tri.neighbours[i].pos].ptr == &tri);
			}
			l++;
		}
		k++;
	}
}
void MeshReconstruction::checkTriangleVerticesConsistency(){

	int k=0;
	for(auto p : meshlets_){
		auto patch = p.second;
		for(int i = 0;i<patch->vertices.size();i++){
			Vertex &vert = patch->vertices[i];
			assert(vert.manifold_valid());
			//check if the vertex is referencing to the correct meshlet
			assert(vert.meshlet == patch.get());
			for(int j = 0;j<vert.triangles.size();j++){
				assert( &vert ==
						vert.triangles[j].triangle->vertices[vert.triangles[j].ind_in_triangle]);
			}
		}

		//testing if one of the triangles is referencing a vertex thats not part of this or one of the neighbouring
		//meshlets
		for(int i = 0;i<patch->triangles.size();i++){
			Triangle &tri = patch->triangles[i];
			for(int j : {0,1,2}){
				Vertex *vert = tri.vertices[j];
				if(vert->meshlet != patch.get()){
					if(!patch->isNeighbourWith(vert->meshlet)){
						assert(0);
					};
				}
			}
		}

		k++;
	}
}
void MeshReconstruction::checkLeftoverEdges() {

	for(auto p : meshlets_){
		for(auto &tri : p.second->triangles){
			for(int i : {0, 1, 2}){
				if(tri.edges[i] != nullptr){
					assert(0);
				}
			}
		}
	}
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


MeshReconstruction::MeshReconstruction(int depth_width, int depth_height,
                                       int rgb_width, int rgb_height) {

	params.depth_res.width = depth_width;
	params.depth_res.height = depth_height;
	params.rgb_res.width = rgb_width;
	params.rgb_res.height = rgb_height;
	//texturing.mesh_reconstruction = this;

	//TODO: name these setup functions consistently
	//geometry_update.setup(this);
}


//we assume that the opengl context still exists when running the destructor
// (otherwise the opengl stuff would not behave nicely)
MeshReconstruction::~MeshReconstruction() {
	// first delete all the triangles before we delete the vertices
	// Deleting triangles and abandoning the vertices is a better idea generally
	// than deleting vertices and not caring for triangles referencing to them.
	// (there is a check in place to prevent that anyways)
	for(std::pair<int, shared_ptr<Meshlet>> id_meshlet : meshlets_){
		id_meshlet.second->triangles.clear();
	}
}

shared_ptr<Meshlet> MeshReconstruction::genMeshlet() {
	meshlet_mutex_.lock();
	current_max_patch_id_ ++;
	auto meshlet = make_shared<Meshlet>(current_max_patch_id_,&octree_);
	meshlet->weak_self = meshlet;
	meshlets_[current_max_patch_id_] = meshlet;
	meshlet_mutex_.unlock();
	return meshlet;
}

shared_ptr<Meshlet> MeshReconstruction::getMeshlet(int id){
	meshlet_mutex_.lock();
	auto result = meshlets_[id];
	meshlet_mutex_.unlock();
	return result;
}

shared_ptr<MeshTexture> MeshReconstruction::genMeshTexture(
		MeshTexture::Type content_type) {
	assert(0);
	return nullptr;
	//return make_shared<MeshTexture>(content_type, this);
}

void MeshReconstruction::setRGBIntrinsics(Vector4f fxycxy) {
	params.rgb_fxycxy = fxycxy;
}

void MeshReconstruction::setDepthIntrinsics(Vector4f fxycxy) {
	params.depth_fxycxy = fxycxy;
}

bool MeshReconstruction::hasGeometry() {
	return meshlets_.size() > 0;
}

//TODO: get rid of this method
/*
void MeshReconstruction::erase() {
	//The scheduler should be closed at this point
	cout << "erase... this function does not do what you expect it to do" << endl;
	vector<weak_ptr<Meshlet>> patches;//collect weak pointers to that stuff to see what of this is still existent
	vector<shared_ptr<Meshlet>> shared_patches;//collect weak pointers to that stuff to see what of this is still existent

	int debug = 0 ;
	for(auto patch : meshlets_) {
		debug++;
		patches.push_back(patch.second);
		shared_patches.push_back(patch.second);
	}

	meshlets_.clear();

	for(int i = 0; i < debug; i++) {
		shared_patches.pop_back();
	}

	//DEBUG:
	//all of this should be zero: TODO FIND OUT WHY, parse through all the texatlas thingis
	//TODO: follow down the octree ( objects don't seem to be deleted accordingly)
	//and then find who is holding references to the gpu structures


	for(int i = 0; i < patches.size(); i++) {
		assert(patches[i].expired());
	}
	vector<gfx::GpuTex2D*> textures = gfx::GpuTex2D::getTexList();

	//TODO: mechanism to delete all fbos that are used in thread
	glFinish();
}
 */

vector<shared_ptr<Meshlet>> MeshReconstruction::GetAllPatches() {
	vector<shared_ptr<Meshlet>> patches;
	for(auto patch : meshlets_) {
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
/*
Triangle* MeshReconstruction::addTriangle_(Vertex* pr1,
										   Vertex* pr2,
										   Vertex* pr3) {
	//same method as below but without the debug
	assert(0);
}
*/
//Creating a new triangle just from vertex references.
//used in the inter frame stitching process
int debug_global_stitch_triangle_ctr = 0;
//TripleStitch *debug_quenstionable_triplestitch = nullptr;

Triangle* MeshReconstruction::addTriangle_(
		Vertex* v1, Vertex* v2, Vertex* v3, int debug_marker) {
    //TODO: remove this debug measures as soon as all failure cases are caught
    bool debug1 = !v1->manifold_valid();
    bool debug2 = !v2->manifold_valid();
    bool debug3 = !v3->manifold_valid();
    if(debug1 || debug2 || debug3){
        assert(0);
    }

    bool debug_encompassed_1 = v1->encompassed();
    bool debug_encompassed_2 = v2->encompassed();
    bool debug_encompassed_3 = v3->encompassed();

    //TODO: what would be nice would be something similar/using the same as in the mesher class:
	//providing points and neighbour references(triangle ptr + index)

	v1->meshlet->triangles.emplace_back();
	Triangle &triangle = v1->meshlet->triangles.back();
	triangle.vertices[0] = v1;
	triangle.vertices[1] = v2;
	triangle.vertices[2] = v3;
	triangle.debug_is_stitch = true;
	triangle.debug_nr = debug_global_stitch_triangle_ctr;
	triangle.debug_marker = debug_marker;
	debug_global_stitch_triangle_ctr++;

	auto getNeighbour = [](Vertex* v1,Vertex* v2) -> Triangle::Neighbour{
		Triangle::Neighbour nb;
		for(auto tri : v1->triangles){
			// check the edges before or after:
			//TODO: make assumptions about clockwise /ccw to reduce cost.
			//after
			int ind = tri.ind_in_triangle + 1;
			if(ind == 3)
				ind = 0;
			if(tri.triangle->vertices[ind] == v2){
				nb.ptr = tri.triangle;
				nb.pos = tri.ind_in_triangle;
				return nb;
			}
			//check the edges before
			ind = tri.ind_in_triangle - 1;
			if(ind == -1)
				ind = 2;
			if(tri.triangle->vertices[ind] == v2){
				nb.ptr = tri.triangle;
				nb.pos = ind;
				return nb;
			}
		}
		// return invalid neighbour if nothing is found
		return nb;
	};
	//this seems expensive
	if(v1->meshlet != v2->meshlet){
		if(!v1->meshlet->isNeighbourWith(v2->meshlet)){
			shared_ptr<Meshlet> sp = std::static_pointer_cast<Meshlet>(v2->meshlet->shared_from_this());
			v1->meshlet->addNeighbour(sp);
			v2->meshlet->addNeighbour(std::static_pointer_cast<Meshlet>(v1->meshlet->shared_from_this()));
		}
	}
	if(v1->meshlet != v3->meshlet){
		if(!v1->meshlet->isNeighbourWith(v3->meshlet)){
			v1->meshlet->addNeighbour(std::static_pointer_cast<Meshlet>(v3->meshlet->shared_from_this()));
			v3->meshlet->addNeighbour(std::static_pointer_cast<Meshlet>(v1->meshlet->shared_from_this()));
		}
	}
	if(v2->meshlet != v3->meshlet){
		if(!v2->meshlet->isNeighbourWith(v3->meshlet)){
			v2->meshlet->addNeighbour(std::static_pointer_cast<Meshlet>(v3->meshlet->shared_from_this()));
			v3->meshlet->addNeighbour(std::static_pointer_cast<Meshlet>(v2->meshlet->shared_from_this()));
		}
	}
	triangle.neighbours[0] = getNeighbour(v1, v2);
	triangle.neighbours[1] = getNeighbour(v2, v3);
	triangle.neighbours[2] = getNeighbour(v3, v1);
	triangle.registerSelf();

	//TODO: remove this debug measures as soon as all failure cases are caught
	//check if the manifold conditions are violated after creating the triangle
	bool debug0 = !triangle.manifold_valid();

    debug1 = !v1->manifold_valid();
    debug2 = !v2->manifold_valid();
    debug3 = !v3->manifold_valid();
    if(debug1 || debug2 || debug3 || debug0){
        //assert(0);
    }
	return &triangle;
/*

	if(v1->meshlet == v2->meshlet && v1->meshlet == v3->meshlet) {
		cout << "ERROR [scaleableMapStitching:createnewTriangle] the points of"
		        " these triangles should never be from the same patch." << endl;
		//this method is usually called during stitching so we don't expect this to be called for trianlges of this source
		assert(0);
		Triangle* tr;
		return tr;
	}

	bool double_stitch = false;
	bool triple_stitch = false;
	if(v1.getPatch() == v2.getPatch() && v1.getPatch() != v3.getPatch() ||//p3 is different
	   v1.getPatch() != v2.getPatch() && v1.getPatch() == v3.getPatch() ||//p2 is different
	   v1.getPatch() != v2.getPatch() && v2.getPatch() == v3.getPatch()) {
		double_stitch = true;

	} else {
		//TODO: test if it is a triple stitch
		if(v1.getPatch() != v2.getPatch() &&
		   v1.getPatch() != v3.getPatch() &&
		   v2.getPatch() != v3.getPatch()) {
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
		Meshlet *patch2 = nullptr;
		if(v1.getPatch() != v2.getPatch()) {
			patch2 = v2.getPatch();
		} else if(v1.getPatch() != v3.getPatch()) {
			patch2 = v3.getPatch();
		}

		shared_ptr<DoubleStitch> stitch = v1.getPatch()->getDoubleStitchWith(patch2);
		//add triangle to the according stitch
		if(!stitch) {
			stitch = make_shared<DoubleStitch>();
			stitch->weak_self = stitch;
			stitch->patches[0] = v1.getPatch()->weak_self;
			stitch->patches[1] = patch2->weak_self;

			v1.getPatch()->addStitchReference(stitch);
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
				v1.getPatch()->getTripleStitchWith(v2.getPatch(), v3.getPatch());
		if(!stitch) {
			stitch = make_shared<TripleStitch>();
			stitch->weak_self = stitch;
			stitch->patches[0] = v1.getPatch()->weak_self;
			stitch->patches[1] = v2.getPatch()->weak_self;
			stitch->patches[2] = v3.getPatch()->weak_self;

			v1.getPatch()->addStitchReference(stitch);
			v2.getPatch()->addStitchReference(stitch);
			v3.getPatch()->addStitchReference(stitch);
			debug_new_stitches.push_back(stitch);

		} else {
			Meshlet *main_patch = stitch->patches[0].lock().get();
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
	*/
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
												  InformationRenderer *information_renderer,
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

	/*
	//2. Step should be the incorporation of new sensor data into the already existing map.
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
	information_renderer->renderDepth(active_set.get(), proj, pose);
	cv::Mat ex_geom(height, width, CV_32FC4);
	information_renderer->getDepthTexture()->downloadData(ex_geom.data);
	cv::Mat int_depth(height,width,CV_16UC1);//store the geometry back to a depthmap
	for(int i = 0; i < height * width; i++) {
		int_depth.at<unsigned short>(i) = ex_geom.at<Vector4f>(i)[2] * 1000.0f;
	}
	return int_depth;
	 */
}

//let this stay within the mesh
vector<cv::Rect2f> MeshReconstruction::genBoundsFromPatches(
		vector<shared_ptr<Meshlet> > &patches, Matrix4f pose, Matrix4f proj,
		shared_ptr<ActiveSet> active_set) {

	assert(0);//this method should be in the mesher class
	/*

	Matrix4f mvp = proj * pose.inverse();

	vector<TexCoordGen::BoundTask> bound_tasks;
	vector<shared_ptr<Meshlet>> valid_mesh_patches;
	int valid_count = 0;
	for(shared_ptr<Meshlet> patch : patches) {
		shared_ptr<MeshletGpuHandle> gpu_patch = patch->gpu.lock();

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
			if(gpu_stitch == nullptr) {
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
	 */
}

void MeshReconstruction::clearInvalidGeometry(shared_ptr<ActiveSet> set, 
                                              cv::Mat depth, 
                                              Matrix4f depth_pose) {
	//assert(0);// this method should not be necessary at all TODO: remove
	/*
	if(set == nullptr) {
		return;
	}

	int width  = depth.cols;
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

	vector<shared_ptr<Meshlet>> patches = set->retained_mesh_patches_cpu;

	vector<gpu::GeometryValidityChecks::VertexTask> tasks;
	for(shared_ptr<Meshlet> patch : patches) {
		shared_ptr<MeshletGpuHandle> gpu_patch = patch->gpu.lock();
		if(gpu_patch == nullptr) {
			continue;
		}
		gpu::GeometryValidityChecks::VertexTask task;

		task.start_source = gpu_patch->vertices_source->getStartingIndex();
		task.start_dest   = gpu_patch->vertices_dest->getStartingIndex();
		task.size         = gpu_patch->vertices_source->getSize();
		tasks.push_back(task);
	}
	gpu::GeometryValidityChecks::checkVertexValidity(
			d_std_max_std_map->getCudaSurfaceObject(), width, height, pose_inv, 
			proj_pose, tasks, gpu_geom_storage_.vertex_buffer->getCudaPtr());//vertices on gpu
			*/
}

vector<shared_ptr<Meshlet>> MeshReconstruction::getVisibleMeshlets(Matrix4f pose, Vector4f intrinsics,cv::Size2i res,float max_dist) {

	/*
	octree_.getVisibleObjects<Meshlet>()
	vector<shared_ptr<Meshlet>> visible_meshlets =
			octree_.getObjects(pose, intrinsics,
							   Vector2f(res.width,res.height),
							   max_dist,
							   0.0f);//don't dilate the frustum in this case
							   */
	//assert(0);
	video::Intrinsics intrinsics1;
	intrinsics1.fx = intrinsics[0];
	intrinsics1.fy = intrinsics[1];
	intrinsics1.cx = intrinsics[2];
	intrinsics1.cy = intrinsics[3];
	octree::Frustum frustum(pose,
			intrinsics1,
			Vector2f(res.width,res.height),
			max_dist);

	vector<shared_ptr<Meshlet>> visible_meshlets;

	octree_.getVisibleObjects<Meshlet>(&frustum,&visible_meshlets);
    return visible_meshlets;
}

/*
shared_ptr<ActiveSet> MeshReconstruction::genActiveSetFromPose(
		Matrix4f depth_pose,
		LowDetailRenderer *low_detail_renderer,
		TextureUpdater *texture_updater,
		InformationRenderer* information_renderer
		) {
	assert(0); // this should definitely not be in this class

	vector<shared_ptr<Meshlet>> visible_shared_patches =
			octree_.getObjects(depth_pose, params.depth_fxycxy, 
			                   Vector2f(params.depth_res.width, 
			                            params.depth_res.height), 
			                   getMaxDistance(),
			                   0.0f);//don't dilate the frustum in this case

	for(shared_ptr<Meshlet> patch : visible_shared_patches) {
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

	set<shared_ptr<Meshlet>> patches_including_neighbours;
	for(shared_ptr<Meshlet> patch : visible_shared_patches) {
		patches_including_neighbours.insert(patch);
		for(shared_ptr<DoubleStitch> stitch : patch->double_stitches) {
			if (stitch->patches[0].lock() != patch) {
				//continue;
			}
			shared_ptr<Meshlet> neighbour = stitch->patches[1].lock();
			if(neighbour->isInsertedInOctree()) {
				patches_including_neighbours.insert(neighbour);
			}
		}
		for(shared_ptr<TripleStitch> stitch : patch->triple_stitches) {
			if(stitch->patches[0].lock() != patch) {
				// continue;
			}
			for(size_t i = 1; i < 3; i++) {
				shared_ptr<Meshlet> neighbour = stitch->patches[i].lock();
				if(neighbour->isInsertedInOctree()) {
					patches_including_neighbours.insert(neighbour);
				}
			}
		}
	}

	vector<shared_ptr<Meshlet>> patches_with_neighbours(
			patches_including_neighbours.begin(),
			patches_including_neighbours.end());

	shared_ptr<ActiveSet> new_active_set = 
			gpu_geom_storage_.makeActiveSet(
					patches_with_neighbours,
					this,
					low_detail_renderer,
					texture_updater,
					information_renderer,
					true);

	//if it fails down here even though it doesn't fail up there....
	//you are doing something wrong
	for(shared_ptr<Meshlet> patch : visible_shared_patches) {
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
 */

/*
void MeshReconstruction::initInGLRenderingContext() {
	assert(0); // don't do this in here!
	gfx::GLUtils::checkForOpenGLError("[ScaleableMap::initInGLRenderingContex] before");

	//to think of: We could ignore any synchronization and begin rendering only if the buffers are created
	//
	while(!gl_logic_initialized_) {
		unique_lock<mutex> lk(condition_variable_mutex_);
		condition_variable_.wait(lk);
	}

	//render_presentation.initInContext(640, 480, this);

	gpu_pre_seg_ = make_shared<GpuNormSeg>(garbage_collector_,
										   params.depth_res.width,
										   params.depth_res.height);

	gfx::GLUtils::checkForOpenGLError(
			"[ScaleableMap::initInGLRenderingContex] Initialization of presentation object.");

	//low_detail_renderer.initInGlContext();

	gfx::GLUtils::checkForOpenGLError("[ScaleableMap::initInGLRenderingContex] end");

}
 */