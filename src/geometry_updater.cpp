#include <geometry_updater.h>

#include <mesh_reconstruction.h>
#include <gpu/active_set.h>
#include <gpu/camera.h>
#include <cuda/std_tex_update.h>
#include <utils/gpu_norm_seg.h>
#include <graph/deformation_node.h>
#include <cuda/xtion_camera_model.h>
#include <scheduler.h>

using namespace std;
using namespace Eigen;

shared_ptr<ActiveSet> GeometryUpdater::extend(
		SchedulerBase* scheduler,
		MeshReconstruction* reconstruction,
		InformationRenderer *information_renderer,
		TextureUpdater* texture_updater,
		LowDetailRenderer* low_detail_renderer,
		GpuStorage* gpu_storage,
		shared_ptr<ActiveSet> active_set_of_formerly_visible_patches,
		shared_ptr<gfx::GpuTex2D> d_std_tex, cv::Mat& d_std_mat,
		Matrix4f depth_pose_in, shared_ptr<gfx::GpuTex2D> rgb_tex,
		Matrix4f color_pose_in) {

	//MeshReconstruction *reconstruction = mesh_reconstruction;
	// TODO: add everything from meshReconstructionLogic.cpp here!!!!!
	shared_ptr<ActiveSet> former_active_set = 
		active_set_of_formerly_visible_patches;

	int width  = d_std_mat.cols;
	int height = d_std_mat.rows;

	// Prepare the intrinsic parameters of the depth cam
	float fxd = reconstruction->params.depth_fxycxy[0];
	float fyd = reconstruction->params.depth_fxycxy[1];
	float cxd = reconstruction->params.depth_fxycxy[2];
	float cyd = reconstruction->params.depth_fxycxy[3];

	Matrix4f proj_depth   = Camera::genProjMatrix(reconstruction->params.depth_fxycxy);
	Matrix4f proj_depth_n = Camera::genScaledProjMatrix(reconstruction->params.depth_fxycxy,
														reconstruction->params.depth_res);
	Matrix4f proj_1_color = Camera::genProjMatrix(reconstruction->params.rgb_fxycxy);

	//extract the visible patches from the active set
	vector<shared_ptr<Meshlet>> visible_meshlets;
	if(active_set_of_formerly_visible_patches){
		for(auto bla : active_set_of_formerly_visible_patches->patch_inds){
			shared_ptr<Meshlet> meshlet =
					reconstruction->getMeshlet(bla.first);
			visible_meshlets.push_back(meshlet);
		}
	}


	//this might me more suitable for the beginning but lets do it here:
	information_renderer->renderDepth(
			active_set_of_formerly_visible_patches.get(),
			gpu_storage,
			proj_depth_n, depth_pose_in);
	cv::Mat ex_geom(height, width, CV_32FC4);//existing geometry
	information_renderer->getDepthTexture()->downloadData(ex_geom.data);

	cv::Mat proj_depth_std(height, width, CV_32FC4);

	information_renderer->getStdTexture()->downloadData(proj_depth_std.data);

	//TODO: DEBUG: //check all the points if they have still edges assigned to them:

	/*******************************************STITCHING NEW********/
	//TODO: fill a list of borders with edges...(without having to )
	vector<vector<Edge>> borders;
	Matrix4f proj_pose = proj_depth * depth_pose_in.inverse();

	stitching.genBorderList(
			visible_meshlets, borders,
			proj_pose);
	stitching.reloadBorderGeometry(borders);
	stitching.rasterBorderGeometry(borders, depth_pose_in, proj_depth, ex_geom);

	//this is a premilary measure to get the geometry adding running....
	float geometry_assign_threshold = 0.05f;//every point within 5cm of existing geometry is considered part of that geometry

	/**
	 * TODO: get rid of this code: It is done in the segmentation part anyway.
	 * Altough the code for projecting points should be separate from this.
	 *
	 */
	cv::Mat points(d_std_mat.rows, d_std_mat.cols, CV_32FC4);

	for(int i = 0; i < d_std_mat.rows; i++) {
		for(int j = 0; j < d_std_mat.cols; j++) {
			float z = d_std_mat.at<cv::Vec4f>(i, j)[0];
			points.at<Vector4f>(i, j) = Vector4f(((float) j - cxd) * z / fxd, 
			                                     ((float) i - cyd) * z / fyd, 
			                                     z, 1.0f);
			if(isnan(z) || z == 0.0f) {
				//actually we want this to be done in the segmentation method
				points.at<Vector4f>(i, j) = Vector4f(NAN, NAN, NAN, NAN);
			}

			float ex_z = ex_geom.at<Vector4f>(i, j)[2];
			if(!isnan(ex_z)) {
				//Threshold dependant on the standard deviation
				float thresh = max(proj_depth_std.at<cv::Vec4f>(i, j)[2], //standard deviation. TODO: WRONG DATA FORMAT!!!
				                   d_std_mat.at<cv::Vec4f>(i, j)[2]); // something not working here
				thresh = xtionStdToThresholdSeg(thresh);
				if(z > ex_z - thresh) {
					points.at<Vector4f>(i, j) = Vector4f(NAN, NAN, NAN, NAN);
				}
			}
		}
	}
	cv::imshow("points",points);
	cv::waitKey(1);

	#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
	cv::imshow("novel geometry", points);
	cv::waitKey(); // This is a debug measure.... because everything is shit
	#endif

	cv::Mat mesh_pointers(height, width, CV_32SC2); // Actually we store pointers in this

	gpu_pre_seg_.fxycxy       = reconstruction->params.depth_fxycxy;
	gpu_pre_seg_.max_distance = reconstruction->getMaxDistance();
	// Do a proper segmentation on all the pixel not part of the existing geometry
	gpu_pre_seg_.segment(d_std_tex, proj_depth_std, ex_geom);

	cv::Mat seg   = gpu_pre_seg_.getSegmentation();
	int seg_count = gpu_pre_seg_.getSegCount();

	//*****************************************TOOOOODOOOOOO*********************
	//TODO: put this piece of code into the meshIt part!!!!!
	vector<shared_ptr<Meshlet>> new_shared_mesh_patches;
	//the same as the one above but with shared elements
	for(int i = 0; i < seg_count; i++) {
		shared_ptr<Meshlet> mesh_patch = reconstruction->genMeshlet();
		mesh_patch->vertices_version = 1;
		mesh_patch->triangles_version = 1;
		new_shared_mesh_patches.push_back(mesh_patch);//storing all shared_ptr to the next reconstruction patch
	}
	for(int i = 0; i < seg.rows; i++) {
		for(int j = 0; j < seg.cols; j++) {
			int index = seg.at<int32_t>(i, j);
			if(index != -1) {
				mesh_pointers.at<Meshlet*>(i, j) =
						new_shared_mesh_patches[index].get();
			} else {
				mesh_pointers.at<Meshlet*>(i, j) = nullptr;
			}
		}
	}

	#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
	cv::imshow("segmentation of new input", 
	           generateColorCodedTexture(mesh_pointers));
	cv::waitKey();
	#endif

	// Add vertices to the accroding patches and store indices into a Mat

	// After we have all the necessary points within each of the patches we can use the
	// them to calculate the center points, bounding sphere and principal point.

	auto time_start_all = chrono::system_clock::now();
	//reconstruction that stuffcv:

	cv::Mat vertex_indices(height, width, CV_32SC1);
	vertex_indices.setTo(cv::Scalar(-1)); // TODO: remove this line, should not be necessary

	meshing.meshIt(points, mesh_pointers, vertex_indices, d_std_mat,
				   reconstruction->params.max_depth_step, depth_pose_in);

	for(size_t i = 0; i < new_shared_mesh_patches.size(); i++) {
		//TODO: unify these functions and maybe do this at the very end of everything!!!!!
		new_shared_mesh_patches[i]->updatePrincipalPlaneAndCenter();
		new_shared_mesh_patches[i]->updateSphereRadius();
	}

	auto time_end = chrono::system_clock::now();
	auto time_elapsed = chrono::duration_cast<chrono::milliseconds>(time_end - time_start_all);
	//present
	cout << "[GeometryUpdate::Extend] time consumed by meshing: " 
	     << time_elapsed.count() << "ms" << endl;

	auto time_start = chrono::system_clock::now();

	/*********************************Initial Stitching!! NEW***************************/

	vector<weak_ptr<GeometryBase>> stitch_list;

	//TODO: reinsert code and fix bugs
	//debugCheckTriangleNeighbourConsistency(GetAllPatches());
	stitching.stitchOnBorders(borders, depth_pose_in, proj_depth, proj_depth_std, 
	                          ex_geom, points, d_std_mat, 
	                          reconstruction->generateColorCodedTexture_(mesh_pointers),
	                          mesh_pointers, vertex_indices, stitch_list);

	stitching.freeBorderList(borders);

	/******************************************Initial Stitching!! NEW***********************/

	time_end = chrono::system_clock::now();
	time_elapsed = chrono::duration_cast<chrono::milliseconds>(time_end - time_start);
	//present
	cout << "[GeometryUpdate::Extend] time consumed by stitching: " <<
	        time_elapsed.count() << "ms" << endl;
	#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
	cv::imshow("project old standard deviation texture to new", exColor);
	#endif // SHOW_SERIOUS_DEBUG_OUTPUTS

	time_start = time_end;

	//remove patches with zero triangles. Even when they pose as a source for stitches
	vector<shared_ptr<Meshlet>> list_of_mesh_patches_to_keep;
	vector<shared_ptr<Meshlet>> set_of_patches_to_be_removed;

	for(size_t i = 0; i < new_shared_mesh_patches.size(); i++) {
		shared_ptr<Meshlet> &patch = new_shared_mesh_patches[i];
		if(patch->triangles.empty()) {
			for(int j=0;j<patch->vertices.size();j++){
				Vertex &vertex = patch->vertices[j];
				// check if vertex is connected to triangles...
				// if it is just std::move it to one possible neighbour
				for(int k=0;k<vertex.triangles.size();k++){
					Triangle  *triangle = vertex.triangles[k].triangle;
					Meshlet *target_meshlet = nullptr;
					for(int l : {0,1,2}){
						if(triangle->vertices[l]->meshlet != patch.get()){
							target_meshlet = triangle->vertices[l]->meshlet;
							break;
						}
					}
					//we found a suitable  spot for the vertex. so we move it
					if(target_meshlet){
						target_meshlet->vertices.push_back(move(vertex));
						break;
					}

				}
			}
			patch->vertices.clear();
			set_of_patches_to_be_removed.push_back(patch);
		} else {
			// If the patch has real triangles we keep it
			list_of_mesh_patches_to_keep.push_back(patch);
		}
	}

	new_shared_mesh_patches = list_of_mesh_patches_to_keep;

	for(size_t i = 0; i < set_of_patches_to_be_removed.size(); i++) {
		// Some of the patches don't have any connection or triangle at all
		// and therefore they would not get deleted until now.
		// Any unconnected or deleted but empty (of triangle) patch
		// gets deleted now.
		reconstruction->removePatch(set_of_patches_to_be_removed[i]);
	}
	set_of_patches_to_be_removed.clear();

	time_end = chrono::system_clock::now();
	time_elapsed = chrono::duration_cast<chrono::milliseconds>(time_end - time_start);
	//present
	cout << "[GeometryUpdate::Extend] time consumed by cleaning up the geometry " <<
	        time_elapsed.count() << "ms" << endl;

	time_start = time_end;

	//meshing.genTexIndices(new_shared_mesh_patches);
	meshing.genLocalIndices(new_shared_mesh_patches);

	//since the texture indices are set we can upload and create a new active set
	//most of the code below this active set creation can be put into the active set
	//creation routine

	visible_meshlets.insert(visible_meshlets.end(),
							new_shared_mesh_patches.begin(),
							new_shared_mesh_patches.end());
	shared_ptr<ActiveSet> new_active_set =
			make_shared<ActiveSet>(gpu_storage,
									visible_meshlets,
									scheduler->getActiveSets());
			/*
			gpu_storage->makeActiveSet(
					visible_meshlets,
					reconstruction,
					low_detail_renderer,
					texture_updater,
					information_renderer, true); // most of the members are initial
					*/
	new_active_set->name = "createdInApplyNewData";

	/*************************************TOOOODOOOOO************************/

	time_end = chrono::system_clock::now();
	time_elapsed = chrono::duration_cast<chrono::milliseconds>(time_end - time_start);
	//present
	cout << "[GeometryUpdate::Extend] Time consumed loading the active set: " <<
	        time_elapsed.count() << "ms" << endl;

	time_start = time_end;

	new_active_set->setupHeaders();
	texture_updater->generateGeomTex(reconstruction,
			                        gpu_storage,
			                        new_shared_mesh_patches, depth_pose_in,
	                                proj_depth, d_std_tex, new_active_set,
	                                information_renderer);


	//until here..... not further (not further we implemented stuff)
	//(and with we i mean me)

	time_end = chrono::system_clock::now();
	time_elapsed = chrono::duration_cast<chrono::milliseconds>(time_end - time_start);
	//present
	cout << "[GeometryUpdate::Extend] time consumed by finalizing the geom tex Coordinates of new patches: " << time_elapsed.count() << "ms" << endl;
	
	time_start = time_end;

	// Add and fill new color patches to the surface
	texture_updater->applyColorData2(gpu_storage,
			                        new_shared_mesh_patches,
			                        rgb_tex,
	                               color_pose_in, proj_1_color, new_active_set);

	// After doing the textures and geometry we are supposed to be done with this
	// Active set
	new_active_set->setupHeaders();

	// The apply new color data is supposed to create new texture coordinates as well as creating new textures
	// this and the above function should replace the two steps below

	time_end     = chrono::system_clock::now();
	time_elapsed = chrono::duration_cast<chrono::milliseconds>(time_end - time_start);
	//present
	cout << "[GeometryUpdate::Extend] time consumed by applying new color data: " <<
	        time_elapsed.count() << "ms" << endl;

	time_start = time_end;


	// Now that we have the geometry on the cpu now do the texture for the geometrical textures:

	time_end     = chrono::system_clock::now();
	time_elapsed = chrono::duration_cast<chrono::milliseconds>(time_end - time_start);
	//present
	cout << "[GeometryUpdate::Extend] Time consumed by "
	        "finalizing the new geom Tex patches: " <<
		      time_elapsed.count() << "ms" << endl;

	time_start = time_end;

	// Before putting the reconstruction patches into the octree we want to
	// set the center points and the radii of the patches right
	for(size_t i = 0; i < new_shared_mesh_patches.size(); i++) {
		new_shared_mesh_patches[i]->updateCenterPoint();
		new_shared_mesh_patches[i]->updateSphereRadius();
	}

	//add the objects to the low detail renderer
	auto start_low_detail = chrono::system_clock::now();
	low_detail_renderer->addPatches(new_shared_mesh_patches,
	                                     -depth_pose_in.block<3, 1>(0, 3));
	auto end_low_detail = chrono::system_clock::now();

	cout << "[GeometryUpdate::Extend] Time consumed by generating the low detail update:" <<
	        chrono::duration_cast<chrono::milliseconds>(end_low_detail - start_low_detail).count() << 
	        "ms" << endl;

	//at last we can add the newly created and filed reconstruction patches to the octree

	time_end = chrono::system_clock::now();
	time_elapsed = chrono::duration_cast<chrono::milliseconds>(time_end - time_start);
	//present
	cout << "[GeometryUpdate::Extend] Time consumed by projecting the geometry to the geometry texture:" <<
	        time_elapsed.count() << "ms" << endl;

	reconstruction->octree_.addObjects(new_shared_mesh_patches);


	time_end = chrono::system_clock::now();
	auto elapsed = chrono::duration_cast<chrono::milliseconds>(time_end - time_start_all);
	cout << "[GeometryUpdate::Extend] Time consumed for adding new "
	        "(novel) data took alltogether " <<
	        elapsed.count() << "ms" << endl;
	//warning: this are old values... i don't even know if they apply to
	//on the initial frame it takes 1.2sec
	//on every additional frame its about 300ms. Almost half of it is the
	//creation and upload of the new active set. (which is shit btw.)

	/*****************************UPDATING GRAPH***************************/
	vector<DeformationNode::NodePixPos> nodes(visible_meshlets.size());
	for(size_t i = 0; i < visible_meshlets.size(); i++) {
		shared_ptr<Meshlet> patch = visible_meshlets[i];
		Vector3f pos = patch->getPos();
		Vector4f pos4(pos[0], pos[1], pos[2], 1.0f);
		pos4 = proj_pose * pos4;
		pos4 *= 1.0f / pos4[3];
		nodes[i].node    = patch->deformation_node;
		nodes[i].pix_pos = cv::Vec2f(pos4[0], pos4[1]);
		nodes[i].pos     = pos;
	}
	for(auto &node : nodes) {
		node.node->findNeighbours(node.pos, node.pix_pos, nodes);
	}
	/*********************************************************************************/


	return new_active_set;
}

shared_ptr<ActiveSet> GeometryUpdater::update(
		GpuStorage* gpu_storage,
		vector<shared_ptr<Meshlet>> requested_meshlets,
		shared_ptr<ActiveSet> preexisting_set, // the active set(if existing) containing all requested meshlets
		SchedulerBase* scheduler,//to generate a new active set with
		shared_ptr<gfx::GpuTex2D> d_std_tex,
		Matrix4f depth_pose_in,
		Matrix4f depth_proj) {

	/**
	 * weighting scheme:
	 * o..... offset from triangulated surface
	 * sm.... minimal possible (due to quantisation) standard deviation alike value
	 * s..... standard deviation alike value
	 * s'=s-sm the actual standard deviation we will be working on.
	 * we want s to always be bigger than sm after an update
	 * smk =min(smk-1, sm)
	 * s'k = (s'k-1 * s')/(s'k-1 + s') the new standard deviation
	 *
	 * the update of the old offset is still done on the old wieghts.
	 * ok=(o/s + ok-1/sk-1)/( (s * sk-1)/(s + sk-1))
	 */

	if(requested_meshlets.size() == 0) {
		return nullptr;//or return a empty active set
	}

	int width  = d_std_tex->getWidth();
	int height = d_std_tex->getHeight();
	vector<shared_ptr<ActiveSet>> active_sets = scheduler->getActiveSets();
	if(preexisting_set == nullptr){
		//create a set of pre existing

		preexisting_set = make_shared<ActiveSet>(gpu_storage,requested_meshlets,active_sets);
	}
	vector<shared_ptr<Meshlet>> meshlets_to_update;
	vector<bool> update_mask(requested_meshlets.size(),false);
	for(size_t i=0;i<requested_meshlets.size();i++){
		shared_ptr<Meshlet> &meshlet = requested_meshlets[i];
		bool has_all_neighbours = true;
		for(size_t j = 0; j<meshlet->neighbours.size();j++){
			shared_ptr<Meshlet> nb = meshlet->neighbours[j].lock();
			if(nb == nullptr)
				continue;
			if(preexisting_set->getGpuMeshlet(nb) == nullptr){
				has_all_neighbours = false;
				continue;
			}
		}

		if(!has_all_neighbours)
			continue;

		//setup
		update_mask[i] = true;
		meshlets_to_update.push_back(meshlet);
	}

	shared_ptr<ActiveSet> updated_set =
			make_shared<ActiveSet>(gpu_storage,requested_meshlets,active_sets,update_mask);

	vector<gpu::UpdateDescriptor> update_descriptors(meshlets_to_update.size());
	for(size_t i=0;i<meshlets_to_update.size();i++){
		gpu::UpdateDescriptor &desc = update_descriptors[i];
		shared_ptr<Meshlet> &meshlet = meshlets_to_update[i];
		MeshletGPU* meshlet_gpu_src =
				preexisting_set->getGpuMeshlet(meshlet);
		MeshletGPU* meshlet_gpu_dst=
				updated_set->getGpuMeshlet(meshlet);


		shared_ptr<gfx::GpuTex2D> tex = meshlet_gpu_dst->std_tex.tex->getTex();
		cv::Rect2i rect = meshlet_gpu_dst->std_tex.tex->getRect();
		float width  = tex->getWidth();
		float height = tex->getHeight();
		desc.destination = rect;
		desc.destination_n = cv::Rect2f(float(rect.x) / width,
										float(rect.y) / height,
										float(rect.width) / width,
										float(rect.height) / height);
		desc.destination_size = cv::Size2i(width, height);
		desc.destination_geometry =
				meshlet_gpu_dst->std_tex.tex->getCudaSurfaceObject(); //texture surface object



		tex = meshlet_gpu_src->std_tex.tex->getTex();
		rect = meshlet_gpu_src->std_tex.tex->getRect();
		width  = tex->getWidth();
		height = tex->getHeight();

		desc.source = rect;
		desc.source_n = cv::Rect2f(float(rect.x) / width,
								   float(rect.y) / height,
								   float(rect.width) / width,
								   float(rect.height) / height);
		desc.source_size = cv::Size2i(width, height);
		desc.source = rect;
		desc.source_n = cv::Rect2f(float(rect.x) / width,
								   float(rect.y) / height,
								   float(rect.width) / width,
								   float(rect.height) / height);
		desc.source_size = cv::Size2i(width, height);
		desc.source_geometry =
				meshlet_gpu_src->std_tex.tex->getCudaSurfaceObject();


		desc.destination_references = meshlet_gpu_dst->geom_lookup_tex->getCudaSurfaceObject();
		desc.reference_offset = meshlet_gpu_dst->geom_lookup_tex->getRect().tl();


		desc.src_verts = meshlet_gpu_src->vertices->getStartingPtr();
		desc.dst_verts = meshlet_gpu_dst->vertices->getStartingPtr();
		desc.triangles = meshlet_gpu_src->triangles->getStartingPtr();
		desc.vertex_count = meshlet_gpu_src->vertices->getSize();

		desc.update_texture = true;



		//TODO: setup descriptors
	}

	Vector4f cam_pos = Camera::calcCamPosFromExtrinsic(depth_pose_in);
	Matrix4f proj_pose; //TODO
	gpu::updateGeometry(
			d_std_tex->getCudaSurfaceObject(),
			d_std_tex->getWidth(),d_std_tex->getHeight(),
			update_descriptors,
			cam_pos,
			depth_pose_in,
			depth_proj,
			gpu_storage->vertex_buffer->getCudaPtr(),//TODO. get rid of these buffers
			gpu_storage->tex_pos_buffer->getCudaPtr(),
			gpu_storage->triangle_buffer->getCudaPtr(),
			gpu_storage->patch_info_buffer->getCudaPtr());




	return updated_set;
	/*
	Matrix4f proj = Camera::genProjMatrix(mesh->params.depth_fxycxy);
	Matrix4f pose = depth_pose_in;
	Matrix4f pose_tmp = pose.inverse();
	Matrix4f proj_pose = proj * pose_tmp;

	// Get the position of the depth camera
	Vector4f cam_pos = Camera::calcCamPosFromExtrinsic(pose_tmp);

	vector<shared_ptr<Meshlet>> patches = active_set->retained_mesh_patches_cpu;

	// Creation of the descriptors for this job
	vector<gpu::UpdateDescriptor> descriptors;
	vector<shared_ptr<Meshlet>> updated_patches;

	vector<shared_ptr<TexAtlasPatch>> dest_tex_handles; //retain the destination tex handles for this function

	for(size_t i = 0; i < patches.size(); i++) {

		//TODO: maybe we should not update every visible patch. we might want
		//to check if it really is feasible on every patch. some of them
		//might be hidden or some of them might be too far away and the surface
		//estimate is already waaay better than what the sensor could deliver.
		//?????
		//TODO: maybe the generation of this descriptor fits into
		//the mesh patch class.
		shared_ptr<Meshlet> patch = patches[i];

		shared_ptr<MeshletGpuHandle> gpu = patch->gpu.lock();
		if(!patch->isPartOfActiveSetWithNeighbours(active_set.get())) {
			continue;
		}

		shared_ptr<MeshTextureGpuHandle> geom_tex_gpu_handle = 
				patch->geom_tex_patch->gpu.lock();
		if(geom_tex_gpu_handle == nullptr) {
			MeshTextureGpuHandle *debug_tex =
					active_set->retained_mesh_patches[i]->geom_tex.get();
			cout << "how come this is not initialized yet?" << endl;
			assert(0);
			continue;//actually this can happen in multithreaded mode
			assert(0);
		}
		if(geom_tex_gpu_handle->tex == nullptr) {
			continue;
		}

		if(!geom_tex_gpu_handle->checkRefTexDependencies()) {
			//assert(0) because we might want to fix the dependencies before entering this loop
			//maybe even before entering this method because the refTexCould be fixed during
			//creation of the activeSet
			assert(0);//should basically do the same as the tests after
		}
		bool expired = false;
		//TODO: Shorten this by putting it into a method of the geometryBase class
		if(geom_tex_gpu_handle->ref_tex_dependencies.empty()) {
			assert(0);
			continue;//don't do a update since the refTex obviously is not set (indicated by it having no dependencies)
			//TODO: create/update the refTex if all the necessary neighbours are also part of this set
		}
		for(size_t j = 0; j < geom_tex_gpu_handle->ref_tex_dependencies.size(); j++) {
			MeshTextureGpuHandle::Dependency dependency = 
					geom_tex_gpu_handle->ref_tex_dependencies[j];

			shared_ptr<GeometryBase> dependence_geom = dependency.geometry.lock();
			if(dependence_geom == nullptr ||
			   dependence_geom->getMostCurrentGpuTriangles() == nullptr) {
				assert(0);
				//this should not happen at any time!!!!!!!!!
			}

			if(dependence_geom->triangles_version != dependency.triangles_version) {
				expired = true;
				assert(0);
			}
			shared_ptr<TriangleBufConnector> dependence_tris = dependence_geom->getMostCurrentGpuTriangles();
			int debug = dependence_tris->getStartingIndex();
			if(dependence_tris->getStartingIndex() != dependency.triangle_position_on_gpu) {
				expired = true;
				assert(0);
			}
		}
		if(expired) {
			//TODO: rebuild the reference texture. (which is supposed to be done in the ActiveSet creation process!)
			assert(0);
		}

		//also not run this if the reference texture is not filled
		if(!patch->geom_tex_patch->ref_tex_filled) {
			assert(0);
			continue;
		}
		updated_patches.push_back(patch);

		gpu::UpdateDescriptor desc;
		shared_ptr<gfx::GpuTex2D> tex = geom_tex_gpu_handle->tex->getTex();
		cv::Rect2i rect = geom_tex_gpu_handle->tex->getRect();
		float width  = tex->getWidth();
		float height = tex->getHeight();
		desc.source_geometry = tex->getCudaSurfaceObject();

		//wow, this really has to be streamlined
		desc.source = rect;
		desc.source_n = cv::Rect2f(float(rect.x) / width,
		                           float(rect.y) / height,
		                           float(rect.width) / width,
		                           float(rect.height) / height);
		desc.source_size = cv::Size2i(width, height);

		desc.patch_info_slot = gpu->patch_infos->getStartingIndex();

		//set the references to the vertex data.
		desc.vertex_source_start_ind = gpu->vertices_source->getStartingIndex();

		desc.vertex_count = gpu->vertices_source->getSize();
		desc.vertex_destination_start_ind = gpu->vertices_dest->getStartingIndex();

		desc.triangle_slot  = gpu->triangles->getStartingIndex();
		desc.triangle_count = gpu->triangles->getSize();

		//now go for the destRect textures
		shared_ptr<TexAtlasPatch> dest_tex = 
				gpu_storage->tex_atlas_stds_->getTexAtlasPatch(rect.size());
		//assert(0); // TODO: also copy over the  dependencies belonging to the referenceTexture if we are really copying

		tex  = dest_tex->getTex();
		rect = dest_tex->getRect();
		//width and height of the atlas texture
		width  = tex->getWidth();
		height = tex->getHeight();

		desc.destination_geometry = tex->getCudaSurfaceObject();
		desc.destination = rect;
		desc.destination_n = cv::Rect2f(float(rect.x) / width,
		                                float(rect.y) / height,
		                                float(rect.width) / width,
		                                float(rect.height) / height);
		desc.destination_size = cv::Size2i(width, height);

		desc.destination_references = 
				geom_tex_gpu_handle->ref_tex->getCudaSurfaceObject();

		rect = geom_tex_gpu_handle->ref_tex->getRect();
		desc.reference_offset = rect.tl();

		descriptors.push_back(desc);
		dest_tex_handles.push_back(dest_tex);
	}

	//make the according buffers resident on the gpu

	auto time_start = chrono::high_resolution_clock::now();

	if(dest_tex_handles.size() != updated_patches.size()) {
		assert(0);
	}

	//desperate debug measure
	for(auto patch : patches) {
		shared_ptr<MeshletGpuHandle> gpuPatch = patch->gpu.lock();
		if(gpuPatch == nullptr) {
			assert(0);
		}
		if(gpuPatch->geom_tex == nullptr) {
			assert(0);
		}
		if(gpuPatch->geom_tex->tex == nullptr) {
			assert(0);
		}
		if(gpuPatch->geom_tex->ref_tex == nullptr) {
			assert(0);
		}
	}

	int debug = updateGeometry(
			d_std_tex->getCudaSurfaceObject(), width, height, descriptors, cam_pos, 
			pose_tmp, proj_pose, 
			(GpuVertex*) gpu_storage->vertex_buffer->getCudaPtr(),
			(Vector2f*) gpu_storage->tex_pos_buffer->getCudaPtr(),
			(GpuTriangle*) gpu_storage->triangle_buffer->getCudaPtr(),
			(GpuPatchInfo* )gpu_storage->patch_info_buffer->getCudaPtr());

	//cout << "here we should first do the vertex update and then the std texture update" << endl;

	gpu::GeometryUpdate::calcCenterAndRadius(patches);

	cudaDeviceSynchronize();

	//unmap the graphics ressources.

	//after updating the target we want to switch target and source
	for(size_t i = 0; i < updated_patches.size(); i++) {
		shared_ptr<Meshlet> patch = updated_patches[i];
		shared_ptr<MeshletGpuHandle> gpu = patch->gpu.lock();
		shared_ptr<MeshTextureGpuHandle> gpu_tex_patch =
				patch->geom_tex_patch->gpu.lock();

		gpu->geom_tex->tex = dest_tex_handles[i];//swap the std texture
		gpu->swapSrcDst();//swap the vertices
		//TODO: get this swap to gpu header

		//TODO: get rid of all the rest!?

		//we trigger the reupload(update) of the patch header on the gpu
		patch->cpu_tex_patch_ahead = true;//maybe we want to do this update with a separate kernel call

		//TODO: get this swap to gpu header
		//TODO: and instead of swapping
		//gpu->swapSrcDst();
		//gpu->geomTex->swapSrcDst(); // forgot to swap the textures
		//patch->swapVertexTargetSource();
		patch->cpu_info_ahead = true;//maybe we want to do this update with a separate kernel call

		//trigger download of geometry in this case.
		gpu->gpu_vertices_changed = true;
		gpu->download_to_when_finished = patch;

		//mask the newly updated geom texture as something that should be downloaded
		//patch->geomTexPatch->gpuContentAhead = true;

		//enable the download of mesh textures
		//gpuTexPatch->downloadToWhenFinished = patch->geomTexPatch;
		gpu_tex_patch->gpu_data_changed = true;
	}
	mesh_reconstruction->cleanupGlStoragesThisThread_();
	*/
}
