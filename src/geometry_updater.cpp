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
		for(auto bla : active_set_of_formerly_visible_patches->meshlet_inds){
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


	/*******************************************STITCHING NEW********/
	vector<vector<Edge>> borders; //TODO: completely remove after the dependency on it is removed
	Matrix4f proj_pose = proj_depth * depth_pose_in.inverse();

	//TODO: comment back in obviously

    stitching.checkTriangleEdgeConsistency();
    int debug_border_count = stitching.border_list.size();
	stitching.genBorderList(
			visible_meshlets, borders,
			proj_pose);

    stitching.checkTriangleEdgeConsistency();
	stitching.reloadBorderGeometry(borders);
	//cv::imshow("DEBUG:geom_before", ex_geom);
	stitching.rasterBorderGeometry(borders, depth_pose_in, proj_depth, ex_geom);
	//cv::imshow("DEBUG:geom_after_rastering", ex_geom);
	//cv::waitKey();
	//this is a premilary measure to get the geometry adding running....
	float geometry_assign_threshold = 0.05f;//every point within 5cm of existing geometry is considered part of that geometry

	/**
	 * Invalidating invalid pixel that for some reasons are not invalidated already
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
	/********************************************************END WORKAROUND*****************************************/



	#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
	cv::imshow("novel geometry", points);
	cv::waitKey(); // This is a debug measure.... because everything is shit
	#endif

	// Actually we store pointers in this
	cv::Mat mesh_pointers(height, width, CV_32SC2);

	gpu_pre_seg_.fxycxy       = reconstruction->params.depth_fxycxy;
	gpu_pre_seg_.max_distance = reconstruction->getMaxDistance();
	// Do a proper segmentation on all the pixel not part of the existing geometry
	gpu_pre_seg_.segment(d_std_tex, proj_depth_std, ex_geom);

	cv::Mat seg   = gpu_pre_seg_.getSegmentation();
	int seg_count = gpu_pre_seg_.getSegCount();

	//*****************************************TOOOOODOOOOOO*********************
	//TODO: put this piece of code into the meshify part!!!!!
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

	meshing.meshify(points, mesh_pointers, vertex_indices, d_std_mat,
					reconstruction->params.max_depth_step, depth_pose_in);

	cv::Mat rgb_mat(rgb_tex->getHeight(),rgb_tex->getWidth(),CV_8UC4);
	rgb_tex->downloadData(rgb_mat.data);

	meshing.colorize(new_shared_mesh_patches, rgb_mat,
			depth_pose_in, color_pose_in,
			reconstruction->params.depth_fxycxy, reconstruction->params.rgb_fxycxy);

	for(size_t i = 0; i < new_shared_mesh_patches.size(); i++) {
		//TODO: unify these functions and maybe do this at the very end of everything!!!!!
		new_shared_mesh_patches[i]->updatePrincipalPlaneAndCenter();
		new_shared_mesh_patches[i]->updateSphereRadius();
	}

	reconstruction->checkNeighbourhoodConsistency();
	reconstruction->checkTriangleVerticesConsistency();

	auto time_end = chrono::system_clock::now();
	auto time_elapsed = chrono::duration_cast<chrono::milliseconds>(time_end - time_start_all);
	//present
	cout << "[GeometryUpdate::Extend] time consumed by meshing: " 
	     << time_elapsed.count() << "ms" << endl;

	auto time_start = chrono::system_clock::now();

	/*********************************Initial Stitching!! NEW***************************/


	//vector<weak_ptr<GeometryBase>> stitch_list;

	//TODO: reinsert code and fix bugs
	//cout << "TODO: reinsert this code and fix bugs" << endl;
    //reconstruction->checkTriangleEdgeConsistency();
    stitching.checkTriangleEdgeConsistency();

	stitching.stitchOnBorders(borders, depth_pose_in, proj_depth, proj_depth_std, 
	                          ex_geom, points, d_std_mat, 
	                          reconstruction->generateColorCodedTexture_(mesh_pointers),
	                          mesh_pointers, vertex_indices);


	stitching.freeBorderList(borders);

	reconstruction->checkLeftoverEdges();// there shouldn't be edges left (seemingly stitchOnBorders isn't really clean)
	reconstruction->checkNeighbourhoodConsistency();
	reconstruction->checkTriangleVerticesConsistency();

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

	/******************************REMOVAL OF UNCONNECTED VERTS TRIS and PATCHES***********/
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
							//found a neighbouring meshlet to house the meshlet
							target_meshlet = triangle->vertices[l]->meshlet;
							break;
						}
					}
					//we found a suitable  spot for the vertex. so we move it
					if(target_meshlet){
						//put the vertex into another meshlet
						vertex.meshlet = target_meshlet;//this is cruical because it is not done by the move operator
						target_meshlet->vertices.push_back(move(vertex));
						//TODO: make move private and provide a method that only allows the valid way of doing this
						cout << "MOVING VERTICES TO OTHER MESHLET" << endl;
						break;
					} else{
						//It should be guaranteed that there is a suitable target meshlet
						assert(0);
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
		//cout << "TODO: Put this back in!" << endl;
		//TODO: fix and reinsert the removal of invalid patches
		reconstruction->removePatch(set_of_patches_to_be_removed[i]);
		cout << "REMOVING PATCH" << endl;
	}
	set_of_patches_to_be_removed.clear();

	//TODO: check consistency on the whole reconstruction

	reconstruction->checkTriangleVerticesConsistency();
	reconstruction->checkNeighbourhoodConsistency();

	/******************************REMOVAL OF UNCONNECTED VERTS TRIS and PATCHES***********/

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
	vector<bool> empty;
	shared_ptr<ActiveSet> new_active_set =
			make_shared<ActiveSet>(gpu_storage,
									visible_meshlets,
									scheduler->getActiveSets(),
                                    empty,true);
			/*
			gpu_storage->makeActiveSet(
					visible_meshlets,
					reconstruction,
					low_detail_renderer,
					texture_updater,
					information_renderer, true); // most of the members are initial
					*/
	new_active_set->name = "createdInApplyNewData";


	time_end = chrono::system_clock::now();
	time_elapsed = chrono::duration_cast<chrono::milliseconds>(time_end - time_start);
	//present
	cout << "[GeometryUpdate::Extend] Time consumed loading the active set: " <<
	        time_elapsed.count() << "ms" << endl;

	time_start = time_end;


	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	new_active_set->setupHeaders();
	texture_updater->generateGeomTex(reconstruction,
			                        gpu_storage,
			                        new_shared_mesh_patches, depth_pose_in,
	                                proj_depth, d_std_tex, new_active_set,
	                                information_renderer);

    //TODO: remove debug:
    for(size_t i=0; i<new_shared_mesh_patches.size();i++){
        auto gpu_meshlet = new_active_set->getGpuMeshlet(new_shared_mesh_patches[i]);
        if(gpu_meshlet->std_tex.tex == nullptr){
            assert(0);
        }
        if(gpu_meshlet->std_tex.tex->getRect().height == 0 || gpu_meshlet->std_tex.tex->getRect().width == 0) {
            assert(0);
        }
        if(gpu_meshlet->std_tex.token == nullptr){
            assert(0);
        }

        /*
        if(new_shared_mesh_patches[i]->geom_tex_patch->mat.empty()){
            assert(0);
        }*/
    }

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

    //TODO: remove this debug
    for(size_t i = 0; i < new_shared_mesh_patches.size(); i++) {
        if(new_shared_mesh_patches[i]->geom_tex_patch == nullptr){
            assert(0);
        }
        //if(new_shared_mesh_patches[i]->geom_tex_patch)
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

	for(auto patch : new_shared_mesh_patches)
		reconstruction->octree_.add(patch);

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
		Vector3f pos = patch->center();
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

	//TODO: remove this debug thingy!
    for(size_t i=0;i<new_shared_mesh_patches.size();i++){
        auto meshlet = new_shared_mesh_patches[i];
        //if(new_shared_mesh_patches[i]->id == 607){
            //cout << "this is that famous 607" << endl;
            auto gpu_meshlet = new_active_set->getGpuMeshlet(meshlet);
            //TODO: maybe check where the token went
            auto active_sets = scheduler->getActiveSets();
            active_sets.push_back(new_active_set);
            bool token_exists=false;
            bool exists_in_active_set = false;
            for(auto s : active_sets){
                if(s==nullptr){
                    continue;
                }
                exists_in_active_set = true;
                auto gpu_meshlet_2 = s->getGpuMeshlet(meshlet);
                if(gpu_meshlet_2 != nullptr){

                    if(meshlet->id == 607){
                        gpu_meshlet_2->std_tex.debug= 607;
                    }
                    if(gpu_meshlet_2->std_tex.token != nullptr){
                        token_exists = true;
                    }
                }
            }
            if(!token_exists){
                assert(0);
            }
        //}
    }
    for(size_t i=0;i<visible_meshlets.size();i++){
        auto meshlet = visible_meshlets[i];
        //if(new_shared_mesh_patches[i]->id == 607){
        //cout << "this is that famous 607" << endl;
        auto gpu_meshlet = new_active_set->getGpuMeshlet(meshlet);
        //TODO: maybe check where the token went
        auto active_sets = scheduler->getActiveSets();
        active_sets.push_back(new_active_set);
        bool token_exists=false;
        bool exists_in_active_set = false;
        for(auto s : active_sets){
            if(s==nullptr){
                continue;
            }
            exists_in_active_set = true;
            auto gpu_meshlet_2 = s->getGpuMeshlet(meshlet);
            if(gpu_meshlet_2 != nullptr){
                if(gpu_meshlet_2->std_tex.token != nullptr){
                    token_exists = true;
                }
            }
        }
        if(!token_exists){
            assert(0);
        }
        //}
    }
	//TODO: remove this debug thingy!
	for(size_t i=0;i<new_active_set->meshlets.size();i++){
	    if(new_active_set->meshlets[i].std_tex.tex == nullptr){
	        assert(0);
	    }
	}

	return new_active_set;
}

shared_ptr<ActiveSet> GeometryUpdater::update(
		GpuStorage* gpu_storage,
		InformationRenderer* information_renderer,
		vector<shared_ptr<Meshlet>> requested_meshlets,
		shared_ptr<ActiveSet> preexisting_set, // the active set(if existing) containing all requested meshlets
		SchedulerBase* scheduler,//to generate a new active set with
		shared_ptr<gfx::GpuTex2D> d_std_tex,
		Matrix4f depth_pose_in,
		Matrix4f depth_proj) {

	//return preexisting_set;
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
	//TODO: reinsert this
	//if(preexisting_set == nullptr){
		cout << "Setting up an active set because the old one is empty" << endl;
		//create a set of pre existing
		preexisting_set = make_shared<ActiveSet>(gpu_storage,requested_meshlets,active_sets);
		active_sets.push_back(preexisting_set); // in case we had to create a new active set put it to the list
	//}

	//preexisting_set->assertAllGeometry();
	{
		for(MeshletGPU &meshlet_gpu : preexisting_set->meshlets){
			if(meshlet_gpu.std_tex.tex == nullptr){
				for(int i=0;i<requested_meshlets.size();i++){
					if(requested_meshlets[i]->id == meshlet_gpu.id){
						shared_ptr<Meshlet> requested_meshlet = requested_meshlets[i];
						assert(0);
					}
				}
				assert(0);
			}
			if(meshlet_gpu.vertices == nullptr)
				assert(0);
			if(meshlet_gpu.triangles == nullptr)
				assert(0);
		}
	}

	assert(preexisting_set->hasAllGeometry());
	vector<shared_ptr<Meshlet>> meshlets_to_update;
	vector<bool> update_mask(requested_meshlets.size(),false);
	for(size_t i=0;i<requested_meshlets.size();i++){
		shared_ptr<Meshlet> &meshlet = requested_meshlets[i];
		bool has_all_neighbours = true;
		for(size_t j = 0; j<meshlet->neighbours.size();j++){
			shared_ptr<Meshlet> nb = meshlet->neighbours[j].lock();
			if(nb == nullptr)
				continue; // should barely ever happen
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

	cout << "GeometryUpdater requested " << requested_meshlets.size() << " vs updated " << meshlets_to_update.size() << endl;

	shared_ptr<ActiveSet> updated_set =
			make_shared<ActiveSet>(gpu_storage,requested_meshlets,active_sets,update_mask);

	vector<gpu::UpdateDescriptor> update_descriptors(meshlets_to_update.size());
	vector<gpu::GeometryUpdate::TranscribeStitchTask> transcribe_tasks(meshlets_to_update.size());

	//in some instances we need to create new lookup textures.
	vector<DilationDescriptor> dilations;
	information_renderer->bindRenderTriangleReferenceProgram(gpu_storage);

	for(size_t i=0;i<meshlets_to_update.size();i++){
		if(i == 2){
			cout << "DEBUG: is this the meshlet where the update fails?" << endl;
		}
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

		//TODO: extend this criteria to things like triangle_version and other more fundamental updates
		if(meshlet_gpu_dst->geom_lookup_tex == nullptr){
			// create a new reference texture here (since it will be needed)
			cv::Size2f s = rect.size();
			meshlet_gpu_dst->geom_lookup_tex = gpu_storage->tex_atlas_geom_lookup_->getTexAtlasPatch(s);
			cv::Rect2i r = meshlet_gpu_dst->geom_lookup_tex->getRect();

			information_renderer->renderReference(meshlet_gpu_dst, meshlet_gpu_dst->std_tex.coords,
												  meshlet_gpu_dst->geom_lookup_tex);

			dilations.emplace_back();
			DilationDescriptor& dil_desc = dilations.back();
			dil_desc.target = meshlet_gpu_dst->geom_lookup_tex->getCudaSurfaceObject();
			dil_desc.width  = r.width;
			dil_desc.height = r.height;
			dil_desc.x      = r.x;
			dil_desc.y      = r.y;
		}else{
			desc.destination_references = meshlet_gpu_dst->geom_lookup_tex->getCudaSurfaceObject();
		}
		desc.reference_offset = meshlet_gpu_dst->geom_lookup_tex->getRect().tl();


		desc.src_verts = meshlet_gpu_src->vertices->getStartingPtr();
		desc.dst_verts = meshlet_gpu_dst->vertices->getStartingPtr();
		desc.triangles = meshlet_gpu_src->triangles->getStartingPtr();
		desc.src_tex_coords = meshlet_gpu_src->std_tex.coords->getStartingPtr();
		desc.vertex_count = meshlet_gpu_src->vertices->getSize();
		//desc.dst_tex_coords = meshlet_gpu_dst->std_tex.coords->getStartingPtr();

		desc.update_texture = true;
		//TODO: setup descriptors


		gpu::GeometryUpdate::TranscribeStitchTask & transcribe_task = transcribe_tasks[i];

		transcribe_task.local_vertices = meshlet_gpu_dst->vertices->getStartingPtr();
		if(meshlet_gpu_dst->gpu_vert_transcribe_tasks == nullptr) { // some meshlets don't contain anything
			bool debug = updated_set->containsNeighbours(meshlet);
			//assert(0);
		}
		transcribe_task.task = meshlet_gpu_dst->gpu_vert_transcribe_tasks;
		transcribe_task.count = meshlet_gpu_dst->gpu_vert_transcribe_task_count;
		transcribe_task.nb_vertices = meshlet_gpu_dst->gpu_neighbour_vertices;


	}


	//TODO: dilate the rendered lookup textures
	dilateLookupTextures(dilations);
	cudaDeviceSynchronize();

	//TODO: remove this desparate attempt to debugging
	//Matrix4f temp = depth_pose_in;
	//depth_pose_in = temp.inverse();


	Vector4f cam_pos = Camera::calcCamPosFromExtrinsic(depth_pose_in);
	Matrix4f depth_pose = depth_pose_in.inverse();
	Matrix4f proj_pose = depth_proj * depth_pose_in.inverse();

	cout << "DEBUG: Geometry Update: " << update_descriptors.size() << " vs " << updated_set->meshlets.size() << endl;
	gpu::updateGeometry(
			d_std_tex->getCudaSurfaceObject(),
			d_std_tex->getWidth(),d_std_tex->getHeight(),
			update_descriptors,
			transcribe_tasks,//updated_set->transcribe_tasks,
			cam_pos,
			depth_pose,
			proj_pose);

	updated_set->setupHeaders(true);

	//explicitely clear some references to find a bug
	preexisting_set = nullptr;
	active_sets.clear();

	return updated_set;

}
