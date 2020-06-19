#include <mesh_stitcher.h>

#include <Eigen/Eigen>

#include <mesh_reconstruction.h>
#include <stitching_utils.h>
#include <cuda/coalesced_memory_transfer.h>
#include <gpu/active_set.h>

using namespace std;
using namespace Eigen;

void MeshStitcher::rasterBorderGeometry(Matrix4f view, Matrix4f proj,
										cv::Mat geometry) {

    vector<vector<Edge>> &borders = this->border_list;
	Matrix4f view_inv = view.inverse();
	cv::Mat debug = geometry.clone(); //TODO: get rid of this
	debug = 0;
	for(size_t i = 0; i < borders.size(); i++) {
		for (size_t j = 0; j < borders[i].size(); j++) {
			Edge *edge = &borders[i][j];
			rasterLineGeometry(view_inv, proj, edge, geometry, debug);
		}
	}
    //cv::imshow("rastered lines",debug);
	//cv::waitKey(100);
}
void MeshStitcher::checkTriangleEdgeConsistency() {

    vector<vector<Edge>> &borders = this->border_list;
    for(size_t i = 0; i < borders.size(); i++) {
        Vertex *last_vert = borders[i][0].vertices(1);

        Vertex* debug1 = borders[i][0].vertices(0);
        Vertex* debug2 = borders[i][0].vertices(1);
        for (size_t j = 0; j < borders[i].size(); j++) {
            Vertex* debug1_old = debug1;
            Vertex* debug2_old = debug2;
            Vertex* debug1 = borders[i][j].vertices(0);
            Vertex* debug2 = borders[i][j].vertices(1);
            assert(last_vert == borders[i][j].vertices((1)));

            last_vert = borders[i][j].vertices(0);
            Edge &edge = borders[i][j];
            if(edge.triangle->edges[edge.pos] != &edge){

                //check if the same edge is in another border
                for(size_t k = 0;k<borders.size();k++){
                    for(size_t l = 0; l<borders[k].size();l++){
                        if(k == i && l == j)
                            continue;
                        if(edge.triangle->edges[edge.pos] == &borders[k][l]){
                            assert(0); // check if the edge is in another border
                        }
                    }
                }
                assert(0);
            }
        }
    }
}

void MeshStitcher::rasterLineGeometry(Matrix4f view, Matrix4f proj, Edge *edge,
									  cv::Mat geometry, cv::Mat debug) {

	//first of all calculate the two points in screen space
	Matrix4f p_v = proj * view;
	int width = geometry.cols;
	int height = geometry.rows;

	//do a proper interpolation between depth values:
	//http://stackoverflow.com/questions/24441631/how-exactly-does-opengl-do-perspectively-correct-linear-interpolation
	Vertex* pi = edge->vertices(1);
	Vector4f P0 = pi->p;
	Vector4f projected0 = p_v * P0;
	Vector2f pix0 = Vector2f(projected0[0] / projected0[3],
	                         projected0[1] / projected0[3]);
	float w0 = projected0[3];
	Vector4f point0 = view * P0; //transform the point into camera space

	pi = edge->vertices(0);
	Vector4f P1 = pi->p;
	Vector4f projected1 = p_v*P1;
	Vector2f pix1 = Vector2f(projected1[0] / projected1[3],
	                         projected1[1] / projected1[3]);

	float w1 = projected1[3];
	Vector4f point1 = view * P1;

	struct Functor {

		Functor(cv::Mat &geometry, Vector4f &point0, Vector4f &point1, float &w0, 
		        float &w1, cv::Mat &debug)
				: geometry(geometry), 
				  point0(point0), 
				  point1(point1), 
				  w0(w0), 
				  w1(w1),
				  debug(debug) { }

		void operator() (int x, int y, float t) {
			int width = geometry.cols;
			int height = geometry.rows;
			if(x < width && y < height && x >= 0 && y >= 0) {
				Vector4f frag_pos = interpolatePerspectific(point0, point1, w0, w1, t);

				float depth = frag_pos[2];
				float current_depth = geometry.at<Vector4f>(y, x)[2];
				if(current_depth > depth || isnan(current_depth)) {
					if(isnan(frag_pos[0])) {
						cout << "this should never occur" << endl;
						assert(0);
					}
					geometry.at<Vector4f>(y, x) = frag_pos;//placeholder unfortunately
					debug.at<Vector4f>(y, x) = Vector4f(1, 1, 1, 1);
				} else {
					debug.at<Vector4f>(y, x) = Vector4f(1, 1, 1, 1);
				}
			}
		}

		cv::Mat &geometry;
		Vector4f &point0;
		Vector4f &point1;
		float &w0, &w1;
		cv::Mat &debug;
	};

	if(pix0.norm() > 10000 ||
	   pix1.norm() > 10000) {
		//this is a poor fix to ignore points which get projected far
		//outside of the view plane
		return;
	}

	Functor f(geometry, point0, point1, w0, w1, debug);
	bresenham(pix0, pix1, f);
}

void MeshStitcher::genBorderList(vector<shared_ptr<Meshlet>> &patches,
								 Matrix4f debug_proj_pose) {
	float s = 2.0f;
	float s2 = 1.0f;
	cv::Vec2f offset(640,480);
	offset = cv::Vec2f(0,0);
	cv::Mat debug_mat(480 * (s + 1), 640 * (s + 1), CV_8UC3);
	debug_mat.setTo(0);

	unordered_set<shared_ptr<Meshlet>> patch_set(patches.begin(), patches.end());
	unordered_set<Meshlet*> meshlets;

	//TODO: remove
	//DEBUG: to enxure readability we will create a vector
	vector<Meshlet*> debug_patches_ordered;
	for(int i = 0; i < patches.size(); i++) {
		debug_patches_ordered.push_back(patches[i].get());
	}
	sort(debug_patches_ordered.begin(), debug_patches_ordered.end(),
	     [](const Meshlet *a, const Meshlet *b) {return a->id > b->id;});

	//iterate over all patches and triangles

	for(shared_ptr<Meshlet> patch : patches) {
		//iterate over all triangles belonging to this patch and its stitches
		//in case of an empty unused edge we follow it in both directions
		meshlets.insert(patch.get());

	}

	cv::Point2i pcv1_old;
	cv::Point2i pcv2_old;
	auto renderEdge = [&](Edge &edge) {
		return;
		Vector4f p1 = edge.vertices(0)->p;
		p1 = debug_proj_pose * p1;
		p1 = p1 * (1.0f / p1[3]);
		Vector4f p2 = edge.vertices(1)->p;
		p2 = debug_proj_pose * p2;
		p2 = p2 * (1.0f / p2[3]);

		//Now draw a line via OpenCV
		cv::Point2i pcv1(p1[0] * s * s2 - s*offset[0], p1[1] * s * s2 - s * offset[1]);
		cv::Point2i pcv2(p2[0] * s * s2 - s*offset[0], p2[1] * s * s2 - s * offset[1]);

		cv::line(debug_mat, pcv1_old, pcv2_old, cv::Scalar(255, 0, 0));
		cv::line(debug_mat, pcv1, pcv2, cv::Scalar(0, 0, 255));
		pcv1_old = pcv1;
		pcv2_old = pcv2;
		cv::imshow("edges", debug_mat);
		cv::waitKey(1);
	};
	auto addBorderStrip = [&]() {
		auto initialEdge = [&]() -> Edge*{
			//The first element of a given border is the seed.
			//and its clear that we always work on the last border in our list.
			return &(this->border_list.back()[0]);
		};
		Edge* current_edge = initialEdge();
		renderEdge(*current_edge);
		bool following_border = true;
		bool ran_in_circle = false;
		vector<Edge> &forward = this->border_list.back();
		vector<Edge> backward;
		Edge* debug_last_edge = current_edge;
		while(following_border) {
			Edge* result;
			current_edge->getOrAttachNextEdge(0, result, forward, false, 10);
			current_edge = result;
			renderEdge(*current_edge); //debug

			if(meshlets.count(current_edge->triangle->getMeshlet()) != 1) {
                //stop following the edge as soon as we encounter an unknown meshlet
                //since that invalid edge already is attached to the list we need to undo this
			    forward.pop_back();
				following_border = false;//if the new edge is attached to a patch outside we abort.
				continue;
			}

			if(current_edge->is_registered) {
				following_border = false; //we potentially ran in circle
				if(current_edge != initialEdge()) {
					//one edge should never hit anoter edge than the initial edge within the same border
					for(int i = 0; i < border_list.back().size(); i++) {
						if(current_edge == &border_list.back()[i]) {
							assert(0);//intersecting the current border
						}
					}
					for(int i=0; i < border_list.size(); i++){
						for(int j=0; j < border_list[i].size(); j++){
							if(current_edge == &border_list[i][j]) {
								cv::waitKey();
								assert(0);//intersecting with another border
							}

						}
					}
					debug_last_edge->getOrAttachNextEdge(0, current_edge, forward,true, 10);
					assert(0);//only the initial triangle should be registered
				} else {
					ran_in_circle = true;
				}
			} else {


				if(current_edge == initialEdge()){
					assert(0);//the initial edge should be registered so we should never be in this branch
				}
				//TODO: update the getOrAttachNextEdge method to initialize the new edge directly in the list
				//this would remove the need to delete the current edge, and also the need for the move operator


				//TODO: remove registration
				//register the new edge:

				//TODO: simon2020 put back in & fix or delete completely
				/*
				current_edge.registerInTriangle(border_list.size() - 1,
				                                border_list.back().size());
				current_edge.debug = border_list.size() - 1;
				border_list.back().push_back(current_edge);
				forward.push_back(current_edge);
				if(current_edge.equalTo(initial_edge)) {
					assert(0);//the initial edge should be registered so we should never be in this branch
				}
				 */
			}
			debug_last_edge = current_edge;
		}


		if(!ran_in_circle) {
			//if we didn't follow the border in a full loop we start again from the initial triangle
			following_border = true;
			current_edge = initialEdge();
			while(following_border) {
				Edge* result;
				current_edge->getOrAttachNextEdge(1, result, backward, false, 11);//other direction
				current_edge = result;
                if(result != result->triangle->edges[result->pos]){
                    //assert(0); //TODO: remove this debug measure
                }
				renderEdge(*current_edge);
				if(meshlets.count(current_edge->triangle->getMeshlet()) != 1) {
					//stop following the edge as soon as we encounter an unknown meshlet
					//since that invalid edge already is attached to the list we need to undo this
					backward.pop_back();
					following_border = false;//if the new edge is attached to a patch outside we abort.
					continue;
				}
				if(current_edge->is_registered) {
					//this means we ran in circles
					if(current_edge != initialEdge()) {
						//the only registered edge we should ever encounter is the initial edge
						// everything in this branch is done for debugging purposes
						//TODO: remove this at one point
						for(int i = 0; i < border_list.size(); i++) {
							for(int j = 0; j < border_list[i].size(); j++) {
								renderEdge(border_list[i][j]);
							}
						}
						for(int i = 0; i < forward.size(); i++) {
							renderEdge(forward[i]);
						}
						for(int i = 0; i < backward.size(); i++) {
							renderEdge(backward[i]);
						}
						cv::waitKey(1);
						int debug_size = border_list.size();
						Triangle *current_triangle = current_edge->triangle;
						Triangle *last_triangle    = debug_last_edge->triangle;
						Edge *new_edge = nullptr;
						debug_last_edge->getOrAttachNextEdge(1, new_edge, backward, true, -1);

						Edge *new_edge2 = nullptr;
						debug_last_edge->getOrAttachNextEdge(1, new_edge2, backward, true, -1);

						assert(0);// this should not happen!!!!!
					} else {
						assert(0);
					}
				} else {
					//new approach: add the new edge to the list
					//backward.push_back(std::move(*current_edge));
					//delete current_edge;
					//current_edge = &backward.back();
					//TODO: see if this is still needed and delete if not/put back in if it is
					/*
					//register the new edge:
					current_edge->registerInTriangle(border_list.size() - 1,
					                                border_list.back().size());
					current_edge.debug = border_list.size() - 1;
					border_list.back().push_back(current_edge);
					backward.push_back(current_edge);

					if(current_edge.equalTo(initial_edge)) {
						assert(0);
					}
					 */
				}
			}

			reverse(backward.begin(), backward.end());
			//we need to use the move operator here so that all the edges get
			//https://stackoverflow.com/questions/38121833/should-stl-mapinsert-support-move-semantics-with-move-iterators
			backward.insert(backward.end(), make_move_iterator(forward.begin()), make_move_iterator(forward.end()));

			//but why is this????
			/*
			if(backward[1610].triangle.get() == backward[2188].triangle.get() &&
				backward[1610].pos == backward[2188].pos){
				assert(0);//why are they the same? they should not be!!!!
			}
			 */
			border_list.back() = move(backward);
		}
		//registering the edges to the triangles
		for(Edge &edge : border_list.back()){
			edge.registerInTriangle();
		}
	};

	for(GeometryBase *geom_base : debug_patches_ordered) {
		for(size_t j = 0; j < geom_base->triangles.size(); j++) {
			Triangle &triangle = geom_base->triangles[j];
			Triangle* tri_ref = & triangle;
			for(int k : {0, 1, 2}){
				if(!triangle.neighbours[k].valid()) {
					if(triangle.edges[k] == nullptr) {
						//register the new edge:
						border_list.emplace_back();//create a new list of edges (read border) at its end
						border_list.back().emplace_back(tri_ref, k);
						Edge &edge = border_list.back().back();
						triangle.edges[k] = &edge;
						edge.registerInTriangle();

						addBorderStrip();//recursively append this edge!!!!
						//cv::waitKey();
					}
				}
			}
		}
	}
	checkTriangleEdgeConsistency();//TODO: remove consistency
}

void MeshStitcher::reloadBorderGeometry(shared_ptr<ActiveSet> active_set) {

	//i very much fear what happens when something here changes
    //TODO: mutex all CPU side geometry since we are working with tons of pointers



	cout << "TODO: fix and put back in MeshStitcher::reloadBorderGeometry" << endl;
	return;
    //TODO: this method needs the map, and according active sets (maybe just one) to retreive the geometry from
	//assert(0);//Do this later... definitely part of the the active sets responsibilities!

	//retreive the set of vertices that require an update and store the index of the vertices in an according gpu-meshlet
	unordered_map<Vertex*, int> vertex_gpu_indices;
    for(size_t i=0;i<border_list.size();i++){
        for(size_t j=0;j<border_list[i].size();j++){
            Triangle* tri = border_list[i][j].triangle;
            //indices within triangles
            int ind1 = border_list[i][j].pos;
            int ind2 = ind1 + 1;
            if(ind2 == 3) {
                ind2 = 0;
            }
            vertex_gpu_indices[border_list[i][j].vertices(0)] =
                    tri->local_indices[ind1];
            vertex_gpu_indices[border_list[i][j].vertices(1)] =
                    tri->local_indices[ind2];
        }
    }

    vector<GpuVertex*> gpu_verts;
    vector<Vertex*> pts;
    //TODO: Create vector of gpu pointers from src to destination (and the proper tasks to write them into the according positions
    for(pair<Vertex*, int> v_i : vertex_gpu_indices){
        Meshlet* m = v_i.first->meshlet;
        MeshletGPU* m_gpu = active_set->getGpuMeshlet(m);
        if(m_gpu != nullptr){
            pts.push_back(v_i.first);

            //pointer to vertex on the gpu
            GpuVertex* gpu_vert = &(m_gpu->vertices->getStartingPtr()[v_i.second]);
            gpu_verts.push_back(gpu_vert);
        }
    }
    vector<GpuVertex> downloaded_data(gpu_verts.size());
    downloadVertices(gpu_verts, &(downloaded_data[0]));
    //spread the data to the vertices to do the mapping magic
    for(size_t i = 0; i < pts.size(); i++) {
        pts[i]->p = downloaded_data[i].p;
        assert(pts[i]->p[3] == 1.0f); // a relatively trivial debug measure...
    }


    //TODO: REMOVE OLD
    /*

	MeshReconstruction *mesh = mesh_reconstruction;

	set<Vertex*> vertices;
	set<VertexReference> point_refs;
	for(size_t i = 0; i < border_list.size(); i++) {
		for(size_t j = 0; j < border_list[i].size(); j++) {
			//all points of related triangles
			for(size_t k = 0; k < 3; k++) {
				VertexReference p = border_list[i][j].triangle.get()->points[k];
			}
		}
	}

	GpuVertex *gpu_vert_buf = mesh->gpu_geom_storage_.vertex_buffer->getCudaPtr();
	vector<GpuVertex> downloaded_data(point_refs.size());

	vector<GpuVertex*> gpu_verts;
	vector<VertexReference> pts;
	for(auto p : point_refs) {
		shared_ptr<MeshletGpuHandle> gpu = p.getPatch()->gpu.lock();
		if(gpu == nullptr) {
			#ifndef IGNORE_SERIOUS_BUG_2
			assert(0);
			#endif // IGNORE_SERIOUS_BUG_2
			//all the vertices fetched here should be loaded to the gpu and
			//and therefore also have vertices and a vertex buffer slot
			continue;
		}
		pts.push_back(p);
		int index = gpu->vertices_source->getStartingIndex() + p.getIndex();
		gpu_verts.push_back(&(gpu_vert_buf[index]));
	}

	downloadVertices(gpu_verts, &(downloaded_data[0]));
	//spread the data to the vertices to do the mapping magic
	for(size_t i = 0; i < pts.size(); i++) {
		pts[i].get()->p = downloaded_data[i].p;
	}
	cout << "[Stitching::reloadBorderGeometry] its over!!" << endl;
     */


}

//TODO: also download the geometry of such list (look at reload border geometry and delete this comment)
void MeshStitcher::freeBorderList() {
	for(size_t i = 0; i < this->border_list.size(); i++) {
		for(size_t j = 0; j < this->border_list[i].size(); j++) {
			Edge &edge = this->border_list[i][j];
			edge.unregister();
		}
	}
	this->border_list.clear();
}

void MeshStitcher::stitchOnBorders(Matrix4f view, Matrix4f proj,
		cv::Mat std_proj, cv::Mat geom_proj_m, cv::Mat new_geom_m, cv::Mat new_std,
		cv::Mat debug_color_coded_new_segmentation, cv::Mat new_seg_pm, 
		cv::Mat new_pt_ind_m) {
	//cout << "IMPLEMENT THIS!!!!! MeshStitcher::stitchOnBorders" << endl;
	//return;
	//assert(0); // this needs to be here but reimplemented
    vector<vector<Edge>> &borders = this->border_list;
	vector<Edge> additional_edges;

	MeshReconstruction *map = mesh_reconstruction;

	Matrix4f view_inv = view.inverse();
	Matrix4f p_v      = proj * view_inv;

	int width  = geom_proj_m.cols;
	int height = geom_proj_m.rows;

	Vector2i debug_last_pix(0, 0);
	auto isCW = [](Vector2f p1, Vector2f p2, Vector2f p3) {
		Vector2f v1 = p2 - p1;
		Vector2f v2 = p3 - p1;
		return (v1[0] * v2[1] - v1[1] * v2[0]) > 0;
	};

	auto isConnected = [](const Vertex* v1, const Vertex* v2) {
		for(int i = 0; i < v1->triangles.size(); i++) {
			//iterating over all the triangles connected to vertex 1
			Triangle *triangle = v1->triangles[i].triangle;
			for(int k: {0, 1, 2}) {
				if(triangle->vertices[k] == v2) {
					return true;
				}
			}
		}
		return false;
	};

	auto isOpenEdge = [](const Vertex* v1, const Vertex* v2) {
		for(int i = 0; i < v1->triangles.size(); i++) {
			//iterating over all the triangles connected to vertex 1
			Triangle *tri = v1->triangles[i].triangle;
			for(int k : {0, 1, 2}) {
				if(tri->vertices[k] == v2) {
					//found a triangle lets calculate the index the potential edge
					int ind = min(k, v1->triangles[i].ind_in_triangle);
					int inds[2] = {ind, max(k, v1->triangles[i].ind_in_triangle)};//is there a +1 missing?
					if(inds[0] == 0 && inds[1] == 2) {
						ind = 2;
					}
					if(tri->neighbours[ind].valid()) {
						return false;
					} else {
						//the triangle does not have any neighbours between points one and two:
						return true;
					}
				}
			}
		}
		#ifdef SHOW_TEXT_STITCHING
		cout << "DEBUG: these two points are not connected by any triangle at all." << endl;
		#endif // SHOW_TEXT_STITCHING
		return false;
	};
	//TODO implement and reduce overall file size

	auto isConnectedClosedEdge = [](Vertex* v1, Vertex* p2) {
		return false;
	};

	auto project3f = [&](const Vector4f &p) {
		Vector3f result;
		Vector4f point = view_inv * p;
		Vector4f projected = p_v * p;
		float w = projected[3];
		Vector2f pix = Vector2f(projected[0] / w, projected[1] / w);
		result = Vector3f(pix[0], pix[1], point[2]);
		return result;
	};

	auto project2i = [&](Vector4f p) {
		Vector3f a = project3f(p);
		Vector2i result(round(a[0]), round(a[1]));//TODO: improve rounding to nearest
		return result;
	};

	auto pos2pix2i = [&](Vector3f p) {
		Vector2i result(round(p[0]),round(p[1]));//TODO: improve rounding to nearest
		return result;
	};

	//TODO: find out what makes this method so necessary and different from "getOrAttachNextEdge"
	auto getNextOpenEdge = [&additional_edges](Vertex* v,
	                          Edge* &resulting_registered_edge) {
		if(v->encompassed()) {
			assert(0);
		}
		//To prevent this from outputting something of the existing geometry, this has to be called before stitching
		for(int i = 0; i < v->triangles.size(); i++) {
			int ind = v->triangles[i].ind_in_triangle;
			//ind--; // decrement if we want to go to the last edge
			if(ind == -1) {
				assert(0); // should not happen since we re not decrementing
				ind = 2;
			}
			Triangle *triangle = v->triangles[i].triangle;
			if(!triangle->neighbours[ind].valid()) {//if the triangle of that given index has
				if(triangle->edges[ind]!=nullptr){
					//there already is a registered option so use it
					resulting_registered_edge = triangle->edges[ind];
					resulting_registered_edge->debug += 1000;
					return true;
				}
				additional_edges.emplace_back();
				Edge &e = additional_edges.back();
				e.triangle = v->triangles[i].triangle;
				e.pos = ind;
				e.registerInTriangle();
				resulting_registered_edge = &e;
				resulting_registered_edge->debug = 100;
				return true;
			}
		}
		return false;
	};

	auto isTrianglePossible = [&](array<Vertex*, 3> v,
	                              array<bool, 3> test = {true, true, true},
	                              array<bool, 3> test_orientation = {true, true, true}) {
		for(size_t i = 0; i < 3; i++) {
			size_t i2 = i == 2 ? 0 : i + 1;
			if(!test[i]) {
				continue;
			}
			if(isConnected(v[i], v[i2])) {
				if(!isOpenEdge(v[i], v[i2])) {
					return false;
				}
				if(!test_orientation[i]) {
					continue;
				}
				Vertex *this_vert = v[i];
				for(int l = 0; l < this_vert->triangles.size(); l++) {
					if(this_vert->triangles[l].triangle->containsPoint(v[i2])) {
						int ind1 = this_vert->triangles[l].ind_in_triangle;
						int ind2 = this_vert->triangles[l].triangle->getPointIndex(v[i2]);
						if(ind1 != 0) {
							//cout << "this is astonishing" << endl;
						}
						if((ind1 > ind2 || (ind1 == 0 && ind2 == 2))) {//this query is just plain wrong
							//return false;
						}
						if((ind2==1 && ind1==0) || 
						   (ind2==2 && ind1==1) || 
						   (ind2==0 && ind1==2)) {
							return false;
						}
						break;
					}
				}
			}
		}
		//TODO: remove this debug query
		//check if this exact triangle already exists
		for(auto &tri : v[0]->triangles){
		    //calculate the other two indices
		    int ind1 = tri.ind_in_triangle;
		    Triangle* triangle = tri.triangle;
		    int ind2 = ind1 + 1;
		    if(ind2==3)
		        ind2=0;
		    int ind3 = ind2 + 1;
		        ind3=0;
            if(triangle->vertices[ind2] == v[1] && triangle->vertices[ind3] == v[2])
                assert(0);
            if(triangle->vertices[ind3] == v[1] && triangle->vertices[ind2] == v[2])
                assert(0);
		}
		return true;
	};

	for(int i = 0; i < borders.size(); i++) {
		//depending on if the last edge had a triangle or not it can be pretty clear 
		// if we want to create a new triangle
		// or not
		bool very_first_point = true;
		Vertex* first_p = borders[i][0].vertices(1);
		Vertex* last_p;
		Vector2f last_pix;
		bool triangle_created_last_edge = false;

		//VertexReference debugLastPr1;
		#ifdef SHOW_TEXT_STITCHING
		cout << "running along new border" << endl;
		#endif // SHOW_TEXT_STITCHING

		//TODO: implement this sewing algorithm that follows edges on both sides of the seam!!!!
		bool sewing_mode = false;
		//Vertex* current_sewing_pr;
		Vertex* current_sewing_p; //TODO: merge these two (this and above)
		//Vertex* last_sewing_pr;
		Vertex* last_sewing_p; //TODO: merge these two
		Edge *current_sewing_edge;
		Vector2i current_sewing_pix(-1, -1);
		Vector2i last_pix_i(-1, -1);

		auto checkAndStartSewing = [&](Vertex* start_pr) {
			Edge *other_edge;
			if(getNextOpenEdge(start_pr, other_edge)) {
				Vertex* vr = other_edge->vertices(1);
				Vector2i pix = project2i(other_edge->vertices(1)->p);
				//TODO: check if the next point really belongs to the newly added geometry
				Meshlet *meshlet = new_seg_pm.at<Meshlet*>(pix[1], pix[0]);
				if(meshlet == vr->meshlet) {
					//if the other side of the stitch is not just a single vertex
					#ifdef SHOW_TEXT_STITCHING
					cout << "starting sewing process" << endl;
					#endif // SHOW_TEXT_STITCHING
					//Starting sewing process!!!!!
					sewing_mode         = true;
					current_sewing_edge = other_edge;
					last_sewing_p       = current_sewing_edge->vertices(0);
					current_sewing_p    = current_sewing_edge->vertices(1);
					current_sewing_pix  = project2i(current_sewing_p->p);
				}
			}
		};

		for(int j = 0; j < borders[i].size(); j++) {
			#ifdef SHOW_TEXT_STITCHING
			cout << "starting new edge" << endl;
			#endif // SHOW_TEXT_STITCHING

			Edge &edge = borders[i][j];

			if(edge.triangle->edges[edge.pos] != &edge) { //Consistency seems violated
				assert(0);
			}
			if(edge.already_used_for_stitch) {
				assert(0);//shouldn't every edge be touched only once?
			}

			Vertex* pr0 = edge.vertices(1);//these are mixed because we screwed up the order at one point
			Vector4f P0         = pr0->p;
			Vector4f point0     = view_inv * P0;//transform the point into camera space
			Vector4f projected0 = p_v * P0;
			float w0 = projected0[3];
			Vector2f pix0  = Vector2f(projected0[0] / w0, projected0[1] / w0);
			Vector2i pix0i = Vector2i(    round(pix0[0]),      round(pix0[1]));

			Vertex* pr1 = edge.vertices(0);//these are mixed because we screwed up the order at one point
			Vector4f P1         = pr1->p;
			Vector4f point1     = view_inv * P1;
			Vector4f projected1 = p_v * P1;
			float w1 = projected1[3];
			Vector2f pix1  = Vector2f(projected1[0] / w1, projected1[1] / w1);
			Vector2i pix1i = Vector2i(    round(pix1[0]),      round(pix1[1]));

			Vertex* pr2 = edge.oppositePoint();//we need the third point of the triangle here!
			Vector4f P2         = pr2->p;
			Vector4f point2     = view_inv * P2;
			Vector4f projected2 = p_v * P2;
			float w2 = projected2[3];
			Vector2f pix2  = Vector2f(projected2[0] / w2, projected2[1] / w2);
			Vector2i pix2i = Vector2i(    round(pix2[0]),      round(pix2[1]));

			Vector2f first_pix;

			// skip triangle if it is not facing camera. (backfacing triangle)

			bool very_first_edge_made = false;//TODO: does this belong one level up?
			Edge very_first_edge;
			bool edge_made = false;
			Edge last_edge;

			bool first_point = true;
			int nr_triangles_this_edge = 0;

			//function that starts stitching on a per pixel basis
			auto func = [&](int x, int y, float t) {
				//TODO: guarantee that lastPix always is set!!!!!!
				if(sewing_mode) {
					Vector4f frag_pos = interpolatePerspectific(point0, point1, w0, w1, t);
					float depth_interpolated = frag_pos[2];
					bool sewing_edge_made = false;

					if(x == last_pix_i[0] && y == last_pix_i[1]) {
						#ifdef SHOW_TEXT_STITCHING
						cout << "this pixel was last pixel" << endl;
						#endif // SHOW_TEXT_STITCHING
						//we want at least one pixel to change
						//if this is the first pixel of this edge then we want to give triangle creation another chance:
						if(!edge_made) {
							if(!isTrianglePossible({last_sewing_p, pr0, pr1})) {
								sewing_mode = false;
								return;
							}
							map->addTriangle_(last_sewing_p, pr0, pr1, 21);
							nr_triangles_this_edge++;
							#ifdef SHOW_TEXT_STITCHING
							cout << "e" << endl;
							#endif // SHOW_TEXT_STITCHING
                            last_p          = last_sewing_p;
							edge_made        = true;
							sewing_edge_made = true;
						}
						//maybe we also need other stuff
						//at least we skip this pixel
						return;
					}
					if(abs(x - current_sewing_pix[0]) > 1 || 
					   abs(y - current_sewing_pix[1]) > 1) {
						//if the sewing pix is not within the a one pixel radius
						//we should wait for another bresenham iteration?

						//TODO: also check for distance > threshold!!!!!!
						sewing_mode = false;
						#ifdef SHOW_TEXT_STITCHING
						cout << "Exiting sewing mode since pixel are deviating too much" << endl;
						#endif // SHOW_TEXT_STITCHING
						//TODO: should there be a way to continue sewing for a little while?


						Vector2i p1 = project2i(last_sewing_p->p);
						Vector2i p2 = project2i(current_sewing_p->p);
						#ifdef SHOW_WINDOW_STITCHING
						debug_color_coded_new_segmentation.at<cv::Vec4b>(p1[1], p1[0]) = 
								cv::Vec4b(0, 0, 255, 0);
						debug_color_coded_new_segmentation.at<cv::Vec4b>(p2[1], p2[0]) = 
								cv::Vec4b(255, 0, 255, 0);
						cv::imshow("stitch", debug_color_coded_new_segmentation);
						cv::waitKey(1);
						#endif // SHOW_WINDOW_STITCHING
						//but this means we should run the logic coming afterwards.
					} else {
						//TODO:create one or two triangles depending on if the current edge already has one
						if(edge_made) {
							//the problem is that edges on both sides do not have the same length! Two cases:
							//1) too slow: each step we will try to add another triangle to the stitch by progressing on the oppositing edge
							//2) too fast: wait till the bresenham progresses one pixel
							//but when to break up?
							while(true) {
								if(current_sewing_edge->vertices(1)->encompassed()) {
									sewing_mode = false;
									break;
								}
								Edge* other_edge;
								current_sewing_edge->getOrAttachNextEdge(1, other_edge, additional_edges, false, 20);
								other_edge->registerInTriangle();
								Vertex*		    vr = other_edge->vertices(1);
								Vertex*          v = vr; //DEBUG: combine these two
								Vector2i       pix = project2i(v->p);
								//check if the pixel of the next edge point really belongs to new geometry:
								Meshlet *patch = new_seg_pm.at<Meshlet*>(pix[1], pix[0]);
								if(patch != vr->meshlet) {
									#ifdef SHOW_TEXT_STITCHING
									cout << "quitting sewing because the next pixel would lie on wrong side" << endl;
									#endif // SHOW_TEXT_STITCHING
									sewing_mode = false;
									break;
								}

								float new_depth = new_geom_m.at<cv::Vec4f>(pix[1], pix[0])[2]; //getting depth
								float depth_threshold = 0.05f;//TODO: change this to something depth (standard deviation dependant)
								if(abs(new_depth - depth_interpolated) > depth_threshold) {
									sewing_mode = false;
									break;
								}
								if(!isTrianglePossible({current_sewing_p, last_sewing_p,pr1})) {
									sewing_mode = false;
									break;
								}
								map->addTriangle_(current_sewing_p, last_sewing_p, pr1, 22);
								nr_triangles_this_edge++;
                                last_p = current_sewing_p;
								#ifdef SHOW_TEXT_STITCHING
								cout << "f2" << endl;
								#endif

								edge_made = true;
								current_sewing_edge = other_edge;
								last_sewing_p       = current_sewing_p;
								current_sewing_p    = current_sewing_edge->vertices(1);
								current_sewing_pix  = project2i(current_sewing_p->p);
								if(abs(pix[0] - x) > 1 || abs(pix[1] - y) > 1) {
									//boo yeah! create another triangle!!!
									#ifdef SHOW_TEXT_STITCHING
									cout << "the next pixel (" << pix[0] << ", " << pix[1] << 
									        ") is too far away. wait for the next bresenham step" << endl;
									#endif // SHOW_TEXT_STITCHING
									break;
								}
							}
						} else {
							//multiple new triangles:
							if(!isTrianglePossible({last_sewing_p, pr0, pr1})) {
								sewing_mode = false;
								return;
							}
							map->addTriangle_(last_sewing_p, pr0, pr1, 23);
							nr_triangles_this_edge++;
                            last_p = last_sewing_p;
							#ifdef SHOW_TEXT_STITCHING
							cout << "g1" << endl;
							#endif
							Vector2i p1 = project2i(last_sewing_p->p);
							Vector2i p2 = project2i(current_sewing_p->p);

							while(true) {
								if(!isTrianglePossible({current_sewing_p, last_sewing_p, pr1})) {
									sewing_mode = false;
									return;
								}
								//TODO: remove ptr to debug triangle
								Triangle* debug_new_triangle =
								        map->addTriangle_(current_sewing_p, last_sewing_p, pr1, 24);

								//TODO: also remove this debug measure:
								if( debug_new_triangle->neighbours[0].ptr != nullptr &&
								    debug_new_triangle->neighbours[1].ptr != nullptr &&
								    debug_new_triangle->neighbours[2].ptr != nullptr){

								    cout << "DEBUG: new triangle totally encompassed" << endl;
								    //assert(0); //TODO: check if the next current sewing point really is not encompassed
								    Vertex* debug_vert = current_sewing_edge->vertices(1);
								    Vertex* debug_vert2 = current_sewing_edge->vertices(0);
								    bool debug1 = debug_vert->encompassed();
                                    bool debug2 = current_sewing_p->encompassed();
                                    //TODO: find out how both debugs are False and getting the next edge fails!!!
                                    Edge* other_edge;
                                    current_sewing_edge->getOrAttachNextEdge(1, other_edge, additional_edges, false, 21);

                                    //maybe this is a valid solution for this case
                                    //NOPE IT ISN't! probably we should rather abort
                                    sewing_mode=false;
								    break;
								}
                                last_p = current_sewing_p;
								#ifdef SHOW_TEXT_STITCHING
								cout << "g2" << endl;
								#endif
								edge_made = true;
								if(current_sewing_p->encompassed()) {
									sewing_mode = false;
									break;
								}
								Edge* other_edge;
								current_sewing_edge->getOrAttachNextEdge(1, other_edge, additional_edges, false, 21);
								other_edge->registerInTriangle();
								Vertex* vr = other_edge->vertices(1);
								Vertex          *v = vr;
								Vector2i       pix = project2i(v->p);
								if(isConnected(pr1, vr)) {
									if(!isOpenEdge(pr1, vr)) {
										sewing_mode = false;
										break;
									}
								}
								Meshlet *patch = new_seg_pm.at<Meshlet*>(pix[1], pix[0]);
								if(patch != vr->meshlet) {
									#ifdef SHOW_TEXT_STITCHING
									cout << "quitting sewing because the next pixel would lie on wrong side" << endl;
									#endif
									sewing_mode = false;
									break;
								}
								//TODO: this could potentially happen at the end of a loop
								if(isConnected(current_sewing_p, pr1)) {
									if(!isOpenEdge(current_sewing_p, pr1)) {
										#ifdef SHOW_TEXT_STITCHING
										cout << "quitting sewing because of pr1 and currentSewingPr are closed" << endl;
										#endif
										sewing_mode = false;
										break;
									}
								}
								float new_depth = new_geom_m.at<cv::Vec4f>(pix[1], pix[0])[2]; //getting depth
								float depth_threshold = 0.05f;//TODO: change this to something depth (standard deviation dependant)
								if(abs(new_depth - depth_interpolated) > depth_threshold) {
									#ifdef SHOW_TEXT_STITCHING
									cout << "quitting sewing because of bigger depth step" << endl;
									#endif
									sewing_mode = false;
									break;
								}

								current_sewing_edge = other_edge;
								last_sewing_p = current_sewing_p;
								current_sewing_p = current_sewing_edge->vertices(1);
								current_sewing_pix = project2i(current_sewing_p->p);
								if(abs(pix[0] - x) > 1 || abs(pix[1] - y) > 1) {
									//boo yeah! create another triangle!!!
									#ifdef SHOW_TEXT_STITCHING
									cout << "the next pixel (" << pix[0] << ", " << pix[1] << 
									        ") is too far away. wait for the next bresenham step" << endl;
									#endif
									break;
								}
							}
							if(current_sewing_p->encompassed()) {
								sewing_mode = false;
								return;
							}
							//get next edge
							edge.already_used_for_stitch = true;
							Edge* other_edge;
							current_sewing_edge->getOrAttachNextEdge(1, other_edge, additional_edges, false, 22);
							other_edge->registerInTriangle();
							current_sewing_edge = other_edge;
							last_sewing_p = current_sewing_p;
							current_sewing_p = current_sewing_edge->vertices(1);
							current_sewing_pix = project2i(current_sewing_p->p);
							edge_made = true;
						}
					}
					//TODO: check if z-distance is within thresholds
				}
				//when in sewing mode we don't need to check for pixelwise neighbourhood:
				if(sewing_mode) {
					return;
				}
				//TODO: think why this block is necessary (and document it)
				if(!very_first_point && first_point) {
					first_point = false;
					//return;//make this whole block useless
				}
				very_first_point = false;

				if(x < 0 || y < 0 || x >= width || y >= height) {
					return;
				}
				#ifdef SHOW_WINDOW_STITCHING
				debug_color_coded_new_segmentation.at<cv::Vec4b>(y, x) = 
						cv::Vec4b(255,255,255,0);
				cv::imshow("stitch",debug_color_coded_new_segmentation);
				cv::waitKey(1);
				#endif // SHOW_WINDOW_STITCHING

				Vector2i neighbours[] = {
						Vector2i(x - 1,     y), Vector2i(x + 1,     y), 
						Vector2i(    x, y - 1), Vector2i(    x, y + 1), // 4 connectivity
						Vector2i(x - 1, y - 1), Vector2i(x + 1, y - 1), 
						Vector2i(x - 1, y + 1), Vector2i(x + 1, y + 1) // 8 connectivity.
				};

				Vector4f geom_proj = geom_proj_m.at<Vector4f>(y, x);

				float depth_sensor = new_geom_m.at<cv::Vec4f>(y, x)[2];
				float depth_projected = geom_proj[2];
				Vector4f frag_pos = interpolatePerspectific(point0, point1, w0, w1, t);
				float depth_interpolated = frag_pos[2];

				// Sort the neighoburing points depending on if there has been a triangle in this patch
				if(!edge_made) {
					Vector2f end = pix1;
					auto compare = [end](Vector2i l, Vector2i r) {
						float dist1 = (Vector2f(l[0], l[1]) - end).norm();
						float dist2 = (Vector2f(r[0], r[1]) - end).norm();
						return dist1 > dist2;
					};
					int n = sizeof(neighbours) / sizeof(neighbours[0]);
					sort(neighbours, neighbours + n, compare);
				} else {
					Vector2f end = pix0;
					auto compare = [end](Vector2i l, Vector2i r) {
						float dist1 = (Vector2f(l[0], l[1]) - end).norm();
						float dist2 = (Vector2f(r[0], r[1]) - end).norm();
						return dist1 < dist2;
					};
					int n = sizeof(neighbours) / sizeof(neighbours[0]);
					sort(neighbours, neighbours + n, compare);
				}

				for(Vector2i &neighbour : neighbours) {
					//iterate over all neighbours
					int xn = neighbour[0];
					int yn = neighbour[1];
					if(xn < 0 || yn < 0 || xn >= width || yn >= height) {
						continue;
					}

					//check if the pixel exists within the newly added points
					Meshlet *patch = new_seg_pm.at<Meshlet*>(yn, xn);
					int index = new_pt_ind_m.at<int>(yn, xn);
					if(patch == nullptr) {
						continue;
					}

					//check if depth threshold is not exceeded
					float new_depth = new_geom_m.at<cv::Vec4f>(yn, xn)[2]; //getting depth
					float depth_threshold = 0.05f;//TODO: change this to something depth (standard deviation dependant)
					if(abs(new_depth-depth_interpolated) > depth_threshold) {
						//don't create new geometry
						continue;
					}

					//check if we are at the right side of the old triangle
					Vector2f new_pix(xn, yn);
					if(isOnSameSide(pix0, pix1, pix2, new_pix)) {
						continue; //no triangle here
					}

					//if there already is a stitch we also want to check if we are on the right side of the stitching triangle
					if(edge_made) {
						if(isOnSameSide(pix1, last_pix, pix0, new_pix)) {
							continue;
						}
					}

					//check if the new pixel is equal to the very first connected one!
					if(very_first_edge_made) {
						if(new_pix[0] == first_pix[0] && new_pix[1] == first_pix[1]) {
							//actually this might mean that we want to close up the stitch
							continue;//TODO: we might even want to end the process of stitching here entirely
						}
					}

					Vertex* this_pr = &patch->vertices[index];
					//this_pr.set(patch, index);
					//Check if this point is already completely encapsulated by triangles (stitching would then be illegal)
					if(this_pr->encompassed()) {
						continue;
					}

					debug_color_coded_new_segmentation.at<cv::Vec4b>(yn, xn) = 
							cv::Vec4b(0, 0, 255, 0);

					//TODO: also check the orientation of the newly generated triangles

					if(!edge_made) {
						if(!triangle_created_last_edge) {
							if(!isTrianglePossible({this_pr, pr0, pr1})) {
								continue;
							}

							checkAndStartSewing(this_pr);

							map->addTriangle_(this_pr, pr0, pr1, 25);
							#ifdef SHOW_TEXT_STITCHING
							cout << "a" << endl;
							#endif // SHOW_TEXT_STITCHING

							if(!very_first_edge_made) {
								very_first_edge_made = true;
								first_p  = this_pr;
								first_pix = new_pix;
							}

							//return because we want to go in sewing mode
							if(sewing_mode) {
								edge_made = true;

                                last_p  = this_pr;
								last_pix = new_pix;
								return;
							}
						} else {
							//No triangle connected to this edge yet.
							//create a triangle that incorporates the last made edge
							if(this_pr == last_p) {
								if(!isTrianglePossible({this_pr, pr0, pr1})) {
									continue;
								}
								checkAndStartSewing(this_pr);

								map->addTriangle_(this_pr, pr0, pr1, 26);
								#ifdef SHOW_TEXT_STITCHING
								cout << "b" << endl;
								#endif // SHOW_TEXT_STITCHING

							} else {
								if(!isTrianglePossible({last_p, pr0, pr1})) {
									continue;
								}

								Edge* other_edge;
								if(getNextOpenEdge(this_pr, other_edge)) {
									Vertex* vr = other_edge->vertices(1);
									Vector2i pix = project2i(other_edge->vertices(1)->p);
									//TODO: check if the next point really belongs to the newly added geometry
									Meshlet *patch = new_seg_pm.at<Meshlet*>(pix[1], pix[0]);
									if(patch == vr->meshlet) {
										//if the other side of the stitch is not just a single vertex
										#ifdef SHOW_TEXT_STITCHING
										cout << "starting sewing process" << endl;
										#endif // SHOW_TEXT_STITCHING
										//Starting sewing process!!!!!
										sewing_mode         = true;
										current_sewing_edge = other_edge;
										last_sewing_p       = current_sewing_edge->vertices(0);
										current_sewing_p    = current_sewing_edge->vertices(1);
										current_sewing_pix  = project2i(current_sewing_p->p);
									}
								}

								map->addTriangle_(last_p, pr0, pr1, 27);//TODO: reinsert this
                #ifdef SHOW_TEXT_STITCHING
								cout << "c" << endl;
                #endif // SHOW_TEXT_STITCHING
								//TODO: reinsert this!!!!!

								nr_triangles_this_edge++; // just so veryFirstEdgeMade will be set
								edge_made = true;

								continue;

								bool need_to_connect_hole = false;

								//TODO: whenever we have a lastPr and a thisPr to create a triangle we have to check if
								//the orientation is correct

								if(!isTrianglePossible({this_pr, last_p, pr1})) {
									continue;
								}
								map->addTriangle_(this_pr, last_p, pr1, 28);
								#ifdef SHOW_TEXT_STITCHING
								cout << "c" << endl;
								#endif // SHOW_TEXT_STITCHING
								if(need_to_connect_hole) {
									//TODO: check if we can do another triangle:
									cout << "fill hole!!!!" << endl;
								}
							}
						}
                        last_p   = this_pr;
						last_pix  = new_pix;
						edge_made = true;

					} else {
						//we continue stitching if this edge already has one stitch.
						//TODO: remove this:
						//continue;//debug
						bool need_to_connect_hole = false;

						if(this_pr == last_p) {
							continue;
						}

						//TODO: whenever we have a lastPr and a thisPr to create a triangle we have to check if
						//the orientation is correct
						//don't be confused this is only running if you delete the continue further up
						checkAndStartSewing(this_pr);

						if(!isTrianglePossible({this_pr, last_p, pr1})) {
							continue;
						}

						if(!isConnected(last_p, this_pr)) {
							need_to_connect_hole = true;
							//TODO: check if the hole is one or two hops long.
						}
						map->addTriangle_(this_pr, last_p, pr1, 29);//third point is from last edge
						#ifdef SHOW_TEXT_STITCHING
						cout << "d" << endl;
						#endif // SHOW_TEXT_STITCHING

						if(need_to_connect_hole) {
							#ifdef SHOW_TEXT_STITCHING
							cout << "holefix" << endl;
							#endif // SHOW_TEXT_STITCHING
						}
                        last_p  = this_pr;
						last_pix = new_pix;
						if(!edge_made) {
							assert(0);
						}
					}
					nr_triangles_this_edge++;
				}

				debug_color_coded_new_segmentation.at<cv::Vec4b>(debug_last_pix[1],
				                                                 debug_last_pix[0]) =
						cv::Vec4b(0, 255, 0, 0);
				debug_last_pix = Vector2i(x, y);
			};

			auto func1 = [&](int x, int y, float t) {
				//same as func1 but with proper setup
				func(x, y, t);

				last_pix_i[0] = x;
				last_pix_i[1] = y;
			};
			//the bresenham algorithm always needs to start with pixel 0 and go to pixel 1
			bresenham(pix0, pix1, func1);

			triangle_created_last_edge = nr_triangles_this_edge > 0;

			edge.already_used_for_stitch = true;
		}
	}
	border_list.push_back(std::move(additional_edges));
}



void MeshStitcher::stitchOnBorders2(Matrix4f view, Matrix4f proj,
                                   cv::Mat std_proj, cv::Mat geom_proj_m, cv::Mat new_geom_m, cv::Mat new_std,
                                   cv::Mat debug_color_coded_new_segmentation, cv::Mat new_seg_pm,
                                   cv::Mat new_pt_ind_m) {

    Vector2i relative_neighbours[] = {
            Vector2i(-1, 0), Vector2i( +1, 0),
            Vector2i( 0, -1), Vector2i(  0, +1), // 4 connectivity
            Vector2i( -1, -1), Vector2i( +1, -1),
            Vector2i( -1, +1), Vector2i( +1, +1) // 8 connectivity.
    };
    Matrix4f view_inv = view.inverse();
    Matrix4f p_v      = proj * view_inv;

    int width  = geom_proj_m.cols;
    int height = geom_proj_m.rows;


    vector<Edge> additional_edges;

    for(size_t i = 0; i < border_list.size(); i++) {
        vector<Edge> &border = border_list[i];
        //cout << "starting new border" << endl;
        // store the last pixel to eventually skip when going from one edge to the next


        Vector2i last_pix_i(-1000,-1000);
        bool sewing_mode = false; //start off with finding pixel in adjacency
        Vector2i last_pix_connected;
        int debug_size = border.size();
        for(size_t j = 0; j < border.size(); j++) {
            Edge &edge = border[j];
            bool connected_nbs[4] = {false, false, false, false};

#ifdef SHOW_TEXT_STITCHING
            //cout << "starting new edge" << endl;
#endif // SHOW_TEXT_STITCHING

            //TODO: remove these debug measures at some point
            if(edge.triangle->edges[edge.pos] != &edge) { //Consistency seems violated
                assert(0);
            }
            if(edge.already_used_for_stitch) {
                assert(0);//shouldn't every edge be touched only once?
            }

            //TODO: reduce this block as much as possible:
            Vertex* v0 = edge.vertices(1);//*1) these are mixed because we screwed up the order at one point
            Vector4f point0     = view_inv * v0->p;
            Vector4f projected0 = p_v * v0->p;
            float w0 = projected0[3];
            Vector2f p0  = Vector2f(projected0[0] / w0, projected0[1] / w0);
            Vector2i p0i = Vector2i(    round(p0[0]),      round(p0[1]));

            Vertex* v1 = edge.vertices(0);//*2) these are mixed because we screwed up the order at one point
            //Vector4f P1         = pr1->p;
            Vector4f point1     = view_inv * v1->p;
            Vector4f projected1 = p_v * v1->p;
            float w1 = projected1[3];
            Vector2f p1  = Vector2f(projected1[0] / w1, projected1[1] / w1);
            Vector2i p1i = Vector2i(    round(p1[0]),      round(p1[1]));

            //The opposite point of the triangle here
            Vertex* v2 = edge.oppositePoint();
            Vector2f p2 = project2f(p_v, view_inv,v2->p);
            //Vector4f P2         = pr2->p;
            //Vector4f point2     = view_inv * P2;
            //Vector4f projected2 = p_v * P2;
            //float w2 = projected2[3];
            //Vector2f pix2  = Vector2f(projected2[0] / w2, projected2[1] / w2);
            //Vector2i pix2i = Vector2i(    round(pix2[0]),      round(pix2[1]));

            Vector2f first_pix;
            bool current_edge_triangulated = false;

            // skip triangle if it is not facing camera. (backfacing triangle)

            bool very_first_edge_made = false;//TODO: does this belong one level up?
            Edge very_first_edge;
            bool edge_made = false;
            Edge last_edge;

            bool first_point = true;
            int nr_triangles_this_edge = 0;

            //function that starts stitching on a per pixel basis
            auto func = [&](int x, int y, float t) {
                Vector4f frag_pos = interpolatePerspectific(point0, point1, w0, w1, t);
                float frag_z = frag_pos[2];
                //cout << x << " " << y << endl;
                if(x != last_pix_i[0] || y != last_pix_i[1]){
                    connected_nbs[0] = connected_nbs[1] =connected_nbs[2] =connected_nbs[3] = false;
                    //return;
                }

                if(!edge_made){
                    Vector2i neighbours[4];
                    for(size_t k : {0, 1, 2, 3})
                        neighbours[k] = Vector2i(x, y) + relative_neighbours[k];

                    for(size_t k: {0, 1, 2, 3}){
                        Vector2i nb = Vector2i(x + relative_neighbours[k][0], y + relative_neighbours[k][1]);
                        //check if neighbour is within border
                        if(!is_in_bound(nb, width, height))
                            continue;
                        //check if neighbour is on right side:
                        if(on_same_sides(p0, p1, p2, nb.cast<float>()))
                            continue;

                        //check if neighbour is a new vertex.
                        Meshlet *meshlet = new_seg_pm.at<Meshlet*>(nb[1], nb[0]);
                        int index = new_pt_ind_m.at<int>(nb[1], nb[0]);
                        if(meshlet == nullptr)
                            continue;
                        Vertex* vert = &(meshlet->vertices[index]);


                        float new_depth = new_geom_m.at<cv::Vec4f>(nb[1], nb[0])[2]; //getting depth
                        float depth_threshold = 0.05f;//TODO: change this to something depth (standard deviation dependant)
                        if(abs(new_depth-frag_z) > depth_threshold) {
                            //don't create new geometry
                            continue;
                        }

                        if(vert->encompassed())
                            continue;
                        if(v0->count_connecting_tringles(vert) == 2)
                            continue;
                        if(v1->count_connecting_tringles(vert) == 2)
                            continue;
                        if(v0->count_connecting_tringles(v1) == 2)
                        {
                            v0->count_connecting_tringles(v1);
                            assert(0);
                        }
                        bool debug1before = vert->encompassed();
                        bool debug2before = v0->encompassed();
                        bool debug3before = v1->encompassed();
                        if(debug1before || debug2before || debug3before){
                            //assert(0);
                        }
                        Triangle* new_triangle =
                                mesh_reconstruction->addTriangle_(vert, v0, v1, 21);
                        if(!new_triangle->manifold_valid() || !new_triangle->orientation_valid()){
                            //mesh_reconstruction->checkNeighbourhoodConsistency();//TODO: REMOVE DEBUG
                            //if the triangle violates the manifold we immediately remove it again.
                            Triangle* before = &new_triangle->getMeshlet()->triangles[0];
                            new_triangle->getMeshlet()->triangles.pop_back();
                            Triangle* after = &new_triangle->getMeshlet()->triangles[0];
                            //TODO: remove the following debug output
                            /*
                            if(before != after)
                                cout << "ptr to vector before: " << before << " and after " << after << endl;
                            cout << "ptr to new triangle " << new_triangle << endl;
                            */
                            //mesh_reconstruction->checkNeighbourhoodConsistency();//TODO: REMOVE DEBUG
                            //TODO: remove the created meshlet neighbourhood if necessary.
                            //storing which neighbourhood got created is probably more involved

                            continue;
                        }

                        assert(new_triangle->orientation_valid());
                        //cout << "it not always has the wrong orientation" << endl;
                        bool debug1 = !new_triangle->vertices[0]->manifold_valid();
                        bool debug2 = !new_triangle->vertices[1]->manifold_valid();
                        bool debug3 = !new_triangle->vertices[2]->manifold_valid();
                        if(debug1 || debug2 || debug3){
                            new_triangle->manifold_valid(0);
                            new_triangle->manifold_valid(1);
                            new_triangle->manifold_valid(2);
                            new_triangle->vertices[0]->manifold_valid();
                            new_triangle->vertices[1]->manifold_valid();
                            new_triangle->vertices[2]->manifold_valid();
                            assert(0);
                        }

                        connected_nbs[k] = true;
                        //TODO: check if triangle is valid. Remove if not
                        //sew in opposite direction:


                        edge_made = true; //maybe also enable stitching mode ?
                        edge.already_used_for_stitch = true;
                        last_pix_connected = nb;
                        //return;
                        //now try to further sew counter the direction of where we went.
                        sewLocally(Vector2i(x, y), frag_z,
                                nullptr, //Last new pix.... whatever i thought this will be
                                v1,v0,vert,
                                p1, p0, nb.cast<float>(),
                                new_seg_pm, new_pt_ind_m, new_geom_m, connected_nbs, true);


                        //now we go in forward direction....
                        for(size_t l : {0, 1, 2, 3}){
                            //disable any potential neighbour on the wrong side
                            Vector2f nb_chck(x + relative_neighbours[l][0], y + relative_neighbours[l][1]);
                            if(on_same_sides(p0, p1, p2, nb_chck))
                                connected_nbs[l] = false;
                        }
                        sewLocally(Vector2i(x, y), frag_z,
                                   &last_pix_connected, //Last new pix.... whatever i thought this will be
                                   v0,v1,vert,
                                   p0, p1, nb.cast<float>(),
                                   new_seg_pm, new_pt_ind_m, new_geom_m, connected_nbs, false);

                        //all the possible triangles are created... we are safe now
                        return;

                        //TODO:
                        //TODO: second,
                        //TODO: third,
                        //TODO: four, generate a triangle.
                        //TODO: sew the counter direction side
                        //TODO: sew the main direction side.
                        //TODO: move next pixel/ edge/ etc
                    }


                }else{
                    //sewing mode. This one is even uglier than the one above
                    Meshlet* meshlet = new_seg_pm.at<Meshlet*>(last_pix_connected[1], last_pix_connected[0]);
                    int ind = new_pt_ind_m.at<int>(last_pix_connected[1], last_pix_connected[0]);
                    Vertex* vert = &(meshlet->vertices[ind]);
                    sewLocally(Vector2i(x, y), frag_z,
                               &last_pix_connected, //Last new pix.... whatever i thought this will be
                               v0,v1,vert,
                               p0, p1, last_pix_connected.cast<float>(),
                               new_seg_pm, new_pt_ind_m, new_geom_m, connected_nbs, false, true);

                }


                if(!is_in_bound(x, y, width, height)) {
                    return;
                }

                //mesh_reconstruction->checkNeighbourhoodConsistency();//TODO: REMOVE DEBUG
            };




            auto func1 = [&](int x, int y, float t) {
                //same as func1 but with proper setup
                func(x, y, t);
                last_pix_i = Vector2i(x,y);
            };
            bresenham(p0, p1, func1);
        }
       // mesh_reconstruction->checkNeighbourhoodConsistency();//TODO: REMOVE DEBUG

    }
    //mesh_reconstruction->checkNeighbourhoodConsistency();//TODO: REMOVE DEBUG
    border_list.push_back(std::move(additional_edges));

    //mesh_reconstruction->checkNeighbourhoodConsistency();//TODO: REMOVE DEBUG
}



void MeshStitcher::sewLocally(Vector2i center_pix, float center_pix_z, Vector2i* last_new_pix,
                              Vertex* vfar, Vertex* v0, Vertex* v1,
                              Vector2f pfar, Vector2f p0, Vector2f p1,
                              const cv::Mat &new_meshlets, const cv::Mat &new_vert_inds, const cv::Mat &new_geom,
                              bool nbs_used[4],
                              bool flip, bool debug){
    Vector2i relative_neighbours[] = {
            Vector2i(-1, 0), Vector2i( +1, 0),
            Vector2i( 0, -1), Vector2i(  0, +1)
    };

    int width = new_meshlets.cols;
    int height = new_meshlets.rows;
    float closest_dist = 1000.0f;
    int ind_closest = -1;
    Vertex* closest_vertex = nullptr;
    for(size_t i: {0, 1, 2, 3}){
        if(nbs_used[i])
            continue;
        Vector2i nb_pos = center_pix + relative_neighbours[i];
        if(!is_in_bound(nb_pos, width, height))
            continue;
        if(on_same_sides(p0, p1, pfar, nb_pos.cast<float>()))
            continue;
        Meshlet* meshlet = new_meshlets.at<Meshlet*>(nb_pos[1],nb_pos[0]);
        int ind = new_vert_inds.at<int>(nb_pos[1],nb_pos[0]);
        if(meshlet == nullptr)
            continue;

        float new_depth = new_geom.at<cv::Vec4f>(nb_pos[1], nb_pos[0])[2]; //getting depth
        float depth_threshold = 0.05f;//TODO: change this to something depth (standard deviation dependant)
        if(abs(new_depth - center_pix_z) > depth_threshold) {
            //don't create new geometry
            continue;
        }

        Vertex* vert = &(meshlet->vertices[ind]);


        float dist = dist_sqr(p1.cast<float>(), nb_pos.cast<float>());
        if(dist < closest_dist) {
            closest_dist = dist;
            ind_closest = i;
            closest_vertex = vert;
        }
    }
    if(ind_closest == -1){
        return;
    }

    if(closest_vertex->encompassed())
        return;
    if(v0->encompassed())
        return;
    if(v1->encompassed())
        return;
    if(closest_vertex->count_connecting_tringles(v0) == 2)
        return;
    if(closest_vertex->count_connecting_tringles(v1) == 2)
        return;
    if(v0->count_connecting_tringles(v1) == 2)
        return;
        //assert(0);



    Triangle* tri = nullptr;
    if(flip){
        tri = mesh_reconstruction->addTriangle_(closest_vertex,v0,v1,22);
    }else{
        tri = mesh_reconstruction->addTriangle_(closest_vertex,v1,v0,23);
    }
    assert(tri->orientation_valid(1));
    if(!tri->manifold_valid() || !tri->orientation_valid()){
        //cout << "undoing triangle due to manifold or orientation issues " <<
        //        !tri->manifold_valid() << " " << !tri->orientation_valid() << endl;
        if(debug && !tri->orientation_valid(1)){
            bool debug0 = tri->orientation_valid(0);
            bool debug1 = tri->orientation_valid(1);
            bool debug2 = tri->orientation_valid(2);
           // assert(0);
        }
        tri->getMeshlet()->triangles.pop_back();
        return;
    }
    //cout << "keeping triangle" << endl;
    nbs_used[ind_closest] = true;
    bool debug1 = !tri->vertices[0]->manifold_valid();
    bool debug2 = !tri->vertices[1]->manifold_valid();
    bool debug3 = !tri->vertices[2]->manifold_valid();
    if(debug1 || debug2 || debug3){
        assert(0);
    }
    if(last_new_pix != nullptr){
        *last_new_pix = center_pix + relative_neighbours[ind_closest];
    }
    sewLocally( center_pix,center_pix_z, last_new_pix,
                v1, v0, closest_vertex,
                p1, p0 , (center_pix + relative_neighbours[ind_closest]).cast<float>(),
                new_meshlets, new_vert_inds, new_geom,
                nbs_used,
                flip);

}