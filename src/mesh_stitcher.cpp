#include <mesh_stitcher.h>

#include <Eigen/Eigen>

#include <mesh_reconstruction.h>
#include <stitching_utils.h>
#include <cuda/coalesced_memory_transfer.h>

using namespace std;
using namespace Eigen;

void MeshStitcher::rasterBorderGeometry(vector<vector<Edge>> &borders,
										Matrix4f view, Matrix4f proj,
										cv::Mat geometry) {

	Matrix4f view_inv = view.inverse();
	cv::Mat debug = geometry.clone(); //TODO: get rid of this
	for(size_t i = 0; i < borders.size(); i++) {
		for (size_t j = 0; j < borders[i].size(); j++) {
			Edge *edge = &borders[i][j];
			rasterLineGeometry(view_inv, proj, edge, geometry, debug);
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
								 vector<vector<Edge>> &border_list,
								 Matrix4f debug_proj_pose) {
	float s = 2.0f;
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
		if(border_list.size() != 49) {
			return;
		}
		Vector4f p1 = edge.vertices(0)->p;
		p1 = debug_proj_pose * p1;
		p1 = p1 * (1.0f / p1[3]);
		Vector4f p2 = edge.vertices(1)->p;
		p2 = debug_proj_pose * p2;
		p2 = p2 * (1.0f / p2[3]);

		//Now draw a line via OpenCV
		cv::Point2i pcv1(p1[0] * s * 3.0f, p1[1] * s * 3.0f);
		cv::Point2i pcv2(p2[0] * s * 3.0f, p2[1] * s * 3.0f);

		cv::line(debug_mat, pcv1_old, pcv2_old, cv::Scalar(255, 0, 0));
		cv::line(debug_mat, pcv1, pcv2, cv::Scalar(0, 0, 255));
		pcv1_old = pcv1;
		pcv2_old = pcv2;
		cv::imshow("edges", debug_mat);
		cv::waitKey();
	};

	auto addBorderStrip = [&](Edge initial_edge) { 
		//get other edge obviously does not cut it
		Edge current_edge = initial_edge;
		renderEdge(current_edge);
		bool following_border = true;
		bool ran_in_circle = false;
		vector<Edge> forward = border_list.back();
		vector<Edge> backward;
		Edge debug_last_edge = current_edge;
		while(following_border) {
			current_edge.getOtherEdge(0, current_edge, border_list);
			renderEdge(current_edge); //debug

			if(meshlets.count(current_edge.triangle->getMeshlet()) != 1) {
				following_border = false;//if the new edge is attached to a patch outside we abort.
				continue;
			}

			if(current_edge.is_registered) {
				following_border = false; //we ran in circle
				if(!current_edge.equalTo(initial_edge)) {
					//one edge should never hit anoter edge within the same
					for(int i = 0; i < border_list.back().size(); i++) {
						if(current_edge.equalTo(border_list.back()[i])) {
							assert(0);//intersecting the current border
						}
					}
					debug_last_edge.getOtherEdge(0, current_edge, border_list);
					assert(0);//only the initial triangle should be registered
				} else {
					ran_in_circle = true;
				}
			} else {
				//TODO: remove registration
				//register the new edge:
				current_edge.registerInTriangle(border_list.size() - 1, 
				                                border_list.back().size());
				current_edge.debug = border_list.size() - 1;
				border_list.back().push_back(current_edge);
				forward.push_back(current_edge);
				if(current_edge.equalTo(initial_edge)) {
					assert(0);//the initial edge should be registered so we should never be in this branch
				}
			}
			debug_last_edge = current_edge;
		}

		if(!ran_in_circle) {
			//if we didn't follow the border in a full loop we start again from the initial triangle
			following_border = true;
			current_edge = initial_edge;
			while(following_border) {
				current_edge.getOtherEdge(1, current_edge, border_list);//other direction
				renderEdge(current_edge);
				if(meshlets.count(current_edge.triangle->getMeshlet()) != 1) {
					following_border = false;//if the new edge is attached to a patch outside we abort.
					continue;
				}
				if(current_edge.is_registered) {
					if(!current_edge.equalTo(initial_edge)) {
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
						Triangle *current_triangle = current_edge.triangle;
						Triangle *last_triangle    = debug_last_edge.triangle;
						Edge new_edge;
						debug_last_edge.getOtherEdge(1, new_edge, border_list);

						Edge new_edge2;
						debug_last_edge.getOtherEdge(1, new_edge2, border_list);

						assert(0);// this should not happen!!!!!
					} else {
						assert(0);
					}
				} else {
					//register the new edge:
					current_edge.registerInTriangle(border_list.size() - 1, 
					                                border_list.back().size());
					current_edge.debug = border_list.size() - 1;
					border_list.back().push_back(current_edge);
					backward.push_back(current_edge);

					if(current_edge.equalTo(initial_edge)) {
						assert(0);
					}
				}
			}

			reverse(backward.begin(), backward.end());
			backward.insert(backward.end(), forward.begin(), forward.end());
			//but why is this????
			/*
			if(backward[1610].triangle.get() == backward[2188].triangle.get() &&
				backward[1610].pos == backward[2188].pos){
				assert(0);//why are they the same? they should not be!!!!
			}
			 */
			border_list.back() = move(backward);
		}
		//changing the registration for the edges
		for(size_t i = 0; i < border_list.back().size(); i++) {
			border_list.back()[i].registerInTriangle(border_list.size() - 1, i);
		}
	};

	for(GeometryBase *geom_base : debug_patches_ordered) {
		for(size_t j = 0; j < geom_base->triangles.size(); j++) {
			Triangle &triangle = geom_base->triangles[j];
			Triangle* tri_ref = & triangle;
			if(!triangle.neighbours[0].valid()) {
				//TODO: also check for the according neighbour not to have a valid edge
				if(!triangle.edges[0].valid()) {
					//register the new edge:
					triangle.edges[0].border_ind = border_list.size();
					border_list.emplace_back();//create a new list of edges (read border) at its end
					triangle.edges[0].ind = border_list.back().size();//obviously
					border_list.back().emplace_back(tri_ref, 0);
					border_list.back().back().is_registered = true;
					border_list.back().back().debug = -border_list.size() + 1;
					Edge &edge = border_list.back().back();

					addBorderStrip(edge);//recursively append this edge!!!!
				}
			}

			if(!triangle.neighbours[1].valid()) {
				if(!triangle.edges[1].valid()) {
					//register the new edge:
					triangle.edges[1].border_ind = border_list.size();
					border_list.emplace_back();//create a new list of edges (read border) at its end
					triangle.edges[1].ind = border_list.back().size();
					border_list.back().emplace_back(tri_ref, 1);
					border_list.back().back().is_registered = true;
					border_list.back().back().debug = -border_list.size() + 1;
					Edge &edge = border_list.back().back();

					addBorderStrip(edge);//recursively append this edge!!!!
				}
				//TODO: also check for the according neighbour not to have a valid edge
			}
			if(!triangle.neighbours[2].valid()) {
				if(!triangle.edges[2].valid()) {
					//register the new edge:
					triangle.edges[2].border_ind = border_list.size();
					border_list.emplace_back();//create a new list of edges (read border) at its end
					triangle.edges[2].ind = border_list.back().size();
					border_list.back().emplace_back(tri_ref,2);
					border_list.back().back().is_registered = true;
					border_list.back().back().debug = -border_list.size() + 1;
					Edge &edge = border_list.back().back();

					addBorderStrip(edge);//recursively append this edge!!!!
				}
				//TODO: also check for the according neighbour not to have a valid edge
			}
		}
	}
}

void MeshStitcher::reloadBorderGeometry(vector<vector<Edge>> &border_list) {
	//i very much fear what happens when something here changes

	cout << "TODO: reimplement MeshStitcher::reloadBorderGeometry" << endl;
	return;
	assert(0);//Do this later... definitely part of the the active sets responsibilities!
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

//TODO: also download the geometry of such list
void MeshStitcher::freeBorderList(vector<vector<Edge>> &border_list) {
	for(size_t i = 0; i < border_list.size(); i++) {
		for(size_t j = 0; j < border_list[i].size(); j++) {
			Edge &edge = border_list[i][j];
			edge.unregister();
		}
	}
	border_list.clear();
}

void MeshStitcher::stitchOnBorders(
		vector<vector<Edge>> &borders, Matrix4f view, Matrix4f proj, 
		cv::Mat std_proj, cv::Mat geom_proj_m, cv::Mat new_geom_m, cv::Mat new_std,
		cv::Mat debug_color_coded_new_segmentation, cv::Mat new_seg_pm, 
		cv::Mat new_pt_ind_m, 
		vector<weak_ptr<GeometryBase>> &debug_list_new_edges) {
	cout << "IMPLEMENT THIS!!!!! MeshStitcher::stitchOnBorders" << endl;
	return;
	assert(0); // this needs to be here but reimplemented
	/*

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

	auto isConnected = [](VertexReference p1, VertexReference p2) {
		Vertex *v1 = p1.get();
		for(int i = 0; i < v1->triangles.size(); i++) {
			//iterating over all the triangles connected to vertex 1
			Triangle *tri = v1->triangles[i].triangle.get();
			for(int k = 0; k < 3; k++) {
				if(tri->points[k].isEqualTo(p2)) {
					return true;
				}
			}
		}
		return false;
	};

	auto isOpenEdge = [](VertexReference p1, VertexReference p2) {
		Vertex *v1 = p1.get();
		for(int i = 0; i < v1->triangles.size(); i++) {
			//iterating over all the triangles connected to vertex 1
			Triangle *tri = v1->triangles[i].triangle.get();
			for(int k = 0; k < 3; k++) {
				if(tri->points[k].isEqualTo(p2)) {
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

	auto isConnectedClosedEdge = [](VertexReference p1, VertexReference p2) {
		return false;
	};

	auto project3f = [&](Vector4f p) {
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

	auto getNextOpenEdge = [](VertexReference p, 
	                          Edge &resulting_unregistered_edge) {
		if(p.get()->encompassed()) {
			assert(0);
		}
		//To prevent this from outputting something of the existing geometry, this has to be called before stitching
		Vertex *v = p.get();
		for(int i = 0; i < v->triangles.size(); i++) {
			int ind = v->triangles[i].ind_in_triangle;
			//ind--; // decrement if we want to go to the last edge
			if(ind == -1) {
				ind = 2;
			}
			Triangle *triangle = v->triangles[i].triangle.get();
			if(!triangle->neighbours[ind].valid()) {
				Edge e;
				e.triangle = v->triangles[i].triangle;
				e.pos = ind;
				resulting_unregistered_edge = e;
				return true;
			}
		}
		return false;
	};

	auto isTrianglePossible = [&](array<VertexReference, 3> p,
	                              array<bool, 3> test = {true, true, true},
	                              array<bool, 3> test_orientation = {true, true, true}) {
		for(size_t i = 0; i < 3; i++) {
			size_t i2 = i == 2 ? 0 : i + 1;
			if(!test[i]) {
				continue;
			}
			if(isConnected(p[i], p[i2])) {
				if(!isOpenEdge(p[i], p[i2])) {
					return false;
				}
				if(!test_orientation[i]) {
					continue;
				}
				Vertex *this_vert = p[i].get();
				for(int l = 0; l < this_vert->triangles.size(); l++) {
					if(this_vert->triangles[l].triangle.get()->containsPoint(p[i2])) {
						int ind1 = this_vert->triangles[l].ind_in_triangle;
						int ind2 = this_vert->triangles[l].triangle.get()->getPointIndex(p[i2]);
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
		return true;
	};

	for(int i = 0; i < borders.size(); i++) {
		//depending on if the last edge had a triangle or not it can be pretty clear 
		// if we want to create a new triangle
		// or not
		bool very_first_point = true;
		VertexReference first_pr = borders[i][0].points(1);
		VertexReference last_pr;
		Vector2f last_pix;
		bool triangle_created_last_edge = false;

		//VertexReference debugLastPr1;
		#ifdef SHOW_TEXT_STITCHING
		cout << "running along new border" << endl;
		#endif // SHOW_TEXT_STITCHING

		//TODO: implement this sewing algorithm that follows edges on both sides of the seam!!!!
		bool sewing_mode = false;
		VertexReference current_sewing_pr;
		Vertex *current_sewing_p;
		VertexReference last_sewing_pr;
		Vertex *last_sewing_p;
		Edge current_sewing_edge;
		Vector2i current_sewing_pix(-1, -1);
		Vector2i last_pix_i(-1, -1);

		auto checkAndStartSewing = [&](VertexReference start_pr) {
			Edge other_edge;
			if(getNextOpenEdge(start_pr, other_edge)) {
				VertexReference vr = other_edge.points(1);
				Vector2i pix = project2i(other_edge.points(1).get()->p);
				//TODO: check if the next point really belongs to the newly added geometry
				Meshlet *patch = new_seg_pm.at<Meshlet*>(pix[1], pix[0]);
				if(patch == vr.getPatch()) {
					//if the other side of the stitch is not just a single vertex
					#ifdef SHOW_TEXT_STITCHING
					cout << "starting sewing process" << endl;
					#endif // SHOW_TEXT_STITCHING
					//Starting sewing process!!!!!
					sewing_mode         = true;
					current_sewing_edge = other_edge;
					last_sewing_pr      = current_sewing_edge.points(0);
					last_sewing_p       = last_sewing_pr.get();
					current_sewing_pr   = current_sewing_edge.points(1);
					current_sewing_p    = current_sewing_pr.get();
					current_sewing_pix  = project2i(current_sewing_p->p);
				}
			}
		};

		for(int j = 0; j < borders[i].size(); j++) {
			#ifdef SHOW_TEXT_STITCHING
			cout << "starting new edge" << endl;
			#endif // SHOW_TEXT_STITCHING

			Edge &edge = borders[i][j];

			if(edge.triangle.get()->edges[edge.pos].get(borders) != &edge) {
				assert(0);
			}
			if(edge.already_used_for_stitch) {
				assert(0);//shouldn't every edge be touched only once?
			}

			VertexReference pr0 = edge.points(1);//these are mixed because we screwed up the order at one point
			Vector4f P0         = pr0.get()->p;
			Vector4f point0     = view_inv * P0;//transform the point into camera space
			Vector4f projected0 = p_v * P0;
			float w0 = projected0[3];
			Vector2f pix0  = Vector2f(projected0[0] / w0, projected0[1] / w0);
			Vector2i pix0i = Vector2i(    round(pix0[0]),      round(pix0[1]));

			VertexReference pr1 = edge.points(0);//these are mixed because we screwed up the order at one point
			Vector4f P1         = pr1.get()->p;
			Vector4f point1     = view_inv * P1;
			Vector4f projected1 = p_v * P1;
			float w1 = projected1[3];
			Vector2f pix1  = Vector2f(projected1[0] / w1, projected1[1] / w1);
			Vector2i pix1i = Vector2i(    round(pix1[0]),      round(pix1[1]));

			VertexReference pr2 = edge.oppositePoint();//we need the third point of the triangle here!
			Vector4f P2         = pr2.get()->p;
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

			auto func = [&](int x, int y, float t) {
				#ifdef SHOW_TEXT_STITCHING
				cout << "another pixel (" << x << ", " << y << ")" << endl;
				#endif // SHOW_TEXT_STITCHING
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
							if(!isTrianglePossible({last_sewing_pr, pr0, pr1})) {
								sewing_mode = false;
								return;
							}
							map->addTriangle_(last_sewing_pr, pr0, pr1, debug_list_new_edges);
							nr_triangles_this_edge++;
							#ifdef SHOW_TEXT_STITCHING
							cout << "e" << endl;
							#endif // SHOW_TEXT_STITCHING
							last_pr          = last_sewing_pr;
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
								if(current_sewing_edge.points(1).get()->encompassed()) {
									sewing_mode = false;
									break;
								}
								Edge other_edge;
								current_sewing_edge.getOtherEdge(1, other_edge, borders);

								VertexReference vr = other_edge.points(1);
								Vertex          *v = vr.get();
								Vector2i       pix = project2i(v->p);
								//check if the pixel of the next edge point really belongs to new geometry:
								Meshlet *patch = new_seg_pm.at<Meshlet*>(pix[1], pix[0]);
								if(patch != vr.getPatch()) {
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
								if(!isTrianglePossible({current_sewing_pr, last_sewing_pr,pr1})) {
									sewing_mode = false;
									break;
								}
								map->addTriangle_(current_sewing_pr, last_sewing_pr, pr1,
								                  debug_list_new_edges);
								nr_triangles_this_edge++;
								last_pr = current_sewing_pr;
								#ifdef SHOW_TEXT_STITCHING
								cout << "f2" << endl;
								#endif

								edge_made = true;
								current_sewing_edge = other_edge;
								last_sewing_pr      = current_sewing_pr;
								last_sewing_p       = current_sewing_p;
								current_sewing_pr   = current_sewing_edge.points(1);
								current_sewing_p    = current_sewing_pr.get();
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
							if(!isTrianglePossible({last_sewing_pr, pr0, pr1})) {
								sewing_mode = false;
								return;
							}
							map->addTriangle_(last_sewing_pr, pr0, pr1, debug_list_new_edges);
							nr_triangles_this_edge++;
							last_pr = last_sewing_pr;
							#ifdef SHOW_TEXT_STITCHING
							cout << "g1" << endl;
							#endif
							Vector2i p1 = project2i(last_sewing_p->p);
							Vector2i p2 = project2i(current_sewing_p->p);

							while(true) {
								if(!isTrianglePossible({current_sewing_pr, last_sewing_pr, pr1})) {
									sewing_mode = false;
									return;
								}
								map->addTriangle_(current_sewing_pr, last_sewing_pr, pr1,
								                  debug_list_new_edges);
								last_pr = current_sewing_pr;
								#ifdef SHOW_TEXT_STITCHING
								cout << "g2" << endl;
								#endif
								edge_made = true;
								if(current_sewing_p->encompassed()) {
									sewing_mode = false;
									break;
								}
								Edge other_edge;
								current_sewing_edge.getOtherEdge(1, other_edge, borders);
								VertexReference vr = other_edge.points(1);
								Vertex          *v = vr.get();
								Vector2i       pix = project2i(v->p);
								if(isConnected(pr1, vr)) {
									if(!isOpenEdge(pr1, vr)) {
										sewing_mode = false;
										break;
									}
								}
								Meshlet *patch = new_seg_pm.at<Meshlet*>(pix[1], pix[0]);
								if(patch != vr.getPatch()) {
									#ifdef SHOW_TEXT_STITCHING
									cout << "quitting sewing because the next pixel would lie on wrong side" << endl;
									#endif
									sewing_mode = false;
									break;
								}
								//TODO: this could potentially happen at the end of a loop
								if(isConnected(current_sewing_pr, pr1)) {
									if(!isOpenEdge(current_sewing_pr, pr1)) {
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
								last_sewing_pr = current_sewing_pr;
								last_sewing_p = current_sewing_p;
								current_sewing_pr = current_sewing_edge.points(1);
								current_sewing_p = current_sewing_pr.get();
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
							Edge other_edge;
							current_sewing_edge.getOtherEdge(1, other_edge, borders);
							current_sewing_edge = other_edge;
							last_sewing_pr = current_sewing_pr;
							last_sewing_p = current_sewing_p;
							current_sewing_pr = current_sewing_edge.points(1);
							current_sewing_p = current_sewing_pr.get();
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

					VertexReference this_pr;
					this_pr.set(patch, index);
					//Check if this point is already completely encapsulated by triangles (stitching would then be illegal)
					if(this_pr.get()->encompassed()) {
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

							map->addTriangle_(this_pr, pr0, pr1, debug_list_new_edges);
							#ifdef SHOW_TEXT_STITCHING
							cout << "a" << endl;
							#endif // SHOW_TEXT_STITCHING

							if(!very_first_edge_made) {
								very_first_edge_made = true;
								first_pr  = this_pr;
								first_pix = new_pix;
							}

							//return because we want to go in sewing mode
							if(sewing_mode) {
								edge_made = true;

								last_pr  = this_pr;
								last_pix = new_pix;
								return;
							}
						} else {
							//No triangle connected to this edge yet.
							//create a triangle that incorporates the last made edge
							if(this_pr.isEqualTo(last_pr)) {
								if(!isTrianglePossible({this_pr, pr0, pr1})) {
									continue;
								}
								checkAndStartSewing(this_pr);

								map->addTriangle_(this_pr, pr0, pr1, debug_list_new_edges);
								#ifdef SHOW_TEXT_STITCHING
								cout << "b" << endl;
								#endif // SHOW_TEXT_STITCHING

							} else {
								if(!isTrianglePossible({last_pr, pr0, pr1})) {
									continue;
								}

								Edge other_edge;
								if(getNextOpenEdge(this_pr, other_edge)) {
									VertexReference vr = other_edge.points(1);
									Vector2i pix = project2i(other_edge.points(1).get()->p);
									//TODO: check if the next point really belongs to the newly added geometry
									Meshlet *patch = new_seg_pm.at<Meshlet*>(pix[1], pix[0]);
									if(patch == vr.getPatch()) {
										//if the other side of the stitch is not just a single vertex
										#ifdef SHOW_TEXT_STITCHING
										cout << "starting sewing process" << endl;
										#endif // SHOW_TEXT_STITCHING
										//Starting sewing process!!!!!
										sewing_mode         = true;
										current_sewing_edge = other_edge;
										last_sewing_pr      = current_sewing_edge.points(0);
										last_sewing_p       = last_sewing_pr.get();
										current_sewing_pr   = current_sewing_edge.points(1);
										current_sewing_p    = current_sewing_pr.get();
										current_sewing_pix  = project2i(current_sewing_p->p);
									}
								}

								map->addTriangle_(last_pr, pr0, pr1, debug_list_new_edges);//TODO: reinsert this
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

								if(!isTrianglePossible({this_pr, last_pr, pr1})) {
									continue;
								}
								map->addTriangle_(this_pr, last_pr, pr1, debug_list_new_edges);
								#ifdef SHOW_TEXT_STITCHING
								cout << "c" << endl;
								#endif // SHOW_TEXT_STITCHING
								if(need_to_connect_hole) {
									//TODO: check if we can do another triangle:
									cout << "fill hole!!!!" << endl;
								}
							}
						}
						last_pr   = this_pr;
						last_pix  = new_pix;
						edge_made = true;

					} else {
						//we continue stitching if this edge already has one stitch.
						//TODO: remove this:
						//continue;//debug
						bool need_to_connect_hole = false;

						if(this_pr.isEqualTo(last_pr)) {
							continue;
						}

						//TODO: whenever we have a lastPr and a thisPr to create a triangle we have to check if
						//the orientation is correct
						//don't be confused this is only running if you delete the continue further up
						checkAndStartSewing(this_pr);

						if(!isTrianglePossible({this_pr,last_pr,pr1})) {
							continue;
						}

						if(!isConnected(last_pr,this_pr)) {
							need_to_connect_hole = true;
							//TODO: check if the hole is one or two hops long.
						}
						map->addTriangle_(this_pr, last_pr, pr1, debug_list_new_edges);//third point is from last edge
						#ifdef SHOW_TEXT_STITCHING
						cout << "d" << endl;
						#endif // SHOW_TEXT_STITCHING

						if(need_to_connect_hole) {
							#ifdef SHOW_TEXT_STITCHING
							cout << "holefix" << endl;
							#endif // SHOW_TEXT_STITCHING
						}
						last_pr  = this_pr;
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
	 */
}
