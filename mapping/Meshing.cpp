#include "Meshing.h"

#include "meshReconstruction.h"
#include "cuda/xtionCameraModel.h"

using namespace Eigen;
using namespace std;

int debug_triangle_count = 0;

TriangleReference Meshing::addTriangle(const VertexReference &pr1,
                                              const VertexReference &pr2,
                                              const VertexReference &pr3,
                                              const Triangle::Neighbour &nr1,
                                              const Triangle::Neighbour &nr2,
                                              const Triangle::Neighbour &nr3,
                                              int &rotated) {

	rotated = 0;
	MeshPatch *patch_1 = pr1.getPatch();
	MeshPatch *patch_2 = pr2.getPatch();
	MeshPatch *patch_3 = pr3.getPatch();
	if(patch_1 == 0 || patch_2 == 0 || patch_3 == 0) {
		assert(0); // Maybe we don't want to have a assert here
		return TriangleReference(); // Can't add this triangle to the
	}

	// Here we should not ask if all the indices are the same...
	// First case is every vertex is from the same patch
	if(patch_1 == patch_2 && patch_1 == patch_3) {
		MeshPatch* patch = patch_1;
		patch->workInProgress.lock();
		patch->cpuTrianglesAhead = true;

		//create a triangle:
		Triangle triangle;
		triangle.points[0] = pr1;
		triangle.points[1] = pr2;
		triangle.points[2] = pr3;
		triangle.neighbours[0] = nr1;
		triangle.neighbours[1] = nr2;
		triangle.neighbours[2] = nr3;
		triangle.debugNr = debug_triangle_count++;

		patch->cpuTrianglesAhead=true;

		TriangleReference triangle_reference;
		triangle_reference.container = patch;
		triangle_reference.index = patch->triangles.size();
		patch->triangles.push_back(triangle);
		Triangle::registerTriangle(triangle_reference, false); // TODO: check if registering the triangle worked
		patch->workInProgress.unlock();
		return triangle_reference;
	}

	// TODO: this triangle double and triple stitches need to be replaced by something simpler.
	// Triple stitch:
	if(patch_1 != patch_2 && patch_2 != patch_3 && patch_1 != patch_3) {

		// First test if there is a triple stitch already
		shared_ptr<TripleStitch> stitch =  patch_1->getTripleStitchWith(patch_2,patch_3);
		// Temporary point references
		VertexReference p1 = pr1;
		VertexReference p2 = pr2;
		VertexReference p3 = pr3;
		Triangle::Neighbour n1 = nr1;
		Triangle::Neighbour n2 = nr2;
		Triangle::Neighbour n3 = nr3;

		// If not create one:
		if(!stitch) {
			stitch = make_shared<TripleStitch>();
			stitch->weakSelf = stitch;
			stitch->patches[0]=patch_1->weakSelf;
			stitch->patches[1]=patch_2->weakSelf;
			stitch->patches[2]=patch_3->weakSelf;

			patch_1->addStitchReference(stitch);
			patch_2->addStitchReference(stitch);
			patch_3->addStitchReference(stitch);
		} else {

			// When a stitch was preexisting the order of points in the triangles has to be changed. to respect that
			MeshPatch* main_patch = stitch->patches[0].lock().get();
			if(main_patch == patch_2) {
				// We have to rotate the print references so that the first reference is pointing to segment 2
				VertexReference psw = p1;
				p1 = p2;
				p2 = p3;
				p3 = psw;

				Triangle::Neighbour nw = n1;
				n1 = n2;
				n2 = n3;
				n3 = nw;
				rotated = 1;
			}
			if(main_patch == patch_3) {
				VertexReference psw = p1;
				p1 = p3;
				p3 = p2;
				p2 = psw;

				Triangle::Neighbour nw = n1;
				n1 = n3;
				n3 = n2;
				n2 = nw;
				rotated = 2;
			}
		}

		Triangle triangle;

		triangle.points[0] = p1;
		triangle.points[1] = p2;
		triangle.points[2] = p3;
		triangle.neighbours[0] = n1;
		triangle.neighbours[1] = n2;
		triangle.neighbours[2] = n3;
		triangle.debugNr = debug_triangle_count++;

		TriangleReference triangle_reference;
		triangle_reference.container = stitch.get();
		triangle_reference.index = stitch->triangles.size();
		stitch->triangles.push_back(triangle);
		Triangle::registerTriangle(triangle_reference, false);
		// We later on want to reupload the triangles to the gpu
		stitch->cpuTrianglesAhead=true;

		return triangle_reference;
	}

	// Double triangle stitch.
	// I know this is quite involved for one triangle:
	if(patch_1 != patch_2 || patch_2 != patch_3 || patch_1 != patch_3) {
		MeshPatch* main_patch = patch_1;
		MeshPatch* secondary_patch = nullptr;
		// Find out the second segment
		if(patch_1 == patch_2 && patch_2 == patch_3 && patch_1 == patch_3) {
			assert(0); // TODO: Make this optional so it only checks this in debug mode
		}
		if(patch_1 == patch_2) {
			secondary_patch = patch_3;
		} else if(patch_1 == patch_3) {
				secondary_patch = patch_2;
		} else if(patch_2 == patch_3) {
			secondary_patch = patch_2;
		}
		assert(secondary_patch != main_patch);
		assert(secondary_patch != nullptr);

		shared_ptr<DoubleStitch> stitch = 
				main_patch->getDoubleStitchWith(secondary_patch);

		// Temporary references to points
		VertexReference p1 = pr1;
		VertexReference p2 = pr2;
		VertexReference p3 = pr3;
		Triangle::Neighbour n1 = nr1;
		Triangle::Neighbour n2 = nr2;
		Triangle::Neighbour n3 = nr3;

		if(!stitch) {
			stitch = make_shared<DoubleStitch>();
			stitch->weakSelf   = stitch;
			stitch->patches[0] = main_patch->weakSelf;
			stitch->patches[1] = secondary_patch->weakSelf;

			patch_1->doubleStitches.push_back(stitch);
			patch_2->doubleStitches.push_back(stitch);
		} else {
			// When a stitch was preexisting the order of points in the triangles has to be changed. to respect that
			MeshPatch* primary_patch = stitch->patches[0].lock().get();
			if(primary_patch != patch_1) {
				if(primary_patch == patch_2) {
					// We have to rotate the print references so that the first reference is pointing to segment 2
					VertexReference psw = p1;
					p1 = p2;
					p2 = p3;
					p3 = psw;

					Triangle::Neighbour nw = n1;
					n1 = n2;
					n2 = n3;
					n3 = nw;
					rotated = 1;
				} else if(primary_patch == patch_3) {
					VertexReference psw = p1;
					p1 = p3;
					p3 = p2;
					p2 = psw;

					Triangle::Neighbour nw = n1;
					n1 = n3;
					n3 = n2;
					n2 = nw;
					rotated = 2;
				}
			}
		}

		Triangle triangle;

		triangle.points[0] = p1;
		triangle.points[1] = p2;
		triangle.points[2] = p3;
		triangle.neighbours[0] = n1;
		triangle.neighbours[1] = n2;
		triangle.neighbours[2] = n3;
		triangle.debugNr = debug_triangle_count++;

		TriangleReference triangle_reference;
		triangle_reference.container = stitch.get();
		triangle_reference.index = stitch->triangles.size();
		stitch->triangles.push_back(triangle);
		Triangle::registerTriangle(triangle_reference, false);

		stitch->cpuTrianglesAhead = true;
		return triangle_reference;
	}
}

void Meshing::meshIt(cv::Mat points, cv::Mat mesh_pointers, 
                     cv::Mat vertex_indices, cv::Mat sensor_std, 
                     float max_depth_step, //deprecate this
                     Matrix4f depth_pose) {
	int width  = points.cols;
	int height = points.rows;

	Triangle::Neighbour nb_up[width];
	Triangle::Neighbour nb_left;
	const Triangle::Neighbour empty;

	int r;
	// In a first step iterate over all points to store the indices
	for(int i = 0; i < height; i++) {
		for(int j = 0; j < width; j++) {
			MeshPatch* mesh = (MeshPatch*) mesh_pointers.at<uint64_t>(i, j);
			if(mesh) {
				Vertex vertex;
				vertex.p = depth_pose * points.at<Vector4f>(i, j);
				if(isnan(vertex.p[0])) {
					assert(0);
					cout << "[ScaleableMap::meshIt] Why is this nan?" << endl;
				}

				int index = mesh->vertices.size();
				vertex_indices.at<int>(i, j) = index;

				mesh->workInProgress.lock();
				mesh->vertices.push_back(vertex);
				mesh->workInProgress.unlock();

				mesh->cpuVerticesAhead = true;

			} else {
				if(!isnan(points.at<Vector4f>(i, j)[0])) {
					//sometimes a point appears valid even though it shouldn't be ( correcting this)
					points.at<Vector4f>(i, j) = Vector4f(NAN, NAN, NAN, NAN);
				}
			}
		}
	}

	// Now create the indices to mesh all this
	for(size_t i = 0; i < height - 1; i++) {
		nb_left.invalidate();
		for(size_t j = 0; j < width - 1; j++) {
			Triangle::Neighbour tn;
			// Two cases: upper left and lower right is closer to each other than upper right and lower left are
			// or the other way around.
			// In each case the possible triangles are different

			float zs[4]; // Storage for depth values
			zs[0] = points.at<Vector4f>(i, j)[2];         // Upper left
			zs[2] = points.at<Vector4f>(i + 1, j + 1)[2]; // Bottom right
			zs[1] = points.at<Vector4f>(i, j + 1)[2];     // Upper right
			zs[3] = points.at<Vector4f>(i + 1, j)[2];     // Bottom left
			float distance_ul_br = fabs(zs[0] - zs[2]);
			float distance_ur_bl = fabs(zs[1] - zs[3]);

			VertexReference pr[4];
			pr[0].set((MeshPatch*) mesh_pointers.at<uint64_t>(i, j),
			          vertex_indices.at<int>(i,j));
			pr[2].set((MeshPatch*) mesh_pointers.at<uint64_t>(i + 1, j + 1),
			          vertex_indices.at<int>(i + 1, j + 1));
			pr[1].set((MeshPatch*) mesh_pointers.at<uint64_t>(i, j + 1),
			          vertex_indices.at<int>(i, j + 1));
			pr[3].set((MeshPatch*) mesh_pointers.at<uint64_t>(i + 1, j),
			          vertex_indices.at<int>(i + 1, j));

			float threshold = max_depth_step;

			int nan_count = 0;
			int nan_at = -1;
			for(size_t k = 0; k < 4; k++) {
				if(isnan(zs[k])) {
					nan_count++;
					nan_at = k;
				}
			}
			if(nan_count == 1) {

				// The threshold has to be taken from a valid pixel
				// which is part of the triangle
				threshold = sensor_std.at<cv::Vec4f>(i, j)[2];
				threshold = xtionStdToThresholdSeg(threshold);
				// With only one invalid triangle we can create one triangle
				bool created = false;
				switch(nan_at) {
					case 0:
						// Create a bottom right triangle
						// but first check for depth values beeing within certain parameters

						// The threshold has to be taken from a valid pixel
						// which is part of the triangle
						threshold = sensor_std.at<cv::Vec4f>(i + 1, j)[2];
						threshold = xtionStdToThresholdSeg(threshold);
						if(fabs(zs[1] - zs[2]) < threshold &&
						   fabs(zs[1] - zs[3]) < threshold) {
							tn.ref = addTriangle(pr[3], pr[2], pr[1], empty, empty, empty, r); // No neighbour in this case
							tn.pos = (1 - r + 3) % 3;
							nb_left = tn;
							tn.pos = (0 - r + 3) % 3;
							nb_up[j] = tn;
							created = true;
						}
						break;
					case 1:
						// Bottom left triangle
						if(fabs(zs[0] - zs[3]) < threshold &&
						   fabs(zs[3] - zs[2]) < threshold) {
							tn.ref = addTriangle(pr[0], pr[3], pr[2], nb_left, empty, empty, 
							                     r);
							tn.pos = (1 - r + 3) % 3;
							nb_up[j] = tn;
							nb_left.invalidate();
							created = true;
						}
						break;
					case 2:
						// Top left triangle
						if(fabs(zs[0] - zs[1]) < threshold &&
						   fabs(zs[0] - zs[2]) < threshold) {
							tn.ref = addTriangle(pr[0], pr[3], pr[1], nb_left, nb_up[j], 
							                     empty, r);
							nb_up[j].invalidate();
							nb_left.invalidate();
							created = true;
						}
						break;
					case 3:
						// Top right triangle
						if(fabs(zs[0] - zs[1]) < threshold &&
						   fabs(zs[1] - zs[2]) < threshold) {
							tn.ref = addTriangle(pr[0], pr[2], pr[1], empty, empty, nb_up[j],
							                     r);
							tn.pos = (1 - r + 3) % 3;
							nb_left = tn;
							nb_up[j].invalidate();
							created = true;
						}
						break;
				}
				if(!created) {
					nb_up[j].invalidate();
					nb_left.invalidate();
				}
				continue;
			}
			if(nan_count > 1) {
				nb_up[j].invalidate();
				nb_left.invalidate();
				continue; // With 2 or more invalid points we can't create a triangle
			}

			bool debug_created = false;
			// If there is no invalid point we create the triangles depending on which
			if(distance_ul_br > distance_ur_bl) { // Even excluding the NAN does not make this code faster

				// The threshold has to be taken from a valid pixel
				// which is part of the triangle
				threshold = sensor_std.at<cv::Vec4f>(i + 1, j)[2];
				threshold = xtionStdToThresholdSeg(threshold);
				if(distance_ur_bl < threshold) {

					Triangle::Neighbour nb_diag;
					if(fabs(zs[1] - zs[0]) < threshold) {
						// Top left triangle
						nb_diag.ref = addTriangle(pr[0], pr[3], pr[1], nb_left, empty, 
						                          nb_up[j], r);
						debug_created = true;

						nb_diag.pos = (1 - r + 3) % 3;
					}

					if(fabs(zs[1] - zs[2]) < threshold) {
						// Bottom right triangle
						tn.ref = addTriangle(pr[3], pr[2], pr[1], empty, empty, nb_diag, r);
						debug_created = true;
						tn.pos = (1 - r + 3) % 3;
						nb_left = tn;
						tn.pos = (0 - r + 3) % 3;
						nb_up[j] = tn;

					} else {
						nb_left.invalidate();
						nb_up[j].invalidate();
					}
				} else {
					nb_left.invalidate();
					nb_up[j].invalidate();
				}

				if(!debug_created) {
					// cout << "no Quad possible a" << j << endl;
				}
			} else {
				// The threshold has to be taken from a valid pixel
				// which is part of the triangle
				threshold = sensor_std.at<cv::Vec4f>(i, j)[2];
				threshold = xtionStdToThresholdSeg(threshold);
				Triangle::Neighbour nb_diag;
				Triangle::Neighbour nb_up_new;
				Triangle::Neighbour nb_left_new;
				if(distance_ul_br < threshold) {
					if(fabs(zs[0] - zs[3]) < threshold) {
						// Bottom left triangle
						if(j == 88 && i == 11) {
							//cout << "Lets fix this" << endl;
						}
						tn.ref = addTriangle(pr[0], pr[3], pr[2], nb_left, empty, empty, r);
						debug_created = true;
						tn.pos = (2 - r + 3) % 3;
						nb_diag = tn;
						tn.pos = (1 - r + 3) % 3;
						nb_up_new = tn;
					}

					if(fabs(zs[0] - zs[1]) < threshold) {
						// Top right triangle
						tn.ref = addTriangle(pr[0], pr[2], pr[1], nb_diag, empty, nb_up[j],
						                     r);
						debug_created = true;
						tn.pos = (1 - r + 3) % 3;
						nb_left_new = tn;
					}
				}
				nb_up[j] = nb_up_new;
				nb_left = nb_left_new;
				if(!debug_created){
					//cout << "no Quad possible b" << j << endl;
				}
			}
		}
	}
}

struct VertexTexConn {
	Vertex* vert;
	// TODO: maybe also add a proper vertex reference
	// with patch and index. this would allow to have
	//  a proper connection between vertices and triangles.
	// THINK: why where some of the vertices not updated???
	// only because they have solely been part of non-patch triangles
	// this unfortunately still might be the case sometimes (even tough
	// this occures less frequently now)
	// DECISION: create a buffer that connects vertices with their tex coordinates
	// ON GPU AND CPU!!!!!!!!!

	vector<uint32_t*> tex_inds;

	// TODO: set the texture index for the vertex if the verte
	// bool setTexIndex=false;
};

inline bool operator<(const VertexTexConn &lhs, const VertexTexConn &rhs) {
	return lhs.vert < rhs.vert;
}

inline bool operator==(const VertexTexConn &lhs, const VertexTexConn &rhs) {
	return lhs.vert == rhs.vert;
}

void Meshing::genTexIndices(vector<shared_ptr<MeshPatch> > &patches) {

	for(size_t i = 0; i < patches.size(); i++) {
		set<VertexTexConn> vert_set;
		auto appendTexCoords = [&vert_set] (Triangle &triangle) {
			for(size_t i = 0; i < 3; i++) {
				VertexTexConn vert;
				vert.vert = triangle.points[i].get();
				set<VertexTexConn>::iterator found = vert_set.find(vert);
				if(found != vert_set.end()) {
					// When one vertex already is within the set we just add the
					// according pointer to the vertex index
					VertexTexConn *tex_conn = const_cast<VertexTexConn*>(&(*found));
					// Only use this cast and modify the elements when
					// you do not modify anything that influences the
					// operators defined on the set
					tex_conn->tex_inds.push_back(&(triangle.texIndices[i]));
				} else {
					// Otherwise we create a new container and push back the
					// right pointer to the texture index
					vert.tex_inds.push_back(&(triangle.texIndices[i]));
					vert_set.insert(vert);
				}
			}
		};

		shared_ptr<MeshPatch> &patch = patches[i];
		patch->geomTexPatch =
				mesh_reconstruction->genMeshTexture(MeshTexture::Type::standardDeviation);

		// To really accomodate for every vertex (even if it may not be visible)
		// we also add the vertices that do not have triangles
		for(size_t j = 0; j < patch->vertices.size(); j++) {
			VertexTexConn vert;
			vert.vert = &patch->vertices[j];
			vert_set.insert(vert);
			// We do this for every vertex because not every vertex would be provided
			// with a texture by one of the triangles.
		}
		// Now create the texture coordinates for the triangles
		// TODO:maybe also a mutex for the triangles? (and vertices)
		for(size_t j = 0; j < patch->triangles.size(); j++) {
			appendTexCoords(patch->triangles[j]);
		}

		// Check all the triangles that lie in double stitches
		patch->doubleStitchMutex.lock();
		for(size_t k = 0; k < patch->doubleStitches.size(); k++) {
			shared_ptr<DoubleStitch> &stitch = patch->doubleStitches[k];
			if(stitch->patches[0].lock() != patch) {
				// The texture for this stitch is provided by another patch
				continue;
			}
			for(size_t j = 0; j < stitch->triangles.size(); j++) {
				appendTexCoords(stitch->triangles[j]);
			}
		}
		patch->doubleStitchMutex.unlock();
		// And now check all the triangles that lie in triple stitches
		patch->tripleStitchMutex.lock();
		for(size_t k = 0; k < patch->tripleStitches.size(); k++) {
			shared_ptr<TripleStitch> &stitch = patch->tripleStitches[k];
			if(stitch->patches[0].lock() != patch) {
				// Obviously this patch is not what we are searching for
				continue;
			}
			for(size_t j = 0; j < stitch->triangles.size(); j++) {
				appendTexCoords(stitch->triangles[j]);
			}
		}
		patch->tripleStitchMutex.unlock();
		patch->geomTexPatch->texCoords.resize(vert_set.size());
		size_t j = 0;
		for(auto v : vert_set) {
			for(size_t k = 0; k < v.tex_inds.size(); k++) {
				*(v.tex_inds[k]) = j;
			}
			j++;
		}

		// Now do the texture indices on the patch vertices
		for(size_t j = 0; j < patch->triangles.size(); j++) {
			// TODO: get better tex coordinates
			for(size_t k = 0; k < 3; k++) {
				if(patch->triangles[j].points[k].getPatch() == patch.get()) {
					patch->triangles[j].points[k].get()->texIndInMainPatch =
							patch->triangles[j].texIndices[k];
				}
			}
		}
	}
}