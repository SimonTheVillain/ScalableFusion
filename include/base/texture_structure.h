#ifndef FILE_TEXTURE_STRUCTURE_H
#define FILE_TEXTURE_STRUCTURE_H

#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <gpu/gpu_tex.h>
#include <gpu/tex_atlas.h>
#include <gpu/gpu_buffer.h>

/**
 * General discussion about textures:
 * 3 Options:
 * 1) Having one texture for every texture patch
 *   -> this isn't that bad of an idea for reading out data since we use bindless textures
 *   -> we would have to bind one framebuffer for every visible patch (several thousand a frame)
 *
 * 2) Having bigger textures storing multiple patch Texture
 *   -> this is way more involved since we would have to maintain all of this
 *   -> when rendering back to the texture we would have to maintain a copy of each texture storage block
 *      every rendering made to one of the texture storage block would have to be copied back to the texture
 *      block. If we do not copy the result back to the texture we would potentially end up with the situation
 *      where we would have to read and write to the same texture. -> this is baaad or impossible.
 *        https://stackoverflow.com/questions/6087441/fast-swapping-framebuffers-opengl
 *        This seems unproven. the article linked provides no timing examples and no explanation of why it might be so.
 *        Having effectively written an OpenGL driver I can tell you that switching between framebuffers should be faster
 *        than switching attachments. The reason is a framebuffer is only complete if a whole set of criteria are true.
 *        Everytime you change an attachment all of those criteria need to be checked. On the other hand a framebuffer
 *        that has already been checked doesn't need to be checked again. QED switching framebuffers should be faster.
 *
 * 3) Every geometrical texture patch has one texture storing if and which (3d) triangle the according pixel are part of.
 *    This texture also has to store information where on the triangle the according point is located on.
 *    (maybe even 2 textures depending on if i need 2 or 3 parameters to parametrise the triangle location)
 *    -> updating the location texture has to be done whenever the the texture coordinates get changed.
 *    -> Triangle location consisting out of multiple parameters:
 *        * the patch id, which is given by the relation between texture and triangle
 *        * the triangle id
 *        * the 3 barycentric coordinates (altough 2 should be enough): (by letting a varying interpolate (1 0 0) (0 1 0) and (0 0 1)
 *              https://codeplea.com/triangular-interpolation
 *              https://stackoverflow.com/questions/25397586/accessing-barycentric-coordinates-inside-fragment-shader
 *        * altogehter each point needs 1 int32 and 3 float32. (this should totally be doable)
 *    -> switching to higher resolution as soon as the camera is close enough
 *    -> maybe 2 textures are required for updating everything. (research if reading and writing one texture
 *       within the same kernel is possible as long as every pixel is read and written just once. ANSWER: NO)
Best Practices Guide
9.2. Device Memory Spaces
In the case of texture access, if a texture reference is bound to a linear array in global
memory, then the device code can write to the underlying array. Texture references that
are bound to CUDA arrays can be written to via surface-write operations by binding
a surface to the same underlying CUDA array storage). Reading from a texture while
writing to its underlying global memory array in the same kernel launch should be
avoided because the texture caches are read-only and are not invalidated when the
associated global memory is modified.
 *    -> one cuda block for each std texture to gather information
 *       of rendered current standard deviation and depth image (maybe sensor resolution or multiple of that)
 *       Questions to ask:
 *       * compute blocks of different size possible? Maybe start threads within threads? Or doing loops within the threads?
 *         https://devblogs.nvidia.com/parallelforall/cuda-dynamic-parallelism-api-principles/
 *       * the location and triangle id texture acts as a mask, is there a way to reuse masked out cuda threads?
 *         (more than half the threads are discarded due to this mask)
 *           What about creating a list of valid pixel for each texture and letting the threads get the positions to work on from that?
 *           There might be something better.
 *       * doing stuff like this in a multiple separate threads:
 *         https://devblogs.nvidia.com/parallelforall/cuda-pro-tip-always-set-current-device-avoid-multithreading-bugs/
 *
 *
 *
 *
 * Verdict: 1 might be faster but switching between framebuffers that often is aweful
 * Switching framebuffers is supposed to be faster than switching the framebuffers attachment
 * TODO: create a FBO for every texture object
 *
 * New Verdict: 3 prevents switching of framebuffers -> the way to go
 *
 */

using namespace std;
using namespace Eigen;

class GeometryBase;
class MeshReconstruction;
class Meshlet;
class ActiveSet;
struct MeshTexture;
class Recycler;

//i think it will be easiest if there is no direct connection between the
//texture gpu handle and the geometry gpu handle
//TODO: mechanism to check out
class MeshTextureGpuHandle {
public:

	struct Dependency {
		weak_ptr<GeometryBase> geometry; //maybe this needs to be a MeshletGpuHandle or GeometryBase
		// (because we care for triangles)
		int triangle_position_on_gpu = 0;
		int triangles_version = 0;
	};

	MeshTextureGpuHandle(
			TexCoordBuffer *tex_coord_buf, int nr_tex_coords, TexAtlas *ref_atlas,
			TexAtlas *data_atlas, int width, int height, 
			shared_ptr<TexAtlasPatch> source_tex_being_downloaded = nullptr,
			shared_ptr<TexCoordBufConnector> tex_coords_being_downloaded = nullptr);

	bool checkRefTexDependencies();

	GpuTextureInfo genTexInfo();

	mutex tex_mutex;
	shared_ptr<TexCoordBufConnector> coords;
	shared_ptr<TexAtlasPatch> ref_tex;
	bool ref_tex_filled = false;
	//TODO: create measure for when this texture is even valid!!!
	//because if not we have a problem.
	//ways to do this might be to connect the validity to certain active sets
	//->unnecessary recomputation
	//other way could be to have revision numbers to all the neighbouring patches
	//and a connection to gpu handles which should not be expired or receive
	//a new revision number for the lookup to stay valid.
	//->a lot of effort for keeping the data consistent
	//(threading is an issue still)

	shared_ptr<TexAtlasPatch> tex;

	vector<Dependency> ref_tex_dependencies;

	bool gpu_data_changed = false;

	//TODO: get rid of this unless i come to the conclusion that it actually is useful
	//TODO: document who is supposed to hold the handle of where to download
	// i really have no clue of how this works
	//weak_ptr<MeshTexture> downloadToWhenFinished;
	//shared_ptr<Recycler> recycler;
};

/**
 * @brief The MeshTexture struct
 * This thing handles the affiliation between triangles and their texture by a lookup texture.
 * It also serves with a CPU backup memory and 2 copies of the GPU payload one to read from
 * and the second to write the updated one to.
 * + id doesn't work yet.
 *
 * somehow the GPU payload type should be able to be created by the CPU payload
 */
struct MeshTexture {
	friend Meshlet;
	friend ActiveSet;

public:
	//TODO: add different types of texture:
	/*
	 * storing median and standard deviation to the surface
	 * color stores projected color for a surface
	 * integerLabels stores labels on the surface
	 * weightedInteger labels stores a label + an accumulated weight
	 *
	 */
	enum Type {
		STANDARD_DEVIATION,
		COLOR,//16 bit float
		COLOR8,
		INTEGER_LABELS,
		WEIGHTED_INTEGER_LABELS
	};

	//TODO: remove this
	MeshTexture(shared_ptr<TexAtlas> reference_atlas,
	            shared_ptr<TexAtlas> data_atlas);//TODO: also add the framebuffer thingy

	MeshTexture(Type type, MeshReconstruction *map);


	/*
	shared_ptr<MeshTextureGpuHandle> genGpuResource(size_t nr_coords, 
	                                                cv::Size2i size);
	*/
	//cv::Rect2i getLookupRect();
	//TODO: logic to determine when said lookup is valid
	//also the tex coords are not valid in any circumstances

	cv::Rect2f getBounds();

	void  scaleAndShiftTexCoordsIntoRect(const cv::Rect2f rect);

	static cv::Rect2f getBounds(const vector<Vector2f> &list);


	static void scaleAndShiftTexCoordsIntoRect(const cv::Rect2f rect,
	                                           const vector<Vector2f> &in,
	                                           vector<Vector2f> &out);

	//ACTUALLY THESE NEXT FEW FUNCTIONS ARE HARD TO IMPLEMENT
	/*
	void isLookupTexFilled();

	//this is the most important question
	void isLookupTexUpToDate();

	void isLookupSecuredAndUpToDate(Meshlet* patch,shared_ptr<ActiveSet>* set);
	*/


	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	alignas(32) Matrix4f cam_pose;

	bool ref_tex_filled = false;

	//TODO: get rid of this
	vector<Vector2f> tex_coords;

	///TODO: might be needed for gpuGeomstorage !It is actually needed to check if the residency of this patch still is required.
	Meshlet *parent_patch = nullptr;

	mutex mat_mutex;
	cv::Mat mat;

	//weak_ptr<MeshTextureGpuHandle> gpu;


	string name;

private:
	//Either we store this and also some information so we don't
	//need to create the gpu
	//or maybe we only store a reference to the map:
	MeshReconstruction *map_;
	Type type_;
	//everything additional like texAtlas derive from the textureContent enum
	//TODO: only let the map call the constructor for meshTexture;
	//Only the meshTexture instantiates
};

//https://stackoverflow.com/questions/6087441/fast-swapping-framebuffers-opengl
//This seems unproven. the article linked provides no timing examples and no
//explanation of why it might be so. Having effectively written an OpenGL driver
//I can tell you that switching between framebuffers should be faster than
//switching attachments. The reason is a framebuffer is only complete if a
//whole set of criteria are true. Everytime you change an attachment all of
//those criteria need to be checked. On the other hand a framebuffer that has
//already been checked doesn't need to be checked again. QED switching f
//ramebuffers should be faster.

#endif // FILE_TEXTURE_STRUCTURE_H