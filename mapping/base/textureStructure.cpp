#include "textureStructure.h"

#include <chrono>
#include <limits>

#include "../gpu/gpuGeomStorage.h"
#include "../gpu/gpuBuffer.h"
#include "../cuda/float16_utils.h"
#include "../cuda/gpuErrchk.h"
#include "../meshReconstruction.h"

using namespace std;
using namespace Eigen;

MeshTexture::MeshTexture(shared_ptr<TexAtlas> reference_atlas, 
                         shared_ptr<TexAtlas> data_atlas) {
	//this->refAtlas = referenceAtlas;
	//this->dataAtlas = dataAtlas;
}

MeshTexture::MeshTexture(MeshTexture::Type type, MeshReconstruction *map) 
		: map_(map),
		  type_(type) {
}

MeshTexture::~MeshTexture() {
	/*if(glFramebufferToPixReference){
		glDeleteFramebuffers(1,&glFramebufferToPixReference);
	}*/
}

shared_ptr<MeshTextureGpuHandle> MeshTexture::genGpuResource(size_t nr_coords,
                                                             cv::Size2i size) {

	TexAtlas* ref_atlas  = map_->texAtlasGeomLookup.get();
	TexAtlas* data_atlas = nullptr;
	switch(type_) {
		case MeshTexture::Type::COLOR:
			assert(0);
			//dataAtlas = map->texAtlasRgb16F.get();
			break;
		case MeshTexture::Type::COLOR8:
			data_atlas = map_->texAtlasRgb8Bit.get();
			//TODO: add this to exarregate the impact of that texture leak
			ref_atlas = nullptr;//TODO: remove this when we finally have a better way of updating the color texture
			break;
		case MeshTexture::Type::INTEGER_LABELS:
			data_atlas = map_->texAtlasSegLabels.get();
			break;
		case MeshTexture::Type::WEIGHTED_INTEGER_LABELS:
			data_atlas = map_->texAtlasSemSegLabelsWeights.get();
			break;
		case MeshTexture::Type::STANDARD_DEVIATION:
			data_atlas = map_->texAtlasStds.get();
			break;
	}

	TexCoordBuffer* coord_buf = map_->m_gpuGeomStorage.texPosBuffer;

	shared_ptr<MeshTextureGpuHandle> meshTexGpu(
			new MeshTextureGpuHandle(coord_buf, nr_coords, ref_atlas, data_atlas,
			                         size.width, size.height, nullptr, nullptr));
	//cout << "[MeshTexture] todo" << endl;
	//Explanation: When textures or tex coordinates are currently being downloaded
	//for this module they should be safed from being scrapped and then reapplied
	//the MeshTexture class is just the right place to handle this!!!!!!
	//TODO!!!
	//take a look at the creation of active sets!
	//assert(0);//sourceTexBeingDownloaded & texCoordsbeeingDownloaded needs to be added
	return meshTexGpu;

}

/*
void MeshTexture::createPixReferencesFramebuffer()
{
	//use the proper function in
}
*/

bool MeshTexture::isGpuResidencyRequired() {
	if(parent_patch != NULL) {
		return parent_patch->isGpuResidencyRequired();
	} else {
		cout << "[MeshTexture::isGpuResidencyRequired] The parentPatch unfortunately is not set, therefore this function fails" << endl;
	}
	return false;
}

cv::Rect2i MeshTexture::getLookupRect() {
	//TODO: get rid of this, Since it is locking a weak pointer this would create a false sense of safety
	shared_ptr<MeshTextureGpuHandle> data = gpu.lock();
	return data->ref_tex->getRect();
}

cv::Rect2f MeshTexture::getBounds(const vector<Vector2f> &list) {
	if(list.size() == 0) {
		cout << "[MeshTexture::getBounds] this is not correct yet." << endl;
		return cv::Rect2f(0, 0, 0, 0); // this is a very dirty workaround
	}
	Vector2f min = Vector2f(numeric_limits<float>::max(), 
	                        numeric_limits<float>::max());
	Vector2f max = Vector2f(numeric_limits<float>::min(), 
	                        numeric_limits<float>::min());
	for(size_t i = 0; i < list.size(); i++) {
		const Vector2f &coord = list[i];
		for(int k = 0; k < 2; k++) {
			if(coord[k] < min[k]) {
				min[k] = coord[k];
			} else if(coord[k] > max[k]) {
				max[k] = coord[k];
			}
		}
	}
	cv::Rect2f rect;
	rect.x      = min[0];
	rect.y      = min[1];
	rect.height = max[1] - min[1];
	rect.width  = max[0] - min[0];

	return rect;
}


cv::Rect2f MeshTexture::getBounds() {
	return getBounds(tex_coords);
}

void MeshTexture::scaleAndShiftTexCoordsIntoRect(const cv::Rect2f rect, 
                                                 const vector<Vector2f> &in,
                                                 vector<Vector2f> &out) {
	float width  = 1.0f / rect.width;
	float height = 1.0f / rect.height;
	for(size_t i = 0; i < in.size(); i++) {
		const Vector2f &coordIn = in[i];
		Vector2f &coordOut = out[i];
		coordOut[0] = (coordIn[0] - rect.x) * width;
		coordOut[1] = (coordIn[1] - rect.y) * height;
	}
}

void MeshTexture::scaleAndShiftTexCoordsIntoRect(cv::Rect2f rect) {
	scaleAndShiftTexCoordsIntoRect(rect, tex_coords, tex_coords);
}

MeshTextureGpuHandle::MeshTextureGpuHandle(
		TexCoordBuffer *texCoordBuf, int nr_tex_coords, TexAtlas *ref_atlas,
		TexAtlas *data_atlas, int width, int height,
		shared_ptr<TexAtlasPatch> source_tex_being_downloaded,
		shared_ptr<TexCoordBufConnector> tex_coords_being_downloaded) {

	//debug
	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	cv::Size2i size = cv::Size2i(width, height); //size of interest
	if(tex_coords_being_downloaded == nullptr) {
		coords = texCoordBuf->getBlock(nr_tex_coords);
	} else {
		coords = tex_coords_being_downloaded;
	}
	//create a reference texture
	if(ref_atlas != nullptr) {
		this->ref_tex = ref_atlas->getTexAtlasPatch(size);
	}
	if(data_atlas != nullptr) {
		//if there is a texture on the gpu that is already being downloaded...
		//use that texture instead of creating a new one
		if(source_tex_being_downloaded == nullptr) {
			this->tex = data_atlas->getTexAtlasPatch(size);
		} else {
			this->tex = source_tex_being_downloaded;
		}
	}

	//debug
	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
}

MeshTextureGpuHandle::~MeshTextureGpuHandle() {
	//shared_ptr<MeshTexture> downloadTo = downloadToWhenFinished.lock();
	//instead of doing the download in a separate recycling task lets do it where it happens.
	//specifically whenever the active set changes!!!!!
	//we might want to aggregate this download

/*    if(recycler!=nullptr &&
			downloadTo!=nullptr &&
			gpuDataChanged){
		//downloadTo->TextureMostCurrent = sourceTex;


		auto downloadFunc = [](shared_ptr<MeshTexture> patch,
				shared_ptr<TexCoordBufConnector> coords,
				shared_ptr<TexAtlasPatch> texture){    //The assumption is that the gpu vertex and the cpu vertex are the
			//same


			if(coords==nullptr){
				assert(0);
			}
			vector<Vector2f> coordData(coords->getSize());
			coords->download(&(coordData[0]));

			//now do the texture:
			cv::Rect2i roi = texture->getRectOfInterest();
			GLuint intFormat = texture->getTex()->getGlInternalFormat();
			uint32_t cvType = texture->getAtlasTex()->getCvType();


			//cout << "DEBUG: roi " << roi << endl;
			if(cvType!=CV_32FC4 &&
					cvType!=CV_8UC4){
				assert(0);
			}
			cv::Mat data(roi.height,roi.width,cvType);
			if(intFormat == GL_RGBA16F){
				if(cvType != CV_32FC4){
					assert(0);
				}


				//maybe we should also hold the texture with a mutex
				cudaSurfaceObject_t surface =
						texture->getTex()->getCudaSurfaceObject();
				castF16SurfaceToF32Buffer(surface,
										  roi.x,roi.y,
										  roi.width,roi.height,
										  (float*)data.data,4);

				cudaDeviceSynchronize();

			}else{

				//data.create(roi.height,roi.width,cvFormat);
				if(cvType!=CV_8UC4){
					assert(0);
				}
				//texture->
				texture->downloadData(data.data);
				cudaDeviceSynchronize();
			}


			patch->cpuPayloadMutex.lock();
			patch->cpuPayload = data;
			patch->texCoords = coordData;
			patch->cpuPayloadMutex.unlock();
		};

		if(downloadTo->texCoordsMostCurrent.lock() == nullptr){
			cout << "can't download the resource. "
					"Something i myself do not really get is wrong" << endl;
			return;
			assert(0);
		}
		if(downloadTo->TextureMostCurrent.lock() == nullptr){
			cout << "can't download the resource. "
					"Something i myself do not really get is wrong" << endl;
			return;
			assert(0);
		}

		//ORIGINAL
		auto task = bind(downloadFunc,downloadTo,
							  downloadTo->texCoordsMostCurrent.lock(),//these should be valid
							  downloadTo->TextureMostCurrent.lock());
		recycler->addRecycleTask(task);
		//DEBUG:
		//downloadFunc(downloadTo,coords,sourceTex);
	}
	*/
}



bool MeshTextureGpuHandle::checkRefTexDependencies() {
	//TODO: maybe check the refTex dependencies relative to a active set. !?
	if(ref_tex_dependencies.empty()) {
		return false;//this list being zero points towards a uninitialized refTexture
	}
	for(size_t j = 0; j < ref_tex_dependencies.size(); j++) {
		Dependency dependency = ref_tex_dependencies[j];
		shared_ptr<GeometryBase> dependence_geom = dependency.geometry.lock();
		if(dependence_geom->getMostCurrentGpuTriangles() == nullptr) {
			return false;//there is no geometry on the gpu for this patch
		}
		//TODO: also check if the dependencies of the triangles themselves are met.

		//if the triangles are of the wrong/older version than we need
		if(dependence_geom->triangles_version != dependency.triangles_version) {
			return false;
		}

		//TODO: get these triangles in respect to the active set!?
		shared_ptr<TriangleBufConnector> dependence_tris = 
				dependence_geom->getMostCurrentGpuTriangles();
		//check if the triangles are at the correct index referred to by the refTex
		if(dependence_tris->getStartingIndex() != 
		   dependency.triangle_position_on_gpu) {
			return false;
		}
	}
	return true;
}

GpuTextureInfo MeshTextureGpuHandle::genTexInfo() {
	GpuTextureInfo info = tex->genTexInfo(coords->getStartingIndex());
	if(ref_tex != nullptr){
		info.glRefTexPtrDEBUG = ref_tex->getAtlasTex()->getTex2D()->getGlHandle();
		cv::Rect2i pos = ref_tex->getPosition();
		info.refTexPosDEBUG = Vector2f(pos.x, pos.y) * (1.0f / 1024.0f);
	}
	return info;
}
