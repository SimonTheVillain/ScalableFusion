#ifndef FILE_EXPORT_MAP_H
#define FILE_EXPORT_MAP_H

#include <string>

using namespace std;

class MeshReconstruction;
class LowDetailRenderer;

class MapExporter {
private:
	//texturing while reducing mesh resolution
	//http://hhoppe.com/tmpm.pdf

	//mesh with variable resolution:
	//http://hhoppe.com/pm.pdf

	enum Properties_ {
		HIGH_RES = 1,
		LOW_RES  = 2,
		TEXTURE  = 4
	};
	
public:
	//TODO: reimplementing a proper integration
	static void exportMapTest(string test) {}
	static void exportMap(MeshReconstruction *map, string path, 
	                      unsigned int properties = 0xFFFFFFFF) {}

	//only HighRes or LowRes is valid
	static void exportMesh(MeshReconstruction *map, string path,
	                       Properties_ properties) {}

	static void storeCoarse(MeshReconstruction *map,LowDetailRenderer* lowDetailRenderer, string file_path);
	static void storeFine(MeshReconstruction *map, string file_path);// {}

	static void storeGraph(MeshReconstruction *map, string file_path) {}
	static void storeDeformationGraph(MeshReconstruction *map, string file_path) {}
};

#endif // FILE_EXPORT_MAP_H
