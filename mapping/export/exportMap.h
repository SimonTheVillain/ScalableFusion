#ifndef FILE_EXPORT_MAP_H
#define FILE_EXPORT_MAP_H

#include <iostream>
class MeshReconstruction;

class Exporter{

    //texturing while reducing mesh resolution
    //http://hhoppe.com/tmpm.pdf

    //mesh with variable resolution:
    //http://hhoppe.com/pm.pdf

    enum Properties{
        HighRes = 1,
        LowRes = 2,
        Texture = 4,

    };


public:
    static void ExportMapTest(std::string test);


    static void ExportMap(MeshReconstruction* map,std::string path,unsigned int properties = 0xFFFFFFFF);

    //only HighRes or LowRes is valid
    static void ExportMesh(MeshReconstruction* map,std::string path,Properties properties);


    static void storeCoarse(MeshReconstruction* map,std::string filePath);
    static void storeFine(MeshReconstruction* map,std::string filePath);


    static void storeGraph(MeshReconstruction* map,std::string filePath);
    static void storeDeformationGraph(MeshReconstruction* map,std::string filePath);
};

#endif
