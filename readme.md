# ScalableFusion:
A mesh based RGB-D reconstruction pipeline which enables large-scale reconstructions.
The advantage over surfel or tsdf based methods is that this allows to store color information in textures of arbitrary resolution.
Coupling between geometrical and color resolution is broken up.
It also features a mechanism which offloads data from the GPU to system memory.

**Warning** This whole thing is far from being finished, we don't recommend using this in its current state.
Issues we are currently encountering:
* random crashes (especially during integration of novel geometry)
* extremely brittle tracking.
* memory leaks
* Nonfunctional multithreading-mode

See paper: ScalableFusion: High-resolution Mesh-based Real-time 3D Reconstruction (https://ieeexplore.ieee.org/document/8793654)


### Installation
* Install the required libraries
* mkdir build
* cd build
* cmake ..
* make -j8

### Required Libraries
* CUDA 10 and up
* Eigen3 the new development build (newer than 3.3.5) so it works with CUDA 10
* Pangolin: (for using the cudaicp stuff from thatmp3 guy)
* Ceres 1.13 and up compiled with new version of eigen
* Sophus: (for cudaicp)
* Radical (https://github.com/taketwo/radical): Photometric calibration
* OpenCV 3.1 (min)
* GLFW3.2 (to create the opengl context)
* PCL
    * hope to get rid of this dependency

### Usage
Right now the library only supports the TUM format of datasets (https://vision.in.tum.de/data/datasets/rgbd-dataset).

It has some extensions though:
Instead of reading from a associations.txt file there is the option to read from a associations_ids.txt file. IDS is the supplier of the high-res camera used in these new HD datasets (See HD command line flag).



### Current state:
##### Implemented:
* The meshing within one frame
* Texturing within one frame
* Stitching between frames
* Standard deviation texturing
    * There is a bug though that lets the texture and geometry degrade at patch borders.

##### About to be implemented:
* nothing!

##### Missing:
* Tracking is not working properly! Something like DSO might be great. 
* Sergeys exposure control and HDR texturing
    * at least a proper integration of it
* Removal of triangles
* Simplification of patches
* Merging of patches


### Possibilites:
* Accumulating Semantic Segmentation
* Maybe a new kind of object detection based on the ease with which we can create models.
* Articulate non-rigid objects and optimize pose based on a course skeleton of triangle shaped bones.
* Model Wires, ropes, pens and fabric which do not have geometry at higher distance but are detectable.
* Estimate reflectiveness and other parameters of surfaces by using any variational or non-vvariational method.
* Superresolution






### Parameters
* **help** Guess what! it creates the help message
* **input,i  <DIRECTORY>** Set the directory to read the TUM dataset.
* **groundtruth,t** By activating this flag, the odometry is taken from the **groundtruth.txt** file.
* **multithreaded,m** Setting this flag enables the multithreaded mode, the frames are read in the same speed as they are captured (assuming 30fps). Is the machine too slow frames are skipped.  DON'T USE ITS NOT STABLE YET.  
* **skipFrames,s <VALUE>** Integer value of the number of frames which should be skipped at the beginning of the trajectory.
* **HD** Flag to enable reading frames as given by the associations_ids.txt.
* **headless,h** Flag to deactivate any window output.
* **output,o** Folder in which all the captured geometry will be stored at the end of the reconstruction. (NOT IMPLEMENTED)
* **coarse <FILENAME>** File (**.ply**) in which the coarse reconstruction is stored at the end of the track.
* **detailed  <FILENAME>** File (**.ply**) in which the fine reconstruction is stored at the end of the track.
* **scaleDepth <VALUE>** The depth usually is given in a 16bit png file such that 1 unit equals to 1mm. If this is different, change this scale factor.
* **scaleGroundtruth <VALUE>** Somethimes the trajectories are not created in correct scale. This is where you could change this.
* **invertGroundtruth <VALUE>** You might have inverted all of your coordinates.... if so then i advise you to set this flag. JB?
* **autoquit,q** When running in windowed mode this flag autoquits the app when all the frames in the dataset are processed.
