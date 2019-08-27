# General TODO List

## Ongoing
* Add interface for proper labelling
    * disable label rendering(render -1) for patches without label texture
    * fix whatever issues we still have with labels. (the coordinates in camera space seem way off! Matrix?)
## General
* Refactor the scaleable map file into:
    * Rasterizer class
    * Mesher class
    * Texturing class
    * depending on how it is needed and whats better style we could
    either let all of them be separate classes 
    (small overhead dereferencing pointers and so on) or inherit them 
    together into the superclass it is.
    * some people on the internet claim that it is a valid practice to have multiple implementation files.
* ORB slam
* Bring multithreading back

## Bugs:
* Some textures seem to be pure nouse after being reuploaded to gpu
* Geometry textures not reloaded when visiting already mapped areas