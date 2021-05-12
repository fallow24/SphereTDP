# Improving the quality of indoor 3D point clouds using plane extraction and pose correction in 3DTK

This directory contains a 3DTK addon.
To compile, put the CMakeLists.txt file into the "addons" folder.
Also, put the corresponding "preg6d" folders into the include- and src-subdirectories, also located in the "addons" folder.
Use cmake to set the "WITH\_ADDONS" flag, then set the "WITH\_PREG6D" flag and compile. 

## Theory and Backround 

Nearly any manmade building consists of regularities regarding planes, which is due to architectural and structural engeneering reasons.
This can be abused to correct a 3D point cloud, recorded in such an environment.
D. Borrmann and A. Nuechter describe this process in ["Interior reconstruction using the 3D hough transform"](https://github.com/fallow24/SphereTDP/blob/master/Organisational/3darch2013.pdf) and ["Non-rigid registration and rectification of 3D laser scans"](https://github.com/fallow24/SphereTDP/blob/master/Organisational/iros2010_1.pdf) with great detail. 

## Practical usage 

Take the following steps as guideline in order to improve your PCL in 3DTK:

1. Rigidly register all scans global
 
   Typically, a scan consists of a directory which contains multiple sub-scans. Use bin/slam6d or bin/correction on the entire directory. This will create .frame files which contain information about the global pose of each scan. (The bin/condense program might help with registration.)

2. Export the scan directory
 
   Next, you will need to export the entire directory into a single scanfile. Use bin/condense to do that. The program needs the .frame files, so if you want to skip the registration for any reason, consider running bin/pose2frames first.

3. Plane extration

   Then, the 3d hough transform should be used to extract planes from the globaly consistent scan. Run bin/planes to do so. There is a config file located in bin/hough.cfg that you should consider when using the program. Not only do you have to set the export-directory, but the quality of the extracted planes highly depends on the parameters found in that file.  
   The program will create planeXXX.3d files and a planes.list file, representing the convex hulls of the planes. This is particularly usefull for visualizing the planes before proceeding (highly recommended!). To visualize the extracted planes, run bin/show on the exported scan using the -l option.  
   E.g: bin/show /your/directory/ -l /your/directory/planes/planes.list   

4. Pose correction based on error model

   TODO: write a program to extract normals from the convex hull (see convexplane.cc::217)  
   The normals must then be used to correct the poses in the original scan directory, containing all .pose and .frames files.  



