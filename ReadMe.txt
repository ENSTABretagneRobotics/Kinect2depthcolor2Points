This folder contains a program used to generate a 3D reconstruction of a room using a Kinect 2 during euRathlon competitions.

The principle is the following : 
_ Using a third-party program, get several depth and color images (e.g. depth.png and color.png) from the Kinect 2 at known positions and orientations.
_ Run Kinect2depthcolor2Points to convert those images in 3D points clouds files (e.g. depth.ply) or mesh-based files (e.g. depth.obj and depth.mtl). The program needs that the dimensions, zoom factors, etc. of the depth and color pictures are the same.
_ Display the generated 3D files (e.g. depth0deg.ply, depth90deg.ply, depth180deg.ply, depth270deg.ply) in e.g. MeshLab. If several files are displayed in the same time, it should give a 3D reconstruction of the room.

During a robot mission using UxVCtrl

Press r on each window to start/stop video recording.
In UxVCtrl, press P (the GUI needs focus) to take snapshots of all videos. Note the current orientation and position.
Do the same 3 times at orientation+90 deg while staying at the same position.

Displaying on MeshLab

Use XnView/IrfanView to flip horizontally depth.png and color.png.
Use Kinect2depthcolor2Points depth.png color.png x y z phi theta psi... (in m and rad) to convert depth.png and color.png to .obj+.mtl and .ply.
Drag and drop .obj or .ply on MeshLab, click on the light icon if needed...

Displaying on Google Earth

Convert depth.png and color.png in 160*120 using XnView/IrfanView (need face number < 64K).
Convert to .obj+.mtl with Kinect2depthcolor2Points.
Convert .obj+.mtl to .dae online on http://www.greentoken.de/onlineconv/ .
Replace polylist by triangles in the .dae and edit snap1.kml to change file names, positions and orientations of the objects and camera point of view...
Open with Google Earth the .kml file (because of backface culling, some points of view might not be visible)...

It should work with the following software :
*** USER and DEVEL ***
_ Windows 8 Professional 64 bit
_ Microsoft Visual C++ 2012/2017 Redistributable Package
_ Kinect v2 SDK
_ IrfanView 4.28
_ MeshLab 1.3.3
_ Google Earth
*** DEVEL ***
_ Visual Studio 2012/2017
_ OpenCV 2.4.9/2.4.13 (download http://www.ensta-bretagne.fr/lebars/Share/OpenCV2.4.13.zip and extract in C:)

The following are required sources dependencies :  
_ ..\OSUtils : useful functions.
_ ..\Extensions : some image functions.
_ ..\interval : interval library.
_ ..\matrix_lib : matrix library compatible with interval.
