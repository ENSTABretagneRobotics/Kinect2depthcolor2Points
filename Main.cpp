#include "OSMisc.h"
//#include "imatrix.h"
//#include "CvInc.h"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <cmath>

cv::Mat CvRotationPhiThetaPsi(double phi, double theta, double psi)
{
    // phi: roll angle in radians
    // theta: pitch angle in radians
    // psi: yaw angle in radians
    // Rotations are applied in the following order:
    // 1. Rotation around the roll axis (X-axis)
    // 2. Rotation around the pitch axis (Y-axis)
    // 3. Rotation around the yaw axis (Z-axis)
    // The composite rotation matrix R = Rz * Ry * Rx
    
    // Precompute sines and cosines of Euler angles
    double cphi = std::cos(phi);
    double sphi = std::sin(phi);
    double ctheta = std::cos(theta);
    double stheta = std::sin(theta);
    double cpsi = std::cos(psi);
    double spsi = std::sin(psi);

    // Initialize the rotation matrix
    cv::Mat A = cv::Mat::zeros(3, 3, CV_64F);

    // Populate the rotation matrix elements
    A.at<double>(0, 0) = ctheta * cpsi;
    A.at<double>(0, 1) = -cphi * spsi + stheta * cpsi * sphi;
    A.at<double>(0, 2) = spsi * sphi + stheta * cpsi * cphi;

    A.at<double>(1, 0) = ctheta * spsi;
    A.at<double>(1, 1) = cpsi * cphi + stheta * spsi * sphi;
    A.at<double>(1, 2) = -cpsi * sphi + stheta * spsi * cphi;

    A.at<double>(2, 0) = -stheta;
    A.at<double>(2, 1) = ctheta * sphi;
    A.at<double>(2, 2) = ctheta * cphi;

    return A;
}

int depthcolor2PLYKinect2SDK(char* depthimgfilename, char* colorimgfilename, 
							 double x, double y, double z, double phi, double theta, double psi, 
							 double horizontalBeam, double verticalBeam, double maxDistance)
{
	char szTemp[256];
	char plyfilename[256];

	// Load color.png and depth.png files.
	cv::Mat imcolor = cv::imread(colorimgfilename);
	cv::Mat imdepth = cv::imread(depthimgfilename);

	// Total number of points needs to be specified in the PLY file.
	unsigned int nbpoints = (unsigned int)(imcolor.cols*imcolor.rows);

	// Focal lengths and optical center in pixels, used to compute the 3D positions of the points from the 2D positions in the image.
	double fx = imcolor.cols/(2*tan(horizontalBeam*0.5*M_PI/180.0));
	double fy = imcolor.rows/(2*tan(verticalBeam*0.5*M_PI/180.0));
	double cx = imcolor.cols/2.0;
	double cy = imcolor.rows/2.0;

	// Angles order...?
	//rmatrix R = RotationPhiThetaPsi(psi, phi, theta);
	cv::Mat R = CvRotationPhiThetaPsi(psi, phi, theta);

	strcpy(szTemp, depthimgfilename);
	RemoveExtensionInFilePath(szTemp);
	sprintf(plyfilename, "%.249s.ply", szTemp);

    // Create the PLY file and write its header.
	std::ofstream f;
	f.open(plyfilename, std::ios::out|std::ios::trunc);
	f << "ply\n"
		 "format ascii 1.0\n"
		 "element vertex " << nbpoints << "\n" <<
		 "property float x\n"
		 "property float y\n"
		 "property float z\n"
		 "property uchar red\n"
		 "property uchar green\n"
		 "property uchar blue\n"
		 //"property float nx\n"
		 //"property float ny\n"
		 //"property float nz\n"
		 //"property float radius\n"
		 "element face 0\n"
		 "property list uchar int vertex_indices\n"
		 "end_header\n";

	// Conversion of the 2D positions in the image to 3D positions in space.
	// The meaning of x,y,z might not be the same between the Kinect coordinate
	// space and the PLY coordinate space, but it should not be important at first...
	unsigned char d = 0, b = 0, g = 0, r = 0;
	for (int i = 0; i < imcolor.rows; i++)
		for (int j = 0; j < imcolor.cols; j++)
		{
			double xp = 0, yp = 0, zp = 0; // Positions in space to write in the PLY file.
			b = imcolor.at<cv::Vec<unsigned char, 3>>(i, j)[0];
			g = imcolor.at<cv::Vec<unsigned char, 3>>(i, j)[1];
			r = imcolor.at<cv::Vec<unsigned char, 3>>(i, j)[2];
			// Conversion of the grey level in the depth image to a distance in m,
			// considering that maxDistance (8 m for Kinect v2) is the max range and
			// would correspond to black (grey level of 0).
			d = imdepth.at<cv::Vec<unsigned char, 3>>(i, j)[0];
			zp = -(255.0-d)*maxDistance/255.0;
			// Then, the x,y position depends on the position of the pixel in the image (i,j),
			// the distance (zp) and the FOV of the color camera (through the fx and fy coefficients).
			yp = zp*(i-cy)/fy;
			xp = -zp*(j-cx)/fx;

			//rmatrix p = rmatrix(box(yp-y, zp-z, xp-x));
			//box q = ToBox(R*p);

			//// Write values defining a point in the PLY file : x, y, z, red, green, blue.
			//f << Center(q[3])+z << " " << Center(q[1])+x << " " << Center(q[2])+y << " " << int(r) << " " << int(g) << " " << int(b) << std::endl;

			cv::Mat p = (cv::Mat_<double>(3, 1) << yp - y, zp - z, xp - x);
			cv::Mat q = R * p;
			cv::Vec3d q_vec(q);

			// Write values defining a point in the PLY file : x, y, z, red, green, blue.
			f << q_vec[2]+z << " " << q_vec[0]+x << " " << q_vec[1]+y << " " << int(r) << " " << int(g) << " " << int(b) << std::endl;
		}

	f.close();

	imcolor.release();
	imdepth.release();

	return EXIT_SUCCESS;
}

int depthcolor2OBJKinect2SDK(char* depthimgfilename, char* colorimgfilename,
							double x, double y, double z, double phi, double theta, double psi,
							double horizontalBeam, double verticalBeam, double maxDistance, 
							double dmin, double dmax, double maxfacesize)
{
	char szTemp[256];
	char objfilename[256];
	char mtlfilename[256];
	char szMTLName[256];
	char colorimgrelativefilename[256];

	cv::Mat imdepth = cv::imread(depthimgfilename);

	// Total number of points needs to be specified in the PLY file.
	//unsigned int nbpoints = (unsigned int)(imcolor.cols*imcolor.rows);

	// Focal lengths and optical center in pixels, used to compute the 3D positions of the points from the 2D positions in the image.
	double fx = imdepth.cols/(2*tan(horizontalBeam*0.5*M_PI/180.0));
	double fy = imdepth.rows/(2*tan(verticalBeam*0.5*M_PI/180.0));
	double cx = imdepth.cols/2.0;
	double cy = imdepth.rows/2.0;

	// Angles order...?
	//rmatrix R = RotationPhiThetaPsi(psi, phi, theta);
	cv::Mat R = CvRotationPhiThetaPsi(psi, phi, theta);

	strcpy(szTemp, depthimgfilename);
	RemoveExtensionInFilePath(szTemp);
	sprintf(objfilename, "%.249s.obj", szTemp);
	sprintf(mtlfilename, "%.249s.mtl", szTemp);
	strcpy(szMTLName, szTemp);
	RemovePathInFilePath(szMTLName);

	FILE* fobj = fopen(objfilename, "w");

	fprintf(fobj, "mtllib %.249s.mtl\n", szMTLName);

	for (int i = 0; i < imdepth.rows; i++)
		for (int j = 0; j < imdepth.cols; j++)
		{
			double xp = 0, yp = 0, zp = 0;
			zp = -(255.0-imdepth.at<cv::Vec<unsigned char, 3>>(i, j)[0])*maxDistance/255.0;
			yp = zp*(i-cy)/fy;
			xp = -zp*(j-cx)/fx;

			//rmatrix p = rmatrix(box(yp-y, zp-z, xp-x));
			//box q = ToBox(R*p);

			//fprintf(fobj, "v %f %f %f\n", Center(q[3])+z, Center(q[1])+x, Center(q[2])+y);

			cv::Mat p = (cv::Mat_<double>(3, 1) << yp - y, zp - z, xp - x);
			cv::Mat q = R * p;
			cv::Vec3d q_vec(q);

			fprintf(fobj, "v %f %f %f\n", q_vec[2]+z, q_vec[0]+x, q_vec[1]+y);
		}

	for (int i = 0; i < imdepth.rows; i++)
		for (int j = 0; j < imdepth.cols; j++)
		{
			double u = j/(double)imdepth.cols, v = (imdepth.rows-1-i)/(double)imdepth.rows;
			fprintf(fobj, "vt %f %f\n", u, v);
		}

	fprintf(fobj, "usemtl %.255s\n", szMTLName);

	for (int i = 0; i < imdepth.rows-1; i++)
		for (int j = 0; j < imdepth.cols-1; j++)
		{
			int b = (j+imdepth.cols*i);
			int a = ((j+1)+imdepth.cols*i);
			int c = (j+imdepth.cols*(i+1));
			int d = ((j+1)+imdepth.cols*(i+1));

			double za = -(255.0-imdepth.at<cv::Vec<unsigned char, 3>>(i, j+1)[0])*maxDistance/255.0;
			double ya = za*(i-cy)/fy;
			double xa = -za*((j+1)-cx)/fx;
			double zb = -(255.0-imdepth.at<cv::Vec<unsigned char, 3>>(i, j)[0])*maxDistance/255.0;
			double yb = zb*(i-cy)/fy;
			double xb = -zb*(j-cx)/fx;
			double zc = -(255.0-imdepth.at<cv::Vec<unsigned char, 3>>(i+1, j)[0])*maxDistance/255.0;
			double yc = zc*((i+1)-cy)/fy;
			double xc = -zc*(j-cx)/fx;
			double zd = -(255.0-imdepth.at<cv::Vec<unsigned char, 3>>(i+1, j+1)[0])*maxDistance/255.0;
			double yd = zd*((i+1)-cy)/fy;
			double xd = -zd*((j+1)-cx)/fx;

			// Order because of Backface Culling...
			if (
				(-za > dmin)&&(-za < dmax)&&(-zb > dmin)&&(-zb < dmax)&&(-zc > dmin)&&(-zc < dmax)&&
				(sqrt(sqr(xa-xb)+sqr(ya-yb)+sqr(za-zb)) < maxfacesize)&&
				(sqrt(sqr(xa-xc)+sqr(ya-yc)+sqr(za-zc)) < maxfacesize)&&
				(sqrt(sqr(xb-xc)+sqr(yb-yc)+sqr(zb-zc)) < maxfacesize)
				)
				fprintf(fobj, "f %d/%d %d/%d %d/%d\n", a+1, a+1, b+1, b+1, c+1, c+1);
			if (
				(-za > dmin)&&(-za < dmax)&&(-zb > dmin)&&(-zb < dmax)&&(-zc > dmin)&&(-zc < dmax)&&
				(sqrt(sqr(xa-xc)+sqr(ya-yc)+sqr(za-zc)) < maxfacesize)&&
				(sqrt(sqr(xa-xd)+sqr(ya-yd)+sqr(za-zd)) < maxfacesize)&&
				(sqrt(sqr(xc-xd)+sqr(yc-yd)+sqr(zc-zd)) < maxfacesize)
				)
				fprintf(fobj, "f %d/%d %d/%d %d/%d\n", a+1, a+1, c+1, c+1, d+1, d+1);
		}

	fclose(fobj);

	FILE* fmtl = fopen(mtlfilename, "w");
	fprintf(fmtl, "newmtl %.255s\n", szMTLName);

	strcpy(colorimgrelativefilename, colorimgfilename);
	RemovePathInFilePath(colorimgrelativefilename);

	fprintf(fmtl, "map_Kd %.255s\n", colorimgrelativefilename);
	fclose(fmtl);

	imdepth.release();

	return EXIT_SUCCESS;
}

int main(int argc, char* argv[])
{
	char szFileDepthPath[256];
	char szFileColorPath[256];
	double x = 0, y = 0, z = 0, phi = 0, theta = 0, psi = 0, horizontalBeam = 70.6, verticalBeam = 53.8, maxDistance = 8;
	double dmin = 0.1, dmax = 7.9, maxfacesize = 1;

	if (argc != 15)
	{
		strcpy(szFileDepthPath, "depth.png");
		strcpy(szFileColorPath, "color.png");
		printf("Warning : No parameter specified.\n");
		printf("Usage : Kinect2depthcolor2Points depth.png color.png x y z phi theta psi horizontalBeam verticalBeam maxDistance dmin dmax maxfacesize.\n");
		printf("Default : Kinect2depthcolor2Points %.255s %.255s %f %f %f %f %f %f %f %f %f %f %f %f.\n",
			szFileDepthPath, szFileColorPath, x, y, z, phi, theta, psi, horizontalBeam, verticalBeam, maxDistance, dmin, dmax, maxfacesize);
	}
	else
	{
		sprintf(szFileDepthPath, "%.249s", argv[1]);
		sprintf(szFileColorPath, "%.249s", argv[2]);
		x = atof(argv[3]);
		y = atof(argv[4]);
		z = atof(argv[5]);
		phi = atof(argv[6]);
		theta = atof(argv[7]);
		psi = atof(argv[8]);
		horizontalBeam = atof(argv[9]);
		verticalBeam = atof(argv[10]);
		maxDistance = atof(argv[11]);
		dmin = atof(argv[12]);
		dmax = atof(argv[13]);
		maxfacesize = atof(argv[14]);
	}

	depthcolor2PLYKinect2SDK(szFileDepthPath, szFileColorPath, x, y, z, phi, theta, psi, horizontalBeam, verticalBeam, maxDistance);
	depthcolor2OBJKinect2SDK(szFileDepthPath, szFileColorPath, x, y, z, phi, theta, psi, horizontalBeam, verticalBeam, maxDistance, dmin, dmax, maxfacesize);

/*
	// Temporary...

	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h42min39s.png", "snap0_2015-08-14_22h42min39s.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h43min37s.png", "snap0_2015-08-14_22h43min37s.png", 0.170, 0.150, 0.060, 0, 0, 0, 70.6, 53.8, 8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h44min04s.png", "snap0_2015-08-14_22h44min04s.png", 0.170, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8, 8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h44min27s.png", "snap0_2015-08-14_22h44min27s.png", 0.170, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8, 8);
	
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h42min39s.png", "snap0_2015-08-14_22h42min39s.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h43min37s.png", "snap0_2015-08-14_22h43min37s.png", 0.170, 0.150, 0.060, 0, 0, 0, 70.6, 53.8, 8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h44min04s.png", "snap0_2015-08-14_22h44min04s.png", 0.170, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8, 8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h44min27s.png", "snap0_2015-08-14_22h44min27s.png", 0.170, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8, 8, 0.1, 7.9, 1);
*/
	/*
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h42min39s.png", "snap0_2015-08-14_22h42min39s.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h43min37s.png", "snap0_2015-08-14_22h43min37s.png", 0.170, 0.150, 0.060, 0, 0, 0, 70.6, 53.8, 8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h44min04s.png", "snap0_2015-08-14_22h44min04s.png", 0.170, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8, 8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h44min27s.png", "snap0_2015-08-14_22h44min27s.png", 0.170, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8, 8);

	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h45min51s.png", "snap0_2015-08-14_22h45min51s.png", 0.260, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h46min14s.png", "snap0_2015-08-14_22h46min14s.png", 0.260, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8, 8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h46min30s.png", "snap0_2015-08-14_22h46min30s.png", 0.260, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8, 8);
	
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h42min39s.png", "snap0_2015-08-14_22h42min39s.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h43min37s.png", "snap0_2015-08-14_22h43min37s.png", 0.170, 0.150, 0.060, 0, 0, 0, 70.6, 53.8, 8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h44min04s.png", "snap0_2015-08-14_22h44min04s.png", 0.170, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8, 8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h44min27s.png", "snap0_2015-08-14_22h44min27s.png", 0.170, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8, 8, 0.1, 7.9, 1);

	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h45min51s.png", "snap0_2015-08-14_22h45min51s.png", 0.260, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h46min14s.png", "snap0_2015-08-14_22h46min14s.png", 0.260, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8, 8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h46min30s.png", "snap0_2015-08-14_22h46min30s.png", 0.260, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8, 8, 0.1, 7.9, 1);

	
	depthcolor2PLYKinect2SDK("depth1.png", "colorondepth1.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 8);
	depthcolor2PLYKinect2SDK("depth2.png", "colorondepth2.png", 0.170, 0.150, 0.060, 0, 0, 0, 70.6, 53.8, 8);
	depthcolor2PLYKinect2SDK("depth3.png", "colorondepth3.png", 0.170, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8, 8);
	depthcolor2PLYKinect2SDK("depth4.png", "colorondepth4.png", 0.170, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8, 8);

	depthcolor2OBJKinect2SDK("depth1.png", "colorondepth1.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("depth2.png", "colorondepth2.png", 0.170, 0.150, 0.060, 0, 0, 0, 70.6, 53.8, 8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("depth3.png", "colorondepth3.png", 0.170, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8, 8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("depth4.png", "colorondepth4.png", 0.170, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8, 8, 0.1, 7.9, 1);


	depthcolor2PLYKinect2SDK("depthsmall_1.png", "colorondepthsmall_1.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 8);

	depthcolor2OBJKinect2SDK("depthsmall_1.png", "colorondepthsmall_1.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 8, 0.1, 7.9, 1);
	*/

	return EXIT_SUCCESS;
}
