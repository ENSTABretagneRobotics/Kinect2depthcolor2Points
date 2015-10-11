#include "OSMisc.h"
#include "imatrix.h"
#include "CvCore.h"

int depthcolor2PLYKinect2SDK(char* depthimgfilename, char* colorimgfilename, 
							 double x, double y, double z, double phi, double theta, double psi, 
							 double HorizontalBeam, double VerticalBeam)
{
	char szTemp[256];
	char plyfilename[256];

	IplImage* depthimg = cvLoadImage(depthimgfilename, 1);
	IplImage* colorimg = cvLoadImage(colorimgfilename, 1);

	unsigned int nbpoints = (unsigned int)depthimg->width*(unsigned int)depthimg->height;
	double fx = tan(HorizontalBeam*0.5*M_PI/180.0)*2;
	double fy = tan(VerticalBeam*0.5*M_PI/180.0)*2;

	// Angles order...?
	rmatrix R = RotationPhiThetaPsi(psi, phi, theta);

	strcpy(szTemp, depthimgfilename);
	RemoveExtensionInFilePath(szTemp);
	sprintf(plyfilename, "%.249s.ply", szTemp);

	FILE* f = fopen(plyfilename, "w");

	fprintf(f, "ply\n");
	fprintf(f, "format ascii 1.0\n");
	fprintf(f, "element vertex %u\n", nbpoints);
	fprintf(f, "property float x\n");
	fprintf(f, "property float y\n");
	fprintf(f, "property float z\n");
	fprintf(f, "property uchar red\n");
	fprintf(f, "property uchar green\n");
	fprintf(f, "property uchar blue\n");
	//fprintf(f, "property float nx\n");
	//fprintf(f, "property float ny\n");
	//fprintf(f, "property float nz\n");
	//fprintf(f, "property float radius\n");
	fprintf(f, "element face 0\n");
	fprintf(f, "property list uchar int vertex_indices\n");
	fprintf(f, "end_header\n");
	
	int i, j, index;
	unsigned char* depthdata = (unsigned char*)depthimg->imageData;
	unsigned char* colordata = (unsigned char*)colorimg->imageData;
	for (i = 0; i < depthimg->height; i++)
	{
		for (j = 0; j < depthimg->width; j++)
		{
			double xp = 0, yp = 0, zp = 0;
			index = 3*(j+depthimg->width*i);
			zp = (255.0-depthdata[index])*8.0/255.0;
			yp = -fy*zp*(i/(double)depthimg->height-0.5);
			xp = fx*zp*(j/(double)depthimg->width-0.5);

			rmatrix p = rmatrix(box(yp-y, zp-z, xp-x));
			box q = ToBox(R*p);

			fprintf(f, "%f %f %f %d %d %d\n", 
				Center(q[3])+z, Center(q[1])+x, Center(q[2])+y, 
				colordata[index+2], colordata[index+1], colordata[index]);
		}
	}

	fclose(f);

	cvReleaseImage(&colorimg);
	cvReleaseImage(&depthimg);
	
	return EXIT_SUCCESS;
}

int depthcolor2OBJKinect2SDK(char* depthimgfilename, char* colorimgfilename, 
							 double x, double y, double z, double phi, double theta, double psi, 
							 double HorizontalBeam, double VerticalBeam, double dmin, double dmax, double maxfacesize)
{
	char szTemp[256];
	char objfilename[256];
	char mtlfilename[256];
	char szMTLName[256];
	char colorimgrelativefilename[256];

	IplImage* depthimg = cvLoadImage(depthimgfilename, 1);

	//unsigned int nbpoints = (unsigned int)depthimg->width*(unsigned int)depthimg->height;
	double fx = tan(HorizontalBeam*0.5*M_PI/180.0)*2;
	double fy = tan(VerticalBeam*0.5*M_PI/180.0)*2;

	// Angles order...?
	rmatrix R = RotationPhiThetaPsi(psi, phi, theta);
	
	strcpy(szTemp, depthimgfilename);
	RemoveExtensionInFilePath(szTemp);
	sprintf(objfilename, "%.249s.obj", szTemp);
	sprintf(mtlfilename, "%.249s.mtl", szTemp);
	strcpy(szMTLName, szTemp);
	RemovePathInFilePath(szMTLName);

	FILE* fobj = fopen(objfilename, "w");

	fprintf(fobj, "mtllib %.249s.mtl\n", szMTLName);	

	int i, j, index;
	unsigned char* depthdata = (unsigned char*)depthimg->imageData;
	for (i = 0; i < depthimg->height; i++)
	{
		for (j = 0; j < depthimg->width; j++)
		{
			double xp = 0, yp = 0, zp = 0;
			index = 3*(j+depthimg->width*i);
			zp = (255.0-depthdata[index])*8.0/255.0;
			yp = -fy*zp*(i/(double)depthimg->height-0.5);
			xp = fx*zp*(j/(double)depthimg->width-0.5);

			rmatrix p = rmatrix(box(yp-y, zp-z, xp-x));
			box q = ToBox(R*p);
	
			fprintf(fobj, "v %f %f %f\n", Center(q[3])+z, Center(q[1])+x, Center(q[2])+y);
		}
	}
 
	for (i = 0; i < depthimg->height; i++)
	{
		for (j = 0; j < depthimg->width; j++)
		{
			double u = j/(double)depthimg->width, v = (depthimg->height-1-i)/(double)depthimg->height;
			fprintf(fobj, "vt %f %f\n", u, v);
		}
	}
 
	fprintf(fobj, "usemtl %.255s\n", szMTLName);

	for (i = 0; i < depthimg->height-1; i++)
	{
		for (j = 0; j < depthimg->width-1; j++)
		{
			int b = (j+depthimg->width*i);
			int a = ((j+1)+depthimg->width*i);
			int c = (j+depthimg->width*(i+1));
			int d = ((j+1)+depthimg->width*(i+1));

			double za = (255.0-depthdata[3*a])*8.0/255.0;
			double ya = -fy*za*(i/(double)depthimg->height-0.5);
			double xa = fx*za*((j+1)/(double)depthimg->width-0.5);
			double zb = (255.0-depthdata[3*b])*8.0/255.0;
			double yb = -fy*zb*(i/(double)depthimg->height-0.5);
			double xb = fx*zb*(j/(double)depthimg->width-0.5);
			double zc = (255.0-depthdata[3*c])*8.0/255.0;
			double yc = -fy*zc*((i+1)/(double)depthimg->height-0.5);
			double xc = fx*zc*(j/(double)depthimg->width-0.5);
			double zd = (255.0-depthdata[3*d])*8.0/255.0;
			double yd = -fy*zd*((i+1)/(double)depthimg->height-0.5);
			double xd = fx*zd*((j+1)/(double)depthimg->width-0.5);

			// Order because of Backface Culling...
			if (
				(za > dmin)&&(za < dmax)&&(zb > dmin)&&(zb < dmax)&&(zc > dmin)&&(zc < dmax)&&
				(sqrt(sqr(xa-xb)+sqr(ya-yb)+sqr(za-zb)) < maxfacesize)&&
				(sqrt(sqr(xa-xc)+sqr(ya-yc)+sqr(za-zc)) < maxfacesize)&&
				(sqrt(sqr(xb-xc)+sqr(yb-yc)+sqr(zb-zc)) < maxfacesize)				
				)
				fprintf(fobj, "f %d/%d %d/%d %d/%d\n", c+1, c+1, b+1, b+1, a+1, a+1);
			if (
				(za > dmin)&&(za < dmax)&&(zd > dmin)&&(zd < dmax)&&(zc > dmin)&&(zc < dmax)&&
				(sqrt(sqr(xa-xc)+sqr(ya-yc)+sqr(za-zc)) < maxfacesize)&&
				(sqrt(sqr(xa-xd)+sqr(ya-yd)+sqr(za-zd)) < maxfacesize)&&
				(sqrt(sqr(xc-xd)+sqr(yc-yd)+sqr(zc-zd)) < maxfacesize)
				)
				fprintf(fobj, "f %d/%d %d/%d %d/%d\n", d+1, d+1, c+1, c+1, a+1, a+1);
		}
	}

	fclose(fobj);

	FILE* fmtl = fopen(mtlfilename, "w");
	fprintf(fmtl, "newmtl %.255s\n", szMTLName);

	strcpy(colorimgrelativefilename, colorimgfilename);
	RemovePathInFilePath(colorimgrelativefilename);

	fprintf(fmtl, "map_Kd %.255s\n", colorimgrelativefilename);
	fclose(fmtl);

	cvReleaseImage(&depthimg);

	return EXIT_SUCCESS;
}

int main(int argc, char* argv[])
{
	char szFileDepthPath[256];
	char szFileColorPath[256];
	double x = 0, y = 0, z = 0, phi = 0, theta = 0, psi = 0, HorizontalBeam = 70.6, VerticalBeam = 53.8;
	double dmin = 0.1, dmax = 7.9, maxfacesize = 1;

	if (argc != 14)
	{
		strcpy(szFileDepthPath, "depth.png");
		strcpy(szFileColorPath, "color.png");
		printf("Warning : No parameter specified.\n");
		printf("Usage : Kinect2depthcolor2Points depth.png color.png x y z phi theta psi HorizontalBeam VerticalBeam dmin dmax maxfacesize.\n");
		printf("Default : Kinect2depthcolor2Points %.255s %.255s %f %f %f %f %f %f %f %f %f %f %f.\n", 
			szFileDepthPath, szFileColorPath, x, y, z, phi, theta, psi, HorizontalBeam, VerticalBeam, dmin, dmax, maxfacesize);
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
		HorizontalBeam = atof(argv[9]);
		VerticalBeam = atof(argv[10]);
		dmin = atof(argv[11]);
		dmax = atof(argv[12]);
		maxfacesize = atof(argv[13]);
	}

	depthcolor2PLYKinect2SDK(szFileDepthPath, szFileColorPath, x, y, z, phi, theta, psi, HorizontalBeam, VerticalBeam);
	depthcolor2OBJKinect2SDK(szFileDepthPath, szFileColorPath, x, y, z, phi, theta, psi, HorizontalBeam, VerticalBeam, dmin, dmax, maxfacesize);

/*
	// Temporary...

	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h42min39s.png", "snap0_2015-08-14_22h42min39s.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h43min37s.png", "snap0_2015-08-14_22h43min37s.png", 0.170, 0.150, 0.060, 0, 0, 0, 70.6, 53.8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h44min04s.png", "snap0_2015-08-14_22h44min04s.png", 0.170, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h44min27s.png", "snap0_2015-08-14_22h44min27s.png", 0.170, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8);
	
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h42min39s.png", "snap0_2015-08-14_22h42min39s.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h43min37s.png", "snap0_2015-08-14_22h43min37s.png", 0.170, 0.150, 0.060, 0, 0, 0, 70.6, 53.8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h44min04s.png", "snap0_2015-08-14_22h44min04s.png", 0.170, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h44min27s.png", "snap0_2015-08-14_22h44min27s.png", 0.170, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8, 0.1, 7.9, 1);
*/
	/*
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h42min39s.png", "snap0_2015-08-14_22h42min39s.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h43min37s.png", "snap0_2015-08-14_22h43min37s.png", 0.170, 0.150, 0.060, 0, 0, 0, 70.6, 53.8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h44min04s.png", "snap0_2015-08-14_22h44min04s.png", 0.170, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h44min27s.png", "snap0_2015-08-14_22h44min27s.png", 0.170, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8);

	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h45min51s.png", "snap0_2015-08-14_22h45min51s.png", 0.260, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h46min14s.png", "snap0_2015-08-14_22h46min14s.png", 0.260, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8);
	depthcolor2PLYKinect2SDK("snap1_2015-08-14_22h46min30s.png", "snap0_2015-08-14_22h46min30s.png", 0.260, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8);
	
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h42min39s.png", "snap0_2015-08-14_22h42min39s.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h43min37s.png", "snap0_2015-08-14_22h43min37s.png", 0.170, 0.150, 0.060, 0, 0, 0, 70.6, 53.8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h44min04s.png", "snap0_2015-08-14_22h44min04s.png", 0.170, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h44min27s.png", "snap0_2015-08-14_22h44min27s.png", 0.170, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8, 0.1, 7.9, 1);

	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h45min51s.png", "snap0_2015-08-14_22h45min51s.png", 0.260, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h46min14s.png", "snap0_2015-08-14_22h46min14s.png", 0.260, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("snap1_2015-08-14_22h46min30s.png", "snap0_2015-08-14_22h46min30s.png", 0.260, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8, 0.1, 7.9, 1);

	
	depthcolor2PLYKinect2SDK("depth1.png", "colorondepth1.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8);
	depthcolor2PLYKinect2SDK("depth2.png", "colorondepth2.png", 0.170, 0.150, 0.060, 0, 0, 0, 70.6, 53.8);
	depthcolor2PLYKinect2SDK("depth3.png", "colorondepth3.png", 0.170, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8);
	depthcolor2PLYKinect2SDK("depth4.png", "colorondepth4.png", 0.170, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8);

	depthcolor2OBJKinect2SDK("depth1.png", "colorondepth1.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("depth2.png", "colorondepth2.png", 0.170, 0.150, 0.060, 0, 0, 0, 70.6, 53.8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("depth3.png", "colorondepth3.png", 0.170, 0.150, 0.060, 0, 0, M_PI, 70.6, 53.8, 0.1, 7.9, 1);
	depthcolor2OBJKinect2SDK("depth4.png", "colorondepth4.png", 0.170, 0.150, 0.060, 0, 0, -M_PI/2, 70.6, 53.8, 0.1, 7.9, 1);


	depthcolor2PLYKinect2SDK("depthsmall_1.png", "colorondepthsmall_1.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8);

	depthcolor2OBJKinect2SDK("depthsmall_1.png", "colorondepthsmall_1.png", 0.170, 0.150, 0.060, 0, 0, M_PI/2, 70.6, 53.8, 0.1, 7.9, 1);
	*/

	return EXIT_SUCCESS;
}
