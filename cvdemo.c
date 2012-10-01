#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include "libfreenect_cv.h"

IplImage *GlViewColor(IplImage *depth)
{
	static IplImage *image = 0;
	if (!image) image = cvCreateImage(cvSize(640,480), 8, 3);
	unsigned char *depth_mid = (unsigned char*)(image->imageData);
	int i;
	for (i = 0; i < 640*480; i++) {
		int lb = ((short *)depth->imageData)[i];
		lb %= 256;
		int ub = ((short *)depth->imageData)[i];
		ub /= 256;
		switch (ub) {
			case 0:
				depth_mid[3*i+2] = 255;
				depth_mid[3*i+1] = 255-lb;
				depth_mid[3*i+0] = 255-lb;
				break;
			case 1:
				depth_mid[3*i+2] = 255;
				depth_mid[3*i+1] = lb;
				depth_mid[3*i+0] = 0;
				break;
			case 2:
				depth_mid[3*i+2] = 255-lb;
				depth_mid[3*i+1] = 255;
				depth_mid[3*i+0] = 0;
				break;
			case 3:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 255;
				depth_mid[3*i+0] = lb;
				break;
			case 4:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 255-lb;
				depth_mid[3*i+0] = 255;
				break;
			case 5:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 0;
				depth_mid[3*i+0] = 255-lb;
				break;
			default:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 0;
				depth_mid[3*i+0] = 0;
				break;
		}
	}
	return image;
}

static IplImage *in;
static IplImage *out;
static int x_click = -1;
static int y_click = -1;

void mouseHandler(int event, int x, int y, int flags, void *param)
{
	switch(event) {
	/* left button down */
	case CV_EVENT_LBUTTONDOWN:
		x_click = x;
		y_click = y;
		fprintf(stdout, "Left button down (%d, %d).\n", x, y);
		break;

		/* right button down */
	case CV_EVENT_RBUTTONDOWN:
		fprintf(stdout, "Right button down (%d, %d).\n", x, y);
		break;

		/* mouse move */
	case CV_EVENT_MOUSEMOVE:
		/* draw a rectangle */
		//out = cvCloneImage( in);
		cvRectangle(out,
				cvPoint(x - 15, y - 15),
				cvPoint(x + 15, y + 15),
				cvScalar(0, 0, 255, 0), 2, 8, 0);
		cvShowImage( "Example1-out", out);
		break;
	}
}

void test_1()
{
	in = cvLoadImage( "Ceiling_Tiles.JPG", CV_LOAD_IMAGE_GRAYSCALE);
	cvNamedWindow( "Example1-in", CV_WINDOW_AUTOSIZE);
	cvNamedWindow( "Example1-out", CV_WINDOW_AUTOSIZE);
	IplImage* out = cvCreateImage( cvGetSize(in), IPL_DEPTH_8U, 1);
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);

	cvSetMouseCallback( "Example1-in", mouseHandler, NULL );

	//cvSmooth(in, out, CV_GAUSSIAN, 11, 11, 0, 0);
	//cvCanny( in, out, 10, 100, 3 );

	while ( cvWaitKey( 10) < 0)
	{
		if (-1 != x_click && -1 != y_click)
		{
			char coord_str[8];
			sprintf(coord_str, "%03d,%03d", x_click, y_click);
			cvPutText(in, coord_str, cvPoint(x_click, y_click), &font, cvScalar(255, 255, 255, 0));
			x_click = y_click = -1;
		}
		cvShowImage( "Example1-in", in);
	}

	cvShowImage( "Example1-out", out);

	cvReleaseImage( &in);
	cvReleaseImage( &out);
	cvDestroyWindow( "Example1-in");
	cvDestroyWindow( "Example1-out");
}

int main(int argc, char **argv)
{
	const char* libraries;
	const char* modules;

	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);

	// Using cvGetModuleInfo() to check for IPP
	cvGetModuleInfo( 0, &libraries, &modules );
	printf("Libraries: %s\nModules: %s\n", libraries, modules );

	cvNamedWindow( "Depth", CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback( "Depth", mouseHandler, NULL );

	//test_1();

	while (cvWaitKey(10) < 0) {
		IplImage *image = freenect_sync_get_rgb_cv(0);
		if (!image) {
		    printf("Error: Kinect not connected?\n");
		    return -1;
		}
		cvCvtColor(image, image, CV_RGB2BGR);
		IplImage *depth = freenect_sync_get_depth_cv(0);
		if (!depth) {
		    printf("Error: Kinect not connected?\n");
		    return -1;
		}

		if (-1 != x_click && -1 != y_click)
		{
			char coord_str[] = "640,480,2048";
			int pixel_depth = ((short *)depth->imageData)[y_click * 640 + x_click];
			sprintf(coord_str, "%03d,%03d,%04d", x_click, y_click, pixel_depth);
			cvPutText(depth, coord_str, cvPoint(x_click, y_click), &font, cvScalar(255, 255, 255, 0));
		}

		cvShowImage("RGB", image);
		cvShowImage("Depth", GlViewColor(depth));
		//cvShowImage("Depth", depth);
	}

	return 0;
}
