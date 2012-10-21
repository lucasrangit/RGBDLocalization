#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include "libfreenect_cv.h"

static IplImage *in;
//static IplImage *out;
static int x_click = -1;
static int y_click = -1;

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

/*
 * From their data, a basic first order approximation for converting the raw
 * 11-bit disparity value to a depth value in centimeters is:
 *   100/(-0.00307 * rawDisparity + 3.33).
 * This approximation is approximately 10 cm off at 4 m away, and less than
 * 2 cm off within 2.5 m.
 */
float raw_depth_to_meters_0(int raw_disparity)
{
	if (raw_disparity < 2047)
	{
		return 1.0 / (raw_disparity * -0.0030711016 + 3.3309495161);
	}
	return 0;
}

/*
 * A better approximation given by Stéphane Magnenat:
 *   distance = 0.1236 * tan(rawDisparity / 2842.5 + 1.1863) in meters.
 * Adding a final offset term of -0.037 centers the original ROS data.
 * The tan approximation has a sum squared difference of .33 cm while the
 * 1/x approximation is about 1.7 cm.
 *
 */
float raw_depth_to_meters(int raw_disparity)
{
	float depth = 0.1236 * tanf(raw_disparity/2842.5 + 1.1863);
	return depth;
}

enum {BLUE_INDEX=0, GREEN_INDEX=1, RED_INDEX=3};
IplImage *kinect_depth_histogram(IplImage *depth)
{
	static IplImage *image = 0;
	if (!image) image = cvCreateImage( cvSize(depth->width, depth->height), IPL_DEPTH_8U, 3); // depth->depth Unsigned 8-bit integer (8u)
	unsigned char *depth_color = (unsigned char*)(image->imageData);
	int i;
	for (i = 0; i < 640*480; i++)
	{
		int raw_disparity = ((short *)depth->imageData)[i];
		//float depth_meters = raw_depth_to_meters(raw_depth);
		if (raw_disparity < 242) // (depth_meters < 0.4)
		{
			// unknown
			depth_color[3*i+RED_INDEX]	= 0;
			depth_color[3*i+GREEN_INDEX]	= 0;
			depth_color[3*i+BLUE_INDEX]	= 0;
		}
		else if (raw_disparity < 658) // (depth_meters < 0.8)
		{
			// too close
			depth_color[3*i+RED_INDEX]	= 255;
			depth_color[3*i+GREEN_INDEX]	= 255;
			depth_color[3*i+BLUE_INDEX]	= 255;
		}
		else if (raw_disparity < 1006) // (depth_meters < 4.0)
		{
			// normal
			depth_color[3*i+RED_INDEX]	= 0;
			depth_color[3*i+GREEN_INDEX]	= 0;
			depth_color[3*i+BLUE_INDEX]	= (raw_disparity % (1006-255)) * (255/(1006-(1006-255)));
		}
		else if (raw_disparity < 1050) // (depth_meters < 8.0)
		{
			// too far
			depth_color[3*i+RED_INDEX]	= 255;
			depth_color[3*i+GREEN_INDEX]	= 0;
			depth_color[3*i+BLUE_INDEX]	= 0;
		}
		else  // if (depth_meters > 8.0)
		{
			// unknown
			depth_color[3*i+RED_INDEX]	= 0;
			depth_color[3*i+GREEN_INDEX]	= 0;
			depth_color[3*i+BLUE_INDEX]	= 0;
		}
	}
	return image;
}

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
#if 0
		/* draw a rectangle */
		//out = cvCloneImage( in);
		cvRectangle(out,
				cvPoint(x - 15, y - 15),
				cvPoint(x + 15, y + 15),
				cvScalar(0, 0, 255, 0), 2, 8, 0);
		cvShowImage( "Example1-out", out);
		break;
#endif
	default:
		/* unhandled event */
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

void get_cv_info() {
	const char* libraries;
	const char* modules;
	// Using cvGetModuleInfo() to check for IPP
	cvGetModuleInfo(0, &libraries, &modules);
	printf("Libraries: %s\nModules: %s\n", libraries, modules);
}

#define WINDOW_RGB "RGB"
#define WINDOW_DEPTH "Depth"

int main(int argc, char **argv)
{
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);
	cvNamedWindow( WINDOW_RGB, CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback( WINDOW_RGB, mouseHandler, NULL );

//	get_cv_info();

//	test_1();

	while (cvWaitKey(500) < 0) {
		IplImage *image_rgb = freenect_sync_get_rgb_cv(0);
		if (!image_rgb) {
		    printf("Error: Kinect not connected?\n");
		    return -1;
		}
		cvCvtColor(image_rgb, image_rgb, CV_RGB2BGR);

		IplImage *image_disparity = freenect_sync_get_depth_cv(0);
		if (!image_disparity) {
		    printf("Error: Kinect not connected?\n");
		    return -1;
		}

		IplImage *image_depth = kinect_depth_histogram(image_disparity);

		IplImage* image_blended = cvCreateImage( cvGetSize(image_depth), IPL_DEPTH_8U, 3);
		cvAddWeighted(image_rgb, 1.0, image_depth, 1.0, 0.0, image_blended);
		cvShowImage( "Blended", image_blended);

		IplImage *image_depth_gray = cvCreateImage( cvGetSize(image_depth), IPL_DEPTH_8U, 1);
		cvCvtColor( image_depth, image_depth_gray, CV_RGB2GRAY);
		IplImage *image_mask = cvCreateImage( cvGetSize(image_depth_gray), IPL_DEPTH_8U, 1);
		cvThreshold(image_depth_gray, image_mask, 128, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

		cvSmooth(image_rgb, image_rgb, CV_GAUSSIAN, 5, 5, 0, 0);
		IplImage* image_gray = cvCreateImage( cvGetSize(image_rgb), IPL_DEPTH_8U, 1);
		cvCvtColor( image_rgb, image_gray, CV_RGB2GRAY);
		IplImage* image_edges = cvCreateImage( cvGetSize(image_gray), IPL_DEPTH_8U, 1);
		cvCanny( image_gray, image_edges, 10, 100, 3 );
		cvShowImage( "Edges", image_edges);

		CvSeq* results = cvHoughLines2(
				image_edges,
				storage,
				CV_HOUGH_STANDARD,
				2.0, 2.0,
				image_edges->width/10,
				0, 0
		);


		if (-1 != x_click && -1 != y_click)
		{
			char coord_str[] = "640,480,-01.234";
			int coord_str_len = strlen(coord_str);
			int pixel_disparity = ((short *)image_disparity->imageData)[y_click * 640 + x_click];
			sprintf(coord_str, "%03d,%03d,%04d", x_click, y_click, pixel_disparity);
//			float pixel_depth_meters = raw_depth_to_meters(pixel_disparity);
//			sprintf(coord_str, "%03d,%03d,%02.03f", x_click, y_click, pixel_depth_meters);
			coord_str[coord_str_len] = '\0';
			cvPutText(image_rgb, coord_str, cvPoint(x_click, y_click), &font, cvScalar(255, 255, 255, 0));
		}

		cvShowImage(WINDOW_RGB, image_rgb);
//		cvShowImage(WINDOW_DEPTH, GlViewColor(depth));
		cvShowImage(WINDOW_DEPTH, image_depth);
		//cvShowImage(WINDOW_DEPTH, depth);
	}

	return 0;
}
