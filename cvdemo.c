#include <stdio.h>
#include <stdbool.h>
//#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <libfreenect.h>
#include <libfreenect_sync.h>
#include "libfreenect_cv.h"

/*
 * How many frames to process per second depends on the application.
 * Tune this based of the performance of the CPU.
 * Sensor over USB cannot handle more than 30 FPS.
 */
enum {
	PROCESS_FPS = 2,
	CONTOUR_AREA_MIN = 1000, // @TODO automatically determine area for smallest and largest ceiling light
	CONTOUR_AREA_MAX = 10000,
	CONTOUR_AREA_DIFFERENCE 	= 500,	// maximum difference between the potential matching contours
	CONTOUR_POSITION_DIFFERENCE = 100	// maximum
};

// Kinect's maximum tilt (from libfreenect header)
enum { MAX_TILT_ANGLE = 31 };

// Channel index for BGR images
enum {
	BGR_BLUE_INDEX	= 0,
	BGR_GREEN_INDEX	= 1,
	BGR_RED_INDEX	= 3
};


static const char windows_name_rbg[] 	= "RBG";
static const char windows_name_depth[] 	= "Depth";

static int x_click = -1;
static int y_click = -1;

static int canny_low = 80;
static int canny_high = 100;

static bool reinitialize = 0;


struct stats {
	int count;
	double average;
	double min;
	double max;
};
typedef enum stats_array_index {
	RGB_CONTOURS = 0,
	DEPTH_CONTOURS,
	STATS_ARRAY_DIMENSIONS
} stats_array_index;
static struct stats stats_array[STATS_ARRAY_DIMENSIONS] =
{
		{ 0, 0, INT32_MAX, 0.0 },
		{ 0, 0, INT32_MAX, 0.0 }
};

enum { LANDMARK_COUNT_MAX = 10 };
static CvContour potential_landmarks[STATS_ARRAY_DIMENSIONS][LANDMARK_COUNT_MAX];

#if 0
/*
 * From their data, a basic first order approximation for converting the raw
 * 11-bit disparity value to a depth value in centimeters is:
 *   100/(-0.00307 * rawDisparity + 3.33).
 * This approximation is approximately 10 cm off at 4 m away, and less than
 * 2 cm off within 2.5 m.
 */
static float raw_depth_to_meters(int raw_disparity)
{
	float depth = 1.0 / (raw_disparity * -0.0030711016 + 3.3309495161);
	return depth;
}
#else
/*
 * A better approximation given by Stéphane Magnenat:
 *   distance = 0.1236 * tan(rawDisparity / 2842.5 + 1.1863) in meters.
 * Adding a final offset term of -0.037 centers the original ROS data.
 * The tan approximation has a sum squared difference of .33 cm while the
 * 1/x approximation is about 1.7 cm.
 *
 */
static float raw_depth_to_meters(int raw_disparity)
{
	float depth = 0.1236 * tanf(raw_disparity/2842.5 + 1.1863);
	return depth;
}
#endif

/*
 * Color only value for disparity values in the ranges of interest.
 * Specifically, keep only the "no data" range.
 * @todo remove right no data band
 */
static IplImage *kinect_disparity_filter(IplImage *depth)
{
	static IplImage *image = 0;
	if (!image) image = cvCreateImage( cvSize(depth->width, depth->height), IPL_DEPTH_8U, 3); // depth->depth Unsigned 8-bit integer (8u)
	unsigned char *depth_color = (unsigned char*)(image->imageData);
	int i;
	for (i = 0; i < 640*480; i++)
	{
		int raw_disparity = ((short *)depth->imageData)[i];
		if (raw_disparity < 242) // (depth_meters < 0.4)
		{
			// unknown
			depth_color[3*i+BGR_RED_INDEX]	= 0;
			depth_color[3*i+BGR_GREEN_INDEX]	= 0;
			depth_color[3*i+BGR_BLUE_INDEX]	= 0;
		}
		else if (raw_disparity < 658) // (depth_meters < 0.8)
		{
			// too close
			depth_color[3*i+BGR_RED_INDEX]	= 0;
			depth_color[3*i+BGR_GREEN_INDEX]	= 0;
			depth_color[3*i+BGR_BLUE_INDEX]	= 0;
		}
		else if (raw_disparity < 1006) // (depth_meters < 4.0)
		{
			// normal
			depth_color[3*i+BGR_RED_INDEX]	= 0;
			depth_color[3*i+BGR_GREEN_INDEX]	= 0;
			depth_color[3*i+BGR_BLUE_INDEX]	= 0;
		}
		else if (raw_disparity < 1050) // (depth_meters < 8.0)
		{
			// too far
			depth_color[3*i+BGR_RED_INDEX]	= 0;
			depth_color[3*i+BGR_GREEN_INDEX]	= 0;
			depth_color[3*i+BGR_BLUE_INDEX]	= 0;
		}
		else  // if (depth_meters > 8.0)
		{
			// unknown
			depth_color[3*i+BGR_RED_INDEX]	= 0;
			depth_color[3*i+BGR_GREEN_INDEX]	= 255;
			depth_color[3*i+BGR_BLUE_INDEX]	= 0;
		}
		// @TODO experiment with setting anything equal to 2047 to 1 only.
	}
	return image;
}

void mouseHandler(int event, int x, int y, int flags, void *param)
{
	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		/* left button down */
		x_click = x;
		y_click = y;
		//		fprintf(stdout, "Left button down (%d, %d).\n", x, y);
		break;
	case CV_EVENT_LBUTTONDBLCLK:
		break;
	case CV_EVENT_RBUTTONDOWN:
		/* right button down */
		//		fprintf(stdout, "Right button down (%d, %d).\n", x, y);
		break;
	case CV_EVENT_RBUTTONDBLCLK:
		break;
	case CV_EVENT_MOUSEMOVE:
		/* mouse move */
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

static void get_cv_info()
{
	const char* libraries;
	const char* modules;
	// Using cvGetModuleInfo() to check for IPP
	cvGetModuleInfo(0, &libraries, &modules);
	printf("Libraries: %s\nModules: %s\n", libraries, modules);
}

/*
 * Find contours in an image and return a new image with the contours drawn.
 * This function performs a filter on the shapes to identify quadrilaterals
 * in the shape of a ceiling light: is convex, has 4 vertices, and a certain size.
 * TODO: consider adding a slider to the window for adjusting the thresholds
 */
static IplImage* detect_contours(IplImage* img, enum stats_array_index stats_index)
{
	CvSeq* contours; // linked list of contours
	CvSeq* result; // pointer to single contour
	CvMemStorage *storage = cvCreateMemStorage(0); // storage for contour linked list
	static IplImage* ret = NULL;
	if (!ret) ret = cvCreateImage(cvGetSize(img), 8, 1);
	cvZero(ret);
	IplImage* temp = cvCreateImage(cvGetSize(img), 8, 1);
	int i;
	double area;
	int contour_index = 0;
	CvScalar color_pallete[] =
	{
			CV_RGB( 255, 0, 0 ), 	// red
			CV_RGB( 0, 255, 0 ), 	// green
			CV_RGB( 0, 0, 255 ), 	// blue
			CV_RGB( 255, 255, 0 ), 	//
			CV_RGB( 255, 0, 255 ), 	//
			CV_RGB( 0, 255, 255 ), 	//
			CV_RGB( 255, 255, 255 ) // white
	};
	int color_pallete_index_max = sizeof(color_pallete)/sizeof(CvScalar) - 1;

	// reset stats for each frame
	{
		stats_array[stats_index].average = 0;
		stats_array[stats_index].count   = 0;
		stats_array[stats_index].min     = INT32_MAX;
		stats_array[stats_index].max     = 0.0;
	}

	cvCopy(img, temp, NULL);
	//	cvFindContours(temp, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	cvFindContours(temp, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

	// draw contours using the cvDrawContours()
	if (0) // disabled, work in progress
	{
		IplImage* image_all_contours = cvCreateImage(cvGetSize(img), 8, 1);
		cvCopy(img, image_all_contours, NULL);
		int contour = contours; // first contour
		// TODO need for loop to iterate through sequence
		cvDrawContours( image_all_contours, contours, cvScalarAll(255), cvScalarAll(0), 0, CV_FILLED, 8, cvPoint(0,0));

		//cvNamedWindow( "All contours", CV_WINDOW_AUTOSIZE);
		cvShowImage( "All contours", image_all_contours);
		cvReleaseImage(&image_all_contours);
	}

	// iterate through the contour tree and filter out the ceiling lights
	while (contours)
	{
		result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0);
		area = fabs(cvContourArea(result, CV_WHOLE_SEQ, 0));

		if ((4 == result->total)  && // has 4 vertices
				(area > CONTOUR_AREA_MIN) && // has "reasonable" area
				(area < CONTOUR_AREA_MAX) &&
				(cvCheckContourConvexity(result))) // is convex
		{
			CvPoint *pt[4];
			for ( i = 0; i < 4; i++)
				pt[i] = (CvPoint*)cvGetSeqElem(result, i);

			// save the contour as a potential landmark
			{
				CvContour *contour = (CvContour*)result;
				potential_landmarks[stats_index][contour_index] = *contour;
			}

			// keep running statistics for each frame
			if (area)
			{
				stats_array[stats_index].count  += 1;
				stats_array[stats_index].average = (stats_array[stats_index].average + area)/stats_array[stats_index].count;
				stats_array[stats_index].min     = MIN(stats_array[stats_index].min, area);
				stats_array[stats_index].max     = MAX(stats_array[stats_index].max, area);
			}

			// draw contour using the verticies so that we can adjust the color and thickness of each
			{
				//CvScalar line_color = color_pallete[contour_index]; // use with color image
				CvScalar line_color = cvScalarAll(255); // use with black and white image
				int line_thickness = contour_index+1; // vary the thickness so that contours can be distinguished in black and white
				cvLine(ret, *pt[0], *pt[1], line_color, line_thickness, 8, 0);
				cvLine(ret, *pt[1], *pt[2], line_color, line_thickness, 8, 0);
				cvLine(ret, *pt[2], *pt[3], line_color, line_thickness, 8, 0);
				cvLine(ret, *pt[3], *pt[0], line_color, line_thickness, 8, 0);
			}

			fprintf(stdout, "%d. (%03d,%03d) (%03d,%03d) (%03d,%03d) (%03d,%03d) area: %.1f\n", contour_index, pt[0]->x, pt[0]->y, pt[1]->x, pt[1]->y, pt[2]->x, pt[2]->y, pt[3]->x, pt[3]->y, area);

			if ( contour_index < color_pallete_index_max)
				contour_index++; // stop at white
		}

		contours = contours->h_next;
	}

	cvReleaseImage(&temp);
	cvReleaseMemStorage(&storage);

	return ret;
}

/*
 * Handle Key Input
 * Note, after making certain changes (such as shifting) any images that
 * are comprised of multiple samples must be cleared.
 */
static void adjust_offset(char key, int *x_offset, int *y_offset)
{
	switch (key)
	{
	case 81: // left
		*x_offset -= 1;
		reinitialize = true;
		break;
	case 82: // up
		*y_offset -= 1;
		reinitialize = true;
		break;
	case 83: // right
		*x_offset += 1;
		reinitialize = true;
		break;
	case 84: // down
		*y_offset += 1;
		reinitialize = true;
		break;
	case '-': // scale down
		// @todo
		reinitialize = true;
		break;
	case '=': // scale up
		// @todo
		reinitialize = true;
		break;
	case 'c':
		// clear
		reinitialize = true;
		break;
	case -1:
		// no key
	case 'q':
		// main loop will handle exit
	default:
		// nothing to adjust
		break;
	}
}

/**
 * Trying to figure out how this works based on
 * http://opencv.willowgarage.com/documentation/camera_calibration_and_3d_reconstruction.html#findextrinsiccameraparams2
 */
void test_cvFindExtrinsicCameraParams2()
{
	// The array of object points in the object coordinate space, 3xN or Nx3 1-channel, or 1xN or Nx1 3-channel, where N is the number of points.
	CvMat* object_points = cvCreateMat( 3, 3, CV_32FC1);
	// The array of corresponding image points, 2xN or Nx2 1-channel or 1xN or Nx1 2-channel, where N is the number of points.
	CvMat* image_points  = cvCreateMat( 3, 3, CV_32FC1);

	// camera and lens model
	CvMat* intrinsic_matrix  = cvCreateMat( 3, 3, CV_32FC1);
	CvMat* distortion_coeffs = NULL; // If it is NULL, all of the distortion coefficients are set to 0

	// output
	CvMat* rotation_vector    = cvCreateMat( 3, 1, CV_32F);
	CvMat* translation_vector = cvCreateMat( 3, 1, CV_32F);

	// Using the RGB camera intrinsics calculated at http://nicolas.burrus.name/index.php/Research/KinectCalibration
	float fx_rgb = 5.2921508098293293e+02;
	float fy_rgb = 5.2556393630057437e+02;
	float cx_rgb = 3.2894272028759258e+02;
	float cy_rgb = 2.6748068171871557e+02;

	// Camera intrinsic matrix:
	//     [ fx, 0,  cx ]
	// K = [ 0,  fx, cy ]
	//     [ 0,  0,  1  ]
	*( (float*)CV_MAT_ELEM_PTR( *intrinsic_matrix, 0, 0 ) ) = fx_rgb;
	*( (float*)CV_MAT_ELEM_PTR( *intrinsic_matrix, 1, 1 ) ) = fy_rgb;
	*( (float*)CV_MAT_ELEM_PTR( *intrinsic_matrix, 0, 2 ) ) = cx_rgb;
	*( (float*)CV_MAT_ELEM_PTR( *intrinsic_matrix, 1, 2 ) ) = cy_rgb;
	*( (float*)CV_MAT_ELEM_PTR( *intrinsic_matrix, 2, 2 ) ) = 1.0;

	cvFindExtrinsicCameraParams2( object_points, image_points, intrinsic_matrix, distortion_coeffs, rotation_vector, translation_vector, 0);

	// release the data
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&intrinsic_matrix);
	cvReleaseMat(&distortion_coeffs);
	cvReleaseMat(&rotation_vector);
	cvReleaseMat(&translation_vector);
}
void test_cvFindHomography()
{
	CvPoint2D32f src[4], dst[4]; // source and destination 2D coordinates

}

// experimenting with http://opencv.willowgarage.com/documentation/operations_on_arrays.html#cvSolve
void test_cvSolve()
{
	// coefficient matrix
	CvMat* matA = cvCreateMat( 2, 2, CV_32FC1 );
	cvSetZero(matA); // set all to zero in case we forgot to set an element
	*( (float*)CV_MAT_ELEM_PTR( *matA, 0, 0 ) ) = 3;
	*( (float*)CV_MAT_ELEM_PTR( *matA, 0, 1 ) ) = -1;
	*( (float*)CV_MAT_ELEM_PTR( *matA, 1, 0 ) ) = -5;
	*( (float*)CV_MAT_ELEM_PTR( *matA, 1, 1 ) ) = 4;
	CvMat* matB = cvCreateMat( 2, 1, CV_32FC1 );
	cvSetZero(matB);
	*( (float*)CV_MAT_ELEM_PTR( *matB, 0, 0 ) ) = 7;
	*( (float*)CV_MAT_ELEM_PTR( *matB, 1, 0 ) ) = -2;
	CvMat* matX = cvCreateMat( 2, 1, CV_32FC1 );
	cvSetZero(matX);
	int return_code = 0;


	return_code = cvSolve( matA, matB, matX, CV_LU);

	float x = CV_MAT_ELEM( *matX, float, 0, 0 );
	float y = CV_MAT_ELEM( *matX, float, 1, 0 );

}
/**
 * Quadrilateral Centroid Algorithm
 *
 * @author Filipi Vianna
 * @ref http://filipivianna.blogspot.com/2009/11/quadrilateral-centroid-algorithm.html
 */
void test_findCentroid()
{
	float verticesX[5];
	float verticesY[5];
	float centroidX = 0;
	float centroidY = 0;

	verticesX[0] = 1; verticesY[0] = 1;
	verticesX[1] = 2; verticesY[1] = 1;
	verticesX[2] = 2; verticesY[2] = 2;
	verticesX[3] = 1; verticesY[3] = 2;
	verticesX[4] = verticesX[0]; verticesY[4] = verticesY[0]; // Repeat the first vertex

	int i, k;
	float area = 0.0f;
	float tmp = 0.0f;

	for (i = 0; i <= 4; i++){
		k = (i + 1) % (4 + 1);
		tmp = verticesX[i] * verticesY[k] -
				verticesX[k] * verticesY[i];
		area += tmp;
		centroidX += (verticesX[i] + verticesX[k]) * tmp;
		centroidY += (verticesY[i] + verticesY[k]) * tmp;
	}
	area *= 0.5f;
	centroidX *= 1.0f / (6.0f * area);
	centroidY *= 1.0f / (6.0f * area);

	printf("Centroid = (%1.2f, %1.2f),  area = %1.2f\n", centroidX, centroidY, area);
}

void test_dilateQuadAboutCenter()
{
	CvPoint2D32f ptA;
	CvPoint2D32f ptB;
	CvPoint2D32f ptC;
	CvPoint2D32f ptD;
	CvPoint2D32f center;
	float scale = 2;

	ptA.x = 1; ptA.y = 1;
	ptB.x = 2; ptB.y = 1;
	ptC.x = 2; ptC.y = 2;
	ptD.x = 1; ptD.y = 2;

	center.x = 1.5; center.y = 1.5;

	CvPoint2D32f pt2A;
	CvPoint2D32f pt2B;
	CvPoint2D32f pt2C;
	CvPoint2D32f pt2D;

	pt2A.x = ptA.x*scale - center.x; pt2A.y = ptA.y*scale - center.y;
	pt2B.x = ptB.x*scale - center.x; pt2B.y = ptB.y*scale - center.y;
	pt2C.x = ptC.x*scale - center.x; pt2C.y = ptC.y*scale - center.y;
	pt2D.x = ptD.x*scale - center.x; pt2D.y = ptD.y*scale - center.y;
}

int main(int argc, char *argv[])
{
	CvFont font;
	char key;
	int x_offset = 0;
	int y_offset = 0;
	IplImage *image_mask_smooth = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	cvZero(image_mask_smooth);
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);
	cvNamedWindow( windows_name_rbg, CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback( windows_name_rbg, mouseHandler, NULL );
	freenect_raw_tilt_state *state = 0;

	//	get_cv_info();

	// Camera Calibration
	//	test_cvFindExtrinsicCameraParams2();

	test_cvSolve();

	test_cvFindHomography();

	test_findCentroid();

	test_dilateQuadAboutCenter();

	if (freenect_sync_set_tilt_degs(MAX_TILT_ANGLE, 0)) {
		printf("Error: Kinect not connected?\n");
		return -1;
	}

	// wait for motor to stop moving before capturing images
	do {
		if (freenect_sync_get_tilt_state(&state, 0)) {
			printf("Error: Kinect not connected?\n");
			return -1;
		}
	} while (TILT_STATUS_MOVING == state->tilt_status);

	sleep(1); // @bug motor doesn't report correct state

	// process frames indefinitely at the rate defined by PROCESS_FPS
	// quit when user presses 'q'
	while (key != 'q')
	{
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
		IplImage *image_depth = kinect_disparity_filter(image_disparity);

		// shift depth image
		IplImage *image_depth_shifted = cvCreateImage( cvGetSize(image_depth), IPL_DEPTH_8U, 3);
		if (x_offset >= 0 && y_offset >= 0)
		{
			cvSetImageROI(image_depth_shifted, cvRect(abs(x_offset), abs(y_offset), 640, 480) );
			cvSetImageROI(image_depth, cvRect(0, 0, 640-abs(x_offset), 480-abs(y_offset)) );
		}
		else if (x_offset >= 0 && y_offset < 0)
		{
			cvSetImageROI(image_depth_shifted, cvRect( abs(x_offset), 0, 640-abs(x_offset), 480-abs(y_offset) ) );
			cvSetImageROI(image_depth, cvRect( 0, abs(y_offset), 640-abs(x_offset), 480-abs(y_offset) ) );
		}
		else if (x_offset < 0 && y_offset >= 0)
		{
			cvSetImageROI(image_depth_shifted, cvRect( 0, abs(y_offset), 640-abs(x_offset), 480-abs(y_offset) ) );
			cvSetImageROI(image_depth, cvRect( abs(x_offset), 0, 640-abs(x_offset), 480-abs(y_offset) ) );
		}
		else if (x_offset < 0 && y_offset < 0)
		{
			cvSetImageROI(image_depth_shifted, cvRect(0, 0, 640-abs(x_offset), 480-abs(y_offset)));
			cvSetImageROI(image_depth, cvRect(abs(x_offset),abs(y_offset),640-abs(x_offset),480-abs(y_offset)));
		}
		cvCopy( image_depth, image_depth_shifted, NULL);
		cvResetImageROI(image_depth_shifted);
		cvResetImageROI(image_depth);
		image_depth = image_depth_shifted;

		// blend RGB and disparity frames after shift
		IplImage* image_blended = cvCreateImage( cvGetSize(image_depth), IPL_DEPTH_8U, 3);
		cvAddWeighted(image_rgb, 1.0, image_depth, 1.0, 0.0, image_blended);
		cvShowImage( "Blended", image_blended);

		// create binary image of disparity
		IplImage *image_depth_gray = cvCreateImage( cvGetSize(image_depth), IPL_DEPTH_8U, 1);
		cvCvtColor( image_depth, image_depth_gray, CV_RGB2GRAY);
		IplImage *image_mask = cvCreateImage( cvGetSize(image_depth_gray), IPL_DEPTH_8U, 1);
		cvThreshold(image_depth_gray, image_mask, 128, 255, CV_THRESH_BINARY);

		if (reinitialize)
		{
			int i;
			reinitialize = false;
			// clear statistics
			for (i = 0; i < STATS_ARRAY_DIMENSIONS; ++i)
			{
				stats_array[i].average = 0;
				stats_array[i].count   = 0;
				stats_array[i].min     = INT32_MAX;
				stats_array[i].max     = 0.0;
			}
			// initialize the mask
			cvZero(image_mask_smooth);
		}

		// take multiple samples of the depth
		// this fills in the depth mask
		cvAdd( image_mask, image_mask_smooth, image_mask_smooth, NULL); // adding multiple samples _grows_ the mask
		//		cvAnd( image_mask, image_mask_smooth, image_mask_smooth, NULL); // anding multiple samples _shrinks_ the mask
		cvShowImage("Mask Smooth", image_mask_smooth);

#if 0
		IplImage *image_rgb_masked = cvCreateImage(cvGetSize(image_rgb), IPL_DEPTH_8U, 3);
		cvCopy(image_rgb, image_rgb_masked, image_mask);
		cvShowImage("RGB Mask", image_rgb_masked);
#endif

		// find polygons in the disparity data
		fprintf(stdout, "Disparity Contours (X,Y)\n");
		IplImage* disparity_contours = cvCreateImage( cvGetSize(image_mask_smooth), IPL_DEPTH_8U, 1);
		cvCopy( detect_contours(image_mask_smooth, RGB_CONTOURS), disparity_contours, NULL);
		cvShowImage("Disparity Contours", disparity_contours);

		// find polygons in the RGB data
		fprintf(stdout, "RGB Contours (X,Y)\n");
		cvSmooth(image_rgb, image_rgb, CV_GAUSSIAN, 5, 5, 0, 0);
		IplImage* image_gray = cvCreateImage(cvGetSize(image_rgb), IPL_DEPTH_8U, 1);
		cvCvtColor(image_rgb, image_gray, CV_RGB2GRAY);

		IplImage* image_edges = cvCreateImage(cvGetSize(image_gray), IPL_DEPTH_8U, 1);
		cvCanny(image_gray, image_edges, canny_low, canny_high, 3);
		cvSmooth(image_edges, image_edges, CV_GAUSSIAN, 5, 5, 0, 0);

		cvShowImage("Edges", image_edges);
		IplImage* rgb_contours = cvCreateImage(cvGetSize(image_edges), IPL_DEPTH_8U, 1);
		cvCopy( detect_contours(image_edges, DEPTH_CONTOURS), rgb_contours, NULL);
		cvShowImage("RGB Contours", rgb_contours);

		//		CvSeq* results = cvHoughLines2(image_edges, storage, CV_HOUGH_STANDARD, 2.0, 2.0, image_edges->width / 10, 0, 0);

		// find matching contours
		if (0) {
			int rgb_index;
			int depth_index;
			for ( rgb_index = 0; rgb_index < LANDMARK_COUNT_MAX; ++rgb_index)
			{
				for ( depth_index = 0; depth_index < LANDMARK_COUNT_MAX; ++depth_index)
				{
					double rgb_area = fabs(cvContourArea( &potential_landmarks[RGB_CONTOURS][rgb_index], CV_WHOLE_SEQ, 0));
					double depth_area = fabs(cvContourArea( &potential_landmarks[DEPTH_CONTOURS][depth_index], CV_WHOLE_SEQ, 0));
					double area_difference = fabs(rgb_area - depth_area);
					if ( CONTOUR_AREA_DIFFERENCE > area_difference)
						// we have a match
						break;
				}
			}
		}

		if (-1 != x_click && -1 != y_click)
		{
			char coord_str[] = "640,480,-01.234";
			int coord_str_len = strlen(coord_str);
			int pixel_disparity = ((short *) image_disparity->imageData)[y_click * 640 + x_click];
			sprintf(coord_str, "%03d,%03d,%04d", x_click, y_click, pixel_disparity);
			//			float pixel_depth_meters = raw_depth_to_meters(pixel_disparity);
			//			sprintf(coord_str, "%03d,%03d,%02.03f", x_click, y_click, pixel_depth_meters);
			coord_str[coord_str_len] = '\0';
			cvPutText(image_rgb, coord_str, cvPoint(x_click, y_click), &font, cvScalar(255, 255, 255, 0));
		}
		cvShowImage(windows_name_rbg, image_rgb);
		//		cvShowImage(windows_name_depth, GlViewColor(depth));
		cvShowImage(windows_name_depth, image_depth);
		//		cvShowImage(windows_name_depth, depth);

		// wait for a key and time delay
		key = cvWaitKey(1000/PROCESS_FPS);
		// shift depth image if necessary
		adjust_offset(key, &x_offset, &y_offset);
	}

	// return the camera horizontal tilt
	freenect_sync_set_tilt_degs(0, 0);

	return 0;
}
