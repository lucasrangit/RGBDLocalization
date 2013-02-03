#include "rgbdlocalization.h"
#include "helpers.h"

static struct stats stats_array[STATS_ARRAY_DIMENSIONS] =
{
		{ 0, 0, INT32_MAX, 0.0 },
		{ 0, 0, INT32_MAX, 0.0 }
};

static CvContour potential_landmarks[STATS_ARRAY_DIMENSIONS][LANDMARK_COUNT_MAX];

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
//		CvSeq* contour = contours; // first contour
		// TODO need for loop to iterate through sequence
		cvDrawContours( image_all_contours, contours, cvScalarAll(255), cvScalarAll(0), 0, CV_FILLED, 8, cvPoint(0,0));

		//cvNamedWindow( "All contours", CV_WINDOW_AUTOSIZE);
		cvShowImage( "All contours", image_all_contours);
		cvReleaseImage(&image_all_contours);
	}

	// iterate through the contour tree and filter out the ceiling lights
	while (contours)
	{
		// Approximate a polygon around contour using the Douglas-Peucker (DP) approximation.
		// Pass a zero as the last argument instructs cvApproxPoly to only operate on the first
		// element of the sequence.
		result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.09, 0);
		area = fabs(cvContourArea(result, CV_WHOLE_SEQ, 0));

		if (4 == result->total)
		{
			// has 4 vertices
			if ((area > CONTOUR_AREA_MIN) &&
				(area < CONTOUR_AREA_MAX))
			{
				// has "reasonable" area
				if (cvCheckContourConvexity(result))
				{
					// is convex
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

					// draw contour using the vertices so that we can adjust the color and thickness of each
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
			}
		}

		contours = contours->h_next;
	}

	cvReleaseImage(&temp);
	cvReleaseMemStorage(&storage);

	return ret;
}

int main(int argc, char *argv[])
{
	IplImage *image_rgb = NULL;
	IplImage *image_disparity = NULL;
	IplImage *image_depth = NULL;
	IplImage* image_blended = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 3);
	IplImage *image_depth_gray = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *image_mask = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	//IplImage *image_rgb_masked = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 3);
	IplImage *image_mask_smooth = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage* disparity_contours = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage* image_gray = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage* image_edges = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage* rgb_contours = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	CvFont font;
	char key;
	int x_offset = 0;
	int y_offset = 0;
	cvZero(image_mask_smooth);
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);
	const char windows_name_rbg[] = "RBG";
	const char windows_name_depth[] = "Depth";
	cvNamedWindow( windows_name_rbg, CV_WINDOW_AUTOSIZE);
	CvPoint mouse_click = { .x = -1, .y = -1 };
	cvSetMouseCallback( windows_name_rbg, mouseHandler, &mouse_click );
	bool reinitialize = false;

	// Point the Kinect at the ceiling for a better view of the lights closest to it
//	tilt_up();

	// process frames indefinitely at the rate defined by PROCESS_FPS
	// quit when user presses 'q'
	while (key != 'q')
	{
		image_rgb = freenect_sync_get_rgb_cv(0);
		if (!image_rgb) {
			printf("Error: Kinect not connected?\n");
			return -1;
		}
		cvCvtColor(image_rgb, image_rgb, CV_RGB2BGR);

		image_disparity = freenect_sync_get_depth_cv(0);
		if (!image_disparity) {
			printf("Error: Kinect not connected?\n");
			return -1;
		}
		image_depth = kinect_disparity_filter(image_disparity);

		// shift depth image
		shift_image( image_depth, x_offset, y_offset);

		// blend RGB and disparity frames after shift
		cvAddWeighted(image_rgb, 1.0, image_depth, 1.0, 0.0, image_blended);
		cvShowImage( "Blended", image_blended);

		// create binary image of disparity
		cvCvtColor( image_depth, image_depth_gray, CV_RGB2GRAY);
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

		// take multiple samples of the depth because it's noisy
		// Adding fills in the depth mask
		cvAdd( image_mask, image_mask_smooth, image_mask_smooth, NULL); // adding multiple samples _grows_ the mask
		// Anding removes pixels
		//cvAnd( image_mask, image_mask_smooth, image_mask_smooth, NULL); // anding multiple samples _shrinks_ the mask
		// TODO: need to adding multiple image_masks in order to only use pixels that are set in all n frames
		cvShowImage("Mask Smooth", image_mask_smooth);

//		cvCopy(image_rgb, image_rgb_masked, image_mask);
//		cvShowImage("RGB Mask", image_rgb_masked);

		// find polygons in the disparity data
		fprintf(stdout, "Disparity Contours (X,Y)\n");
		cvCopy( detect_contours(image_mask_smooth, RGB_CONTOURS), disparity_contours, NULL);
		cvShowImage("Disparity Contours", disparity_contours);

		// find polygons in the RGB data
		fprintf(stdout, "RGB Contours (X,Y)\n");
		cvSmooth(image_rgb, image_rgb, CV_GAUSSIAN, 5, 5, 0, 0);
		cvCvtColor(image_rgb, image_gray, CV_RGB2GRAY);

		cvCanny(image_gray, image_edges, CANNY_LOW, CANNY_HIGH, 3);
		cvSmooth(image_edges, image_edges, CV_GAUSSIAN, 5, 5, 0, 0);

		cvShowImage("Edges", image_edges);
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

		// write the coordinate and the depth where user has left-clicked
		if (-1 != mouse_click.x && -1 != mouse_click.y)
		{
			char coord_str[] = "640,480,-01.234"; // max coordinate length
			int coord_str_len = strlen(coord_str);
			int pixel_disparity = ((short *) image_disparity->imageData)[mouse_click.y * 640 + mouse_click.x];
			sprintf(coord_str, "%03d,%03d,%04d", mouse_click.x, mouse_click.y, pixel_disparity);
			//float pixel_depth_meters = raw_depth_to_meters(pixel_disparity);
			//sprintf(coord_str, "%03d,%03d,%02.03f", mouse_click.x, mouse_click.y, pixel_depth_meters);
			coord_str[coord_str_len] = '\0';
			cvPutText(image_rgb, coord_str, cvPoint(mouse_click.x, mouse_click.y), &font, cvScalar(255, 255, 255, 0));
		}
		cvShowImage(windows_name_rbg, image_rgb);
		//cvShowImage(windows_name_depth, GlViewColor(depth));
		cvShowImage(windows_name_depth, image_depth);
		//cvShowImage(windows_name_depth, depth);

		// wait for a key and time delay
		key = cvWaitKey(1000/PROCESS_FPS);
		// shift depth image if necessary
		reinitialize = handle_key_input(key, &x_offset, &y_offset);
	}

	// return the camera horizontal tilt
//	tilt_reset();

	return 0;
}
