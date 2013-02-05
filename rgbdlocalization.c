#include "rgbdlocalization.h"
#include "helpers.h"

static CvContour potential_landmarks[LANDMARK_COUNT_MAX];

/*
 * Find contours in an image and return a new image with the contours drawn.
 * This function performs a filter on the shapes to identify quadrilaterals
 * in the shape of a ceiling light: is convex, has 4 vertices, and a certain size.
 * TODO: consider adding a slider to the window for adjusting the thresholds
 */
static IplImage* detect_contours(IplImage* img)
{
	CvSeq* contours; // linked list of contours
	CvSeq* polygon; // pointer to single polygon contour
	CvMemStorage *storage = cvCreateMemStorage(0); // storage for contour linked list
	static IplImage* ret = NULL;
	if (!ret)
		ret = cvCreateImage(cvGetSize(img), 8, 1);
	else
		cvZero(ret);
	IplImage* temp = cvCreateImage(cvGetSize(img), 8, 1);
	int i;
	double area;
	int contour_index = 0;
	CvPoint potential_ceiling_lights[LANDMARK_COUNT_MAX][4];

	/*
	 * Search input image for contours
	 */
	cvCopy(img, temp, NULL);
	//	cvFindContours(temp, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	cvFindContours(temp, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

	/*
	 * Find contours that have the shape of ceiling lights
	 */
	while (contours)
	{
		// Approximate a polygon around contour using the Douglas-Peucker (DP) approximation.
		// Pass a zero as the last argument instructs cvApproxPoly to only operate on the first
		// element of the sequence.
		polygon = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.06, 0);

		if (4 == polygon->total)
		{
			// has 4 vertices
			area = fabs(cvContourArea(polygon, CV_WHOLE_SEQ, 0));
			if ((area > CONTOUR_AREA_MIN) &&
				(area < CONTOUR_AREA_MAX))
			{
				// has "reasonable" area
				if (cvCheckContourConvexity(polygon))
				{
					// is convex
					CvPoint *pt[4];
					for ( i = 0; i < 4; i++)
					{
						pt[i] = (CvPoint*)cvGetSeqElem(polygon, i);

						// save the contour as a potential landmark
						if (contour_index < LANDMARK_COUNT_MAX)
							potential_ceiling_lights[contour_index][i] = *pt[i];
					}

					// draw contour using the vertices so that we can adjust the color and thickness of each
					{
						CvScalar line_color = cvScalarAll(255); // use with black and white image
						int line_thickness = contour_index+1; // vary the thickness so that contours can be distinguished in black and white
						cvLine(ret, *pt[0], *pt[1], line_color, line_thickness, 8, 0);
						cvLine(ret, *pt[1], *pt[2], line_color, line_thickness, 8, 0);
						cvLine(ret, *pt[2], *pt[3], line_color, line_thickness, 8, 0);
						cvLine(ret, *pt[3], *pt[0], line_color, line_thickness, 8, 0);
					}

					fprintf(stdout, "%d. (%03d,%03d) (%03d,%03d) (%03d,%03d) (%03d,%03d) area: %.1f\n", contour_index, pt[0]->x, pt[0]->y, pt[1]->x, pt[1]->y, pt[2]->x, pt[2]->y, pt[3]->x, pt[3]->y, area);
				}
			}
		}

		contours = contours->h_next;
		contour_index++;
	}

	cvReleaseImage(&temp);
	cvReleaseMemStorage(&storage);

	return ret;
}

int main(int argc, char *argv[])
{
	IplImage *image_rgb = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 3);
	IplImage *image_disparity = cvCreateImage( cvSize(640, 480), IPL_DEPTH_16U, 1);
	IplImage *image_nodepth = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *image_nodepth_mask = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *image_nodepth_color = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 3);
	IplImage *image_blended = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 3);
	IplImage *disparity_contours = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *image_gray = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *image_edges = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *rgb_contours = cvCreateImage( cvSize(640, 480), IPL_DEPTH_8U, 1);
	bool reinitialize = true;
	char key;
	int x_offset = 0;
	int y_offset = 0;
	const char window_name_live[] = "Kinect Color and Depth";
	cvNamedWindow( window_name_live, CV_WINDOW_AUTOSIZE);
	CvPoint mouse_click = { .x = -1, .y = -1 };
	cvSetMouseCallback( window_name_live, mouseHandler, &mouse_click );

	// Point the Kinect at the ceiling for a better view of the lights closest to it
//	tilt_up();

	// process frames indefinitely at the rate defined by PROCESS_FPS
	// quit when user presses 'q'
	while (key != 'q')
	{
		/*
		 * Capture data from Kinect
		 */
		if (acquire_color_and_disparity( image_rgb, image_disparity ))
		{
			// error getting data, skip processing this frame
			continue;
		}

		/*
		 * Create RGB and No Depth images for analysis
		 */
		filter_out_of_range_disparity(image_disparity, image_nodepth);

		shift_image( image_nodepth, x_offset, y_offset);

		// blend live RGB and no depth images for monitoring purposes
		cvMerge( NULL, image_nodepth, NULL, NULL, image_nodepth_color);
		cvAddWeighted(image_rgb, 1.0, image_nodepth_color, 1.0, 0.0, image_blended);

		// take multiple samples of the missing depth data because it's noisy
		cvAdd( image_nodepth, image_nodepth_mask, image_nodepth_mask, NULL); // adding  _grows_ the mask
		//cvAnd( image_depth, image_depth_smooth, image_depth_smooth, NULL); // anding _shrinks_ the mask
		// TODO: use weighted samples (e.g. 3 successive pixels must match)

		// write the coordinate and the disparity where user has left-clicked
		if (-1 != mouse_click.x && -1 != mouse_click.y)
		{
			int pixel_disparity = ((short *) image_disparity->imageData)[(mouse_click.y - y_offset) * 640 + (mouse_click.x - x_offset)];
			image_paint_value( image_blended, pixel_disparity, mouse_click.x, mouse_click.y);
		}

		/*
		 * Find polygons in the disparity data
		 */
		fprintf(stdout, "Disparity Contours (X,Y)\n");
		cvCopy( detect_contours(image_nodepth_mask, RGB_CONTOURS), disparity_contours, NULL);

		/*
		 * find polygons in the RGB data
		 */
		// apply Gaussian blur to reduce noise
		cvSmooth(image_rgb, image_rgb, CV_GAUSSIAN, 5, 5, 0, 0);
		// convert to gray scale (required by Canny edge detector)
		cvCvtColor(image_rgb, image_gray, CV_RGB2GRAY);
		// detect edges
		cvCanny(image_gray, image_edges, CANNY_LOW, CANNY_HIGH, 3);
		// blurring edges improves contour detection
		cvSmooth(image_edges, image_edges, CV_GAUSSIAN, 5, 5, 0, 0);
		// detect contours from edges
		fprintf(stdout, "RGB Contours (X,Y)\n");
		cvCopy( detect_contours(image_edges, DEPTH_CONTOURS), rgb_contours, NULL);

		/*
		 * find matching contours
		 */

		/*
		 * Display input and intermediate data for monitoring
		 */
		cvShowImage( window_name_live, image_blended);
		cvShowImage( "Depth Mask", image_nodepth_mask);
		cvShowImage( "Disparity Contours", disparity_contours);
		//cvShowImage( "Edges", image_edges);
		cvShowImage("RGB Contours", rgb_contours);

		/*
		 * Wait for user input or timeout
		 */
		key = cvWaitKey(1000/PROCESS_FPS);
		reinitialize = handle_key_input(key, &x_offset, &y_offset);

		if (reinitialize)
		{
			// initialize the mask
			cvZero(image_nodepth_mask);
			// clear flag
			reinitialize = false;
		}
	}

	/*
	 * Cleanup
	 */
	// return the camera horizontal tilt
//	tilt_reset();

	cvDestroyAllWindows();

	cvReleaseImage( &image_rgb);
	cvReleaseImage( &image_disparity);
	cvReleaseImage( &image_nodepth);
	cvReleaseImage( &image_nodepth_mask);
	cvReleaseImage( &image_nodepth_color);
	cvReleaseImage( &image_blended);
	cvReleaseImage( &disparity_contours);
	cvReleaseImage( &image_gray);
	cvReleaseImage( &image_edges);
	cvReleaseImage( &rgb_contours);


	/*
	 * Exit
	 */
	return 0;
}
