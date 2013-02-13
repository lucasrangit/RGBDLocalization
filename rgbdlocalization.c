#include "rgbdlocalization.h"
#include "helpers.h"

float approximate_depth( IplImage *disparity, quad_coord quad)
{
	float depth = -1.0;
	float vertex_depths[4] = { 0 };
	int i;

	/*
	 * Set the depth for each vertex.
	 * If a vertex does not have a valid depth dilate the quad until one is found.
	 */
	for ( i = 0; i < 4; ++i)
	{
		if ( QC_INVALID == quad.valid)
		{
			// quad has been flagged as invalid
			goto exit;
		}

		int pixel_disparity = get_disparity( disparity, quad.vertices[i]);
		if ( pixel_disparity > 0 && pixel_disparity < 2047)
			vertex_depths[i] = pixel_disparity;
		else
		{
			// grow each vertex until a known depth is found
			int looplimit = 100;
			do
			{
				quad_coord scaled_quad = dilateQuadAboutCenter( quad, 1.5);

				pixel_disparity = get_disparity( disparity, scaled_quad.vertices[i]);

				// stop if the dilation is out of range
				if ( -1 == pixel_disparity)
					// invalid parameter, out of range?
					goto exit;
				else if ( pixel_disparity >= 0 && pixel_disparity <= 2047) // change to useful range
					vertex_depths[i] = pixel_disparity; // TODO use disparity2depth

				if (!looplimit--)
				{
					// could not find a valid depth (does the sensor have full view of the quad?)
					vertex_depths[i] = -1.0;
					continue;
				}
			} while ( 0 == vertex_depths[i] );
		}
	}

	// return average of known depth
	for ( i = 0; i < 4; ++i)
	{
		depth += vertex_depths[i];
	}
	depth /= i;

	exit:
	return depth;
}

/*
 * Find contours in an image and return a new image with the contours drawn.
 * This function performs a filter on the shapes to identify quadrilaterals
 * in the shape of a ceiling light: is convex, has 4 vertices, and a certain size.
 * TODO: consider adding a slider to the window for adjusting the thresholds
 */
static IplImage* detect_contours(IplImage* img, quad_coord *found_quads)
{
	CvSeq* contours; // linked list of contours
	CvSeq* polygon; // pointer to single polygon contour
	CvMemStorage *storage = cvCreateMemStorage(0); // storage for contour linked list
	static IplImage* ret = NULL;
	if (!ret)
		ret = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	else
		cvZero(ret);
	IplImage* temp = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	int i;
	double area;
	int contour_index = 0;

	/*
	 * Search input image for contours
	 */
	cvCopy(img, temp, NULL);
	// Find only the outer contours
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
						// total number of vertices is 4, verified earlier
						pt[i] = (CvPoint*)cvGetSeqElem(polygon, i);

						// save the contour as a potential landmark
						// only save up to LANMARK_COUNT_MAX
						if (contour_index < LANDMARK_COUNT_MAX)
						{
							found_quads[contour_index].vertices[contour_index] = *pt[i];
							found_quads[contour_index].valid = QC_VALID;
						}
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

					fprintf(stdout, "%d. (%03d,%03d) (%03d,%03d) (%03d,%03d) (%03d,%03d) area: %.1f\n", contour_index+1, pt[0]->x, pt[0]->y, pt[1]->x, pt[1]->y, pt[2]->x, pt[2]->y, pt[3]->x, pt[3]->y, area);
					contour_index++;
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
	int i;

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
			goto waitloop;
		}

		/*
		 * Filter raw depth image to create "No Depth" images
		 */
		filter_out_of_range_disparity(image_disparity, image_nodepth);

		// offset new depth image based off user input
		shift_image( image_nodepth, x_offset, y_offset);


		// take multiple samples of the missing depth data because it's noisy
		cvAdd( image_nodepth, image_nodepth_mask, image_nodepth_mask, NULL); // adding _grows_ the mask
		//cvAnd( image_depth, image_depth_smooth, image_depth_smooth, NULL); // anding _shrinks_ the mask
		// TODO: use weighted samples (e.g. 3 successive pixels must match)

		/*
		 * Find polygons in the disparity data
		 */
		quad_coord lights_depth[4];
		quad_coord_clear(lights_depth);
		fprintf(stdout, "Disparity Contours (X,Y)\n");
		cvCopy( detect_contours(image_nodepth_mask, lights_depth), disparity_contours, NULL);

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
		quad_coord lights_rgb[4];
		quad_coord_clear(lights_rgb);
		fprintf(stdout, "RGB Contours (X,Y)\n");
		cvCopy( detect_contours(image_edges, lights_rgb), rgb_contours, NULL);

		/*
		 * Find valid landmark by matching contours
		 */
		for (i = 0; i < LANDMARK_COUNT_MAX; ++i)
		{
			CvPoint centroid_rgb = findCentroid( lights_rgb[i]);
			draw_value( rgb_contours, i, centroid_rgb);
			CvPoint centroid_depth = findCentroid( lights_depth[i]);
			draw_value( image_disparity, i, centroid_depth);
			int distance = distance2f(centroid_rgb, centroid_depth);
			// can return -1.0 indicating invalid/unknown
			printf("Centroid #%d distance = %d\n", i, distance);
		}

		/*
		 * Approximate depth to each landmark
		 */
		for (i = 0; i < LANDMARK_COUNT_MAX; ++i)
		{
			float depth = approximate_depth( image_disparity, lights_depth[i]);
			CvPoint centroid_depth = findCentroid( lights_depth[i]);
			draw_value( image_disparity, depth, centroid_depth);
			printf( "Approximate depth[%d] = %.1f\n", i, depth);
		}

		/*
		 * Display input and intermediate data for monitoring
		 */
		// blend live RGB and no depth images for monitoring purposes
		cvMerge( NULL, image_nodepth, NULL, NULL, image_nodepth_color);
		cvAddWeighted(image_rgb, 1.0, image_nodepth_color, 1.0, 0.0, image_blended);

		// write the coordinate and the disparity where user has left-clicked
		if (-1 != mouse_click.x && -1 != mouse_click.y)
		{
			int pixel_disparity = ((short *) image_disparity->imageData)[(mouse_click.y - y_offset) * 640 + (mouse_click.x - x_offset)];
			draw_value( image_blended, pixel_disparity, mouse_click);
		}

		cvShowImage( window_name_live, image_blended);
		cvShowImage( "Depth Mask", image_nodepth_mask);
		cvShowImage( "Disparity Contours", disparity_contours);
		//cvShowImage( "Edges", image_edges);
		cvShowImage("RGB Contours", rgb_contours);

		/*
		 * Wait for user input or timeout before continuing
		 */
		waitloop:
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

	// return the camera horizontal tilt
//	tilt_reset();

	/*
	 * Exit
	 */
	return 0;
}
