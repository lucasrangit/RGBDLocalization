/*
 * helpers.c
 *
 *  Created on: Jan 28, 2013
 *      Author: lucasrangit
 */
#include "rgbdlocalization.h"

#if 0
/*
 * From their data, a basic first order approximation for converting the raw
 * 11-bit disparity value to a depth value in centimeters is:
 *   100/(-0.00307 * rawDisparity + 3.33).
 * This approximation is approximately 10 cm off at 4 m away, and less than
 * 2 cm off within 2.5 m.
 */
float raw_depth_to_meters(int raw_disparity)
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
float raw_depth_to_meters(int raw_disparity)
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
IplImage *kinect_disparity_filter(IplImage *depth)
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

void get_cv_info()
{
	const char* libraries;
	const char* modules;
	// Using cvGetModuleInfo() to check for IPP
	cvGetModuleInfo(0, &libraries, &modules);
	printf("Libraries: %s\nModules: %s\n", libraries, modules);
}

/**
 * Tilt the Kinect to the maximum angle.
 */
void tilt_up()
{
	freenect_raw_tilt_state *state = 0;

	if (freenect_sync_set_tilt_degs(MAX_TILT_ANGLE, 0)) {
		printf("Error: Kinect not connected?\n");
		return;
	}

	// wait for motor to stop moving before capturing images
	do {
		if (freenect_sync_get_tilt_state(&state, 0)) {
			printf("Error: Kinect not connected?\n");
			return;
		}
	} while (TILT_STATUS_MOVING == state->tilt_status);

	sleep(1); // @bug motor doesn't report correct state
}

/**
 * Return the Kinect to the horizontal position.
 * If the Kinect is tilted, such as by a tripod, this will return the camera to
 * the horizontal position relative to the ground, not the base of the Kinect.
 */
void tilt_horizontal()
{
	freenect_sync_set_tilt_degs(0, 0);
}
