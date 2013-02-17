/*
 * helpers.h
 *
 *  Created on: Jan 28, 2013
 *      Author: lucasrangit
 */

#ifndef HELPERS_H_
#define HELPERS_H_

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

bool handle_key_input(char key, int *x_offset, int *y_offset);
void mouseHandler(int event, int x, int y, int flags, void *param);
void shift_image( IplImage *image_src, int x_offset, int y_offset);
int acquire_color_and_disparity( IplImage *image_dst_color, IplImage *image_disparity);
void filter_out_of_range_disparity(IplImage *disparity_16u_1, IplImage *image_8u_1);
void draw_value( IplImage *image, int value, CvPoint pixel);
float distance2f( CvPoint a, CvPoint b);
int get_disparity( IplImage *disparity, CvPoint coord);
void tilt_horizontal();
void tilt_up();
float scale_cartician( float point, float scale, float center);

#endif /* HELPERS_H_ */
