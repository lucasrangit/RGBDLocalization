/*
 * helpers.h
 *
 *  Created on: Jan 28, 2013
 *      Author: lucasrangit
 */

#ifndef HELPERS_H_
#define HELPERS_H_

bool handle_key_input(char key, int *x_offset, int *y_offset);
void mouseHandler(int event, int x, int y, int flags, void *param);
void shift_image( IplImage *image_src, int x_offset, int y_offset);
int acquire_color_and_disparity( IplImage *image_dst_color, IplImage *image_disparity);
void filter_out_of_range_disparity(IplImage *disparity_16u_1, IplImage *image_8u_1);
void image_paint_value( IplImage *image, int value, int x, int y);
CvPoint2D32f findCentroid( quad_coord input_quad);
float distance2f( CvPoint2D32f a, CvPoint2D32f b);
quad_coord dilateQuadAboutCenter( quad_coord quad, float scale);
int get_disparity( IplImage *disparity, CvPoint coord);
void quad_coord_clear(quad_coord lights_depth[4]);

#endif /* HELPERS_H_ */
