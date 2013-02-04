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
int acquire_color_and_depth( IplImage *image_dst_color, IplImage *image_dst_depth, IplImage *image_disparity);

#endif /* HELPERS_H_ */
