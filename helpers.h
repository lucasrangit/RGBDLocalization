/*
 * helpers.h
 *
 *  Created on: Jan 28, 2013
 *      Author: lucasrangit
 */

#ifndef HELPERS_H_
#define HELPERS_H_

IplImage *kinect_disparity_filter(IplImage *depth);
bool handle_key_input(char key, int *x_offset, int *y_offset);
void mouseHandler(int event, int x, int y, int flags, void *param);
void shift_image( IplImage *image_src, int x_offset, int y_offset);

#endif /* HELPERS_H_ */
