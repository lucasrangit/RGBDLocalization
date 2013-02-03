/*
 * test.c
 *
 *  Created on: Jan 29, 2013
 *      Author: lucasrangit
 */
#include "rgbdlocalization.h"

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
//	CvPoint2D32f src[4], dst[4]; // source and destination 2D coordinates

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
	int return_code = cvSolve( matA, matB, matX, CV_LU);

	float x = CV_MAT_ELEM( *matX, float, 0, 0 );
	float y = CV_MAT_ELEM( *matX, float, 1, 0 );

	if (!return_code)
		printf("Solution (x,y) = (%f, %f)\n", x, y);
	else
		printf("No solution found");
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

static float scale_cartician( float point, float scale, float center)
{
	return (point*scale + center*(1-scale));
}

/**
 * Dilate a quadrilateral about a point.
 * TODO: evaluate another algorithm that works for convex or concave:
 *       http://stackoverflow.com/questions/7995547/enlarge-and-restrict-a-quadrilateral-polygon-in-opencv-2-3-with-c
 * @param[in] verticies going ccw
 */
void dilateQuadAboutCenter( CvPoint2D32f vertices[4], CvPoint2D32f origin, float scale)
{
	vertices[0].x = scale_cartician(vertices[0].x, scale, origin.x); vertices[0].y = scale_cartician(vertices[0].y, scale, origin.y);
	vertices[1].x = scale_cartician(vertices[1].x, scale, origin.x); vertices[1].y = scale_cartician(vertices[1].y, scale, origin.y);
	vertices[2].x = scale_cartician(vertices[2].x, scale, origin.x); vertices[2].y = scale_cartician(vertices[2].y, scale, origin.y);
	vertices[3].x = scale_cartician(vertices[3].x, scale, origin.x); vertices[3].y = scale_cartician(vertices[3].y, scale, origin.y);
}

void test_dilateQuadAboutCenter()
{
	CvPoint2D32f ptA[4];
	CvPoint2D32f ptB[4];
	CvPoint2D32f center;
	float scale;

	// define a test quad
	ptA[0].x = 0.5; ptA[0].y = 0.5;
	ptA[1].x = 2.5; ptA[1].y = 0.5;
	ptA[2].x = 2.5; ptA[2].y = 2.5;
	ptA[3].x = 0.5; ptA[3].y = 2.5;

	// keep a copy of original
	memcpy( ptB, ptA, sizeof(ptA));

	// use center of quad
	// TODO: calculate center
	center.x = 1.5; center.y = 1.5;

	// scale quad
	scale = 0.5;
	dilateQuadAboutCenter( ptB, center, scale);
}


