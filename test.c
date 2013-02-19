/*
 * test.c
 *
 *  Created on: Jan 29, 2013
 *      Author: lucasrangit
 */
#include "rgbdlocalization.h"
#include "helpers.h"

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
	cvSetZero(matA);
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

// new dilateQuadAboutCenter parameter type, this wont work until updated
//void test_dilateQuadAboutCenter()
//{
//	CvPoint2D32f ptA[4];
//	CvPoint2D32f ptB[4];
//	CvPoint2D32f center;
//	float scale;
//
//	// define a test quad
//	ptA[0].x = 0.5; ptA[0].y = 0.5;
//	ptA[1].x = 2.5; ptA[1].y = 0.5;
//	ptA[2].x = 2.5; ptA[2].y = 2.5;
//	ptA[3].x = 0.5; ptA[3].y = 2.5;
//
//	// keep a copy of original
//	memcpy( ptB, ptA, sizeof(ptA));
//
//	// use center of quad
//	// TODO: calculate center
//	center.x = 1.5; center.y = 1.5;
//
//	// scale quad
//	scale = 0.5;
//	dilateQuadAboutCenter( ptB, scale);
//}

/**
 * Paint all contours with a single OpenCV call on an image.
 */
void test_cvDrawContours( IplImage *img, CvSeq* contours)
{
	IplImage* image_all_contours = cvCreateImage(cvGetSize(img), 8, 1);
	cvCopy(img, image_all_contours, NULL);
//		CvSeq* contour = contours; // first contour
	// TODO need for loop to iterate through sequence
	cvDrawContours( image_all_contours, contours, cvScalarAll(255), cvScalarAll(0), 0, CV_FILLED, 8, cvPoint(0,0));

	cvShowImage( "All contours", image_all_contours);
	cvReleaseImage(&image_all_contours);
}

/**
 * Estimate the 3D position based on the locations of 3 or more reference satellites.
 * The delta time component is 0 because the distances are measured simultaneously.
 *
 * This algorithm was from X and was written for Matlab by Dr. Jidong Huang
 * and converted to C and OpenCV by Lucas Magasweran.
 *
 * @ref APPENDIX A LINEARIZING THE GPS PSEUDORANGE EQUATIONS
 */
//function [xyz] = solve3D(svrange, svpos)
void solve3D( CvMat *svrange, CvMat *svpos)
{
	int i = 0, j = 0;
//eps = 1e-10;
	float eps = powf(10,-10);
//% setup the maximum number of iterations
//max_iter = 10;
	int max_iter = 10;
//
//% number of measurements
//num_sv = length(svrange);
	int num_sv = svrange->rows;

//iter = 0; % number of iterations
	int iter = 0;

//err_est = 100; % a large error to start
	float err_est = 100;

//% initial estimate from location [0 0 0]
//xyz = [0 0 0]';
	CvMat* xyz = cvCreateMat( 3, 1, CV_32FC1 );
	cvSetZero(xyz);

//% setup the Geometry matrix H, H is a num_sv x 3 matrix
	CvMat* H = cvCreateMat( num_sv, 3, CV_32FC1 );
	cvSetZero(H);

	CvMat* dRi = cvCreateMat( num_sv, 1, CV_32FC1 );
	cvSetZero(dRi);

//while ((iter<max_iter) & (err_est>eps))
	while ( (iter < max_iter) && (err_est > eps))
	{
//    iter = iter + 1;
		iter++;

//    for i = 1:num_sv
		for (i = 0; i < num_sv; ++i)
		{
			CvMat* svpos_i = cvCreateMat( 3, 1, CV_32FC1 );
			cvSetZero(svpos_i);
//        Ri = norm(xyz - svpos(:, i));
			get_vector_column( svpos, svpos_i, i);
			double Ri = cvNorm(xyz, svpos_i, CV_L2, NULL);

//        H(i, 1) = (xyz(1) - svpos(1, i))/Ri;
//        H(i, 2) = (xyz(2) - svpos(2, i))/Ri;
//        H(i, 3) = (xyz(3) - svpos(3, i))/Ri;
			for (j = 0; j < 3; ++j)
			{
				*( (float*)CV_MAT_ELEM_PTR( *H, i, j ) ) =
					(CV_MAT_ELEM( *xyz, float, j, 0 ) - CV_MAT_ELEM( *svpos, float, j, i )) / Ri;
			}

//        % setup the dRi vector, which is num_sv x 1 vector
//        dRi(i, 1) = svrange(i) - Ri;
			*( (float*)CV_MAT_ELEM_PTR( *dRi, i, 0 ) ) =
					CV_MAT_ELEM( *svrange, float, i, 0 ) - Ri;
//    end
		}
//
//    % calculate the corrections to xyz vector
		CvMat* delta_xyz = cvCreateMat( 3, 1, CV_32FC1 );
		cvSetZero(delta_xyz);
//    delta_xyz = inv(H'*H)*H'*dRi;
		int return_code = cvSolve( H, dRi, delta_xyz, CV_SVD);

		for (j = 0; j < 3; ++j)
		{
//    % apply the corrections to xyz vector
//    xyz = xyz + delta_xyz;
			*( (float*)CV_MAT_ELEM_PTR( *xyz, j, 0 ) ) =
					CV_MAT_ELEM( *xyz, float, j, 0 ) + CV_MAT_ELEM( *delta_xyz, float, j, 0 );
//    err_est = norm(xyz - svpos(:, i)) - svrange(i);
//			CvMat* err_est = cvCreateMat( 3, 1, CV_32FC1 );
//			cvSetZero(err_est);

		}
//end
	}

	float x = CV_MAT_ELEM( *xyz, float, 0, 0 );
	float y = CV_MAT_ELEM( *xyz, float, 1, 0 );
	float z = CV_MAT_ELEM( *xyz, float, 2, 0 );

}

void test_solve3D( )
{
//	int i = 0;
	CvMat* user = cvCreateMat( 3, 1, CV_32FC1 );
	cvSetZero(user);
	double svpos_i_norm = 0;

//	user = [-1, 1, 2.5]'
	*( (float*)CV_MAT_ELEM_PTR( *user, 0, 0 ) ) = -1.0;
	*( (float*)CV_MAT_ELEM_PTR( *user, 1, 0 ) ) =  1.0;
	*( (float*)CV_MAT_ELEM_PTR( *user, 2, 0 ) ) =  2.5;

//	% satellite positions
	CvMat* svpos = cvCreateMat( 3, 3, CV_32FC1 );
	cvSetZero(svpos);
	// temp vector for each satellite for use in norm calculations
	CvMat* svpos_temp = cvCreateMat( 3, 1, CV_32FC1 );
	cvSetZero(svpos_temp);
	CvMat* svrange = cvCreateMat( 3, 1, CV_32FC1 );
	cvSetZero(svrange);

//	svpos(:, 1) = [2, 3, 4]';
	*( (float*)CV_MAT_ELEM_PTR( *svpos, 0, 0 ) ) = 2.0;
	*( (float*)CV_MAT_ELEM_PTR( *svpos, 1, 0 ) ) = 3.0;
	*( (float*)CV_MAT_ELEM_PTR( *svpos, 2, 0 ) ) = 4.0;
//	svrange(1,1) = norm(user - svpos(:, 1))+.05*randn;
	get_vector_column( svpos, svpos_temp, 0);
	svpos_i_norm = cvNorm(user, svpos_temp, CV_L2, NULL);
	*( (float*)CV_MAT_ELEM_PTR( *svrange, 0, 0 ) ) = svpos_i_norm;

//	svpos(:, 2) = [3, 4, 3]';
	*( (float*)CV_MAT_ELEM_PTR( *svpos, 0, 1 ) ) = 3.0;
	*( (float*)CV_MAT_ELEM_PTR( *svpos, 1, 1 ) ) = 4.0;
	*( (float*)CV_MAT_ELEM_PTR( *svpos, 2, 1 ) ) = 3.0;
	get_vector_column( svpos, svpos_temp, 1);
	svpos_i_norm = cvNorm(user, svpos_temp, CV_L2, NULL);
	*( (float*)CV_MAT_ELEM_PTR( *svrange, 1, 0 ) ) = svpos_i_norm;

//	svrange(2,1) = norm(user - svpos(:, 2))+.05*randn;

//	svpos(:, 3) = [1, 3, 6]';
	*( (float*)CV_MAT_ELEM_PTR( *svpos, 0, 2 ) ) = 1.0;
	*( (float*)CV_MAT_ELEM_PTR( *svpos, 1, 2 ) ) = 3.0;
	*( (float*)CV_MAT_ELEM_PTR( *svpos, 2, 2 ) ) = 6.0;
//	svrange(3,1) = norm(user - svpos(:, 3))+.05*randn;
	get_vector_column( svpos, svpos_temp, 2);
	svpos_i_norm = cvNorm(user, svpos_temp, CV_L2, NULL);
	*( (float*)CV_MAT_ELEM_PTR( *svrange, 2, 0 ) ) = svpos_i_norm;

//	svpos(:, 4) = [-1, 2, 3]';
//	svrange(4,1) = norm(user - svpos(:, 4))+.05*randn;

//	svpos(:, 5) = [-2, 1.5, 3]';
//	svrange(5,1) = sqrt((user(1) - svpos(1, 5))^2 + (user(2) - svpos(2, 5))^2 ...
//	    + (user(3) - svpos(3, 5))^2 );

//	xyz = solve3D(svrange, svpos)
	solve3D( svrange, svpos);
}

