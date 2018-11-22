/***********************************************************************************
Name:           chessboard.cpp
Revision:
Date:           05-10-2013
Author:         Paulo Dias
Comments:       ChessBoard Tracking


images
Revision:
Libraries:
Notes:          Code generated with Visual Studio 2013 Intel OpenCV 2.4.8 libraries 
***********************************************************************************/
#include <iostream>
#include <vector>

#include <stdio.h>

// OpenCV Includes
#include <opencv2/core/core.hpp>        
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"

using namespace cv;


// Function FindAndDisplayChessboard
// find corners in a cheesboard with board_w x board_h dimensions
// Display the corners in image and return number of detected corners
int FindAndDisplayChessboard(cv::Mat image,int board_w,int board_h, std::vector<cv::Point2f> *corners)
{
  int board_size = board_w * board_h;
  CvSize board_sz = cvSize(board_w,board_h);

  cv::Mat grey_image;

  cv::cvtColor(image, grey_image, CV_BGR2GRAY);
  
  // find chessboard corners
  bool found = cv::findChessboardCorners(grey_image, board_sz, *corners,0);
  
  // Draw results
  if (true)
  {
	  cv::drawChessboardCorners(image, board_sz, cv::Mat(*corners), found);
	  cv::imshow("Calibration",image);
	  cv::waitKey(0); 
  }
  return corners->size();
}

int main(int argc, char **argv)
{
  // ChessBoard Properties
  int n_boards = 13; //Number of images
  int board_w = 9;
  int board_h = 6;

  int board_sz = board_w * board_h;

  char filenameLeft[200];
  char filenameRight[200];


  // Chessboard coordinates and image pixels
  std::vector<std::vector<cv::Point3f> > opts;
  std::vector<std::vector<cv::Point2f> > ipts1;
  std::vector<std::vector<cv::Point2f> > ipts2;
  std::vector<int> npts;

  // Corners detected in each image
  std::vector<cv::Point2f> imagePtsLeft;
  std::vector<cv::Point2f> imagePtsRight;
  
  int corner_count_left;
  int corner_count_right;

  cv::Mat imageLeft;
  cv::Mat imageRight;

  int i;

  int sucesses = 0;
  // chessboard coordinates
  std::vector<cv::Point3f> obj;
  for(int j=0;j<board_sz;j++)
    obj.push_back(cv::Point3f(float(j/board_w), float(j%board_w), 0.0));
  
  for (i=0;i<n_boards;i++)
  {
    // read image 
    sprintf(filenameLeft,"../images/left%02d.jpg",i+1);
    sprintf(filenameRight,"../images/right%02d.jpg",i+1);

    printf("\nReading %s",filenameLeft);
    printf("\nReading %s",filenameRight);

    imageLeft = cv::imread(filenameLeft, CV_LOAD_IMAGE_COLOR);
    imageRight = cv::imread(filenameRight, CV_LOAD_IMAGE_COLOR);

    if(!imageLeft.data) 
    {
      printf("\nCould not load image file: %s\n",filenameLeft);
	  getchar();
      return 0;
    }
    
    if(!imageRight.data) 
    {
      printf("\nCould not load image file: %s\n",filenameRight);
	  getchar();
      return 0;
    }
    
	// find and display corners
    corner_count_left =  FindAndDisplayChessboard(imageLeft,board_w,board_h,&imagePtsLeft);
    corner_count_right = FindAndDisplayChessboard(imageRight,board_w,board_h,&imagePtsRight);
    
    // output of stereoCalibration
    cv::Mat intrinsic_matrix_1;
    cv::Mat distortion_coeffs_1;
    cv::Mat intrinsic_matrix_2;
    cv::Mat distortion_coeffs_2;
    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;
    
	if (corner_count_left == board_w * board_h && corner_count_right == board_w * board_h)
    {
      ipts1.push_back(imagePtsLeft);
      ipts2.push_back(imagePtsRight);
      npts.push_back(corner_count_left);
	    opts.push_back(obj);
      sucesses++;
    

    double rms = stereoCalibrate(opts, ipts1, ipts2,
                    intrinsic_matrix_1, distortion_coeffs_1,
                    intrinsic_matrix_2, distortion_coeffs_2,
                    imageLeft.size(), R, T, E, F,
                    CALIB_SAME_FOCAL_LENGTH);

    // Save successfull calibrations
    cv::FileStorage fs("stereoParams.xml", cv::FileStorage::WRITE);
    if (!fs.isOpened()){
        std::cout << "Failed to open stereoParams.xml" << std::endl;
        return 1;
    }
    else
        std::cout << "Writing camera parameters" << std::endl;
    
    fs << "cameraMatrix_1" <<  intrinsic_matrix_1;
    fs << "distCoeffs_1" <<  distortion_coeffs_1;
    fs << "cameraMatrix_2" <<  intrinsic_matrix_2;
    fs << "distCoeffs_2" <<  distortion_coeffs_2;
    fs << "rotation" << R;
    fs << "translation" <<  T;
    fs << "essential" << E;
    fs << "fundamental" << F;
    fs.release();
    }
  
  }
  return 0;
}
