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
using namespace std;



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

  cv::Mat imageLeftUndistorted;
  cv::Mat imageRightUndistorted;

  int i;

  int sucesses = 0;
	 
  // chessboard coordinates
  std::vector<cv::Point3f> obj;
  for(int j=0;j<board_sz;j++)
    obj.push_back(cv::Point3f(float(j/board_w), float(j%board_w), 0.0));
  
  for (i=0;i<n_boards;i++)
  {
    // read image 
    sprintf(filenameLeft,"images/left%02d.jpg",i+1);
    sprintf(filenameRight,"images/right%02d.jpg",i+1);

    printf("\nReading %s",filenameLeft);
    printf("\nReading %s",filenameRight);
    cout << "88" << endl;


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
    
    FileStorage fs("stereoParams.xml", FileStorage::READ);

    // stereoCalibration params
    cv::Mat intrinsic_matrix_1;
    cv::Mat distortion_coeffs_1;
    cv::Mat intrinsic_matrix_2;
    cv::Mat distortion_coeffs_2;
    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;

    fs["cameraMatrix_1"] >>  intrinsic_matrix_1;
    fs["distCoeffs_1"] >>  distortion_coeffs_1;
    fs["cameraMatrix_2"] >>  intrinsic_matrix_2;
    fs["distCoeffs_2"] >>  distortion_coeffs_2;
    fs["rotation"] >> R;
    fs["translation"] >>  T;
    fs["essential"] >> E;
    fs["fundamental"] >> F;

    undistort(imageLeft, imageLeftUndistorted, intrinsic_matrix_1, distortion_coeffs_1);
    do{
    cv::imshow("Left image",imageLeft);
    cv::imshow("Left image undistorted",imageLeftUndistorted);
    }
    //printf("\n Number of corners: %d",corners->size());
	  while((char)waitKey(0) == 27); 

    
    undistort(imageRight, imageRightUndistorted, intrinsic_matrix_2, distortion_coeffs_2);
    do{
    cv::imshow("Right image",imageRight);
    cv::imshow("Right image undistorted",imageRightUndistorted);
    }while((char)waitKey(0) == 27);
    //printf("\n Number of corners: %d",corners->size());
	  cv::waitKey(0); 




  }
  return 0;
}
