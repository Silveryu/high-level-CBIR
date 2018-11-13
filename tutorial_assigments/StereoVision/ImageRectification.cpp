#include <iostream>
#include <vector>

#include <stdio.h>

// OpenCV Includes
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdlib.h>

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

  cv::Mat imageLeftRectified;
  cv::Mat imageRightRectified;

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

    cv::Mat map1x;
    cv::Mat map1y;
    cv::Mat map2x;
    cv::Mat map2y;

    cv::Mat R1;
    cv::Mat R2;
    
    cv::Mat P1;
    cv::Mat P2;
    
    cv::Mat Q;

    Size img_size = imageLeft.size();

    cout << "stereoRectify" << endl;


    stereoRectify(intrinsic_matrix_1, distortion_coeffs_1, intrinsic_matrix_2, distortion_coeffs_2, img_size, R, T, R1, R2, P1, P2, Q);


    initUndistortRectifyMap(intrinsic_matrix_1, distortion_coeffs_1, R1, P1, img_size, CV_16SC2, map1x, map1y);
    initUndistortRectifyMap(intrinsic_matrix_2, distortion_coeffs_2, R2, P2, img_size, CV_16SC2, map2x, map2y);


    cout << "remap" << endl;
    remap(imageLeft, imageLeftRectified, map1x, map1y, INTER_LINEAR);
    remap(imageRight, imageRightRectified, map2x, map2y, INTER_LINEAR);

    
    // draw rectangles                
    for(int j = 0; j < imageLeftRectified.cols; j += 16 )
      line(imageLeftRectified, Point(0, j), Point(imageLeftRectified.cols, j), Scalar(0, 255, 0), 1, 8);


    // draw rectangles                
    for(int j = 0; j < imageRightRectified.cols; j += 16 )
      line(imageRightRectified, Point(0, j), Point(imageRightRectified.cols, j), Scalar(0, 255, 0), 1, 8);

    


    do{
      //cv::imshow("Left image",imageLeft);
      cv::imshow("Left image rectified", imageLeftRectified);
    }
    //printf("\n Number of corners: %d",corners->size());
	  while((char)waitKey(0) == 27); 

    
    do{
      cv::imshow("Right image rectified", imageRightRectified);
    }while((char)waitKey(0) == 27);
    //printf("\n Number of corners: %d",corners->size());
	  cv::waitKey(0); 




  }
  return 0;
}
