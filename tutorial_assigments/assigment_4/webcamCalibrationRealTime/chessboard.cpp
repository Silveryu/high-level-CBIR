/***********************************************************************************
Name:           chessboard.cpp
Revision:
Author:         Paulo Dias
Comments:       ChessBoard Tracking


images
Revision:
Libraries:
***********************************************************************************/
#include <iostream>
#include <vector>

// OpenCV Includes
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgproc/imgproc_c.h"


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
	printf("\n Number of corners: %lu",corners->size());
	cv::waitKey(0);
  }
  return corners->size();
}

int FindAndDisplayChessboardContinuos(cv::Mat image,int board_w,int board_h, std::vector<cv::Point2f> *corners)
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
	cv::imshow("Calibration1",image);
	printf("\n Number of corners: %lu",corners->size());
  cv::waitKey(30);
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

  char filename[200];

  // Chessboard coordinates and image pixels
  std::vector<std::vector<cv::Point3f> > object_points;
  std::vector<std::vector<cv::Point2f> > image_points;

  // Corners detected in each image
  std::vector<cv::Point2f> corners;

  int corner_count;

  cv::Mat image;
  int i;

  int sucesses = 0;


  // chessboard coordinates
  std::vector<cv::Point3f> obj;
  for(int j=0;j<board_sz;j++)
    obj.push_back(cv::Point3f(float(j/board_w), float(j%board_w), 0.0));


  std::vector<cv::Point3f> points;
  std::vector<cv::Point2f> i_points;


  points.push_back(cv::Point3d(0.0, 0.0, 0.0));
  points.push_back(cv::Point3d(0.0, 0.0, 1.0));



  cv::VideoCapture cap(0); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
      return -1;


  for(;;){
    corners.clear();
    image_points.clear();
    object_points.clear();
    cap >> image;

    corner_count = FindAndDisplayChessboardContinuos(image.clone(),board_w,board_h,&corners);

  	if (corner_count == board_w * board_h)
      {
        image_points.push_back(corners);
  	    object_points.push_back(obj);
        sucesses++;

        cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
        cv::Mat distCoeffs;
        std::vector<cv::Mat> rotation_vectors;
        std::vector<cv::Mat> translation_vectors;

        calibrateCamera(object_points,image_points,image.size(),intrinsic,distCoeffs,rotation_vectors,translation_vectors);

        projectPoints(points,rotation_vectors.at(0),translation_vectors.at(0),intrinsic,distCoeffs,i_points);

        cv::line(image,i_points.at(0),i_points.at(1),cvScalar(255,0,0),2, 0);

        //cv::imshow("Result",image);

      }

      cv::imshow("Result",image);
    }



  return 0;
}
