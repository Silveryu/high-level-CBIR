/**
 * @file CannyDetector_Demo.cpp
 * @brief Sample code showing how to detect edges using the Canny Detector
 * @author OpenCV team
 */

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;

//![variables]
Mat src, src_gray;
Mat dst, detected_edges;

int lowThreshold = 0;
const int max_lowThreshold = 100;
const int ratio = 3;
const int kernel_size = 5;
const char* window_name = "Edge Map";
//![variables]

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
static void CannyThreshold(int, void*)
{
    //![reduce_noise]
    /// Reduce noise with a kernel 3x3
    blur( src_gray, detected_edges, Size(3,3) );
    //![reduce_noise]

    //![canny]
    /// Canny detector
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
    //![canny]

    /// Using Canny's output as a mask, we display our result
    //![fill]
  //  dst = Scalar::all(0);
    //![fill]

    //![copyto]
  //  src.copyTo( dst, detected_edges);
    //![copyto]

    //![display]
    imshow( window_name, detected_edges );
    //![display]
}


/**
 * @function main
 */
int main( int argc, char** argv )
{
  //![load]
  CommandLineParser parser( argc, argv, "{@input | ../data/fruits.jpg | input image}" );
  src = imread( parser.get<String>( "@input" ), IMREAD_COLOR ); // Load an image

  if( src.empty() )
  {


  }

  VideoCapture cap(0); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
      return -1;

  Mat frame;
  namedWindow( window_name, WINDOW_AUTOSIZE );
  createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );


  for (;;) {
    if(src.empty()){
        cap >> frame;
    }else{
        frame = src.clone();
    }

    dst.create( frame.size(), frame.type() );
    cvtColor( frame, src_gray, COLOR_BGR2GRAY );
    CannyThreshold(0, 0);

    waitKey(1);
  }
  //![load]

  //![create_mat]
  /// Create a matrix of the same type and size as src (for dst)
  //![create_mat]

  //![convert_to_gray]

  //![convert_to_gray]

  //![create_window]

  //![create_window]

  //![create_trackbar]
  /// Create a Trackbar for user to enter threshold

  //![create_trackbar]

  /// Show the image


  /// Wait until user exit program by pressing a key


  return 0;
}
