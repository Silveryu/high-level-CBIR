/**
 * @file Sobel_Demo.cpp
 * @brief Sample code uses Sobel or Scharr OpenCV functions for edge detection
 * @author OpenCV team
 */

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>

using namespace cv;
using namespace std;

/**
 * @function main
 */

int main( int argc, char** argv )
{
  cv::CommandLineParser parser(argc, argv,
                               "{@input   |../data/lena.jpg|input image}"
                               "{ksize   k|1|ksize (hit 'K' to increase its value)}"
                               "{scale   s|1|scale (hit 'S' to increase its value)}"
                               "{delta   d|0|delta (hit 'D' to increase its value)}"
                               "{help    h|false|show help message}");



  cout << "The sample uses Sobel or Scharr OpenCV functions for edge detection\n\n";
  parser.printMessage();
  cout << "\nPress 'ESC' to exit program.\nPress 'R' to reset values ( ksize will be -1 equal to Scharr function )";

  //![variables]
  // First we declare the variables we are going to use
  Mat frame,src, src_gray;
  Mat grad;
  Mat image;
  const String window_name = "Sobel Demo - Simple Edge Detector";
  int ksize = parser.get<int>("ksize");
  int scale = parser.get<int>("scale");
  int delta = parser.get<int>("delta");
  int ddepth = CV_16S;
  //![variables]

  //![load]
  String imageName = parser.get<String>("@input");
  // As usual we load our source image (src)
  image = imread( imageName, IMREAD_COLOR ); // Load an image

  // Check if image is loaded fine
  //if( image.empty() )
  //{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
      return -1;
  //}
  //![load]

  for (;;)
  {
    if( image.empty() )
    {
      cap >> frame;
    //  GaussianBlur(frame, src, Size(3, 3), 0, 0, BORDER_DEFAULT);
    }else{
    //  GaussianBlur(image, src, Size(3, 3), 0, 0, BORDER_DEFAULT);
    }

    //![reduce_noise]
    // Remove noise by blurring with a Gaussian filter ( kernel size = 3 )

    //![reduce_noise]src

    //![convert_to_gray]
    // Convert the image to grayscale
    cvtColor(frame, src_gray, COLOR_BGR2GRAY);
    //![convert_to_gray]

    //![sobel]
    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    Sobel(src_gray, grad_x, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT);

    /// Gradient Y
    Sobel(src_gray, grad_y, ddepth, 0, 1, ksize, scale, delta, BORDER_DEFAULT);
    //![sobel]

    //![convert]
    // converting back to CV_8U
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);
    //![convert]

    //![blend]
    /// Total Gradient (approximate)
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
    //![blend]

    //![display]
    imshow(window_name, grad);
    char key = (char)waitKey(1);
    //![display]
    cout << delta;
    if(key == 27)
    {
      return 0;
    }

    if (key == 'k' || key == 'K')
    {
      ksize = ksize < 30 ? ksize+2 : -1;
    }

    if (key == 's' || key == 'S')
    {
      scale++;
    }

    if (key == 'd' || key == 'D')
    {
      delta++;
    }

    if (key == 'r' || key == 'R')
    {
      scale =  1;
      ksize = -1;
      delta =  0;
    }
  }
  return 0;
}
