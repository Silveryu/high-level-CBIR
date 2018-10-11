#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;


const int threshold_slider_max = 300;
int threshold_slider=0;
Mat edges;
Mat frame;
int KERNEL_LENGTH = 31;
int option;

void on_trackbar( int, void*)
{

 //adaptiveThreshold(edges, edges, threshold_slider, ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,7,5);
  if(threshold_slider!=0){
    switch (option) {
      case 0:
          blur( frame, edges, Size( threshold_slider, threshold_slider ), Point(-1,-1) );
        break;
      case 1:
          GaussianBlur( frame, edges, Size( 63, 63 ), 0, 0 );
        break;
      case 2:
          medianBlur ( frame, edges,11);
        break;
      case 3:
          bilateralFilter ( frame, edges, 5, threshold_slider, threshold_slider);
        break;
      case 4:
          boxFilter(frame, edges,-1,Size( threshold_slider, threshold_slider ),Point(-1,-1),true, BORDER_DEFAULT);
          break;
    }
    imshow("edges", edges);
  }else{
    imshow("edges",frame);
  }


}


int main(int, char* argv[])
{
  //  Mat edges;

    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;


    namedWindow("edges",1);
    for(;;)
    {
        cap >> frame; // get a new frame from camera
        //cvtColor(frame, edges, COLOR_RGB2GRAY);

        option=atoi(argv[1]);

        char TrackbarName[50];
        createTrackbar( TrackbarName, "edges", &threshold_slider, threshold_slider_max, on_trackbar );

        on_trackbar( threshold_slider,0);

      //  imshow("edges", edges);

      //  imshow("edges", edges);
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
