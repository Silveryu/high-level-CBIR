#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;

/// Global Variables
const int threshold_slider_max = 255;
int threshold_slider=255;
Mat edges;

void on_trackbar( int, void* )
{

 adaptiveThreshold(edges, edges, threshold_slider, ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,7,5);

 imshow("edges", edges);

}

int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    int frame_width = cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(CAP_PROP_FRAME_HEIGHT);
    VideoWriter video("outcpp.avi",VideoWriter::fourcc('M','J','P','G'),10, Size(frame_width,frame_height));

    namedWindow("edges",1);
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        cvtColor(frame, edges, COLOR_RGB2GRAY);

        /// Create Trackbars
        char TrackbarName[50];
        sprintf( TrackbarName, "Threshold x %d", threshold_slider_max );


        createTrackbar( TrackbarName, "edges", &threshold_slider, threshold_slider_max, on_trackbar );

        on_trackbar( threshold_slider, 0 );


        // If the frame is empty, break immediately
           if (edges.empty())
             break;

           // Write the frame into the file 'outcpp.avi'
           video.write(edges);

      //  imshow("edges", edges);
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
