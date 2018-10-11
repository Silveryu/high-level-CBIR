#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;


int main(int, char**)
{
    VideoCapture vcap(0); // open the default camera
    if(!vcap.isOpened())  // check if we succeeded
        return -1;
    
    Mat gray;
    Mat bw;
    //namedWindow("edges gray",1); 
    //namedWindow("edges bw" ,1);

    int frame_width=   vcap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height=   vcap.get(CV_CAP_PROP_FRAME_HEIGHT);
    // color = false only work in windows, wtf
    cout << CV_CAP_PROP_FPS;
    VideoWriter video_gray("out_gray.avi",CV_FOURCC('M','J','P','G'),CV_CAP_PROP_FPS, Size(frame_width,frame_height),true);
    VideoWriter video_bw("out_bw.avi",CV_FOURCC('M','J','P','G'),CV_CAP_PROP_FPS, Size(frame_width,frame_height),true);

    namedWindow("Video Gray");
    namedWindow("Video BW");
    int maxValue = 128;
    int threshold_value;
    int max_BINARY_value = 255;

    for(;;){

        Mat frame;
        vcap >> frame;
        cvtColor(frame, gray, CV_BGR2GRAY);
        createTrackbar("maxValue", "Video BW", &threshold_value, 255);
        
        //  adaptiveThreshold(gray, bw, maxValue, ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,15,-5);
        threshold( gray, bw, threshold_value, max_BINARY_value,THRESH_BINARY);

        // conversion to BGR is needed to write the video to disk, the color of the video won't change becouse of this
        cvtColor(gray, gray, CV_GRAY2BGR);
        cvtColor(bw, bw, CV_GRAY2BGR);

        imshow( "Video Gray", gray );
        imshow( "Video BW", bw);

        video_gray.write(gray);
        video_bw.write(bw);

       
       // exit on ESC
       if( (char)waitKey(33) == 27 ) break;
    }
    

/*     for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        outputVideo_gray.write(frame);
        
        cvtColor(frame, edges_gray, CV_BGR2GRAY);
        adaptiveThreshold(edges_gray, edges_bw, 255, ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,15,-5);
        //imshow("edges gray", edges_gray);
        //imshow("edges bw", edges_bw);

        if(waitKey(30) >= 0) break;
    }
    cap.release();
    outputVideo_gray.release(); */

    destroyAllWindows();
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
