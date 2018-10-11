
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <libgen.h>

using namespace std;
using namespace cv;
const char* keys =
    "{ help  h| | Print help message. }"
    "{ img    | | Path to input image. }"
    "{ folder | | Path to input folder. }";
int main( int argc, char** argv )
{
    CommandLineParser parser( argc, argv, keys );
    String imgFilename = parser.get<String>("img");
    Mat src_img = imread(imgFilename);
    String folder = parser.get<String>("folder");

    vector<String> filenames;
    glob(folder, filenames);

    if( src_img.empty())
    {
        cout << "Could not open or find the images!\n" << endl;
        parser.printMessage();
        return -1;
    }
    Mat hsv_src, hsv_cmp;
    cvtColor( src_img, hsv_src, COLOR_BGR2HSV );

    int h_bins = 50, s_bins = 60;
    int histSize[] = { h_bins, s_bins };
    // hue varies from 0 to 179, saturation from 0 to 255
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };
    const float* ranges[] = { h_ranges, s_ranges };
    // Use the 0-th and 1-st channels
    int channels[] = { 0, 1 };

    Mat hist_src;
    calcHist( &hsv_src, 1, channels, Mat(), hist_src, 2, histSize, ranges, true, false );
    normalize( hist_src, hist_src, 0, 1, NORM_MINMAX, -1, Mat() );

    String minDistfilename = filenames[0];
    Mat closest_img = imread(minDistfilename);
    double minDist = 3; // value not possible in our distance measure (1-pearsons corr)
    double dist;
    String filename;

    String src_basename(basename(const_cast<char*>((imgFilename.operator std::string()).c_str())));




    for(size_t i = 0; i < filenames.size(); ++i)
    {
        
        filename = filenames[i];
        String cmp_basename(basename(const_cast<char*>((filename.operator std::string()).c_str())));

        
        if(cmp_basename == src_basename){
            continue;
        }
        Mat cmp_img = imread(filename);
        if(!cmp_img.data)
            cerr << "Problem loading image!!!" << endl;

        /* do whatever you want with your images here */
        
        // get hist of image
        cvtColor( cmp_img, hsv_cmp, COLOR_BGR2HSV );
        Mat hist_cmp;
        calcHist( &hsv_cmp, 1, channels, Mat(), hist_cmp, 2, histSize, ranges, true, false );
        normalize( hist_cmp, hist_cmp, 0, 1, NORM_MINMAX, -1, Mat() );

        dist = 1 - compareHist( hist_src, hist_cmp, CV_COMP_CORREL );
        if(dist < minDist){
            closest_img = cmp_img;
            minDist = dist;
            minDistfilename = filename;
        }
    }

    namedWindow("source");
    namedWindow("closest image");
    
    imshow( "source", src_img );                // Show our image inside it.
    imshow( "closest image", closest_img );

    cout << "image with minimum distance: " << minDistfilename << endl;
     while (true) {
        if( (char)waitKey(0) == 27 ) break;
    }

    cout << "Method " << "CV_COMP_CORREL" << " Dist:" <<  minDist << endl;
    return 0;
}