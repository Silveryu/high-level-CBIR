/*  Snippet code for Operations with images tutorial (not intended to be run but should built successfully) */

#include "opencv2/core.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int,char**)
{
  Mat img = imread("ua.jpeg");
    img = Scalar(0);
    Rect r(10, 10, 100, 100);
    Mat smallImg = img(r);
    namedWindow("image1", WINDOW_AUTOSIZE);
    namedWindow("image2", WINDOW_AUTOSIZE);
    imshow("image1", img);
    imshow("image2", smallImg);
 waitKey();
    return 0;
}
