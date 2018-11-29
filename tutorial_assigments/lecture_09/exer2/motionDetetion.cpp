#include <iostream>
#include <vector>

// OpenCV Includes
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgproc/imgproc_c.h"



using namespace cv;
using namespace std;


char keyboard; //input from keyboard

int detectMotion(VideoCapture capture, bool refresh) {
	RNG rng(12345);
	bool changed = false;
	Mat frame;
	if (!capture.isOpened())
		throw "Error when reading steam_avi";
	Mat first_frame;
	Mat frame_Delta;
	Mat thresh;
	Mat changed_frame;
	int dilation_type = MORPH_RECT;
	int dilation_size = 0;
	Mat element = getStructuringElement(dilation_type,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	namedWindow("MotionDetetion", 1);
	for (; ; )
	{
		capture >> frame;
		if (frame.empty())
			break;
		//code to analize frame
		cvtColor(frame, changed_frame, CV_BGR2GRAY);
		GaussianBlur(changed_frame, changed_frame, Size(21, 21), 0, 0);
		if (first_frame.empty()) {
			changed_frame.copyTo(first_frame);
		}
		absdiff(first_frame, changed_frame, frame_Delta);
		threshold(frame_Delta, thresh, 25, 255, THRESH_BINARY);

		dilate(thresh, thresh, element);
		vector<Vec4i> hierarchy;
		vector<vector<Point> > contours;
		findContours(thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		vector<vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect(contours.size());
		vector<Point2f>center(contours.size());
		vector<float>radius(contours.size());

		for (int i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
			boundRect[i] = boundingRect(Mat(contours_poly[i]));
			minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
		}

		Mat drawing = Mat::zeros(thresh.size(), CV_8UC3);
		for (int i = 0; i< contours.size(); i++)
		{
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			drawContours(frame, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
			rectangle(frame, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
		}
		if (refresh) {
			if (countNonZero(thresh) < 1) {
				if (!changed) {
					changed_frame.copyTo(first_frame);
					cout << "changed" << endl;
					changed = true;
				}
			}
			else {
				changed = false;
			}

		}

		if (countNonZero(thresh) > 1) {
			putText(frame, "Movement Detected", cvPoint(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(0, 0, 250), 2, CV_AA);
		}

		//imshow("EmotionDetetion", drawing);
		imshow("MotionDetetion", frame);
		//imshow("EmotionDetetion", thresh);
		if (waitKey(30) >= 0) break;
	}
	return 0;
}

int subtration_algorithm(VideoCapture capture) {
	Mat frame;
	if (!capture.isOpened())
		throw "Error when reading steam_avi";
	Mat first_frame;
	Mat frame_Delta;
	Mat thresh;
	Mat changed_frame;
	int dilation_type = MORPH_RECT;
	int dilation_size = 0;
	Mat element = getStructuringElement(dilation_type,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	namedWindow("MotionDetetion", 1);
	namedWindow("Subtration", 1);

	for (; ; )
	{
		capture >> frame;
		if (frame.empty())
			break;
		//code to analize frame
		cvtColor(frame, changed_frame, CV_RGB2GRAY);
		GaussianBlur(changed_frame, changed_frame, Size(21, 21), 0, 0);
		if (first_frame.empty()) {
			changed_frame.copyTo(first_frame);
		}
		absdiff(first_frame, changed_frame, frame_Delta);
		threshold(frame_Delta, thresh, 25, 255, THRESH_BINARY);

		dilate(thresh, thresh, element);


		//imshow("EmotionDetetion", drawing);
		imshow("MotionDetetion", frame);
		imshow("Subtration",thresh);
		//imshow("EmotionDetetion", thresh);
		if (waitKey(30) >= 0) break;
	}
	return 0;
}

int captureVideo() {
	VideoCapture cap(0); // open the default camera
	detectMotion(cap,true);
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}

int readVideoFile(string videoFilename) {
	VideoCapture capture(videoFilename);
	detectMotion(capture,true);
	waitKey(0); // key press to close window
				// releases and window destroy are automatic in C++ interface
	return 0;
}

int processVideoSub(string videoFilename) {
	VideoCapture capture(videoFilename);
  cout << "teste1\n";
	subtration_algorithm(capture);
	waitKey(0); // key press to close window
				// releases and window destroy are automatic in C++ interface
	return 0;
}

void printMenu() {
	cout << "Options:\n";
	cout << "1- Detect Motion From WebCam\n";
	cout << "2- Detect Motion from file\n";
	cout << "3- Analyze subtration with algorithm used to detect motion	\n";
	cout << "0- End Program\n" << endl;

}
void selectOption(int optionChoose) {
	string userInput = "";
	switch (optionChoose)
	{
	case 1:
		captureVideo();
		break;
	case 2:
		cout << "File Name:\n" << endl;
		getline(cin, userInput);
		readVideoFile(userInput); 
		//readVideoFile(userInput);
		break;
	case 3:
		processVideoSub("video1.mp4");
		break;
	default:
		break;
	}
}
int main(int, char**)
{
	string userInput;
	int optionChoose = 0;
	do {
		printMenu();
		getline(cin, userInput);
		optionChoose = stoi(userInput);
		selectOption(optionChoose);
	} while (optionChoose != 0);

	destroyAllWindows();
	return 0;

}
