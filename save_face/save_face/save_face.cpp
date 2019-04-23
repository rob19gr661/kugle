#include "pch.h"
#include <iostream>
#include <cstdio>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <fstream>
#include <string>
//#include <opencv2/tracking.hpp>
//#include <opencv2/face.hpp>
using namespace cv;
using namespace std;



void detectAndDisplay(Mat frame);

string face_cascade_name = "D:\\VS_projects\\test2\\haarcascades\\haarcascade_frontalface_alt2.xml";
CascadeClassifier face_cascade;
string window_name = "Window";
int filenumber;
string filename;
ofstream fs;


int main(void)
{
	//create a name for the file output
	std::string outputFilename = "data1.csv";
	//create and open the .csv file
	fs.open(outputFilename);
	VideoCapture capture(0);

	if (!capture.isOpened())
		return -1;

	if (!face_cascade.load(face_cascade_name))
	{
		cout << "error" << endl;
		return (-1);
	};

	Mat frame;
	int counter = 0;

	for (;;)
	{
		capture >> frame;

		if (!frame.empty())
		{

			detectAndDisplay(frame);
			
		}
		else
		{
			cout << "error2" << endl;
			break;
		}

		int c = waitKey(10);

		if (27 == char(c))
		{
			break;
		}
	}
	fs.close();
	return 0;
}


void detectAndDisplay(Mat frame)
{
	std::vector<Rect> faces;
	Mat frame_gray;
	Mat crop;
	Mat res;
	Mat gray;
	string text;
	stringstream sstm;

	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

	cv::Rect roi_b;
	cv::Rect roi_c;

	size_t ic = 0;
	int ac = 0;

	size_t ib = 0;
	int ab = 0;

	int counter = 0;

	
		for (ic = 0; ic < faces.size(); ic++)
		{
			cout << "hello " << faces.size() << endl;
			if (ic % 100 == 0) {
				cout << "hello1 ";
				// Stuff every other loop


				int label = 0;


				roi_c.x = faces[ic].x;
				roi_c.y = faces[ic].y;
				roi_c.width = (faces[ic].width);
				roi_c.height = (faces[ic].height);

				ac = roi_c.width * roi_c.height;

				roi_b.x = faces[ib].x;
				roi_b.y = faces[ib].y;
				roi_b.width = (faces[ib].width);
				roi_b.height = (faces[ib].height);


				crop = frame(roi_b);
				resize(crop, res, Size(128, 128), 0, 0, INTER_LINEAR);
				cvtColor(crop, gray, COLOR_BGR2GRAY);


				filename = "D:\\VS_projects\\save_face\\save_face";
				stringstream ssfn;
				ssfn << filename.c_str() << filenumber << ".jpg";
				filename = ssfn.str();

				//cv::imwrite(filename, res);
				fs << filename << ";" << label << std::endl;

				filenumber++;

				Point pt1(faces[ic].x, faces[ic].y);
				Point pt2((faces[ic].x + faces[ic].height), (faces[ic].y + faces[ic].width));
				rectangle(frame, pt1, pt2, Scalar(0, 255, 0), 2, 8, 0);
			}
		}
	


	//fs.close();
	sstm << "Crop area size: " << roi_b.width << "x" << roi_b.height << " Filename: " << filename;
	text = sstm.str();

	if (!crop.empty())
	{
		imshow("detected", crop);
	}
	else
		destroyWindow("detected");

}