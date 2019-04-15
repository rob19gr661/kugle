// face_recon.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <cstdio>
#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>
//#include <opencv2/tracking.hpp>
#include <fstream>
#include <sstream>


using namespace cv;
using namespace std;

double scale = 1;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

static void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator = ';') {
	std::ifstream file(filename.c_str(), ifstream::in);
	if (!file) {
		string error_message = "No valid input file was given, please check the given filename.";
		CV_Error(Error::StsBadArg, error_message);
	}
	string line, path, classlabel;
	while (getline(file, line)) {
		stringstream liness(line);
		getline(liness, path, separator);
		getline(liness, classlabel);
		if (!path.empty() && !classlabel.empty()) {
			images.push_back(imread(path, 0));
			labels.push_back(atoi(classlabel.c_str()));
			//resize(images, ReImages, cv::Size(3000, 4000));
		}
	}
}

// List of tracker types in OpenCV 3.4.1
//string trackerTypes[8] = { "BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT" };
// vector <string> trackerTypes(types, std::end(types));

 // create a tracker object
//Ptr<Tracker> tracker = TrackerKCF::create();

//Ptr<Tracker> tracker;


//face verification

int main(int argc, const char *argv[])
{
	// These vectors hold the images and corresponding labels.
	vector<Mat> images;
	vector<Mat> ReImages;
	vector<int> labels;
	// Read in the data. This can fail if no valid
	// input filename is given.
	try {
		read_csv("data.csv", images, labels);
		//cout << "File " << 0 << ": " << images[0] << endl;
		cout << "size of image: " << images.size() << endl;
	}
	catch (const cv::Exception& e) {
		cerr << "Error opening file \"" << "\". Reason: " << e.msg << endl;
		// nothing more we can do
		exit(1);
	}
	// Quit if there are not enough images for this demo.
	if (images.size() <= 1) {
		string error_message = "This demo needs at least 2 images to work. Please add more images to your data set!";
		CV_Error(Error::StsBadArg, error_message);
	}

	int im_width = images[0].cols;
	int im_height = images[0].rows;

	// Create a new Fisherfaces model and retain all available Fisherfaces,
	// this is the most common usage of this specific FaceRecognizer:
	//Ptr<face::FaceRecognizer> model = face::FisherFaceRecognizer::create();
	Ptr<face::FisherFaceRecognizer> model = face::FisherFaceRecognizer::create();
	cout << "hello9";
	//string name = model->name();
	model->train(images, labels);

	VideoCapture cap(0);
	if (!cap.isOpened()) { //check if video device has been initialised
		cout << "cannot open camera";
	}
	CascadeClassifier face_cascade;
	face_cascade.load("D:\\VS_projects\\test2\\haarcascades\\haarcascade_frontalface_alt2.xml");
	cout << "hello20";
	namedWindow("video", WINDOW_AUTOSIZE);


	// Now updating the model is as easy as calling:
	//model->update(newImages, newLabels);

	//unconditional loop
	while (&VideoCapture::isOpened) {
		Mat cameraFrame;
		Mat grayscale;
		vector<Rect> faces;
		
		Scalar color = Scalar(255, 0, 0); // Color for Drawing tool
		while (!cap.read(cameraFrame));
		cap.read(cameraFrame);
		cvtColor(cameraFrame, grayscale, COLOR_BGR2GRAY);
		
		face_cascade.detectMultiScale(grayscale, faces, 1.1, 2, CASCADE_FIND_BIGGEST_OBJECT, Size(80, 80));
		printf("%zd face(s) are found.\n", faces.size());

		for (int i = 0; i < faces.size(); i++) {
			Rect r = faces[i];
			Mat face = grayscale(r);

			Mat face_resized;
			cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);
			int prediction = model->predict(face_resized);
			//int predicted_label = 2;
			//double predicted_confidence = 0.0;
				//model->predict(face_resized,predicted_label,predicted_confidence);


			rectangle(cameraFrame, Point(cvRound(r.x*scale), cvRound(r.y*scale)), Point(cvRound((r.x + r.width - 1)*scale), cvRound((r.y + r.height - 1)*scale)), color, 3, 8, 0);
			string box_text = format("Prediction = %d", prediction);
			printf("a face is found at Rect(%d,%d,%d,%d). Prediction: %d.\n", r.x, r.y, r.width, r.height, prediction);
			// Calculate the position for annotated text (make sure we don't
		   // put illegal values in there):
			int pos_x = max(r.tl().x - 10, 0);
			int pos_y = max(r.tl().y - 10, 0);
			// And now put it into the image:
			putText(cameraFrame, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);

			if (prediction == 2) {
				cout << "Keep driving" << endl;

			}
			else {
				cout << "Face not detected. Keep looking" << endl;
			}

		}

		

		//rectangle(cameraFrame, cvPoint(cvRound(r.x*scale), cvRound(r.y*scale)), cvPoint(cvRound((r.x + r.width - 1)*scale), cvRound((r.y + r.height - 1)*scale)), color, 3, 8, 0);
		//imshow("cam", cameraFrame);
		imshow("video", cameraFrame);
		if (waitKey(30) >= 0)
			break;
	}
	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
