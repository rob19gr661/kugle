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
		}
	}
}

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
	// Quit if there are not enough images for this.
	if (images.size() <= 1) {
		string error_message = "This demo needs at least 2 images to work. Please add more images to your data set!";
		CV_Error(Error::StsBadArg, error_message);
	}
	//Used for resizing images later
	int im_width = images[0].cols;
	int im_height = images[0].rows;

	// Create a new Fisherfaces model and retain all available Fisherfaces,
	// this is the most common usage of this specific FaceRecognizer:
	Ptr<face::FisherFaceRecognizer> model = face::FisherFaceRecognizer::create();
	
	//Train recognizer model
	model->train(images, labels);

	//Get video stream from webcam
	VideoCapture cap(0);
	if (!cap.isOpened()) { //check if video device has been initialised
		cout << "cannot open camera";
	}
	//Initialize classifier
	CascadeClassifier face_cascade;
	face_cascade.load("D:\\VS_projects\\test2\\haarcascades\\haarcascade_frontalface_alt2.xml");

	namedWindow("video", WINDOW_AUTOSIZE);


	// Now updating the model is as easy as calling:
	//model->update(newImages, newLabels);

	//conditional loop
	while (&VideoCapture::isOpened) { //Only do stuff if the camera is running
		Mat cameraFrame;
		Mat grayscale;
		vector<Rect> faces;
		
		Scalar color = Scalar(255, 0, 0); // Color for Drawing tool
		while (!cap.read(cameraFrame));
		cap.read(cameraFrame);
		cvtColor(cameraFrame, grayscale, COLOR_BGR2GRAY);
		
		//Detect faces in grayscale frames, and store them in vector faces
		face_cascade.detectMultiScale(grayscale, faces, 1.1, 2, CASCADE_FIND_BIGGEST_OBJECT, Size(80, 80)); 
		printf("%zd face(s) are found.\n", faces.size());

		//For-loop for running through all the detected faces
		for (int i = 0; i < faces.size(); i++) {
			Rect r = faces[i];
			Mat face = grayscale(r); //Making detected faces grayscale

			Mat face_resized; //Variable for holding resized images
			cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC); //FisherRecognizer need images to be same size
			int prediction = model->predict(face_resized); //Predic label of face

			//Draw a rectangle around detected faces
			rectangle(cameraFrame, Point(cvRound(r.x*scale), cvRound(r.y*scale)), Point(cvRound((r.x + r.width - 1)*scale), cvRound((r.y + r.height - 1)*scale)), color, 3, 8, 0);
			string box_text = format("Prediction = %d", prediction); //text to put on the rectangle
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

		imshow("video", cameraFrame);
		if (waitKey(30) >= 0)
			break;
	}
	return 0;
}
