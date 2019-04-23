#include "stdafx.h"
#include <librealsense2\rs.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>
//#include "pch.h"
#include <cstdio>
#include <opencv2/imgproc.hpp>
#include <opencv2/face.hpp>
#include <fstream>
#include <sstream>


using namespace cv;
using namespace std;

void Threshold_Slider(int, void*);
void remove_background(rs2::video_frame& other, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist);
float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

// Adjust Threshold minimum 
int A_T_min = 50;

// Define a variable for controlling the distance to clip
float depth_clipping_distance = 0.6f;

double scale = 1;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

//Function for getting values from csv file
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

int main(int argc, char * argv[]) try
{

	rs2::colorizer c;                     // Helper to colorize depth images
	bool work = true;
	bool Adjust_Threshold = true;

	// Create a pipeline to easily configure and start the camera
	rs2::pipeline pipe;

	//Calling pipeline's start() without any additional parameters will start the first device
	// with its default streams.
	//The start function returns the pipeline profile which the pipeline used to start the device
	rs2::pipeline_profile profile = pipe.start();

	// Each depth camera might have different units for depth pixels, so we get it here
	// Using the pipeline's profile, we can retrieve the device that the pipeline uses
	float depth_scale = get_depth_scale(profile.get_device());

	//Pipeline could choose a device that does not have a color stream
	//If there is no color stream, choose to align depth to another stream
	rs2_stream align_to = find_stream_to_align(profile.get_streams());

	// Create a rs2::align object.
	// rs2::align allows us to perform alignment of depth frames to others frames
	//The "align_to" is the stream type to which we plan to align depth frames.
	rs2::align align(align_to);






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
	//VideoCapture cap(0);
	//if (!cap.isOpened()) { //check if video device has been initialised
		//cout << "cannot open camera";
	//}
	//Initialize classifier
	CascadeClassifier face_cascade;
	face_cascade.load("D:\\VS_projects\\test2\\haarcascades\\haarcascade_frontalface_alt2.xml");

	namedWindow("video", WINDOW_AUTOSIZE);


	// Now updating the model is as easy as calling:
	//model->update(newImages, newLabels);






	while (1) // Application still alive?
	{

		// Using the align object, we block the application until a frameset is available
		rs2::frameset frameset = pipe.wait_for_frames();
		rs2::frameset super = pipe.wait_for_frames();
		rs2::frameset data2 = pipe.wait_for_frames();

		rs2::frame color = super.get_color_frame();

		// rs2::pipeline::wait_for_frames() can replace the device it uses in case of device error or disconnection.
		// Since rs2::align is aligning depth to some other stream, we need to make sure that the stream was not changed
		//  after the call to wait_for_frames();
		if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
		{
			//If the profile was changed, update the align object, and also get the new device's depth scale
			profile = pipe.get_active_profile();
			align_to = find_stream_to_align(profile.get_streams());
			align = rs2::align(align_to);
			depth_scale = get_depth_scale(profile.get_device());
		}

		//Get processed aligned frame
		auto processed = align.process(frameset);

		// Trying to get both other and aligned depth frames
		rs2::video_frame other_frame = processed.first(align_to);
		rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

		//If one of them is unavailable, continue iteration
		if (!aligned_depth_frame || !other_frame)
		{
			continue;
		}
		// Passing both frames to remove_background so it will "strip" the background
		// NOTE: in this example, we alter the buffer of the other frame, instead of copying it and altering the copy
		//       This behavior is not recommended in real application since the other frame could be used elsewhere
		remove_background(other_frame, aligned_depth_frame, depth_scale, depth_clipping_distance);
		

		Mat frame(Size(aligned_depth_frame.get_width(), aligned_depth_frame.get_height()), CV_8UC3, (void*)other_frame.get_data(), Mat::AUTO_STEP);
		Mat colorframe(Size(other_frame.get_width(), other_frame.get_height()), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

		
		


		

		Mat cameraFrame;
		Mat grayscale;
		vector<Rect> faces;
		
		Scalar color = Scalar(255, 0, 0); // Color for Drawing tool
		//while (!cap.read(cameraFrame));
		//cap.read(cameraFrame);
		cvtColor(frame, grayscale, COLOR_BGR2GRAY);
		
		//Detect faces in grayscale frames, and store them in vector faces
		face_cascade.detectMultiScale(grayscale, faces, 1.1, 2, CASCADE_FIND_BIGGEST_OBJECT, Size(80, 80)); 
		printf("%zd face(s) are found.\n", faces.size());

		//For-loop for running through all the detected faces
		for (int i = 0; i < faces.size(); i++) {
			Rect r = faces[i];
			Mat face = grayscale(r); //Making detected faces grayscale

			int predicted_label = 0;
			double predicted_confidence = 0.0;

			Mat face_resized; //Variable for holding resized images
			cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC); //FisherRecognizer need images to be same size
			//int prediction = model->predict(face_resized); //Predic label of face
			model->predict(face_resized, predicted_label, predicted_confidence);
			
			//Draw a rectangle around detected faces
			rectangle(frame, Point(cvRound(r.x*scale), cvRound(r.y*scale)), Point(cvRound((r.x + r.width - 1)*scale), cvRound((r.y + r.height - 1)*scale)), color, 3, 8, 0);
			
			float center_x = (r.x + (r.x + r.width)) / 2;
			float center_y = (r.y + (r.y + r.height)) / 2;
			//Get distance to centerpoint from realsense
			float distance = aligned_depth_frame.get_distance(center_x, center_y);
			cout << " Distance = " << distance << "meters" << "\n";
			//printf("x1,y1 = (%d,%d). x2,y2 = (%d,%d)",r.x,r.y,r.width,r.height);
			//printf("Center point: (%f , %f)\n", center_x, center_y);

			
			// We send our Distance though the ROS system


			string box_text = format("Prediction = %d", predicted_label); //text to put on the rectangle
			printf("a face is found at Rect(%d,%d,%d,%d). Prediction: %d. - %lf\n", r.x, r.y, r.width, r.height, predicted_label, predicted_confidence);
			// Calculate the position for annotated text (make sure we don't
		    	// put illegal values in there):
			int pos_x = max(r.tl().x - 10, 0);
			int pos_y = max(r.tl().y - 10, 0);
			// And now put it into the image:
			putText(frame, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);

			if (predicted_label == 2 && predicted_confidence < 4000) {
				cout << "Keep driving" << endl;

			}
			else {
				cout << "Face not detected. Keep looking" << endl;
			}

		}

		imshow("video", frame);
		if (waitKey(30) >= 0)
			break;



	


		
	}

	return EXIT_SUCCESS;
}

catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

void Threshold_Slider(int, void*)
{

}

float get_depth_scale(rs2::device dev)
{
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}


void remove_background(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist)
{
	const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
	uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

	int width = other_frame.get_width();
	int height = other_frame.get_height();
	int other_bpp = other_frame.get_bytes_per_pixel();

#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
	for (int y = 0; y < height; y++)
	{
		auto depth_pixel_index = y * width;
		for (int x = 0; x < width; x++, ++depth_pixel_index)
		{
			// Get the depth value of the current pixel
			auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

			// Check if the depth value is invalid (<=0) or greater than the threashold
			if (pixels_distance <= 0.f || pixels_distance > clipping_dist)
			{
				// Calculate the offset in other frame's buffer to current pixel
				auto offset = depth_pixel_index * other_bpp;

				// Set pixel to "background" color (0x999999)
				std::memset(&p_other_frame[offset], 0x99, other_bpp);
			}
		}
	}
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
	//Given a vector of streams, we try to find a depth stream and another stream to align depth with.
	//We prioritize color streams to make the view look better.
	//If color is not available, we take another stream that (other than depth)
	rs2_stream align_to = RS2_STREAM_ANY;
	bool depth_stream_found = false;
	bool color_stream_found = false;
	for (rs2::stream_profile sp : streams)
	{
		rs2_stream profile_stream = sp.stream_type();
		if (profile_stream != RS2_STREAM_DEPTH)
		{
			if (!color_stream_found)         //Prefer color
				align_to = profile_stream;

			if (profile_stream == RS2_STREAM_COLOR)
			{
				color_stream_found = true;
			}
		}
		else
		{
			depth_stream_found = true;
		}
	}

	if (!depth_stream_found)
		throw std::runtime_error("No Depth stream available");

	if (align_to == RS2_STREAM_ANY)
		throw std::runtime_error("No stream found to align with Depth");

	return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
	for (auto&& sp : prev)
	{
		//If previous profile is in current (maybe just added another)
		auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
		if (itr == std::end(current)) //If it previous stream wasn't found in current
		{
			return true;
		}
	}
	return false;
}




