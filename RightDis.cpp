#include "stdafx.h"
#include <librealsense2\rs.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>


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

		// Render depth (as picture in pipcture)
		Mat Gray;
		Mat edges;

		Mat frame(Size(aligned_depth_frame.get_width(), aligned_depth_frame.get_height()), CV_8UC3, (void*)other_frame.get_data(), Mat::AUTO_STEP);
		Mat colorframe(Size(other_frame.get_width(), other_frame.get_height()), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

		cvtColor(frame, edges, COLOR_RGB2GRAY);
		GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
		threshold(edges, edges, A_T_min, 255, THRESH_BINARY);

		// Setup SimpleBlobDetector parameters.
		SimpleBlobDetector::Params params;

		// Change thresholds
		params.minThreshold = 1;
		params.maxThreshold = 255;

		// Filter by Area.
		params.filterByArea = true;
		params.minArea = 4000;
		params.maxArea = 100000;

		// Filter by color
		params.filterByColor = true;
		params.blobColor = 0;

		// Filter by Circularity
		params.filterByCircularity = false;
		params.minCircularity = 0.1;

		// Filter by Convexity
		params.filterByConvexity = false;
		params.minConvexity = 0.1;

		// Filter by Inertia
		params.filterByInertia = false;
		params.minInertiaRatio = 0.01;

		// Storage for blobs
		vector<KeyPoint> keypoints;

		// Set up detector with params
		Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

		// Detect blobs
		detector->detect(edges, keypoints);


		// Draw detected blobs as red circles.
		// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
		// the size of the circle corresponds to the size of blob

		Mat im_with_keypoints;
		Mat im_with_keypoints2;

		drawKeypoints(colorframe, keypoints, im_with_keypoints2, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		drawKeypoints(edges, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		// Find the distance to the center of the BLOB

		if (keypoints.size()>0) {

			float x = keypoints[0].pt.x;
			float y = keypoints[0].pt.y;
			//cout << " X = " << x << "\n";
			//cout << " Y = " << y << "\n";

			float distance = aligned_depth_frame.get_distance(x, y);
			cout << " Distance = " << distance << "meters" << "\n";

			// We send our Distance though the ROS system

		}



		imshow("Binary", im_with_keypoints);
		imshow("RGB", im_with_keypoints2);

		// If the bool statement "Adjust_Threshold" is true, do the following
		if (Adjust_Threshold)
		{
			createTrackbar("Threshold", "Binary", &A_T_min, 255, Threshold_Slider);
			Threshold_Slider(0, 0);
		}

		if (waitKey(30) >= 0) break;



		//cvtColor(frame, Gray, COLOR_RGB2GRAY);
		//threshold(Gray, bi, 100, 255, THRESH_BINARY);
		//imshow("come on", bi);
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





















/*
#include "stdafx.h"
#include <librealsense2\rs.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>


using namespace cv;
using namespace std;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

string intToString(int number) {


	std::stringstream ss;
	ss << number;
	return ss.str();
}

void drawObject(int x, int y, Mat &frame) {

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

	//UPDATE:JUNE 18TH, 2013
	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);

}




int main(int argc, char * argv[])
{

	bool work = true;
	// Create a pipeline to easily configure and start the camera
	rs2::pipeline pipe;
	//Calling pipeline's start() without any additional parameters will start the first device
	// with its default streams.
	//The start function returns the pipeline profile which the pipeline used to start the device
	rs2::pipeline_profile profile = pipe.start();



	while (work) // Application still alive?
	{
		// Using the align object, we block the application until a frameset is available
		rs2::frameset frameset = pipe.wait_for_frames();

		rs2::frameset data2 = pipe.wait_for_frames();

		rs2::depth_frame depth2 = data2.get_depth_frame();

		rs2::frame color = frameset.get_color_frame();


		Mat Gray;
		Mat edges;

		Mat frame(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);


		cvtColor(frame, edges, COLOR_BGR2GRAY);
		GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
		threshold(edges, edges, 100, 255, THRESH_BINARY);


		// Setup SimpleBlobDetector parameters.
		SimpleBlobDetector::Params params;

		// Change thresholds
		params.minThreshold = 1;
		params.maxThreshold = 255;

		// Filter by Area.
		params.filterByArea = true;
		params.minArea = 1000;
		params.maxArea = 10000;

		// Filter by color
		params.filterByColor = true;
		params.blobColor = 0;

		// Filter by Circularity
		params.filterByCircularity = true;
		params.minCircularity = 0.1;

		// Filter by Convexity
		params.filterByConvexity = false;
		params.minConvexity = 0.1;

		// Filter by Inertia
		params.filterByInertia = false;
		params.minInertiaRatio = 0.01;


		// Storage for blobs
		vector<KeyPoint> keypoints;

		// Set up detector with params
		Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

		// Detect blobs
		detector->detect(edges, keypoints);


		// Draw detected blobs as red circles.
		// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
		// the size of the circle corresponds to the size of blob

		Mat im_with_keypoints;
		Mat im_with_keypoints2;


		drawKeypoints(frame, keypoints, im_with_keypoints2, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		drawKeypoints(edges, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		//Find the distance to the center of the BLOB
		if (keypoints.size() > 0) {

			float x = keypoints[0].pt.x;
			float y = keypoints[0].pt.y;
			cout << " X = " << x << "\n";
			cout << " Y = " << y << "\n";

			drawObject(x, y, im_with_keypoints2);
			float distance = depth2.get_distance(x, y);
			cout << " Distance = " << distance << "meters" << "\n";

		}


		imshow("Binary", im_with_keypoints);
		imshow("RGB", im_with_keypoints2);
		if (waitKey(30) >= 0) break;


		/*cvtColor(frame, Gray, COLOR_RGB2GRAY);
		threshold(Gray, bi, 100, 255, THRESH_BINARY);
		imshow("come on", bi);
	}

	return EXIT_SUCCESS;
}
*/
