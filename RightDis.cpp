#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include <ros/ros.h>
//#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>


using namespace cv;
using namespace std;

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

    ros::init(argc, argv, "CamDistance");
    ros::NodeHandle N;

    ros::Publisher CamDist_pub = N.advertise<std_msgs::Float32>("CamDist", 1);
    ros::Rate loop_rate(100);

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

		const int w = color.as<rs2::video_frame>().get_width();
		const int h = color.as<rs2::video_frame>().get_height();

		Mat frame(Size(w,h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
		

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

			float distance = depth2.get_distance(x, y);
			cout << " Distance = " << distance << "meters" << "\n";
			drawObject(x, y, im_with_keypoints);


			std_msgs::Float32 msg;
                        msg.data = distance;
                        CamDist_pub.publish(msg);
                        ros::spinOnce();
                        loop_rate.sleep();

		}


		imshow("Binary", im_with_keypoints);
		imshow("RGB", im_with_keypoints2);
		if (waitKey(30) >= 0) break;


		/*cvtColor(frame, Gray, COLOR_RGB2GRAY);
		threshold(Gray, bi, 100, 255, THRESH_BINARY);
		imshow("come on", bi);*/
	}

	return EXIT_SUCCESS;
}
