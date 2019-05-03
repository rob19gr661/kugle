//For realsense
#include <librealsense2/rs.hpp>

//For OpenCV
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/face.hpp>

//Normal
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>
#include <sstream>
//#include "pch.h"
#include <cstdio>
#include <ctime>
#include <time.h>

//For ROS
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

using namespace cv;
using namespace std;


void velocity(float);

//void remove_background(rs2::video_frame& other, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist);
float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);



//For velocity
float distance_old =0.5;

// Define a variable for controlling the distance to clip
//float depth_clipping_distance = 0.6f;

double t0 = 0.0;

int main(int argc, char * argv[]) try
{

  ros::init(argc, argv, "VelCal");
  ros::Time::init();

    /*
    //For ROS publishing
    ros::init(argc, argv, "CamDistance");
    ros::NodeHandle N;

    ros::Publisher CamDist_pub = N.advertise<std_msgs::Float32>("CamDist", 1);
    ros::Rate loop_rate(100);
    */

    //For realsense depthstream
  	rs2::colorizer c;        // Helper to colorize depth images
  	bool work = true;

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
//float t1; //ros::Time::now().toSec();


  	while (ros::ok()) // Application still alive?
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
  		//remove_background(other_frame, aligned_depth_frame, depth_scale, depth_clipping_distance);


  		Mat frame(Size(aligned_depth_frame.get_width(), aligned_depth_frame.get_height()), CV_8UC3, (void*)other_frame.get_data(), Mat::AUTO_STEP);
  		Mat colorframe(Size(other_frame.get_width(), other_frame.get_height()), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
      cvtColor(colorframe, colorframe, COLOR_BGR2RGB);


      bool work = true;
  		if (work) {

        float x = other_frame.get_width()/2;
        float y = other_frame.get_height()/2;
        float distance_new = aligned_depth_frame.get_distance(x, y);
        circle(colorframe, Point(x,y), 5, (0,0,255), -1);
        //velocity(distance_new);
        printf("Distance\t%f\n",distance_new);

        waitKey(10);


  		}



  		//imshow("Binary", im_with_keypoints);
      //imshow("RGB", im_with_keypoints2);

      imshow("RGB", colorframe);


  		if (waitKey(30) >= 0) break;



  		//cvtColor(frame, Gray, COLOR_RGB2GRAY);
  		//threshold(Gray, bi, 100, 255, THRESH_BINARY);
  		//imshow("come on", bi);
  	}

	return EXIT_SUCCESS;
}



//Don't think about the code below..................................................................................................................




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


void velocity(float distance_new){
  //float x = keypoints[0].pt.x;
  //float y = keypoints[0].pt.y;
  //cout << " X = " << x << "\n";
  //cout << " Y = " << y << "\n";

  // = aligned_depth_frame.get_distance(x, y);

  //distance_new = aligned_depth_frame.get_distance(x, y);
  float diff = distance_old - distance_new;
  //float time_diff = t1 - t0;

  distance_old = distance_new;
  double t1 = ros::Time::now().toSec();
  double time_diff = t0-t1;
  double Vel = diff/time_diff;
  //printf("old %f\tnew %f\tdiff %f\ttime0 %f\ttime1 %f\ttime_diff %f\tvel %f\n",distance_old,distance_new,diff,t0, t1, time_diff,Vel);
  //printf("vel\t%f\n",Vel);
  ROS_INFO("vel\t%f\n",Vel);



  //cout << " Distance = " << distance << "meters" << "\n";



  /*
  // We send our Distance though the ROS system
  std_msgs::Float32 msg;
  msg.data = distance;
  CamDist_pub.publish(msg);
  ros::spinOnce();
  loop_rate.sleep();
  */
  t0=t1;
}
