#include <stdlib.h>
#include "ros/ros.h"
#include <iostream>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"

using namespace std;
ros::Publisher velocity_publisher;
ros::Subscriber distance_sub;


float camdist;
double speed=0.4;

void move(double speedmove, double distance);

void cameradistance(const std_msgs::Float32::ConstPtr & dist){
  camdist = dist->data;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlesim_cleaner");
    ros::NodeHandle n;

   velocity_publisher = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
   distance_sub = n.subscribe("CamDist", 1000, cameradistance);

double  distance;
double angularspeed,angle;


  ros::Rate loop_rate(10);
    //distance = speed*time

    
    distance = 10;

	move(speed, distance);


    ros::spin();

    return 0;
}

void move(double speedmove, double distance){
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis



	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(10);



	while(current_distance<distance && speed < 1.5) {
	if (camdist > 1.2){
		speed = speed - 0.1;
vel_msg.linear.x =abs(speed);
	}
	else{
		speed = speed + 0.1;
vel_msg.linear.x =abs(speed);
	}
ROS_INFO("speed %f",speed);
ROS_INFO("speed %f",camdist);
	velocity_publisher.publish(vel_msg);
	double t1 = ros::Time::now().toSec();

	current_distance = speed * (t1-t0);

	ros::spinOnce();
	loop_rate.sleep();

	}
	/*do{	


	velocity_publisher.publish(vel_msg);
	double t1 = ros::Time::now().toSec();
	current_distance = speed * (t1-t0);



	
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);*/


	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);

}
