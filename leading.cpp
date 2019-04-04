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

void move(double speed, double distance, bool isForward);

void cameradistance(const std_msgs::Float32::ConstPtr & dist){
  camdist = dist->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlesim_cleaner");
    ros::NodeHandle n;

   velocity_publisher = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
   distance_sub = n.subscribe("chatter", 1000, cameradistance);

    double speed, distance;
    double angularspeed,angle;
    bool isForward;


  ros::Rate loop_rate(10);
    //distance = speed*time

    speed = 0.01;
    distance = 4;
    isForward = 1;
	move(speed, distance, isForward);


    ros::spin();

    return 0;
}

void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis

		
if (camdist > 1.2){
	speed = speed - 0.01;
}
else{
	speed = speed + 0.03;
}


	if (isForward){
	vel_msg.linear.x =abs(speed);
}else{
	vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
}
	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	
	




	do{	

velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);



	
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);


	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);


}
