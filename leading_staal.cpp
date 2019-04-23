#include <stdlib.h>
#include "ros/ros.h"
#include <iostream>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <fstream>

using namespace std;
ros::Publisher velocity_publisher;
ros::Subscriber distance_sub;
ros::Subscriber pose_subscriber;

nav_msgs::Odometry turtlebot_odom_pose;
	geometry_msgs::Twist vel_msg;

int staal = 2;


void move(double speedmove, double distance);
bool GoToGoal(nav_msgs::Odometry  goal_pose, double distance_tolerance);
void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message);
nav_msgs::Odometry setGoalPosition(double x, double y);

float camdist;
double speed=0.001;
float maxspeed = 0.4;


void cameradistance(const std_msgs::Float32::ConstPtr & dist){
  camdist = dist->data;
}

float speedup(){
if (speed >= maxspeed){
	speed = maxspeed;
	vel_msg.linear.x =abs(speed);
	}else {
		speed = speed + 0.01;
		vel_msg.linear.x =abs(speed);
	}

}
float speeddown(){
	if (speed <= 0.1){
	speed = 0.1;
	vel_msg.linear.x =abs(speed);
	}else {
		speed = speed - 0.01;
		vel_msg.linear.x =abs(speed);
	}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlesim_cleaner");
    ros::NodeHandle n;

   velocity_publisher = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
//velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);
	distance_sub = n.subscribe("CamDist", 1000, cameradistance);
	pose_subscriber = n.subscribe("/odom", 10, poseCallback);

double  distance;
double angularspeed,angle;


  ros::Rate loop_rate(10);
    //distance = speed*time

    
    distance = 4;

move(speed, distance);

//nav_msgs::Odometry  goal_pose = setGoalPosition(1.5, 3.5);
//GoToGoal(goal_pose, 0.3);

    ros::spin();

    return 0;
}
	
void move(double speedmove, double distance){
//	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis



	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(10);

while(ros::ok() && current_distance<distance) {

int tooclose, toofar, is;
    
float mean;
    
		
vel_msg.linear.x =abs(speed);
	    
	for(int i=1;i<20;i++){

	is+=camdist
		if(camdist<2.5 && camdist>1.2)
			vel_msg.linear.x =abs(speed);
		else if(camdist < 1.2){
			if(camdist < 0.6){
				tooclose++;
			}else if(tooclose == 2){
				//something();
				speeddown();
				//maybe set a varible that changes the max/min for the robot when returning
			}
			speedup();
			}
		else if(camdist > 2.5){
			if(camdist > 3.5){
				toofar++;
			}else if(toofar == 2){
				//something();
				speeddown();
				//maybe set a varible that changes the max/min for the robot when returning
			}
			speeddown();
			}
		else if (distance-current_distance < 1.0) {
			speeddown();
		}
	}mean = is/count;

	/*while(ros::ok() && current_distance<distance) {

		vel_msg.linear.x =abs(speed);
	
		if(camdist<2.5 && camdist>1.2)
			vel_msg.linear.x =abs(speed);
		else if(camdist < 0.8){
			speedup();
			}
		else if(camdist > 1.0){
			speeddown();
			}
		else if (distance-current_distance < 1.0) {
			speeddown();
			//speed = 0.1;
			//vel_msg.linear.x =abs(speed);
			}*/


/*
if (speed <= maxspeed){


	if (camdist > 1.2){
		speed = speed - 0.01;
vel_msg.linear.x =abs(speed);
	}
	if (camdist < 2.5){
		speed = speed + 0.01;
vel_msg.linear.x =abs(speed);
	}

} else {
speed = 0.3;
vel_msg.linear.x =abs(speed);
}


if (distance-current_distance < 2.5) {
speed = 0.1;
vel_msg.linear.x =abs(speed);
}
*/

ROS_INFO("%f\t %f\t %f\t",speed,current_distance,camdist);
//ROS_INFO("speed %f",speed);
//ROS_INFO("dist %f",current_distance);
//ROS_INFO("camdist %f",camdist);
	velocity_publisher.publish(vel_msg);
	double t1 = ros::Time::now().toSec();

	current_distance = speed * (t1-t0);

	ros::spinOnce();
	loop_rate.sleep();

	}


	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);


}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

bool GoToGoal(nav_msgs::Odometry  goal_pose, double distance_tolerance){

	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(100);
	double E = 0.0;
	double Xr=0.0;
	double Yr=0.0;
	double Xg=0.0;
	double Yg=0.0;
	ofstream outfile;
	outfile.open("distancelog.cvs");
	do{
		/****** Proportional Controller ******/
		//gains of the proportional controllers
		double Kp_v=1.0;
		double Kp_w=4.0;

		//get coordinates of the robot
		Xr = turtlebot_odom_pose.pose.pose.position.x;
		Yr = turtlebot_odom_pose.pose.pose.position.y;

		//get the coordinates of the goal location
		Xg = goal_pose.pose.pose.position.x;
		Yg = goal_pose.pose.pose.position.y;

		double e = getDistance(Xr, Yr, Xg,Yg);
		//double E = E+e;
		double theta_robot = tf::getYaw(turtlebot_odom_pose.pose.pose.orientation);
		double theta_goal = atan2(Yg-Yr, Xg-Xr);
		//Kp = v0 * (exp(-alpha)*error*error)/(error*error);

		//linear velocity control
		vel_msg.linear.x = (Kp_v*e)/abs(Kp_w*(theta_goal-theta_robot));
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;

		//angular velocity control
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =Kp_w*(theta_goal-theta_robot);

		//publish the velocity message
		velocity_publisher.publish(vel_msg);

		ros::spinOnce();

		loop_rate.sleep();
		//watch dog
		/*if (minDistanceToObstacle()<DSafe){
			ROS_INFO("Goal could not be reached. Stopped because of an obstacle\n");
			vel_msg.linear.x =0;
			vel_msg.angular.z = 0;
			velocity_publisher.publish(vel_msg);
			ros::spinOnce();
			return false;
		}*/

		//Statistics


		outfile << e << endl;


	}while(getDistance(Xr, Yr, Xg,Yg)>distance_tolerance);
	outfile.close();
	ROS_INFO("Goal is reached. Mission completed\n");
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
	return true;
}

nav_msgs::Odometry setGoalPosition(double x, double y){
	nav_msgs::Odometry  goal_pose;
	goal_pose.pose.pose.position.x=x;
	goal_pose.pose.pose.position.y=y;

	goal_pose.pose.pose.orientation.w=0;
	goal_pose.pose.pose.orientation.x=0;
	goal_pose.pose.pose.orientation.y=0;
	goal_pose.pose.pose.orientation.z=1;

	return goal_pose;
}

void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message){
	turtlebot_odom_pose.pose.pose.position.x=pose_message->pose.pose.position.x;
	turtlebot_odom_pose.pose.pose.position.y=pose_message->pose.pose.position.y;
	turtlebot_odom_pose.pose.pose.position.z=pose_message->pose.pose.position.z;

	turtlebot_odom_pose.pose.pose.orientation.w=pose_message->pose.pose.orientation.w;
	turtlebot_odom_pose.pose.pose.orientation.x=pose_message->pose.pose.orientation.x;
	turtlebot_odom_pose.pose.pose.orientation.y=pose_message->pose.pose.orientation.y;
	turtlebot_odom_pose.pose.pose.orientation.z=pose_message->pose.pose.orientation.z;
}

