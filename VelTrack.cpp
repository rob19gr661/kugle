#include "iostream"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"

#include "kugle_pkg/PersonArray.h"


#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

using namespace std;


double starttime = 0;
double lasttime = 0;

float test;
nav_msgs::Odometry turtle_twist;


//void callback(const geometry_msgs::Twist::ConstPtr & msg){
void callback(const nav_msgs::Odometry::ConstPtr & msg){

  turtle_twist.twist.twist.linear.x = msg->twist.twist.linear.x;

}



void LidarDistance(const kugle_pkg::PersonArray::ConstPtr & dist){
  geometry_msgs::Point32 firstlidarpoint, distancediff;

 //firstlidarpoint.x = static_cast<float>(dist->people[0].pose.position.x);
 firstlidarpoint.x = dist->people[0].pose.position.x;
 double nowtime = ros::Time::now().toSec();

 distancediff.x = firstlidarpoint.x - test;






 //test = oldpos.x;

 test = firstlidarpoint.x;

 //ROS_INFO("time: %x", dist->header.stamp.to_sec());
 //       double nowtime = ros::Time::now().toSec();
 //double timediff = newtime - starttime;
 double timediff = nowtime - lasttime;
 //starttime = newtime;
lasttime = nowtime;

 float vel = distancediff.x/timediff;
 float vel_relative = vel + abs(turtle_twist.twist.twist.linear.x);
 float vel_diff = abs(vel) - abs(turtle_twist.twist.twist.linear.x);

 cout << "twist: \t" << turtle_twist.twist.twist.linear.x << "\t";
 cout << "vel: \t " << vel << "\t";
 cout << "vel_diff: \t " << vel_diff << "\t";
 cout << "vel_relative: \t " << vel_relative << endl;
 //ROS_INFO("Vel: %f \t Vel_realative: %f \t twist: %f \n", vel, vel_relative, turtle_twist)

}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "test");
    ros::NodeHandle n;
  while(ros::ok()){
    //ros::Subscriber vel_sub = n.subscribe<geometry_msgs::Twist>("odom", 100, callback);

    ros::Subscriber vel_sub = n.subscribe<nav_msgs::Odometry>("odom", 100, callback);

    ros::Subscriber distance_sub = n.subscribe<kugle_pkg::PersonArray>("people_tracked", 10, LidarDistance);

    ros::spin();
}
    return 0;

}
