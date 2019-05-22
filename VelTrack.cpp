#include "iostream"
#include "fstream"
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

ofstream myfile;
double starttime = 0;
double lasttime = 0;
float vel_relative;
float test;
float vel_relative_old;
nav_msgs::Odometry turtle_twist;

//For the kalman filter
float Q_k = 0.001;
float fi_k = 1;
float R_k = 0.1;
int Ii = 1;
int H_k = 1;
float xp_k = 0;
float Pp_k = 1;






//void callback(const geometry_msgs::Twist::ConstPtr & msg){
void callback(const nav_msgs::Odometry::ConstPtr & msg){

  turtle_twist.twist.twist.linear.x = msg->twist.twist.linear.x;

}



void LidarDistance(const kugle_pkg::PersonArray::ConstPtr & dist){
  geometry_msgs::Point32 firstlidarpoint, distancediff;


    if(!dist->people.empty()){
     //firstlidarpoint.x = static_cast<float>(dist->people[0].pose.position.x);
     firstlidarpoint.x = dist->people[0].pose.position.x;
     int id = dist->people[0].id;
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
     vel_relative = vel + abs(turtle_twist.twist.twist.linear.x);
     float vel_diff = abs(vel) - abs(turtle_twist.twist.twist.linear.x);




     			float xm_k = fi_k * xp_k; //%+G_k * U_k - 1
     			float z_k = H_k * xm_k;
     			float Pm_k = fi_k * Pp_k * fi_k + Q_k;

     			float K_k = Pm_k * H_k*(1/(H_k*Pm_k*H_k + R_k));

     			xp_k = xm_k + K_k * (vel_relative - z_k);
     			Pp_k = (Ii - K_k * H_k)*Pm_k;

          cout << "Vel: \t " << vel_relative;
          cout << "\tKalman Adjusted Vel: \t " << xp_k;
          cout << "\tDist: \t " << firstlidarpoint.x << endl;


     //float vel_relative_diff = vel_relative - vel_relative_old;
     //vel_relative_old = vel_relative;

//     myfile << vel_relative << endl;

     //float acc = vel_relative_diff/timediff;

     //cout << "twist: \t" << turtle_twist.twist.twist.linear.x << "\t";
     //cout << "vel: \t " << vel << "\t";
     //cout << "vel_diff: \t " << vel_diff << "\t";
     //cout << "distancediff: \t " << distancediff;
     //cout << "time diff: \t " << timediff << "\t";
     //cout << "vel_relative: \t " << vel_relative << "\t";
     //cout << "Accleration: \t " << acc << endl;
     //cout << "ID: \t" << id << endl;
     //ROS_INFO("Vel: %f \t Vel_realative: %f \t twist: %f \n", vel, vel_relative, turtle_twist)
    }
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Subscriber vel_sub = n.subscribe<nav_msgs::Odometry>("odom", 1000, callback);
    myfile.open("/home/holly/data.txt");
    ros::Subscriber distance_sub = n.subscribe<kugle_pkg::PersonArray>("people_tracked", 1000, LidarDistance);
    ros::Publisher vel_pub = n.advertise<std_msgs::Float32>("veltrack", 100000);
    ros::Rate loop_rate(100);


  while(ros::ok()){
    std_msgs::Float32 msg;
//
//      ofstream myfile;



//
        msg.data=xp_k;
        vel_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

}
myfile.close();

ros::spin();
    return 0;

}


//
// #include "iostream"
// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "std_msgs/Float32.h"
// #include "std_msgs/Float64.h"
//
// #include "kugle_pkg/PersonArray.h"
//
//
// #include "geometry_msgs/Point.h"
// #include "geometry_msgs/Point32.h"
// #include "geometry_msgs/Twist.h"
// #include "nav_msgs/Odometry.h"
//
// using namespace std;
//
//
// double starttime = 0;
// double lasttime = 0;
// float vel_relative;
// float test;
// nav_msgs::Odometry turtle_twist;
//
//
// //void callback(const geometry_msgs::Twist::ConstPtr & msg){
// void callback(const nav_msgs::Odometry::ConstPtr & msg){
//
//   turtle_twist.twist.twist.linear.x = msg->twist.twist.linear.x;
//
// }
//
//
//
// void LidarDistance(const kugle_pkg::PersonArray::ConstPtr & dist){
//  geometry_msgs::Point32 firstlidarpoint, distancediff;
//
//  //firstlidarpoint.x = static_cast<float>(dist->people[0].pose.position.x);
//  firstlidarpoint.x = dist->people[0].pose.position.x;
//  //int id = dist->people[0].id;
//
//  double nowtime = ros::Time::now().toSec();
//
//  distancediff.x = firstlidarpoint.x - test;
//
//  //test = oldpos.x;
//
//  test = firstlidarpoint.x;
//
//  //ROS_INFO("time: %x", dist->header.stamp.to_sec());
//  //       double nowtime = ros::Time::now().toSec();
//  //double timediff = newtime - starttime;
//  double timediff = nowtime - lasttime;
//
//  //starttime = newtime;
//  lasttime = nowtime;
//
//  float vel = distancediff.x/timediff;
//  vel_relative = vel + abs(turtle_twist.twist.twist.linear.x);
//  float vel_diff = abs(vel) - abs(turtle_twist.twist.twist.linear.x);
//
//
//  //cout << "ID: \t" << id << endl;
//  //cout << "twist: \t" << turtle_twist.twist.twist.linear.x << "\t";
//  cout << "vel: \t " << vel << "\t";
//  cout << "vel_diff: \t " << vel_diff << "\t";
//  cout << "vel_relative: \t " << vel_relative << endl;
//  //ROS_INFO("Vel: %f \t Vel_realative: %f \t twist: %f \n", vel, vel_relative, turtle_twist)
//
// }
//
//
//
// int main(int argc, char **argv)
// {
//
//     ros::init(argc, argv, "test");
//     ros::NodeHandle n;
//
//     ros::Publisher vel_pub = n.advertise<std_msgs::Float32>("veltrack", 1);
//
//     ros::Subscriber vel_sub = n.subscribe<nav_msgs::Odometry>("odom", 100, callback);
//     ros::Subscriber distance_sub = n.subscribe<kugle_pkg::PersonArray>("people_tracked", 10, LidarDistance);
//
//     ros::Rate loop_rate(10);
//
//     while(ros::ok()){
//
//     //ros::Subscriber vel_sub = n.subscribe<geometry_msgs::Twist>("odom", 100, callback);
//         std_msgs::Float32 msg;
//
//
//
//         msg.data=vel_relative;
//         vel_pub.publish(msg);
//         ros::spinOnce();
//         loop_rate.sleep();
//
//
//
//       }
//       ros::spin();
//     return 0;
//
// }
