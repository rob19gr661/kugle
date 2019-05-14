#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <fstream>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <simple_navigation_goals/PersonArray.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"

using namespace std;


float camdist;
double speed=0.02;
float goldenzone[2]={1, 1.2};	//inside the prefered distance
double MinMax[2]={0.1, 0.4};    	//min/max speed
double tooCFD[2]={0.6, 2};   		//too close/far min/max distance
int tooCF[2]={0, 0};      		//too close/far attempts

class SendGoal {

private:
ros::Subscriber distance_sub; 
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal goal;
dynamic_reconfigure::ReconfigureResponse srv_resp;

public:
SendGoal();
void speedup();
void speeddown();



private:
void lidardistance(const simple_navigation_goals::PersonArray::ConstPtr & dist);
void SendGoalposition();

};


SendGoal::SendGoal() :  MoveBaseClient("move_base", true){
  ros::NodeHandle n;
  distance_sub = n.subscribe<simple_navigation_goals::PersonArray>("people_tracked", 100, &SendGoal::lidardistance,this);



  while(!MoveBaseClient.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

	SendGoalposition();



	dynamic_reconfigure::ReconfigureResponse srv_resp;
   	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config conf;
	double_param.name = "max_vel_x";


	ros::Rate rosRate(1);
 while (ros::ok()) {




      //while(ros::ok() && current_distance<distance ) {
      	//double_param.value = speed;
 /*
      	  int count = 0;
      	  float is, mean;

          count++;
      	  is += camdist;
*/


      	    if(camdist<goldenzone[1] && camdist>goldenzone[0]){//1.5-1
      	      double_param.value = speed;

		conf.doubles.push_back(double_param);
		srv_req.config = conf;
		ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp);

      	    }else if(camdist < goldenzone[0]){
      	      if(camdist < tooCFD[0] && camdist > 0){ //0.6
      		        tooCF[0]++;
      	     }if(tooCF[0] > 200){
      		       speeddown();
      		       tooCF[0] = 200;
      		   }else {speedup();}

      	    }else if(camdist > goldenzone[1] && camdist > 1){
      	      if(camdist > tooCFD[1]){
      		        tooCF[1]++;
      	     }if(tooCF[1] > 2){
      		       speeddown();
      		       tooCF[1] = 2;
      	     }else {speeddown();}

           }/*else if (distance-current_distance < 1) {
      	       speeddown();
      	}mean = is/count;*/
    
			ROS_INFO("speed: %f cam: %f", speed, camdist);


			
		ros::spinOnce();
		rosRate.sleep();
}

}


void SendGoal::lidardistance(const simple_navigation_goals::PersonArray::ConstPtr & dist){


geometry_msgs::Point32 lidarpoint;

lidarpoint.x= static_cast<float>(dist->people[0].pose.position.x);
camdist = lidarpoint.x;

ROS_INFO("data: %f", lidarpoint.x);
}


void SendGoal::SendGoalposition(){


/***************Send Goal ******************/


	  goal.target_pose.header.frame_id = "base_link";
	  goal.target_pose.header.stamp = ros::Time::now();

	  goal.target_pose.pose.position.x = 7.0;

	  goal.target_pose.pose.orientation.w = 1.0;


	  ROS_INFO("Sending goal");
	  MoveBaseClient.sendGoal(goal);



/*****************Speed adjust******************/

}

void SendGoal::speedup(){
dynamic_reconfigure::ReconfigureResponse srv_resp;
dynamic_reconfigure::ReconfigureRequest srv_req;
dynamic_reconfigure::DoubleParameter double_param;
dynamic_reconfigure::Config conf;
double_param.name = "max_vel_x";


if (speed >= MinMax[1]){
  speed = MinMax[1];
  double_param.value = speed;
			
  }else {
    speed = speed + 0.1;
    double_param.value = speed;

  }
			conf.doubles.push_back(double_param);
			srv_req.config = conf;
			ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp);

}



void SendGoal::speeddown(){
dynamic_reconfigure::ReconfigureResponse srv_resp;
dynamic_reconfigure::ReconfigureRequest srv_req;
dynamic_reconfigure::DoubleParameter double_param;
dynamic_reconfigure::Config conf;
double_param.name = "max_vel_x";


  if (speed <= MinMax[0]){
    speed = MinMax[0];
    double_param.value = speed;
			
  }else {
    speed = speed - 0.1;
    double_param.value = speed;

	}
			conf.doubles.push_back(double_param);			
			srv_req.config = conf;
			ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  SendGoal now;



 return 0;
}

