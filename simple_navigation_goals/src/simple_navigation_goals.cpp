#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <fstream>
#include <stdlib.h>

using namespace std;


float camdist; 
float max_person_velocity = 0.3;
float max_close_distance = 1.0;
float max_far_distance = 2.0;


class SendGoal {

private:
ros::Subscriber distance_sub; //, getgoal_sub;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal goal;
bool isGuidingActive = false;
dynamic_reconfigure::ReconfigureResponse srv_resp;

public:
SendGoal();


private:
void cameradistance(const std_msgs::Float32::ConstPtr & dist);
void SendGoalposition();
};


SendGoal::SendGoal() :  MoveBaseClient("move_base", true){
  ros::NodeHandle n;
  distance_sub = n.subscribe("CamDist", 100, &SendGoal::cameradistance, this);




  //wait for the action server to come up
  while(!MoveBaseClient.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

SendGoalposition();

ros::Rate loop_rate(10);

	while(ros::ok()){
        ros::spinOnce();
	loop_rate.sleep();
}



}


void SendGoal::cameradistance(const std_msgs::Float32::ConstPtr & dist){
  camdist = dist->data;
ROS_INFO("cam: %f", camdist);
}


void SendGoal::SendGoalposition(){


/***************Send Goal *****************/


	  goal.target_pose.header.frame_id = "base_link";
	  goal.target_pose.header.stamp = ros::Time::now();

	  goal.target_pose.pose.position.x = 7.0;

	  goal.target_pose.pose.orientation.w = 1.0;


	  ROS_INFO("Sending goal");
	  MoveBaseClient.sendGoal(goal);

	if(MoveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	 ROS_INFO("Hooray, the base moved 1 meter forward");
	else
	 ROS_INFO("The base failed to move forward 1 meter for some reason");


/*****************Speed adjust******************/

		   	dynamic_reconfigure::ReconfigureRequest srv_req;
    			dynamic_reconfigure::DoubleParameter double_param;
    			dynamic_reconfigure::Config conf; 
			double_param.name = "max_vel_x";
			conf.doubles.push_back(double_param);


		if(camdist < max_close_distance && max_person_velocity<0.4){

			max_person_velocity = max_person_velocity + 0.1;

			double_param.value = max_person_velocity;



			}
 		if(camdist > max_far_distance && max_person_velocity>0.1){
		
			max_person_velocity = max_person_velocity - 0.1;


	
			double_param.value = max_person_velocity;

			}

			srv_req.config = conf;
			ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp);



}



int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
SendGoal now;

ros::spin();
 return 0;
}


