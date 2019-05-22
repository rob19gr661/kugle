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


float camdist, mean,is;
int j;
double speed=0;
float goldenzone[2]={1, 1.4};	//inside the prefered distance
float preferredVG[3] = {138,0,0}; // dynamic values (velocity, gold 0, gold 1)
int state = 1, prox=0;
float velocity;


class SendGoal {

private:
ros::Subscriber distance_sub;
ros::Subscriber veltrack_sub;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal goal;
dynamic_reconfigure::ReconfigureResponse srv_resp;

public:
SendGoal();
double PID(double, double);



private:
void lidardistance(const simple_navigation_goals::PersonArray::ConstPtr & dist);
void lidarvelocity(const std_msgs::Float32::ConstPtr & vel);
void SendGoalposition();

};


SendGoal::SendGoal() :  MoveBaseClient("move_base", true){
  ros::NodeHandle n;
  distance_sub = n.subscribe<simple_navigation_goals::PersonArray>("people_tracked", 100, &SendGoal::lidardistance,this);
veltrack_sub = n.subscribe<std_msgs::Float32>("veltrack", 100, &SendGoal::lidarvelocity,this);


  while(!MoveBaseClient.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

	SendGoalposition();



	dynamic_reconfigure::ReconfigureResponse srv_resp;
   	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config conf;
	double_param.name = "max_vel_x";



	ros::Rate rosRate(0.1);
 while (ros::ok()) {

   double inc = PID(preferredVG[0], speed);
   speed += inc;
   double turt = speed / 1000;

   switch(state){
     case 1:{
       preferredVG[1] = preferredVG[0] - (preferredVG[0]*0.05);

       if (speed > preferredVG[1]){
       state++;
     }
     }break;

     case 2:{
       preferredVG[1] = goldenzone[0];
       preferredVG[2] = goldenzone[1];

       preferredVG[0]=velocity;

       if(camdist < preferredVG[1] || camdist > preferredVG[2]){
         prox++;
         state++;
       }
     }break;

     case 3:{
       preferredVG[0]=velocity;
       is+=fabs(velocity);
         j++;
         mean=is/j;
         if(camdist > preferredVG[1] && camdist < preferredVG[2]){
           if(prox >= 3){
               state=4;
           }else{state=2;}
         }

     }break;

     case 4:{
         preferredVG[1] = goldenzone[0];
         preferredVG[2] = goldenzone[1];

         preferredVG[0]=mean*1000;
     }break;
   }


   printf("state: %d\tvel: %f\tcamdist: %f\tmean: %f\n",state,turt,camdist,mean);
     double_param.value = turt;                     //sender til botten
     conf.doubles.push_back(double_param);
     srv_req.config = conf;
     ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp);


		ros::spinOnce();
		rosRate.sleep();

}
}


void SendGoal::lidardistance(const simple_navigation_goals::PersonArray::ConstPtr & dist){


geometry_msgs::Point32 lidarpoint;

lidarpoint.x= static_cast<float>(dist->people[0].pose.position.x);
camdist = fabs(lidarpoint.x);

//ROS_INFO("data: %f", lidarpoint.x);
}

void SendGoal::lidarvelocity(const std_msgs::Float32::ConstPtr & vel){


velocity = vel->data;

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

double SendGoal::PID(double setpoint, double pv){

  double dt = 0.001;        //-  loop interpreferredVBgold[0] time
  double Kp = 0.04;         //-  proportional gain
  double Ki = 0.5;         //-  derivative gain
  double Kd = 0.05;         //-  derivative gain

  double pre_error;
  double integral;

  // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = Kp * error;

    integral += error *dt;
    double Iout =Ki*integral;

    // Derivative term
    double derivative = (error - pre_error) / dt;
    double Dout = Kd * derivative;

  //printf("%d = (%d - %d)/%d",derivative, error, pre_error, dt);
    // Calculate total output
    double outputPD = Pout + Iout;

    // Save error to previous error
    pre_error = error;

    return outputPD;
}







int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  SendGoal now;



 return 0;
}
