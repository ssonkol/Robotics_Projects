#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "move_base_msgs/MoveBaseAction.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "robotnav");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("/move_base", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  
  move_base_msgs::MoveBaseGoal goal;
    
  
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose.orientation.w = 0.67;
  goal.target_pose.pose.orientation.z = -0.74;
  
  goal.target_pose.pose.position.x = 10.6;
  goal.target_pose.pose.position.y = 7.10;
  
  ac.sendGoal(goal);
    
  bool finished_before_timeout = false;
  
  while (!(finished_before_timeout = ac.waitForResult(ros::Duration(5.0)))) {

    ROS_INFO("still waiting...");
  }
  
  actionlib::SimpleClientGoalState state = ac.getState();

  ROS_INFO("Action finished: %s",state.toString().c_str());


  //exit
  return 0;
}