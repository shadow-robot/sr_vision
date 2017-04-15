#include <iostream> 
 
#include <ros/ros.h> 
#include <actionlib/client/simple_action_client.h> 
#include <actionlib/client/terminal_state.h> 
#include <sr_recognizer/RecognizerAction.h> 
 
 
int main (int argc, char **argv) 
{ 
  ros::init(argc, argv, "test_Recognizer"); 
 
  // create the action client 
  // true causes the client to spin its own thread 
  actionlib::SimpleActionClient<sr_recognizer::RecognizerAction> ac("recognizer", true); 
 
  ROS_INFO("Waiting for action server to start."); 
  // wait for the action server to start 
  ac.waitForServer(); //will wait for infinite time 
 
  ROS_INFO("Action server started, sending goal."); 
  // send a goal to the action 
  sr_recognizer::RecognizerGoal goal; 
  goal.start = 1; 
  ac.sendGoal(goal); 
 
  //wait for the action to return 
  bool finished_before_timeout = ac.waitForResult(ros::Duration(600.0)); 
 
 
  if (finished_before_timeout) 
  { 
    actionlib::SimpleClientGoalState state = ac.getState(); 
 
    sr_recognizer::RecognizerResult result_ = *ac.getResult(); 
 
    ROS_INFO("Recognized Objects: %s", result_.ids[0].data.c_str()); 
 
    ROS_INFO("Action finished: %s",state.toString().c_str()); 
  } 
  else 
    ROS_INFO("Action did not finish before the time out."); 
 
  //exit 
  return 0; 
} 
