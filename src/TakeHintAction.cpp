/** @ package exp_ass2
* 
*  \file TakeHintAction.cpp
*  \brief This script implements an action that move the robot's arm to take the hints
*
*  \author Gianluca Piquet
*  \version 1.0
*  \date 01/09/2022
*  \details
*   
*  Subscribes to: <BR>
*    /visualization_marker
*    /oracle_hint
*    odom
*
*  Publishes to: <BR>
*    /new_hint
*
*  Services: <BR>
*    None
* 
*   Client Services: <BR>
*   None
*    
*
*  Action Services: <BR>
*   None
*
*  Description: <BR>
*  This program implements the action that allow the robot to move the arm and collect hints, the arm can be set in two different z position "up"-> 1.25
*  "down" -> 0.75. 
*/

#include <ros/ros.h>
#include <cstdlib>
#include "nav_msgs/Odometry.h"

#include "exp_ass2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <iostream>
#include <string>
#include <exp_ass2/ErlOracle.h>
#include <exp_ass2/NewHint.h>
#include <exp_ass2/NewHyp.h>
#include <exp_ass2/Oracle.h>

#include <visualization_msgs/MarkerArray.h>

int prev_pose = 0;
int first_motion = 0;
int move_up();
int move_down();
int move_arm();
ros::Publisher hint_pub;
int wp = 0;
float wp1_z;
float wp2_z;
float wp3_z;
float wp4_z;
int first_look_wp1 = 0;
int first_look_wp2 = 0;
int first_look_wp3 = 0;
int first_look_wp4 = 0;


namespace KCL_rosplan {

  TakeHintInterface::TakeHintInterface(ros::NodeHandle &nh) {
  // here the initialization
  }

/**
 * \brief: TakeHintInterface function
 * 
 * \param msg : the variables received from the plan dispatcher
 * 
 * \return true
 * 
 * This function implements the movement of the robot's arm, with the function move_arm() we can move the arm in different positions due to our necessity
 */

bool TakeHintInterface::concreteCallback (const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg){

ROS_INFO("Try to get hint");
  
  move_arm();
  
ROS_INFO("Action performed: completed!");
return true;

}
}

/**
 * \brief: take_hint function
 * 
 * \param: msg
 * 
 * \return 
 *   
 * This function handles the hints taken from the arm; if the hint has the correct format the function publlishes it on the topic /new_hint 
 */

void take_hint(const exp_ass2::ErlOracle::ConstPtr& msg){
  std::string i("ID");
  std::string points(":");
  std::string new_hint;
  std::string id = std::to_string(msg->ID);
  std::string key = msg->key.c_str();
  std::string value = msg->value.c_str();

  new_hint=i+id+points+key+points+value;
  //ROS_INFO("HINT: [%s]", new_hint.c_str());
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";
  std::cout<<"FOUND HINT: ";
  std::cout<<new_hint;
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";
  exp_ass2::NewHint hint_msg;
  hint_msg.hint=new_hint.c_str();
  if(id == "" || id =="-1"||key == "" || key =="-1" ||value == "" || value =="-1"){ 
  //ROS_INFO("WRONG HINT! (-1 or "" is there)");
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";
  std::cout<<"WRONG HINT! (-1 or "" is there)";
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";

  }
  else
  {
    hint_pub.publish(hint_msg);
  }
}

/**
 * \brief: move_arm function
 * 
 * \param: None
 * 
 * \return 
 *   
 * This function implements the control of arm movement. The particularity of this code is that it checks whether a waypoint 
 * has already been visited in fact, if the robot encounters this waypoint for the first time it will move the arm to either 
 * the "up" or "down" position, if on the other hand the robot has already encountered this wayoint it will have already recorded 
 * the position of the hint taken in the first visit and will change the arm position only if necessary
 */

int move_arm()
{
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";
  std::cout<<"MOVE THE ARM TO TAKE HINT.....";
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";
  if (wp == 1 && first_look_wp1 == 0){
    if (prev_pose == 1){
          std::cout << "Move to down position" << std::endl;
          // move the arm low
          move_down();
          // update with the current position
          prev_pose=0;
          // wait one second
          sleep(1);
          first_look_wp1 = 1;
      if (first_motion == 0){
            move_up();
            prev_pose =1;
            first_motion = 1;
          }
      }
    else if (prev_pose == 0)
        {
          std::cout << "Move to up position" << std::endl;
          // move the arm high
          move_up();
          // update with the current position
          prev_pose=1;
          //wait one second
          sleep(1);
          first_look_wp1 = 1;
          if (first_motion == 0){
            move_down();
            prev_pose= 0;
            first_motion = 1;
            }
        }
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";
  std::cout<< "Now, robot knows that in wp1 the z coordinate is:" << wp1_z <<std::endl;
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";
  }
  else if(wp == 2 && first_look_wp2 == 0){
    if (prev_pose == 1){
          std::cout << "Move to down position" << std::endl;
          // move the arm low
          move_down();
          // update with the current position
          prev_pose=0;
          // wait one second
          sleep(1);
          first_look_wp2 = 1;
      if (first_motion == 0){
            move_up();
            prev_pose =1;
            first_motion = 1;
          }
      }
    else if (prev_pose == 0)
        {
          std::cout << "Move to up position" << std::endl;
          // move the arm high
          move_up();
          // update with the current position
          prev_pose=1;
          //wait one second
          sleep(1);
          first_look_wp2 = 1;
          if (first_motion == 0){
            move_down();
            prev_pose= 0;
            first_motion = 1;
            }
        }
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";
  std::cout<< "Now, robot knows that in wp2 the z coordinate is:" << wp2_z <<std::endl;
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";
  }
  else if(wp == 3 && first_look_wp3 == 0){
    if (prev_pose == 1){
          std::cout << "Move to down position" << std::endl;
          // move the arm low
          move_down();
          // update with the current position
          prev_pose=0;
          // wait one second
          sleep(1);
          first_look_wp3 = 1;
      if (first_motion == 0){
            move_up();
            prev_pose =1;
            first_motion = 1;
          }
      }
    else if (prev_pose == 0)
        {
          std::cout << "Move to up position" << std::endl;
          // move the arm high
          move_up();
          // update with the current position
          prev_pose=1;
          //wait one second
          sleep(1);
          first_look_wp3 = 1;
          if (first_motion == 0){
            move_down();
            prev_pose= 0;
            first_motion = 1;
            }
        }
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";
  std::cout<< "Now, robot knows that in wp3 the z coordinate is:" << wp3_z <<std::endl;
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";
  }
  else if(wp == 4 && first_look_wp4 == 0){
    if (prev_pose == 1){
          std::cout << "Move to down position" << std::endl;
          // move the arm low
          move_down();
          // update with the current position
          prev_pose=0;
          // wait one second
          sleep(1);
          first_look_wp4 = 1;
      if (first_motion == 0){
            move_up();
            prev_pose =1;
            first_motion = 1;
          }
      }
    else if (prev_pose == 0)
        {
          std::cout << "Move to up position" << std::endl;
          // move the arm high
          move_up();
          // update with the current position
          prev_pose=1;
          //wait one second
          sleep(1);
          first_look_wp4 = 1;
          if (first_motion == 0){
            move_down();
            prev_pose= 0;
            first_motion = 1;
            }
        }
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";
  std::cout<< "Now, robot knows that in wp4 the z coordinate is:" << wp4_z <<std::endl;
  std::cout<<"\n";
  std::cout<<"-------------------------";
  std::cout<<"\n";
  }
  else if (wp == 1 && first_look_wp1 == 1 && wp1_z ==0.75){
    std::cout << "Move to down position" << std::endl;
    move_down();
  }
  else if (wp == 1 && first_look_wp1 == 1 && wp1_z ==1.25){
    std::cout << "Move to up position" << std::endl;
    move_up();
  }
  else if (wp == 2 && first_look_wp2 == 1 && wp2_z ==0.75){
    std::cout << "Move to down position" << std::endl;
    move_down();
  }
  else if (wp == 2 && first_look_wp2 == 1 && wp2_z ==1.25){
    std::cout << "Move to up position" << std::endl;
    move_up();
  }
  else if (wp == 3 && first_look_wp3 == 1 && wp3_z ==0.75){
    std::cout << "Move to down position" << std::endl;
    move_down();
  }
  else if (wp == 3 && first_look_wp3 == 1 && wp3_z ==1.25){
    std::cout << "Move to up position" << std::endl;
    move_up();
  }
  else if (wp == 4 && first_look_wp4 == 1 && wp4_z ==0.75){
    std::cout << "Move to down position" << std::endl;
    move_down();
  }
  else if (wp == 4 && first_look_wp4 == 1 && wp4_z ==1.25){
    std::cout << "Move to up position" << std::endl;
    move_up();
  }


return 0;
}

/**
 * \brief: move_up function
 * 
 * \param: None
 * 
 * \return 0
 *   
 * This function uses moveit to move the robot arm to the highest position which corresponds to the "up" position
 */

int move_up()
{
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
 // ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
  moveit::planning_interface::MoveGroupInterface group("arm");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  group.setNamedTarget("up");
  group.move();   

  return 0;
}

/**
 * \brief: move_down function
 * 
 * \param: None
 * 
 * \return 0
 *   
 * This function uses moveit to move the robot arm to the lowest position which corresponds to the "down" position
 */

int move_down()
{
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  // ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
  moveit::planning_interface::MoveGroupInterface group("arm");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  group.setNamedTarget("down");
  group.move();   

  return 0;
}

/**
 * \brief: marker_position function
 * 
 * \param: msg
 * 
 * \return 
 *   
 * This function is used to acquire the z position of markers 
 */

 void marker_position(const visualization_msgs::MarkerArray::ConstPtr& msg){
   wp1_z = msg->markers[1].pose.position.z;
   wp2_z = msg->markers[3].pose.position.z;
   wp3_z = msg->markers[0].pose.position.z;
   wp4_z = msg->markers[2].pose.position.z;
 }

/**
 * \brief: wp_pose function
 * 
 * \param: msg
 * 
 * \return 
 *   
 * This function is used to know which waypoint the robot is at, so if the robot has already visited that waypoint it can use 
 * the corresponding z to position the arm
 */

 void wp_pose(const nav_msgs::Odometry::ConstPtr& msg){
  float x = msg->pose.pose.position.x;
  float y = msg->pose.pose.position.y;
  if(x > 2.0 ){
    wp = 1;
  }
  else if(x < -2.0){
    wp = 3;
  }
  else if(y > 2.0){
    wp = 2;
  }
  else if( y < -2.0){
    wp = 4;
  }
 }

/**
 * \brief: main function
 * 
 * \param:
 * 
 * \return: 0 
 *   
 * This function initialize the node and all the sub/pub used 
 */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "takehintaction");
  ros::NodeHandle nh;
  ros::Subscriber oracle_sub = nh.subscribe("/oracle_hint",1000, take_hint);
  ros::Subscriber odom_sub = nh.subscribe("odom", 1000, wp_pose);
  hint_pub = nh.advertise<exp_ass2::NewHint>("/new_hint", 0);
  ros::Subscriber vis_sub = nh.subscribe( "/visualization_marker", 1000, marker_position);
  KCL_rosplan::TakeHintInterface my_aci(nh);
  my_aci.runActionInterface();
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);
  return 0;
}