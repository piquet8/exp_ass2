/** @ package exp_ass2
* 
*  \file CheckHypAction.cpp
*  \brief This script implements an action that check if a hypothesis is the winner one
*
*  \author Gianluca Piquet
*  \version 1.0
*  \date 01/09/2022
*  \details
*   
*  Subscribes to: <BR>
*    None
*
*  Publishes to: <BR>
*    /hypothesis
*
*  Services: <BR>
*    /oracle_solution
* 
*   Client Services: <BR>
*   None
*    
*
*  Action Services: <BR>
*   None
*
*  Description: <BR>
*  This program acquires a hypothesis by subscribing to the topic /hypothesis and compares its ID with the winning one
* 
*/

#include "exp_ass2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>

#include <iostream>
#include <string>
#include <exp_ass2/ErlOracle.h>
#include <exp_ass2/NewHint.h>
#include <exp_ass2/NewHyp.h>
#include <exp_ass2/Oracle.h>

bool winner = false;
std::string who = "";
std::string what = "";
std::string where = "";

namespace KCL_rosplan {

  CheckHypInterface::CheckHypInterface(ros::NodeHandle &nh) {
  // here the initialization
  }

/**
 * \brief: CheckHypInterface function
 * 
 * \param msg : the variables received from the plan dispatcher
 * 
 * \return true
 * 
 * This function tells what is the hypothesis obtained by the robot and tells if it is the winning or wrong one
 */

  bool CheckHypInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
  std::cout<<"\n";
  std::cout << "---------------";
  std::cout<<"\n";
  std::cout<< "The hypothesis is: ";
  std::cout << who;
  std::cout<< " with the ";
  std::cout << what;
  std::cout << " in the ";
  std::cout << where;
  std::cout<<"\n";
  std::cout << "---------------";
  std::cout<<"\n";

  if (winner == true){
    ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
    std::cout<<"\n";    
    std::cout<<"-------------------------";
    std::cout<<"\n";
    std::cout<<" RIGHT HYPOTHESIS!!!!";
    std::cout<<"\n";
    std::cout<<"-------------------------";
    std::cout<<"\n";
    return true;
  }
  else{
    ROS_INFO("Action (%s) performed: wrong!", msg->name.c_str());
    std::cout<<"\n";
    std::cout<<"-------------------------";
    std::cout<<"\n";
    std::cout<<" WRONG HYPOTHESIS!";
    std::cout<<"\n";
    std::cout<<"-------------------------";
    std::cout<<"\n";
    return false;
  }
}
}

/**
 * \brief: new_hypothesis function
 * 
 * \param msg : the variables received from the plan dispatcher
 * 
 * \return 
 * 
 * this function via the /oracle_solution service acquires the hypothesis values and assigns them to the respective variables, also checks 
 * if the hypothesis is the correct one and sets winner to true or false
 */

void new_hypothesis(const exp_ass2::NewHyp::ConstPtr& msg){
  ros::NodeHandle nh("~");
  ros::ServiceClient win_hyp = nh.serviceClient<exp_ass2::Oracle>("/oracle_solution");
  std::string w_id = msg->ID.c_str();
 // ROS_INFO("ID: [%s]", msg->ID.c_str());
 // ROS_INFO("WHO: [%s]", msg->who.c_str());
 // ROS_INFO("WHAT: [%s]", msg->what.c_str());
 // ROS_INFO("WHERE: [%s]", msg->where.c_str());
 
  who = msg->who;
  what = msg->what;
  where = msg->where;


  std::string i("ID");
  exp_ass2::Oracle srv;
  win_hyp.call(srv);
  std::string win = std::to_string(srv.response.ID);
  std::string win_hp =i+win;
  std::cout<<win_hp;
  std::cout<<w_id;
  std::cout<<"\n";
  if (win_hp == w_id){
    winner = true;
  }
  else{
    winner = false;
  }
}
  
/**
 * \brief: main function
 * 
 * \param 
 * 
 * \return 0 
 * 
 * This function initialize the node and all the sub/pub used 
 */

int main(int argc, char **argv) {
    ros::init(argc, argv, "checkhypaction", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    ros::Subscriber hyp_sub = nh.subscribe("/hypothesis", 1000, new_hypothesis);
    KCL_rosplan::CheckHypInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
  }