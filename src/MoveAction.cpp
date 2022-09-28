/** @ package exp_ass2
* 
*  \file MoveAction.cpp
*  \brief This script implements an action that move the robot from one waypoint to another one
*
*  \author Gianluca Piquet
*  \version 1.0
*  \date 01/09/2022
*  \details
*   
*  Subscribes to: <BR>
*	None 
*
*  Publishes to: <BR>
*	 None
*
*  Services: <BR>
*    None
* 
*   Client Services: <BR>
*   None
*    
*
*  Action Services: <BR>
*    reaching_goal
*
*  Description: <BR>
*  This program implements the action that allow the robot to move from a waypoint to another one, to do that it used the "reaching goal" of the go_to_point_action 
*  node in the motion_plan package. To avoid the problem of the orientation of the robot at the end of the goal I imposed that any time after the robot reach a 
*  position it pass through the center of the arena, in this way it's sure thata the robot will arrive to the waypoint with frontal orientation repspect to the wall
*/

#include "exp_ass2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>

namespace KCL_rosplan {

	MoveInterface::MoveInterface(ros::NodeHandle &nh) {
	// here the initialization
	}

/**
 * \brief: MoveInterface function
 * 
 * \param msg : the variables received from the plan dispatcher
 * 
 * \return true
 * 
 * This function implements the movement of the robot between the waypoints when the planner dispatches the action goto_waypoint. It takes the waypoints name and 
 * sends the request to the go_to_point server to reach the new position. When the robot reach the position the action is marked as performed.
 */	

	bool MoveInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		ros::NodeHandle nh("~");
			// here the implementation of the action
		std::cout<<"\n";
 		std::cout<<"-------------------------";
        std::cout<<"\n";
		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		std::cout<<"\n";
 		std::cout<<"-------------------------";
        std::cout<<"\n";
		actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("reaching_goal", true);
		motion_plan::PlanningGoal goal;
		ac.waitForServer();
		//impose that robot has to pass trhough the center 
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 1.0;

		ac.sendGoal(goal);
		ac.waitForResult();
		//set the target that has to be raeched depending on the waypoint select
		if(msg->parameters[1].value == "wp1"){
		goal.target_pose.pose.position.x = 2.5;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "wp2"){
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = 2.5;
		goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "wp3"){
		goal.target_pose.pose.position.x = -2.5;
		goal.target_pose.pose.position.y = 0.0;
		goal.target_pose.pose.orientation.w = 1.0;
		}
		else if (msg->parameters[1].value == "wp4"){
		goal.target_pose.pose.position.x = 0.0;
		goal.target_pose.pose.position.y = -2.5;
		goal.target_pose.pose.orientation.w = 1.0;
		}	
		
		ac.sendGoal(goal);
		ac.waitForResult();
		
		
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}

/**
 * \brief: Main moveaction function
 *
 * \param msg : 
 * 
 * \return 0
 * 
 * The main function initializes the node and the interface to associate the action of the ros-plan
 */

	int main(int argc, char **argv) {
		ros::init(argc, argv, "moveaction", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		KCL_rosplan::MoveInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}




