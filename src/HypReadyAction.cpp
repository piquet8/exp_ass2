/** @ package exp_ass2
* 
*  \file HypReadyAction.cpp
*  \brief This script implements an action that check if a hypothesis is ready
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
*   None
* 
*   Client Services: <BR>
*   None
*    
*
*  Action Services: <BR>
*   None
*
*  Description: <BR>
*  this function checks if a hypothesis is ready, to do this I check if the value of ready within the acquired hypothesis is set to true
* 
*/

#include "exp_ass2/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>

#include <string>
#include <exp_ass2/ErlOracle.h>
#include <exp_ass2/NewHint.h>
#include <exp_ass2/NewHyp.h>
#include <exp_ass2/Oracle.h>

int x = 0;

namespace KCL_rosplan {

	HypReadyInterface::HypReadyInterface(ros::NodeHandle &nh) {
	// here the initialization
	}

/**
 * \brief: HypReadyInterface function
 * 
 * \param msg : the variables received from the plan dispatcher
 * 
 * \return true
 * 
 * This function tells a new hypothesis is founded or not
 */

	bool HypReadyInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
				
        if (x == 1){
        	ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
			std::cout<<"\n";
			std::cout<<"-------------------------";
			std::cout<<"\n";
			std::cout<<"I FOUND NEW HYPOTHESIS!";
			std::cout<<"\n";
			std::cout<<"-------------------------";
			std::cout<<"\n";
			x=0;
			return true;
		}
        else{
        	ROS_INFO("Action (%s) performed: wrong!", msg->name.c_str());
        	std::cout<<"\n";
        	std::cout<<"-------------------------";
        	std::cout<<"\n";
			std::cout<<"I DIDN'T FIND NEW HYPOTHESIS YET!";
			std::cout<<"\n";
			std::cout<<"-------------------------";
			std::cout<<"\n";
			return false;
        }
	}
}

/**
 * \brief: new_hyp function
 * 
 * \param: msg
 * 
 * \return 
 * 
 * This function checks if the value of the ready hypothesis is true and if so sets a variable x to 1 so that the 
 * function above knows that a new hypothesis has been found
 */

void _new_hyp(const exp_ass2::NewHyp::ConstPtr& msg){
	if (msg->ready == true){
		x = 1; 
	}else{
		x = 0;
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
		ros::init(argc, argv, "hypreadyaction", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		ros::Subscriber hyp_sub = nh.subscribe("/hypothesis", 1000, _new_hyp);
		KCL_rosplan::HypReadyInterface my_aci(nh);
		my_aci.runActionInterface();

		return 0;
	}

