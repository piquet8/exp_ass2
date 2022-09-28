#! /usr/bin/env python

## @package exp_ass2
#
#  \file plan.py
#  \brief this file implements the calls to the planning server
#
#  \author Gianluca Piquet
#  \version 1.0
#  \date 01/09/2022
#  \details
#  
#  Subscribes to: <BR>
#	 
#  Publishes to: <BR>
#	 
#  Services: <BR>
#    
#  Action Services: <BR>
#
#  Client Services: <BR>
#  /rosplan_problem_interface/problem_generation_server
#  /rosplan_planner_interface/planning_server
#  /rosplan_parsing_interface/parse_plan
#  /rosplan_plan_dispatcher/dispatch_plan
#  /rosplan_knowledge_base/update
#    
#
#  Description: <BR>
#  This script implements the plan that allows the robot to achieve its goal by performing the different actions. Until all actions 
#  are successful ( especially the "CheckHypAction" action) a re-planning takes place. Within the script we find, for the previous 
#  purpose, several update functions.


import rospy
import time
import actionlib
import exp_ass2.msg
import random
from rosplan_knowledge_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse, PlanningService, PlanningServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue

x = 4
##
#	\brief Initalize plan servers
#	\param : 
#	\return : None
# 	
#	This function initializes all the servers needed for the generation, planning and execution of the plan

def init_plan():
    global problem_generation, planning, parsing, dispatch, update
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    problem_generation = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server',Empty)
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    planning = rospy.ServiceProxy('/rosplan_planner_interface/planning_server',Empty)
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    parsing = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan',Empty)
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
    dispatch = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan',DispatchService)	
    rospy.wait_for_service('/rosplan_knowledge_base/update')
    update= rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    print('Ready to play the game!')

##
#   \brief This function implements the random wayponints choice
#   \param :
#   \return : None
#   
#   This function is used to implement the random choice of waypoint to visit. A random number between 1 and 4 is generated, and the 
#   corresponding waypoint found is updated to unvisited by calling the update_waypoint() function, which changes the value of new_hint 
#   from True to False. Finally, the variable ready_hp is also changed to False by the update_ready_hyp() function. 

def random_position():
    global x
    n=random.randint(1,4)
    while (n==x):
        n=random.randint(1,4)
    x=n
    if n==1:
        update_waypoint('wp1')
    elif n==2: 
        update_waypoint('wp2')
    elif n==3:
        update_waypoint('wp3')
    elif n==4:
        update_waypoint('wp4')
    update_ready_hyp()

##
#	\brief Waypoint update
#	\param name: waypoint 
#	\return : None
# 	
#	This function implement the calls to the knowledge base server to update the predicate (new_hint) for one waypoint, in this way the waypoint becomes
#   not visited again

def update_waypoint(name):
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'new_hint'
    req.knowledge.values.append(diagnostic_msgs.msg.KeyValue('waypoint', name))	
    result=update(req)
    
##
#	\brief Ready hypothesis update
#	\param :
#	\return : None
# 	
#   This function implement the calls to the knowledge base server to update the predicate (ready_hp), in this way the robot continues its research

def update_ready_hyp():
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'ready_hp'
    result=update(req)	

##
#	\brief Main function
#	\param :
#	\return : None
# 	
#   This function initializes all the servers needed to implement the plan, also, starts re-planning until the goal and success conditions are true

def main():
    global pub_, active_, act_s
    rospy.init_node('plan')
    init_plan()
    success=False
    goal=False

    while ( success== False or goal == False):
        
        resp_pg=problem_generation()
        time.sleep(1)
        resp_pl=planning()
        time.sleep(1)
        resp_pars=parsing()
        time.sleep(1)
        resp_dis=dispatch()

        success= resp_dis.success
        goal= resp_dis.goal_achieved
        
        random_position()
        time.sleep(1)

if __name__ == '__main__':
    main()
