# Experimental Robotics Laboratory - Assignment 2
This project implements a simplified version of the game Cluedo, in which a robot moves around inside a square arena searching for hints to find the murderer, the weapon, and the location of the crime.

The hints are placed above the arena walls placed in the center of the four walls at corresponding waypoints and at different heights (0.75 and 1.25) that change each time the simulation is launched. To acquire the clues, the robot will have to reach for the different wayponints and use its arm to pick them up.

When the robot, after collecting a sufficient number of clues, finds a complete and consistent hypothesis, it reaches the central location and tells its hypothesis. If the hypothesis is the winning one, the game ends otherwise the robot resumes the search.

ARENA - from gazebo | HINTS - from rviz

![ARENA](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/arena_hints.jpeg)

## Expected Behaviour
The robot should:
- move itslef and the arm to takes the hints in different positions: (3,0) (-3,0) (0, 3) (0,-3) with two possible z positions (0.75 , 1.25)
- when a consistent hypothesis is deducible, it should go the center of the arena and express it in English
- if the hypothesis is wrong, it should keep exploring and find new hints
## Features of the project
The assignment requires:
- use **ROSPlan** to plan the behaviour of the robot choosing a set of actions and creating a domain and a problem
- create a model of the robot and implement the behaviour to move the arm using **MoveIt** that allows it to take hints

# How it works
In this project, the purpose of the robot is to reach certain points within the arena in order to acquire hints that can be used to find the correct hypothesis. Initially, the robot performs a 'reconnaissance' round in which it visits all 4 points of interest one after the other, moving its arm in the two possible positions so as to acquire the first clues but above all to identify the z-position relative to the point visited in this way it can used it in subsequent visits, thus avoiding wasting time by moving the arm when it is not needed. Once the reconnaissance round is over and the positions relative to the different points to be visited have thus been acquired, the robot will again visit the different points randomly until it has obtained enough hints to determine a complete hypothesis. When the hypothesis is ready the robot goes to the center of the arena and says its hypothesis, if the hypothesis is the correct one the game ends otherwise the robot resumes the search.

# Project structure

All of the above behaviors are performed through the use of the ROSPlan module that coordiantes the actions that follow one another based on the success or failure of the previous action. To move the robot from one waypoint to another, so in the initial reconnaissance phase the [MoveAction](https://github.com/piquet8/exp_ass2/blob/main/src/MoveAction.cpp) is used, instead for the moves from the oracle (center of the arena) to the waypoints and vice versa the [MoveToOracleAction](https://github.com/piquet8/exp_ass2/blob/main/src/MoveToOracleAction.cpp) and the [MoveToHintAction](https://github.com/piquet8/exp_ass2/blob/main/src/MoveToHintAction.cpp) are utlized these nodes use the [go_to_point_action.py](https://github.com/piquet8/exp_ass2/blob/main/rt2_packages/motion_plan/scripts/go_to_point_action.py) node. When the robot arrives at a waypoint it uses the [TakeHintAction](https://github.com/piquet8/exp_ass2/blob/main/src/TakeHintAction.cpp) to move its arm using the functions provided by MoveIt; the positions were created on previously on MoveIt and differ based on the height reached, the 'up' position is used to reach clues placed at 1.25 while the 'down' position for those at 0.75. Each time the robot acquires a clue it is taken by the ontology which evaluates whether it has enough clues to formulate a complete hypothesis. When this hypothesis is ready the [HypReadyAction](https://github.com/piquet8/exp_ass2/blob/main/src/HypReadyAction.cpp) detects it and communicates that it has a ready hypothesis, the robot then moves to the center of the arena and the [CheckHypAction](https://github.com/piquet8/exp_ass2/blob/main/src/CheckHypAction.cpp) checks if the hypothesis found matches the winning one. If the hypothesis is the winning one the game ends otherwise the robot resumes the search.

## Nodes
### Scripts folder
[hint_armor.py](https://github.com/piquet8/exp_ass2/blob/main/scripts/hint_armor.py): this node implements the cluedo ontology of the robot, it takes hints from the topic `/new_hint` and manages them to achieve a complete and consistent hypothesis. It's the same node of the previous assignment, you can find more information there [exp_ass1](https://github.com/piquet8/exp_ass1) (there the name of the node is hint.py)

[plan.py](https://github.com/piquet8/exp_ass2/blob/main/scripts/plan.py): this node implements the plan that allows the robot to achieve its goal by performing the different actions. It initializes all the servers needed for the generation, planning and execution of the plan and implements several update functions. Until all actions are successful (especially the "CheckHypAction" action) a re-planning takes place

### Src folder
[MoveAction.cpp](https://github.com/piquet8/exp_ass2/blob/main/src/MoveAction.cpp): this action it used in the reconnaissance phase, it forces the robot to visit all the waypoint one after another. To avoid the problem of the orientation of the robot repsect to the wall (it could be a problem for move arm and take hint) I impose to robot to pass through the center each time before reach the new waypoint. To move the robot this node uses the action `reaching_goal` provided by the [go_to_point_action.py](https://github.com/piquet8/exp_ass2/blob/main/rt2_packages/motion_plan/scripts/go_to_point_action.py) inside of the [rt2_packages](https://github.com/piquet8/exp_ass2/tree/main/rt2_packages/motion_plan)

[MoveToOracleAction.cpp](https://github.com/piquet8/exp_ass2/blob/main/src/MoveToOracleAction.cpp): this action it used to move the robot from a waypoint to the center of the arena. To move the robot this node uses the action `reaching_goal` provided by the [go_to_point_action.py](https://github.com/piquet8/exp_ass2/blob/main/rt2_packages/motion_plan/scripts/go_to_point_action.py)

[MoveToHintAction.cpp](https://github.com/piquet8/exp_ass2/blob/main/src/MoveToHintAction.cpp): this action it used to move the robot from the center of the arena to one of the waypoint. To move the robot this node uses the action `reaching_goal` provided by the [go_to_point_action.py](https://github.com/piquet8/exp_ass2/blob/main/rt2_packages/motion_plan/scripts/go_to_point_action.py) 

[TakeHintAction.cpp](https://github.com/piquet8/exp_ass2/blob/main/src/TakeHintAction.cpp): this node implements the hints acquisition process i.e. arm movement and saving the obtained clue. The *moveit* module is used to move the arm, through which the robot can move the arm in two positions depending on the position of the hint to be reached. In this node also it is checked whether the obtained hint has a suitable format and if so this is rewritten in a more readable format for the hint_armor node. This node subscribes to the `/oracle_hint` topic to get the hint once the marker is reached, to the `/odom` topic and `/visualization_marker` topic to know in which waypoint the robot is located and at what height the marker is placed at that location. Instead, publish in the `/new_hint` topic the message containing the clue found and processed into a unique string

[HypReadyAction.cpp](https://github.com/piquet8/exp_ass2/blob/main/src/HypReadyAction.cpp): this node is only concerned with checking whether a hypothesis is ready to be tested, to do this it subscribes to the `/hypothesis` topic and checks the value of *ready* if this comes out True then it sets a variable by which it is reported that a hypothesis has been found. Otherwise it is reported that no hypothesis has yet been found

[CheckHypAction.cpp](https://github.com/piquet8/exp_ass2/blob/main/src/CheckHypAction.cpp): this node has the function of checking whether the hypothesis stated by the robot is the correct one or not. To do this it subscribes to the topic `/hypothesis` and acquires the values related to the hypothesis; after declaring the hypothesis the robot compares the id of the examined hypothesis with the id of the winning hypothesis which is obtained with a client to the service `/oracle_solution`. In case of a positive outcome the game ends with a win message, otherwise the oracle notifies that the hypothesis is incorrect and the robot resumes the search

[simulation_node.cpp](https://github.com/piquet8/exp_ass2/blob/main/src/simulation_node.cpp): this node was provided by the professor in the [erl2](https://github.com/CarmineD8/erl2) package, it implements the simulation and in particular provides the positions of the markers, the hints randomly generated and published on the topic `/oracle_hint` and the id of the winning hypothesis also randomly generated and provided by the service `/oracle_solution`

*If you need you can find a more accurate description of the nodes used in the documentation* [docs](https://github.com/piquet8/exp_ass2/tree/main/docs)

## Messages
[ErlOracle.msg](https://github.com/piquet8/exp_ass2/blob/main/msg/ErlOracle.msg): this message contains the information of the hint provided by the *simulation_node* on the `/oracle_hint` topic

[NewHint.msg](https://github.com/piquet8/exp_ass2/blob/main/msg/NewHint.msg): this message contains the hint taken by the robot redesigned to be more readable in only one string, it is publish on the `/new_hint` topic

[NewHyp.msg](https://github.com/piquet8/exp_ass2/blob/main/msg/NewHyp.msg): this message contains the hypothesis complete, it is publish on the `/hypothesis` topic

## Services
[Oracle.srv](https://github.com/piquet8/exp_ass2/blob/main/srv/Oracle.srv): this service is provedid by the *simulation_node* and contains the winner ID value, it is callable from the  `/oracle_solution`

## Pddl
[domain.pddl](https://github.com/piquet8/exp_ass2/blob/main/pddl/domain.pddl): in the domain we define the universal aspects of the problem; it implements the requirements, the types, the predicates and the action used to achieve the solution of the problem. Some information about the predicates and the action are written inside the file

[problem.pddl](https://github.com/piquet8/exp_ass2/blob/main/pddl/problem.pddl): in the problem we solidifie the domain expression by define exactly what objects exist, and what is true about them and then finally what the end goal is. Inside the file we can se that the objects are the waypoints and the home, instead the initial condition are that robot is in home position and no
hints are found yet and also no hypothesis. In the end the goal is that we checked the hypothesis as true.

The logic of operation follows a plan whereby initially no waypoints are visited and no hypotheses are found. Then the robot will visit all waypoints, once this is done it is checked to see if there is a hypothesis ready to be tested, if not then randomly a wapoint will be marked as unvisited and the robot will go for its new hint. This is done in a loop until a complete and consistent hypothesis is found. When this happens ready_hp also results true and then it is checked to see if the hypothesis found is the winning one; if it is the winning one check_hp also results true and the game ends otherwise ready_hp goes back to being set to false and the robot resumes searching according to the previous logic

## Urdf model
[robot.urdf](https://github.com/piquet8/exp_ass2/blob/main/urdf/robot8.urdf): this file contains the model of the robot

## UML
![UML](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/UML.jpg)

## Rqt-graph
![Rqt-graph](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/rqt_graph.png)

# How to run the program
## Requirements
For run this project you will need the following packages:
- **armor server** you can find the installation procedure here: https://github.com/EmaroLab/armor

ARMOR is the management system we use for our ontology in a ROS system

- **ROSPlan** you can find it here: https://github.com/KCL-Planning/ROSPlan

ROSPlan framework provides a generic method for task planning in a ROS system

- **rt2_packages** you can find it here: https://github.com/CarmineD8/rt2_packages

This package provides within it the *motion_plan* package that implements the *go_to_point* nodes used for robot motion

*SUGGESTION: you should find these packages already installed in this docker workspace:* https://hub.docker.com/r/carms84/exproblab


## How to launch

1. Firstly, open the terminal, go to your workspace and in the src folder run:
```
git clone https://github.com/piquet8/exp_ass2.git
```
After that you need to build the package in the workspace: *catkin_make --only-pkg-with-deps exp_ass2*

2. Then to launch the simulation environment and relative nodes open a new shell tab and run the command:
```
roslaunch exp_ass2 final.launch
```
*SUGGESTION: For a cleaner terminal with no warning messages I suggest you to try to use this command to launch:*
```
roslaunch exp_ass2 final.launch 2>/dev/null
```
3. Finally to starts the game open a new shell tab and run the command:
```
rosrun exp_ass2 plan.py
```


# Video and images of the running program
- [VIDEO_DEMO](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/demo_exp2.mp4)

Here is possible see a small demo video that shows the robot in action, in particular in this video is possible to see the 'reconnaissance' phase where the robot visits all the waypoint for the first time and check both the z possible solutions, then it starts the random search but now when the robot is in the waypoint position it moves the arm in different position only if it is necessary. It's possible see in the Rviz interface that the markers are reached form the arm and in the terminal is possibile to see the infomration about the execution, the hints and the hypothesis states. Due to the slowness of the simulation (also due to the weak features of my pc) the video has been sped up and shows only a part of the whole simulation, so I have added below some images showing the most important messages displayed at the terminal during the simulation

- In this image you can see the sequence of commands executed by ROSPlan to generate and painify an execution plan for the robot

![ROSPlan](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/rosplan_screen.png)

- Instead here it's possible to see the re-plan phase, the hypothesis is not ready but the robot has visited all the waypoints so we put randomly one of the waypoint as not-visited and the robot starts again the search

![ROSPlan](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/replan_screen.png)

- Here we can see the message that robot print when it visits for the first time a waypoint:

![z stored](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/store_z.png)

![z2 stored](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/store_z2.png)

- In the next images we see the message about the hint taken, the robot move the arm and takes the hint. Sometime the hint could be in the wrong format and the robot in this case reports it and discards it

![move_arm](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/move_arm.png)

![hint](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/found_hint.png)

![hint_wrong](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/wronghint_screen.png)

- In the last two images we see the screen about the wrong hypothesis and the winner hypothesis

![wrong](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/wrong_screen.png)

![winner](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/winning_screen.png)


# Working hypothesis and environment
For this project, I started from the assumptions of the previous assignment ([exp_ass1](https://github.com/piquet8/exp_ass1)), specifically on the behaviors needed to the robot to achieve the goal, which are: movement of the robot to search and collect clues, processing of the clues in the ontology and checking for ready hypotheses, and finally testing the hypotheses found. Accordingly, I developed for the first aspect 4 actions involving the movement of the robot between waypoints, between waypoints and home, between home and waypoints and the movement of the arm to collect the clue once the waypoint is reached. Next I developed the action that checks whether the ontology found a complete and consistent hypothesis and finally an action that checks whether hypothesis found is winning or wrong. Having never used ROSPlan before, I preferred to use a large number of actions to make sure that the plan would actually lead to the achievement of the desired goal; it was probably possible to reduce the number of actions by nesting some of them

## System's features
The main feature of this system is the use of ROSPlan instead of the finite state machine. The use of this tool makes it possible to have a system capable of adapting to the robot's behavior during the simulation, in the sense that it allows the robot to expect failures of certain actions and to reschedule the plan to still achieve the final goal. Furthermore, it is possible, using domain-defined aspects, to create different problems with different purposes without having to put hands on the code and without having to change the implemented actions

## System's limitations
The limitations of the project, although not preventing achievement of the goal and execution, are mainly three:
- the simulation is very slow (in part probably due to the low performance of my pc) and the robot takes a long time to move and consequently also to get a complete guess (this also depends on the number of clues with wrong format that are found)
- the robot is forced to go through the center of the arena moving from waypoint to waypoint because this is the only way I ensure that the robot arrives frontally to the wall and consequently to the marker placed at that waypoint
- the way the plan is designed, after the robot has visited all the waypoints initially, in subsequent searches only one waypoint at a time (chosen at random) is changed to unvisited, as a result the robot every time it finds a new clue is forced to go back to the center of the arena and check if a guess is ready, this wastes a lot of time. As another rationale I had implemented a behavior of visiting all four waypoints every round, this technique however produced a problem, in fact if the robot at the first waypoint completed the correct hypothesis, it still continued the round and it happened that for example at the third waypoint it completed a new hypothesis which was then 'overwritten' to the previous one. This sometimes led to the failure of the project, as the correct hypothesis could be overwritten and thus would no longer occur

## System's technical improvements
The possible improvements that should be implemented are:
- modify the go_to_point_action file in order to handle the orientation with which the robot arrives when it finishes the movement between waypoints in this way I could avoid having to go through the center of the arena speeding up the process of collecting clues, another alternative could be instead to handle differently the movement of the arm that for the moment uses two predefined positions, choosing a solution that moves the arm to a certain desired point in space regardless of the position of the base of our robot
- modify the logic of the plan so that the solution requires fewer steps between different actions and a different sequence of execution; in addition, the problem highlighted above could be solved by finding a solution that would allow interroping (perhaps with a service or using the delete action method) the round of waypoints when a hypothesis is complete thus avoiding overwriting, and in this way all waypoints could be marked as unvisited each round so that the robot could search for more clues in each round
- an additional idea that might speed up clue collection, although I don't know if it goes against the assignment rules might be that if the robot finds a clue of the wrong format, the robot turns on itself 360 degrees and picks up a new clue from the same waypoint, that way there would never be 'wasted' visits to a waypoint

# Specific link to folders  
- Here you can find the documentation: [docs](https://github.com/piquet8/exp_ass2/tree/main/docs)
- Here you can find the media and diagram file: [media and diagram](https://github.com/piquet8/exp_ass2/tree/main/media_exp2)

# Authors and contacts
AUTHOR: Gianluca Piquet

CONTACT: gianlucapiquet8@gmail.com 

