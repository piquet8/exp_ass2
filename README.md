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

All of the above behaviors are performed through the use of the ROSPlan module that coordiantes the actions that follow one another based on the success or failure of the previous action. To move the robot from one waypoint to another, so in the initial reconnaissance phase the [MoveAction](https://github.com/piquet8/exp_ass2/blob/main/src/MoveAction.cpp) is used, instead for the moves from the oracle (center of the arena) to the waypoints and vice versa the [MoveToOracleAction](https://github.com/piquet8/exp_ass2/blob/main/src/MoveToOracleAction.cpp) and the [MoveToHintAction](https://github.com/piquet8/exp_ass2/blob/main/src/MoveToHintAction.cpp) are utlized these nodes use the [go_to_point_action.py](https://github.com/piquet8/exp_ass2/blob/main/rt2_packages/motion_plan/scripts/go_to_point_action.py) node. When the robot arrives at a waypoint it uses the [TakeHintAction](https://github.com/piquet8/exp_ass2/blob/main/src/TakeHintAction.cpp) to move its arm using the functions provided by MoveIt; the positions were created on previously on MoveIt and differ based on the height reached, the 'up' position is used to reach clues placed at 1.25 while the 'down' position for those at 0.75. Each time the robot acquires a clue it is taken by the ontology which evaluates whether it has enough clues to formulate a complete hypothesis. When this hypothesis is ready the [HypReadyAction](https://github.com/piquet8/exp_ass2/blob/main/src/HypReadyAction.cpp) detects it and communicates that it has a ready hypothesis, the robot then moves to the center of the arena and the [CheckHypAction](https://github.com/piquet8/exp_ass2/blob/main/src/CheckHypAction.cpp) checks if the hypothesis found matches the winning one. If the hypothesis is the winning one the game ends otherwise the robot resumes the search.

# Project structure

## Nodes

## Messages

## Services

## Parameters

## UML
![UML](https://github.com/piquet8/exp_ass2/blob/main/media_exp2/UML.jpg)

## Rqt-graph

# How to launch

# Video and images of the running programme
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
## System's features

## System's limitations

## System's technical improvements

# Specific link to folders  
- Here you can find the documentation: [docs](https://github.com/piquet8/exp_ass1/tree/master/docs)
- Here you can find the media and diagram file: [media and diagram](https://github.com/piquet8/exp_ass2/media_exp2)

# Authors and contacts
AUTHOR: Gianluca Piquet

CONTACT: gianlucapiquet8@gmail.com 

