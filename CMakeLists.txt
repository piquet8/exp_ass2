cmake_minimum_required(VERSION 3.1.3)
project(exp_ass2)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  visualization_msgs
  gazebo_msgs
  message_generation
  moveit_ros_planning_interface
  rosplan_planning_system
  urdf
  sensor_msgs
  tf
  geometry_msgs
  moveit_commander
  moveit_msgs
  actionlib
)

 add_message_files(
   FILES
   ErlOracle.msg
   NewHint.msg
   NewHyp.msg
 )
 
 add_service_files(
   FILES
   Oracle.srv
 )
 
  generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
   geometry_msgs
 )
 
catkin_package()

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
 add_executable(simulation_node src/simulation_node.cpp)
 add_dependencies(simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(simulation_node ${catkin_LIBRARIES})
 
 add_executable(MoveToHintAction src/MoveToHintAction.cpp)
 add_dependencies(MoveToHintAction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(MoveToHintAction ${catkin_LIBRARIES})
 
 add_executable(MoveToOracleAction src/MoveToOracleAction.cpp)
 add_dependencies(MoveToOracleAction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(MoveToOracleAction ${catkin_LIBRARIES})
 
 add_executable(MoveAction src/MoveAction.cpp)
 add_dependencies(MoveAction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(MoveAction ${catkin_LIBRARIES})
 
 add_executable(TakeHintAction src/TakeHintAction.cpp)
 add_dependencies(TakeHintAction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(TakeHintAction ${catkin_LIBRARIES})
 
  add_executable(CheckHypAction src/CheckHypAction.cpp)
  add_dependencies(CheckHypAction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(CheckHypAction ${catkin_LIBRARIES})
 
  add_executable(HypReadyAction src/HypReadyAction.cpp)
  add_dependencies(HypReadyAction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(HypReadyAction ${catkin_LIBRARIES})
 
  catkin_install_python(PROGRAMS
  
   scripts/hint_armor.py
   scripts/plan.py
   
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
