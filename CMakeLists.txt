cmake_minimum_required(VERSION 2.8.3)
project(osa_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  rosbag
  roscpp
  roslib
  std_msgs
  tf
  osa_msgs
  osa_communication
  visualization_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
  actionlib_msgs
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)
#find_package(Eigen REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  getAllCurrentCmd.srv
  getHeadPositionCmd.srv
  getSlaveCmdArray.srv
  getSlaveCurrentCmd.srv
  getSlaveLockCmd.srv
  switchNode.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES PlaySequence.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  osa_msgs
  actionlib_msgs
)

## Generate dynamic reconfigure options
generate_dynamic_reconfigure_options(
  cfg/hsa_balance_dyn.cfg
  cfg/motor_dyn_config.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include include/osa_control
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rosbag roscpp roslib std_msgs tf osa_msgs osa_communication visualization_msgs actionlib_msgs
  DEPENDS Eigen
)

###########
## Build ##
###########

add_compile_options(-std=c++11)

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(include)
#include_directories(include/osa_control)
include_directories(
  include
#  include/osa_control
  ${catkin_INCLUDE_DIRS}
#  ${Boost_INCLUDE_DIRS}
  /usr/include/eigen3
)
#include_directories(/usr/include/eigen3)

## Declare a cpp library
# add_library(osa_control
#   src/${PROJECT_NAME}/osa_control.cpp
# )

#add_library(play_sequence_action_client src/play_sequence_action_client.cpp)
#add_library(osa_control src/play_sequence_action_client.cpp)

## Declare a cpp executable
add_executable(osa_command_builder src/command_builder.cpp)
add_executable(osa_command_filter src/command_filter.cpp)
add_executable(osa_bag_recorder_motion src/bag_recorder_motion.cpp)
add_executable(osa_bag_recorder src/bag_recorder.cpp)
add_executable(osa_xbox_joy_conductor src/xbox_joy_conductor.cpp)
add_executable(osa_arm_manual src/arm_manual.cpp)
add_executable(osa_mobile_base_manual src/mobile_base_manual.cpp) 
add_executable(osa_mobile_base_follower src/mobile_base_follower.cpp) 
add_executable(osa_base_test_modes src/base_test_modes.cpp)
add_executable(osa_tank_drive src/tank_drive.cpp)
add_executable(osa_hsa_balance src/hsa_balance.cpp)

add_executable(osa_play_sequence_action_server src/play_sequence_action_server.cpp)
add_executable(osa_joystick_action_client src/joystick_action_client.cpp src/play_sequence_action_client.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(osa_bag_recorder osa_control_generate_messages_cpp)
add_dependencies(osa_arm_manual osa_control_generate_messages_cpp)
add_dependencies(osa_mobile_base_manual osa_control_generate_messages_cpp)
add_dependencies(osa_mobile_base_follower osa_control_generate_messages_cpp)
add_dependencies(osa_xbox_joy_conductor osa_control_generate_messages_cpp)
add_dependencies(osa_tank_drive osa_control_generate_messages_cpp)
add_dependencies(osa_command_filter osa_control_generate_messages_cpp osa_control_gencfg)
add_dependencies(osa_hsa_balance osa_control_generate_messages_cpp osa_control_gencfg)

add_dependencies(osa_play_sequence_action_server osa_control_generate_messages_cpp)
add_dependencies(osa_joystick_action_client osa_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(osa_command_builder ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} osa_common)
target_link_libraries(osa_command_filter ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} osa_common)
target_link_libraries(osa_bag_recorder_motion ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(osa_bag_recorder ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(osa_xbox_joy_conductor ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(osa_base_test_modes ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(osa_arm_manual ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(osa_mobile_base_manual ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(osa_mobile_base_follower ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(osa_tank_drive ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(osa_hsa_balance ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

target_link_libraries(osa_play_sequence_action_server ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
#target_link_libraries(osa_joystick_action_client ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} play_sequence_action_client)
target_link_libraries(osa_joystick_action_client ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS osa_control osa_control_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_osa_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
