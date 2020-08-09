cmake_minimum_required(VERSION 2.8.3)
project(put_to_wall_manipulation)


find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
link_libraries(${OCTOMAP_LIBRARIES})
find_package( PCL REQUIRED )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  moveit_core
  std_msgs
  trajectory_msgs
  dynamixel_msgs
  sensor_msgs
  message_generation
  dynamixel_workbench_msgs
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR} 
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
  ${OCTOMAP_INCLUDE_DIRS}
)

add_message_files(
 FILES
 WorkSpace_custom.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
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
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arm_controller
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#)

catkin_package(
  INCLUDE_DIRS 
  include
  LIBRARIES put_to_wall_manipulation
  CATKIN_DEPENDS moveit_core roscpp message_runtime
  #  DEPENDS system_lib
)

###########
## Build ##
###########

add_compile_options(-std=c++11)

add_executable(generate_planning_scene src/generate_planning_scene.cpp)
target_link_libraries(generate_planning_scene ${catkin_LIBRARIES})
add_dependencies(generate_planning_scene put_to_wall_manipulation_gencpp)

add_executable(put_to_wall_sim src/put_to_wall_octagon_simulation.cpp)
target_link_libraries(put_to_wall_sim ${catkin_LIBRARIES})
add_dependencies(put_to_wall_sim put_to_wall_manipulation_gencpp)

add_executable(get_current_pose src/get_current_pose.cpp)
target_link_libraries(get_current_pose ${catkin_LIBRARIES})
add_dependencies(get_current_pose arm_controller_gencpp)

#add_executable(io_control src/io_control.cpp)
#target_link_libraries(io_control ${catkin_LIBRARIES})
#add_dependencies(io_control io_control_gencpp aubo_msgs_gencpp)

#add_executable(io_subscriber src/io_subscriber.cpp)
#target_link_libraries(io_subscriber ${catkin_LIBRARIES})
#add_dependencies(io_subscriber io_subscriber_gencpp aubo_msgs_gencpp) 

#add_executable(main_octomap_publisher src/main_octomap_publisher.cpp)
#target_link_libraries(main_octomap_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(main_octomap_publisher main_octomap_publisher_gencpp)

#add_executable(gripper_joint_publisher src/gripper_joint_publisher.cpp)
#target_link_libraries(gripper_joint_publisher ${catkin_LIBRARIES})
#add_dependencies(gripper_joint_publisher gripper_joint_publisher_gencpp)

#add_executable(gripper_demo_node src/gripper_demo_node.cpp)
#target_link_libraries(gripper_demo_node ${catkin_LIBRARIES})
#add_dependencies(gripper_demo_node gripper_demo_node_gencpp)

## For adding libraries ##
#set(MAIN_SOURCE_FILES
#    services/Barcode_Scanning.cpp
#    src/create_obstacles_for_boston_lab.cpp)

#add_library(put_to_wall_manipulation ${MAIN_SOURCE_FILES})

#target_link_libraries(put_to_wall_manipulation ${catkin_LIBRARIES})

#add_executable(barcode_scanning src/barcode_scan_test.cpp services/Barcode_Scanning.cpp)
#target_link_libraries(barcode_scanning ${catkin_LIBRARIES})
#add_dependencies(barcode_scanning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(caching_subscriber src/caching_subscriber.cpp)
#target_link_libraries(caching_subscriber ${catkin_LIBRARIES})
#add_dependencies(caching_subscriber caching_subscriber_gencpp arm_controller_generate_messages_cpp)

#add_executable(gripper_action_node src/gripper_action_node.cpp) 
#target_link_libraries(gripper_action_node ${catkin_LIBRARIES}) 
#add_dependencies(gripper_action_node gripper_action_node_gencpp) 

#add_executable(gripper_test src/gripper_test.cpp)
#add_dependencies(gripper_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(gripper_test ${catkin_LIBRARIES})

#add_executable(gripper_v1_calib src/gripper_v1_calib.cpp)
#add_dependencies(gripper_v1_calib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(gripper_v1_calib ${catkin_LIBRARIES})

#add_executable(robot_logger src/ros_out_logging.cpp)
#add_dependencies(robot_logger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(robot_logger ${catkin_LIBRARIES})

#add_executable(gripper_motion_test src/Gripper_V1_motion_test.cpp)
#add_dependencies(gripper_motion_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(gripper_motion_test ${catkin_LIBRARIES})
