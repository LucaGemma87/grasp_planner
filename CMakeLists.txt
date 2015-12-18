cmake_minimum_required(VERSION 2.8.3)
project(grasp_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs
  geometry_msgs shape_msgs sensor_msgs visualization_msgs urdf tf pcl_conversions pcl_ros pcl_msgs visual_perception moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib  visual_perception moveit_msgs)



## System dependencies are found with CMake's conventions
##find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL 1.7 REQUIRED)

# set(CMAKE_CXX_FLAGS "-std=c++0x")
# set(PACMAN /home/pacman/Projects/pacman)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
FILES
Trajectory.msg
)



## Generate services in the 'srv' folder
add_service_files(
  FILES
  GraspPlanning.srv
  TrajectoryDemo.srv
  TrajectoryDemoOnline.srv
  
  TrajectoryDemoCylinder.srv
  TrajectoryDemoSphere.srv
  TrajectoryDemoCone.srv
  TrajectoryDemoPlane.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  shape_msgs
  sensor_msgs
  visual_perception
  moveit_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 
 ##LIBRARIES 
 ##CATKIN_DEPENDS roscpp
 ##DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}  
  visual_perception
  # ${PACMAN}/pacman/include/
)

# link_directories(
#   ${PACMAN}/bin/
#   )

## Declare a cpp library


## Declare a cpp executable
add_executable(grasp_planning src/grasp_planning.cpp)
add_executable(trajectory_manager_demo src/trajectory_manager_demo.cpp)
add_executable(trajectory_manager_online src/trajectory_manager_online.cpp)
add_executable(trajectory_manager_online_demo src/trajectory_manager_online_demo.cpp)

add_executable(trajectory_demo_cylinder_affordance_1_online src/trajectory_demo_cylinder_affordance_1_online.cpp)
add_executable(trajectory_demo_cylinder_affordance_2_online src/trajectory_demo_cylinder_affordance_2_online.cpp)

add_executable(trajectory_demo_sphere_affordance_1_online src/trajectory_demo_sphere_affordance_1_online.cpp)
add_executable(trajectory_demo_sphere_affordance_2_online src/trajectory_demo_sphere_affordance_2_online.cpp)

add_executable(trajectory_demo_cone_affordance_1_online src/trajectory_demo_cone_affordance_1_online.cpp)
add_executable(trajectory_demo_cone_affordance_2_online src/trajectory_demo_cone_affordance_2_online.cpp)

add_executable(trajectory_demo_box_affordance_1_online src/trajectory_demo_box_affordance_1_online.cpp)
add_executable(trajectory_demo_box_affordance_2_online src/trajectory_demo_box_affordance_2_online.cpp)

add_executable(trajectory_demo_cylinder_affordance_1 src/trajectory_demo_cylinder_affordance_1.cpp)
add_executable(trajectory_demo_cylinder_affordance_2 src/trajectory_demo_cylinder_affordance_2.cpp)

add_executable(trajectory_demo_sphere_affordance_1 src/trajectory_demo_sphere_affordance_1.cpp)
add_executable(trajectory_demo_sphere_affordance_2 src/trajectory_demo_sphere_affordance_2.cpp)

add_executable(trajectory_demo_cone_affordance_1 src/trajectory_demo_cone_affordance_1.cpp)
add_executable(trajectory_demo_cone_affordance_2 src/trajectory_demo_cone_affordance_2.cpp)

add_executable(trajectory_demo_box_affordance_1 src/trajectory_demo_box_affordance_1.cpp)
add_executable(trajectory_demo_box_affordance_2 src/trajectory_demo_box_affordance_2.cpp)

#add_executable(trajectory_demo_2 src/trajectory_demo_2.cpp)
#add_executable(trajectory_demo_3 src/trajectory_demo_3.cpp)
#add_executable(trajectory_demo_point_to_point src/trajectory_demo_point_to_point.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(grasp_planning grasp_planner_generate_messages
  grasp_planner_gen_cpp
)
add_dependencies(trajectory_manager_demo grasp_planner_gen_cpp visual_perception_gencpp grasp_planner_generate_messages)
add_dependencies(trajectory_manager_online grasp_planner_gen_cpp visual_perception_gencpp grasp_planner_generate_messages)
add_dependencies(trajectory_manager_online_demo grasp_planner_gen_cpp visual_perception_gencpp grasp_planner_generate_messages)


add_dependencies(trajectory_demo_cylinder_affordance_1
  grasp_planner_gen_cpp visual_perception_gencpp  grasp_planner_generate_messages                  )
add_dependencies(trajectory_demo_cylinder_affordance_2
  grasp_planner_gen_cpp  grasp_planner_generate_messages
)

add_dependencies(trajectory_demo_sphere_affordance_1
  grasp_planner_gen_cpp  grasp_planner_generate_messages visual_perception_gencpp visual_perception_gencpp
)
add_dependencies(trajectory_demo_sphere_affordance_2
  grasp_planner_gen_cpp  grasp_planner_generate_messages visual_perception_gencpp visual_perception_gencpp
)
add_dependencies(trajectory_demo_cone_affordance_1
  grasp_planner_gen_cpp  grasp_planner_generate_messages visual_perception_gencpp visual_perception_gencpp
)
add_dependencies(trajectory_demo_cone_affordance_2
  grasp_planner_gen_cpp grasp_planner_generate_messages visual_perception_gencpp visual_perception_gencpp
)

add_dependencies(trajectory_demo_box_affordance_1
  grasp_planner_gen_cpp  grasp_planner_generate_messages visual_perception_gencpp visual_perception_gencpp
)

add_dependencies(trajectory_demo_box_affordance_2
  grasp_planner_gen_cpp visual_perception_gencpp                  visual_perception_generate_messages_cpp   visual_perception_generate_messages_py    visual_perception_genpy
visual_perception_generate_messages       visual_perception_generate_messages_lisp  visual_perception_genlis visual_perception_gencpp
)
add_dependencies(trajectory_demo_cylinder_affordance_1_online visual_perception_gencpp
grasp_planner_generate_messages
)

add_dependencies(trajectory_demo_cylinder_affordance_2_online visual_perception_gencpp
grasp_planner_generate_messages
)

add_dependencies(trajectory_demo_sphere_affordance_1_online visual_perception_gencpp
grasp_planner_generate_messages
)

add_dependencies(trajectory_demo_sphere_affordance_2_online visual_perception_gencpp
grasp_planner_generate_messages
)

add_dependencies(trajectory_demo_cone_affordance_1_online visual_perception_gencpp
grasp_planner_generate_messages
)

add_dependencies(trajectory_demo_cone_affordance_2_online visual_perception_gencpp
grasp_planner_generate_messages
)

add_dependencies(trajectory_demo_box_affordance_1_online visual_perception_gencpp
grasp_planner_generate_messages
)

add_dependencies(trajectory_demo_box_affordance_2_online visual_perception_gencpp
grasp_planner_generate_messages
)

#add_dependencies(trajectory_demo_2 grasp_planner_gen_cpp)
#add_dependencies(trajectory_demo_3 grasp_planner_gen_cpp)
#add_dependencies(trajectory_demo_point_to_point grasp_planner_gen_cpp)



## Specify libraries to link a library or executable target against
target_link_libraries(grasp_planning  ${catkin_LIBRARIES})
target_link_libraries(trajectory_manager_demo  ${catkin_LIBRARIES})
target_link_libraries(trajectory_manager_online  ${catkin_LIBRARIES})
target_link_libraries(trajectory_manager_online_demo  ${catkin_LIBRARIES})



target_link_libraries(trajectory_demo_cylinder_affordance_1  ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_cylinder_affordance_2  ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_sphere_affordance_1  ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_sphere_affordance_2  ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_cone_affordance_1  ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_cone_affordance_2  ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_box_affordance_1  ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_box_affordance_2  ${catkin_LIBRARIES})


target_link_libraries(trajectory_demo_cylinder_affordance_1_online ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_cylinder_affordance_2_online ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_sphere_affordance_1_online ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_sphere_affordance_2_online ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_cone_affordance_1_online ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_cone_affordance_2_online ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_box_affordance_1_online ${catkin_LIBRARIES})
target_link_libraries(trajectory_demo_box_affordance_2_online ${catkin_LIBRARIES})

#target_link_libraries(trajectory_demo_2  ${catkin_LIBRARIES})
#target_link_libraries(trajectory_demo_3  ${catkin_LIBRARIES})
#target_link_libraries(trajectory_demo_point_to_point  ${catkin_LIBRARIES})




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS phobic_robot_demo phobic_robot_demo_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_phobic_robot_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

link_directories(
  ${PCL_LIBRARY_DIRS}
  )
add_definitions(
  ${PCL_DEFINITIONS}
  )