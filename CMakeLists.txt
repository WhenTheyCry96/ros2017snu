cmake_minimum_required(VERSION 2.8.3)
project(final_project)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_msgs
  geometry_msgs
  joy
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  project2
)

# Find Eigen3 (from http://wiki.ros.org/jade/Migration)
#find_package(Eigen3)
#if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  #find_package(cmake_modules REQUIRED)
  #find_package(Eigen REQUIRED)
  #set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  #set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
#else()
  #set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
#endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   gazebo_msgs#   geometry_msgs#   nav_msgs#   sensor_msgs
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES project4
#  CATKIN_DEPENDS cv_bridge gazebo_msgs geometry_msgs joy nav_msgs roscpp rospy sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#set(ROOT_DIR ~/catkin_ws/src/project4/include/project4)
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIRS}
)
include_directories(${PROJECT4_INCLUDE_DIRS})
#include_directories(${ROOT_DIR})
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
#add_library(Utility SHARED "${ROOT_DIR}/pid.cpp" "${ROOT_DIR}/rrtTree.cpp") 

## Declare a cpp library
# add_library(project4
#   src/${PROJECT_NAME}/project4.cpp
# )

## Declare a cpp executable
# add_executable(project4_node src/project4_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(project4_node project4_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(project4_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS project4 project4_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_project2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#############
## custom  ##

add_executable(final_project src/main.cpp)
target_link_libraries(final_project
   ${catkin_LIBRARIES} ${GAZEBO_INCLUDE_DIRS} ${RRT_LIBRARIES}
)

#add_library(map_saver include/project4/map_saver.cpp)
#target_link_libraries(map_saver ${catkin_LIBRARIES} ${QT_LIBRARIES})
#add_dependencies(map_saver ${catkin_EXPORTED_TARGETS})

#add_executable(odometry src/odometry.cpp)
#target_link_libraries(odometry
#   ${catkin_LIBRARIES}
#)
#############
catkin_install_python(PROGRAMS
	scripts/gazebo_odometry_prj4.py
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
