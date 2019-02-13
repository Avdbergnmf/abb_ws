
# What minimum version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)
# Name of this Package
project(pathmaker_abb)

# add c++11 library
add_compile_options(-std=c++11)

# add the eigen packages.  required for kinematics stuff
find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# Find the catkin build system, and any other packages on which we depend
find_package(catkin REQUIRED
  COMPONENTS
    roscpp std_msgs geometry_msgs tf control_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    eigen_conversions
)

find_package(Boost REQUIRED system filesystem date_time thread) # no fucking clue

# Add some custom messages
add_message_files(FILES GeomagicButtonEvent.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

# Declare our catkin package
catkin_package(
    CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning_interface
    DEPENDS
        EIGEN3
)

# Specify locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

# include header files in this folder
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")

    

# Declare the executable, along wit its source files.
# If there are multiple executables, use multiple copies of this line. 
add_executable(pathmaker_abb src/pathmaker_abb.cpp)

# Specify libaraies against which to link. Again, this line should
# be copied for each distinct executable in the package.
target_link_libraries(pathmaker_abb ${catkin_LIBRARIES})

