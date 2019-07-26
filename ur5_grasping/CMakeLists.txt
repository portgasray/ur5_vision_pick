cmake_minimum_required(VERSION 2.8.3)
project(ur5_grasping)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS gazebo_msgs roscpp geometry_msgs std_srvs std_msgs message_generation)

add_message_files(FILES Tracker.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(visonManager src/vision_manager.cpp)

# target_link_libraries(visonManager ${OpenCV_LIBS})

# Declare the executable, along with its source files.
# add_executable(graspingDemo src/grasping_demo.cpp)

# Specify libraries against which to link.
# target_link_libraries(graspingDemo visonManager ${catkin_LIBRARIES})