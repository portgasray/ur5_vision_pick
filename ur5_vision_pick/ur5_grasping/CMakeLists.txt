cmake_minimum_required(VERSION 3.1.0)
project(ur5_grasping)

# DEPS
# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS 
    gazebo_msgs
    roscpp
    geometry_msgs
    std_srvs
    std_msgs 
    message_generation
    )

find_package(OpenCV REQUIRED)

add_message_files(FILES Tracker.msg)
generate_messages(DEPENDENCIES std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} )

catkin_package()

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(realsense2_DIR )


## Declare a C++ library
add_library(visonManager src/vision_manager.cpp)

target_link_libraries(visonManager ${OpenCV_LIBS})

# Declare the executable, along with its source files.
add_executable(graspingDemo src/grasping_demo.cpp)

# Specify libraries against which to link.
target_link_libraries(graspingDemo visonManager ${catkin_LIBRARIES})
# add_executable(eye_to_hand src/eye_to_hand.cpp)
# target_link_libraries(eye_to_hand ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
