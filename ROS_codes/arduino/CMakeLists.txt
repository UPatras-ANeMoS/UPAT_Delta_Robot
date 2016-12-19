cmake_minimum_required(VERSION 2.8.3)
project(arduino)
find_package(catkin REQUIRED 
	COMPONENTS
	roscpp 
	std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

#INCLUDE HEADERS
include_directories("${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include")
include_directories("${CMAKE_SOURCE_DIR}/include")

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

add_executable(arduino src/serial_publisher.cpp src/serial_board.cpp)
target_link_libraries(arduino ${catkin_LIBRARIES})

#add_executable(serial_listen src/serial_listener.cpp)
#target_link_libraries(serial_listen ${catkin_LIBRARIES})
