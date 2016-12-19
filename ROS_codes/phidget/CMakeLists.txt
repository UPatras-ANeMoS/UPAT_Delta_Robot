cmake_minimum_required(VERSION 2.8.3)
project(phidget)
find_package(catkin REQUIRED 
	COMPONENTS
	roscpp 
	std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

#set_target_properties(phidget PROPERTIES LINKER_LANGUAGE CXX)
#message(STATUS ${CMAKE_SOURCE_DIR})
#add_library(phidget21 STATIC "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/")

#find_library(libphidget21 "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/libphidget21.so")



catkin_package()


#INCLUDE HEADERS
include_directories("${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include")
include_directories("${CMAKE_SOURCE_DIR}/include")


#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

add_executable(phidget src/phidget.cpp )
# target_link_libraries(phidget ${catkin_LIBRARIES} "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/libphidget21.a" "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/libphidget21.so")
target_link_libraries(phidget ${catkin_LIBRARIES} phidget21)


#add_executable(serial_listen src/serial_listener.cpp)
#target_link_libraries(serial_listen ${catkin_LIBRARIES})
