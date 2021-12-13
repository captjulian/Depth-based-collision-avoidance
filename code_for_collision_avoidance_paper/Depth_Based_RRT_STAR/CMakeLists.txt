cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME Depth_BASED_RRT_STAR)

project(${PROJECT_NAME})

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(Depth_BASED_RRT_STAR_SOURCE_DIR
        src/sources)

set(Depth_BASED_RRT_STAR_INCLUDE_DIR
        src/include
        )

    set(Depth_BASED_RRT_STAR_SOURCE_FILES
	${Depth_BASED_RRT_STAR_SOURCE_DIR}/pugixml.cpp
	${Depth_BASED_RRT_STAR_SOURCE_DIR}/mtrand.cpp
	${Depth_BASED_RRT_STAR_SOURCE_DIR}/rrtstar.cpp
	#${Depth_BASED_RRT_STAR_SOURCE_DIR}/depth_based_rrt_star.cpp
        ${Depth_BASED_RRT_STAR_SOURCE_DIR}/sdf_collision_detector.cpp
        ${Depth_BASED_RRT_STAR_SOURCE_DIR}/fast_replan_using_sdf.cpp
        ${Depth_BASED_RRT_STAR_SOURCE_DIR}/fast_replan_using_sdf_real_flight.cpp
        ${Depth_BASED_RRT_STAR_SOURCE_DIR}/trajectory_generator_waypoint.cpp
            )


    set(Depth_BASED_RRT_STAR_HEADER_FILES
	 #${Depth_BASED_RRT_STAR_INCLUDE_DIR}/depth_based_rrt_star.h
	 ${Depth_BASED_RRT_STAR_INCLUDE_DIR}/rrtstar.h
	 ${Depth_BASED_RRT_STAR_INCLUDE_DIR}/mtrand.h
	 ${Depth_BASED_RRT_STAR_INCLUDE_DIR}/pugiconfig.hpp
	 ${Depth_BASED_RRT_STAR_INCLUDE_DIR}/pugixml.hpp
	 ${Depth_BASED_RRT_STAR_INCLUDE_DIR}/sdf_collision_detector.h
	 ${Depth_BASED_RRT_STAR_INCLUDE_DIR}/fast_replan_using_sdf.h
         ${Depth_BASED_RRT_STAR_INCLUDE_DIR}/fast_replan_using_sdf_real_flight.h
	 ${Depth_BASED_RRT_STAR_INCLUDE_DIR}/trajectory_generator_waypoint.h
            )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy  
  droneModuleROS
  droneMsgsROS
  opencv_apps
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  message_generation
  cv_bridge
  image_transport
  mav_trajectory_generation_ros
#  robot_process
##  NLopt
)

#if ompl
	
#find_package(ompl REQUIRED)



#if OpenCV
find_package(OpenCV REQUIRED)
#set(OpenCV_DIR /home/liang/third_part_package/opencv-3.3.0/build)
#find_package(OpenCV REQUIRED PATHS /home/liang/third_part_package/opencv-3.3.0/cmake)
#if(NOT OpenCV_FOUND)
#  message(FATAL_ERROR "Failed to find OpenCV")
#endif(NOT OpenCV_FOUND)
#include_directories(${OpenCV_INCLUDE_DIR})
#MESSAGE(STATUS "OPENCVLIB IS : " ${OpenCV_INCLUDE_DIR})
#MESSAGE(STATUS "OpenCV_DIR is: " ${OpenCV_INCLUDE_DIRS})
#MESSAGE(STATUS "OpenCV_LIBS is: " ${OpenCV_LIBS})


find_package(Eigen3 REQUIRED)
##add_definitions(${EIGEN_DEFINITIONS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#add_message_files(
#  FILES

#)

#add_service_files(
#  FILES
#  MakeRRTPlan.srv
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs

)

###################################
## catkin specific configuration ##


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs 
                 droneModuleROS droneMsgsROS opencv_apps message_runtime
                 geometry_msgs nav_msgs mav_trajectory_generation_ros
                 sensor_msgs visualization_msgs cv_bridge image_transport
		 #robot_process
  
  DEPENDS OpenCV Eigen
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

##include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
    ${Depth_BASED_RRT_STAR_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)


#include_directories(${OpenCV_INCLUDE_DIRS})



#add_library(RTRRTSTAR_lib ${RTRRTSTAR_SOURCE_FILES} ${RTRRTSTAR_HEADER_FILES})
add_library(Depth_BASED_RRT_STAR_lib ${Depth_BASED_RRT_STAR_SOURCE_FILES} ${Depth_BASED_RRT_STAR_HEADER_FILES})
add_dependencies(Depth_BASED_RRT_STAR_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(Depth_BASED_RRT_STAR_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBS})

#add_executable(depth_based_rrt_star_node src/sources/depth_based_rrt_star_node.cpp)
#add_dependencies(depth_based_rrt_star_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(depth_based_rrt_star_node Depth_BASED_RRT_STAR_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARY})

add_executable(fast_replan_using_sdf_node src/sources/fast_planner_using_sdf_node.cpp)
add_dependencies(fast_replan_using_sdf_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(fast_replan_using_sdf_node Depth_BASED_RRT_STAR_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARY})

add_executable(fast_planner_using_sdf_rf_node src/sources/fast_planner_using_sdf_rf_node.cpp)
add_dependencies(fast_planner_using_sdf_rf_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(fast_planner_using_sdf_rf_node Depth_BASED_RRT_STAR_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARY})

#add_executable(gazebo_pos_node src/sources/gazebo_pos.cpp)
#add_dependencies(gazebo_pos_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(gazebo_pos_node Depth_BASED_RRT_STAR_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARY})

#add_executable(image_sub src/sources/img_sub_test.cpp)
#add_dependencies(image_sub ${catkin_EXPORTED_TARGETS})
#target_link_libraries(image_sub ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARY})

