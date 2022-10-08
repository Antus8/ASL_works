cmake_minimum_required(VERSION 2.8.3)
project(stereo_vision)

# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  camera_info_manager
  roslaunch
  pcl_ros
  pcl_conversions
)


find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc imgcodecs video calib3d)

find_package(PCL 1.8.0 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge image_transport nav_msgs roscpp rospy sensor_msgs std_msgs camera_info_manager pcl_ros pcl_conversions
  DEPENDS Boost
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(FATAL_ERROR "${CMAKE_CXX_COMPILER} doesn't provide c++11 support.")
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${catkin_LIBRARY_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/camera_publisher.cpp src/scan_matcher.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(left_camera_publisher src/left_camera_publisher_node.cpp)
add_dependencies(left_camera_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(left_camera_publisher 
					  ${PROJECT_NAME} 
					  ${catkin_LIBRARIES} 
					  ${OpenCV_LIBRARIES}
					  ${Boost_LIBRARIES}
					  )
add_executable(right_camera_publisher src/right_camera_publisher_node.cpp)
add_dependencies(right_camera_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(right_camera_publisher 
					  ${PROJECT_NAME} 
					  ${catkin_LIBRARIES} 
					  ${OpenCV_LIBRARIES}
					  ${Boost_LIBRARIES}
					  )
add_executable(scan_matcher src/scan_matcher_node.cpp)
add_dependencies(scan_matcher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(scan_matcher 
					  ${PROJECT_NAME} 
					  ${catkin_LIBRARIES} 
					  ${OpenCV_LIBRARIES}
					  ${Boost_LIBRARIES}
            ${PCL_LIBRARIES}
					  )





