cmake_minimum_required(VERSION 3.16)
project(kinect)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# libfreenect2 via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(FREENECT2 REQUIRED freenect2)

include_directories(
  include
  ${FREENECT2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} # ADD THIS
)

link_directories(${FREENECT2_LIBRARY_DIRS})

# Build kinect_node (RGB + Depth + IR)
add_executable(kinect_node src/kinect_node.cpp)
ament_target_dependencies(kinect_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(kinect_node 
  ${FREENECT2_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Install executables
install(TARGETS kinect_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
