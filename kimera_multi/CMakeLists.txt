cmake_minimum_required(VERSION 3.5)
project(kimera_multi)

SET(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs sensor_msgs geometry_msgs
  std_msgs cv_bridge pose_graph_tools_msgs dbow2_catkin)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp nav_msgs sensor_msgs
  geometry_msgs std_msgs cv_bridge pose_graph_tools_msgs dbow2_catkin)

add_executable(odin_sdk_frontend src/odin_sdk_frontend.cpp)
target_include_directories(odin_sdk_frontend PRIVATE include ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(odin_sdk_frontend ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(odin_sdk_frontend ${catkin_EXPORTED_TARGETS})
install(TARGETS odin_sdk_frontend RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(DIRECTORY launch params adapters
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.egg-info" EXCLUDE)
