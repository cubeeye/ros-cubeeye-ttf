cmake_minimum_required(VERSION 2.8.3)
project(cubeeye_ttf)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall" )

generate_dynamic_reconfigure_options(
  cfg/mdc_depth.cfg
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include/voxel
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
)

add_executable(${PROJECT_NAME}_node src/DepthTTFNode.cpp src/DepthTTFReader.cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${CMAKE_CURRENT_SOURCE_DIR}/lib/libTTF_API.so
)

