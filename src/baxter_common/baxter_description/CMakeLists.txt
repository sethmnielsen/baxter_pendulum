cmake_minimum_required(VERSION 2.8.4)
project(baxter_description)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED)

catkin_package()

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  std_srvs
  gazebo_ros
  gazebo_plugins
)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs std_srvs gazebo_ros gazebo_plugins
DEPENDS EIGEN3 GAZEBO
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(pose_plugin
  src/pose_plugin.cpp)
target_link_libraries(pose_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(pose_plugin ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    pose_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

foreach(dir meshes urdf)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
