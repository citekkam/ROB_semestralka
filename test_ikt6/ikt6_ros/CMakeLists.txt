cmake_minimum_required(VERSION 2.8.12)
project(ikt6_ros)

add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

find_package(Eigen3 REQUIRED)
find_package(Boost)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  pluginlib
  roscpp
  srdfdom
  urdf
  tf
  tf2_eigen
)


set(THIS_PACKAGE_INCLUDE_DIRS
    ikt6_kinematics_plugin/include
    ikt6_iiwa_kinematics_plugin/include
    ikt6/include
    ikt6/tests
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM
                    ${THIS_PACKAGE_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS})


# we will ignore the makefile from ikt6 and compile the library here
add_library(ikt6 ikt6/src/util.cpp ikt6/src/ikt6.cpp)

add_subdirectory(ikt6_kinematics_plugin)
add_subdirectory(ikt6_iiwa_kinematics_plugin)
add_subdirectory(tests)

install(TARGETS ikt6
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  FILES
    ikt6_kinematics_plugin_description.xml
    ikt6_iiwa_kinematics_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
