cmake_minimum_required(VERSION 2.8.3)
project(pybind11_utils)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
              pybind11_catkin
              eigen_utils
              control_arch
              roscpp
              nav_msgs
            )

catkin_package(
            INCLUDE_DIRS include
            DEPENDS EIGEN3
            CATKIN_DEPENDS
            pybind11_catkin
            roscpp
            control_arch
            eigen_utils
	    nav_msgs
            )

catkin_python_setup()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# pybind modules
pybind_add_module(test_ros MODULE src/pybind11_utils/test_ros.cc)
pybind_add_module(test_geometry MODULE src/pybind11_utils/test_geometry.cc)

# install libraries
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
