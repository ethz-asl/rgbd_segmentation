cmake_minimum_required(VERSION 2.8.3)
project(rgbd_segmentation)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/rgbd_segmentation.cc
)

cs_add_executable(rgbd_segmentation_node
  src/node.cc
)
target_link_libraries(rgbd_segmentation_node ${PROJECT_NAME})

cs_install()
cs_export()
