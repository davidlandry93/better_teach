
cmake_minimum_required(VERSION 2.6)
project(BetterTeach)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Boost COMPONENTS program_options REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(libpointmatcher REQUIRED)

include_directories(include/ "${libpointmatcher_INCLUDE_DIRS}" ${Boost_INCLUDE_DIRS})
link_directories({$libpointmatcher_LIB})

add_library(teachrepeat
  src/anchor_point.cpp
  src/teach_repeat_map.cpp
  include/anchor_point.h
  include/teach_repeat_map.h
  src/pose.cpp
  include/pose.h
  src/command.cpp
  include/command.h)

add_executable(BetterTeach src/grid_maps_main.cpp src/grid_maps.cpp)
target_link_libraries(BetterTeach teachrepeat pointmatcher ${Boost_LIBRARIES})
