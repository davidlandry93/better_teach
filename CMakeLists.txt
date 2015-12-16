cmake_minimum_required(VERSION 3.1)
cmake_policy(SET CMP0015 OLD)

project(BetterTeach)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(YAMLCPP REQUIRED)

include_directories(src "${libpointmatcher_INCLUDE_DIRS}" ${Boost_INCLUDE_DIR})
link_directories({$libpointmatcher_LIB})

add_library(pointmatcherservice STATIC
        src/transform.cpp
        src/localised_point_cloud.cpp
        )
target_link_libraries(pointmatcherservice pointmatcher)

add_library(teachrepeat STATIC
        src/teach_repeat_map.cpp
        src/pose.cpp
        src/command.cpp
        )
target_link_libraries(teachrepeat pointmatcherservice)

add_executable(ToleranceEllipse
        src/tolerance_ellipse_main.cpp)
target_link_libraries(ToleranceEllipse teachrepeat yaml-cpp ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES})

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
