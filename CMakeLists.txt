cmake_minimum_required(VERSION 3.1)
cmake_policy(SET CMP0015 OLD)

project(BetterTeach)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(YAMLCPP REQUIRED)

include_directories("${libpointmatcher_INCLUDE_DIRS}" ${Boost_INCLUDE_DIR})
link_directories({$libpointmatcher_LIB})

add_library(pointmatcherservice STATIC
        src/transform.cpp
        src/transform.h
        src/localised_point_cloud.cpp
        src/localised_point_cloud.h
        src/pointmatcherservice.hpp
        src/pointmatcherservice.h
        )
target_link_libraries(pointmatcherservice pointmatcher)

add_library(teachrepeat STATIC
        src/teach_repeat_map.cpp
        src/teach_repeat_map.h
        src/pose.cpp
        src/pose.h
        src/command.cpp
        src/command.h
        src/ellipse.hpp)

add_executable(BetterTeach src/attraction_bassin_main.cpp
        src/attraction_bassin_builder.cpp
        src/attraction_bassin_builder.h)
target_link_libraries(BetterTeach teachrepeat yaml-cpp ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES})

add_executable(ToleranceEllipse
        src/tolerance_ellipse_main.cpp
        src/tolerance_ellipse_calculator.hpp
        src/ellipse.hpp
        src/point.hpp
        src/convergence_distance_function.hpp
        )
target_link_libraries(ToleranceEllipse pointmatcherservice teachrepeat yaml-cpp ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES})
set_property(TARGET ToleranceEllipse PROPERTY CXX_STANDARD 11)