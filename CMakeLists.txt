cmake_minimum_required(VERSION 2.6)
cmake_policy(SET CMP0015 OLD)

project(BetterTeach)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(YAMLCPP REQUIRED)

include_directories(include/ "${libpointmatcher_INCLUDE_DIRS}" ${Boost_INCLUDE_DIR})
link_directories({$libpointmatcher_LIB})

add_library(teachrepeat
        src/teach_repeat_map.cpp
        include/teach_repeat_map.h
        src/pose.cpp
        include/pose.h
        src/command.cpp
        include/command.h
        src/transform.cpp
        include/transform.h
        src/localised_point_cloud.cpp
        include/localised_point_cloud.h
        src/attraction_bassin_builder.cpp
        include/attraction_bassin_builder.h include/ellipse.hpp)

add_executable(BetterTeach
        src/attraction_bassin_main.cpp
        include/tolerance_ellipse_calculator.hpp)
target_link_libraries(BetterTeach pointmatcher teachrepeat yaml-cpp ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES})

add_executable(ToleranceEllipse
        src/tolerance_ellipse_main.cpp
        include/tolerance_ellipse_calculator.hpp
        include/convergence_distance_function.hpp)
target_link_libraries(ToleranceEllipse teachrepeat pointmatcher yaml-cpp ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES})
