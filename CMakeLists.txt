cmake_minimum_required(VERSION 3.1)
cmake_policy(SET CMP0015 OLD)

project(BetterTeach)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations" )

find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(YAMLCPP REQUIRED)

include_directories(src "${libpointmatcher_INCLUDE_DIRS}" ${Boost_INCLUDE_DIR})
link_directories({$libpointmatcher_LIB})

add_library(pointmatcherservice STATIC
        src/transform.cpp
        src/localised_point_cloud.cpp)
target_link_libraries(pointmatcherservice pointmatcher)

add_library(teachrepeat STATIC
        src/teach_repeat_map.cpp
        src/pose.cpp
        src/command.cpp)
target_link_libraries(teachrepeat pointmatcherservice)

add_library(optimizeteachrepeat STATIC
        src/teachrepeat_map_optimizer.cpp
        src/teachrepeat_map_optimizer.h
        src/localizability_graph.cpp
        src/localizability_graph.h)
target_link_libraries(optimizeteachrepeat teachrepeat yaml-cpp ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES})

add_executable(optimizeteachrepeatmain
        src/tr_map_optimization_main.cpp)
target_link_libraries(optimizeteachrepeatmain optimizeteachrepeat)

add_executable(attractionbassinmain
        src/attraction_bassin_main.cpp
        src/attraction_bassin_builder.cpp)
target_link_libraries(attractionbassinmain pointmatcherservice optimizeteachrepeat)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Tests
configure_file(ExternalProjects.txt googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )

# Add googletest directly to our build. This adds
# the following targets: gtest, gtest_main, gmock
# and gmock_main
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL )

# The gtest/gmock targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include"
                      "${gmock_SOURCE_DIR}/include")
endif()

# Now simply link your own targets against gtest, gmock,
# etc. as appropriate

add_executable(runalltests tests/pointMatcherServiceTests.cpp tests/localizabilityGraphTest.cpp)
target_link_libraries(runalltests gtest gtest_main optimizeteachrepeat ${Boost_SYSTEM_LIBRARY} ${Boost_LIBRARIES})

add_executable(rununittests tests/localizabilityGraphTest.cpp)
target_link_libraries(rununittests optimizeteachrepeat gtest gtest_main)

add_test( runUnitTests runUnitTests )