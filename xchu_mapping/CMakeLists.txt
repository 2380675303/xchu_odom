cmake_minimum_required(VERSION 2.8.3)
project(xchu_mapping)

message("Compiler version is: ${CMAKE_SYSTEM_VERSION}")
message("System version is: ${CMAKE_SYSTEM_NAME}")

set(CMAKE_BUILD_TYPE "Debug")

if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Darwin")
    set(OSX True)
    add_definitions(-D NO_OMP)
    find_package(catkin REQUIRED COMPONENTS
            roscpp
            nav_msgs
            pcl_conversions
            sensor_msgs
            tf
            std_msgs
            geometry_msgs
            ndt_cpu
            ndt_omp
            )
else ()
    find_package(catkin REQUIRED COMPONENTS
            roscpp
            nav_msgs
            pcl_conversions
            sensor_msgs
            tf
            std_msgs
            geometry_msgs
            ndt_cpu
            ndt_omp
            )
endif ()

add_compile_options(-std=c++11)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)


if (OSX)
    link_directories(${PCL_LIBRARY_DIRS})
    catkin_package(
            INCLUDE_DIRS include
            CATKIN_DEPENDS nav_msgs roscpp tf sensor_msgs std_msgs geometry_msgs pcl_conversions ndt_cpu ndt_omp)
else ()
    link_directories(#[[lib]] ${PCL_LIBRARY_DIRS})
    catkin_package(
            INCLUDE_DIRS include
            CATKIN_DEPENDS nav_msgs roscpp tf sensor_msgs std_msgs geometry_msgs pcl_conversions ndt_cpu ndt_omp)
endif ()
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

include_directories(
        include ${catkin_INCLUDE_DIRS}
        include/
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

add_executable(xchu_odom_node src/xchu_mapping.cpp)
if (OSX)
    target_link_libraries(xchu_odom_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
else ()
    target_link_libraries(xchu_odom_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} #[[ndt_cpu ndt_omp]])
endif ()


