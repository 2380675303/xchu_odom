cmake_minimum_required(VERSION 2.8.3)
project(xchu_mapping)

add_compile_options(-std=c++11)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        nav_msgs
        pcl_conversions
        sensor_msgs
        tf
        std_msgs
        geometry_msgs
        )

link_directories(lib ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS nav_msgs roscpp tf sensor_msgs std_msgs geometry_msgs pcl_conversions
)

include_directories(
        include ${catkin_INCLUDE_DIRS}
        include/
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

add_executable(xchu_odom_node  src/xchu_mapping.cpp)
target_link_libraries(xchu_odom_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ndt_cpu  ndt_omp)


