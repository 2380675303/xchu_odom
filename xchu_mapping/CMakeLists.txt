cmake_minimum_required(VERSION 2.8.3)
project(xchu_mapping)

add_compile_options(-std=c++11)

find_package(PCL 1.7 REQUIRED)
find_package(CUDA)
if (CUDA_FOUND)
    add_definitions(-DCUDA_FOUND)
    include_directories(${CUDA_INCLUDE_DIRS})
endif ()
include_directories(include)
link_directories(lib)
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

link_directories(${PCL_LIBRARY_DIRS})
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

add_executable(xchu_mapping_node src/xchu_mapping_node.cpp src/app/xchu_mapping.cpp #[[include/xchu_mapping/LidarMapping.h]])
target_link_libraries(xchu_mapping_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(xchu_mapping_node ndt_cpu pcl_omp_registration ndt_omp)
if (CUDA_FOUND)
    target_link_libraries(fast_ndt_mapping_node ndt_gpu)
endif ()
