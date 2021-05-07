//
// Created by xchu on 2021/4/13.
//

#ifndef SRC_XCHU_MAPPING_SRC_MAPPING_NODE_H_
#define SRC_XCHU_MAPPING_SRC_MAPPING_NODE_H_


#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <thread>
#include <mutex>
#include <queue>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZI PointType;

ros::Publisher final_map_pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr  globalmap_ptr;  // 此处定义地图  --global map

std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue;
std::queue<nav_msgs::Odometry::ConstPtr> odom_queue;

std::mutex mutex_lock;

pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[500];

//ros::Subscriber odom_sub, points_sub;

double time_odom = 0;
double time_cloud = 0;


pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterGlobalMap, downSizeFilterKeyFrames; // for global map visualization


/*class MappingThread {
 public:
  MappingThread(ros::NodeHandle &nh);

  MappingThread() {};


  void Run();




  *//**
   * 点云处理和匹配
   * @param input
   * @param current_scan_time
   *//*
  void process_cloud(const pcl::PointCloud<pcl::PointXYZI> &input, const ros::Time &current_scan_time);

  void param_initial(ros::NodeHandle &private_handle);

  *//**
   * 保存地图及其他点云
   *//*
  void save_map();

  void PublishCloud();

  void ViewerThread();

  void imu_callback(const sensor_msgs::Imu::Ptr &input);

  void odom_callback(const nav_msgs::Odometry::ConstPtr &input);

  void PointCb(const sensor_msgs::PointCloud2::ConstPtr &input);

  void imu_odom_calc(ros::Time current_time);

  void imu_calc(ros::Time current_time);

  void odom_calc(ros::Time current_time);

  void imuUpSideDown(const sensor_msgs::Imu::Ptr input);

  void OdomCb(const nav_msgs::Odometry::ConstPtr &input);

  void imu_info(const sensor_msgs::Imu &input);

 private:
  ros::Subscriber odom_sub, points_sub;

  pcl::PointCloud<pcl::PointXYZI>::Ptr  globalmap_ptr, transformed_scan_ptr;  // 此处定义地图  --global map

  std::queue<sensor_msgs::PointCloud2::ConstPtr> cloud_queue;
  std::queue<nav_msgs::Odometry::ConstPtr> odom_queue;

  std::mutex mutex_lock;


};*/
#endif //SRC_XCHU_MAPPING_SRC_MAPPING_NODE_H_
