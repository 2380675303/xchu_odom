//
// Created by xchu on 2021/4/15.
//

#ifndef SRC_XCHU_MAPPING_SRC_LOOP_DETETCION_H_
#define SRC_XCHU_MAPPING_SRC_LOOP_DETETCION_H_

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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>


#include <xchu_mapping/LoopInfo.h>


using PointT = pcl::PointXYZI;
struct PointXYZIRPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x)(float, y, y)
                                       (float, z, z)(float, intensity, intensity)
                                       (float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)
                                       (double, time, time)
)
typedef PointXYZIRPYT PointTypePose;

ros::Publisher loop_info_pub;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue_;
std::queue<nav_msgs::Odometry::ConstPtr> odom_queue_;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterKeyFrames, downSizeFilterHistoryKeyFrames; // for global map visualization

pcl::PointCloud<pcl::PointXYZI>::Ptr near_history_frame_cloud_;
pcl::PointCloud<pcl::PointXYZI>::Ptr latest_frame_cloud_;

pcl::PointCloud<PointTypePose>::Ptr cloud_keyposes_6d_;
pcl::PointCloud<PointT>::Ptr cloud_keyposes_3d_;
std::vector<double> travel_distance_arr_;
std::vector<Eigen::Vector3d> pos_arr_;
std::vector<Eigen::Isometry3d> odom_arr_;
std::vector<ros::Time> time_arr_;
int current_frame_id_ = 0;
std::vector<int> matched_frame_id;


int historyKeyframeSearchNum = 15;

std::vector<pcl::PointCloud<PointT>::Ptr> key_frames_vec_;

pcl::KdTreeFLANN<PointT>::Ptr kdtree_history_key_poses_;

pcl::IterativeClosestPoint<PointT, PointT> icp;


#endif //SRC_XCHU_MAPPING_SRC_LOOP_DETETCION_H_
