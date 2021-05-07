//
// Created by xchu on 2021/4/16.
//

#ifndef SRC_XCHU_MAPPING_SRC_MAP_OPTIMIZE_H_
#define SRC_XCHU_MAPPING_SRC_MAP_OPTIMIZE_H_

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
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <xchu_mapping/LoopInfo.h>

//stop loop check for the next N frames if loop is identified
#define STOP_LOOP_CHECK_COUNTER 40

using namespace gtsam;
using PointT = pcl::PointXYZI;

ros::Publisher odom_pub, path_pub, pubLaserCloudSurround, optimize_poses_pub, origin_odom_pub;

std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue_;
std::queue<nav_msgs::Odometry::ConstPtr> odom_queue_;
std::queue<xchu_mapping::LoopInfo::ConstPtr> loop_queue_;

/*NonlinearFactorGraph gtSAMgraph;
Values initialEstimate;
Values optimizedEstimate;
ISAM2 *isam;
Values isamCurrentEstimate;*/

std::vector<gtsam::Pose3> pose_optimized_arr; // 优化后的pose
std::vector<gtsam::Pose3> odom_original_arr;  // odom先验
gtsam::Pose3 last_pose3;
Eigen::Isometry3d odom_curr_ = Eigen::Isometry3d::Identity(); //map
Eigen::Isometry3d odom_curr_last_ = Eigen::Isometry3d::Identity(); //map
Eigen::Isometry3d map_curr_ = Eigen::Isometry3d::Identity(); //map
Eigen::Isometry3d map_curr_last_ = Eigen::Isometry3d::Identity(); //map
Eigen::Isometry3d map_odom_curr_ = Eigen::Isometry3d::Identity(); //map
Eigen::Isometry3d map_odom_curr_last_ = Eigen::Isometry3d::Identity(); //map
int temp_laserCloudMap_Ind = 0;  // id序号


gtsam::noiseModel::Diagonal::shared_ptr priorModel;
gtsam::noiseModel::Diagonal::shared_ptr odomModel;
gtsam::noiseModel::Diagonal::shared_ptr loopModel;

int stop_check_loop_count = 0;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterKeyFrames, downSizeFilterHistoryKeyFrames, downSizeFilterGlobalMapKeyFrames; // for global map visualization


noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odometryNoise;
noiseModel::Diagonal::shared_ptr constraintNoise;

std::vector<pcl::PointCloud<PointT>::Ptr> key_frames_vec_;

nav_msgs::Path path_optimized;
Eigen::Isometry3d w_odom_curr = Eigen::Isometry3d::Identity();
std::vector<int> matched_frame_id;

bool potentialLoopFlag = false;
double timeLaserOdometry = 0.0, timeSaveFirstCurrentScanForLoopClosure = 0.0;
bool aLoopIsClosed = false;
int closestHistoryFrameID = -1;
int current_frame_id_ = -1;

pcl::KdTreeFLANN<PointT>::Ptr kdtreeCloudFromMap;

pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurroundingKeyPoses;
pcl::KdTreeFLANN<PointT>::Ptr kdtreeHistoryKeyPoses;

pcl::PointCloud<PointT>::Ptr nearHistoryCloudKeyFrameCloud;

pcl::PointCloud<PointT>::Ptr latestKeyFrameCloud;
pcl::PointCloud<PointT>::Ptr latestKeyFrameCloudDS;

pcl::PointCloud<PointT>::Ptr latest_frame_cloud_;
std::vector<ros::Time> time_arr_;
pcl::PointCloud<PointT>::Ptr cloud_keyposes_3d_;
std::vector<Eigen::Isometry3d> odom_arr_;

bool newLaserOdometry = false;
std::mutex mtx;

pcl::KdTreeFLANN<PointT>::Ptr kdtreeGlobalMap;
pcl::PointCloud<PointT>::Ptr globalMapKeyPoses;
pcl::PointCloud<PointT>::Ptr globalMapKeyPosesDS;
pcl::PointCloud<PointT>::Ptr globalMapKeyFrames;
pcl::PointCloud<PointT>::Ptr globalMapKeyFramesDS;

#endif //SRC_XCHU_MAPPING_SRC_MAP_OPTIMIZE_H_
