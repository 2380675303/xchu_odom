//
// Created by xchu on 2021/4/20.
//

#ifndef SRC_XCHU_MAPPING_SRC_ODOM_OPTIMIZE_GTSAM_H_
#define SRC_XCHU_MAPPING_SRC_ODOM_OPTIMIZE_GTSAM_H_

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
#include <std_srvs/Empty.h>

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
#include <pcl/common/transforms.h>

#include <xchu_mapping/LoopInfo.h>

using namespace gtsam;
using PointT = pcl::PointXYZI;

ros::Publisher odom_pub, path_pub, pubLaserCloudSurround, optimize_poses_pub, origin_odom_pub;
ros::Publisher pub_keyposes_, pub_globalmap_, pub_history_keyframes_, pub_recent_keyframes_;
ros::ServiceServer srv_save_map_;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterKeyFrames, downSizeFilterHistoryKeyFrames; // for global map visualization

std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue_;
std::queue<nav_msgs::Odometry::ConstPtr> odom_queue_;
std::queue<xchu_mapping::LoopInfo::ConstPtr> loop_queue_;

ISAM2 *isam;
NonlinearFactorGraph gtSAMgraph;
Values initialEstimate;
Values isam_current_estimate_;

std::string save_dir_ = "/home/xchu/xchujwu_slam/src/xchu_mapping/";

/*NonlinearFactorGraph gtSAMgraph;
Values initialEstimate;
Values optimizedEstimate;
ISAM2 *isam;
Values isamCurrentEstimate;*/

std::vector<gtsam::Pose3> pose_optimized_arr; // 优化后的pose
std::vector<Eigen::Isometry3d> cloud_keyposes_6d_; // 优化后的pose
std::vector<gtsam::Pose3> odom_original_arr;  // odom先验
gtsam::Pose3 last_pose3, curr_pose3;
Eigen::Isometry3d pre_key_odom_ = Eigen::Isometry3d::Identity(); //map
Eigen::Isometry3d current_odom_ = Eigen::Isometry3d::Identity(); //map

std::vector<ros::Time> key_time_arr_;
ros::Time pre_key_time_;

std::deque<pcl::PointCloud<PointT>::Ptr> recent_keyframes_;
int latest_frame_id_ = -1;
int closest_history_frame_id_ = -1;
pcl::PointCloud<PointT>::Ptr latest_keyframe_;
pcl::PointCloud<PointT>::Ptr near_history_keyframes_;
std::mutex mtx_;
pcl::KdTreeFLANN<PointT>::Ptr kdtree_poses_;
pcl::PointCloud<PointT>::Ptr globalMap_keyPoses_;
pcl::PointCloud<PointT>::Ptr localmap_frames_;
pcl::PointCloud<PointT>::Ptr localmap_cloud_;
pcl::PointCloud<PointT>::Ptr localmap_pose_;
pcl::PointCloud<PointT>::Ptr global_map_cloud_;

PointT cur_point, pre_point; // current position



float history_search_radius_ = 10; // 回环检测参数
int history_search_num_ = 20;
float history_fitness_score_ = 0.3;

ros::Time pointcloud_time;
double timeSaveFirstCurrentScanForLoopClosure;

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

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterGlobalMapKeyFrames; // for global map visualization


noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odometryNoise;
noiseModel::Diagonal::shared_ptr constraintNoise;

std::vector<pcl::PointCloud<PointT>::Ptr> key_frames_vec_;

nav_msgs::Path path_optimized;
Eigen::Isometry3d w_odom_curr = Eigen::Isometry3d::Identity();
std::vector<int> matched_frame_id;

bool potentialLoopFlag = false;
//double timeLaserOdometry = 0.0, timeSaveFirstCurrentScanForLoopClosure = 0.0;
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

std::mutex mtx;

//pcl::KdTreeFLANN<PointT>::Ptr kdtreeGlobalMap;
//pcl::PointCloud<PointT>::Ptr globalMapKeyPoses;
//pcl::PointCloud<PointT>::Ptr globalMapKeyPosesDS;
//pcl::PointCloud<PointT>::Ptr globalMapKeyFrames;
//pcl::PointCloud<PointT>::Ptr globalMapKeyFramesDS;

bool loop_closure_enabled_ = true;
bool loop_closed_ = false;

#endif //SRC_XCHU_MAPPING_SRC_ODOM_OPTIMIZE_GTSAM_H_
