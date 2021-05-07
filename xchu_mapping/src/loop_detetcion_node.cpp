//
// Created by xchu on 2021/4/15.
//

#include <pcl/common/transforms.h>
#include "xchu_mapping/loop_detetcion_node.h"

void PointCb(const sensor_msgs::PointCloud2ConstPtr &msg) {
  mutex_lock.lock();
  cloud_queue_.push(msg);
  mutex_lock.unlock();
}

//receive odomtry
void OdomCb(const nav_msgs::Odometry::ConstPtr &msg) {
  mutex_lock.lock();
  odom_queue_.push(msg);
  mutex_lock.unlock();
}

void Process() {
  while (1) {
    // odom和point时间对齐
    if (!cloud_queue_.empty() && !odom_queue_.empty()) {
      //align time stamp
      mutex_lock.lock();
      if (!odom_queue_.empty() && odom_queue_.front()->header.stamp.toSec()
          < cloud_queue_.front()->header.stamp.toSec() - 0.5 * 0.1) {
        ROS_WARN(
            "loop_detetcion: time stamp unaligned error and odom discarded, pls check your data; odom time %f, pc time %f",
            odom_queue_.front()->header.stamp.toSec(),
            cloud_queue_.front()->header.stamp.toSec());
        odom_queue_.pop();
        mutex_lock.unlock();
        continue;
      }

      if (!cloud_queue_.empty() && cloud_queue_.front()->header.stamp.toSec()
          < odom_queue_.front()->header.stamp.toSec() - 0.5 * 0.1) {
        ROS_WARN(
            "loop_detetcion: time stamp unaligned error and odom discarded, pls check your data; odom time %f, pc time %f",
            odom_queue_.front()->header.stamp.toSec(),
            cloud_queue_.front()->header.stamp.toSec());
        cloud_queue_.pop();
        mutex_lock.unlock();
        continue;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*cloud_queue_.front(), *pointcloud_in); // 最新的点云帧
      ros::Time pointcloud_time = (cloud_queue_.front())->header.stamp;
      Eigen::Isometry3d odom_in = Eigen::Isometry3d::Identity(); // 最新的odom
      odom_in.rotate(Eigen::Quaterniond(odom_queue_.front()->pose.pose.orientation.w,
                                        odom_queue_.front()->pose.pose.orientation.x,
                                        odom_queue_.front()->pose.pose.orientation.y,
                                        odom_queue_.front()->pose.pose.orientation.z));
      odom_in.pretranslate(Eigen::Vector3d(odom_queue_.front()->pose.pose.position.x,
                                           odom_queue_.front()->pose.pose.position.y,
                                           odom_queue_.front()->pose.pose.position.z));
      odom_queue_.pop();
      cloud_queue_.pop();
      mutex_lock.unlock();

      Eigen::Vector3d current_t = odom_in.translation(); // 当前odom位置

      // 进行回环检测
      //near_history_frame_cloud_->clear();

      // current_pose 对应的是全局下的坐标!
      PointT this_pose_3d;
      PointTypePose this_pose_6d;

      this_pose_3d.x = current_t[0];
      this_pose_3d.y = current_t[1];
      this_pose_3d.z = 0;
      // 强度字段表示pose的index

      this_pose_3d.intensity = cloud_keyposes_3d_->points.size();
      cloud_keyposes_3d_->points.push_back(this_pose_3d);
      key_frames_vec_.push_back(pointcloud_in);

      // 拼接near_cloud
      if (travel_distance_arr_.size() == 0) {
        travel_distance_arr_.push_back(0);
      } else {
        // 这里是计算里程距离？
        double dis_temp = travel_distance_arr_.back() + std::sqrt((pos_arr_.back() - current_t).array().square().sum());
        travel_distance_arr_.push_back(dis_temp);
      }
      pos_arr_.push_back(current_t); // odom装到这里
      odom_arr_.push_back(odom_in); // isc特征全部装到这里面
      time_arr_.push_back(pointcloud_time);
//      ROS_INFO("-----------------------detetct loop ------------------------ ");

      std::vector<int> pointSearchIndLoop;
      std::vector<float> pointSearchSqDisLoop;
      kdtree_history_key_poses_->setInputCloud(cloud_keyposes_3d_);
      kdtree_history_key_poses_->radiusSearch(this_pose_3d, 10.0, pointSearchIndLoop, pointSearchSqDisLoop, 0);

      int current_frame_id = pos_arr_.size() - 1;

      //ROS_INFO("-----------------------pointSearchIndLoop size %d ------------------------ ", pointSearchIndLoop.size());

      // 寻找候选帧
      matched_frame_id.clear();
      int closestHistoryFrameID = -1;
      int curMinID = 1000000;
      for (int i = 0; i < pointSearchIndLoop.size(); ++i) {
        int id = pointSearchIndLoop[i];

        if (abs(time_arr_[id].toSec() - pointcloud_time.toSec()) > 30.0 /*&& travel_distance_arr_.back() > 15*/) {
          //closestHistoryFrameID = id;

          //if (id < curMinID) {
          //curMinID = id;
//            closestHistoryFrameID = curMinID;
          closestHistoryFrameID = id;
          matched_frame_id.push_back(id);
          //std::cout << "-------------loop time: " << time_arr_[id].toSec() - pointcloud_time.toSec()
//                    << std::endl;
          //}
        }
      }
      // ROS_INFO("-----------------------pointSearchIndLoop after size %d, %d, %d, %d------------------------ ",
//               pointSearchIndLoop.size(),
//               matched_frame_id.size(),
//               current_frame_id,
//               closestHistoryFrameID);

      int best_match_id = -1;
      double best_score = 0.0;
      Eigen::VectorXf transform(12);
      transform.setIdentity();

      Eigen::Matrix4f final_trans;
      final_trans.setIdentity();

      if (closestHistoryFrameID == -1) {
        // 回环检测失败　什么都不做

      } else {

        // 提取附近点云,用icp匹配判定是否为回环
        pcl::PointCloud<pcl::PointXYZI>::Ptr frame_cloud_(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*key_frames_vec_[current_frame_id], *frame_cloud_,
                                 odom_arr_[current_frame_id].cast<float>());
        *latest_frame_cloud_ += *frame_cloud_;
/*        pcl::PointCloud<PointT>::Ptr hahaCloud(new pcl::PointCloud<PointT>());
        int cloudSize = latest_frame_cloud_->points.size();
        for (int i = 0; i < cloudSize; ++i) {
          if ((int) latest_frame_cloud_->points[i].intensity >= 0) {
            hahaCloud->push_back(latest_frame_cloud_->points[i]);
          }
        }
        latest_frame_cloud_->clear();
        *latest_frame_cloud_ = *hahaCloud;*/

        downSizeFilterHistoryKeyFrames.setInputCloud(latest_frame_cloud_);
        downSizeFilterHistoryKeyFrames.filter(*latest_frame_cloud_);


//#pragma omp parallel for
//        for (int i = 0; i < matched_frame_id.size(); ++i) {
//          int match_id = matched_frame_id[i];
        int match_id = closestHistoryFrameID;

        near_history_frame_cloud_->clear();
        for (int j = -15; j <= historyKeyframeSearchNum; ++j) {
          if (match_id + j < 0 || match_id + j > current_frame_id)
            continue;
          pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_(new pcl::PointCloud<pcl::PointXYZI>());
          pcl::transformPointCloud(*key_frames_vec_[match_id + j],
                                   *temp_cloud_,
                                   odom_arr_[match_id + j].cast<float>());
          *near_history_frame_cloud_ += *temp_cloud_;
        }

        downSizeFilterHistoryKeyFrames.setInputCloud(near_history_frame_cloud_);
        downSizeFilterHistoryKeyFrames.filter(*near_history_frame_cloud_);

//          ROS_INFO("Latest cloud size: %d, %d ", latest_frame_cloud_->size(), near_history_frame_cloud_->size());

        if (latest_frame_cloud_->empty() || near_history_frame_cloud_->empty()) {
          ROS_ERROR("loop: Empty cloud.....");
          continue;
        }

        icp.setInputSource(latest_frame_cloud_);
        icp.setInputTarget(near_history_frame_cloud_);
        pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
        icp.align(*unused_result);

        double score = icp.getFitnessScore();
        if (icp.hasConverged() && score < 0.3) {
          best_match_id = match_id;
          best_score = score;
          final_trans = icp.getFinalTransformation();
          ROS_WARN("loop detect success %d, %d", current_frame_id, match_id);
          break;
        }
      }
      near_history_frame_cloud_->clear();
//      }
      latest_frame_cloud_->clear();

      // 继续寻找best match i
      //ROS_INFO("detetct loop success %d, %d: ", current_frame_id, matched_frame_id.size());
      // 发布回环检测的结果
      xchu_mapping::LoopInfo loop;
      loop.header.stamp = pointcloud_time;
      loop.header.frame_id = "velo_link";
      loop.current_id = current_frame_id;
      if (best_match_id != -1) {
        loop.matched_id.push_back(best_match_id);
        loop.score = best_score;

        transform[0] = final_trans(0, 0);
        transform[1] = final_trans(0, 1);
        transform[2] = final_trans(0, 2);
        transform[3] = final_trans(1, 0);
        transform[4] = final_trans(1, 1);
        transform[5] = final_trans(1, 2);
        transform[6] = final_trans(2, 0);
        transform[7] = final_trans(2, 1);
        transform[8] = final_trans(2, 2);

        transform[9] = final_trans(0, 3);
        transform[10] = final_trans(1, 3);
        transform[11] = final_trans(2, 3);

        //std::cout << "transform1: " << final_trans << std::endl;

        for (int kI = 0; kI < transform.size(); ++kI) {
          loop.transform.push_back(transform[kI]);
        }

      } /*else {
        for (int i = 0; i < matched_frame_id.size(); i++) {
          loop.matched_id.push_back(matched_frame_id[i]);
        }
      }*/

      loop_info_pub.publish(loop);
    }
    //sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "filter_node");
  ros::NodeHandle nh;

  // 订阅odom节点
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/current_odom", 10, OdomCb);
  ros::Subscriber points_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_points", 10, PointCb);

  // 检测回环，并发布实时的isc图像
  loop_info_pub = nh.advertise<xchu_mapping::LoopInfo>("/loop_closure", 100);

/*  final_ground_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 32);
  non_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/no_ground_points", 32);
  normal_ground_pub = nh.advertise<sensor_msgs::PointCloud2>("/normal_ground_points", 32);
  floor_pub = nh.advertise<xchu_mapping::FloorCoeffs>("/ground_coeffs", 32);*/

//  downSizeFilterGlobalMap.setLeafSize(1.0, 1.0, 1.0); // 保存全局地图时下采样size，可以设置小一些方便看清楚点云细节
//  globalmap_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
//  transformed_scan_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());

  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  downSizeFilterKeyFrames.setLeafSize(0.5, 0.5, 0.5); // 发布全局地图的采样size,设置小了导致系统卡顿
  downSizeFilterHistoryKeyFrames.setLeafSize(1.0, 1.0, 1.0); // 发布全局地图的采样size,设置小了导致系统卡顿

  near_history_frame_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  latest_frame_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  cloud_keyposes_6d_.reset(new pcl::PointCloud<PointTypePose>());
  cloud_keyposes_3d_.reset(new pcl::PointCloud<PointT>());
  kdtree_history_key_poses_.reset(new pcl::KdTreeFLANN<PointT>());

  std::thread loop_closure_detection{Process};

  ros::spin();

  return 0;
}
