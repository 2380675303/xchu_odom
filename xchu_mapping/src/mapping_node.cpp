//
// Created by xchu on 2021/4/13.
//

#include <pcl_ros/transforms.h>
#include "xchu_mapping/mapping_node.h"

/*MappingThread::MappingThread(ros::NodeHandle &nh) {

  globalmap_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  transformed_scan_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());

  // 订阅odom节点见图
  odom_sub = nh.subscribe("/current_odom", 50, &MappingThread::OdomCb, this);
  points_sub = nh.subscribe("/kitti/velo/pointcloud", 10, &MappingThread::PointCb, this);

}*/

void PointCb(const sensor_msgs::PointCloud2ConstPtr &msg) {
//  ROS_INFO("GET CLOUD DATA %d...................................", cloud_queue.size());
  // 栈来接受点云
  mutex_lock.lock();
  cloud_queue.push(msg);
  mutex_lock.unlock();
}

void OdomCb(const nav_msgs::Odometry::ConstPtr &msg) {
//  ROS_INFO("GET ODOM DATA %d...................................", odom_queue.size());
  mutex_lock.lock();
  odom_queue.push(msg);
  mutex_lock.unlock();
}

/*
void MappingThread::Run() {

  if (!cloud_queue.empty() && !odom_queue.empty()) {
    mutex_lock.lock();
    while (!odom_queue.empty() && odom_queue.front()->header.stamp.toSec() < cloud_queue.front()->header.stamp.toSec())
      odom_queue.pop();
    if (odom_queue.empty()) {
      mutex_lock.unlock();
      break;
    }
  }

  ros::Time odom_time = input.header.stamp;
  // 时间对齐
  if (cloud_queue.empty()) {
    ROS_WARN("Waiting cloud...");
    return;
  }

  std::cout << "size: " << cloud_queue.size() << std::endl;
  mutex_lock.lock();
  pcl::PointCloud<pcl::PointXYZI> tmp;
  pcl::fromROSMsg(cloud_queue.front(), tmp);
  ros::Time current_scan_time = (cloud_queue.front()).header.stamp;
  double time_diff = current_scan_time.toSec() - odom_time.toSec();

  while (!cloud_queue.empty() && cloud_queue.front().header.stamp.toSec() < odom_time.toSec())
    cloud_queue.pop();

  std::cout << "time diff: " << time_diff << std::endl;

//  cloud_queue.pop(); // pop掉
  mutex_lock.unlock();

  // 取每一帧对应的pose, 并讲点云拼接起来
  //*globalmap_ptr += *transformed_scan_ptr;
}
*/

void Process() {
  int i = 1;
  while (i > 0) {
//    ROS_INFO("DATA SIZE %d, %d ...................................", cloud_queue.size(), odom_queue.size());

    if (!cloud_queue.empty() && !odom_queue.empty()) {

      mutex_lock.lock();
      if (!odom_queue.empty() && odom_queue.front()->header.stamp.toSec()
          < cloud_queue.front()->header.stamp.toSec() - 0.5 * 0.1) {
        double time_diff = cloud_queue.front()->header.stamp.toSec() - odom_queue.front()->header.stamp.toSec();
        ROS_INFO("DATA SIZE %d, %d, %f ...................................",
                 cloud_queue.size(),
                 odom_queue.size(),
                 time_diff);
        ROS_WARN(
            "time stamp unaligned error and odom discarded, pls check your data; odom time %f, pc time %f",
            odom_queue.front()->header.stamp.toSec(),
            cloud_queue.front()->header.stamp.toSec());
        odom_queue.pop();
        mutex_lock.unlock();
        continue;
      }

      if (!cloud_queue.empty() && cloud_queue.front()->header.stamp.toSec()
          < odom_queue.front()->header.stamp.toSec() - 0.5 * 0.1) {
        ROS_WARN(
            "time stamp unaligned error and odom discarded, pls check your data; odom time %f, pc time %f",
            odom_queue.front()->header.stamp.toSec(),
            cloud_queue.front()->header.stamp.toSec());
        cloud_queue.pop();
        mutex_lock.unlock();
        continue;
      }

      ROS_INFO("align data success------------------");

      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*cloud_queue.front(), *pointcloud_in); // 最新的点云帧
      ros::Time pointcloud_time = (cloud_queue.front())->header.stamp;
      Eigen::Isometry3d odom_in = Eigen::Isometry3d::Identity(); // 最新的odom
      odom_in.rotate(Eigen::Quaterniond(odom_queue.front()->pose.pose.orientation.w,
                                        odom_queue.front()->pose.pose.orientation.x,
                                        odom_queue.front()->pose.pose.orientation.y,
                                        odom_queue.front()->pose.pose.orientation.z));
      odom_in.pretranslate(Eigen::Vector3d(odom_queue.front()->pose.pose.position.x,
                                           odom_queue.front()->pose.pose.position.y,
                                           odom_queue.front()->pose.pose.position.z));
      odom_queue.pop();
      cloud_queue.pop();
      mutex_lock.unlock();

      // 先拼接地图吧
      pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      downSizeFilterKeyFrames.setInputCloud(pointcloud_in);
      downSizeFilterKeyFrames.filter(*temp_cloud_ptr);
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(*temp_cloud_ptr, *transformed_scan_ptr, odom_in.cast<float>());  // tf_btol为初始变换矩阵
      *globalmap_ptr += *transformed_scan_ptr;

      pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
     /* if (i % 100 == 0) {
        downSizeFilterGlobalMap.setInputCloud(globalmap_ptr);
        downSizeFilterGlobalMap.filter(*map_cloud_ptr);
      }
      i++;*/

      if (final_map_pub.getNumSubscribers() > 0) {

        downSizeFilterGlobalMap.setInputCloud(globalmap_ptr);
        downSizeFilterGlobalMap.filter(*map_cloud_ptr);

        sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*map_cloud_ptr, *map_msg_ptr);
        map_msg_ptr->header.frame_id = "map";
        final_map_pub.publish(*map_msg_ptr);
      }
      ROS_INFO("Update global map %d", globalmap_ptr->size());
    }

    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapping_node");
  ros::NodeHandle nh("~");

  // 订阅odom节点见图
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/current_odom", 10, OdomCb);
  ros::Subscriber points_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_points", 10, PointCb);
  final_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 10);

  downSizeFilterGlobalMap.setLeafSize(2.0, 2.0, 2.0); // 保存全局地图时下采样size，可以设置小一些方便看清楚点云细节
  globalmap_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  downSizeFilterKeyFrames.setLeafSize(1.0, 1.0, 1.0); // 发布全局地图的采样size,设置小了导致系统卡顿

  std::thread mapping_process{Process};

  ros::spin();

  return 0;
}