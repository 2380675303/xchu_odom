//
// Created by xchu on 2021/4/16.
//

#include "xchu_mapping/map_optimize.h"

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

  // high frequence publish
  Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
  odom_to_base.rotate(Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                         msg->pose.pose.orientation.x,
                                         msg->pose.pose.orientation.y,
                                         msg->pose.pose.orientation.z));
  odom_to_base.pretranslate(Eigen::Vector3d(msg->pose.pose.position.x,
                                            msg->pose.pose.position.y,
                                            msg->pose.pose.position.z));
  Eigen::Isometry3d w_curr = w_odom_curr * odom_to_base;
  Eigen::Quaterniond q_temp(w_curr.rotation());

  static tf::TransformBroadcaster br_realtime;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(w_curr.translation().x(), w_curr.translation().y(), w_curr.translation().z()));
  tf::Quaternion q(q_temp.x(), q_temp.y(), q_temp.z(), q_temp.w());
  transform.setRotation(q);
  br_realtime.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom_final"));

  // publish odometry
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = "/map"; //world
  laserOdometry.child_frame_id = "/velo_link"; //odom
  laserOdometry.header.stamp = msg->header.stamp;
  laserOdometry.pose.pose.orientation.x = q_temp.x();
  laserOdometry.pose.pose.orientation.y = q_temp.y();
  laserOdometry.pose.pose.orientation.z = q_temp.z();
  laserOdometry.pose.pose.orientation.w = q_temp.w();
  laserOdometry.pose.pose.position.x = w_curr.translation().x();
  laserOdometry.pose.pose.position.y = w_curr.translation().y();
  laserOdometry.pose.pose.position.z = w_curr.translation().z();
  odom_pub.publish(laserOdometry);
}

//receive all point cloud
void loopCb(const xchu_mapping::LoopInfoConstPtr &msg) {
  mutex_lock.lock();
  loop_queue_.push(msg);
  mutex_lock.unlock();
}

gtsam::Pose3 EigenToPose3(const Eigen::Isometry3d &pose_eigen) {
  Eigen::Quaterniond q(pose_eigen.rotation());
  return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()),
                      gtsam::Point3(pose_eigen.translation().x(),
                                    pose_eigen.translation().y(),
                                    pose_eigen.translation().z()));
}

Eigen::Isometry3d Pose3ToEigen(const gtsam::Pose3 &pose3) {
  Eigen::Isometry3d pose_eigen = Eigen::Isometry3d::Identity();
  gtsam::Quaternion q_temp = pose3.rotation().toQuaternion();
  pose_eigen.rotate(Eigen::Quaterniond(q_temp.w(), q_temp.x(), q_temp.y(), q_temp.z()));
  pose_eigen.pretranslate(Eigen::Vector3d(pose3.translation().x(), pose3.translation().y(), pose3.translation().z()));
  return pose_eigen;

}

void Process() {
  // gtsam　参数设置
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 *isam = new ISAM2(parameters);
  NonlinearFactorGraph gtSAMgraph;
  Values initialEstimate;

  gtsam::Vector Vector6(6);
  Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  priorModel = noiseModel::Diagonal::Variances(Vector6);
  odomModel = noiseModel::Diagonal::Variances(Vector6);

  while (1) {
    // 时间戳对齐
    if (!loop_queue_.empty() && !cloud_queue_.empty() && !odom_queue_.empty()) {
      mutex_lock.lock();
      double time1 = loop_queue_.front()->header.stamp.toSec();
      double time2 = odom_queue_.front()->header.stamp.toSec();
      double time3 = cloud_queue_.front()->header.stamp.toSec();
      if (!loop_queue_.empty() && (time1 < time2 - 0.5 * 0.1 || time1 < time3 - 0.5 * 0.1)) {
        ROS_WARN("time stamp unaligned error and loopInfoBuf discarded, pls check your data -->  optimization");
        loop_queue_.pop();
        mutex_lock.unlock();
        continue;
      }
      if (!odom_queue_.empty() && (time2 < time1 - 0.5 * 0.1 || time2 < time3 - 0.5 * 0.1)) {
        ROS_WARN("time stamp unaligned error and odometryBuf discarded, pls check your data -->  optimization");
        odom_queue_.pop();
        mutex_lock.unlock();
        continue;
      }
      if (!cloud_queue_.empty() && (time3 < time1 - 0.5 * 0.1 || time3 < time2 - 0.5 * 0.1)) {
        ROS_WARN("time stamp unaligned error and pointCloudEdgeBuf discarded, pls check your data -->  optimization");
        cloud_queue_.pop();
        mutex_lock.unlock();
        continue;
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*cloud_queue_.front(), *pointcloud_in);
      ros::Time pointcloud_time = (cloud_queue_.front())->header.stamp;
      timeLaserOdometry = pointcloud_time.toSec();
      // odom数据
      Eigen::Isometry3d odom_in = Eigen::Isometry3d::Identity();
      odom_in.rotate(Eigen::Quaterniond(odom_queue_.front()->pose.pose.orientation.w,
                                        odom_queue_.front()->pose.pose.orientation.x,
                                        odom_queue_.front()->pose.pose.orientation.y,
                                        odom_queue_.front()->pose.pose.orientation.z));
      odom_in.pretranslate(Eigen::Vector3d(odom_queue_.front()->pose.pose.position.x,
                                           odom_queue_.front()->pose.pose.position.y,
                                           odom_queue_.front()->pose.pose.position.z));

      // get loop result
      xchu_mapping::LoopInfoConstPtr loop_msg = loop_queue_.front();
      int current_frame_id = loop_queue_.front()->current_id;
      int match_id = -1;
      bool is_loop = false;
      double loop_score = 0.0;
      Eigen::Matrix4d loop_transform = Eigen::Matrix4d::Identity();
      if (loop_msg->matched_id.size() == 1) {
        is_loop = true;
        loop_score = loop_msg->score;

        match_id = loop_msg->matched_id.front();
        loop_transform(0, 0) = loop_msg->transform[0];
        loop_transform(0, 1) = loop_msg->transform[1];
        loop_transform(0, 2) = loop_msg->transform[2];
        loop_transform(1, 0) = loop_msg->transform[3];
        loop_transform(1, 1) = loop_msg->transform[4];
        loop_transform(1, 2) = loop_msg->transform[5];
        loop_transform(2, 0) = loop_msg->transform[6];
        loop_transform(2, 1) = loop_msg->transform[7];
        loop_transform(2, 2) = loop_msg->transform[8];

        loop_transform(0, 3) = loop_msg->transform[9];
        loop_transform(1, 3) = loop_msg->transform[10];
        loop_transform(2, 3) = loop_msg->transform[11];
      }

      cloud_queue_.pop();
      odom_queue_.pop();
      loop_queue_.pop();
      mutex_lock.unlock();

      // 有回环时执行回环检测, 无回环时只添加位姿图节点
      if (current_frame_id != (int) key_frames_vec_.size()) {
        ROS_WARN_ONCE("graph optimization frame not aligned, pls check your data");
      }

      // 位子图添加节点
//      gtsam::Pose3 pose3_current = EigenToPose3(odom_in);  // se3 eigen转gtsam
      if (cloud_keyposes_3d_->empty()) {
        //ROS_INFO("---------------------------add first pose-----------------------");
        // 因子图中添加一元因子, 系统先验, back是最新, front是醉酒
//        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, pose3_current, priorModel));
//        initialEstimate.insert(0, pose3_current);  // 初值容器初始化

        //保存初始点信息，为先验因子。下面操作是把四元数转换为旋转矩阵，旋转矩阵变化为欧拉角
        gtsam::Pose3 curr_pose = EigenToPose3(map_curr_);
        initialEstimate.insert(temp_laserCloudMap_Ind + 1, curr_pose);
        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
            (Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished());
        gtSAMgraph.add(PriorFactor<Pose3>(1, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, 0, 0)), priorNoise));

      } else {
        // 世界坐标系下的位姿
        map_curr_ = map_odom_curr_ * odom_in;

        // odom_original_arr pose_optimized_arr中分别存储odom位姿和优化后的odom位姿
        /*  odom_original_arr.push_back(last_pose3.between(pose3_current)); // 存相对变换
          // 因子图中加入二元因子,相邻节点之间边的约束，通过相邻帧边缘点来关联
          gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(key_frames_vec_.size() - 1,
                                                            key_frames_vec_.size(),
                                                            odom_original_arr.back(),
                                                            odomModel));
          // initialEstimate.insert(key_frames_vec_.size(), pose_optimized_arr.back()); // 加入一个变量
          initialEstimate.insert(key_frames_vec_.size(), pose3_current); // 加入一个变量*/

        //保存当前位姿和里程计变换的信息到gtsam的graph，
        gtsam::Pose3 last_pose3_ = EigenToPose3(map_curr_last_);
        gtsam::Pose3 curr_pose3_ = EigenToPose3(map_curr_);
        initialEstimate.insert(temp_laserCloudMap_Ind + 1, curr_pose3_);
        //gtsam提供一种容易求两点之间位姿变化的函数between，用于里程计变化信息的录入
        noiseModel::Diagonal::shared_ptr
            odom_noise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtSAMgraph.add(BetweenFactor<Pose3>(temp_laserCloudMap_Ind,
                                            temp_laserCloudMap_Ind + 1,
                                            last_pose3_.between(curr_pose3_),
                                            odom_noise));


        // 会还
      /*  if (is_loop && match_id != -1) {
          //获取变换矩阵，求四元数q和位移t_vector
          Eigen::Matrix3d rotation = loop_transform.block<3, 3>(0, 0);
          Eigen::Vector3d t = loop_transform.block<3, 1>(0, 3);
          Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
          transform.rotate(rotation);
          transform.pretranslate(t);

          //位姿变换
          Eigen::Isometry3d temp_curr = map_curr_ * transform;

          //发生闭环，添加闭环信息到gtsam的graph，闭环noise使用icp得分
          Values isamCurrentEstimate1;
          Pose3 closeEstimate;
          isamCurrentEstimate1 = isam->calculateEstimate();
          closeEstimate = isamCurrentEstimate1.at<Pose3>(match_id);

          gtsam::Pose3 loop_temp = EigenToPose3(temp_curr);
          noiseModel::Diagonal::shared_ptr closure_noise = noiseModel::Diagonal::Variances((Vector(6)
              << loop_score, loop_score, loop_score, loop_score, loop_score, loop_score).finished());
          gtSAMgraph.add(BetweenFactor<Pose3>(match_id,
                                              temp_laserCloudMap_Ind + 1,
                                              closeEstimate.between(loop_temp),
                                              closure_noise));
        }*/
      }

      //每次循环后进行gtsam迭代
      isam->update(gtSAMgraph, initialEstimate);
      isam->update();
      //获取优化后的所有位姿
      Pose3 currEstimate;
      Values isamCurrentEstimate2;
      isamCurrentEstimate2 = isam->calculateEstimate();
      //获取当前帧的位姿，保存在当前位姿里
      currEstimate = isamCurrentEstimate2.at<Pose3>(temp_laserCloudMap_Ind + 1);
      map_curr_ = Pose3ToEigen(currEstimate);
      map_odom_curr_ = map_curr_ * odom_in.inverse();


      //清空原来的约束。已经加入到isam2的那些会用bayes tree保管
      gtSAMgraph.resize(0);
      initialEstimate.clear();
      //当前帧赋值给历史帧，用于之后里程计信息添加到gtsam的边
      map_curr_last_ = map_curr_;
      odom_curr_last_ = odom_in;
      map_odom_curr_last_ = map_odom_curr_;

//      q_w_last = q_w_curr;
//      t_w_last = t_w_curr;

      //记录当前帧保存后的点数便于之后查询，自加帧数
//      laserCloudMap_Ind[temp_laserCloudMap_Ind] = laserCloudMap->points.size();
      temp_laserCloudMap_Ind++;

      //保存位姿，便于之后位姿KD树搜索
      PointT point;
      point.x = map_curr_.translation().x();
      point.y = map_curr_.translation().y();
      point.z = map_curr_.translation().z();
      point.intensity = cloud_keyposes_3d_->points.size();
      cloud_keyposes_3d_->push_back(point);

//      isam->update(gtSAMgraph, initialEstimate);
//      isam->update();
//      gtSAMgraph.resize(0);
//      initialEstimate.clear(); // 清空初值
//
//      // 优化之后的pose
//      Pose3 latestEstimate;
//      isamCurrentEstimate = isam->calculateEstimate();
//      pose3_current = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
//
//      time_arr_.push_back(pointcloud_time);
//      key_frames_vec_.push_back(pointcloud_in);
//      pose_optimized_arr.push_back(pose3_current);
//      odom_arr_.push_back(Pose3ToEigen(pose3_current)); // 原始odom
//
//      // update pose
//      last_pose3 = pose3_current;
//      PointT point;
//      point.x = pose3_current.x();
//      point.y = pose3_current.y();
//      point.z = pose3_current.z();
//      point.intensity = cloud_keyposes_3d_->points.size();
//      cloud_keyposes_3d_->push_back(point);
//
//      // publish pose
//      Eigen::Isometry3d pose_current = Pose3ToEigen(pose_optimized_arr.back());
//      Eigen::Quaterniond q_current(pose_current.rotation());
//      Eigen::Vector3d t_current = pose_current.translation();
//      w_odom_curr = pose_current * odom_in.inverse();

      /*
      static tf::TransformBroadcaster br_realtime;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(pose3_current.translation().x(),
                                      pose3_current.translation().y(),
                                      pose3_current.translation().z()));
      tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
      transform.setRotation(q);
      br_realtime.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom_final"));

      // publish odometry
      nav_msgs::Odometry laserOdometry;
      laserOdometry.header.frame_id = "/map"; //world
      laserOdometry.child_frame_id = "/odom_final"; //odom
      laserOdometry.header.stamp = pointcloud_time;
      laserOdometry.pose.pose.orientation.x = q_current.x();
      laserOdometry.pose.pose.orientation.y = q_current.y();
      laserOdometry.pose.pose.orientation.z = q_current.z();
      laserOdometry.pose.pose.orientation.w = q_current.w();
      laserOdometry.pose.pose.position.x = pose_current.translation().x();
      laserOdometry.pose.pose.position.y = pose_current.translation().y();
      laserOdometry.pose.pose.position.z = pose_current.translation().z();
      odom_pub.publish(laserOdometry);*/

      // 发布关键帧位姿
      if (optimize_poses_pub.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud_keyposes_3d_, msg);
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        optimize_poses_pub.publish(msg);
      }
    }
    //sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }//end of while(1)
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "iscOptimization");
  ros::NodeHandle nh("~");

  // 里程计不断优化, 有回环则执行优化

  // 订阅odom节点
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/current_odom", 10, OdomCb);
  ros::Subscriber points_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_points", 10, PointCb);
  ros::Subscriber loop_sub = nh.subscribe<xchu_mapping::LoopInfo>("/loop_closure", 10, loopCb); // 在isc节点中检测回环

  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_final", 100);
  //origin_odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_before", 100);
  // path_pub = nh.advertise<nav_msgs::Path>("/final_path", 100);
  pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
  optimize_poses_pub = nh.advertise<sensor_msgs::PointCloud2>("/final_poses", 1);  // 关键帧点云

  downSizeFilterKeyFrames.setLeafSize(0.5, 0.5, 0.5); // 发布全局地图的采样size,设置小了导致系统卡顿
  downSizeFilterGlobalMapKeyFrames.setLeafSize(1.0, 1.0, 1.0); // 发布全局地图的采样size,设置小了导致系统卡顿

  // 初始化一些参数，包括gtsam噪声模型
  key_frames_vec_.clear();

  nearHistoryCloudKeyFrameCloud.reset(new pcl::PointCloud<PointT>());
  cloud_keyposes_3d_.reset(new pcl::PointCloud<PointT>());

  latestKeyFrameCloud.reset(new pcl::PointCloud<PointT>());
  latestKeyFrameCloudDS.reset(new pcl::PointCloud<PointT>());
  latest_frame_cloud_.reset(new pcl::PointCloud<PointT>());

  kdtreeCloudFromMap.reset(new pcl::KdTreeFLANN<PointT>());
  kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointT>());
  kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointT>());

  kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointT>());
  globalMapKeyPoses.reset(new pcl::PointCloud<PointT>());
  globalMapKeyPosesDS.reset(new pcl::PointCloud<PointT>());
  globalMapKeyFrames.reset(new pcl::PointCloud<PointT>());
  globalMapKeyFramesDS.reset(new pcl::PointCloud<PointT>());

  std::thread global_optimization{Process};

  // 新开一个线程, 一较低频率执行优化




  ros::spin();

  return 0;
}