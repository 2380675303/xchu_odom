//
// Created by xchu on 2021/4/16.
//

#include "xchu_mapping/odom_optimize_gtsam.h"

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

void publishKeyposesAndFrames() {
  // key pose
  if (pub_keyposes_.getNumSubscribers() == 0 || cloud_keyposes_3d_->empty())
    return;
  if (pub_globalmap_.getNumSubscribers() == 0)
    return;
  if (pub_recent_keyframes_.getNumSubscribers() == 0)
    return;

  // publish key poses
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud_keyposes_3d_, msg);
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  pub_keyposes_.publish(msg);

  // publish localmap
  std::vector<int> search_idx_;
  std::vector<float> search_dist_;
  mtx.lock();
  kdtree_poses_->setInputCloud(cloud_keyposes_3d_);
  // 通过KDTree进行最近邻搜索
  kdtree_poses_->radiusSearch(cur_point, 10.0, search_idx_, search_dist_, 0);
  mtx.unlock();

  for (int i = 0; i < search_idx_.size(); ++i)
    globalMap_keyPoses_->points.push_back(cloud_keyposes_3d_->points[search_idx_[i]]);

  downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMap_keyPoses_);
  downSizeFilterGlobalMapKeyFrames.filter(*localmap_pose_);

  for (int j = 0; j < localmap_pose_->points.size(); ++j) {
    int keyId = localmap_pose_->points[j].intensity;
    pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*key_frames_vec_[keyId], *tmp, cloud_keyposes_6d_[keyId].matrix().cast<float>());
    *localmap_frames_ += *tmp;
  }
  // 对globalMapKeyFrames进行下采样
  downSizeFilterGlobalMapKeyFrames.setInputCloud(localmap_frames_);
  downSizeFilterGlobalMapKeyFrames.filter(*localmap_cloud_);

  sensor_msgs::PointCloud2 cloudMsg;
  pcl::toROSMsg(*localmap_cloud_, cloudMsg);
  cloudMsg.header.stamp = ros::Time::now();
  cloudMsg.header.frame_id = "/map";
  pub_recent_keyframes_.publish(cloudMsg);

  // global map
  int num_points = 0;
  Eigen::Matrix4f T(Eigen::Matrix4f::Identity());
  for (int i = 0; i < key_frames_vec_.size(); ++i) {
    pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*key_frames_vec_[i], *tmp, cloud_keyposes_6d_[i].matrix().cast<float>());
    *global_map_cloud_ += *tmp;
    num_points += tmp->points.size();
  }
  downSizeFilterGlobalMapKeyFrames.setInputCloud(global_map_cloud_);
  downSizeFilterGlobalMapKeyFrames.filter(*global_map_cloud_);

  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(*global_map_cloud_, map_msg);
  map_msg.header.frame_id = "map";
  map_msg.header.stamp = ros::Time::now();
  pub_globalmap_.publish(map_msg);

  globalMap_keyPoses_->clear();
  localmap_cloud_->clear();
  localmap_frames_->clear();
  global_map_cloud_->clear();
}

/**
 * 保存全局地图
 *
 */
void TransformAndSaveMap() {
  std::stringstream ss;
  ss << int(ros::Time::now().toSec());
  std::string stamp = ss.str();

  int num_points = 0, num_points1 = 0;
  pcl::PointCloud<PointT>::Ptr map(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr map_no_ground(new pcl::PointCloud<PointT>());
  for (int i = 0; i < key_frames_vec_.size(); ++i) {
    pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*key_frames_vec_[i], *tmp, cloud_keyposes_6d_[i].matrix().cast<float>());

    //tmp = transformPointCloud(key_frames_vec_[i], cloud_keyposes_6d_[i].matrix().cast<float>();
    *map += *tmp;
    num_points += tmp->points.size();

//    pcl::PointCloud<PointT>::Ptr tmp1(new pcl::PointCloud<PointT>());
//    ground_filter.convert(cloud_keyframes_[i], tmp1);
//    tmp1 = transformPointCloud(*cloud_keyframes_[i], *tmp1, cloud_keyposes_6d_[i].matrix().cast<float>());
//
//    *map_no_ground += *tmp1;
//    num_points1 += tmp1->points.size();
  }

  map->width = map->points.size();
  map->height = 1;
  map->is_dense = false;

  map_no_ground->width = map_no_ground->points.size();
  map_no_ground->height = 1;
  map_no_ground->is_dense = false;

  pcl::PointCloud<PointT>::Ptr poses(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*cloud_keyposes_3d_, *poses);
  poses->width = poses->points.size();
  poses->height = 1;
  poses->is_dense = false;

  pcl::io::savePCDFile(save_dir_ + "trajectory_" + stamp + ".pcd", *poses);
  pcl::io::savePCDFile(save_dir_ + "finalCloud_" + stamp + ".pcd", *map);
  pcl::io::savePCDFile(save_dir_ + "final_no_ground_" + stamp + ".pcd", *map_no_ground);

  ROS_WARN("Save map. pose size: %d, cloud size: %d, cloud no ground size: %d", poses->points.size(), num_points,
           num_points1);

//    Eigen::Translation3f tf_t(init_tf_x, init_tf_y, init_tf_z);         // tl: translation
//    Eigen::AngleAxisf rot_x(init_tf_roll, Eigen::Vector3f::UnitX());    // rot: rotation
//    Eigen::AngleAxisf rot_y(init_tf_pitch, Eigen::Vector3f::UnitY());
//    Eigen::AngleAxisf rot_z(init_tf_yaw, Eigen::Vector3f::UnitZ());
//    Eigen::Matrix4f map_to_init_trans_matrix = (tf_t * rot_z * rot_y * rot_x).matrix();
//
//    pcl::PointCloud<PointType>::Ptr transformed_pc_ptr(new pcl::PointCloud<PointType>());
//
//    pcl::transformPointCloud(*globalMapKeyFramesDS, *transformed_pc_ptr, map_to_init_trans_matrix);
//
//    pcl::io::savePCDFileASCII(pcd_file_path + "finalCloud.pcd", *transformed_pc_ptr);
//
//    // 后面的各种特征点地图可以注释掉
//    pcl::PointCloud<PointType>::Ptr cornerMapCloud(new pcl::PointCloud<PointType>());
//    pcl::PointCloud<PointType>::Ptr cornerMapCloudDS(new pcl::PointCloud<PointType>());
//    pcl::PointCloud<PointType>::Ptr surfaceMapCloud(new pcl::PointCloud<PointType>());
//    pcl::PointCloud<PointType>::Ptr surfaceMapCloudDS(new pcl::PointCloud<PointType>());
//
//    for (int i = 0; i < cornerCloudKeyFrames.size(); i++) {
//        *cornerMapCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
//        *surfaceMapCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
//        *surfaceMapCloud += *transformPointCloud(outlierCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
//    }
//
//    downSizeFilterCorner.setInputCloud(cornerMapCloud);
//    downSizeFilterCorner.filter(*cornerMapCloudDS);
//    downSizeFilterSurf.setInputCloud(surfaceMapCloud);
//    downSizeFilterSurf.filter(*surfaceMapCloudDS);
//
//    pcl::transformPointCloud(*globalMapKeyFramesDS, *transformed_pc_ptr, map_to_init_trans_matrix);
//
//    pcl::io::savePCDFileASCII(pcd_file_path + "cornerMap.pcd", *cornerMapCloudDS);
//    pcl::io::savePCDFileASCII(pcd_file_path + "surfaceMap.pcd", *surfaceMapCloudDS);
//    pcl::io::savePCDFileASCII(pcd_file_path + "trajectory.pcd", *cloudKeyPoses3D);

}

bool saveKeyframesAndFactor(pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud_in,
                            Eigen::Isometry3d &odom_in,
                            ros::Time &current_time) {
  // 此处的当前位姿(cur_pose_ndt_)为 ndt 匹配后的 final_transformation
  curr_pose3 = EigenToPose3(odom_in);
  cur_point.x = curr_pose3.translation().x();
  cur_point.y = curr_pose3.translation().y();
  cur_point.z = curr_pose3.translation().z();

  bool saveThisKeyFrame = true;
  if (sqrt((pre_point.x - cur_point.x) * (pre_point.x - cur_point.x)
               + (pre_point.y - cur_point.y) * (pre_point.y - cur_point.y)
               + (pre_point.z - cur_point.z) * (pre_point.z - cur_point.z)) < 0.3) {
    saveThisKeyFrame = false;
  }
  if (saveThisKeyFrame == false && !cloud_keyposes_3d_->points.empty())
    return false;
  pre_point = cur_point;

  if (cloud_keyposes_3d_->points.empty()) {
    gtSAMgraph.add(PriorFactor<Pose3>(0, curr_pose3, priorModel));
    initialEstimate.insert(0, curr_pose3);
    last_pose3 = curr_pose3;
  } else {
    //const auto &pre_pose = op->points[cloud_keyposes_3d_->points.size() - 1];
    const auto &pre_pose = pose_optimized_arr[pose_optimized_arr.size() - 1];
    gtSAMgraph.add(BetweenFactor<Pose3>(cloud_keyposes_3d_->points.size() - 1, cloud_keyposes_3d_->points.size(),
                                        pre_pose.between(curr_pose3), odomModel));
    initialEstimate.insert(cloud_keyposes_3d_->points.size(), curr_pose3);
  }

  isam->update(gtSAMgraph, initialEstimate);
  isam->update();

  gtSAMgraph.resize(0);
  initialEstimate.clear();

  PointT this_pose_3d;
  Eigen::Isometry3d update_odom = Eigen::Isometry3d::Identity();
  isam_current_estimate_ = isam->calculateEstimate();
  Pose3 latest_estimate = isam_current_estimate_.at<Pose3>(isam_current_estimate_.size() - 1);

  this_pose_3d.x = latest_estimate.translation().x();
  this_pose_3d.y = latest_estimate.translation().y();
  this_pose_3d.z = latest_estimate.translation().z();
  this_pose_3d.intensity = cloud_keyposes_3d_->points.size();

  update_odom.setIdentity();
  update_odom = Pose3ToEigen(latest_estimate);

  pose_optimized_arr.push_back(latest_estimate);
  cloud_keyposes_6d_.push_back(update_odom);
  time_arr_.push_back(current_time);
  cloud_keyposes_3d_->points.push_back(this_pose_3d);

  if (cloud_keyposes_3d_->points.size() > 1) {
    pre_key_odom_ = update_odom;
    pre_key_time_ = current_time;
  }
  current_odom_ = pre_key_odom_ = update_odom;

  pcl::PointCloud<PointT>::Ptr cur_keyframe(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*pointcloud_in, *cur_keyframe);
  for (auto &p : cur_keyframe->points) {
    p.intensity = this_pose_3d.intensity;
  }
  key_frames_vec_.push_back(cur_keyframe);
  return true;
}

void correctPoses() {
  if (loop_closed_) {
    recent_keyframes_.clear();
    ROS_WARN("correctPoses");
    int num_poses = isam_current_estimate_.size();
    for (int i = 0; i < num_poses; ++i) {
      Eigen::Isometry3d new_odom = Pose3ToEigen(isam_current_estimate_.at<Pose3>(i));
      cloud_keyposes_6d_[i].setIdentity();
      cloud_keyposes_6d_[i] = new_odom;
      cloud_keyposes_3d_->points[i].x = new_odom.translation().x();
      cloud_keyposes_3d_->points[i].y = new_odom.translation().y();
      cloud_keyposes_3d_->points[i].z = new_odom.translation().z();
    }
    loop_closed_ = false;
  }
}

bool saveMapCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  std::stringstream ss;
  ss << int(ros::Time::now().toSec());
  std::string stamp = ss.str();

  int num_points = 0, num_points1 = 0;
  pcl::PointCloud<PointT>::Ptr map(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr map_no_ground(new pcl::PointCloud<PointT>());
  for (int i = 0; i < key_frames_vec_.size(); ++i) {
    pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*key_frames_vec_[i], *tmp, cloud_keyposes_6d_[i].matrix().cast<float>());
    *map += *tmp;
    num_points += tmp->points.size();

    /* pcl::PointCloud<PointT>::Ptr tmp1(new pcl::PointCloud<PointT>());
     ground_filter.convert(cloud_keyframes_[i], tmp1);
     tmp1 = transformPointCloud(tmp1, cloud_keyposes_6d_->points[i]);
     *map_no_ground += *tmp1;
     num_points1 += tmp1->points.size();*/
  }

  map->width = map->points.size();
  map->height = 1;
  map->is_dense = false;

  map_no_ground->width = map_no_ground->points.size();
  map_no_ground->height = 1;
  map_no_ground->is_dense = false;

  pcl::PointCloud<PointT>::Ptr poses(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*cloud_keyposes_3d_, *poses);
  poses->width = poses->points.size();
  poses->height = 1;
  poses->is_dense = false;

  pcl::io::savePCDFile(save_dir_ + "pose3d_" + stamp + ".pcd", *poses);
  pcl::io::savePCDFile(save_dir_ + "frames_" + stamp + ".pcd", *map);
  pcl::io::savePCDFile(save_dir_ + "frames_no_ground_" + stamp + ".pcd", *map_no_ground);

  ROS_WARN("Save map. pose size: %d, cloud size: %d, cloud no ground size: %d", poses->points.size(), num_points,
           num_points1);
}

/**
 * @brief 基于里程计的回环检测，直接提取当前位姿附近(radiusSearch)的 keyframe 作为 icp 的 target
 *
 */
bool detectLoopClosure() {
  latest_keyframe_->clear();
  near_history_keyframes_->clear();

  std::lock_guard<std::mutex> lock(mtx_);

  latest_frame_id_ = key_frames_vec_.size() - 1;
//  cur_point.x = cloud_keyposes_6d_[latest_history_frame_id_].translation().x();
//  cur_point.y = cloud_keyposes_6d_[latest_history_frame_id_].translation().y();
//  cur_point.z = cloud_keyposes_6d_[latest_history_frame_id_].translation().z();

  std::vector<int> search_idx_;
  std::vector<float> search_dist_;
  kdtree_poses_->setInputCloud(cloud_keyposes_3d_);
  kdtree_poses_->radiusSearch(cur_point, 5.0, search_idx_, search_dist_, 0);

  closest_history_frame_id_ = -1;
  for (int i = 0; i < search_idx_.size(); i++) {
    int id = search_idx_[i];
    double time = std::abs(pointcloud_time.toSec() - time_arr_[id].toSec());
    // ROS_INFO("time %d, %d", time, search_idx_.size());
    if (time > 20) {
      closest_history_frame_id_ = id;
      break;
    }
  }
  // 时间太短不做回环
  if (closest_history_frame_id_ == -1) {
    return false;
  }

  ROS_INFO("find close frame %d", closest_history_frame_id_);
  pcl::transformPointCloud(*key_frames_vec_[latest_frame_id_],
                           *latest_keyframe_,
                           cloud_keyposes_6d_[latest_frame_id_].matrix().cast<float>());

  // filter intensity = -1
  pcl::PointCloud<PointT>::Ptr hahaCloud(new pcl::PointCloud<PointT>());
  int cloudSize = latest_keyframe_->points.size();
  for (int i = 0; i < cloudSize; ++i){
    // intensity不小于0的点放进hahaCloud队列
    // 初始化时intensity是-1，滤掉那些点
    if ((int)latest_keyframe_->points[i].intensity >= 0){
      hahaCloud->push_back(latest_keyframe_->points[i]);
    }
  }
  latest_keyframe_->clear();
  *latest_keyframe_   = *hahaCloud;

  pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr clear_cloud(new pcl::PointCloud<PointT>());
  for (int i = -history_search_num_, j; i <= history_search_num_; ++i) {
    j = closest_history_frame_id_ + i;
    if (j < 0 || j > latest_frame_id_) {
      continue;
    }
    clear_cloud->clear();
    pcl::transformPointCloud(*key_frames_vec_[j], *tmp_cloud, cloud_keyposes_6d_[j].matrix().cast<float>());
    *tmp_cloud += *clear_cloud;
  }
  downSizeFilterKeyFrames.setInputCloud(tmp_cloud);
  downSizeFilterKeyFrames.filter(*near_history_keyframes_);

  return true;
}

/**
 * @brief 回环检测及位姿图更新
 * ICP 匹配添加回环约束
 */
void performLoopClosure() {
  if (cloud_keyposes_3d_->points.empty()) {
    ROS_WARN("Waiting key pose...");
    return;
  }

  if (!detectLoopClosure()) {
    return;
  } else {
    ROS_WARN("detected loop closure");
  }

  if (near_history_keyframes_->empty() || latest_keyframe_->empty()) {
    ROS_ERROR("Empty cloud...");
    return;
  }
  ROS_INFO("icp size: %d, %d", near_history_keyframes_->size(), latest_keyframe_->size());

  // 检测到回环后，用icp去匹配
  auto start = std::chrono::system_clock::now();

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);
  icp.setInputSource(latest_keyframe_);
  icp.setInputTarget(near_history_keyframes_);
  pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
  icp.align(*unused_result);
  bool has_converged = icp.hasConverged();
  float fitness_score = icp.getFitnessScore();

  if(!has_converged || fitness_score > 0.3){
    ROS_INFO("Frame id: %d, %d, %f", latest_frame_id_, closest_history_frame_id_, fitness_score);
    ROS_WARN("loop cannot closed");
    return;
  }
  ROS_WARN("loop closed");
  ROS_INFO("closed id: %d, %d, %f", latest_frame_id_, closest_history_frame_id_, fitness_score);

  Eigen::Matrix4f correction_frame = icp.getFinalTransformation();
/*  Eigen::Quaternionf tmp_q(correction_frame.block<3, 3>(0, 0));
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);*/
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;

  Eigen::Matrix4f t_wrong =  cloud_keyposes_6d_[latest_frame_id_].matrix().cast<float>();
  Eigen::Matrix4f t_correct = correction_frame * t_wrong;
  Eigen::Quaternionf r_correct(t_correct.block<3, 3>(0, 0));
  gtsam::Pose3 pose_from = Pose3(Rot3::Quaternion(r_correct.w(), r_correct.x(), r_correct.y(), r_correct.z()),
                                 Point3(t_correct(0, 3), t_correct(1, 3), t_correct(2, 3)));
  gtsam::Pose3 pose_to = EigenToPose3(cloud_keyposes_6d_[closest_history_frame_id_]);

  gtsam::Vector vector6(6);
  vector6 << fitness_score, fitness_score, fitness_score, fitness_score, fitness_score, fitness_score;
  loopModel = noiseModel::Diagonal::Variances(vector6);

  std::lock_guard<std::mutex> lock(mtx_);
  gtSAMgraph.add(BetweenFactor<Pose3>(latest_frame_id_, closest_history_frame_id_, pose_from.between(pose_to),
                                      loopModel));
  isam->update(gtSAMgraph);
  isam->update();
  gtSAMgraph.resize(0);
  loop_closed_ = true;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Time elapsed: " << elapsed.count() << std::endl;
  std::cout << "Number of source points: " << latest_keyframe_->size() << " points." << std::endl;
  std::cout << "target: " << near_history_keyframes_->points.size() << " points." << std::endl;
  std::cout << "ICP has converged: " << has_converged << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
/*  std::cout << "initial (x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "final (x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << t_correct(0, 3) << ", " << t_correct(1, 3) << ", " << t_correct(2, 3) << ", "
            << roll << ", " << pitch << ", " << yaw << ")" << std::endl;*/
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_correct << std::endl;
  std::cout << "shift: " << std::sqrt(std::pow(correction_frame(0, 3) - t_wrong(0, 3), 2) +
      std::pow(correction_frame(1, 3) - t_wrong(1, 3), 2) +
      std::pow(correction_frame(2, 3) - t_wrong(2, 3), 2)) << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

void loopClosureThread() {
  if (!loop_closure_enabled_) {
    return;
  }
  // 不停地进行回环检测
  ros::Duration duration(1);
  while (ros::ok()) {
    duration.sleep();
    performLoopClosure();
  }
}

void visualThread() {
  ros::Rate rate(0.2);
  while (ros::ok()) {
    rate.sleep();
    publishKeyposesAndFrames();
  }
  // save final point cloud
  TransformAndSaveMap();
}

void Run() {
  if (!cloud_queue_.empty() && !odom_queue_.empty()) {
    mutex_lock.lock();
    double time2 = odom_queue_.front()->header.stamp.toSec();
    double time3 = cloud_queue_.front()->header.stamp.toSec();

    if (!odom_queue_.empty() && (time2 < time3 - 0.5 * 0.1)) {
      ROS_WARN("time stamp unaligned error and odometryBuf discarded, pls check your data -->  optimization");
      odom_queue_.pop();
      mutex_lock.unlock();
      return;
    }
    if (!cloud_queue_.empty() && (time3 < time2 - 0.5 * 0.1)) {
      ROS_WARN("time stamp unaligned error and pointCloudEdgeBuf discarded, pls check your data -->  optimization");
      cloud_queue_.pop();
      mutex_lock.unlock();
      return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_queue_.front(), *pointcloud_in);
    pointcloud_time = (cloud_queue_.front())->header.stamp;

    // odom数据
    Eigen::Isometry3d odom_in = Eigen::Isometry3d::Identity();
    odom_in.rotate(Eigen::Quaterniond(odom_queue_.front()->pose.pose.orientation.w,
                                      odom_queue_.front()->pose.pose.orientation.x,
                                      odom_queue_.front()->pose.pose.orientation.y,
                                      odom_queue_.front()->pose.pose.orientation.z));
    odom_in.pretranslate(Eigen::Vector3d(odom_queue_.front()->pose.pose.position.x,
                                         odom_queue_.front()->pose.pose.position.y,
                                         odom_queue_.front()->pose.pose.position.z));

    cloud_queue_.pop();
    odom_queue_.pop();
    mutex_lock.unlock();

    // std::lock_guard<std::mutex> lock(mtx_);

    // 寻找关键帧并修正
    if (saveKeyframesAndFactor(pointcloud_in, odom_in, pointcloud_time)) {
      pcl::PointCloud<PointT>::Ptr pc_m(new pcl::PointCloud<PointT>());
      pcl::transformPointCloud(*pointcloud_in, *pc_m, odom_in.matrix());
    } else {
      //std::cout << "too close" << std::endl;
      return;
    }
    //pre_key_odom_ = odom_in;

    // 匹配完了去更新当前pose
    correctPoses();
  }
}

//void Process() {
//  while (1) {
//    // 时间戳对齐
//    if (/*!loop_queue_.empty() && */!cloud_queue_.empty() && !odom_queue_.empty()) {
//      mutex_lock.lock();
//      double time1 = loop_queue_.front()->header.stamp.toSec();
//      double time2 = odom_queue_.front()->header.stamp.toSec();
//      double time3 = cloud_queue_.front()->header.stamp.toSec();
//      if (!loop_queue_.empty() && (time1 < time2 - 0.5 * 0.1 || time1 < time3 - 0.5 * 0.1)) {
//        ROS_WARN("time stamp unaligned error and loopInfoBuf discarded, pls check your data -->  optimization");
//        loop_queue_.pop();
//        mutex_lock.unlock();
//        continue;
//      }
//      if (!odom_queue_.empty() && (time2 < time1 - 0.5 * 0.1 || time2 < time3 - 0.5 * 0.1)) {
//        ROS_WARN("time stamp unaligned error and odometryBuf discarded, pls check your data -->  optimization");
//        odom_queue_.pop();
//        mutex_lock.unlock();
//        continue;
//      }
//      if (!cloud_queue_.empty() && (time3 < time1 - 0.5 * 0.1 || time3 < time2 - 0.5 * 0.1)) {
//        ROS_WARN("time stamp unaligned error and pointCloudEdgeBuf discarded, pls check your data -->  optimization");
//        cloud_queue_.pop();
//        mutex_lock.unlock();
//        continue;
//      }
//      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
//      pcl::fromROSMsg(*cloud_queue_.front(), *pointcloud_in);
//      pointcloud_time = (cloud_queue_.front())->header.stamp;
////      timeLaserOdometry = pointcloud_time.toSec();
//      // odom数据
//      Eigen::Isometry3d odom_in = Eigen::Isometry3d::Identity();
//      odom_in.rotate(Eigen::Quaterniond(odom_queue_.front()->pose.pose.orientation.w,
//                                        odom_queue_.front()->pose.pose.orientation.x,
//                                        odom_queue_.front()->pose.pose.orientation.y,
//                                        odom_queue_.front()->pose.pose.orientation.z));
//      odom_in.pretranslate(Eigen::Vector3d(odom_queue_.front()->pose.pose.position.x,
//                                           odom_queue_.front()->pose.pose.position.y,
//                                           odom_queue_.front()->pose.pose.position.z));
//
//      // get loop result
//      xchu_mapping::LoopInfoConstPtr loop_msg = loop_queue_.front();
//      int current_frame_id = loop_queue_.front()->current_id;
//      int match_id = -1;
//      bool is_loop = false;
//      double loop_score = 0.0;
//      Eigen::Matrix4d loop_transform = Eigen::Matrix4d::Identity();
//      if (loop_msg->matched_id.size() == 1) {
//        is_loop = true;
//        loop_score = loop_msg->score;
//
//        match_id = loop_msg->matched_id.front();
//        loop_transform(0, 0) = loop_msg->transform[0];
//        loop_transform(0, 1) = loop_msg->transform[1];
//        loop_transform(0, 2) = loop_msg->transform[2];
//        loop_transform(1, 0) = loop_msg->transform[3];
//        loop_transform(1, 1) = loop_msg->transform[4];
//        loop_transform(1, 2) = loop_msg->transform[5];
//        loop_transform(2, 0) = loop_msg->transform[6];
//        loop_transform(2, 1) = loop_msg->transform[7];
//        loop_transform(2, 2) = loop_msg->transform[8];
//
//        loop_transform(0, 3) = loop_msg->transform[9];
//        loop_transform(1, 3) = loop_msg->transform[10];
//        loop_transform(2, 3) = loop_msg->transform[11];
//      }
//
//      cloud_queue_.pop();
//      odom_queue_.pop();
//      loop_queue_.pop();
//      mutex_lock.unlock();
//
//      // 有回环时执行回环检测, 无回环时只添加位姿图节点
//      if (current_frame_id != (int) key_frames_vec_.size()) {
//        ROS_WARN_ONCE("graph optimization frame not aligned, pls check your data");
//      }
//
//      // 位子图添加节点
////      gtsam::Pose3 pose3_current = EigenToPose3(odom_in);  // se3 eigen转gtsam
//      if (cloud_keyposes_3d_->empty()) {
//        //ROS_INFO("---------------------------add first pose-----------------------");
//        // 因子图中添加一元因子, 系统先验, back是最新, front是醉酒
////        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, pose3_current, priorModel));
////        initialEstimate.insert(0, pose3_current);  // 初值容器初始化
//
//        //保存初始点信息，为先验因子。下面操作是把四元数转换为旋转矩阵，旋转矩阵变化为欧拉角
//        gtsam::Pose3 curr_pose = EigenToPose3(map_curr_);
//        initialEstimate.insert(temp_laserCloudMap_Ind + 1, curr_pose);
//        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
//            (Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished());
//        gtSAMgraph.add(PriorFactor<Pose3>(1, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, 0, 0)), priorNoise));
//
//      } else {
//        // 世界坐标系下的位姿
//        map_curr_ = map_odom_curr_ * odom_in;
//
//        // odom_original_arr pose_optimized_arr中分别存储odom位姿和优化后的odom位姿
//        /*  odom_original_arr.push_back(last_pose3.between(pose3_current)); // 存相对变换
//          // 因子图中加入二元因子,相邻节点之间边的约束，通过相邻帧边缘点来关联
//          gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(key_frames_vec_.size() - 1,
//                                                            key_frames_vec_.size(),
//                                                            odom_original_arr.back(),
//                                                            odomModel));
//          // initialEstimate.insert(key_frames_vec_.size(), pose_optimized_arr.back()); // 加入一个变量
//          initialEstimate.insert(key_frames_vec_.size(), pose3_current); // 加入一个变量*/
//
//        //保存当前位姿和里程计变换的信息到gtsam的graph，
//        gtsam::Pose3 last_pose3_ = EigenToPose3(map_curr_last_);
//        gtsam::Pose3 curr_pose3_ = EigenToPose3(map_curr_);
//        initialEstimate.insert(temp_laserCloudMap_Ind + 1, curr_pose3_);
//        //gtsam提供一种容易求两点之间位姿变化的函数between，用于里程计变化信息的录入
//        noiseModel::Diagonal::shared_ptr
//            odom_noise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
//        gtSAMgraph.add(BetweenFactor<Pose3>(temp_laserCloudMap_Ind,
//                                            temp_laserCloudMap_Ind + 1,
//                                            last_pose3_.between(curr_pose3_),
//                                            odom_noise));
//
//
//        // 会还
//        /*  if (is_loop && match_id != -1) {
//            //获取变换矩阵，求四元数q和位移t_vector
//            Eigen::Matrix3d rotation = loop_transform.block<3, 3>(0, 0);
//            Eigen::Vector3d t = loop_transform.block<3, 1>(0, 3);
//            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
//            transform.rotate(rotation);
//            transform.pretranslate(t);
//
//            //位姿变换
//            Eigen::Isometry3d temp_curr = map_curr_ * transform;
//
//            //发生闭环，添加闭环信息到gtsam的graph，闭环noise使用icp得分
//            Values isamCurrentEstimate1;
//            Pose3 closeEstimate;
//            isamCurrentEstimate1 = isam->calculateEstimate();
//            closeEstimate = isamCurrentEstimate1.at<Pose3>(match_id);
//
//            gtsam::Pose3 loop_temp = EigenToPose3(temp_curr);
//            noiseModel::Diagonal::shared_ptr closure_noise = noiseModel::Diagonal::Variances((Vector(6)
//                << loop_score, loop_score, loop_score, loop_score, loop_score, loop_score).finished());
//            gtSAMgraph.add(BetweenFactor<Pose3>(match_id,
//                                                temp_laserCloudMap_Ind + 1,
//                                                closeEstimate.between(loop_temp),
//                                                closure_noise));
//          }*/
//      }
//
//      //每次循环后进行gtsam迭代
//      isam->update(gtSAMgraph, initialEstimate);
//      isam->update();
//      //获取优化后的所有位姿
//      Pose3 currEstimate;
//      Values isamCurrentEstimate2;
//      isamCurrentEstimate2 = isam->calculateEstimate();
//      //获取当前帧的位姿，保存在当前位姿里
//      currEstimate = isamCurrentEstimate2.at<Pose3>(temp_laserCloudMap_Ind + 1);
//      map_curr_ = Pose3ToEigen(currEstimate);
//      map_odom_curr_ = map_curr_ * odom_in.inverse();
//
//
//      //清空原来的约束。已经加入到isam2的那些会用bayes tree保管
//      gtSAMgraph.resize(0);
//      initialEstimate.clear();
//      //当前帧赋值给历史帧，用于之后里程计信息添加到gtsam的边
//      map_curr_last_ = map_curr_;
//      odom_curr_last_ = odom_in;
//      map_odom_curr_last_ = map_odom_curr_;
//
////      q_w_last = q_w_curr;
////      t_w_last = t_w_curr;
//
//      //记录当前帧保存后的点数便于之后查询，自加帧数
////      laserCloudMap_Ind[temp_laserCloudMap_Ind] = laserCloudMap->points.size();
//      temp_laserCloudMap_Ind++;
//
//      //保存位姿，便于之后位姿KD树搜索
//      PointT point;
//      point.x = map_curr_.translation().x();
//      point.y = map_curr_.translation().y();
//      point.z = map_curr_.translation().z();
//      //point.intensity = cloud_keyposes_3d_->points.size();
//      cloud_keyposes_3d_->push_back(point);
//
////      isam->update(gtSAMgraph, initialEstimate);
////      isam->update();
////      gtSAMgraph.resize(0);
////      initialEstimate.clear(); // 清空初值
////
////      // 优化之后的pose
////      Pose3 latestEstimate;
////      isamCurrentEstimate = isam->calculateEstimate();
////      pose3_current = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
////
////      time_arr_.push_back(pointcloud_time);
////      key_frames_vec_.push_back(pointcloud_in);
////      pose_optimized_arr.push_back(pose3_current);
////      odom_arr_.push_back(Pose3ToEigen(pose3_current)); // 原始odom
////
////      // update pose
////      last_pose3 = pose3_current;
////      PointT point;
////      point.x = pose3_current.x();
////      point.y = pose3_current.y();
////      point.z = pose3_current.z();
////      point.intensity = cloud_keyposes_3d_->points.size();
////      cloud_keyposes_3d_->push_back(point);
////
////      // publish pose
////      Eigen::Isometry3d pose_current = Pose3ToEigen(pose_optimized_arr.back());
////      Eigen::Quaterniond q_current(pose_current.rotation());
////      Eigen::Vector3d t_current = pose_current.translation();
////      w_odom_curr = pose_current * odom_in.inverse();
//
//      /*
//      static tf::TransformBroadcaster br_realtime;
//      tf::Transform transform;
//      transform.setOrigin(tf::Vector3(pose3_current.translation().x(),
//                                      pose3_current.translation().y(),
//                                      pose3_current.translation().z()));
//      tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
//      transform.setRotation(q);
//      br_realtime.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom_final"));
//
//      // publish odometry
//      nav_msgs::Odometry laserOdometry;
//      laserOdometry.header.frame_id = "/map"; //world
//      laserOdometry.child_frame_id = "/odom_final"; //odom
//      laserOdometry.header.stamp = pointcloud_time;
//      laserOdometry.pose.pose.orientation.x = q_current.x();
//      laserOdometry.pose.pose.orientation.y = q_current.y();
//      laserOdometry.pose.pose.orientation.z = q_current.z();
//      laserOdometry.pose.pose.orientation.w = q_current.w();
//      laserOdometry.pose.pose.position.x = pose_current.translation().x();
//      laserOdometry.pose.pose.position.y = pose_current.translation().y();
//      laserOdometry.pose.pose.position.z = pose_current.translation().z();
//      odom_pub.publish(laserOdometry);*/
//
//      // 发布关键帧位姿
//      if (optimize_poses_pub.getNumSubscribers() > 0) {
//        sensor_msgs::PointCloud2 msg;
//        pcl::toROSMsg(*cloud_keyposes_3d_, msg);
//        msg.header.frame_id = "map";
//        msg.header.stamp = ros::Time::now();
//        optimize_poses_pub.publish(msg);
//      }
//    }
//    //sleep 2 ms every time
//    std::chrono::milliseconds dura(2);
//    std::this_thread::sleep_for(dura);
//  }//end of while(1)
//}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Optimization");
  ros::NodeHandle nh("~");

  // publisher
  // optimize_poses_pub = nh.advertise<sensor_msgs::PointCloud2>("/final_poses", 1);  // 关键帧点云
  pub_keyposes_ = nh.advertise<sensor_msgs::PointCloud2>("/keyposes", 1);  // key pose
  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_current", 100);
  pub_globalmap_ = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1);  // 提取到的附近点云
  pub_history_keyframes_ = nh.advertise<sensor_msgs::PointCloud2>("/history_keyframes", 1);
  pub_recent_keyframes_ = nh.advertise<sensor_msgs::PointCloud2>("/recent_keyframes", 1);
  srv_save_map_ = nh.advertiseService("/save_map", saveMapCB);

  // 订阅odom节点
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/current_odom", 10, OdomCb);
  ros::Subscriber points_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_points", 10, PointCb);
  //ros::Subscriber loop_sub = nh.subscribe<xchu_mapping::LoopInfo>("/loop_closure", 10, loopCb); // 在isc节点中检测回环

  downSizeFilterKeyFrames.setLeafSize(0.5, 0.5, 0.5); // 发布全局地图的采样size,设置小了导致系统卡顿
  downSizeFilterGlobalMapKeyFrames.setLeafSize(1.0, 1.0, 1.0); // 发布全局地图的采样size,设置小了导致系统卡顿
  downSizeFilterKeyFrames.setLeafSize(0.5, 0.5, 0.5); // 发布全局地图的采样size,设置小了导致系统卡顿
  downSizeFilterHistoryKeyFrames.setLeafSize(1.0, 1.0, 1.0); // 发布全局地图的采样size,设置小了导致系统卡顿

  // 初始化一些参数，包括gtsam噪声模型
  key_frames_vec_.clear();
  nearHistoryCloudKeyFrameCloud.reset(new pcl::PointCloud<PointT>());
  cloud_keyposes_3d_.reset(new pcl::PointCloud<PointT>());

  latestKeyFrameCloud.reset(new pcl::PointCloud<PointT>());
  latestKeyFrameCloudDS.reset(new pcl::PointCloud<PointT>());
  latest_frame_cloud_.reset(new pcl::PointCloud<PointT>());

  latest_keyframe_.reset(new pcl::PointCloud<PointT>());
  near_history_keyframes_.reset(new pcl::PointCloud<PointT>());

  kdtreeCloudFromMap.reset(new pcl::KdTreeFLANN<PointT>());
  kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointT>());
  kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointT>());

  kdtree_poses_.reset(new pcl::KdTreeFLANN<PointT>());

  globalMap_keyPoses_.reset(new pcl::PointCloud<PointT>());
  localmap_pose_.reset(new pcl::PointCloud<PointT>());
  localmap_cloud_.reset(new pcl::PointCloud<PointT>());
  localmap_frames_.reset(new pcl::PointCloud<PointT>());
  global_map_cloud_.reset(new pcl::PointCloud<PointT>());
//  globalMapKeyPosesDS.reset(new pcl::PointCloud<PointT>());
//  globalMapKeyFrames.reset(new pcl::PointCloud<PointT>());
//  globalMapKeyFramesDS.reset(new pcl::PointCloud<PointT>());

  // gtsam　参数设置
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = new ISAM2(parameters);

  gtsam::Vector Vector6(6);
  Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  priorModel = noiseModel::Diagonal::Variances(Vector6);
  odomModel = noiseModel::Diagonal::Variances(Vector6);

  // 新开一个线程, 一较低频率执行优化
  std::thread viewer_thread(&visualThread);
  std::thread loop_thread(&loopClosureThread);

  ros::Rate rate(100);
  while (ros::ok()) {
    Run();

    ros::spinOnce();
    rate.sleep();
  }
  viewer_thread.join();
  loop_thread.join();

  ros::spin();

  return 0;
}