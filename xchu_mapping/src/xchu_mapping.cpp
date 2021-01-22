
/**
 * @file xchu_mapping.cpp
 * @author XCHU (2022087641@qq.com)
 * @brief
 * @version 1.0
 * @date 2020-09-20
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <xchu_mapping/xchu_mapping.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "xchu_mapping_node");

  LidarMapping mapping;
  mapping.run();

  return 0;
}

LidarMapping::LidarMapping() : privateHandle("~") {

  // 初始化参数
  param_initial(privateHandle);

  ndt_map_pub = privateHandle.advertise<sensor_msgs::PointCloud2>("/global_map", 10);
  local_map_pub = privateHandle.advertise<sensor_msgs::PointCloud2>("/local_map", 10);
  current_points_pub = privateHandle.advertise<sensor_msgs::PointCloud2>("/current_points", 10);
  current_odom_pub = privateHandle.advertise<nav_msgs::Odometry>("/current_odom", 100);
  keyposes_pub = privateHandle.advertise<sensor_msgs::PointCloud2>("/keyposes", 1);  // 关键帧点云

  points_sub = privateHandle.subscribe(_lidar_topic, 100, &LidarMapping::points_callback, this);
  odom_sub = privateHandle.subscribe(_odom_topic, 1000, &LidarMapping::odom_callback, this);
  imu_sub = privateHandle.subscribe(_imu_topic, 4000, &LidarMapping::imu_callback, this);
}

void LidarMapping::param_initial(ros::NodeHandle &privateHandle) {
  std::cout << "******************************************" << std::endl;
  privateHandle.param<float>("ndt_resolution", ndt_res, 1.0);
  privateHandle.param<double>("ndt_step_size", step_size, 0.1);
  privateHandle.param<double>("ndt_trans_eps", trans_eps, 0.01);
  privateHandle.param<int>("ndt_max_iter", max_iter, 30);
  privateHandle.param<double>("voxel_leaf_size", voxel_leaf_size, 0.01);
  privateHandle.param<double>("min_scan_range", min_scan_range, 5);
  privateHandle.param<double>("max_scan_range", max_scan_range, 80);
  privateHandle.param<double>("min_add_scan_shift", min_add_scan_shift, 0.5);
  privateHandle.param<double>("max_submap_size", max_localmap_size, 400);
  privateHandle.param<std::string>("map_saved_dir", map_saved_dir, "");

  std::cout << "NDT_param_initial" << std::endl;
  std::cout << "ndt_res: " << ndt_res << std::endl;
  std::cout << "step_size: " << step_size << std::endl;
  std::cout << "trans_epsilon: " << trans_eps << std::endl;
  std::cout << "max_iter: " << max_iter << std::endl;
  std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
  std::cout << "min_scan_range: " << min_scan_range << std::endl;
  std::cout << "max_scan_range: " << max_scan_range << std::endl;
  std::cout << "min_add_scan_shift: " << min_add_scan_shift << std::endl;
  std::cout << "max_submap_size: " << max_localmap_size << std::endl;
  std::cout << "map_saved_dir: " << map_saved_dir << std::endl;
  std::cout << std::endl;

  privateHandle.param<bool>("use_imu", _use_imu, false);
  privateHandle.param<bool>("use_odom", _use_odom, false);
  privateHandle.param<bool>("imu_upside_down", _imu_upside_down, false);

  std::cout << "use imu: " << _use_imu << std::endl;
  std::cout << "use_odom: " << _use_odom << std::endl;
  std::cout << "reverse imu: " << _imu_upside_down << std::endl;
  std::cout << std::endl;

  privateHandle.param<std::string>("imu_topic", _imu_topic, "/imu_raw");
  privateHandle.param<std::string>("odom_topic", _odom_topic, "/odom_raw");
  privateHandle.param<std::string>("lidar_topic", _lidar_topic, "/velodyne_points");

  std::cout << "imu topic: " << _imu_topic << std::endl;
  std::cout << "odom topic: " << _odom_topic << std::endl;
  std::cout << "lidar topic: " << _lidar_topic << std::endl;
  std::cout << std::endl;

  privateHandle.param<bool>("incremental_voxel_update", _incremental_voxel_update, true);
  std::cout << "incremental_voxel_update: " << _incremental_voxel_update << std::endl;  // 是否在cpu_ndt里做update????


  downSizeFilterKeyFrames.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size); // 发布全局地图的采样size,设置小了导致系统卡顿
  downSizeFilterGlobalMap.setLeafSize(voxel_leaf_size,
                                      voxel_leaf_size,
                                      voxel_leaf_size); // 保存全局地图时下采样size，可以设置小一些方便看清楚点云细节
  downSizeFilterLocalmap.setLeafSize(voxel_leaf_size * 2,
                                     voxel_leaf_size * 2,
                                     voxel_leaf_size * 2);// 发布localmap的下采样size

  cloud_keyposes_3d_.reset(new pcl::PointCloud<PointT>());

  int method_type_temp;
  privateHandle.param<int>("ndt_method_type", method_type_temp, 1);
  _method_type = static_cast<MethodType>(method_type_temp);
  std::cout << std::endl;

  if (_method_type == MethodType::use_pcl) {
    std::cout << ">> Use PCL NDT <<" << std::endl;
    pcl_ndt.setTransformationEpsilon(trans_eps);
    pcl_ndt.setStepSize(step_size);
    pcl_ndt.setResolution(ndt_res);
    pcl_ndt.setMaximumIterations(max_iter);
  } else if (_method_type == MethodType::use_cpu) {
    std::cout << ">> Use CPU NDT <<" << std::endl;
    cpu_ndt.setTransformationEpsilon(trans_eps);
    cpu_ndt.setStepSize(step_size);
    cpu_ndt.setResolution(ndt_res);
    cpu_ndt.setMaximumIterations(max_iter);
  }
#ifndef NO_OMP
  else if (_method_type == MethodType::use_omp) {
    std::cout << ">> Use OMP NDT <<" << std::endl;
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(
        new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    //ndt->setNumThreads(omp_get_num_threads());  //  设置最大线程, 注意：需要引入头文件omp.h
    ndt->setTransformationEpsilon(trans_eps);
    ndt->setStepSize(step_size);
    ndt->setResolution(ndt_res);
    ndt->setMaximumIterations(max_iter);
    // 注意：此处设置ndt参数之后才能赋值给registration,否则后面无法使用getFinalScore函数！
    omp_ndt = ndt;
  }
#endif
  else {
    ROS_ERROR("Please Define _method_type to conduct NDT");
  }

  Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();  // 保存GPS信号的变量

  // 这里一般需要设定初始的雷达高度init_z
  privateHandle.param<double>("init_x", _tf_x, 0);
  privateHandle.param<double>("init_y", _tf_y, 0);
  privateHandle.param<double>("init_z", _tf_z, 0);
  privateHandle.param<double>("init_roll", _tf_roll, 0);
  privateHandle.param<double>("init_pitch", _tf_pitch, 0);
  privateHandle.param<double>("init_yaw", _tf_yaw, 0);
  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;
  std::cout << std::endl;
  std::cout << "******************************************" << std::endl;

  cloud_keyposes_3d_.reset(new pcl::PointCloud<PointT>());
  cloud_keyposes_6d_.reset(new pcl::PointCloud<PointXYZIRPYT>());

  // 根据初始设定的_tf,_roll等,初始化tl_btol rot_x_btol等
  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation  // tl_btol是初始设定的
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());

  // 初始化tf_btol == 根据初始化得到的tl_btol等初始化tf_btol -----以获得初始变换矩阵 ---当车辆初始的起点不在预定义的globalMap原点时,就需要tf_btol了
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
  tf_ltob = tf_btol.inverse();  // 将tf_btol取逆 --因为我们后面配准的时候都是相对于localMap的原点的,因此tf_ltob.inv将作为补偿的矩阵

  // 定义速度值 --包括实际速度值,和imu取到的速度值
  current_velocity_x = 0.0;
  current_velocity_y = 0.0;
  current_velocity_z = 0.0;

  current_velocity_imu_x = 0.0;
  current_velocity_imu_y = 0.0;
  current_velocity_imu_z = 0.0;

  ROS_INFO("params init");
}

void LidarMapping::run() {
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

    if (!cloudBuf.empty()) {
      std::cout << "cloud buffer size : " << cloudBuf.size() << std::endl;
      mutex_lock.lock();
      pcl::PointCloud<pcl::PointXYZI> tmp;
      pcl::fromROSMsg(cloudBuf.front(), tmp);
      ros::Time current_scan_time = (cloudBuf.front()).header.stamp;
      cloudBuf.pop(); // pop掉
      mutex_lock.unlock();

      // 应该去查询最近的imu数据并清空
      if (_use_imu) {
        std::lock_guard<std::mutex> lock(imu_data_mutex);
        auto imu_iter = imu_data.begin();
        for (imu_iter; imu_iter != imu_data.end(); imu_iter++) {
          if (current_scan_time < (*imu_iter)->header.stamp) {
            break;
          }
          sensor_msgs::ImuConstPtr data = (*imu_iter);
          imu_info(*data);
        }
        imu_data.erase(imu_data.begin(), imu_iter);
      }
      // std::cout << "imu buffer size : " << imu_data.size() << std::endl;

      // 取queue里面的第一条数据
      if (tmp.empty()) {
        ROS_ERROR("Scan is empty !!!");
        continue;
      }

      //remove NAN
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(tmp, tmp, indices);

      // 去除远近的点云
      double r;
      pcl::PointCloud<pcl::PointXYZI> scan;
      for (auto point:tmp.points) {
        r = std::sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));
        if (min_scan_range < r && r < max_scan_range) {
          scan.points.push_back(point);
        }
      }
      // 匹配
      process_cloud(scan, current_scan_time);
    }
  }

  // ctrl+c关闭终端时保存地图
  save_map();
}

void LidarMapping::process_cloud(const pcl::PointCloud<pcl::PointXYZI> &scan, const ros::Time &current_scan_time) {

  t_localizer.setIdentity();
  t_base_link.setIdentity();

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  filtered_scan_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  transformed_scan_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());

  // downsize scan
  downSizeFilterKeyFrames.setInputCloud(scan_ptr);
  downSizeFilterKeyFrames.filter(*filtered_scan_ptr);

  ndt_start = ros::Time::now();

  // 第一帧直接加入地图
  if (initial_scan_loaded == 0) {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);  // tf_btol为初始变换矩阵
    localmap += *transformed_scan_ptr;
    globalmap += *transformed_scan_ptr;
    initial_scan_loaded = 1;
  }

  // 用以保存降采样后的localmap
  pcl::PointCloud<pcl::PointXYZI>::Ptr localmap_ptr(new pcl::PointCloud<pcl::PointXYZI>(localmap));
  if (_method_type == MethodType::use_pcl) {
    pcl_ndt.setInputSource(filtered_scan_ptr);
    pcl_ndt.setInputTarget(localmap_ptr);
  } else if (_method_type == MethodType::use_cpu) {
    cpu_ndt.setInputSource(filtered_scan_ptr);
    cpu_ndt.setInputTarget(localmap_ptr);
  } else if (_method_type == MethodType::use_omp) {
    omp_ndt->setInputSource(filtered_scan_ptr);
    omp_ndt->setInputTarget(localmap_ptr);
  } else {
    ROS_ERROR("Please Define _method_type to conduct NDT");
    exit(1);
  }

  // 计算初始姿态，上一帧点云结果+传感器畸变
  guess_pose.x = previous_pose.x + diff_pose.x;  // 初始时diff_x等都为0
  guess_pose.y = previous_pose.y + diff_pose.y;
  guess_pose.z = previous_pose.z + diff_pose.z;
  guess_pose.roll = previous_pose.roll;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.yaw = previous_pose.yaw + diff_pose.yaw;
  guess_pose.updateT();

  // 根据是否使用imu和odom, calculate diff
  Pose guess_pose_for_ndt;
  if (_use_imu && _use_odom) {
    imu_odom_calc(current_scan_time);
    guess_pose_for_ndt = guess_pose_imu_odom;
  } else if (_use_imu && !_use_odom) {
    imu_calc(current_scan_time);
    guess_pose_for_ndt = guess_pose_imu;
  } else if (!_use_imu && _use_odom) {
    odom_calc(current_scan_time);
    guess_pose_for_ndt = guess_pose_odom;
  } else {
    guess_pose_for_ndt = guess_pose;
  }

  // start3 以下:根据guess_pose_for_ndt 来计算初始变换矩阵guess_init -- 针对TargetSource
  Eigen::Matrix4f init_guess = guess_pose_for_ndt.t.matrix() * tf_btol;  // tf_btol

  t3_end = ros::Time::now();
  d3 = t3_end - t3_start;

  // 用以保存ndt转换后的点云,align参数
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (_method_type == MethodType::use_pcl) {
    pcl_ndt.align(*output_cloud, init_guess);  // pcl::aligin 需传入转换后的点云(容器),估计变换
    fitness_score = pcl_ndt.getFitnessScore();
    t_localizer = pcl_ndt.getFinalTransformation();  // t_localizer为ndt变换得到的最终变换矩阵(即source和target之间的变换)
    has_converged = pcl_ndt.hasConverged();
    final_num_iteration = pcl_ndt.getFinalNumIteration();
  } else if (_method_type == MethodType::use_cpu) {
    cpu_ndt.align(init_guess);            // cpu::align 只需要传入估计变换 --建图的时候传入估计变换,定位matching的时候传入空的单位Eigen
    fitness_score = cpu_ndt.getFitnessScore();
    t_localizer = cpu_ndt.getFinalTransformation();
    has_converged = cpu_ndt.hasConverged();
    final_num_iteration = cpu_ndt.getFinalNumIteration();
  } else if (_method_type == MethodType::use_omp) {
    omp_ndt->align(*output_cloud, init_guess);
    fitness_score = omp_ndt->getFitnessScore();
    t_localizer = omp_ndt->getFinalTransformation();
    has_converged = omp_ndt->hasConverged();
    final_num_iteration = omp_ndt->getFinalNumIteration();
  }
  ndt_end = ros::Time::now();

  // bask_link 需要排除掉全局起始点偏移造成的影响，全局起点偏移就是雷达起始有个高度和yaw偏角
  // t_localizer是相对位姿,t_base_link对应的是全局位姿
  t_base_link = t_localizer * tf_ltob;

  localizer_pose.updatePose(t_localizer);
  current_pose.updatePose(t_base_link);

  // 当前帧转换到全局地图上
  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  // 以下:对current_pose做tf变换,
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  // Calculate the shift between added_pos and current_pos. added_pose 将一直定位于localMap的原点
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  if (shift >= min_add_scan_shift) {
    downSizeFilterLocalmap.setInputCloud(transformed_scan_ptr);
    downSizeFilterLocalmap.filter(*transformed_scan_ptr);

    localmap_size += shift;
    odom_size += shift;

    // 只有在fitnessscore状态好的情况下才选取作为关键帧加入到localmap中
    localmap += *transformed_scan_ptr; // localmap内的距离达到阈值就清空,并重新从0开始一帧一帧添加点云
    submap += *transformed_scan_ptr;
    added_pose = current_pose;

    // 只是说map更新了,因此target也要更新,不要落后太多
    // 注意:此时加入的target:map_ptr并不包括刚加入点云的transformed_scan_ptr
    if (_method_type == MethodType::use_pcl)
      pcl_ndt.setInputTarget(localmap_ptr);
    else if (_method_type == MethodType::use_cpu) {
      if (_incremental_voxel_update)
        cpu_ndt.updateVoxelGrid(transformed_scan_ptr);
      else
        cpu_ndt.setInputTarget(localmap_ptr);
    } else if (_method_type == MethodType::use_omp)
      omp_ndt->setInputTarget(localmap_ptr);
  }

  // 当局部地图内的距离大于阈值,则清空localmap
  if (localmap_size >= max_localmap_size) {
    localmap = submap;
    submap.clear();
    localmap_size = 0.0;
  }

  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of localmap points: " << localmap.size() << " points." << std::endl;
  std::cout << "global_map size : " << globalmap.size() << " points." << std::endl;
  std::cout << "Aligned Time: " << (ndt_end - ndt_start) * 1000 << " ms" << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << final_num_iteration << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "scan shift: " << shift << std::endl;
  std::cout << "localmap shift: " << localmap_size << std::endl;
  std::cout << "global path: " << odom_size << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

  // choose key frames
  save_keyframes();

  // publish cloud
  publish_cloud();
}

bool LidarMapping::save_keyframes() {
  if (cloud_keyposes_3d_->points.empty()) {
    previous_pose = current_pose;
  } else {
    const auto &pre_pose = cloud_keyposes_6d_->points[cloud_keyposes_3d_->points.size() - 1];

    // 关集帧之间的距离大于0.5米才将其加到位姿图中
    if (std::pow(current_pose.x - pre_pose.x, 2) +
        std::pow(current_pose.y - pre_pose.y, 2) +
        std::pow(current_pose.z - pre_pose.z, 2) <
        keyframe_dist * keyframe_dist) {
      ROS_WARN("frames are too close...");
      return false;
    }
//    if (fitness_score > 1.0) {
//      ROS_WARN("bad tranforme...");
//      return false;
//    }
  }

  // 保存关键帧和位姿, 更新current_pose
  // current_pose 对应的是全局下的坐标!
  PointT this_pose_3d;
  PointXYZIRPYT this_pose_6d;

  this_pose_6d.x = this_pose_3d.x = current_pose.x;
  this_pose_6d.y = this_pose_3d.y = current_pose.y;
  this_pose_6d.z = this_pose_3d.z = current_pose.z;
  this_pose_6d.roll = current_pose.roll;
  this_pose_6d.pitch = current_pose.pitch;
  this_pose_6d.yaw = current_pose.yaw;
  this_pose_6d.time = current_scan_time.toSec();
  this_pose_6d.intensity = this_pose_3d.intensity = cloud_keyposes_3d_->points.size();

  cloud_keyposes_3d_->points.push_back(this_pose_3d);
  cloud_keyposes_6d_->points.push_back(this_pose_6d);

  // 存储关键帧
  pcl::PointCloud<PointT>::Ptr cur_keyframe(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*transformed_scan_ptr, *cur_keyframe);
  for (auto &p : cur_keyframe->points) {
    p.intensity = this_pose_3d.intensity;
  }
  cloud_keyframes.push_back(cur_keyframe);
  globalmap += *transformed_scan_ptr;  // update global map
  ROS_INFO("saveKeyframesAndFactor: %d points", cur_keyframe->points.size());

  // 根据current和previous两帧之间的scantime,以及两帧之间的位置,计算两帧之间的变化
  scan_duration = current_scan_time - previous_scan_time;
  double secs = scan_duration.toSec();

  // Calculate the offset (curren_pos - previous_pos)
  diff_pose.x = current_pose.x - previous_pose.x;
  diff_pose.y = current_pose.y - previous_pose.y;
  diff_pose.z = current_pose.z - previous_pose.z;
  diff_pose.yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
  diff = sqrt(diff_pose.x * diff_pose.x + diff_pose.y * diff_pose.y + diff_pose.z * diff_pose.z);

  // estimate velocity
  current_velocity_x = diff_pose.x / secs;
  current_velocity_y = diff_pose.y / secs;
  current_velocity_z = diff_pose.z / secs;

  // 修正imu速度
  current_velocity_imu_x = current_velocity_x;
  current_velocity_imu_y = current_velocity_y;
  current_velocity_imu_z = current_velocity_z;

  current_pose_imu = current_pose_odom = current_pose_imu_odom = current_pose;

  // Update position and posture. current_pos -> previous_pos
  previous_pose = current_pose;
  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  offset_imu_pose.init();
  offset_odom_pose.init();
  offset_imu_odom_pose.init();

  return true;
}

void LidarMapping::imu_callback(const sensor_msgs::Imu::Ptr &input) {

  std::lock_guard<std::mutex> lock(imu_data_mutex);
  if (_imu_upside_down)  // _imu_upside_down指示是否进行imu的正负变换
    imuUpSideDown(input);

  if (_use_imu) {
    imu_data.push_back(input);
  }

  //  mutex_lock.lock();
  //imuBuf.push(*input);
  //  mutex_lock.unlock();

  /*const ros::Time current_time = input->header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();

  // 解析imu消息,获得rpy
  double imu_roll, imu_pitch, imu_yaw;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(input->orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

  imu_roll = warpToPmPi(imu_roll);  // 调整,防止超过PI(180°)  --保持在±180°内
  imu_pitch = warpToPmPi(imu_pitch);
  imu_yaw = warpToPmPi(imu_yaw);

  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
  const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
  const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);
  //

  imu.header = input->header;
  imu.linear_acceleration.x = input->linear_acceleration.x;
  // imu.linear_acceleration.y = input->linear_acceleration.y;
  // imu.linear_acceleration.z = input->linear_acceleration.z;
  imu.linear_acceleration.y = 0;
  imu.linear_acceleration.z = 0;

  if (diff_time != 0) {
    imu.angular_velocity.x = diff_imu_roll / diff_time;
    imu.angular_velocity.y = diff_imu_pitch / diff_time;
    imu.angular_velocity.z = diff_imu_yaw / diff_time;
  } else {
    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;
  }

  imu_calc(input->header.stamp);

  previous_time = current_time;
  previous_imu_roll = imu_roll;
  previous_imu_pitch = imu_pitch;
  previous_imu_yaw = imu_yaw;*/
}

void LidarMapping::odom_callback(const nav_msgs::Odometry::ConstPtr &input) {
  odom = *input;
  odom_calc(input->header.stamp);

//  mutex_lock.lock();
//  odomBuf.push(*input);
//  mutex_lock.unlock();
}

void LidarMapping::points_callback(const sensor_msgs::PointCloud2::ConstPtr &input) {
  mutex_lock.lock();
  cloudBuf.push(*input);
  mutex_lock.unlock();
}

void LidarMapping::imu_odom_calc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();  // static声明的变量只会在第一次使用时被声明,因此不会被覆盖

  // imu信息处理,计算 -- imu只使用陀螺仪,即 只输出转角信息roll,pitch,yaw
  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu_odom.roll += diff_imu_roll;  // 更新current_pose_imu_odom相关,作为历史记录
  current_pose_imu_odom.pitch += diff_imu_pitch;
  current_pose_imu_odom.yaw += diff_imu_yaw;

  // odom信息处理,计算 -- xyz移动距离的计算,融合odom的速度(位移)信息和imu的转角信息
  double diff_distance = odom.twist.twist.linear.x * diff_time;
/*  offset_imu_odom_x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
  offset_imu_odom_y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
  offset_imu_odom_z += diff_distance * sin(-current_pose_imu_odom.pitch);

  offset_imu_odom_roll += diff_imu_roll;
  offset_imu_odom_pitch += diff_imu_pitch;
  offset_imu_odom_yaw += diff_imu_yaw;*/

  offset_imu_odom_pose.x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
  offset_imu_odom_pose.y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
  offset_imu_odom_pose.z += diff_distance * sin(-current_pose_imu_odom.pitch);

  offset_imu_odom_pose.roll += diff_imu_roll;
  offset_imu_odom_pose.pitch += diff_imu_pitch;
  offset_imu_odom_pose.yaw += diff_imu_yaw;

  // ==> 最终的目的是融合imu和odom输出一个guess_pose
  // 注:guess_pose是在previous_pose基础上叠加一个offset,包括xyz的和rpy的
  // xyz的offset需要融合imu的转角和odom的速度(位移)
  // rpy的offset直接采用imu的rpy偏差值
/*  guess_pose_imu_odom.x = previous_pose.x + offset_imu_odom_x;
  guess_pose_imu_odom.y = previous_pose.y + offset_imu_odom_y;
  guess_pose_imu_odom.z = previous_pose.z + offset_imu_odom_z;
  guess_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_roll;
  guess_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pitch;
  guess_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_yaw;*/

  guess_pose_imu_odom.x = previous_pose.x + offset_imu_odom_pose.x;
  guess_pose_imu_odom.y = previous_pose.y + offset_imu_odom_pose.y;
  guess_pose_imu_odom.z = previous_pose.z + offset_imu_odom_pose.z;
  guess_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_pose.roll;
  guess_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pose.pitch;
  guess_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_pose.yaw;

  guess_pose_odom.updateT();
  previous_time = current_time;
}

void LidarMapping::imu_calc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu.roll += diff_imu_roll;
  current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  // 对imu由于不平衡造成的补偿问题,在这里解决
  // start1
  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
      std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
      std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

  double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
  double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
  double accZ = accZ2;
  // end1

  // imu计算xyz方向上的偏移,初始速度用的imu_x为slam计算获得,后面再加上考虑加速度以及时间参数,获得较为准确的距离偏移
/*  offset_imu_x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0; */

  offset_imu_pose.x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_pose.y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_pose.z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  current_velocity_imu_x += accX * diff_time;  // imu的速度值会通过slam进行修正,以避免累计误差
  current_velocity_imu_y += accY * diff_time;  // 或者说imu计算时所用的速度并不是用imu得到的,而是用slam得到的
  current_velocity_imu_z += accZ * diff_time;    // imu所提供的参数,主要用来计算角度上的偏移,以及加速度导致的距离上的偏移!@

/*  offset_imu_roll += diff_imu_roll;
  offset_imu_pitch += diff_imu_pitch;
  offset_imu_yaw += diff_imu_yaw;*/

  offset_imu_pose.roll += diff_imu_roll;
  offset_imu_pose.pitch += diff_imu_pitch;
  offset_imu_pose.yaw += diff_imu_yaw;

/*  guess_pose_imu.x = previous_pose.x + offset_imu_x;
  guess_pose_imu.y = previous_pose.y + offset_imu_y;
  guess_pose_imu.z = previous_pose.z + offset_imu_z;
  guess_pose_imu.roll = previous_pose.roll + offset_imu_roll;
  guess_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
  guess_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;*/

  guess_pose_imu.x = previous_pose.x + offset_imu_pose.x;
  guess_pose_imu.y = previous_pose.y + offset_imu_pose.y;
  guess_pose_imu.z = previous_pose.z + offset_imu_pose.z;
  guess_pose_imu.roll = previous_pose.roll + offset_imu_pose.roll;
  guess_pose_imu.pitch = previous_pose.pitch + offset_imu_pose.pitch;
  guess_pose_imu.yaw = previous_pose.yaw + offset_imu_pose.yaw;
  guess_pose.updateT();

  previous_time = current_time;
}

void LidarMapping::odom_calc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
  double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
  double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

  current_pose_odom.roll += diff_odom_roll;
  current_pose_odom.pitch += diff_odom_pitch;
  current_pose_odom.yaw += diff_odom_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  /*offset_odom_x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  offset_odom_y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  offset_odom_z += diff_distance * sin(-current_pose_odom.pitch);

  offset_odom_roll += diff_odom_roll;
  offset_odom_pitch += diff_odom_pitch;
  offset_odom_yaw += diff_odom_yaw;*/

  offset_odom_pose.x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  offset_odom_pose.y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  offset_odom_pose.z += diff_distance * sin(-current_pose_odom.pitch);

  offset_odom_pose.roll += diff_odom_roll;
  offset_odom_pose.pitch += diff_odom_pitch;
  offset_odom_pose.yaw += diff_odom_yaw;

  guess_pose_odom.x = previous_pose.x + offset_odom_pose.x;
  guess_pose_odom.y = previous_pose.y + offset_odom_pose.y;
  guess_pose_odom.z = previous_pose.z + offset_odom_pose.z;
  guess_pose_odom.roll = previous_pose.roll + offset_odom_pose.roll;
  guess_pose_odom.pitch = previous_pose.pitch + offset_odom_pose.pitch;
  guess_pose_odom.yaw = previous_pose.yaw + offset_odom_pose.yaw;
  guess_pose_odom.updateT();

  previous_time = current_time;
}

void LidarMapping::imuUpSideDown(const sensor_msgs::Imu::Ptr input) {
  double input_roll, input_pitch, input_yaw;

  tf::Quaternion input_orientation;
  tf::quaternionMsgToTF(input->orientation, input_orientation);
  tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);

  input->angular_velocity.x *= -1;
  input->angular_velocity.y *= -1;
  input->angular_velocity.z *= -1;

  input->linear_acceleration.x *= -1;
  input->linear_acceleration.y *= -1;
  input->linear_acceleration.z *= -1;

  input_roll *= -1;
  input_pitch *= -1;
  input_yaw *= -1;

//  input_yaw += M_PI/2;
  input->orientation = tf::createQuaternionMsgFromRollPitchYaw(input_roll, input_pitch, input_yaw);
}

void LidarMapping::imu_info(const sensor_msgs::Imu &input) {
  const ros::Time current_time = input.header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();

  // 解析imu消息,获得rpy
  double imu_roll, imu_pitch, imu_yaw;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(input.orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

  imu_roll = warpToPmPi(imu_roll);  // 调整,防止超过PI(180°)  --保持在±180°内
  imu_pitch = warpToPmPi(imu_pitch);
  imu_yaw = warpToPmPi(imu_yaw);

  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
  const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
  const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);

  imu.header = input.header;
  imu.linear_acceleration.x = input.linear_acceleration.x;
  // imu.linear_acceleration.y = input.linear_acceleration.y;
  // imu.linear_acceleration.z = input.linear_acceleration.z;
  imu.linear_acceleration.y = 0;
  imu.linear_acceleration.z = 0;

  if (diff_time != 0) {
    imu.angular_velocity.x = diff_imu_roll / diff_time;
    imu.angular_velocity.y = diff_imu_pitch / diff_time;
    imu.angular_velocity.z = diff_imu_yaw / diff_time;
  } else {
    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;
  }

  imu_calc(input.header.stamp);

  previous_time = current_time;
  previous_imu_roll = imu_roll;
  previous_imu_pitch = imu_pitch;
  previous_imu_yaw = imu_yaw;
}

void LidarMapping::odom_info(const nav_msgs::Odometry &input) {
  odom_calc(input.header.stamp);
}

void LidarMapping::publish_cloud() {

  // publish odom
  if (current_odom_pub.getNumSubscribers()) {
    nav_msgs::Odometry odom;
    odom.header.stamp = current_scan_time;
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = current_pose.x;
    odom.pose.pose.position.y = current_pose.y;
    odom.pose.pose.position.z = current_pose.z;

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = current_velocity_x;
    odom.twist.twist.linear.y = current_velocity_y;
    odom.twist.twist.angular.z = current_velocity_z;
    current_odom_pub.publish(odom);
  }
  // 发布关键帧位姿
  if (keyposes_pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_keyposes_3d_, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    keyposes_pub.publish(msg);
  }

  // 实时的点云也发布
  if (current_points_pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2::Ptr pointcloud_current_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*transformed_scan_ptr, *pointcloud_current_ptr);
    pointcloud_current_ptr->header.frame_id = "map";
    current_points_pub.publish(*pointcloud_current_ptr);
  }

  // 发布globalmap
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(globalmap, *target_cloud);
  pcl::copyPointCloud(localmap, *local_cloud);
  downSizeFilterGlobalMap.setInputCloud(target_cloud);
  downSizeFilterGlobalMap.filter(*target_cloud);
  downSizeFilterLocalmap.setInputCloud(local_cloud);
  downSizeFilterLocalmap.filter(*local_cloud);

  if (ndt_map_pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*target_cloud, *map_msg_ptr);
    map_msg_ptr->header.frame_id = "map";
    ndt_map_pub.publish(*map_msg_ptr);
  }
  target_cloud->clear();
  if (local_map_pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2::Ptr localmap_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*local_cloud, *localmap_msg_ptr);
    localmap_msg_ptr->header.frame_id = "map";
    local_map_pub.publish(*localmap_msg_ptr);
  }
  local_cloud->clear();


/*  // 发布全局地图，非最终地图，只是里程计关键帧拼接而成的
  if (pub_globalmap_.getNumSubscribers() > 0) {
    int num_points = 0;
    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<PointT>::Ptr globalmap_ptr(new pcl::PointCloud<PointT>());
    Eigen::Matrix4f T(Eigen::Matrix4f::Identity());
    // 把关键帧拼接而成，这里关键帧及其对应的位姿都存储好了
    for (int i = 0; i < cloud_keyframes.size(); ++i) {
      pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
      tmp = transformPointCloud(cloud_keyframes[i], cloud_keyposes_6d_->points[i]);
      *globalmap_ptr += *tmp;
      num_points += tmp->points.size();
    }

    // downsample visualized points
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalmap_ptr);
    downSizeFilterGlobalMapKeyFrames.filter(*globalmap_ptr);

    pcl::toROSMsg(*globalmap_ptr, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_globalmap_.publish(msg);
    ROS_INFO("Map Size: %d points", num_points);
  }*/

}

void LidarMapping::save_map() {
  // 关闭终端时保存地图
  std::stringstream ss;
  ss << int(ros::Time::now().toSec());
  std::string stamp = ss.str();

  std::string pcd_filename = map_saved_dir + "finalCLoud_" + stamp + ".pcd";
  if (!globalmap.empty()) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(globalmap, *map_cloud);
    pcl::VoxelGrid<pcl::PointXYZI> map_grid_filter;
    map_grid_filter.setLeafSize(1.0, 1.0, 1.0);
    map_grid_filter.setInputCloud(map_cloud);
    map_grid_filter.filter(*map_cloud);

    if (pcl::io::savePCDFileASCII(pcd_filename, *map_cloud) == -1) {
      std::cout << "Failed saving " << pcd_filename << "." << std::endl;
    }
    std::cout << "Saved globalmap " << pcd_filename << " (" << map_cloud->size() << " points)" << std::endl;
  }

  // 优化后的位姿也保存一份
  pcl::PointCloud<PointT>::Ptr poses(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*cloud_keyposes_3d_, *poses);
  poses->width = poses->points.size();
  poses->height = 1;
  poses->is_dense = false;

  pcl::io::savePCDFile(map_saved_dir + "trajectory_" + stamp + ".pcd", *poses);

}
