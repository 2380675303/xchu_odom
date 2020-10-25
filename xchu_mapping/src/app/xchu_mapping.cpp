
/**
 * @file xchu_mapping.cpp
 * @author xiangcheng hu (2022087641@qq.com)
 * @brief
 * @version 1.0
 * @date 2020-09-20
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <xchu_mapping/xchu_mapping.h>

LidarMapping::LidarMapping() : privateHandle("~") {

  param_initial(privateHandle);

  ndt_map_pub = privateHandle.advertise<sensor_msgs::PointCloud2>("/ndt_map", 100);
  local_map_pub = privateHandle.advertise<sensor_msgs::PointCloud2>("/local_map", 100);
  current_points_pub = privateHandle.advertise<sensor_msgs::PointCloud2>("/current_points", 100);

  current_odom_pub = privateHandle.advertise<nav_msgs::Odometry>("/current_odom", 100);

  points_sub = privateHandle.subscribe(_lidar_topic, 100, &LidarMapping::points_callback, this);
  odom_sub = privateHandle.subscribe(_odom_topic, 1000, &LidarMapping::odom_callback, this);
  imu_sub = privateHandle.subscribe(_imu_topic, 4000, &LidarMapping::imu_callback, this);
}

void LidarMapping::param_initial(ros::NodeHandle &privateHandle) {

  _method_type = MethodType::use_cpu;

  // 定义各种差异值(两次采集数据之间的差异,包括点云位置差异,imu差异,odom差异,imu-odom差异)
  diff = 0.0;
  diff_x = diff_y = diff_z = 0.0;  // current_pose - previous_pose // 定义两帧点云差异值 --以确定是否更新点云等

  // 定义速度值 --包括实际速度值,和imu取到的速度值
  current_velocity_x = 0.0;
  current_velocity_y = 0.0;
  current_velocity_z = 0.0;

  current_velocity_imu_x = 0.0;
  current_velocity_imu_y = 0.0;
  current_velocity_imu_z = 0.0;

  Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();  // 保存GPS信号的变量
  _use_imu = false;
  _use_odom = false;
  _imu_upside_down = false;  // 用以解决坐标系方向(正负变换)问题 (比如x变更为-x等)
  _incremental_voxel_update = false;  // ??????????
  initial_scan_loaded = 0;  // 用以确定是否为第一帧点云(第一帧点云不做匹配,直接添加到地图中去)


  std::cout << "******************************************" << std::endl;
  privateHandle.param<float>("ndt_resolution", ndt_res, 1.0);
  privateHandle.param<double>("ndt_step_size", step_size, 0.1);
  privateHandle.param<double>("ndt_trans_eps", trans_eps, 0.01);
  privateHandle.param<int>("ndt_max_iter", max_iter, 30);
  privateHandle.param<double>("voxel_leaf_size", voxel_leaf_size, 0.01);
  privateHandle.param<double>("min_scan_range", min_scan_range, 5);
  privateHandle.param<double>("max_scan_range", max_scan_range, 80);
  privateHandle.param<double>("min_add_scan_shift", min_add_scan_shift, 0.5);
  privateHandle.param<double>("max_submap_size", max_submap_size, 400);
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
  std::cout << "max_submap_size: " << max_submap_size << std::endl;
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

  // 注意自定义类型格式在获取参数时的实现方式 !!
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
  } else if (_method_type == MethodType::use_gpu) {
    std::cout << ">> Use GPU NDT <<" << std::endl;
    //    gpu_ndt.setTransformationEpsilon(trans_eps);
    //    gpu_ndt.setStepSize(step_size);
    //    gpu_ndt.setResolution(ndt_res);
    //    gpu_ndt.setMaximumIterations(max_iter);
  } else if (_method_type == MethodType::use_omp) {
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
  } else {
    ROS_ERROR("Please Define _method_type to conduct NDT");
  }

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

  // 根据初始设定的_tf,_roll等,初始化tl_btol rot_x_btol等
  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation  // tl_btol是初始设定的
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());

  // 初始化tf_btol == 根据初始化得到的tl_btol等初始化tf_btol -----以获得初始变换矩阵 ---当车辆初始的起点不在预定义的globalMap原点时,就需要tf_btol了
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
  tf_ltob = tf_btol.inverse();  // 将tf_btol取逆 --因为我们后面配准的时候都是相对于localMap的原点的,因此tf_ltob.inv将作为补偿的矩阵

  map.header.frame_id = "map";

  previous_pose.x = 0.0;
  previous_pose.y = 0.0;
  previous_pose.z = 0.0;
  previous_pose.roll = 0.0;
  previous_pose.pitch = 0.0;
  previous_pose.yaw = 0.0;

  ndt_pose.x = 0.0;
  ndt_pose.y = 0.0;
  ndt_pose.z = 0.0;
  ndt_pose.roll = 0.0;
  ndt_pose.pitch = 0.0;
  ndt_pose.yaw = 0.0;

  current_pose.x = 0.0;
  current_pose.y = 0.0;
  current_pose.z = 0.0;
  current_pose.roll = 0.0;
  current_pose.pitch = 0.0;
  current_pose.yaw = 0.0;

  current_pose_imu.x = 0.0;
  current_pose_imu.y = 0.0;
  current_pose_imu.z = 0.0;
  current_pose_imu.roll = 0.0;
  current_pose_imu.pitch = 0.0;
  current_pose_imu.yaw = 0.0;

  guess_pose.x = 0.0;
  guess_pose.y = 0.0;
  guess_pose.z = 0.0;
  guess_pose.roll = 0.0;
  guess_pose.pitch = 0.0;
  guess_pose.yaw = 0.0;

  added_pose.x = 0.0;
  added_pose.y = 0.0;
  added_pose.z = 0.0;
  added_pose.roll = 0.0;
  added_pose.pitch = 0.0;
  added_pose.yaw = 0.0;

  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_yaw = 0.0;

  offset_imu_x = 0.0;
  offset_imu_y = 0.0;
  offset_imu_z = 0.0;
  offset_imu_roll = 0.0;
  offset_imu_pitch = 0.0;
  offset_imu_yaw = 0.0;

  offset_odom_x = 0.0;
  offset_odom_y = 0.0;
  offset_odom_z = 0.0;
  offset_odom_roll = 0.0;
  offset_odom_pitch = 0.0;
  offset_odom_yaw = 0.0;

  offset_imu_odom_x = 0.0;
  offset_imu_odom_y = 0.0;
  offset_imu_odom_z = 0.0;
  offset_imu_odom_roll = 0.0;
  offset_imu_odom_pitch = 0.0;
  offset_imu_odom_yaw = 0.0;

  ROS_INFO("params init");
}

void LidarMapping::run() {
  std::deque<sensor_msgs::Imu> imu_data_buff;
  std::deque<nav_msgs::Odometry> odom_data_buff;
  std::deque<sensor_msgs::PointCloud2> cloud_data_buff;

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
//    rate.sleep();

    // 往buffer中装数据

//    if (imuBuf.size() > 0) {
//      imu_data_buff.insert(imu_data_buff.end(), imuBuf.begin(), imuBuf.end());
//      imuBuf.clear();
//    }
//    if (odomBuf.size() > 0) {
//      odom_data_buff.insert(odom_data_buff.end(), odomBuf.begin(), odomBuf.end());
//      odomBuf.clear();
//    }
    mutex_lock.lock();
    if (cloudBuf.size() > 0) {
      cloud_data_buff.insert(cloud_data_buff.end(), cloudBuf.begin(), cloudBuf.end());
      cloudBuf.clear();
    }
    mutex_lock.unlock();

//    std::cout << "cloud imu odom buffer size : " << cloud_data_buff.size() << ", " << imu_data_buff.size()
//              << ", " << odom_data_buff.size() << std::endl;

    while (cloud_data_buff.size() > 0) {
      sensor_msgs::PointCloud2 cloud_msg = cloud_data_buff.front(); //  处理先进来的数据
      const auto cloud_time = cloud_msg.header.stamp;
      cloud_data_buff.pop_front();

      /*  sensor_msgs::Imu imu_msg;
        ros::Time imu_time;
        if (imu_data_buff.size() > 0 && _use_imu) {
          imu_msg = imu_data_buff.front();
          imu_time = imu_msg.header.stamp;
          imu_data_buff.pop_front();
          imu_info(imu_msg);
        }
        nav_msgs::Odometry odom_msg;
        ros::Time odom_time;
        if (odom_data_buff.size() > 0 && _use_odom) {
          odom_msg = odom_data_buff.front();
          odom_time = imu_msg.header.stamp;
          odom_data_buff.pop_front();
          odom_info(odom_msg);
        }

        if (imu_data_buff.size() > 10000 || odom_data_buff.size() > 5000) {
          imu_data_buff.clear();
          odom_data_buff.clear();
        }*/

      pcl::PointCloud<pcl::PointXYZI> tmp;
      pcl::fromROSMsg(cloud_msg, tmp);
      if (tmp.empty()) {
        ROS_ERROR("Scan is empty !!!");
        continue;
      }
      std::vector<int> indices;   //remove NAN
      pcl::removeNaNFromPointCloud(tmp, tmp, indices);
      ros::Time current_scan_time = cloud_msg.header.stamp;

      process_cloud(tmp, current_scan_time);
    }
  }

  // 关闭终端时保存地图
  std::stringstream ss;
  ss << int(ros::Time::now().toSec());
  std::string stamp = ss.str();

  std::string pcd_filename = map_saved_dir + "ndtmap_" + stamp + ".pcd";
  if (pcl::io::savePCDFileASCII(pcd_filename, globalmap) == -1) {
    std::cout << "Failed saving " << pcd_filename << "." << std::endl;
  }
  std::cout << "Saved globalmap " << pcd_filename << " (" << globalmap.size() << " points)" << std::endl;
}

void LidarMapping::process_cloud(const pcl::PointCloud<pcl::PointXYZI> &tmp, const ros::Time &current_scan_time) {
  ros::Time test_time_1 = ros::Time::now();
  double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

  tf::Quaternion q;
  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());

  static tf::TransformBroadcaster br;
  tf::Transform transform;

  for (auto point:tmp.points) {
    r = std::sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));
    if (min_scan_range < r && r < max_scan_range) {
      scan.points.push_back(point);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  ndt_start = ros::Time::now();
  if (initial_scan_loaded == 0) {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);  // tf_btol为初始变换矩阵
    map += *transformed_scan_ptr;
    dense_map += *transformed_scan_ptr;
    initial_scan_loaded = 1;
  }

  // Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);

  ros::Time test_time_2 = ros::Time::now();  // TODO:

  // map_ptr是map的一个指针
  // TODO:即map_ptr只是指向map,而并不是将map进行了拷贝
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));  // 重要作用,用以保存降采样后的全局地图

  if (_method_type == MethodType::use_pcl) {
    pcl_ndt.setInputSource(filtered_scan_ptr);
  } else if (_method_type == MethodType::use_cpu) {
    cpu_ndt.setInputSource(filtered_scan_ptr);
  } else if (_method_type == MethodType::use_gpu) {
  } else if (_method_type == MethodType::use_omp) {
    omp_ndt->setInputSource(filtered_scan_ptr);
  } else {
    ROS_ERROR("Please Define _method_type to conduct NDT");
    exit(1);
  }
  ros::Time test_time_3 = ros::Time::now();  // TODO:

  static bool is_first_map = true;    // 第一帧点云直接作为 target
  if (is_first_map) {
    ROS_INFO("add first map");
    if (_method_type == MethodType::use_pcl) {
      pcl_ndt.setInputTarget(map_ptr);
    } else if (_method_type == MethodType::use_cpu) {
      cpu_ndt.setInputTarget(map_ptr);
    } else if (_method_type == MethodType::use_gpu) {
      //      gpu_ndt.setInputTarget(map_ptr);
    } else if (_method_type == MethodType::use_omp) {
      omp_ndt->setInputTarget(map_ptr);
    }
    is_first_map = false;
  }

  guess_pose.x = previous_pose.x + diff_x;  // 初始时diff_x等都为0
  guess_pose.y = previous_pose.y + diff_y;
  guess_pose.z = previous_pose.z + diff_z;
  guess_pose.roll = previous_pose.roll;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.yaw = previous_pose.yaw + diff_yaw;

  // 根据是否使用imu和odom,按照不同方式更新guess_pose(xyz,or/and rpy)
  if (_use_imu && _use_odom)
    imu_odom_calc(current_scan_time);
  if (_use_imu && !_use_odom)
    imu_calc(current_scan_time);
  if (!_use_imu && _use_odom)
    odom_calc(current_scan_time);

  // start2 只是为了把上面不同方式的guess_pose都标准化成guess_pose_for_ndt,为了后续操作方便
  pose guess_pose_for_ndt;
  if (_use_imu && _use_odom)
    guess_pose_for_ndt = guess_pose_imu_odom;
  else if (_use_imu && !_use_odom)
    guess_pose_for_ndt = guess_pose_imu;
  else if (!_use_imu && _use_odom)
    guess_pose_for_ndt = guess_pose_odom;
  else
    guess_pose_for_ndt = guess_pose;
  // end2

  // start3 以下:根据guess_pose_for_ndt 来计算初始变换矩阵guess_init -- 针对TargetSource
  Eigen::AngleAxisf init_rotation_x(static_cast<const float &>(guess_pose_for_ndt.roll), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(static_cast<const float &>(guess_pose_for_ndt.pitch), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(static_cast<const float &>(guess_pose_for_ndt.yaw), Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(static_cast<const float &>(guess_pose_for_ndt.x),
                                        static_cast<const float &>(guess_pose_for_ndt.y),
                                        static_cast<const float &>(guess_pose_for_ndt.z));
  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;  // tf_btol
  // end3

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
    transformation_probability = pcl_ndt.getTransformationProbability();
  } else if (_method_type == MethodType::use_cpu) {
    cpu_ndt.align(init_guess);            // cpu::align 只需要传入估计变换 --建图的时候传入估计变换,定位matching的时候传入空的单位Eigen
    fitness_score = cpu_ndt.getFitnessScore();
    t_localizer = cpu_ndt.getFinalTransformation();
    has_converged = cpu_ndt.hasConverged();
    final_num_iteration = cpu_ndt.getFinalNumIteration();
  } else if (_method_type == MethodType::use_gpu) {
//    gpu_ndt.align(init_guess);  // ndt_gpu库的align,不传出配准后的点云 ---用法同cpu_ndt
//    fitness_score = gpu_ndt.getFitnessScore();
//    t_localizer = gpu_ndt.getFinalTransformation();
//    has_converged = gpu_ndt.hasConverged();
//    final_num_iteration = gpu_ndt.getFinalNumIteration();
  } else if (_method_type == MethodType::use_omp) {
//    omp_ndt.align(*output_cloud, init_guess);   // omp_ndt.align用法同pcl::ndt
//    fitness_score = omp_ndt.getFitnessScore();
//    t_localizer = omp_ndt.getFinalTransformation();
//    has_converged = omp_ndt.hasConverged();
//    final_num_iteration = omp_ndt.getFinalNumIteration();
    omp_ndt->align(*output_cloud, init_guess);   // omp_ndt.align用法同pcl::ndt
    fitness_score = omp_ndt->getFitnessScore();
    t_localizer = omp_ndt->getFinalTransformation();
    has_converged = omp_ndt->hasConverged();
    final_num_iteration = omp_ndt->getFinalNumIteration();
  }
  ndt_end = ros::Time::now();
  ros::Time test_time_4 = ros::Time::now();  // TODO:

  // bask_link 需要排除掉全局起始点偏移造成的影响
  // !!!!!!!!!!也即t_localizer对应的是当前局部地图;t_base_link对应的是全局的global地图!!!!!!!!!!!
  // 二者皆为4x4的齐次坐标表示
  t_base_link = t_localizer * tf_ltob;  // t_localizer为每个配准库所计算出的final_transformation !!!!!

  // 配准变换 --注意:
  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  tf::Matrix3x3 mat_l, mat_b;  // 用以根据齐次坐标下的旋转变换,来求rpy转换角度
  mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                 static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                 static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                 static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                 static_cast<double>(t_localizer(2, 2)));

  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update localizer_pose.  // 更新局部下的坐标
  localizer_pose.x = t_localizer(0, 3);
  localizer_pose.y = t_localizer(1, 3);
  localizer_pose.z = t_localizer(2, 3);
  mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

  // Update ndt_pose.  // 更新全局下的坐标
  ndt_pose.x = t_base_link(0, 3);
  ndt_pose.y = t_base_link(1, 3);
  ndt_pose.z = t_base_link(2, 3);
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

  // current_pose对应的当然是全局下的坐标!
  current_pose.x = ndt_pose.x;
  current_pose.y = ndt_pose.y;
  current_pose.z = ndt_pose.z;
  current_pose.roll = ndt_pose.roll;
  current_pose.pitch = ndt_pose.pitch;
  current_pose.yaw = ndt_pose.yaw;

  // 以下:对current_pose做变换, ---------tf变换 ????????
//		static tf::TransformBroadcaster br;  //
//		tf::Transform transform;             //
  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  // 根据current和previous两帧之间的scantime,以及两帧之间的位置,计算两帧之间的变化
  scan_duration = current_scan_time - previous_scan_time;
  double secs = scan_duration.toSec();

  // Calculate the offset (curren_pos - previous_pos)
  // *****************************************
  diff_x = current_pose.x - previous_pose.x;
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  current_velocity_x = diff_x / secs;
  current_velocity_y = diff_y / secs;
  current_velocity_z = diff_z / secs;

  current_pose_imu.x = current_pose.x;
  current_pose_imu.y = current_pose.y;
  current_pose_imu.z = current_pose.z;
  current_pose_imu.roll = current_pose.roll;
  current_pose_imu.pitch = current_pose.pitch;
  current_pose_imu.yaw = current_pose.yaw;

  current_pose_odom.x = current_pose.x;
  current_pose_odom.y = current_pose.y;
  current_pose_odom.z = current_pose.z;
  current_pose_odom.roll = current_pose.roll;
  current_pose_odom.pitch = current_pose.pitch;
  current_pose_odom.yaw = current_pose.yaw;

  current_pose_imu_odom.x = current_pose.x;
  current_pose_imu_odom.y = current_pose.y;
  current_pose_imu_odom.z = current_pose.z;
  current_pose_imu_odom.roll = current_pose.roll;
  current_pose_imu_odom.pitch = current_pose.pitch;
  current_pose_imu_odom.yaw = current_pose.yaw;

  current_velocity_imu_x = current_velocity_x;  // 修正imu速度
  current_velocity_imu_y = current_velocity_y;
  current_velocity_imu_z = current_velocity_z;
  // ************************************************

// Update position and posture. current_pos -> previous_pos
  // -----------------------------------------------
  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  offset_imu_x = 0.0;
  offset_imu_y = 0.0;
  offset_imu_z = 0.0;
  offset_imu_roll = 0.0;
  offset_imu_pitch = 0.0;
  offset_imu_yaw = 0.0;

  offset_odom_x = 0.0;
  offset_odom_y = 0.0;
  offset_odom_z = 0.0;
  offset_odom_roll = 0.0;
  offset_odom_pitch = 0.0;
  offset_odom_yaw = 0.0;

  offset_imu_odom_x = 0.0;
  offset_imu_odom_y = 0.0;
  offset_imu_odom_z = 0.0;
  offset_imu_odom_roll = 0.0;
  offset_imu_odom_pitch = 0.0;
  offset_imu_odom_yaw = 0.0;
  // ------------------------------------------------

  //  实时的点云也发布
  sensor_msgs::PointCloud2::Ptr pointcloud_current_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*transformed_scan_ptr, *pointcloud_current_ptr);
  pointcloud_current_ptr->header.frame_id = "map";
  current_points_pub.publish(*pointcloud_current_ptr);

  // Calculate the shift between added_pos and current_pos // 以确定是否更新全局地图
  // added_pose将一直定位于localMap的原点
  // ################################################################################
  ros::Time test_time_5 = ros::Time::now();
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  if (shift >= min_add_scan_shift) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_tomap;
    voxel_grid_filter_tomap.setLeafSize(voxel_leaf_size * 2, voxel_leaf_size * 2, voxel_leaf_size * 2);
    voxel_grid_filter_tomap.setInputCloud(transformed_scan_ptr);
    voxel_grid_filter_tomap.filter(*transformed_scan_ptr);

    submap_size += shift;
    odom_size += shift;
    map += *transformed_scan_ptr;

    globalmap += *transformed_scan_ptr;
    submap += *transformed_scan_ptr;
    added_pose.x = current_pose.x;
    added_pose.y = current_pose.y;
    added_pose.z = current_pose.z;
    added_pose.roll = current_pose.roll;
    added_pose.pitch = current_pose.pitch;
    added_pose.yaw = current_pose.yaw;

    if (_method_type == MethodType::use_pcl)
      pcl_ndt.setInputTarget(map_ptr);  // 注意:此时加入的target:map_ptr并不包括刚加入点云的transformed_scan_ptr
    else if (_method_type == MethodType::use_cpu)  // 只是说map更新了,因此target也要更新,不要落后太多
    {
      if (_incremental_voxel_update)
        cpu_ndt.updateVoxelGrid(transformed_scan_ptr);
      else
        cpu_ndt.setInputTarget(map_ptr);
    } /*else if (_method_type == MethodType::use_gpu)
//      gpu_ndt.setInputTarget(map_ptr);*/
    else if (_method_type == MethodType::use_omp)
//      omp_ndt.setInputTarget(map_ptr);
      omp_ndt->setInputTarget(map_ptr);
  }// end if(shift)
  // ################################################################################

  // start5
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(globalmap, *target_cloud);
  pcl::VoxelGrid<pcl::PointXYZI> map_grid_filter;
  map_grid_filter.setLeafSize(0.5, 0.5, 0.5);
  map_grid_filter.setInputCloud(target_cloud);
  map_grid_filter.filter(*target_cloud);

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*target_cloud, *map_msg_ptr);
  //  pcl::toROSMsg(globalmap, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  ndt_map_pub.publish(*map_msg_ptr);
  target_cloud->clear();

  // localmap
  sensor_msgs::PointCloud2::Ptr localmap_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(map, *localmap_msg_ptr);
  localmap_msg_ptr->header.frame_id = "map";
  local_map_pub.publish(*localmap_msg_ptr);

  // publish the odom
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

  // odom.pose.covariance[0] = cov;
  current_odom_pub.publish(odom);

  ros::Time test_time_6 = ros::Time::now();
  // end5

  // 清空localmap
  int localmap_size = 0;
  if (submap_size >= max_submap_size) {
    map = submap;
    localmap_size = submap.size();
    submap.clear();
    submap_size = 0.0;
  }

  //std::cout << "-----------------------------------------------------------------" << std::endl;
//  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of localmap points: " << localmap_size << " points." << std::endl;
  std::cout << "global_map size : " << map.points.size() << " points." << std::endl;
  std::cout << "Aligned Time: " << (ndt_end - ndt_start) * 1000 << " ms" << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << final_num_iteration << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "localmap shift: " << shift << std::endl;
  std::cout << "global path: " << odom_size << std::endl;
  std::cout << "1-2: " << (test_time_2 - test_time_1) * 1000 << "ms" << "--downsample inputCloud" << std::endl;
  std::cout << "2-3: " << (test_time_3 - test_time_2) * 1000 << "ms" << "--set params and inputSource" << std::endl;
  std::cout << "3-4: " << (test_time_4 - test_time_3) * 1000 << "ms" << "--handle imu/odom and cal ndt resule"
            << std::endl;
  std::cout << "4-5: " << (test_time_5 - test_time_4) * 1000 << "ms" << "--get current pose" << std::endl;
  std::cout << "5-6: " << (test_time_6 - test_time_5) * 1000 << "ms" << "--publish current pose" << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

}

void LidarMapping::imu_callback(const sensor_msgs::Imu::Ptr &input) {
  // std::cout << __func__ << std::endl;

  if (_imu_upside_down)  // _imu_upside_down指示是否进行imu的正负变换
    imuUpSideDown(input);

//  mutex_lock.lock();
  imuBuf.push_back(*input);
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
//  odom = *input;
//  odom_calc(input->header.stamp);

//  mutex_lock.lock();
  odomBuf.push_back(*input);
//  mutex_lock.unlock();
}

void LidarMapping::points_callback(const sensor_msgs::PointCloud2::ConstPtr &input) {
  mutex_lock.lock();
  cloudBuf.push_back(*input);
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
  offset_imu_odom_x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
  offset_imu_odom_y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
  offset_imu_odom_z += diff_distance * sin(-current_pose_imu_odom.pitch);

  offset_imu_odom_roll += diff_imu_roll;
  offset_imu_odom_pitch += diff_imu_pitch;
  offset_imu_odom_yaw += diff_imu_yaw;

  // ==> 最终的目的是融合imu和odom输出一个guess_pose
  // 注:guess_pose是在previous_pose基础上叠加一个offset,包括xyz的和rpy的
  // xyz的offset需要融合imu的转角和odom的速度(位移)
  // rpy的offset直接采用imu的rpy偏差值
  guess_pose_imu_odom.x = previous_pose.x + offset_imu_odom_x;
  guess_pose_imu_odom.y = previous_pose.y + offset_imu_odom_y;
  guess_pose_imu_odom.z = previous_pose.z + offset_imu_odom_z;
  guess_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_roll;
  guess_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pitch;
  guess_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_yaw;

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
  offset_imu_x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  current_velocity_imu_x += accX * diff_time;  // imu的速度值会通过slam进行修正,以避免累计误差
  current_velocity_imu_y += accY * diff_time;  // 或者说imu计算时所用的速度并不是用imu得到的,而是用slam得到的
  current_velocity_imu_z += accZ * diff_time;    // imu所提供的参数,主要用来计算角度上的偏移,以及加速度导致的距离上的偏移!@

  offset_imu_roll += diff_imu_roll;
  offset_imu_pitch += diff_imu_pitch;
  offset_imu_yaw += diff_imu_yaw;

  guess_pose_imu.x = previous_pose.x + offset_imu_x;
  guess_pose_imu.y = previous_pose.y + offset_imu_y;
  guess_pose_imu.z = previous_pose.z + offset_imu_z;
  guess_pose_imu.roll = previous_pose.roll + offset_imu_roll;
  guess_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
  guess_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;

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
  offset_odom_x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  offset_odom_y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  offset_odom_z += diff_distance * sin(-current_pose_odom.pitch);

  offset_odom_roll += diff_odom_roll;
  offset_odom_pitch += diff_odom_pitch;
  offset_odom_yaw += diff_odom_yaw;

  guess_pose_odom.x = previous_pose.x + offset_odom_x;
  guess_pose_odom.y = previous_pose.y + offset_odom_y;
  guess_pose_odom.z = previous_pose.z + offset_odom_z;
  guess_pose_odom.roll = previous_pose.roll + offset_odom_roll;
  guess_pose_odom.pitch = previous_pose.pitch + offset_odom_pitch;
  guess_pose_odom.yaw = previous_pose.yaw + offset_odom_yaw;

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

//        input_yaw += M_PI/2;
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
  //

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