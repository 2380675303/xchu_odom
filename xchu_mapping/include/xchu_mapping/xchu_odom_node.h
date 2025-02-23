

#ifndef NDT_MAPPING_LIDARMAPPING_H
#define NDT_MAPPING_LIDARMAPPING_H

#define OUTPUT  // define OUTPUT if you want to log postition.txt

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <deque>
#include <mutex>
#include <queue>
#include <thread>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h>
#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pclomp/ndt_omp.h>
#include "omp.h"

using PointT = pcl::PointXYZI;

struct pose {
  double x, y, z, roll, pitch, yaw;

  pose() {
    x = y = z = roll = pitch = yaw = 0.0;
  }

  pose(double _x,
       double _y,
       double _z,
       double _roll,
       double _pitch,
       double _yaw
  ) : x(_x), y(_y), z(_z) {}

  void init() {
    x = y = z = roll = pitch = yaw = 0.0;
  }

  Eigen::Matrix4d rotateRPY() {
    Eigen::Translation3d tf_trans(x, y, z);
    Eigen::AngleAxisd rot_x(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d mat = (tf_trans * rot_z * rot_y * rot_x).matrix();
    return mat;
  }
};

pose operator+(const pose &A, const pose B) //给结构体定义加法；
{
  return pose(A.x + B.x, A.y + B.y, A.z + B.z, A.roll + B.roll, A.pitch + B.pitch, A.yaw + B.yaw);
}
pose operator-(const pose &A, const pose B) {
  return pose(A.x - B.x, A.y - B.y, A.z - B.z, A.roll - B.roll, A.pitch - B.pitch, A.yaw - B.yaw);
}
std::ostream &operator<<(std::ostream &out, const pose &p)  //定义结构体流输出
{
  out << "(" << p.x << "," << p.y << "," << p.z << "," << p.roll << "," << p.pitch << "," << p.yaw << ")";
  return out;
}

enum class MethodType {
  use_pcl = 0,
  use_cpu = 1,
  use_gpu = 2,
  use_omp = 3,
};

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

class LidarOdom {
 private:
  pose previous_pose, guess_pose, guess_pose_imu, guess_pose_odom, guess_pose_imu_odom;
  pose current_pose, current_pose_imu, current_pose_odom, current_pose_imu_odom;
  pose ndt_pose, localizer_pose;
  pose added_pose;     // 初始设为0即可,因为第一帧如论如何也要加入地图的

  Eigen::Matrix4f pre_pose_, guess_pose_, guess_pose_imu_, guess_pose_odom_, guess_pose_imu_odom_;
  Eigen::Matrix4f current_pose_, current_pose_imu_, current_pose_odom_, current_pose_imu_odom_;
  Eigen::Matrix4f ndt_pose_, localizer_pose_, added_pose_;

  // 定义各种差异值(两次采集数据之间的差异,包括点云位置差异,imu差异,odom差异,imu-odom差异)
  pose diff_pose, offset_imu_pose, offset_odom_pose, offset_imu_odom_pose;
  double diff;
//  double diff_x, diff_y, diff_z, diff_yaw;  // current_pose - previous_pose // 定义两帧点云差异值 --以确定是否更新点云等
//  double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;
//  double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll, offset_odom_pitch, offset_odom_yaw;
//  double offset_imu_odom_x, offset_imu_odom_y, offset_imu_odom_z, offset_imu_odom_roll, offset_imu_odom_pitch,
//      offset_imu_odom_yaw;

  // 定义速度值 --包括实际速度值,和imu取到的速度值
  double current_velocity_x, current_velocity_y, current_velocity_z;
  double current_velocity_imu_x, current_velocity_imu_y, current_velocity_imu_z;

  //  ros::Time current_scan_time;
  ros::Time previous_scan_time;
//  ros::Duration scan_duration;

  // 定义Publisher
  ros::Publisher current_odom_pub, keyposes_pub, current_points_pub, ndt_stat_pub;  // TODO:这是个啥发布????
  ros::Subscriber points_sub, imu_sub, odom_sub;

  pcl::PointCloud<PointTypePose>::Ptr cloud_keyposes_6d_;

  geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;
  std_msgs::Bool ndt_stat_msg;  // 确认是否是ndt的第一帧图像 bool类型
  sensor_msgs::Imu imu;
  nav_msgs::Odometry odom;

  MethodType _method_type = MethodType::use_cpu;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> pcl_ndt;
  cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> cpu_ndt;  // cpu方式  --可以直接调用吗??可以
  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr omp_ndt;

  // 设置变量,用以接受ndt配准后的参数
  double fitness_score;
  bool has_converged;
  int final_num_iteration;
  double transformation_probability;

  // Default values  // 公共ndt参数设置
  int max_iter;        // Maximum iterations
  float ndt_res;      // Resolution
  double step_size;   // Step size
  double trans_eps;  // Transformation epsilon
  Eigen::Matrix4f gnss_transform;  // 保存GPS信号的变量

  // 读写文件流相关
  std::ofstream ofs; // 写csv文件
  std::string filename;

  pcl::PointCloud<pcl::PointXYZI> localmap, globalmap, submap;

#ifdef CUDA_FOUND
  gpu::GNormalDistributionsTransform gpu_ndt;
  // TODO:此处增加共享内存方式的gpu_ndt_ptr
   std::shared_ptr<gpu::GNormalDistributionsTransform> gpu_ndt = std::make_shared<gpu::GNormalDistributionsTransform>();
#endif

  double min_add_scan_shift;  // 定义将点云添加到locaMap里的最小间隔值  --应该是添加到localMap吧??
  int initial_scan_loaded = 0;

  bool _incremental_voxel_update = false;
  ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, ndt_start, ndt_end,
      t5_start, t5_end;
  ros::Duration d_callback, d1, d2, d3, d4, d5;
  int submap_num = 0;
  double localmap_size = 0.0;
  double max_localmap_size, odom_size;

  double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
  Eigen::Matrix4f tf_b2l, tf_l2b;

  // 重要变量参数 :: 用以指示是否使用imu,是否使用odom
  bool _use_imu = false;
  bool _use_odom = false;
  bool _imu_upside_down = false;  // 用以解决坐标系方向(正负变换)问题 (比如x变更为-x等)
  int method_type_temp = 0;

//  std::string _imu_topic;  // 定义imu消息的topic
//  std::string _odom_topic;
  std::string map_saved_dir;

  // mutex
  std::mutex mutex_lock;
  std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue_;
  std::queue<sensor_msgs::ImuConstPtr> imu_queue_;
  std::queue<nav_msgs::OdometryConstPtr> odom_queue_;

  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::ImuConstPtr> imu_data;

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterGlobalMap; // for global map visualization
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterKeyFrames; // for global map visualization
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterLocalmap; // for global map visualization
  pcl::VoxelGrid<PointT> downSizeFilterSource;

  pcl::PointCloud<PointT>::Ptr cloud_keyposes_3d_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr;
  //pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr;
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr;

  Eigen::Matrix4f t_localizer;
  Eigen::Matrix4f t_base_link;
 public:
  ros::NodeHandle nh;

  LidarOdom();

  inline double warpToPm(double a_num, const double a_max) {
    if (a_num >= a_max) {
      a_num -= 2.0 * a_max;
    }
    return a_num;
  }

  inline double warpToPmPi(double a_angle_rad) {
    return warpToPm(a_angle_rad, M_PI);
  }

  inline double calcDiffForRadian(const double lhs_rad, const double rhs_rad) {
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad >= M_PI)
      diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
      diff_rad = diff_rad + 2 * M_PI;
    return diff_rad;
  }

  void Run();

  void OdomEstimate(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input, const ros::Time &current_scan_time);

  void ParamInitial();

  void PublishCloud(const ros::Time &current_scan_time);

  void ViewerThread();

  void ImuCB(const sensor_msgs::ImuConstPtr &msg);

  void OdomCB(const nav_msgs::OdometryConstPtr &msg);

  void PcCB(const sensor_msgs::PointCloud2ConstPtr &msg);

  void ImuOdomCalc(ros::Time current_time);

  void ImuCalc(ros::Time current_time);

  void OdomCalc(ros::Time current_time);

  void SaveMap();

  void imuUpSideDown(const sensor_msgs::Imu::Ptr input);

  void odom_info(const nav_msgs::Odometry &input);

  void imu_info(const sensor_msgs::Imu &input);
};

#endif
