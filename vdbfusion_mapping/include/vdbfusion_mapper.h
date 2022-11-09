/**
 * Copyright (C) 2022, IADC, Hong Kong University of Science and Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2022-11-05
 */
#ifndef VDBFUSIOON_MAPPER_H
#define VDBFUSIOON_MAPPER_H

#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "MarchingCubesConst.h"
#include "VDBVolume.h"
#include "transform.h"

#include "openvdb/openvdb.h"
#include "vdbfusion_mapping_msgs/SaveMap.h"

namespace vdbfusion_mapping {
class VDBFusionMapper {
 public:
  typedef tf::StampedTransform Transformation;
  struct Config {
    int ros_spinner_threads = std::thread::hardware_concurrency();

    // default param
    bool sdf_space_carving = true;
    bool fill_holes_ = true;
    double max_height = 2.0;
    double min_scan_range = 0.4;
    double max_scan_range = 12.0;
    double sdf_trunc = 0.15;

    double sdf_voxel_size = 0.04;
    double sdf_min_weight = 0.5;
    double sdf_deactivate = 0.05;
  };

  VDBFusionMapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~VDBFusionMapper() = default;

  // ROS callbacks.
  void points_callback(const sensor_msgs::PointCloud2::Ptr &input);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &input);
  bool saveMap_callback(vdbfusion_mapping_msgs::SaveMap::Request &req,
                        vdbfusion_mapping_msgs::SaveMap::Response &res);

  // function
  void setConfig();
  void mapIntegrateProcess();

  void processPointCloudMessage(
      const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
      const Eigen::Matrix4d &tf_matrix);

  template <typename PCLPoint>
  void filterptRange(const typename pcl::PointCloud<PCLPoint> &pointcloud_pcl,
                     pcl::PointCloud<pcl::PointXYZ> &cloud_filter,
                     std::vector<openvdb::Vec3i> &color);

  // IO.
  const Config &getConfig() const { return config_; }

 private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // variables
  Config config_;
  vdbfusion::VDBVolume tsdf_volume, tsdf_volume_hdda;
  common::Transformer retrive_mpose;

  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber points_sub, odom_sub;
  ros::Publisher vdbmap_pub;
  ros::ServiceServer save_map_srv;

  std::mutex m_fullmap, m_data;

  std::queue<std::tuple<Eigen::Matrix4d, std::vector<Eigen::Vector3d>,
                        std::vector<openvdb::Vec3i>>>
      data_buf;

  std::thread integrate_thread;

  std::string _lidar_topic = "/odom_lidar";
  bool _debug_print = true;
  bool color_pointcloud = false;
  int enqueue = 0, dequeue = 0;
};
}  // namespace vdbfusion_mapping

#endif