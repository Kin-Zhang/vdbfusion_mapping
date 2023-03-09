/**
 * Copyright (C) 2022, IADC, Hong Kong University of Science and Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2022-11-05
 * Description: main cpp for vdbfusion_mapping, process using the thread
 */

#include <ros/ros.h>
#include <iostream>

#include <igl/write_triangle_mesh.h>
#include <open3d/Open3D.h>
#include <thread>

// our define
#include "timer.h"
#include "utils.h"
#include "vdbfusion_mapper.h"

namespace vdbfusion_mapping {
VDBFusionMapper::VDBFusionMapper(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), retrive_mpose(nh, nh_private) {
  setConfig();
  vdbmap_pub = nh_.advertise<sensor_msgs::PointCloud2>("/vdbmap", 10);
  points_sub =
      nh_.subscribe(_lidar_topic, 10, &VDBFusionMapper::points_callback, this);

  save_map_srv = nh_.advertiseService("/save_map",
                                      &VDBFusionMapper::saveMap_callback, this);
  // initial tsdf volume from vdbfusion
  openvdb::initialize();
  tsdf_volume = vdbfusion::VDBVolume(config_.sdf_voxel_size, config_.sdf_trunc,
                                     config_.sdf_space_carving);
}

void VDBFusionMapper::mapIntegrateProcess() {
  while (ros::ok()) {
    m_data.lock();
    if (data_buf.empty()) {
      std::chrono::seconds dura(5);
      m_data.unlock();
      LOG(INFO) << "There is no data now, finished all data. After saving the "
                   "result, you can kill then";
      std::this_thread::sleep_for(dura);
      continue;
    }

    TIC;
    int total_num = data_buf.size();
    LOG(INFO) << "Total data frames need to integrate: " << total_num;

    Eigen::Matrix4d tf_matrix = std::get<0>(data_buf.front());
    std::vector<Eigen::Vector3d> points = std::get<1>(data_buf.front());
    std::vector<openvdb::Vec3i> color = std::get<2>(data_buf.front());

    data_buf.pop();
    m_data.unlock();

    Eigen::Vector3d origin = tf_matrix.block<3, 1>(0, 3);

    tsdf_volume.Integrate(points, color, origin,
                          common::WeightFunction::constant_weight);

    LOG_IF(INFO, _debug_print) << "cloud point size: " << points.size();
    TOC("TSDF Integrate", _debug_print);

    if (_debug_print)
      std::cout << "------------------------------------------------------"
                << std::endl;
  }
}
void VDBFusionMapper::points_callback(
    const sensor_msgs::PointCloud2::Ptr &input) {
  Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identity();
  if (!retrive_mpose.lookUpTransformfromSource(input, tf_matrix)) {
    // LOG(WARNING) << "Didn't find the pair pose, skip this message";
    return;
  }
  // Horrible hack fix to fix color parsing colors in PCL.
  bool has_intensity = false;
  for (size_t d = 0; d < input->fields.size(); ++d) {
    if (input->fields[d].name == std::string("rgb")) {
      input->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      color_pointcloud = true;
    } else if (input->fields[d].name == std::string("intensity")) {
      has_intensity = true;
    }
  }
  // filter by range
  pcl::PointCloud<pcl::PointXYZ> cloud_filter, trans_cloud;
  std::vector<openvdb::Vec3i> colors;
  // Convert differently depending on RGB or I type.
  if (color_pointcloud) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    pcl::fromROSMsg(*input, pointcloud_pcl);
    filterptRange(pointcloud_pcl, cloud_filter, colors);
  } else if (has_intensity) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    pcl::fromROSMsg(*input, pointcloud_pcl);
    filterptRange(pointcloud_pcl, cloud_filter, colors);
  } else {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
    pcl::fromROSMsg(*input, pointcloud_pcl);
    filterptRange(pointcloud_pcl, cloud_filter, colors);
  }

  // TODO voxblox bundlerays process => speed up
  // multi-thread index  => speed up
  // RGB involved => more great into the downstream task
  TIC;
  pcl::transformPointCloud(cloud_filter, trans_cloud, tf_matrix.cast<float>());
  std::vector<Eigen::Vector3d> points, points_bundle;
  common::PCL2Eigen(trans_cloud, points);
  m_data.lock();
  data_buf.push(std::make_tuple(tf_matrix, points, colors));
  m_data.unlock();
}

template <typename PCLPoint>
void VDBFusionMapper::filterptRange(
    const typename pcl::PointCloud<PCLPoint> &pointcloud_pcl,
    pcl::PointCloud<pcl::PointXYZ> &cloud_filter,
    std::vector<openvdb::Vec3i> &colors) {
  cloud_filter.reserve(pointcloud_pcl.size());
  colors.reserve(pointcloud_pcl.size());

  pcl::PointXYZ p;
  double p_radius;
  bool enable_filter = true;
  if (config_.min_scan_range < 0 || config_.max_scan_range < 0)
    enable_filter = false;

  int id = 0;
  for (auto item = pointcloud_pcl.begin(); item != pointcloud_pcl.end();
       item++, id++) {
    if (!(std::isfinite(item->x) && std::isfinite(item->y) &&
          std::isfinite(item->z)))
      continue;
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    if (enable_filter) {
      p_radius = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
      if (config_.min_scan_range < p_radius &&
          p_radius < config_.max_scan_range && p.z < config_.max_height) {
        colors.push_back(common::convertColor(pointcloud_pcl.points[id]));
        cloud_filter.push_back(p);
      }
    } else {
      colors.push_back(common::convertColor(pointcloud_pcl.points[id]));
      cloud_filter.push_back(p);
    }
  }
}

bool VDBFusionMapper::saveMap_callback(
    vdbfusion_mapping_msgs::SaveMap::Request &req,
    vdbfusion_mapping_msgs::SaveMap::Response &res) {
  std::cout << "=================== SAVE PROCESS START ======================"
            << std::endl;
  double save_map_filter_res = req.filter_res;
  TIC;
  std::string save_map_path = req.path;
  // to point cloud
  m_fullmap.lock();

  auto [vertices, triangles, color_] = tsdf_volume.ExtractTriangleMesh(
      config_.fill_holes_, config_.sdf_min_weight);

  m_fullmap.unlock();

  LOG(INFO) << "points num: " << vertices.size();
  if (vertices.size() == 0) {
    LOG(WARNING) << "NO DATA NOW, please try again later";
    return false;
  }
  if (color_pointcloud && color_.size() != 0) {
    pcl::PointCloud<pcl::PointXYZRGB> map_cloud;
    for (size_t i = 0; i < vertices.size(); i++) {
      pcl::PointXYZRGB pt;
      pt.x = static_cast<float>(vertices[i].x());
      pt.y = static_cast<float>(vertices[i].y());
      pt.z = static_cast<float>(vertices[i].z());
      pt.r = static_cast<uint8_t>(color_[i][0]);
      pt.g = static_cast<uint8_t>(color_[i][1]);
      pt.b = static_cast<uint8_t>(color_[i][2]);
      // std::cout << "r,g,b: " << color_[i][0] << " , " << color_[i][1] << " ,
      // " << color_[i][2] <<std::endl; // debug only
      map_cloud.push_back(pt);
    }
    auto map_ptr = map_cloud.makeShared();
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = retrive_mpose.getWorldframe();
    vdbmap_pub.publish(*map_msg_ptr);
    TOC("PUBLISH Color Msg", _debug_print);
    if (save_map_filter_res == 0.0)
      pcl::io::savePCDFileASCII(save_map_path + "_vertices.pcd", map_cloud);
    else {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_filtered(
          new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;
      voxel_grid_filter.setLeafSize(save_map_filter_res, save_map_filter_res,
                                    save_map_filter_res);
      voxel_grid_filter.setInputCloud(map_ptr);
      voxel_grid_filter.filter(*map_filtered);
      pcl::io::savePCDFileASCII(save_map_path + "_vertices.pcd", *map_filtered);
      std::cout << "Original: " << map_cloud.points.size() << " points."
                << std::endl;
      std::cout << "Filtered: " << map_filtered->points.size() << " points."
                << std::endl;
    }

  } else {
    pcl::PointCloud<pcl::PointXYZ> map_cloud;
    for (size_t i = 0; i < vertices.size(); i++) {
      pcl::PointXYZ pt;
      pt.x = static_cast<float>(vertices[i].x());
      pt.y = static_cast<float>(vertices[i].y());
      pt.z = static_cast<float>(vertices[i].z());
      map_cloud.push_back(pt);
    }
    auto map_ptr = map_cloud.makeShared();
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = retrive_mpose.getWorldframe();
    vdbmap_pub.publish(*map_msg_ptr);
    TOC("PUBLISH XYZ Msg", _debug_print);
    // Writing Point Cloud data to PCD file
    if (save_map_filter_res == 0.0)
      pcl::io::savePCDFileASCII(save_map_path + "_vertices.pcd", map_cloud);
    else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr map_filtered(
          new pcl::PointCloud<pcl::PointXYZ>());
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
      voxel_grid_filter.setLeafSize(save_map_filter_res, save_map_filter_res,
                                    save_map_filter_res);
      voxel_grid_filter.setInputCloud(map_ptr);
      voxel_grid_filter.filter(*map_filtered);
      pcl::io::savePCDFileASCII(save_map_path + "_vertices.pcd", *map_filtered);
      std::cout << "Original: " << map_cloud.points.size() << " points."
                << std::endl;
      std::cout << "Filtered: " << map_filtered->points.size() << " points."
                << std::endl;
    }
  }
  LOG(INFO) << "save pcd in path: " << save_map_path + "_vertices.pcd";
  TOC("SAVE PCD", _debug_print);

  open3d::geometry::TriangleMesh mesh_o3d =
      open3d::geometry::TriangleMesh(vertices, triangles);
  if (color_pointcloud && color_.size() != 0) {
    mesh_o3d.vertex_colors_.reserve(mesh_o3d.vertices_.size());
    for (size_t i = 0; i < mesh_o3d.vertices_.size(); i++) {
      mesh_o3d.vertex_colors_.emplace_back(
          color_[i][0] / 255.0, color_[i][1] / 255.0, color_[i][2] / 255.0);
    }
  }
  open3d::io::WriteTriangleMesh(save_map_path + "_mesh.ply", mesh_o3d);
  LOG(INFO) << "save mesh in path: " << save_map_path + "_mesh.ply";
  TOC("SAVE MESH", _debug_print);

  // if you don't want to use the open3d to save, use this one for simple but no
  // RGB! Eigen::MatrixXd V(vertices.size(), 3); for (size_t i = 0; i <
  // vertices.size(); i++) {
  //   V.row(i) = Eigen::VectorXd::Map(&vertices[i][0], vertices[i].size());
  // }
  // Eigen::MatrixXi F(triangles.size(), 3);
  // for (size_t i = 0; i < triangles.size(); i++) {
  //   F.row(i) = Eigen::VectorXi::Map(&triangles[i][0], triangles[i].size());
  // }
  // igl::write_triangle_mesh(save_map_path + "_mesh.ply", V, F,
  //                          igl::FileEncoding::Binary);

  res.success = true;
  std::cout << "=================== SAVE PROCESS END ======================"
            << std::endl;
  return res.success;
}

void VDBFusionMapper::setConfig() {
  nh_private_.getParam("lidar_topic", _lidar_topic);
  nh_private_.getParam("debug_print", _debug_print);

  nh_private_.getParam("min_scan_range", config_.min_scan_range);
  nh_private_.getParam("max_scan_range", config_.max_scan_range);
  nh_private_.getParam("max_height", config_.max_height);
  nh_private_.getParam("sdf_space_carving", config_.sdf_space_carving);
  nh_private_.getParam("sdf_trunc", config_.sdf_trunc);
  nh_private_.getParam("sdf_voxel_size", config_.sdf_voxel_size);
  nh_private_.getParam("sdf_deactivate", config_.sdf_deactivate);
  nh_private_.getParam("sdf_min_weight", config_.sdf_min_weight);

  LOG(INFO) << "==========> Setting Config Success, start for running";
}

}  // namespace vdbfusion_mapping