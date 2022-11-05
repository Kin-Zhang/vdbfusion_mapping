#include <iostream>
#include <ros/ros.h>

#include <igl/write_triangle_mesh.h>
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
  // tsdf_volume_hdda = vdbfusion::VDBVolume(config_.sdf_voxel_size, config_.sdf_trunc,
  //                                    config_.sdf_space_carving);
}

void VDBFusionMapper::mapIntegrateProcess(){
  while(ros::ok()){
    m_data.lock();
    if (data_buf.empty()) {
      std::chrono::seconds dura(5);
      m_data.unlock();
      LOG(INFO) << "There is no data now, finished all data";
      std::this_thread::sleep_for(dura);
      continue;
    }

    TIC;
    int total_num = data_buf.size();
    LOG(INFO) << "Total data frames need to integrate: " << total_num;
    Eigen::Matrix4d tf_matrix=data_buf.front().first;
    std::vector<Eigen::Vector3d> points=data_buf.front().second;
    data_buf.pop();
    m_data.unlock();

    Eigen::Vector3d origin = tf_matrix.block<3, 1>(0, 3);
    tsdf_volume.Integrate(points, origin, common::WeightFunction::constant_weight);
    LOG_IF(INFO, _debug_print) << "cloud point size: " << points.size();
    TOC("TSDF Integrate", _debug_print);
    // tsdf_volume_hdda.Integrate_HDDA(points, origin, common::WeightFunction::linear_weight);
    // TOC("TSDF HDDA Intergrate", _debug_print);

    if (_debug_print)
      std::cout << "------------------------------------------------------"
                << std::endl;
  }

}
void VDBFusionMapper::points_callback(
    const sensor_msgs::PointCloud2::ConstPtr &input) {
  Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identity();
  if(!retrive_mpose.lookUpTransformfromSource(input, tf_matrix)){
    // LOG(WARNING) << "Didn't find the pair pose, skip this message";
    return;
  }
  // Horrible hack fix to fix color parsing colors in PCL.
  bool color_pointcloud = false;
  bool has_intensity = false;
  if (input->fields[0].name == std::string("rgb"))
    color_pointcloud = true;
  else if (input->fields[0].name == std::string("intensity"))
    has_intensity = true;

  // filter by range
  pcl::PointCloud<pcl::PointXYZ> cloud_filter, trans_cloud;

  // Convert differently depending on RGB or I type.
  if (color_pointcloud){
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    pcl::fromROSMsg(*input, pointcloud_pcl);
    filterptRange(pointcloud_pcl, cloud_filter);
  }
  else if (has_intensity){
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    pcl::fromROSMsg(*input, pointcloud_pcl);
    filterptRange(pointcloud_pcl, cloud_filter);
  }
  else{
    pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
    pcl::fromROSMsg(*input, pointcloud_pcl);
    filterptRange(pointcloud_pcl, cloud_filter);
  }

  // TODO voxblox bundlerays process
  // multi-thread index also
  TIC;
  pcl::transformPointCloud(cloud_filter, trans_cloud, tf_matrix.cast<float>());
  std::vector<Eigen::Vector3d> points, points_bundle;
  common::PCL2Eigen(trans_cloud, points);
  m_data.lock();
  data_buf.push(std::make_pair(tf_matrix, points));
  m_data.unlock();
}

template <typename PCLPoint>
void VDBFusionMapper::filterptRange(const typename pcl::PointCloud<PCLPoint>& pointcloud_pcl, 
                                    pcl::PointCloud<pcl::PointXYZ>& cloud_filter){
  pcl::PointXYZ p;
  double p_radius;
  bool enable_filter = true;
  if(config_.min_scan_range < 0 || config_.max_scan_range < 0)
    enable_filter=false;

  for (auto item = pointcloud_pcl.begin(); item != pointcloud_pcl.end(); item++) {
    if(!(std::isfinite(item->x) && std::isfinite(item->y) && std::isfinite(item->z)))
      continue;
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    if(enable_filter){
      p_radius = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
      if (config_.min_scan_range < p_radius &&
          p_radius < config_.max_scan_range && p.z < config_.max_height)
        cloud_filter.push_back(p);
    }
    else
      cloud_filter.push_back(p);
  }
}

bool VDBFusionMapper::saveMap_callback(
    vdbfusion_mapping_msgs::SaveMap::Request &req,
    vdbfusion_mapping_msgs::SaveMap::Response &res) {
  std::cout << "=================== SAVE PROCESS START ======================"
            << std::endl;
  TIC;
  std::string save_map_path = req.path;
  // to point cloud
  m_fullmap.lock();
  auto [vertices, triangles] = tsdf_volume.ExtractTriangleMesh(
      config_.fill_holes_, config_.sdf_min_weight);
  m_fullmap.unlock();
  TOC("EXTRACT Triangle", _debug_print);
  pcl::PointCloud<pcl::PointXYZ> map_cloud;
  map_cloud.reserve(vertices.size());
  for (size_t i = 0; i < vertices.size(); i++) {
    pcl::PointXYZ pt;
    pt.x = static_cast<float>(vertices[i].x());
    pt.y = static_cast<float>(vertices[i].y());
    pt.z = static_cast<float>(vertices[i].z());
    map_cloud.push_back(pt);
  }

  auto map_ptr = map_cloud.makeShared();
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  map_ptr->header.frame_id = "map";
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  vdbmap_pub.publish(*map_msg_ptr);
  TOC("PUBLISH Msg", _debug_print);

  Eigen::MatrixXd V(vertices.size(), 3);
  for (size_t i = 0; i < vertices.size(); i++) {
    V.row(i) = Eigen::VectorXd::Map(&vertices[i][0], vertices[i].size());
  }
  Eigen::MatrixXi F(triangles.size(), 3);
  for (size_t i = 0; i < triangles.size(); i++) {
    F.row(i) = Eigen::VectorXi::Map(&triangles[i][0], triangles[i].size());
  }
  igl::write_triangle_mesh(save_map_path + "_mesh.ply", V, F,
                           igl::FileEncoding::Binary);
  LOG(INFO) << "save mesh in path: " << save_map_path + "_mesh.ply";
  TOC("SAVE MESH", _debug_print);

  double save_map_filter_res = req.filter_res;
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

  LOG(INFO) << "save pcd in path: " << save_map_path + "_vertices.pcd";
  TOC("SAVE PCD", _debug_print);

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
  nh_private_.getParam("sdf_min_weight", config_.sdf_min_weight);
  
  LOG(INFO) << "==========> Setting Config Success, start for running";
}

} // namespace vdbfusion_mapping