#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <cmath>
#include <pcl/point_cloud.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <deque>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace common {
// part of code from voxblox: https://github.com/ethz-asl/voxblox
// part of reference vdbfusion_ros: https://github.com/PRBonn/vdbfusion_ros
class Transformer {
    public:
    
      Transformer(){};
      Transformer(const ros::NodeHandle& nh,
                  const ros::NodeHandle& nh_private);

      virtual ~Transformer() = default;

      bool lookUpTransformfromSource(const sensor_msgs::PointCloud2::ConstPtr &input, Eigen::Matrix4d& tf_matrix);

    private:
      bool tfTreeRead(const ros::Time& timestamp,
                      const ros::Duration& tolerance,
                      geometry_msgs::TransformStamped& transform);

      void tfCallback(const geometry_msgs::TransformStamped& input);

      void odomCallback(const nav_msgs::Odometry::ConstPtr &input);


      ros::NodeHandle nh_;
      ros::NodeHandle nh_private_;
      tf::TransformListener tf_listener_;
      ros::Subscriber tf_sub_, odom_sub_;

      // save all pose queue
      std::vector<std::pair<ros::Time, Eigen::Matrix4d>> T_Matrixs;


      int64_t tolerance_ns_;
      int pose_source_ = -1; // [0:tf_tree, 1:tf_topic, 2:odom_topic]
      
      std::string world_frame_ = "map";
    
};

} // namespace common

#endif // TRANSFORM_H