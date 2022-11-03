#include <gflags/gflags.h>
#include <glog/logging.h>
#include <tf_conversions/tf_eigen.h>

#include "transform.h"

namespace common{

Transformer::Transformer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
    nh_private_(nh_private){
    nh_private_.getParam("world_frame", world_frame_);
    nh_private_.getParam("pose_source", pose_source_);
    
    double tolerance_ms;
    nh_private_.getParam("timestamp_tolerance_ms", tolerance_ms);
    tolerance_ns_ = static_cast<int64_t>(tolerance_ms*1000000); // 1ms = 1000000 ns

    // [0:tf_tree, 1:tf_topic, 2:odom_topic]
    if(pose_source_ == -1){
        // ERROR!!!
        // double tx, ty, tz, x, y, z, w; // static
    }
    else if(pose_source_ == 1){
        std::string tf_topic_;
        nh_private_.getParam("tf_topic", tf_topic_);
        tf_sub_ = nh_.subscribe(tf_topic_, 500, &Transformer::tfCallback, this);
        LOG(INFO) << "TF TF TF TOPIC FOR POSE, Please check the topic name:" << tf_topic_;
    }
    else if(pose_source_ == 2){
        std::string odom_topic_;
        nh_private_.getParam("odom_topic", odom_topic_);
        odom_sub_ = nh_.subscribe(odom_topic_, 500, &Transformer::odomCallback, this);
        LOG(INFO) << "ODOM ODOM ODOM TOPIC FOR POSE, Please check the topic name:" << odom_topic_;
    }
    LOG(INFO) << "Transformer init success";
}
bool Transformer::lookUpTransformfromSource(const sensor_msgs::PointCloud2::ConstPtr &input, Eigen::Matrix4d& tf_matrix){
    ros::Time timestamp = input->header.stamp;
    // [0:tf_tree, 1:tf_topic, 2:odom_topic]
    if(pose_source_ == 0){
        tf::StampedTransform T_G_C;
        try {
            tf_listener_.lookupTransform(input->header.frame_id, world_frame_, timestamp, T_G_C);
        }
        catch (tf::TransformException& ex) {  // NOLINT
            LOG(ERROR) << "Please check the tf tree between: " << input->header.frame_id << " and " << world_frame_;
            ROS_ERROR_STREAM(
                "Error getting TF transform from sensor data: " << ex.what());
        }
        Eigen::Matrix3d rot_matrix;
        Eigen::Vector3d t(T_G_C.getOrigin().getX(), T_G_C.getOrigin().getY(), T_G_C.getOrigin().getZ());
        tf::matrixTFToEigen(tf::Matrix3x3(T_G_C.getRotation()), rot_matrix);
        tf_matrix.block<3, 3>(0, 0) = rot_matrix;
        tf_matrix.block<3, 1>(0, 3) = t;
        return true;
    }
    else{
        if (T_Matrixs.empty()) {
            LOG(WARNING) << "No match found for transform timestamp: "<< timestamp << " as transform queue is empty.";
            return false;
        }
        bool match_found = false;
        int i = 0;
        std::vector<std::pair<ros::Time, Eigen::Matrix4d>>::iterator it = T_Matrixs.begin();
        for (; it != T_Matrixs.end(); it++) {
            i++;
            if (it->first >= timestamp) {
                if ((it->first - timestamp).toNSec() <= tolerance_ns_){
                    match_found = true;
                }
                break;
            }

            if ((timestamp - it->first).toNSec() < tolerance_ns_) {
                match_found = true;
                break;
            }
        }

        if (match_found) {
            tf_matrix = it->second;
            T_Matrixs.erase(T_Matrixs.begin(), it);
            LOG(INFO) << "Found! id: "<< i << ", Finished copy and erase";
            return true;
        }
        else
            return false;
        // TODO interpolate 
        // else{
        //     if (i == Mpose_queue_.size()) {
        //         ROS_WARN_STREAM_THROTTLE(30, "No match found for transform timestamp: " << timestamp);
        //         return false;
        //     }
        //     int j = i;
        //     int64_t newest_timestamp_ns = time_queue_[j].toNSec();
        //     --j;
        //     int64_t oldest_timestamp_ns = time_queue_[j].toNSec();

        //     double alpha = 0.0;
        //     if (newest_timestamp_ns != oldest_timestamp_ns) {
        //         alpha = static_cast<double>(timestamp.toNSec() - oldest_timestamp_ns) /
        //                 static_cast<double>(newest_timestamp_ns - oldest_timestamp_ns);
        //     }
        //     transform.transform = interpolate(time_queue_[j], tf_newest.transform, alpha);
        // }
    }

}
void Transformer::tfCallback(const geometry_msgs::TransformStamped& transform_msg) {
    geometry_msgs::Transform tf_msg_ = transform_msg.transform;

    Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identity();
    Eigen::Vector3d t(tf_msg_.translation.x, tf_msg_.translation.y, tf_msg_.translation.z);
    Eigen::Quaterniond q(
        tf_msg_.rotation.w, tf_msg_.rotation.x,
        tf_msg_.rotation.y, tf_msg_.rotation.z);

    tf_matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
    tf_matrix.block<3, 1>(0, 3) = t;
    T_Matrixs.push_back(std::make_pair(transform_msg.header.stamp, tf_matrix));
}
void Transformer::odomCallback(const nav_msgs::Odometry::ConstPtr &input) {

    Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identity();
    Eigen::Vector3d t(input->pose.pose.position.x, input->pose.pose.position.y,
                        input->pose.pose.position.z);
    Eigen::Quaterniond q(
        input->pose.pose.orientation.w, input->pose.pose.orientation.x,
        input->pose.pose.orientation.y, input->pose.pose.orientation.z);

    tf_matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
    tf_matrix.block<3, 1>(0, 3) = t;
    T_Matrixs.push_back(std::make_pair(input->header.stamp, tf_matrix));
}
}  