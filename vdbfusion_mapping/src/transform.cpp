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
        auto trans = T_G_C.getOrigin();
        auto rota = T_G_C.getRotation();
        transform2Eigen(tf_matrix, trans.getX(), trans.getY(), trans.getZ(),
                        rota.getW(), rota.getX(),rota.getY(), rota.getZ());
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
                // If the current transform is newer than the requested timestamp, we need
                // to break.
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
        // TODO still have problems ====> TODO find it out 
        // else{
        //     // If we think we have an inexact match, have to check that we're still
        //     // within bounds and interpolate.
        //     if (it == T_Matrixs.begin() || it == T_Matrixs.end()) {
        //         ROS_WARN_STREAM_THROTTLE(
        //             30, "No match found for transform timestamp: "
        //                     << timestamp
        //                     << " Queue front: " << T_Matrixs.front().first
        //                     << " back: " << T_Matrixs.back().first);
        //         return false;
        //     }
        //     Eigen::Matrix4d tf_matrix_newest = it->second;
        //     int64_t offset_newest_ns = (it->first - timestamp).toNSec();
        //     // We already checked that this is not the beginning.
        //     it--;
        //     Eigen::Matrix4d tf_matrix_oldest = it->second;
        //     int64_t offset_oldest_ns = (timestamp - it->first).toNSec();

        //     // Interpolate between the two transformations using the exponential map.
        //     float t_diff_ratio = static_cast<float>(offset_oldest_ns) / static_cast<float>(offset_newest_ns + offset_oldest_ns);

        //     // 
        //     Eigen::Matrix4d diff_tf = tf_matrix_oldest.inverse() * tf_matrix_newest;

        //     double roll = M_PI/atan2(diff_tf(2,1),diff_tf(2,2));
        //     double pitch = M_PI/atan2(-diff_tf(2,0), std::pow( diff_tf(2,1)*diff_tf(2,1) +diff_tf(2,2)*diff_tf(2,2) ,0.5));
        //     double yaw = M_PI/atan2( diff_tf(1,0),diff_tf(0,0));
        //     Eigen::Matrix<double, 6, 1> pose_6axis;
        //     pose_6axis << diff_tf(0,3), diff_tf(1,3), diff_tf(2,3), roll, pitch, yaw;

        //     Eigen::Matrix<double, 6, 1> sample_diff = t_diff_ratio * pose_6axis;
        //     SixVector2Eigen(diff_tf, sample_diff(0,0), sample_diff(1,0), sample_diff(2,0),
        //                              sample_diff(3,0), sample_diff(4,0), sample_diff(5,0));

        //     tf_matrix = tf_matrix_oldest*diff_tf;
        //     return true;
        // }
        return false;
    }

}
void Transformer::tfCallback(const geometry_msgs::TransformStamped& transform_msg) {
    geometry_msgs::Transform tf_msg_ = transform_msg.transform;

    Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identity();
    transform2Eigen(tf_matrix, tf_msg_.translation.x, tf_msg_.translation.y, tf_msg_.translation.z,
    tf_msg_.rotation.w, tf_msg_.rotation.x,tf_msg_.rotation.y, tf_msg_.rotation.z);

    T_Matrixs.push_back(std::make_pair(transform_msg.header.stamp, tf_matrix));
}
void Transformer::odomCallback(const nav_msgs::Odometry::ConstPtr &input) {

    Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identity();
    transform2Eigen(tf_matrix, input->pose.pose.position.x, input->pose.pose.position.y, input->pose.pose.position.z,
    input->pose.pose.orientation.w, input->pose.pose.orientation.x,
    input->pose.pose.orientation.y, input->pose.pose.orientation.z);
    T_Matrixs.push_back(std::make_pair(input->header.stamp, tf_matrix));
}
}  