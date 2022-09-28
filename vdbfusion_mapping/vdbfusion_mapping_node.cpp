#include <glog/logging.h>
#include <ros/ros.h>

#include "vdbfusion_mapper.h"

int main(int argc, char **argv) {
  // Start Ros.
  ros::init(argc, argv, "vdbfusion_mapping",
            ros::init_options::NoSigintHandler);

  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::SetStderrLogging(google::INFO);
  FLAGS_colorlogtostderr = true;

  // Setup node.
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  vdbfusion_mapping::VDBFusionMapper mapper(nh, nh_private);

  // Setup spinning.
  ros::AsyncSpinner spinner(mapper.getConfig().ros_spinner_threads);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
