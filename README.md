# VDBFUSION_ROS_MAPPING

This repo is **<u>modified version of vdbfusion</u>** for mapping incrementally based on received odometry and corresponding point cloud message. The whole process is based on the ROS1, please check [origin repo of vdbfusion](https://github.com/PRBonn/vdbfusion) if you'd like to use directly.

# Install

Only provide the Docker version for convenient  usage, more dependencies or install on your own environment please check their origin repo link. Docker pull only [在内地的同学先换一下dockerhub的源]

```bash
docker pull zhangkin/vdbmapping_mapping

# or build through Dockerfile
docker build -t zhangkin/vdbfusion_mapping .

# =========== RUN
docker run -it --net=host -v /dev/shm:/dev/shm -v /home/kin/bags:/workspace/data --name vdbfusion_mapping zhangkin/vdbfusion_mapping /bin/zsh
```

## Dependencies

Here is some dependencies for desktop installed if you'd like to try. Please follow their dependencies to install, [Dockerfile](Dockerfile) may help you with that.

- [IGL](https://github.com/libigl/libigl): mesh save
- [OpenVDB](https://github.com/nachovizzo/openvdb.git): vdb data structure, ATTENTION Boost need 1.70, Ubuntu default is 1.65
- [glog, gflag](https://github.com/google/glog.git): for output log
- [ROS1](http://wiki.ros.org/ROS/Installation): ROS-full (tested on melodic)

# Usage

Please note that this is the for incremental mapping, no! odom output! So, you have to have odom/tf topic with same timestamp lidar msg.


## Config

The only thing you have to change is the config file about the topic name on your own dataset/equipment.

```yaml
# input topic name setting ===========> Please change according to your dataset
lidar_topic: "/odom_lidar"
odom_topic: "/auto_odom"
```

## Run

run launch with bag directly

```bash
roslaunch vdbfusion_ros vdbfusion_mapping.launch
```

save and pub map, open with visualization tools example image
```bash
rosservice call /save_map '/workspace/data/test' 0.0
```

![](assets/readme/save_mesh_pcd.png)
# Acknowledgement

- [PRBonn/vdbfusion](https://github.com/PRBonn/vdbfusion)

- [PRBonn/vdbfusion_ros](https://github.com/PRBonn/vdbfusion_ros)

- Style Formate: [https://github.com/ethz-asl/linter](https://github.com/ethz-asl/linter)

  ```bash
  cd $YOUR_REPO
  init_linter_git_hooks # install
  linter_check_all # run
  
  init_linter_git_hooks --remove # remove
  ```