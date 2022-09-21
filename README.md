# VDBFUSION_ROS_MAPPING

This repo is modified version of vdbfusion for mapping incrementally based on received odometry and corresponding point cloud message. The whole process is based on the ROS1, please check [origin repo of vdbfusion](https://github.com/PRBonn/vdbfusion) if you'd like to use directly.

# Install

Only provide the Docker version for convenient  usage, more dependencies or install on your own environment please check their origin repo link.



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
