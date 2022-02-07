
# visual slam

### Tools/Algorithms/tech-stack used for creating this demo:
- igntion-gazebo fortress, ros2.
- [rtabmap_ros](https://github.com/introlab/rtabmap_ros) for autonomous localisation and mapping using stereo camera.

### Instructions to run the demo:
- build/install ROS2 foxy, rosdep, colcon and cyclonedds.
- build/install ignition-gazebo fortress version.
- Run following commands:
  ```zsh
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws
  git clone git@github.com:TW-Gazebo/sandwich_bot_2.0-ros2.git ./src/sandwich_bot_2_0 -b demo
  source /opt/ros/foxy/setup.zsh
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy -y
  colcon build --packages-up-to sandwich_bot_2_0_desktop
  source ./install/setup.zsh
  # switch middleware to cyclone dds using the following env variable.
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  # launch all components
  ros2 launch sandwich_bot_2_0_desktop visual_slam_start.launch.py use_sim_time:=True
  ```
- open second terminal. move the bot using teleop-tools.
  ```zsh
  ros2 run key_teleop key_teleop --ros-args -r key_vel:=sandwich_bot_base_controller/cmd_vel_smoother
  ```
- Now you should be able to see 3d point-cloud forming around the bot !
