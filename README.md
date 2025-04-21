# Introduction

This repository contains ros-ign based code for Sandwich-bot-2. Sandwich-bot-2 is a successor of sandwich-bot-1. What's new in 2?

- Hardware features:
  - bigger in size.
  - More number of sensors: imu, stereo camera and lidar are added for slam and autonomous navigation
  - improved hardware with bigger motors, efficient battery utilization etc.
  - modular design of hardware components.

- Software Features:
  - Behaviour modeling using behaviour trees.
  - Mapping using slam toolbox.
  - Autonomous navigation from one point to another using navigation2 tech stack.
  - rrt-exploration for autonomous mapping.
  - sensor fusion such as lidar-camera fusion.
  - rtabmap for visual slam using stereo camera.

The following gif shows the fast-forwarded video of sandwich bot picking up red boxes scattered around the floor.

![](demos/box_picker/media/demo.gif)

## note:
- checkout examples in `demos` directory for instructions to run the code. 
