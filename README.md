# Welcome to the vrx autonav repository

Problem Statement

The task is to have an USV (Unmanned Surface Vessel) traverse a lawn mower pattern while performing obstacle avoidance.  Place a minimum of  2 obstacles of your choice (buoy, marker, etc.) in the USV's proposed trajectory. You are free to choose your sensor suite.

The VRX Simulation can be used to showcase the USV and the trajectories (https://github.com/osrf/vrx)

The usage of the packages are given below

### load_env
This loads the simulation environment as rviz along with the damped setpoint pid  controller from `vrx_control` this establishes an `odometry` topic and a `cmd_vel` topic 

### vrx_control
This package takes care of the control of the robot. and ensures that it moves with the required velocity with the help of pid controllers whose setpoints get damped to prevent overshooting and to compensate for the lack of friction in a less viscous medium such as water. This package was written in rust as it made it easier to write multithreaded code.

### vrx_mb
The movebase package with parameters optimized for wamv. It doesn't require a static map and is capable of navigating dynamic environments such as seas and rivers. It takes the tf between `odom` and `wamv/base_link` which is published by the ekf node and the `PointCloud2` data from the velodyne lidar to make navigation decisions. 

### vrx_sensor_fusion
This establishes the `odometry` topic which is used in the control loops and also publishes the tf between the `odom` frame and the `wamv/base_link` frame which is used by movebase

### lawnmover_trajectory
Contains a python script which publishes movebase goals in a loop to make sure the robot moves in a lawnmover-esque scanning pattern.


## OUTPUT
The obstacle avoidance with the help of movebase can be found here: https://drive.google.com/file/d/1xADmPRtv4KyFTQObUaaijxLsCfZgC9Zx/view?usp=sharing

The lawn mover trajectory following without obstacles can be found here: https://drive.google.com/file/d/1BhBBEkl2RxrfcNAThcOiydU5ELrSKpy7/view?usp=sharing

The lawn mover trajectory following with obstacles can be found in these videos:
https://drive.google.com/file/d/1au-6pKYp6INkZHwNyPMBTwCpxChOsKAz/view?usp=sharing <br>
https://drive.google.com/file/d/1i_3ZMv6Mb-NM1FmNkvXbNzS2zwQ5007m/view?usp=sharing

### NOTE: Watch them at atleast 3x speed
