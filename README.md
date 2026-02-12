# FASTLIO2 ROS2

>**FRANCOR**: We have expanded SLAM a little so that it can also be used in the simulator. To do this, we added a topic for a point cloud and a second topic for the IMU, because the original SLAM version expects the IMU data in the unit g, which does not match the ROS2 message.

## Main Features
1. Refactored [FASTLIO2](https://github.com/hku-mars/FAST_LIO) adapted for ROS2
2. Added loop closure node based on position prior + ICP for loop detection, GTSAM-based pose graph optimization
3. Added relocalization node based on coarse-to-fine two-stage ICP for relocalization
4. Added consistent map optimization based on [BLAM](https://github.com/hku-mars/BALM) (small scene maps) and [HBA](https://github.com/hku-mars/HBA) (large scene maps)

## Environment Requirements
1. Ubuntu 24.04
2. ROS2 Jazzy

## Build Dependencies
```text
pcl
Eigen
sophus
gtsam
livox_ros_driver2
```

## Detailed Instructions
### 1. Compile LIVOX-SDK2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

### 2. Compile livox_ros_driver2
```shell
mkdir -r ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```

### 3. Compile Sophus
```shell
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```

>**Note**: we used here the ros-jazzy-sophus as alternative. For this we added an plain include in CMakeLists.txt. Yes not the best way to do...

**The newer version of Sophus depends on fmt. You can add add_compile_definitions(SOPHUS_USE_BASIC_LOGGING) in CMakeLists.txt to remove this dependency, otherwise it will throw an error**


## Example Dataset
```text
Link: https://pan.baidu.com/s:1rTTUlVwxi1ZNo7ZmcpEZ7A?pwd=t6yb Extract code: t6yb 
--Shared from Baidu Cloud Super Member v7
```

## Usage Scripts

### 1. LiDAR Inertial Odometry 
```shell
ros2 launch fastlio2 lio_launch.py
ros2 bag play your_bag_file
```

### 2. Odometry with Loop Closure
#### Launch Loop Closure Node
```shell
ros2 launch pgo pgo_launch.py
ros2 bag play your_bag_file
```
#### Save Map
```shell
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: 'your_save_dir', save_patches: true}"
```

### 3. Odometry with Relocalization
#### Launch Relocalization Node
```shell
ros2 launch localizer localizer_launch.py
ros2 bag play your_bag_file // optional
```
#### Set Relocalization Initial Value
```shell
ros2 service call /localizer/relocalize interface/srv/Relocalize "{"pcd_path": "your_map.pcd", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
```
#### Check Relocalization Result
```shell
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
```

### 4. Consistent Map Optimization
#### Launch Consistent Map Optimization Node
```shell
ros2 launch hba hba_launch.py
```
#### Call Optimization Service
```shell
ros2 service call /hba/refine_map interface/srv/RefineMap "{"maps_path": "your maps directory"}"
```
**If you need to call the optimization service, you need to set save_patches to true when saving the map**

## Acknowledgments
1. [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
2. [BLAM](https://github.com/hku-mars/BALM)
3. [HBA](https://github.com/hku-mars/HBA)
## Performance Related Issues
This code mainly uses timerCB as the frequency-triggered main function. Since timer, subscriber, and service callbacks in ROS2 actually run on the same thread, blocking calls may occur when computer performance is not optimal. It is recommended to use thread concurrency to separate time-consuming callbacks (such as timerCB) to improve performance

