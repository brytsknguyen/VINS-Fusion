# [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) branch for OpenCV 4.2.0
**[A General Optimization-based Framework for Local Odometry Estimation with Multiple Sensors](https://arxiv.org/pdf/1901.03638.pdf)**

## Modifications:
1. all CMakeFiles.txt: set(CMAKE_CXX_FLAGS "-std=c++14")
2. #include <opencv2/imgproc/types_c.h>
   - camera_model/src/chessboard/Chessboard.cc
3. CV_AA = cv::LINE_AA, CV_GRAY2BGR = cv::COLOR_GRAY2BGR, CV_RGB2GRAY = cv::COLOR_RGB2GRAY
   - camera_model/src/intrinsic_calib.cc
   - camera_model/src/calib/CameraCalibration.cc
   - camera_model/src/chessboard/Chessboard.cc
   - vins_estimator/src/featureTracker/feature_tracker.cpp
4. cv::CALIB_CB_ADAPTIVE_THRESH, cv::CALIB_CB_NORMALIZE_IMAGE, cv::CALIB_CB_FILTER_QUADS, cv::CALIB_CB_FAST_CHECK
   - camera_model/src/chessboard/Chessboard.cc:
5. cv::FONT_HERSHEY_SIMPLEX
   - loop_fusion/src/pose_graph.cpp
6. CV_LOAD_IMAGE_GRAYSCALE = cv::IMREAD_GRAYSCALE
   - vins_estimator/src/KITTIOdomTest.cpp
   - vins_estimator/src/KITTIGPSTest.cpp
7. modify output_path & pose_graph_save_path ("./output" & "./output/pose_graph")
   - .yaml in config folder

## 1. Prerequisites
### Ubuntu 20.04.4-LTS
* Python 3.8.10
* OpenCV 4.2.0

### ROS1 installation
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
export ROS1_DISTRO=noetic # kinetic=16.04, melodic=18.04, noetic=20.04
sudo apt-get install ros-$ROS1_DISTRO-desktop-full
sudo apt-get install python3-catkin-tools python3-osrf-pycommon # ubuntu 20.04
sudo apt-get install libeigen3-dev libboost-all-dev libceres-dev
```
```
echo "alias source_ros1=\"source /opt/ros/$ROS1_DISTRO/setup.bash\"" >> ~/.bashrc
echo "alias source_devel=\"source devel/setup.bash\"" >> ~/.bashrc
source ~/.bashrc
```

### [Ceres Solver installation](http://ceres-solver.org/installation.html)
```
sudo apt-get install cmake 
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libsuitesparse-dev
```
```
wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz
tar zxf ceres-solver-2.1.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.1.0
make -j4
make install
```
## 2. Build on ROS
```
sudo apt install metis
source_ros1
```
```
make -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/rkuo/VINS-Fusion.git
cd ../
catkin build
source devel/setup.bash
```
<span style="color: red;">*The build took ~30 hours on i7-920 @2.67GHz*<span style="color: red;"><br>

## 3. Exercises:
### VI-Car
![](https://github.com/rkuo2000/Robotics/blob/gh-pages/images/VINS-Fusion_vi_car.png?raw=true)
* VI-Car
  - Download [car bag](https://drive.google.com/open?id=10t9H1u8pMGDOI6Q2w2uezEq5Ib-Z8tLz) to YOUR_DATASET_FOLDER.
  - `roslaunch vins vins_rviz.launch`
  - `rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/vi_car/vi_car.yaml`
  - (optional) `rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/vi_car/vi_car.yaml`
  - `rosbag play YOUR_DATASET_FOLDER/car.bag`
  
### EuRoC-MAV
**MH_01_easy**<br>
![](https://github.com/rkuo2000/Robotics/blob/gh-pages/images/VINS-Fusion_MH_01_easy.png?raw=true)
![](https://github.com/rkuo2000/Robotics/blob/gh-pages/images/VINS-Fusion-EoRoC-MAV-MH_01_easy.gif?raw=true)

* Monocualr camera + IMU
  - `roslaunch vins vins_rviz.launch`
  - `rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml`
  - (optional) `rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml`
  - `rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag`

