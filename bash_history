make test --debug
python
make 
make test
cd Desktop/Beckhoff/ADS/example/
make
make test
make
make test
make
make test
python example.py 
cd Desktop/Beckhoff/pyads/
python example.py 
python
wget -qO - https://typora.io/linux/public-key.asc | sudo apt-key add -
sudo add-apt-repository 'deb https://typora.io/linux ./'
sudo apt-get update
sudo apt-get install typora
typora
cd Desktop/Beckhoff/pyads/
python example.py 
clear
python example.py 
roslaunch realsense2_camera rs_camera.launch 
roslaunch realsense2_camera rs_rgbd.launch 
roslaunch realsense2_camera rs_rtabmap.launch 
roscd realsense2_camera/
rosrun rviz rviz 
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
tmux
sudo apt-get remove -purge teamviewer 
sudo apt-get remove -p teamviewer 
sudo apt-get purge teamviewer 
roslaunch rtabmap_ros cartographer.launch 
grep -r "remap"
roslaunch realsense2_camera rtapmap.launch 
rostopic list
rosrun rqt_tf_tree rqt_tf_tree 
rostopic list
clear
rostopic list
roscd octomap
htop
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
roslaunch turtlebot_gazebo turtlebot_world.launch
roslaunch turtlebot3_gazebo turtlebot3_world.launch 
source ~/.bsahrc
source ~/.bashrc
roslaunch turtlebot3_gazebo turtlebot3_world.launch 
source ~/.bashrc
roslaunch turtlebot3_gazebo turtlebot3_world.launch 
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
rosrun rqt_graph rqt_graph 
rosrun rqt_reconfigure rqt_reconfigure 
roslaunch rtabmap_ros rgbd_mapping.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
roslaunch rtabmap_ros demo_find_object.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
roslaunch rtabmap_ros demo_turtlebot_rviz.launch
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
roslaunch rtabmap_ros demo_turtlebot_mapping.launch simulation:=true
source ~/.bashrc
roslaunch rtabmap_ros demo_turtlebot_mapping.launch simulation:=true
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
tmux
rostopic list
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
rostopic info /rtabmap/octomap_full 
rviz
rostopic list
rostopic echo /camera/color/camera_info -n1
rostopic echo /camera/de/camera_info -n1
rostopic echo /camera/depth/camera_info -n1
htop
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
tmux
gedit ~/tf_tutorial.launch
rviz
roslaunch tf_tutorial.launch 
rtabmap-databaseViewer ~/.ros/rtabmap.db 
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
tmux
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
roslaunch rtabmap_ros demo_hector_mapping.launch hector:=true
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_apps.git
cd ..
rosdep install --from-paths ./src --ignore-src --rosdistro=kinetic
catkin build
roslaunch turtlebot_bringup minimal.launch
tmux
rostopic list
sudo apt-get install ros-kinetic-turtlebot-bringup ros-kinetic-turtlebot-navigation ros-kinetic-rtabmap-ros
rostopic list
roslaunch rtabmap_ros demo_turtlebot_mapping.launch
rostopic list
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_bringup 3dsensor.launch 
roslaunch rtabmap_ros demo_turtlebot_mapping.launch args:="--delete_db_on_start"
grep -r "realsense"
rosrun rqt_tf_tree rqt_tf_tree 
rostopic list
rviz
roslaunch turtlebot3_slam turtlebot3_gmapping.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
rostopic list
roslaunch rtabmap_ros demo_turtlebot_mapping.launch
roslaunch rtabmap_ros demo_turtlebot_mapping.launch simulation:=true
roslaunch turtlebot3_bringup turtlebot3_realsense.launch 
source ~/.bashrc
roslaunch turtlebot3_bringup turtlebot3_realsense.launch 
roslaunch realsense2_camera rs_camera.launch 
grep -r "realsense"
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_gazebo turtlebot_world.launch
env | grep TURTLEBOT_
export TURTLEBOT_SIMULATION=true
roslaunch turtlebot_bringup minimal.launch
env | grep TURTLEBOT_
roslaunch turtlebot_bringup minimal.launch robot_name:=bob --screen
roslaunch turtlebot_bringup minimal.launch
roslaunch rtabmap_ros demo_turtlebot_mapping.launch
roslaunch rtabmap_ros demo_turtlebot_mapping.launch args:="--delete_db_on_start"
roslaunch rtabmap_ros demo_turtlebot_rviz.launch
roslaunch rtabmap_ros demo_turtlebot3_navigation.launch
rostopic list
rostopic info /map
rostopic list
rviz
rosrun rqt_tf_tree rqt_tf_tree 
rosrun rqt_graph rqt_graph 
roslaunch rtabmap_ros rgbd_mapping.launch 
roslaunch rtabmap_ros demo_turtlebot3_navigation.launch
rostopic list
rviz
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_bringup turtlebot3_realsense.launch
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_bringup turtlebot3_realsense.launch 
roslaunch turtlebot3_gazebo turtlebot3_world.launch
tmux
git clone https://github.com/OctoMap/octomap_rviz_plugins.git
cd ..
catkin build
rosrun rqt_tf_tree rqt_tf_tree 
roscd octomap
roscd octomap_msgs/
roscd octomap_rviz_plugins/
rviz
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
tmux
git clone https://github.com/googlecartographer/cartographer_ros.git
ddcd ..
cd ..
catkin build
git clone https://github.com/googlecartographer/cartographer.git
cd ..
catkin build
grep -r "g2o"
git clone https://ceres-solver.googlesource.com/ceres-solver
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-1.14.0
cmake ../ceres-solver
cmake ..
make -j16
make test
make install
sudo make install
chmod +x plot_results.py 
python plot_results.py 
bin/simple_bundle_adjuster ../ceres-solver-1.14.0/data/problem-16-22106-pre.txt
bin/simple_bundle_adjuster ../data/problem-16-22106-pre.txt 
# CMake
sudo apt-get install cmake
sudo apt-get install libgoogle-glog-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libsuitesparse-dev
cd ..
catkin build
rosdep install --from-paths ./src --ignore-src --rosdistro=kinetic
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin build
sudo apt install lua5.3
catkin build
cd workspaces/3d_slam_ws/
catkin build
lua
lua5.3 
sudo apt purge lua5.3
sudo apt install lua5.2
cd workspaces/3d_slam_ws/
catkin build
htop
rviz
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
tmux
catkin build
cd /home/brky/workspaces/3d_slam_ws/build/cartographer; catkin build --get-env cartographer | catkin env -si  /usr/bin/cmake /home/brky/workspaces/3d_slam_ws/src/cartographer --no-warn-unused-cli -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/3d_slam_ws/devel; cd -
apt-get install lua5.2 liblua5.2-dev
sudo apt-get install lua5.2 liblua5.2-dev
cd /home/brky/workspaces/3d_slam_ws/build/cartographer; catkin build --get-env cartographer | catkin env -si  /usr/bin/cmake /home/brky/workspaces/3d_slam_ws/src/cartographer --no-warn-unused-cli -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/3d_slam_ws/devel; cd -
apt-get install python-sphinx
sudo apt-get install python-sphinx
catkin build
./install_proto3.sh
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build
./install_proto3.sh
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin build
cd workspaces/3d_slam_ws/
catkin build
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build
mkdir catkin_ws
cd catkin_ws
wstool init src
catkin clean
catkin build
wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
git clone https://github.com/googlecartographer/cartographer.git
git clone https://github.com/googlecartographer/cartographer_ros.git
src/cartographer/scripts/install_proto3.sh
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-14-14-00.bag
rostopic list
rostopic info /vertical_laser_2d 
rostopic info /horizontal_laser_2d 
rostopic list
rostopic echo /imu -n1
rosrun rqt_tf_tree rqt_tf_tree 
rosrun rqt_graph rqt_graph 
rostopic list
rosrun map_server map_saver -f cartographer
rostopic list
roslaunch agv_simulation agv_slam_gmapping.launch 
rosrun rqt_tf_tree rqt_tf_tree 
rostopic list
rostopic info /scan
rostopic info /odom
rosrun rqt_tf_tree rqt_tf_tree 
rostopic echo /tf -n1
rostopic echo /tf_static -n1
rostopic info /tf
rostopic info /tf_static 
rosrun rqt_tf_tree rqt_tf_tree 
rostopic echo /scan -n1
rostopic echo /scan/header -n1
rosrun rqt_tf_tree rqt_tf_tree 
rostopic list
rostopic info /odom 
roslaunch agv_simulation agv_slam_gmapping.launch 
roslaunch laser_scan_matcher agv_demo.launch 
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag
cd Downloads/
ls
rosparam set use_sim_time true
rosbag play cartographer_paper_deutsches_museum.bag 
rosbag play cartographer_paper_deutsches_museum.bag /vertical_laser_2d:=scan
roscore
rostopic list
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch laser_scan_matcher agv_demo.launch 
roslaunch agv_simulation agv_navigation.launch 
roslaunch agv_simulation agv_rviz.launch 
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch agv_sla
roslaunch agv_simulation agv_slam_
roslaunch agv_simulation agv_slam_gmapping.launch 
roslaunch agv_simulation agv_gazebo.launch 
tmux
roslaunch agv_simulation agv_slam_cartographer.launch 
rosrun rqt_tf_tree rqt_tf_tree 
rosbag play ~/Downloads/cartographer_paper_deutsches_museum.bag /vertical_laser_2d:=scan
rosbag play ~/Downloads/cartographer_paper_deutsches_museum.bag 
rostopic info /vertical_laser_2d 
roscd laser_proc/
rosrun laser_proc laser_proc 
rosrun laser_proc laser_proc first:=scan
rostopic list
clear
rostopic list
clear
rostopic list
clear
rostopic list
rostopic info /first 
rostopic list
rostopic info /scan
rostopic info /scan/header
rostopic echo /scan/header -n1
rostopic echo /odom/header -n1
rosbag play ~/Downloads/cartographer_paper_deutsches_museum.bag 
rosbag play ~/Downloads/cartographer_paper_deutsches_museum.bag /vertical_laser_2d:=echos
rosbag play ~/Downloads/cartographer_paper_deutsches_museum.bag /vertical_laser_2d:=echoes
rostopic list
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch agv_simulation agv_rviz.launch 
roslaunch agv_simulation agv_slam_cartographer.launch 
roslaunch laser_scan_matcher agv_demo.launch 
roscore
tmux
rostopic list
rostopic echo /echoes/header -n1
rosrun laser_proc laser_proc 
rosrun laser_proc laser_proc first:=scan
rosbag play ~/Downloads/cartographer_paper_deutsches_museum.bag /vertical_laser_2d:=echoes
rviz
rosrun rqt_graph rqt_graph 
rostopic info /first 
rostopic info /scan 
rostopic echo /scan/header -n1 
rostopic list
clear
rostopic list
rosrun rqt_tf_tree rqt_tf_tree 
rostopic list
rostopic info /TF
rostopic info /tf
roslaunch laser_scan_matcher agv_demo.launch scan:=first
roslaunch laser_scan_matcher agv_demo.launch 
roslaunch agv_simulation agv_slam_cartographer.launch 
rosparam set use_sim_time true
roslaunch agv_simulation agv_slam_cartographer.launch 
rostopic echo /joint_states -n1
rosbag play ~/Downloads/cartographer_paper_deutsches_museum.bag 
rostopic list
rostopic echo /first 
rostopic echo /first/header -n1
rosrun rqt_tf_tree rqt_tf_tree 
tmux
roscore
clear
roscore
rosrun laser_proc laser_proc first:=scan
rosparam set use_sim_time true
roslaunch agv_simulation agv_slam_cartographer.launch 
rostopic info /scan
rostopic info /odom
rosrun rqt_tf_tree rqt_tf_tree 
rostopic echo /odom/header -n1
rostopic echo /odom/h -n1
rostopic echo /odom /
rostopic echo /odom 
rostopic list
rostopic echo /scan/header -n1
rostopic echo /scan/header 
rostopic list
rostopic echo /scan
rostopic echo /echoes 
rostopic echo /scan 
rostopic echo /scan/header 
rosrun rqt_tf_tree rqt_tf_tree 
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 laser base_link 1000
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 laser vertical_laser_link  1000
rostopic list
rostopic echo /vertical_laser_2d 
rosrun laser_proc laser_proc first:=scan
rosbag play ~/Downloads/cartographer_paper_deutsches_museum.bag 
rosbag play ~/Downloads/cartographer_paper_deutsches_museum.bag /vertical_laser_2d:=echoes
tmux
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin build
cd ..
ls
cd bags/
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-13-54-42.bag
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=/home/brky/Downloads/b3-2016-04-05-14-14-00.bag
rosbag info b3-2016-04-05-14-14-00.bag 
rosbag reindex b3-2016-04-05-14-14-00.bag 
rosbag info b3-2016-04-05-14-14-00.bag 
rostopic list
rosbag play ~/Downloads/b3-2016-04-05-14-14-00.bag 
rosbag play ~/Downloads/b3-2016-04-05-14-14-00.bag -r
rosbag play -r ~/Downloads/b3-2016-04-05-14-14-00.bag 
rosbag play -R ~/Downloads/b3-2016-04-05-14-14-00.bag 
rosbag play help
rosbag -r play ~/Downloads/b3-2016-04-05-14-14-00.bag 
rosbag play ~/Downloads/b3-2016-04-05-14-14-00.bag -R
rosbag play -r 1 ~/Downloads/b3-2016-04-05-14-14-00.bag 
rosbag play -r 2 ~/Downloads/b3-2016-04-05-14-14-00.bag 
rosbag play -l ~/Downloads/b3-2016-04-05-14-14-00.bag 
roscore
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag
tmux
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
roscore
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
rosbag play -l ~/Downloads/b3-2016-04-05-14-14-00.bag 
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
rosrun rqt_graph rqt_graph 
rostopic list
rosbag record -a
clear
rostopic list
rostopic info /vertical_laser_3d 
clear
rostopic list
rostopic info /voxel_cloud 
rosbag record /voxel_cloud /rtabmap/imu /rtabmap/odom
cartographer_rosbag_validate -bag_filename realsense.bag 
rosrun cartographer_ros cartographer_rosbag_validate realsense.bag 
rosrun cartographer_ros cartographer_rosbag_validate -b realsense.bag 
rosrun cartographer_ros cartographer_rosbag_validate
rosrun cartographer_ros cartographer_rosbag_validate --help
rosrun cartographer_ros cartographer_rosbag_validate -bag_filename realsense.bag 
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/realsense.bag
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/realsense.bag vertical_laser_3d:=voxel_cloud imu:=rtabmap/imu
rostopic echo /imu 
rostopic echo /scan 
rostopic list

rosrun laser_proc laser_proc
rosrun laser_proc laser_proc first:=scan
roscore
rosbag play cartographer_paper_deutsches_museum.bag 
rosbag play cartographer_paper_deutsches_museum.bag horizontal_laser_2d:=scan
rosbag play cartographer_paper_deutsches_museum.bag horizontal_laser_2d:=echoes
rostopic list
rostopic echo /scan 
rostopic info /scan 
clear
rostoic list
rostopic list
rostopic echo /first 
rostopic info /scan 
rosnode info /laser_proc 
rostopic echo /first 
rostopic info /first 
rostopic list
rostopic echo /scan 
rosbag record /scan /imu -o museum.bag
rostopic list
rostopic echo /imu 
rostopic echo /scan 
rostopic info /scan 
rosbag play museum_2019-09-30-16-48-36.bag 
roscore
rosbag play museum_2019-09-30-16-48-36.bag 
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 base_link horizontal_laser_link  1000
rosrun tf tf_empty_listener /base_link
rosrun rqt_graph rqt_graph 
rosrun rqt_tf_tree rqt_tf_tree 
rviz
rosrun tf tf_monitor 
rosparam get /use_sim_time 
rosbag play museum_2019-09-30-16-48-36.bag 
rosbag play museum_2019-09-30-16-48-36.bag -l
rostopic echo /scan/header -n1
roslaunch laser_scan_matcher museum.launch 
rosrun tf tf_monitor
rosparam get /use_sim_time
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 /base_link /horizontal_laser_link  1000
rosrun rqt_tf_tree rqt_tf_tree 
rosparam set /use_sim_time true
rosrun rqt_tf_tree rqt_tf_tree 
roscore
tmux
roslaunch laser_scan_matcher 
roslaunch laser_scan_matcher museum.launch 
rostopic list
rostopic echo /odom 
roslaunch agv_simulation agv_slam_gmapping.launch 
rosparam set /use_sim_time true
roslaunch agv_simulation agv_slam_gmapping.launch 
rosrun rqt_tf_tree rqt_tf_tree 
rosparam set /use_sim_time false
roslaunch agv_simulation agv_slam_gmapping.launch 
rosrun rqt_tf_tree rqt_tf_tree 
rosbag play museum_2019-09-30-16-48-36.bag 
rosbag play museum_2019-09-30-16-48-36.bag -l
rosparam set /use_sim_time true
rosbag play museum_2019-09-30-16-48-36.bag -l
rostopic list
rostopic echo /map
rosrun rqt_tf_tree rqt_tf_tree 
rviz
rosrun rviz rviz 
rosparam get /use_sim_time 
rosparam set /use_sim_time true
rosbag play museum_2019-09-30-16-48-36.bag 
rostopic list
rosrun rqt_tf_tree rqt_tf_tree 
rostopic list
rosrun rviz rviz
roscore
roslaunch laser_scan_matcher museum.launch 
roslaunch laser_scan_matcher demo_gmapping.launch 
roslaunch laser_scan_matcher museum.launch 
rosparam set /use_sim_time true
rosparam get /use_sim_time
roslaunch laser_scan_matcher museum.launch 
rostopic info /scan
rostopic info /odom
roscore
rosrun rqt_tf_tree rqt_tf_tree 
roswtf 
rosrun rqt_graph rqt_graph 
rostopic info /map
rostopic echo /map
rosrun rqt_tf_tree rqt_tf_tree 
rostopic echo /odom/header -n1
rostopic list
rostopic echo /odom 
rostopic list
rostopic echo /tf
rostopic echo /tf_static 
rostopic info /tf_static 
rostopic info /tf
rostopic info /scan
rostopic info /tf
rostopic info /tf_static 
roscore
rostopic list
rosrun rqt_graph rqt_graph 
roscore
rostopic info /scan
rosconsole set /slam_gmapping ros.gmapping.message_filter debug 
rostopic echo /scan/header
rosbag play museum_2019-09-30-16-48-36.bag --clock
rosbag play museum_2019-09-30-16-48-36.bag --clock -l
rosbag play museum_2019-09-30-16-48-36.bag --clock 
roscd laser_scan_matcher/launch/
rosbag play demo.bag --clock
rosbag play museum.bag --clock
rosbag fix museum.bag 
rosbag fix museum.bag museum_fixed.bag
rosrun rqt_tf_tree rqt_tf_tree 
rviz
rosrun rqt_tf_tree rqt_tf_tree 
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch agv_simulation agv_slam_gmapping.launch 
rosparam set /use_sim_time true
roslaunch agv_simulation agv_slam_gmapping.launch 
roslaunch agv_simulation agv_rviz.launch 
rostopic echo /scan/header -n1
rosrun rqt_tf_tree rqt_tf_tree 
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link horizontal_laser_link 100
tmux
rostopic list
rosbag play museum.bag 
roscore
roslaunch agv_simulation agv_slam_gmapping.launch 
roslaunch laser_scan_matcher museu│done                                              │brky@brky-AIR:~$ rostopic echo /scan/header^C
roslaunch laser_scan_matcher museum.launch 
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch agv_simulation agv_rviz.launch 
rostopic list
clear
rostopic info /scan
rostopic info /imu
rosbag check b3-2016-04-05-13-54-42.bag 
clear
rostopic list
rostopic echo /scan 
clear
rosbag record /imu /scan -O deutsches_museum.bag
roscore
rosbag play cartographer_paper_deutsches_museum.bag --clock
rosbag play --clock ~/Downloads/cartographer_paper_deutsches_museum.bag /vertical_laser_2d:=echoes 
rosbag play --clock ~/cartographer_paper_deutsches_museum.bag /vertical_laser_2d:=echoes 
rosrun laser_proc laser_proc first:=scan
tmux
rosbag check deutsches_museum.bag 
rostopic echo /scan/header
rosparam set /use_sim_time true
roslaunch laser_scan_matcher museum.launch 
roscd laser_scan_matcher/
cd launch/
rosbag play deutsches_museum.bag 
roslaunch agv_simulation agv_slam_gmapping.launch 
roscore
roscd laser_scan_matcher/
cd launch/
rostopic list
rostopic echo /scan 
rostopic list
rostopic echo /imu 
rostopic echo /scan/header
rosbag record /imu /scan -O deutsches_museum.bag
rosbag play --clock ~/cartographer_paper_deutsches_museum.bag /horizontal_laser_2d:=echoes
rosrun laser_proc laser_proc first:=scan
rosbag check deutsches_museum.bag 
tmux
roscore
roslaunch laser_scan_matcher museum.launch 
roslaunch agv_simulation agv_slam_gmapping.launch 
rosrun rqt_tf_tree rqt_tf_tree 
rosparam set /use_sim_time true
roslaunch laser_scan_matcher museum.launch 
roslaunch laser_scan_matcher museum.launch 
rosrun rqt_tf_tree rqt_tf_tree 
roscore
rosrun rqt_tf_tree rqt_tf_tree 
roscore
roslaunch laser_scan_matcher museum.launch 
roscd laser_scan_matcher/launch/
rosbag fix museum.launch 
rosbag fix deutsches_museum.bag 
rosbag play deutsches_museum.bag --clock
rosrun rqt_tf_tree rqt_tf_tree 
rosrun rviz rviz 
rosrun laser_scan_matcher laser_scan_matcher_node 
roslaunch laser_scan_matcher museum.launch 
rosrun rqt_tf_tree rqt_tf_tree 
rosrun rqt_tf_tree 
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch laser_scan_matcher museum.launch 
rosrun rqt_tf_tree rqt_tf_tree 
rosrun rqt_tf_tree rqt_tf_tree 
roscd laser_scan_matcher/launch/
rosbag play deutsches_museum.bag --clock
roslaunch laser_scan_matcher museum.launch 
rosparam set /use_sim_time true
roslaunch laser_scan_matcher museum.launch 
rosparam set /use_sim_time true
rosbag play deutsches_museum.bag --clock
roscd laser_scan_matcher/launch/
rosbag play deutsches_museum.bag --clock
roscore
rosrun rqt_tf_tree rqt_tf_tree 
rosparam get /use_sim_time
rostopic list
rosparam get /use_sim_time
rosrun rqt_tf_tree rqt_tf_tree 
rostopic list
rostopic echo /scan/header
rostopic list
rostopic echo /clock 
rostopic info /clock 
rostopic echo /clock 
rostopic info /clock 
roslaunch laser_scan_matcher museum.launch 
roscd laser_scan_matcher/launch/
rosbag play deutsches_museum.bag 
rosbag play deutsches_museum.bag --clock
rosbag play deutsches_museum.bag 
rosbag play--clock deutsches_museum.bag 
rosbag play --clock deutsches_museum.bag 
rosparam set /use_sim_time true
rosbag play --clock deutsches_museum.bag 
rosbag play --help
rosbag play deutsches_museum.bag 
rosbag play --clock deutsches_museum.bag 
rosrun rqt_tf_tree rqt_tf_tree 
rosparam get /use_sim_time 
rostopic list
rostopic echo /clock
rostopic list
rostopic echo /clock
rostopic echo /scan/header
rosbag record /scan 
rostopic echo /scan/header
rostopic list
rostopic echo /scan/header
rosbag record /scan 
rostopic echo /scan/header
rostopic list
rostopic echo /clock 
rosbag play 2019-10-01-11-44-54.bag --clock
rosbag play 2019-10-01-11-44-54.bag --clock -l
rosbag play 2019-10-01-11-46-45.bag --clock -l
roslaunch laser_scan_matcher museum.launch 
roscd laser_scan_matcher/launch/
rosbag play demo.bag --clock
rosbag play deutsches_museum.bag --clock
rosbag play demo.bag --clock
rosbag play deutsches_museum.bag --clock
rosparam get /use_sim_time
rosparam set /use_sim_time true
rosbag play deutsches_museum.bag --clock
roslaunch agv_simulation agv_slam_gmapping.launch 
roscore
rostopic info /odom 
rosnode info /slam_gmapping 
roslaunch laser_scan_matcher demo_gmapping.launch 
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch laser_scan_matcher museum.launch 
rosrun rqt_tf_tree 
rosrun rqt_tf_tree rqt_tf_tree 
htop
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch agv_simulation agv_slam_gmapping.launch 
roslaunch laser_scan_matcher museum.launch 
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-14-14-00.bag
rostopic info /vertical_laser_3d 
rostopic list
rostopic info /scan_matched_points2 
htop
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '/home/brky/Downloads/b3-2016-04-05-13-54-42.bag.pbstream', include_unfinished_submaps: 'true'}"
rosservice call /write_state "filename: '/home/brky/Downloads/b3-2016-04-05-13-54-42.bag',include_unfinished_submaps: true" 
rosservice call /write_state "filename: '/home/brky/Downloads/b3-2016-04-05-13-54-42.bag' include_unfinished_submaps: true" 
rosservice call /write_state "filename: '/home/brky/Downloads/b3-2016-04-05-13-54-42.bag.pbstream', include_unfinished_submaps: true" 
rosservice call /write_state "filename: '/home/brky/Downloads/b3-2016-04-05-13-54-42.bag.pbstream' include_unfinished_submaps: true" 
rosservice call /write_state "filename: '/home/brky/Downloads/b3-2016-04-05-13-54-42.bag.pbstream'
include_unfinished_submaps: true" 
roslaunch cartographer_ros assets_writer_backpack_3d.launch    bag_filenames:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag \
roslaunch cartographer_ros assets_writer_backpack_3d.launch    bag_filenames:=${HOME}/Downloads/b3-2016-04-05-13-54-42.bag    pose_graph_filename:=${HOME}/Downloads/b3-2016-04-05-13-54-42.bag.pbstream 
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/cartographer_paper_deutsches_museum.bag
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/b3-2016-04-05-13-54-42.bag
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/b3-2016-04-05-13-54-42.bag
roscd octomap
roscd octomap_msgs/
sudo apt-get purge ros-kinetic-octomap*
git clone https://github.com/OctoMap/octomap.git
cd octomap/
mkdir build
cd build/
cmake ..
make
git clone https://github.com/OctoMap/octomap_msgs.git
cd ..
catkin build
cd src/octomap-kinetic/
git clone https://github.com/OctoMap/octomap_mapping.git
catkin build
catkin clean
catkin build
git clone https://github.com/OctoMap/octomap_ros.git
roscd rgbd_launch/
ls
cd launch/
ls
gedit kinect_frames.launch 
htop
grep -R "glew"
git clone https://github.com/felixendres/rgbdslam_v2.git
cd ..
catkin build
source ~/.bashrc 
catkin build
export G2O_DIR=~/optimisers/g2o/
catkin build
export G2O_DIR=~/optimisers/g2o/g2o/
catkin build
export G2O_DIR=~/optimisers/g2o/bin/
catkin build
export G2O_DIR=~/optimisers/g2o/lib/
catkin build
export $G2O_DIR=~/optimisers/g2o/
export $G2O_DIR=~/optimisers/g2o
export G2O_DIR=~/optimisers/g2o
catkin build
export G2O_DIR=~/optimisers/g2o/install
catkin build
cd /home/brky/workspaces/3d_slam_ws/build/rgbdslam; catkin build --get-env rgbdslam | catkin env -si  /usr/bin/cmake /home/brky/workspaces/3d_slam_ws/src/rgbdslam_v2 --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/3d_slam_ws/devel/.private/rgbdslam -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/3d_slam_ws/install; cd -
sudo apt-get install build-essential libxmu-dev libxi-dev libgl-dev
mkdir build
cd build/
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DG2O_BUILD_EXAMPLES=OFF
ls
git clone https://github.com/felixendres/g2o.git
cd g2o/
ls
mkdir build
cd build/
cmake ../
make
make -j8
cd build/
cmake ./cmake
make -j8
make -j5
make -j4
sudo make -j4
git clone https://github.com/nigels-com/glew.git
cd glew/
make
ls
cd build/
make
cd ..
sudo rm -rf glew/
catkin clean
catkin build
cd ..
catkin build
cd Desktop/
bash install.sh 
export G2O_DIR=$G2O_REPO_DIR/install
echo $G2O_DIR 
G2O_REPO_DIR=$SUBDIR/g2ofork
echo $G2O_REPO_DIR 
SUBDIR=~/workspaces/3d_slam_ws/src/RGBDSLAM
G2O_REPO_DIR=$SUBDIR/g2ofork
echo $G2O_DIR 
echo $G2O_REPO_DIR 
export G2O_DIR=$G2O_REPO_DIR/install
echo $G2O_DIR 
git clone -b kinetic https://github.com/felixendres/rgbdslam_v2.git ~/workspaces/3d_slam_ws/src/RGBDSLAM/
git clone -b kinetic https://github.com/felixendres/rgbdslam_v2.git ~/workspaces/3d_slam_ws/src/RGBDSLAM/..
cd  ~/workspaces/3d_slam_ws/src/RGBDSLAM/
git clone -b kinetic https://github.com/felixendres/rgbdslam_v2.git
rosdep install rgbdslam
rosdep install rgbdslam_v2/
cd ..
catkin build
cd /home/brky/workspaces/3d_slam_ws/build/rgbdslam; catkin build --get-env rgbdslam | catkin env -si  /usr/bin/cmake /home/brky/workspaces/3d_slam_ws/src/RGBDSLAM/rgbdslam_v2 --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/3d_slam_ws/devel/.private/rgbdslam -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/3d_slam_ws/install; cd -
sudo apt-get install libglew1.13 libglew-dev 
catkin build
sudo apt-get install libdevil1c2 libdevil-dev
catkin build
roscd octomap
roscd octomap_ros/
tmux
catkin clean
catkin build
tmux
roscd octomap
roscd octomap_msgs/
roscd octomap_mapping/
roscd octomap_ros/
roscd octomap_rviz_plugins/
roscd octomap_server/
rosnode list
rosnode info /points_xyzrgb 
rosrun rqt_graph rqt_graph 
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
tmux
sudo apt-get purge ros-kinetic-libg2o libqglviewer-dev
sudo apt-get purge libqglviewer-dev
sudo apt-get purge ros-kinetic-libg2o 
sudo apt-get remove ros-kinetic-libg2o 
sudo apt-get install libsuitesparse-dev libeigen3-dev
cmake .. -DCMAKE_INSTALL_PREFIX=$G2O_REPO_DIR/install -DG2O_BUILD_EXAMPLES=OFF
cd build/
cmake .. -DCMAKE_INSTALL_PREFIX=$G2O_REPO_DIR/install -DG2O_BUILD_EXAMPLES=OFF
nice --help
nice make -j8 install
sudo nice make -j8 install
cd ..
catkin build
catkin list
rostopic list
roslaunch rgbdslam realsense.launch 
roslaunch rgbdslam realsense.launch --verbose
roslaunch rgbdslam realsense.launch --debug
roslaunch rgbdslam realsense.launch --help
roslaunch rgbdslam realsense.launch -v
roslaunch rgbdslam realsense.launch --help
roslaunch rgbdslam realsense.launch --screen
roslaunch rgbdslam realsense.launch --help
roslaunch rgbdslam realsense.launch 
gedit plc.cpp
gcc plc.cpp 
rm plc.cpp 
ls
dpkg -l 'pcl'
dpkg -l '*pcl*'
roslaunch realsense2_camera rs_camera.launch 
roslaunch realsense2_camera rs_camera.launch align_depth:=true
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz
tar -xvzf pcl-1.8.0.tar.gz
cd pcl-pcl-1.8.0/
vi
cd pcl-pcl-1.8.0/
vi Cmakelists.txt
vi
ls
rm dummy 
ls
htop
mkdir build
cd build/
cmake ../
make VERBOSE=1
make -j8
sudo make install
sudo roboware-studio /opt/ros/kinetic/share/pcl_ros/cmake/pcl_rosConfig.cmake
sudo su
sudo roboware-studio /opt/ros/kinetic/share/pcl_ros/cmake/pcl_rosConfig.cmake
sudo nano /opt/ros/kinetic/share/pcl_ros/cmake/pcl_rosConfig.cmake
sudo gedit /opt/ros/kinetic/share/pcl_ros/cmake/pcl_rosConfig.cmake
sudo apt-get install libglew-dev
sudo apt-get install libdevil1c2 libdevil-dev
make -j8
cd ..
ls
gedit CMakeLists.txt 
cd ..
catkin build
catkin build rgbdslam 
cd build/rgbdslam/
make VERBOSE=1
make
make -j16 VERBOSE=1
make install
roslaunch rgbdslam realsense.launch 
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
tmu
tmux
htop
cd workspaces/3d_slam_ws/
catkin clean
catkin build
rosparam get /use_sim_time
rosrun rqt_graph rqt_graph 
roswtf 
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
roslaunch rgbdslam realsense.launch 
rostopic info /camera/color/image_raw
rostopic info /camera/aligned_depth_to_color/image_raw
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
roslaunch realsense2_camera rs_camera.launch align_depth:=true 
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
rviz
rostopic list
clear
rostopic list
tmux
htop
clear
rostopic list
rostopic hz /rgbdslam/batch_clouds 
rostopic hz /rgbdslam/aggregate_clouds 
rosrun rqt_tf_tree rqt_tf_tree 
rostopic list
rostopic hz /rgbdslam/online_clouds 
rostopic echo /rgbdslam/online_clouds/header -n1
rostopic echo /rgbdslam/batch_clouds/header -n1
htop
wget http://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.bag
roslaunch rgbdslam test_settings.launch bagfile_name:="/home/brky/workspaces/3d_slam_ws/src/RGBDSLAM/rgbdslam_v2/test/rgbd_dataset_freiburg1_xyz.bag"
rosbag play rgbd_dataset_freiburg1_xyz.bag --clock
rosparam set /use_sim_time true
rosbag play rgbd_dataset_freiburg1_xyz.bag --clock
rosbag play rgbd_dataset_freiburg1_xyz.bag --clock -l
rostopic list
roslaunch rgbdslam rgbdslam.launch 
rviz
rostopic list
rviz
roscore
rostopic lis
rostopic list
rostopic info /camera/aligned_depth_to_color/image_raw 
rostopic info /camera/color/image_raw 
rostopic info /camera/rgb/camera_info 
roslaunch rgbdslam realsense.launch --param
roslaunch rgbdslam realsense.launch -param
rostopic list
rostopic info /camera/depth/image_rect_raw
rostopic list
roswtf
rostopic echo /camera/depth/image_rect_raw/header -n1
rostopic echo /camera/color/image_raw -n1
rostopic echo /camera/color/image_raw/header -n1
rostopic echo /camera/color/image_raw -n1
rostopic echo /camera/color/image_raw/header -n1
rostopic echo /camera/depth/image_rect_raw/header -n1
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch rgbdslam realsense.launch 
rostopic list
rostopic info /camera/rgb/camera_info
clear
roslaunch realsense2_camera rs_camera.launch 
rviz
sudo cp pcl_rosConfig.cmake  .pcl_rosConfig_rgbdslam_pcl1.8.cmake
ls
sudo mv .pcl_rosConfig_rgbdslam_pcl1.8.cmake ./pcl_rosConfig_rgbdslam_pcl1.8.cmake
sudo gedit pcl_rosConfig.cmake 
catkin clean
catkin_release 
catkin clean
catkin build 
catkin clean
catkin build 
roslaunch rtabmap_ros rtabmap.launch
cd workspaces/3d_slam_ws/
catkin build
sudo apt-get install ros-kinetic-pcl-*
sudo apt-get install libpcl1.7
sudo apt-get install libpcl-dev 
sudo apt-get clean
grep -r "libfreenect_sync.so"
catkin clean
catkin build
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
roslaunch rtabmap_ros rtabmap.launch 
tmux
catkin clean
catkin build rtabmap_ros 
sudo make uninstall
cd build/
sudo make uninstall
catkin clean 
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
tmux
catkin build rtabmap_ros 
cd /home/brky/workspaces/3d_slam_ws/build/rtabmap_ros; catkin build --get-env rtabmap_ros | catkin env -si  /usr/bin/cmake /home/brky/workspaces/3d_slam_ws/src/rtabmap_ros-kinetic-devel --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/3d_slam_ws/devel/.private/rtabmap_ros -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/3d_slam_ws/install; cd -
catkin build rtabmap_ros 
catkin build 
htop
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
tmux
roslaunch rgbdslam test_settings.launch bagfile_name:="/home/brky/workspaces/3d_slam_ws/src/RGBDSLAM/rgbdslam_v2/test/rgbd_dataset_freiburg1_xyz.bag"
ifconfig
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
tmux
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
sudo apt-get install python-wstool python-catkin-tools clang-format-3.9
htop
wstool init .
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool update -t .
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
cd ..
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build -j16
catkin build
gedit ~/.bashrc 
git clone https://github.com/ompl/ompl
cd ompl/
wget https://raw.githubusercontent.com/ros-gbp/ompl-release/debian/kinetic/xenial/ompl/package.xml
cd ..
catkin build
sudo apt-get -qq install libccd-dev
sudo apt-get -h
sudo apt-get -qq 
sudo apt-get --help
cd workspaces/move_it_ws/src/
git clone https://github.com/flexible-collision-library/fcl
cd fcl
git checkout fcl-0.5   # for kinetic
wget https://raw.githubusercontent.com/ros-gbp/fcl-release/debian/jade/fcl/package.xml
catkin build
htop
git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic
sudo apt-get install libccd-dev
source ~/.bashrc
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic
cd ..
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic
sudo apt-get install libccd2*
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic
cd ..
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic
source ~/.bashrc
cd move_it_ws/
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic
cd src/
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic
cd ..
catkin config --extend /opt/ros/kinetic
catkin build
catkin clean
catkin build
cd /home/brky/workspaces/move_it_ws/build/moveit_planners_ompl; catkin build --get-env moveit_planners_ompl | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin clean
catkin config --extend /opt/ros/kinetic
catkin build
git clone https://github.com/ros-planning/moveit.git
git clone -b kinetic-devel https://github.com/ros-planning/moveit.git
cd ..
catkin clean
catkin build
wstool update -t src
catkin build
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
cd workspaces/move_it_ws/src/
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
cd ..
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool update -t src
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
htop
mkdir build
cd build/
cmake ..
make -j16
sudo make install
htop
git clone -b kinetic-devel https://github.com/ros-planning/moveit_tutorials.git
cd ..
catkin clean
catkin build
catkin clean
catkin build
catkin clean
catkin build
catkin build ompl 
catkin build
source ~/.bashrc
catkin build
catkin build moveit_planners_ompl 
source ~/.bashrc
catkin build moveit_planners_ompl 
catkin build moveit_planners_ompl  -lompl
sudo apt-get purge ros-kinetic-ompl 
mkdir -p build/Release
make -j16
sudo make install
sudo make uninstall
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
roscd ompl/
sudo apt-get remove ros-kinetic-ompl 
catkin build
git clone https://github.com/ompl/ompl
cd ompl/
wget https://raw.githubusercontent.com/ros-gbp/ompl-release/debian/kinetic/xenial/ompl/package.xml
cd ..
catkin build
cd workspaces/move_it_ws/
catkin build
sudo apt-get install libccd-dev
sudo apt autoremove 
catkin clean
roscd ompl/
cd ..
catkin build
gedit ~/.bashrc
gedit ~/.bash_history 
git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git
cd ..
catkin build
git clone https://github.com/frankaemika/franka_ros.git
cd ..
catkin build
cd src/
git clone https://github.com/ros-controls/ros_control.git
cd ..
catkin build
cd src/
git clone https://github.com/frankaemika/libfranka.git
cd libfranka/
ls
mkdir build
cd build/
cmake ..
make -j4
make 
make -j4 ..
make .. -j4
make . -j4
cd ..
make
cd build/
sudo make install
sudo make install .
sudo make install ..
cd ..
sudo make install
make
cd build/
make
cmake ..
cmake .
cmake -DCMAKE_BUILD_TYPE=Release ..
htop
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka/
mkdir build
cd build/
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
cd ..
cd workspaces/
cd move_it_ws/
catkin build
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
git clone git clone --recursive https://github.com/frankaemika/libfranka
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka/
mkdir build
cd build/
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
roslaunch panda_moveit_config demo.launch rviz_tutorial:=true
cd workspaces/move_it_ws/
catkin build
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/workspaces/move_it_ws/src/libfranka/build
catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/workspaces/move_it_ws/src/libfranka/build
catkin build
htop
roslaunch panda_moveit_config demo.launch rviz_tutorial:=true
roslaunch moveit_tutorials move_group_interface_tutorial.launch
cd workspaces/move_it_ws/
catkin build
source ~/.bashrc
roslaunch moveit_tutorials move_group_interface_tutorial.launch
cd workspaces/move_it_ws/
catkin build
source ~/.bashrc
catkin build
source ~/.bashrc
roslaunch moveit_tutorials move_group_interface_tutorial.launch
roslaunch panda_moveit_config demo.launch
tmux
cd workspaces/move_it_ws/
catkin build
tmux
roslaunch panda_moveit_config demo.launch
roslaunch moveit_tutorials move_group_interface_tutorial.launch
catkin build
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
tmux
roscd realsense_camera/
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
htop
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
tmux
roslaunch panda_moveit_config demo.launch
rosrun moveit_tutorials move_group_python_interface_tutorial.py
catkin build
rosrun ros_ads ros_ads
roscd ros_ads
python ros_ads.py 
roscor
roscore
rosrun rqt_gui rqt_gui 
catkin build
catkin clean
catkin build
roscd ros_ads/
python v1
python v1.py 
ping 192.168.2.10
ifconfig
python v1.py 
ping 192.168.2.10.1.1
ping 169.254.123.205
ping 192.168.2.10
python v1.py 
roscore
rosrun ros_ads ros_ads.py
rosrun ros_ads ros_ads.py 
sudo /usr/bin/python -m pip install pylint
pip install --upgrade pip
sudo /usr/bin/python -m pip install pylint
rosrun rqt_gui rqt_gui 
rosrun ros_ads ros_ads.py 
roscore
roslaunch panda_moveit_config demo.launch
tmux
rosrun moveit_commander moveit_commander_cmdline.py
pip install --user pyassimp
pip install --user pyassimp=3.3
pip install --user pyassimp==3.3
rosrun moveit_commander moveit_commander_cmdline.py
clear
rosrun moveit_commander moveit_commander_cmdline.py
roslaunch panda_moveit_config demo.launch
roslaunch moveit_tutorials robot_model_and_robot_state_tutorial.launch
roslaunch moveit_tutorials planning_scene_tutorial.launch
roslaunch moveit_tutorials planning_scene_ros_api_tutorial.launch
roslaunch panda_moveit_config demo.launch
tmux
rostopic list
rostopic info /move_group/result 
rostopic echo /move_group/result 
roslaunch moveit_tutorials move_group_interface_tutorial.launch
roslaunch panda_moveit_config demo.launch
wget -r -p -U Mozilla //docs.ros.org/kinetic/api/moveit_tutorials/
wget -r -p -U Mozilla //docs.ros.org/kinetic/api/moveit_tutorials
wget -r -p -U Mozilla //docs.ros.org
wget -r -p -U Mozilla //docs.ros.org/kinetic
wget -r -p //docs.ros.org/kinetic/api/moveit_tutorials
wget -r -p http://docs.ros.org/kinetic/api/moveit_tutorials/
wget -r -p http://docs.ros.org/kinetic/api/moveit_tutorials/html/
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
source ~/.bashrc
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
source ~/.bashrc
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
source ~/.bashrc
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
source ~/.bashrc
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
source ~/.bashrc
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
source ~/.bashrc
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
source ~/.bashrc
roslaunch panda_moveit_config demo.launch
tmux
roslaunch moveit_tutorials motion_planning_pipeline_tutorial.launch
roslaunch panda_moveit_config demo.launch
roslaunch moveit_tutorials visualizing_collisions_tutorial.launch
tmux
roslaunch panda_moveit_config demo.launch
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc
rosrun moveit_tutorials pick_place_tutorial
roslaunch panda_moveit_config demo.launch
roslaunch panda_moveit_config demo.launch --debug
roslaunch panda_moveit_config demo.launch --verbose
roslaunch panda_moveit_config demo.launch --help
roslaunch panda_moveit_config demo.launch -v
rosrun moveit_tutorials pick_place_tutorial --help
roslaunch panda_moveit_config demo.launch 
rosrun moveit_tutorials pick_place_tutorial 
roscd franka_description/
catkin clean panda_moveit_config
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/move_it_ws/franka_description.urdf -urdf -x 0 -y 0 -z 1 -model panda
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/move_it_ws/franka_description.urdf -urdf -x 0 -y 0 -z 0 -model panda
roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true
roslaunch moveit_setup_assistant setup_assistant.launch
catkin build
roslaunch panda_moveit_config demo.launch 
catkin build
roslaunch moveit_tutorials obstacle_avoidance_demo.launch
rosrun rqt_graph rqt_graph 
roswtf
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
roslaunch moveit_tutorials obstacle_avoidance_demo.launch 
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
python dl_hw_1.py 
python3 dl_hw_1.py 
pip3 install matplotlib
pip3 upgrade
pip3 update
sudo -H pip3 install --upgrade pip
pip3 install matplotlib
pip3 install --user matplotlib
python3 dl_hw_1.py 
pip3 install --user _tkinter
sudo apt-get install python3-tk 
sudo apt-get install cmake g++ git ipython minizip python-dev python-h5py python-numpy python-scipy python-sympy qt4-dev-tools
sudo apt-get install python3-tk 
python3 dl_hw_1.py 
python3
python3
python3 dl_hw_1.py 
sudo apt-get install libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev libogg-dev libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev
git clone https://github.com/rdiankov/collada-dom.git
cd collada-dom && mkdir build && cd build
cmake ..
make -j8
sudo make install
sudo /usr/bin/python -m pip install pylint
make -j8
sudo make install
sudo apt-get update 
sudo apt-get install libcairo2-dev libjasper-dev libpoppler-glib-dev libsdl2-dev libtiff5-dev libxrandr-dev
git clone --branch OpenSceneGraph-3.4 https://github.com/openscenegraph/OpenSceneGraph.git
cd OpenSceneGraph && mkdir build && cd build
cmake .. -DDESIRED_QT_VERSION=4
make -j8
sudo make install
git clone --branch latest_stable https://github.com/rdiankov/openrave.git
sudo apt-get install libccd-dev
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
git checkout 0.5.0  # use FCL 0.5.0
mkdir build && cd build
cmake ..
make -j8
sudo make install
sudo ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
git clone --branch latest_stable https://github.com/rdiankov/openrave.git
git clone --branch latest_stable https://github.com/rdiankov/openrave.git,
git clone --branch latest_stable https://github.com/rdiankov/openrave.git
git checkout 9c79ea260e1c009b0a6f7c03ec34f59629ccbe2c
cd openrave-latest_stable/
git checkout 9c79ea260e1c009b0a6f7c03ec34f59629ccbe2c
mkdir build && cd build
cmake .. -DOSG_DIR=/usr/local/lib64/
make -j8
sudo make install
pip install --upgrade --user sympy==0.7.1
sudo apt remove python-mpmath
export MYROBOT_NAME="panda_arm"
rosrun xacro xacro --inorder -o "$MYROBOT_NAME".urdf "$MYROBOT_NAME".urdf.xacro
openrave.py --example graspplanning
sudo apt-get install cmake g++ git ipython minizip python-dev python-h5py python-numpy python-scipy qt4-dev-tools
sudo apt-get install libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev libogg-dev libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev
openrave
cd Desktop/
gcc detect_apple.cpp 
cd Desktop/
python detect_apple.py 
python3 detect_apple.py 
python detect_apple.py 
python detect_apple.py 
python color_threshold.py 
cd Desktop/
python circle_detection.py 
ls
chmod +x setupvmb.exe 
sudo chmod +x setupvmb.exe 
lsusb
vodafone-mobile-connect-card-driver-for-linux
ls /dev/
sudo usb_modeswitch -v 12d1 -p 1506 -J
ls /dev/sg1 
udevadm /dev/sg1 
udevadm 
lsusb
sudo usb_modeswitch -v 12d1 -p 1f0a -J
sudo nano /etc/usb_modeswitch.conf
udevadm info /dev/sg1
libusb /dev/sg1
libusb-config /dev/sg1
lsusb /dev/sg1
lsusb
lsusb /dev/sg1
lsusb -v /dev/sg1
lsusb -d /dev/sg1
lsusb -d
lsusb -v
ls /etc/udev/rules.d/
nano /etc/udev/rules.d/huawei_r281h.rules
sudo nano /etc/udev/rules.d/huawei_r281h.rules
sudo udevadm control -R
lsusb
reboot
lsusb
sudo nano /etc/udev/rules.d/huawei_r281h.rules
sudo udevadm control -R
sudo nano /etc/udev/rules.d/huawei_r281h.rules
lsusb
lsusb --help
lsusb -d 12d1:1f0a
lsusb -d 12d1:1f0a -v
roscd laser_proc/
ls
ping 192.168.2.10
ping 192.168.2.20
ifconfig
ping 192.168.2.10
ping 192.168.2.20
ping 192.168.2.10
rosrun rqt_gui rqt_gui 
rostopic info /cmd_vel 
rosrun ros_ads ros_ads.py 
roscore
lsb_release -a
sha256sum ubuntu-16.04.6-desktop-amd64.iso 
gedit 
python color_threshold.py 
python circle_detection.py 
python3 circle_detection.py 
python circle_detection.py 
roscd navigation/
ls
find turtlebot3_navigation
roscd turtlebot3_navigation
find turtlebot3_navigation
rospack find turtlebot3_navigation 
rospack find navigation
python detect_apple.py 
python color_threshold.py 
python detect_blue.py 
python detect_apple.py 
python detect_blue.py 
python detect_blue.py --camera 1
python detect_blue.py --camera 0
python detect_blue.py --camera 3
python detect_blue.py --camera 2
catkin_create_pkg agv_dog rospy move_base_msgs actionlib
cd ..
catkin build
cd workspaces/
cd air_agv_ws/
catkin build
rostopic list
rostopic info /camera/color/image_raw
rviz
roslaunch realsense2_camera rs_camera.launch 
python
htop
rosrun agv_dog agv_dog.py
rostopic list
rviz
rostopic list
rviz
roslaunch realsense2_camera rs_camera.launch 
rosrun agv_dog agv_dog.py
python color_threshold.py 
rosrun agv_dog agv_dog.py
roscore
python color_threshold.py 
sudo ufw status
roslaunch agv_simulation agv_navigation.launch 
roslaunch agv_simulation agv_slam_gmapping.launch 
roslaunch laser_scan_matcher agv_demo.launch 
roslaunch agv_simulation agv_rviz.launch 
rosrun rqt_tf_tree rqt_tf_tree 
rosrun tf tf_echo /map /base_link
rosrun agv_dog agv_dog.py 
rosrun tf tf_echo /map /base_link
rosrun agv_dog agv_dog.py 
rosrun agv_dog deneme.py 
rosrun agv_dog agv_dog.py 
rosrun rqt_tf_tree rqt_tf_tree 
roswtf
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch realsense2_camera rs_camera.launch 
tmux
roslaunch agv_simulation agv_gazebo.launch 
rostopic list
roslaunch agv_simulation agv_rviz.launch 
rviz
rostopic echo /scan/header -n1
sudo gedit /etc/hosts
ssh 192.168.0.100
ifconfig
ssh 192.168.0.100
sudo nano /etc/ssh/sshd_config
service sshd restart
ssh 192.168.0.100
ssh brkygkcn@192.168.0.100
ifconfig
ssh 192.168.0.100
ssh 192.168.0.100@brkygkcn
ssh 192.168.0.100

ssh brkygkcn@192.168.0.100
ssh
ssh localhost
rosrun rqt_reconfigure rqt_reconfigure 
rviz
ifconfig
roscd teleop_twist_joy 
roslaunch teleop_twist_joy teleop.launch 
rosrun rqt_reconfigure rqt_reconfigure 
ifconfig
ssh brkygkcn@192.168.0.100
tmux
rviz
roslaunch teleop_twist_joy teleop.launch 
ssh brkygkcn@192.168.0.100
ifconfig
ssh brkygkcn@192.168.0.100
roslaunch agv_simulation agv_gazebo.launch 
pip install --user pylint
python costmap_debug.py 
sudo /usr/bin/python -m pip install pylint
rostopic echo /move_base/local_costmap/costmap
rostopic echo /move_base/local_costmap/costmap -n1
rostopic info /move_base/local_costmap/costmap -n1
rostopic info /move_base/local_costmap/costmap 
rostopic echo /move_base/local_costmap/costmap -n1
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch laser_scan_matcher agv_demo.launch 
rostopic list
rosrun rqt_reconfigure rqt_reconfigure 
roslaunch agv_simulation agv_rviz.launch 
roslaunch agv_simulation agv_navigation.launch 
roslaunch agv_simulation agv_slam_
roslaunch agv_simulation agv_slam_gmapping.launch 
tmux
roslaunch teleop_twist_joy  teleop.launch 
rosrun rqt_gui rqt_gui 
roslaunch teleop_twist_joy teleop.launch 
rosrun rqt_reconfigure rqt_reconfigure 
rostopic echo /cmd_vel 
rosrun rqt_gui rqt_gui 
rviz
ssh brkygkcn@192.168.0.100
roslaunch agv_simulation agv_gazebo.launch 
rosrun agv_dog agv_dog.py 
roslaunch realsense2_camera rs_camera.launch 
roslaunch laser_scan_matcher agv_demo.launch 
roslaunch agv_simulation agv_slam_gmapping.launch 
roslaunch agv_simulation agv_navigation.launch 
roslaunch agv_simulation agv_rviz.launch 
tmux
rviz
ssh brkygkcn@192.168.0.100
roslaunch teleop_twist_joy teleop.launch 
rosrun agv_dog agv_dog.py 
rosrun agv_dog agv_dog.py 
roslaunch agv_simulation agv_slam_gmapping.launch 
roslaunch agv_simulation agv_navigation.launch 
roslaunch laser_scan_matcher agv_demo.launch 
roslaunch realsense2_camera rs_camera.launch 
rviz
roslaunch agv_simulation agv_gazebo.launch 
tmux
htop
rosrun agv_dog agv_dog.py 
rviz
roslaunch realsense2_camera rs_camera.launch 
rviz
roslaunch laser_scan_matcher agv_demo.launch 
roslaunch agv_simulation agv_slam_gmapping.launch 
roslaunch agv_simulation agv_navigation.launch 
roslaunch agv_simulation agv_rviz.launch 
rosrun agv_dog agv_dog.py 
roslaunch agv_simulation agv_gazebo.launch 
tmux
ssh brkygkcn@192.168.0.100
roslaunch agv_dog ball_tracker.launch 
clear
ssh brkygkcn@192.168.0.100
rviz
rostopic list
rostopic info /output_compressed 
rostopic hz /output_compressed 
rosrun image_view image_view image:=/output_compressed compressed
rosrun image_view image_view image:=/output compressed
rostopic list
rostopic info /output_compressed 
rosrun rqt_image_view rqt_image_view 
rostopic list
rosrun rqt_gui
rosrun rqt_gui rqt_gui 
ssh brkygkcn@192.168.0.100
rviz
rosrun rviz rviz 
ssh brkygkcn@192.168.0.100
rostopic echo /joy
rosnode list
rosrun ros_ads ros_ads.py 
rosnode kill /*
rosnode kill /move_base 
rosnode kill /laser_scan_matcher_node 
rosnode kill /joy_node 
rosnode kill /slam_gmapping 
rosnode kill /camera/realsense2_camera
rosnode list
rosnode kill /urg_node 
rosnode kill /camera/realsense2_camera_manager 
rosnode list
rostopic echo /cmd_vel 
rostopic echo /move_base/result 
rostopic echo /move_base/status 
rostopic list
rostopic echo /move_base/status 
tmux
rosbag record -a
rosnode list
rviz
ssh brkygkcn@192.168.0.100
rosrun rviz rviz
roslaunch agv_simulation agv_rviz.launch 
rosrun rviz rviz
rosbag record -a
rostopic list
rviz
rosbag record -a
rviz
rostopic list
rosparam set /use_sim_time true
rosbag play 2019-11-01-11-46-12.bag --clock
rosbag play 2019-11-01-11-46-12.bag
rosbag play 2019-11-01-11-46-12.bag --clock
rosrun rviz rviz 
roscore
rostopic list
rosnode list
rostopic echo /move_base/sta
rostopic echo /move_base/status 
ssh brkygkcn@192.168.0.100
rosrun rviz rviz 
sudo add-apt-repository ppa:maarten-baert/simplescreenrecorder
sudo apt update
sudo apt install simplescreenrecorder
simplescreenrecorder 
rviz
ssh brkygkcn@192.168.0.100
rviz
sudo apt-add-repository ppa:kdenlive/kdenlive-stable
sudo apt update
sudo apt-get install kdenlive
kdenlive
sudo nano /etc/udev/rules.d/huawei_r281h.rules
rosbag check realsense_lab_2019-11-05-14-38-17.bag 
roscore
rostopic list
rostopic echo /tf_static
rostopic echo /tf
rostopic list
rostopic info /voxel_cloud
rosparam get /use_sim_time
clear
rosbag record /camera/aligned_depth_to_color/image_raw /camera/color/image_raw /camera/color/camera_info /rtabmap/imu /tf_static
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
roscore
tmux
rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
tmux
htop
kdenlive
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
clear
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
clear
rostopic list
clear
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
clear
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
rosparam set use_sim_time true
rosbag play realsense_lab_2019-11-05-14-38-17.bag --clock
rosparam set use_sim_time true
rosbag play 2019-11-05-15-30-52.bag --clock
rosparam set use_sim_time true
rosparam get /use_sim_time
rosbag play 2019-11-05-15-46-38.bag --clock
rosparam set use_sim_time true
rosparam get /use_sim_time
rosbag play 2019-11-05-15-52-03.bag --clock
htop
grep -R "BRIEF"
rosrun rtabmap_ros rtabmap --params
rosrun rtabmap_ros rtabmap --params | grep -R "descripter"
rosrun rtabmap_ros rtabmap --params >> log_rtabmap_params.txt | grep -R "descripter" log_rtabmap_params.txt
rosrun rtabmap_ros rtabmap --params >> log_rtabmap_params.txt | grep -R "descriptor" log_rtabmap_params.txt
grep -R "SURF/Extended"
cd ..
grep -R "SURF/Extended"
grep -R "SURF"
grep -R "SURF "
ifconfig
rostopic echo /cmd_vel 
rviz
rosrun rviz rviz
ifconfig
ping 192.168.0.100
rosrun rqt_gui rqt_gui 
rviz
roslaunch realsense2_camera rs_camera.launch 
rv,z
rviz
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
roslaunch moveit_tutorials obstacle_avoidance_demo.launch
[A
roslaunch moveit_tutorials obstacle_avoidance_demo.launch
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
python3 Soru2.py 
python Soru2.py 
python3 Soru2.py 
clear
python3 Soru2.py 
clear
python3 Soru2.py 
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch --debug
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch -debug
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch --help
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch --screen
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch --launch-prefix="xterm -e python -m pdb"
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch -launch-prefix="xterm -e python -m pdb"
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch launch-prefix="xterm -e python -m pdb"
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch --launch-prefix="xterm -e python -m pdb"
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch 
sudo apt-get install valgrind*
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch 
lxc file push ~/Downloads/install_ros_kinetic.sh moveit/home/ubuntu/
sudo lxc file push ~/Downloads/install_ros_kinetic.sh moveit/home/ubuntu/
python3 Soru2.py 
pip3 install --user sklearn
python3 brky_nn.py 
htop
python3 brky_nn.py 
python3 soru1.py 
tmux
ls
ls --help
ls -a
sudo lxc file push ~/.tmux.conf moveit/home/ubuntu/
sudo lxc file push ~/workspaces/move_it_ws/src/ moveit/home/ubuntu/catkin_ws/src
sudo lxc file push -R ~/workspaces/move_it_ws/src/ moveit/home/ubuntu/catkin_ws/src
sudo lxc file push --help
sudo lxc file push -R ~/workspaces/move_it_ws/src.tar.gz moveit/home/ubuntu/catkin_ws/
sudo lxc file push ~/workspaces/move_it_ws/src.tar.gz moveit/home/ubuntu/catkin_ws/
htop
sudo lxc list
sudo lxc exec moveit -- sudo --login --user ubuntu
sudo lxc list
sudo lxc exec moveit -- sudo --login --user ubuntu
sudo lxc stop moveit
sudo lxc start moveit
sudo lxc list
sudo lxc exec moveit -- sudo --login --user ubuntu
lxc exec moveit -- sudo --login --user ubuntu
sudo lxc exec moveit -- sudo --login --user ubuntu
sudo lxc-attach -n moveit passwd
sudo apt-get update
sudo apt-get install zfsutils-linux
groups
sudo usermod --append --groups lxd brky
sudo apt-get install lxd lxd-client 
sudo usermod --append --groups lxd brky
groups
sudo lxd init
sudo lxc list
lxc launch ubuntu:16.04 moveit -s storage
sudo lxc launch ubuntu:16.04 moveit -s storage
sudo lxc launch ubuntu:16.04 moveit 
sudo lxc list
lxc exec moveit -- sudo --login --user ubuntu
sudo lxc exec moveit -- sudo --login --user ubuntu
sudo lxc list
sudo lxc exec moveit -- sudo --login --user ubuntu
sudo lxc exec moveit bash
sudo lxc exec moveit -- sudo --login --user ubuntu
sudo lxc exec moveit bash
sudo lxc exec moveit -- sh -c "exec >/dev/tty 2>/dev/tty </dev/tty && /usr/bin/screen -x"
sudo lxc exec moveit -- sh -c "exec >/dev/tty 2>/dev/tty </dev/tty && /usr/bin/tmux -x"
sudo lxc exec moveit -- sh -c "exec >/dev/tty 2>/dev/tty </dev/tty && /usr/bin/tmux "
tmux
htop
sudo lxc list
ifconfig
rviz
rosnode list
sudo nano /etc/hosts
rviz
rviz
sudo gedit ~/.bashrc
rosnode list
rosrun rqt_gui rqt_gui 
roscore
rosnode list
ping 10.29.245.1
clera
clear
ifconfig
sudo nano /etc/hosts
hostname
sudo gedit ~/.bashrc
roscore
ping 10.29.245.102
roscore
rosnode lit
rosnode list
ifconfig
rosnode list
ip addr show
rosnode list
rosocre
roscore
ping http
sudo gedit ~/.bashrc
source ~/.bashrc
roscore
ssh ubuntu@10.29.245.102
sudo ssh ubuntu@10.29.245.102
ssh-keygen 
sudo ssh ubuntu@10.29.245.102
sudo nano /etc/ssh/sshd_config
reload ssh
sudo reload ssh
sudo nano /etc/ssh/sshd_config
sudo reload ssh
sudo ssh ubuntu@10.29.245.102
ping 10.29.245.102
ssh -X ubuntu@10.29.245.102
ssh -p22 ubuntu@10.29.245.102
ssh-copy-id 10.29.245.102
ssh ubuntu@10.29.245.102
rosnode list
rviz
python3 soru1.py 
roslaunch panda_moveit_config demo_rviz.launch 
rosnode list
ssh ubuntu@10.29.245.102
tmux
python3 create_dataset.py 
python3 soru1.py 
python soru1.py 
python3 soru1.py 
sud0 ~/bashrc
sudo ~/.bashrc
sudo  gedit ~/.bashrc
rostopic list
roslaunch moveit_tutorials move_group_interface_tutorial.launch
rosrun moveit_tutorials move_group_python_interface_tutorial.py
roslaunch panda_moveit_config demo.launch rviz_tutorial:=true
roslaunch panda_moveit_config demo.launch
tmux
ifconfig
./brky_deneme.sh 
sudo lxc list
ifconfig
sudo mkdir www
cd www/
sudo nano index.html
ping 192.168.66.2
ssh air_server@192.168.66.2
ssh airlab@192.168.66.2
cd Desktop/
python -m http.server 
python -m http.server 80
python3 -m http.server 80
sudo python3 -m http.server 80
sudo python3 -m http.server a.html 80
sudo python3 -m http.server ./a.html 80
sudo python3 -m http.server .a.html 80
sudo python3 -m http.server /a.html 80
sudo python3 -m http.server ./a.html -p 80
cd asd/
sudo python3 -m http.server 80
cd /var/local/
ls
cd ..
ls
cd Desktop/
python3 -m http.server 80
python -m SimpleHTTPServer 
python -m SimpleHTTPServer 80
python -m SimpleHTTPServer 2759
cd /var/www/
ls
sudo mv index.html ~/Desktop/
ls
cd ..
sudo rm www/
sudo rm -rf www/
ls
ifconfig
cd Desktop/
python -m SimpleHTTPServer 2759
python -m SimpleHTTPServer 2755
ssh airlab@192.168.66.2
cd Desktop/
python -m SimpleHTTPServer 2755
cd Desktop/
./server_mainpage.sh 
crontab -e
htop
python -m SimpleHTTPServer ~/Desktop/ 2755
python -m SimpleHTTPServer 2755 ~/Desktop/
python -m SimpleHTTPServer 2755
grep -R "server_mainpage"
cd /etc/init.d/
ls
cd ~/.config/autostart/
ls
nano server_mainpage.sh.desktop 
python -m SimpleHTTPServer 2755
python -m SimpleHTTPServer 2752
ssh airlab@192.168.66.2
ssh airlab@192.168.66.2
cd ~/.config/autostart/
ls
nano server_mainpage.sh.desktop 
ping 192.168.66.2:2759
ping 192.168.66.2
python -m SimpleHTTPServer 27124
ssh airlab@192.168.66.2
ifconfig
ssh airlab@192.168.66.2
cd ~
cd ~dnsmasq/
ls
ssh airlab@192.168.66.2
nano .bashrc 
asda
asdfsadf
nano .bashrc 
rosrun moveit_tutorials move_group_python_interface_tutorial.py
roslaunch panda_moveit_config demo.launch
roslaunch panda_moveit_config demo.launch
tmux
rosrun moveit_tutorials move_group_python_interface_tutorial.py
roslaunch panda_moveit_config demo.launch
roslaunch moveit_tutorials robot_model_and_robot_state_tutorial.launch
ssh airlab@192.168.66.2
rosrun moveit_tutorials move_group_python_interface_tutorial.py
roslaunch panda_moveit_config demo.launch
tmux
roslaunch panda_moveit_config demo.launch
roslaunch moveit_tutorials planning_scene_ros_api_tutorial.launch
roslaunch moveit_tutorials robot_model_and_robot_state_tutorial.launch
roslaunch moveit_tutorials planning_scene_tutorial.launch
roslaunch panda_moveit_config demo.launch
tmuc
tmux
ifconfig
ssh brkygkcn@192.168.0.100
ping 192.168.0.100
tmux
ssh brkygkcn@192.168.0.100
rostopic list
rviz
roslaunch panda_moveit_config demo.launch
tmux
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
roslaunch panda_moveit_config demo.launch
ssh airlab@192.168.66.2
ifconfig
rotopic list
rostopic list
rostopic lisrt
rostopic list
rviz
sudo nano /etc/hosts
rviz
rostopic hz /camera/color/image_raw
ifconfig
rosrun rviz rviz
rostopic list
rostopic info /camera/color/image_raw
rostopic list
rosrun rqt_gui rqt_gui 
rviz
rosrun rviz rviz
rviz
ifconfig
ping 192.168.1.102
rosrun rqt_gui rqt_gui 
ifconfig
remmina
ssh brkygkcn@192.168.1.102
ifconfig
ssh brkygkcn@192.168.1.102
rostopic list
rviz
rosrun rviz rviz
ifconfig
rosrun rviz rviz
rvi
rviz
ifconfig
ssh airlab@192.168.66.2
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
roscd moveit_tutorials/
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
roslaunch panda_moveit_config demo.launch
ssh airlab@192.168.66.2
grep -r "RRTConnect"
grep -r "planner_configs"
grep -r "RRTConnect"
cd ..
grep -r "RRTConnect"
cd ..
grep -r "RRTConnect"
roslaunch moveit_tutorials motion_planning_pipeline_tutorial.launch
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
roslaunch panda_moveit_config demo.launch
tmux
roslaunch moveit_tutorials motion_planning_pipeline_tutorial.launch
tmux
ifconfig
ssh brkygkcn@192.168.1.102
ssh brkygkcn@192.168.0.102
ssh brkygkcn@192.168.0.100
rostopic list
rviz
ifconfig
ssh brkygkcn@192.168.0.102
ssh brkygkcn@192.168.0.101
sudo nano /etc/hosts
ifconfig
ssh brkygkcn@192.168.0.201
rviz
ssh brkygkcn@192.168.0.201
rviz
rostopic list
rosnode list
clear
rviz
roslaunch panda_moveit_config demo.launch
roslaunch moveit_tutorials visualizing_collisions_tutorial.launch
tmux
htop
ssh airlab@192.168.66.2
ifconfig
ssh airlab@192.168.66.2
grep -R "valgrind"
rosrun moveit_tutorials pick_place_tutorial
roslaunch panda_moveit_config demo.launch
roslaunch panda_moveit_config demo.launch --launch-prefix="valgrind"
roslaunch panda_moveit_config demo.launch -launch-prefix="valgrind"
roslaunch panda_moveit_config demo.launch --help
roslaunch panda_moveit_config demo.launch
roslaunch panda_moveit_config demo.launch --launch-prefix
roslaunch panda_moveit_config demo.launch --launch-prefix:="valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all"
roslaunch panda_moveit_config demo.launch
roslaunch panda_moveit_config demo.launch
tmux
ssh airlab@192.168.66.2
grep -R "kinemtaics_solver_attempts"
grep -R "kinematics_solver_attempts"
sudo ufw status
roslaunch moveit_tutorials planning_scene_tutorial.launch
rosrun moveit_tutorials pick_place_tutorial
rosservice lit
rosservice list
rosservice info /get_planning_scene 
rosservice call /get_planning_scene 
rosservice call /get_planning_scene  "components:
  components: 0" 
rosrun moveit_tutorials pick_place_tutorial
rosparam get /use_sim_time
rosrun moveit_tutorials pick_place_tutorial
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
rosparam set /use_sim_time true
roslaunch panda_moveit_config demo.launch
rosparam get /use_sim_time 
roslaunch panda_moveit_config demo.launch
roslaunch panda_moveit_config demo.launch
roscore
roslaunch panda_moveit_config demo.launch
roslaunch panda_moveit_config demo.launch > debug.txt
roslaunch panda_moveit_config demo.launch
roslaunch panda_moveit_config demo.launch > log.txt 2>&1
tmux
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
roslaunch panda_moveit_config demo.launch
roscore
roslaunch panda_moveit_config demo.launch
rosrun moveit_tutorials pick_place_tutorial
rosrun moveit_tutorials pick_place_tutorial --help
rosrun moveit_tutorials pick_place_tutorial --launch-prefix=""
rosrun moveit_tutorials pick_place_tutorial --launch-prefix="valgrind"
rosrun moveit_tutorials pick_place_tutorial --launch-prefix="valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all"
ssh airlab@192.168.66.2
rosrun moveit_tutorials pick_place_tutorial
cd src/
rm -rf panda_moveit_config
catkin clean panda_moveit_config
roslaunch moveit_setup_assistant setup_assistant.launch
grep -R "kinematics_solver_attempts"
sudo nano /etc/hosts
catkin clean
catkin_debug 
catkin_debug -DFranka_DIR:PATH=~/home/brky/workspaces/move_it_ws/src/libfranka/build
catkin_make -DCMAKE_BUILD_TYPE=Debug -DFranka_DIR:PATH=/path/to/libfranka/build
catkin clean
catkin_debug -DCMAKE_BUILD_TYPE=Debug -DFranka_DIR:PATH=/path/to/libfranka/build
catkin_debug -DCMAKE_BUILD_TYPE=Debug -DFranka_DIR:PATH=/home/brky/workspaces/move_it_ws/src/libfranka/build/
catkin_debug -DFranka_DIR:PATH=/home/brky/workspaces/move_it_ws/src/libfranka/build/
roslaunch moveit_tutorials obstacle_avoidance_demo.launch
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
ssh airlab@192.168.66.2
roslaunch panda_moveit_config demo.launch
roslaunch panda_moveit_config demo.launch
roslaunch moveit_setup_assistant setup_assistant.launch
rtcwake
/usr/sbin/rtcwake -m off -s 60
root /usr/sbin/rtcwake -m off -s 60
sudo apt install root-system-bin
brky /usr/sbin/rtcwake -m off -s 60
sudo apt install root-system-bin
clear
root /usr/sbin/rtcwake -m off -s 60
crontab -e
32 8 * * * root /usr/sbin/rtcwake -m off -s 60
crontab -e
crontab -l
crontab -e
[A
crontab -e
sudo apt purge root-system-bin
roscd octomap
sudo gedit rosconsole.config 
roslaunch moveit_tutorials obstacle_avoidance_demo.launch
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
htop
rosrun rqt_tf_tree rqt_tf_tree 
sudo gedit rosconsole.config 
rosclean
rosclean purge
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
rosclean purge
htop
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
gzsdf print AIRARM.urdf > air_arm.sdf
gz sdf print AIRARM.urdf > air_arm.sdf
gzsdf print AIRARM.urdf > air_arm.sdf
gz sdf -p AIRARM.urdf > air_arm.sdf
roscore
rosrun gazebo_ros gzserver 
rosrun gazebo_ros gzclient 
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
cd workspaces/move_it_ws/
rosdep install --from-paths ./src --ignore-src --rosdistro=kinetic
catkin_debug panda_moveit_config-kinetic-devel/
catkin_debug
roscore
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
gazebo
rosrun moveit_tutorials cylinder_segment 
rosrun 
rosrun moveit_tutorials cylinder_segment --help
rosrun moveit_tutorials cylinder_segment --launch-prefix=valgrind
source ~/.bashrc
rosrun moveit_tutorials cylinder_segment --launch-prefix=valgrind
rosrun moveit_tutorials cylinder_segment
source ~/.bashrc
rosrun moveit_tutorials cylinder_segment
source ~/.bashrc
rosrun moveit_tutorials cylinder_segment
source ~/.bashrc
rosrun moveit_tutorials cylinder_segment
source ~/.bashrc
rosrun moveit_tutorials cylinder_segment
source ~/.bashrc
rosrun moveit_tutorials cylinder_segment
source ~/.bashrc
rosrun moveit_tutorials cylinder_segment
source ~/.bashrc
rosrun moveit_tutorials cylinder_segment
source ~/.bashrc
rosrun moveit_tutorials cylinder_segment
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
source ~/.bashrc
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
roslaunch moveit_tutorials obstacle_avoidance_demo.launch
clear
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
rosconsole 
rosconsole list -h
rosconsole -h
rosconsole list
rosconsole get
clear
rosrun rqt_tf_tree rqt_tf_tree 
clear
tmux
cd workspaces/move_it_ws/
catkin_debug
roslaunch moveit_tutorials obstacle_avoidance_demo.launch
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
cd workspaces/move_it_ws/
catkin_debug
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
cd workspaces/move_it_ws/
catkin_debug
source ~/.bashrc
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
sudo apt install akregator
akregator
sudo apt purge akregator
sudo apt uninstall akregator
sudo apt remove akregator
flatpak install http://feedreader.xarbit.net/feedreader-repo/feedreader.flatpakref
sudo add-apt-repository ppa:quiterss/quiterss
sudo apt-get update
sudo apt-get install quiterss
quiterss 
sudo delete-apt-repository ppa:quiterss/quiterss
sudo add-apt-repository --remove ppa:quiterss/quiterss
sudo apt-get remove quiterss
sudo apt-get install cmake g++ git ipython minizip python-dev python-h5py python-numpy python-scipy qt4-dev-tools
sudo apt-get install libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev libogg-dev libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev
sudo apt-get install libcairo2-dev libjasper-dev libpoppler-glib-dev libsdl2-dev libtiff5-dev libxrandr-dev
cd OpenSceneGraph
ls
cd build/
ls
cmake .. -DDESIRED_QT_VERSION=4
make -j$(nproc)
sudo make install
pip install --upgrade --user sympy==0.7.1
sudo apt remove python-mpmath
sudo apt-get install ros-kinetic-openrave
sudo apt-get remove ros-kinetic-openrave
cmake -DODE_USE_MULTITHREAD=ON -DOSG_DIR=/usr/local/lib64/ ..
make -j$(nproc)
sudo make install
rosrun collada_urdf urdf_to_collada panda_arm_hand.urdf.xacro "$MYROBOT_NAME".dae
rosrun xacro xacro.py model.xacro > model.urdf
rosrun xacro xacro.py panda_arm_hand.urdf.xacro > model.urdf
rosrun collada_urdf urdf_to_collada model.urdf "$MYROBOT_NAME".dae
openrave-robot.py "$MYROBOT_NAME".dae --info links
openrave "$MYROBOT_NAME".dae
rosrun xacro xacro.py panda_arm.urdf.xacro > model.urdf
rosrun xacro xacro --inorder panda_arm.urdf.xacro > model.urdf
rosrun collada_urdf urdf_to_collada model.urdf "$MYROBOT_NAME".dae
openrave "$MYROBOT_NAME".dae
sudo apt-get install libsoqt4-dev
openrave-robot.py "$MYROBOT_NAME".dae --info links
openrave
export FREE_INDEX="1"
openrave-robot.py "$MYROBOT_NAME".dae --info links
openrave "$MYROBOT_NAME".dae
openrave-robot.py "$MYROBOT_NAME".dae --info links
openrave-robot.py "$MYROBOT_NAME".dae --info links --freeindex=1
openrave-robot.py "$MYROBOT_NAME".dae --info links --freeindex="1"
gz sdf print model.urdf > 1_dummy.sdf
gz sdf -p model.urdf > 1_dummy.sdf
gz sdf -p model.urdf > model.sdf
check_urdf model.urdf 
gzsdf print model.urdf > newname.sdf
gz sdf print model.urdf > newname.sdf
gz sdf --print model.urdf > newname.sdf
gz sdf -p ./model.urdf > ./model.sdf
gz sdf -p model.urdf > my_sdf.sdf
sudo nano /etc/hosts
openrave-robot.py "$MYROBOT_NAME".dae --info links
openrave "$MYROBOT_NAME".dae
roscd franka_description
rosversion 
rosversion -h
rosversion roscpp
rosrun collada_urdf urdf_to_collada AIRARM.urdf "$MYROBOT_NAME".dae
ssh airlab@192.168.66.2
ifconfig
ping 192.168.66.2
ifconfig
htop
rosrun xacro xacro --inorder -o "$MYROBOT_NAME".urdf "$MYROBOT_NAME".urdf.xacro
rosrun collada_urdf urdf_to_collada "$MYROBOT_NAME".urdf "$MYROBOT_NAME".dae
export IKFAST_PRECISION="5"
cp "$MYROBOT_NAME".dae "$MYROBOT_NAME".backup.dae  # create a backup of your full precision dae.
rosrun moveit_kinematics round_collada_numbers.py "$MYROBOT_NAME".dae "$MYROBOT_NAME".dae "$IKFAST_PRECISION"
openrave-robot.py "$MYROBOT_NAME".dae --info links
openrave "$MYROBOT_NAME".dae
openrave "$MYROBOT_NAME".dae --freeindex=1
openrave "$MYROBOT_NAME".dae --freeindex="1"
openrave panda_arm.dae 
pip install --upgrade --user sympy==0.7.1
sudo pip install --upgrade sympy==0.7.1
sudo pip install --upgrade pip
sudo pip install --upgrade sympy==0.7.1
pip install --user geos
pip install --user shapely
sudo pip install --upgrade sympy==0.7.1
sudo pip install  shapely
sudo pip install geos
sudo pip install --upgrade sympy==0.7.1
sudo pip install gitsome
sudo pip install --upgrade sympy==0.7.1
which pg_config
sudo which pg_config
sudo apt-get install libpq-dev python-dev
python --version
sudo pip install --upgrade sympy==0.7.1
sudo pip install -H --upgrade sympy==0.7.1
sudo pip -H install --upgrade sympy==0.7.1
sudo pip install --upgrade sympy==0.7.1 -H
sudo pip install --upgrade -H sympy==0.7.1 
sudo -H pip install --upgrade  sympy==0.7.1 
sudo -H pip3 install --upgrade  sympy==0.7.1 
cd /tmp/
ls
rm -r ./*
ls
sudo python setup.py install
sudo -H pip install --upgrade  sympy==0.7.1 
sudo -H pip install  sympy==0.7.1 
sudo -H pip install  sympy==0.7.3
sudo -H pip install  sympy==0.7.2
sudo -H pip install  sympy==0.7.3
sudo -H pip install  sympy==0.7.1
sudo apt-get update
sudo -H pip install  sympy==0.7.1
man sudo
sudo -H pip install  sympy==0.7.1
openrave-robot.py "$MYROBOT_NAME".dae --info links
openrave "$MYROBOT_NAME".dae
sudo make uninstall
htop
sudo make uninstall
sudo apt-get install cmake g++ git ipython minizip python-dev python-h5py python-numpy python-scipy qt4-dev-tools
sudo apt-get install libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev libogg-dev libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev
sudo apt-get install libcairo2-dev libjasper-dev libpoppler-glib-dev libsdl2-dev libtiff5-dev libxrandr-dev
git clone https://github.com/openscenegraph/OpenSceneGraph.git --branch OpenSceneGraph-3.4
cd OpenSceneGraph
mkdir build; cd build
cmake .. -DDESIRED_QT_VERSION=4
make -j16
sudo make install
python sympy --version
pip sympy | grep lxml
pip sympy 
python
sudo apt remove python-mpmath
apt remove python-mpmath
roscd moveit_kinematics/
git clone --branch latest_stable https://github.com/rdiankov/openrave.git
cd openrave && mkdir build && cd build
cmake -DODE_USE_MULTITHREAD=ON -DOSG_DIR=/usr/local/lib64/ ..
make -j16
sudo make install
rosrun moveit_kinematics round_collada_numbers.py "$MYROBOT_NAME".dae "$MYROBOT_NAME".dae "$IKFAST_PRECISION"
export IKFAST_PRECISION="5"
rosrun moveit_kinematics round_collada_numbers.py "$MYROBOT_NAME".dae "$MYROBOT_NAME".dae "$IKFAST_PRECISION"
htop
openrave-robot.py "$MYROBOT_NAME".dae --info links
openrave-robot.py "$MYROBOT_NAME".dae --info links
openrave "$MYROBOT_NAME".dae
export PLANNING_GROUP="panda_arm"
openrave-robot.py "$MYROBOT_NAME".dae --info links
export BASE_LINK="0"
export EEF_LINK="8"
export FREE_INDEX="1"
export IKFAST_OUTPUT_PATH=`pwd`/ikfast71_"$PLANNING_GROUP".cpp
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot="$MYROBOT_NAME".dae --iktype=transform6d --baselink="$BASE_LINK" --eelink="$EEF_LINK" --freeindex="$FREE_INDEX" --savefile="$IKFAST_OUTPUT_PATH"
htop
export MOVEIT_IK_PLUGIN_PKG="$MYROBOT_NAME"_ikfast_"$PLANNING_GROUP"_plugin
cd ~/workspaces/move_it_ws/src/
catkin_create_pkg "$MOVEIT_IK_PLUGIN_PKG"
cd ..
catkin_debug 
export PLANNING_GROUP="panda_arm"
cd src/franka_ros/franka_description/
ls
cd robots/
ls
export IKFAST_OUTPUT_PATH=`pwd`/ikfast61_"$PLANNING_GROUP".cpp
rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" "$IKFAST_OUTPUT_PATH"
export MYROBOT_NAME="panda_arm"
export PLANNING_GROUP="panda_arm"
export MOVEIT_IK_PLUGIN_PKG="$MYROBOT_NAME"_ikfast_"$PLANNING_GROUP"_plugin
export IKFAST_OUTPUT_PATH=`pwd`/ikfast71_"$PLANNING_GROUP".cpp
rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" "$IKFAST_OUTPUT_PATH"
rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" "panda_link0" "panda_link8" "$IKFAST_OUTPUT_PATH"
export MOVEIT_CONFIG_PKG="panda_moveit_config"
rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" "panda_link0" "panda_link8" "$IKFAST_OUTPUT_PATH"
rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" "panda_link0" "panda_link8" "$IKFAST_OUTPUT_PATH" --moveit_config_pkg="panda_moveit_config"
rosed "$MYROBOT_NAME"_moveit_config kinematics.yaml
rosed panda_moveit_config kinematics.yaml
sudo apt-get install vim
rosed panda_moveit_config kinematics.yaml
ssh airlab@192.168.66.2
catkin_debug 
grep -R "IKFastKinematicsPlugin"
roslaunch panda_moveit_config demo.launch 
roscd franka_description/
roslaunch panda_moveit_config demo.launch 
roslaunch panda_moveit_config demo.launch rviz_tutorial:=true
export FREE_INDEX="1"
export BASE_LINK="0"
export EEF_LINK="8"
export IKFAST_OUTPUT_PATH=`pwd`/ikfast1231_"$PLANNING_GROUP".cpp
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot="$MYROBOT_NAME".dae --iktype=transform6d --baselink="$BASE_LINK" --eelink="$EEF_LINK" --freeindex="$FREE_INDEX" --savefile="$IKFAST_OUTPUT_PATH"
sh update_ikfast_plugin.sh 
roslaunch panda_moveit_config demo.launch 
rostopic list
rostopic info /trajectory_execution_event 
rostopic info /move_group/display_planned_path 
rostopic info /move_group/goal
rostopic echo /move_group/goal
rostopic echo /move_group/display_planned_path 
ifconfig
ssh airlab@192.168.66.2
ping 192.168.66.1
catkin_debug 
catkin_debug trac_ik
roscd trac_ik_kinematics_plugin
sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin
cd workspaces/move_it_ws/src/
git clone git clone https://bitbucket.org/traclabs/trac_ik.git
git clone https://bitbucket.org/traclabs/trac_ik.git
roslaunch panda_moveit_config demo.launch 
grep -R "moveit_constraint_sampler_plugin"
grep -R "enforce_joint_model_state_space"
ssh airlab@192.168.66.2
htop
mkdir -p ~/chomp_ws/src
rmdir -p ~/chomp_ws/src
rm rf ~/chomp_ws/src
rm rf ~/chomp_ws/
cd ..
ls
cd workspaces/
mkdir -p ./chomp_ws/src
cd chomp_ws/
catkin build
cd src/
cd ..
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
cd /home/brky/workspaces/chomp_ws/build/moveit_planners_ompl; catkin build --get-env moveit_planners_ompl | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
git clone https://github.com/ros-planning/panda_moveit_config.git
catkin build
catkin clean
catkin build
roslaunch panda_moveit_config demo_chomp.launch
rosrun moveit_tutorials collision_scene_example.py cluttered
rosrun moveit_tutorials collision_scene_example.py sparse
rosrun moveit_tutorials collision_scene_example.py cluttered
roslaunch panda_moveit_config demo_chomp.launch
tmux
catkin build
roslaunch panda_moveit_config demo_chomp.launch
source ~/.bashrc
roslaunch panda_moveit_config demo_chomp.launch
rosrun moveit_tutorials collision_scene_example.py cluttered
roslaunch panda_moveit_config demo_chomp.launch
git clone https://github.com/ros-industrial/industrial_moveit.git
cd ..
catkin build
git clone https://github.com/ros-industrial/industrial_moveit.git
cd ..
catkin build
roslaunch panda_moveit_config demo.launch
tmux
roslaunch panda_moveit_config demo.launch
grep -R "brkygkcn"
roslaunch panda_moveit_config demo_stomp.launch 
roslaunch panda_moveit_config demo.launch 
cd workspaces/move_it_ws/
catkin debug
catkin_debug
roslaunch panda_moveit_config demo.launch 
roslaunch panda_moveit_config demo.launch
roslaunch panda_moveit_config demo_stomp.launch
tmux
file:///home/brky/workspaces/move_it_ws/src/panda_moveit_config-kinetic-devel/launch/demo_stomp.launch 
roslaunch panda_moveit_config demo_stomp.launch
catkin build
cd /home/brky/workspaces/chomp_ws/build/industrial_collision_detection; catkin build --get-env industrial_collision_detection | catkin env -si  /usr/bin/make cmake_check_build_system; cd -
catkin build
catkin_clean
catkin build
catkin build industrial_collision_detection
catkin build
catkin_clean
catkin build
cd /home/brky/workspaces/chomp_ws/build/industrial_collision_detection; catkin build --get-env industrial_collision_detection | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd workspaces/chomp_ws/
catkin_clean
clear
catkin build
catkin_clean
catkin build
catkin_clean
catkin build
roslaunch panda_moveit_config demo_stomp.launch
ssh airlab@192.168.66.2
wget https://launchpad.net/complexshutdown/trunk/0.5/+download/complexshutdown_0.5_all.deb -O complexshutdown.deb
sudo dpkg -i complexshutdown.deb 
sudo apt-get install -f
sudo apt-get remove complexshutdown
sudo gedit /etc/crontab 
ssh airlab@192.168.66.2
sudo gedit /etc/crontab 
sudo rtcwake -m no -l -t "$(date -d 'today 10:16:00' '+%s')"
sudo rtcwake -m no -t "$(date -d 'today 10:16:00' '+%s')"
sudo rtcwake -m no -l -t "$(date -d 'today 07:18:00' '+%s')"
sudo rtcwake -m no -l -t "$(date -d 'today 10:18:00' '+%s')"
sudo rtcwake -m no -l -s 1
sudo rtcwake -m no -s 1
sudo rtcwake -m no -l -t "$(date -d 'today 10:18:00' '+%s')"
sudo rtcwake -m no -l -t "$(date -d 'today 07:18:00' '+%s')"
sudo rtcwake -m no -l -t "$(date -d 'today 08:18:00' '+%s')"
sudo rtcwake -m no -l -t "$(date -d 'today 10:20:00' '+%s')"
sudo poweroff
sudo gedit /etc/crontab 
ssh airlab@192.168.66.2
ifconfig
catkin_make run_tests_stomp_moveit stomp_moveit_utest
cd ..
catkin_make run_tests_stomp_moveit stomp_moveit_utest
catkin_debug run_tests_stomp_moveit stomp_moveit_utest
ssh airlab@192.168.66.2
htop
cd workspaces/
mkdir -p /pat_ws/src
mkdir -p ./pat_ws/src
ls
cd pat_ws/src/
catkin clean
cd ..
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool update -t src
echo ${ROS_DISTRO}
wstool update -t srcrosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin_release 
cd src/
git clone -b kinetic-devel https://github.com/ros-planning/moveit_tutorials.git
git clone -b kinetic-devel https://github.com/ros-planning/moveit.git
git clone -b kinetic-devel https://github.com/ros-planning/moveit_msgs.git
git clone -b kinetic-devel https://github.com/ros-planning/geometric_shapes.git
git clone -b kinetic-devel https://github.com/ros-planning/moveit_visual_tools.git
git clone -b kinetic-devel https://github.com/PickNikRobotics/rviz_visual_tools.git
git clone -b kinetic-devel https://github.com/ompl/ompl.git
cd ompl/
ls
nano install-ompl-ubuntu.sh.in 
htop
mkdir -p build/Release
cmake ../..
cd build/Release
cmake ../..
make -j 4 update_bindings # if you want Python bindings
make -j 16
catkin build
catkin clean
catkin_release 
cd /home/brky/workspaces/pat_ws/build/moveit_planners_ompl; catkin build --get-env moveit_planners_ompl | catkin env -si  /usr/bin/cmake /home/brky/workspaces/pat_ws/src/moveit/moveit_planners/ompl --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/pat_ws/devel/.private/moveit_planners_ompl -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/pat_ws/install -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_TYPE=Release '-DCMAKE_C_FLAGS=-march=native -mtune=native -ftree-vectorize' '-DCMAKE_CXX_FLAGS=-march=native -mtune=native -ftree-vectorize'; cd -
cd src/
ls
cd ompl/
sh install-ompl-ubuntu.sh 
sudo sh install-ompl-ubuntu.sh 
cd ..
catkin_release 
nano ~/.bashrc 
git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git
cd ..
catkin_release 
roslaunch panda_moveit_config demo_chomp.launch
git clone -b kinetic-devel https://github.com/frankaemika/franka_ros.git
cd ..
catkin_release 
cd workspaces/pat_ws/src/
git clone -b kinetic-devel https://github.com/ros-controls/ros_control.git
roslaunch panda_moveit_config demo_chomp.launch
sudo ufw status
grep -R "ready"
grep -R "ready1"
grep -R "ready "
grep -R ""ready""
grep -R "\"ready\""
grep -R "fake_controller_joint_states"
roslaunch panda_moveit_config demo.launch 
roslaunch panda_moveit_config demo_chomp.launch
rosparam set /use_sim_time true
roslaunch panda_moveit_config demo_chomp.launch
roslaunch panda_moveit_config demo.launch
roslaunch panda_moveit_config demo_chomp.launch
roscore
rocore
tmux
roslaunch panda_moveit_config demo_chomp.launch
rosclean purge 
roslaunch panda_moveit_config demo_chomp.launch
ssh airlab@192.168.66.2
roslaunch panda_moveit_config demo_chomp.launch
rosclean purge
roslaunch panda_moveit_config demo_chomp.launch
roslaunch panda_moveit_config demo.launch 
htop
roslaunch panda_moveit_config demo.launch 
catkin_create_pkg trajectory_to_matlab
rostopic echo /joint_states
rostopic echo /move_group/status
rostopic echo /joint_states_desired
rostopic echo /move_group/result
cd workspaces/move_it_ws/
catkin_debug
cd ../catkin_ws/
catkin build
rosrun trajectory_to_matlab trajectory_to_matlab.py
gedit ~/.bashrc
rostopic echo /move_group/display_planned_path
rostopic info /move_group/display_planned_path
rostopic list
rostopic list
tmux
rosrun trajectory_to_matlab trajectory_to_matlab.py 
rostopic info /move_group/display_planned_path 
python
roslaunch panda_moveit_config demo.launch 
catkin build
roslaunch panda_moveit_config demo.launch 
cd workspaces/move_it_ws/
catkin_debug 
catkin build
roslaunch panda_moveit_config demo.launch 
tmux
rostopic echo /move_group/display_planned_path 
roslaunch panda_moveit_config demo_chomp.launch
htop
roslaunch panda_moveit_config demo_chomp.launch
roslaunch panda_moveit_config demo.launch 
roslaunch panda_moveit_config demo_chomp.launch
roslaunch panda_moveit_config demo.launch 
catkin_clean
catkin build
roslaunch panda_moveit_config demo_chomp.launch 
catkin_clean
catkin build
cd /home/brky/workspaces/pat_ws/build/moveit_core; catkin build --get-env moveit_core | catkin env -si  /usr/bin/cmake /home/brky/workspaces/pat_ws/src/moveit/moveit_core --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/pat_ws/devel/.private/moveit_core -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/pat_ws/install -DCMAKE_BUILD_TYPE=Release; cd -
catkin build
catkin_clean
catkin build
cd /home/brky/workspaces/pat_ws/build/geometric_shapes; catkin build --get-env geometric_shapes | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
catkin_clean
catkin build
roslaunch panda_moveit_config demo_chomp.launch 
catkin build
ssh airlab@192.168.66.2
rosrun trajectory_to_matlab trajectory_to_matlab.py 
sudo apt-get install ros-kinetic-chomp-motion-planner 
catkin_clean
catkin_release 
catkin_clean
catkin_debug 
roslaunch panda_moveit_config demo_chomp.launch 
roslaunch panda_moveit_config demo_chomp.launch 
sudo apt-get remove ros-kinetic-chomp-motion-planner 
catkin_debug 
sudo apt-get install ros-kinetic-moveit-msgs 
roslaunch panda_moveit_config demo_chomp.launch 
catkin_clean
catkin_debug 
catkin_clean
catkin_debug 
cd workspaces/pat_ws/
catkin_clean
catkin build
roslaunch panda_moveit_config demo_chomp.launch 
ssh airlab@192.168.66.2
catkin_clean
catkin_debug 
roslaunch panda_moveit_config demo_chomp.launch 
cd workspaces/pat_ws/
catkin_clean
catkin_debug 
ssh airlab@192.168.66.2
roslaunch panda_moveit_config demo_chomp.launch 
sudo gedit /etc/crontab 
cd workspaces/pat_ws/
catkin_clean
catkin_debug 
grep -R "Start state violates joint limits"
roslaunch panda_moveit_config demo_chomp.launch 
cd workspaces/pat_ws/
catkin_clean
catkin_debug 
roslaunch panda_moveit_config demo.launch 
roslaunch panda_moveit_config demo_chomp.launch 
cd workspaces/pat_ws/
catkin_clean
catkin_debug 
sudo apt-get install ros-kinetic-moveit
mkdir -p dummy_ws/src
cd dummy_ws/
catkin_debug 
roslaunch panda_moveit_config demo_chomp.launch 
catkin_debug 
cmd /home/brky/workspaces/pat_ws/devel/lib/moveit_ros_move_group/move_group --debug /joint_states:=/joint_states_desired
/home/brky/workspaces/pat_ws/devel/lib/moveit_ros_move_group/move_group --debug /joint_states:=/joint_states_desired
roscd moveit_planners_chomp/
sudo apt-get install ros-kinetic-moveit-resources 
roslaunch panda_moveit_config demo.launch
roscd chomp_motion_planner/
roslaunch panda_moveit_config demo.launch
export LC_NUMERIC="en_US.UTF-8
export LC_NUMERIC="en_US.UTF-8"
roslaunch panda_moveit_config demo.launch
catkin_clean 
catkin_debug 
roslaunch panda_moveit_config demo_chomp.launch
roslaunch panda_moveit_config demo.launch
rostest moveit_ros_planning_interface python_move_group.test
roslaunch panda_moveit_config demo.launch
catkin run_tests -iv
cd workspaces/dummy_ws/
catkin run_tests -iv
cd workspaces/dummy_ws/
catkin run_tests --no-deps --this -iv
roslaunch panda_moveit_config demo.launch db:=true
git clone -b 26compat https://github.com/mongodb/mongo-cxx-driver.git
sudo apt-get install scons
cd mongo-cxx-driver
sudo scons --prefix=/usr/local/ --full --use-system-boost --disable-warnings-as-errors
sudo scons install
sudo scons install --use-system-boost
sudo scons --prefix=/usr/local/ --full --use-system-boost --disable-warnings-as-errors
sudo scons install
sudo make install
sudo scons --prefix=/usr/local/ --full --use-system-boost --disable-warnings-as-errors install
sudo scons install
cd build/linux2/use-system-boost/
sudo scons install
cd ..
cd linux2/
sudo scons install
cd ..
sudo scons install
cd ..
sudo scons install
sudo scons install --use-system-boost
sudo scons install 
sudo scons install
sudo scons --prefix=/usr/local/ --full --use-system-boost --disable-warnings-as-errors
cd ..
wstool set -yu warehouse_ros_mongo --git https://github.com/ros-planning/warehouse_ros_mongo.git -v jade-devel
cd ..
wstool set -yu warehouse_ros_mongo --git https://github.com/ros-planning/warehouse_ros_mongo.git -v jade-devel
git clone -b jade-devel https://github.com/ros-planning/warehouse_ros_mongo.git
cd src/
wstool set -yu warehouse_ros --git https://github.com/ros-planning/warehouse_ros.git -v jade-devel
cd ..
catkin build
ssh airlab@192.168.66.2
htop
wstool set -yu warehouse_ros_mongo --git https://github.com/ros-planning/warehouse_ros_mongo.git -v jade-devel
wstool set -yu warehouse_ros_mongo --git https://github.com/ros-planning/warehouse_ros_mongo.git -v jade-devel ~/workspaces/dummy_ws/src/
wstool set -yu warehouse_ros_mongo --git https://github.com/ros-planning/warehouse_ros_mongo.git -v jade-devel ~/workspaces/dummy_ws
wstool set -yu warehouse_ros_mongo --git https://github.com/ros-planning/warehouse_ros_mongo.git -v jade-devel ~/workspaces/dummy_ws/
wstool --help
wstool cd ..
cd ..
wstool init /src
wstool init
wstool init ./src/
cd src/
wstool set -yu warehouse_ros_mongo --git https://github.com/ros-planning/warehouse_ros_mongo.git -v jade-devel
wstool set -yu warehouse_ros --git https://github.com/ros-planning/warehouse_ros.git -v jade-devel
catkin build
cd ..
catkin_clean 
wstool set -yu warehouse_ros_mongo --git https://github.com/ros-planning/warehouse_ros_mongo.git -v jade-devel
cd src/
wstool set -yu warehouse_ros_mongo --git https://github.com/ros-planning/warehouse_ros_mongo.git -v jade-devel
wstool set -yu warehouse_ros --git https://github.com/ros-planning/warehouse_ros.git -v jade-devel
catkin build
wstool set -yu warehouse_ros_mongo --git https://github.com/ros-planning/warehouse_ros_mongo.git -v jade-devel
wstool set -yu warehouse_ros --git https://github.com/ros-planning/warehouse_ros.git -v jade-devel
cd ..
catkin build
roslaunch panda_moveit_config demo.launch db:=true
roslaunch panda_moveit_config demo.launch db:=true
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
cd workspaces/
mkdir -p ~/ros_ws
cd ros
rmdir ~/ros_ws/
mkdir -p ~/workspaces/ros_ws
cd ros_ws/
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool update -t src
gedit moveit/moveit_planners/ompl/CMakeLists.txt
cd src/
gedit moveit/moveit_planners/ompl/CMakeLists.txt
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/master/baxter_simulator.rosinstall
wstool update
git clone -b kinetic-devel https://github.com/ompl/ompl.git
cd ompl
wget https://raw.githubusercontent.com/ros-gbp/ompl-release/debian/kinetic/xenial/ompl/package.xml
cd ~/workspaces/ros_ws
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
cd /home/brky/workspaces/ros_ws/build/baxter_sim_io; catkin build --get-env baxter_sim_io | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
catkin_clean 
catkin build
roslaunch airarm gazebo.launch 
roslaunch airarm display.launch 
roslaunch airarm gazebo.launch 
roslaunch airarm display.launch 
roslaunch airarm gazebo.launch 
catkin build
roslaunch panda_moveit_config demo.launch db:=true
sudo apt-get install ros-kinetic-mongodb-store
roslaunch panda_moveit_config demo.launch db:=true
mongorestore --version
mongodump version 2.2.0
mongodump --version
mongo --version
mongorestore --version
mongorestore  -uuser -ppasswd -d brky ./backup1/admin
mongorestore  -uuser -ppasswd -d brky 
mongorestore   
mongorestore  -uuser -ppasswd 
mongorestore  -uuser -ppasswd -d admin ./backup1/admin
ssh airlab@192.168.66.2
ifconfig
ssh airlab@192.168.66.2
roscore
rosbag play 2019-11-05-15-52-03.bag --clock
rviz
tmux
cd bags/airlab/realsense/
clear
rosbag play 2019-11-05-15-52-03.bag --clock
roscore
rosparam set /use_sim_time true
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false wait_imu_to_init:=true imu_topic:=/rtabmap/imu
rostopic list
rosbag play 2019-11-01-11-46-12.bag 
roscore
sudo rm -rf ./pat_ws/
sudo rm -rf ./dummy_ws/
sudo rm -rf ./chomp_ws/
mkdir -p /pat_ws/src
mkdir -p ./pat_ws/src
cd pat_ws/
wstool init src
ls
cd ..
mkdir -p ./chomp_ws/src
cd chomp_ws/
wstool init src
cd src/
catkin clean
cd ..
catkin clean
catkin build
cds
cd src/
catkin clean
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
sudo apt-get install python-wstool python-catkin-tools clang-format-3.9
echo ${ROS_DISTRO}
cd ..
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/${ROS_DISTRO}-devel/moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
cd src/
git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git
cd ..
catkin build
roslaunch panda_moveit_config demo_chomp.launch
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
catkin_solve_dependencies 
cd src/
catkin_solve_dependencies 
cd ..
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
sudo apt-get install ros-kinetic-urdfdom-py 
catkin build
catkin_clean 
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
sudo apt-get install curl
catkin clean
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
cd /home/brky/workspaces/chomp_ws/build/chomp_motion_planner; catkin build --get-env chomp_motion_planner | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
catkin_clean
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin_clean
catkin build
catkin_clean
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
catkin_clean
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
rosrun moveit_tutorials collision_scene_example.py cluttered
curl -sSL https://get.docker.com/ | sh
sudo usermod -aG docker brky
nvidia-smi 
./gui-docker -it --rm moveit/moveit:kinetic-source /bin/bash
./gui-docker -it --rm moveit/moveit:melodic-source /bin/bash
./gui-docker -it --rm moveit/moveit:kinetic-source /bin/bash
cd workspaces/chomp_ws/src/
ls
docker cp ./panda_moveit_config/ moveit:kinetic-source
docker cp ./panda_moveit_config/ moveit:kinetic-source/
docker cp ./panda_moveit_config/ moveit
docker cp ./panda_moveit_config/ ~/moveit_ws:/root/
./gui-docker -it --rm moveit/moveit:melodic-source /bin/bash
./gui-docker -it --rm moveit/moveit:kineticic-source /bin/bash
roscore
roscore & rosrun rviz rviz
docker ps –aq
docker ps 
docker ps -a
docker ps -aq
./gui-docker -c my_moveit_container -v ~/workspaces/chomp_ws/:/root/linked_moveit_ws
./gui-docker -v ~/workspaces/chomp_ws/:/root/linked_moveit_ws
docker run --help
ls
cd source/
docker build -t moveit/moveit:kinetic-source .
./gui-docker -v ~/workspaces/chomp_ws/:/root/linked_moveit_ws
cd ..
./gui-docker -v ~/workspaces/chomp_ws/:/root/linked_moveit_ws
./gui-docker -it --rm moveit/moveit:kineticic-source /bin/bash
./gui-docker 
./gui-docker source/
./gui-docker -ti
./gui-docker -c my_moveit_container -it moveit/moveit:kinetic-source /bin/bash
./gui-docker -it moveit/moveit:kinetic-source /bin/bash
./gui-docker -it 
./gui-docker -it gui-docker 
./gui-docker -it source/Dockerfile 
catkin_clean 
catkin build
catkin list --deps
catkin build
catkin_clean 
catkin build
catkin_clean 
catkin build
catkin_clean 
catkin build
gz sdf -p airarm.urdf > model.sdf
roslaunch moveit_setup_assistant setup_assistant.launch
roslaunch airarm display.launch 
roslaunch airarm gazebo.launch 
catkin_clean
catkin 
catkin_release 
roslaunch moveit_setup_assistant setup_assistant.launch
roslaunch airarm gazebo.launch 
catkin build
roslaunch airarm gazebo.launch 
roslaunch airarm display.launch 
groups
sudo gpasswd -a brky dialout
groups
roslaunch airarm gazebo.launch 
sudo apt update
mkdir -p ~/workspaces/qb_hand_ws/src
cd ~/workspaces/qb_hand_ws/src
cd ..
catkin build
cd src/
git clone --branch production-kinetic https://bitbucket.org/qbrobotics/qbdevice-ros.git
git clone --branch production-kinetic https://bitbucket.org/qbrobotics/qbhand-ros.git
cd ..
catkin build
cd src/
git clone -b kinetic-devel https://github.com/ros-controls/ros_control.git
cd ..
catkin build
roslaunch qb_device_bringup robot_description_bringup.launch 
catkin build
roslaunch airarm gazebo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch qb_device_driver communication_handler.launch
cd workspaces/qb_hand_ws/src/
git clone --branch production-kinetic https://bitbucket.org/qbrobotics/qbhand-ros.git
git clone --branch production-kinetic https://bitbucket.org/qbrobotics/qbchain-ros.git
cd ..
catkin build
roslaunch qb_hand_control control.launch 
git clone -b kinetic-devel https://github.com/ros-controls/ros_controllers.git
cd ..
catkin build
rostopic list
roslaunch qb_hand_control control.launch use_controller_gui:=true
roslaunch qb_hand_description display.launch 
rosrun rqt_gui rqt_gui 
rosservice list 
rostopic list
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=qbhand1
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:="qbhand1"
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1
rosrun rqt_gui rqt_gui 
rosrun rqt_graph rqt_graph 
rosservice call /communication_handler/activate_motors {"id: <actual_device_id>, max_repeats: 0"}
rosservice call /communication_handler/activate_motors {"id: 1, max_repeats: 0"}
rosservice call /communication_handler/activate_motors {"id:1, max_repeats: 0"}
rosservice call /communication_handler/activate_motors {"id: 1, max_repeats: 0"}
rosservice call /communication_handler/get_measurements "{id: 1, max_repeats: 0, get_positions: true, get_currents: false, get_distinct_packages: false}" 
rosservice call /communication_handler/get_info "id: 1
max_repeats: 0" 
rosservice list
rosrun rqt_gui
rosrun rqt_gui rqt_gui 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1
rosservice list
rosservice call /communication_handler/set_commands "id: 0
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [1]" 
grep -r "robot_package"
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=0
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_controller_gui:=true
rosservice call /communication_handler/activate_motors {"id: 1, max_repeats: 0"}
rosservice list
rosservice list
roslaunch qb_device_bringup robot_description_bringup.launch 
roslaunch moveit_setup_assistant setup_assistant.launch
catkin_clean 
catkin build 
roslaunch moveit_setup_assistant setup_assistant.launch
catkin_clean 
catkin build 
roslaunch moveit_setup_assistant setup_assistant.launch
catkin_clean 
catkin_release 
catkin_clean 
catkin_release 
roslaunch panda_moveit_config demo_chomp.launch
catkin_clean 
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/${ROS_DISTRO}-devel/moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
grep -R "Start state is empty"
roslaunch panda_moveit_config demo_chomp.launch
catkin_clean 
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch
catkin_clean 
catkin build
[A
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch
roslaunch panda_moveit_config demo_chomp.launch
catkin_clean 
catkin build
catkin_clean 
catkin build
roslaunch airarm gazebo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin_clean 
cd ..
catkin build
roslaunch airarm gazebo.launch 
cd workspaces/air_arm_ws/
catkin_clean 
catkin build
roslaunch airarm gazebo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch airarm gazebo.launch 
cd workspaces/air_arm_ws/
catkin_clean 
catkin build
roslaunch airarm gazebo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch airarm gazebo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin_clean 
catkin build
catkin_clean 
catkin build
catkin_clean 
catkin build
catkin_clean 
catkin build
roslaunch airarm gazebo.launch 
killall -9 gzserver
killall -9 gzclient
catkin_clean 
catkin build
roslaunch airarm gazebo.launch 
roslaunch airarm gazebo.launch 
roslaunch airarm gazebo.launch 
roslaunch airarm gazebo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build
catkin_clean 
roslaunch airarm gazebo.launch 
catkin_clean 
catkin build
catkin_clean 
catkin build
roslaunch airarm gazebo.launch 
cd ..
catkin build
catkin_clean 
roslaunch airarm gazebo.launch 
catkin_clean 
catkin build
roslaunch airarm gazebo.launch 
catkin_clean 
catkin build
catkin_clean 
cd workspaces/air_a
cd workspaces/air_arm_ws/
catkin build
catkin_clean 
catkin build
roslaunch airarm gazebo.launch 
cd workspaces/air_arm_ws/
catkin_clean 
catkin build
roslaunch airarm gazebo.launch 
catkin_clean 
roscore
rosrun gazebo_ros gzserver
rosrun gazebo_ros gzclient 
catkin build
grep -R "gazebo_ros"
roslaunch airarm gazebo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch airarm gazebo.launch 
catkin_clean 
catkin build
roslaunch airarm gazebo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build
catkin_clean 
catkin build
roslaunch airarm gazebo.launch 
catkin_clean 
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin_clean 
cd ..
catkin build
roslaunch airarm gazebo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch airarm gazebo.launch 
<?xml version="1.0" encoding="utf-8" ?>
<robot name="airarm">
<inertial>
<transmission name="trans_s2">
rosrun gazebo_ros spawn_model -file /home/brky/Desktop/air_arm_msa.urdf -urdf -x 0 -y 0 -z 1 -model airarm
roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build
roslaunch airarm_moveit_config demo.launch 
rosrun gazebo_ros spawn_model -file /home/brky/Desktop/air_arm_msa.urdf -urdf -x 0 -y 0 -z 1 -model airarm
roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true
roslaunch airarm gazebo.launch 
rosrun gazebo_ros spawn_model -file /home/brky/Desktop/air_arm_msa.urdf -urdf -x 0 -y 0 -z 1 -model airarm
roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true
roslaunch airarm gazebo.launch 
git clone -b kinetic-devel https://github.com/ros-simulation/gazebo_ros_pkgs.git
cd ..
catkin_clean 
catkin_release 
catkin_clean 
catkin build
cd /home/brky/workspaces/air_arm_ws/build/gazebo_ros; catkin build --get-env gazebo_ros | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
sudo apt-get install ros-kinetic-gazebo-ros-control 
catkin_clean 
catkin build
catkin_clean 
catkin build
cd /home/brky/workspaces/air_arm_ws/build/gazebo_plugins; catkin build --get-env gazebo_plugins | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
protoc --version
catkin build
roslaunch airarm gazebo.launch 
catkin build
htop
roslaunch moveit_setup_assistant setup_assistant.launch 
rostopic echo /move_group/monitored_planning_scene
rostopic echo /move_group/motion_plan_request
rostopic echo /planning_scene
rostopic echo /move_group/result
clear
catkin_create_pkg arm_planned_path_to_matlab rospy moveit_msgs
cd arm_planned_path_to_matlab/
mkdir scripts
rostopic echo /move_group/result
rostopic echo /joint_states 
rostopic echo /move_group/fake_controller_joint_states 
cd workspaces/air_arm_ws/src/
ls
cd ..
catkin build
grep -R "clustered"
grep -R "cluster"
grep -R "cluttered"
rosrun arm_planned_path_to_matlab arm_planned_path_to_matlab.py 
rostopic echo /move_group/display_planned_path
clear
rostopic echo /move_group/displaplanned_path
rostopic echo /move_group/display_planned_path 
rostopic infoo /move_group/display_planned_path 
rostopic info /move_group/display_planned_path 
roscore
rostopic list
clear
rostopic list
clear
rostopic list
rostopic list
rostopic echo /joint_states 
tmux
roslaunch airarm_movit_config de
roslaunch airarm_movit_config demo.launch 
ssh airlab@192.168.66.2
catkin build
catkin_release 
catkin_clean 
catkin build
rostopic info /move_group/display_planned_path 
rostopic list
rostopic echo /move_group/display_pa
rostopic echo /move_group/display_planned_path 
rostopic list
rostopic echo /move_group/result 
rostopic list
rostopic echo /joint_states_desired 
rostopic list
rostopic echo /trajectory_execution_event 
rostopic list
rostopic echo /execute_trajectory/result 
rostopic echo /move_group/result 
rostopic list
rostopic echo /move_group/fake_controller_joint_states 
rostopic list
rostopic echo /move_group/result 
rostopic info /move_group/result 
rostopic info /move_group/monitored_planning_scene 
rostopic echo /move_group/monitored_planning_scene 
rostopic echo /move_group/result 
source ~/.bashrc
rostopic echo /move_group/result 
roslaunch panda_moveit_config demo_chomp.launch
source ~/.bashrc
roslaunch panda_moveit_config demo_chomp.launch
source ~/.bashrc
roslaunch panda_moveit_config demo_chomp.launch
source ~/.bashrc
roslaunch panda_moveit_config demo_chomp.launch
source ~/.bashrc
roslaunch panda_moveit_config demo_chomp.launch
source ~/.bashrc
roslaunch panda_moveit_config demo_chomp.launch
source ~/.bashrc
roslaunch panda_moveit_config demo_chomp.launch
source ~/.bashrc
roslaunch panda_moveit_config demo_chomp.launch
source ~/.bashrc
roslaunch panda_moveit_config demo_chomp.launch
rostopic list
rostopic echo /joint_states
rostopic echo /move_group/display_planned_path 
source ~/.bashrc
rosrun arm_planned_path_to_matlab arm_planned_path_to_matlab.py 
source ~/.bashrc
rosrun arm_planned_path_to_matlab arm_planned_path_to_matlab.py 
rosrun arm_planned_path_to_matlab arm_planned_path_to_matlab.py chomp_planned_path
source ~/.bashrc
rosrun arm_planned_path_to_matlab arm_planned_path_to_matlab.py chomp_planned_path
grep -R "panda_rightfinger"
rosrun arm_planned_path_to_matlab arm_planned_path_to_matlab.py 
rostopic list
rostopic echo /move_group/display_
rostopic echo /move_group/display_planned_path 
rostopic echo /move_group/result 
roscd moveit_msgs/
roslaunch airarm_movit_config demo_chomp.launch 
rostopic echo /move_group/display_planned_path 
htop
catkin_clean 
catkin build 
roscd moveit_msgs/
rostopic echo /move_group/display_planned_path 
rosrun arm_planned_path_to_matlab arm_planned_path_to_matlab.py 
roslaunch airarm_movit_config demo_chomp.launch 
python Soru2.py 
python3 Soru2.py 
python user_interface.py 
python3 user_interface.py 
python
python3 user_interface.py 
python user_interface.py 
echo "# BLM5135_HW3" >> README.md
git init
git add README.md 
git commit -m "init"
git remote add origin git@github.com:brkygokcen/BLM5135_HW3.git
git push -u origin master
git add *
git commit -m "user_interface done"
git push -u origin master
roslaunch airarm_movit_config demo_chomp.launch 
python main.py 
git add *
git commit -m "main added"
git push -u origin master
python main.py 
git add *
git commit -m "NN.set_params(UI) added"
git push -u origin master
git clone git@github.com:brkygokcen/BLM5135_HW3.git
ls
catkin build
roslaunch 191217_0946_TestLink gazebo.launch 
roslaunch 191217_0946_TestLink display.launch 
roslaunch 191217_0946_TestLink gazebo.launch 
python main.py 
python3
python
python3 main.py 
pip3 install --user sklearn
python main.py 
python3 main.py 
clear
python3 main.py 
Configuration:
input_layer_neuron_size: 2
output_layer_neuron_size: 1
number_of_hidden_layer: 3
hidden_layers_neuron_size: [3 4 5]
hidden_layer_activation_f: Sigmoid
last_layer_activation_f: Sigmoid
loss_function: MSE
epoch: 1001
eta: 0.01
bach_size: 1
python3 main.py 
python3 Soru2.py 
sudo dpkg -i virtualbox-6.1_6.1.0-135406~Ubuntu~xenial_amd64.deb 
nvidia-smi 
ls /dev
git clone git@github.com:brkygokcen/BLM5135_Project.git
git add *
git commit -m "batch processing added"
git push -u origin master
python 
python main.py 
python3 main.py 
clear
python3 main.py 
python3 softmax.py 
python3 main.py 
htop
rosnode list
rostopic list
rostopic echo /move_group/ompl/parameter_descriptions 
roslaunch panda_moveit_config demo.launch 
roslaunch airarm_movit_config demo.launch 
ifconfig
dpkg -i texstudio_2.12.18-1+5.1_i386.deb 
sudo dpkg -i texstudio_2.12.18-1+5.1_i386.deb 
sudo apt install texmaker
texmaker 
ifconfig
groups
sudo apt update
mkdir -p qb_hand_ws/src
cd qb_hand_ws/
catkin build
roslaunch qb_device_driver communication_handler.launch
rosservice call /communication_handler/activate_motors "id: 0
max_repeats: 0" 
rosservice call /communication_handler/activate_motors "id: 1
max_repeats: 0" 
roslaunch qb_hand_control control.launch use_controller_gui:=true
roslaunch qb_hand_control control.launch use_controller_gui:=true
roslaunch qb_hand_control control.launch use_controller_gui:=true standalone:=true activate_on_initialization:=true
tmuz
tmux
roslaunch qb_device_driver communication_handler.launch
ssh airlab@192.168.66.2
history
git init
git remote add origin git@192.168.66.3:brkygkcn/qb_hand.git
git add .
git commit -m "Initial commit"
git push -u origin master
ssh airlab@192.168.66.2
rosservice call /communication_handler/activate_motors "id: 1 
max_repeats: 0"
roslaunch qb_hand_control control.launch use_controller_gui:=true standalone:=true activate_on_initialization:=true
roslaunch qb_device_driver communication_handler.launch
roslaunch qb_device_driver communication_handler.launch
tmux
rosservice call /communication_handler/activate_motors "id: 1 
max_repeats: 0"
roslaunch qb_hand_control control.launch use_controller_gui:=true standalone:=true activate_on_initialization:=true
roslaunch qb_device_driver communication_handler.launch
rosservice call /communication_handler/activate_motors "id: 0
max_repeats: 0"
rosservice call /communication_handler/activate_motors "id: 1
max_repeats: 0"
roslaunch qb_hand_control control.launch use_controller_gui:=true
ssh airlab@192.168.66.2
git clone https://github.com/ros-interactive-manipulation/pr2_object_manipulation.git
cd ..
catkin build
catkin_release 
catkin_clean 
catkin build
catkin_clean 
cd /usr/local/include/google/
ls
sudo rm -rf protobuf/
ls
sudo mv protobuf-2.6.1 /usr/local/include/google/
ls
cd /usr/local/include/google/
ls
cd protobuf-2.6.1/
ls
./autogen.sh 
sudo ./autogen.sh 
sudo apt-get install autoconf
sudo ./autogen.sh 
sudo ./configure 
make
sudo make
sudo make check
sudo make install
sudo ldconfig
catkin_clean 
catkin build
roslaunch pr2_gazebo pr2_table_object.launch 
roslaunch pr2_gazebo pr2_table_object.launch 
rosrun pr2_tuckarm tuck_arms.py -l t -r u
mkfir -p carl_ws/src
mkdir -p carl_ws/src
cd carl_ws/
cd src/
cd ..
catkin build
cd src/
git clone https://github.com/GT-RAIL/carl_moveit.git
cd ..
catkin build
cd src/
git clone https://github.com/cwru-robotics/external_packages.git
git clone https://github.com/GT-RAIL/rail_manipulation_msgs.git
cd ..
catkin build
rosdep install libphidgets
cd src/
git clone https://github.com/ipa320/cob_extern.git
cd cob_extern/libphidgets/
ls
roscd libphidgets
mkdir build
cd build/
make
make .
cmake .
cd ..
cmake .
sudo make install
cd workspaces/carl_ws/
catkin build
cd src/
git clone https://github.com/sniekum/ar_track_alvar_msgs.git
cd ..
catkin build
cd src/
git clone https://github.com/RIVeR-Lab/wpi_jaco.git
cd ..
catkin build
cd /home/brky/workspaces/carl_ws/build/libphidgets; catkin build --get-env libphidgets | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/carl_ws/build/carl_tools; catkin build --get-env carl_tools | catkin env -si  /usr/bin/cmake /home/brky/workspaces/carl_ws/src/carl_bot/carl_tools --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/carl_ws/devel/.private/carl_tools -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/carl_ws/install; cd -
catkin build
sudo apt-get update
sudo apt-get install libphidgets-dev
wget -qO- http://www.phidgets.com/gpgkey/pubring.gpg | apt-key add -
sudo wget -qO- http://www.phidgets.com/gpgkey/pubring.gpg | apt-key add -
sudo su
cd workspaces/carl_ws/
catkin build
sudo rm -rf cob_extern/
rosdep install libphidgets
rosmake libphidgets
cd workspaces/carl_ws/src/
git clone https://github.com/KTH-RAS/ras_install.git
cd ras_install/scripts/
ls
sudo chmod +x install_phidgets.sh 
sudo ./install_phidgets.sh 
grep -R "brkygkcn"
grep -R "add_compile_options(-std=c++11)"
cd ..
catkin build
sudo apt-get install libphidget22-dev 
catkin build
grep -R "add_compile"
cd workspaces/carl_ws/
catkin build
sudo apt-get install ros-kinetic-libphidgets
catkin build
catkin_clean 
catkin build
catkin_clean 
catkin build
cd src/
git clone https://github.com/GT-RAIL/carl_moveit.git
cd ..
catkin build
cd /home/brky/workspaces/carl_ws/build/carl_moveit; catkin build --get-env carl_moveit | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
roslaunch manipulation_worlds pr2_table_object.launch
mkdir -p pr2_moveit/src
cd pr2_moveit/
catkin build
cd src/
git clone https://github.com/ros-planning/moveit_pr2.git
cd ..
catkin build
catkin_clean 
catkin build
mkdir build
cd build/
sudo make .
sudo make ..
sudo make 
sudo cmake 
sudo cmake .
sudo cmake ..
sudo make install
git clone https://github.com/wg-perception/object_recognition_ros.git
cd ..
catkin build
sudo apt-get install ros-kinetic-ecto*
sudo apt-get install ros-kinetic-ecto
catkin build
sudo apt-get install ros-kinetic-ecto-ros 
catkin build
cd src/
git clone https://github.com/wg-perception/object_recognition_core.git
cd ..
catkin build
source ~/.bashrc 
roslaunch pr2_moveit_config tabletop_object_recognition.launch 
cd src/
git clone https://github.com/ros-planning/moveit_advanced.git
cd ..
catkin build
cd src/
git clone https://github.com/ros/console_bridge.git
cd ..
catkin build
cd src/
git clone https://github.com/ros/console_bridge
cd console_bridge/
ls
mkdir build
cd build/
cmkae
cmake .
cmake ..
sudo make install
cd ..
catkin build
source ~/.bashrc 
catkin build
mkdir build
cd src/
mkdir build
cd build/
source ~/.bashrc
cd .
cd ..
catkin build
cd src/
git clone https://github.com/ros/rosconsole_bridge.git
cd ..
catkin build
cd src/
git clone https://github.com/ros/console_bridge.git
cd .
cd ..
catkin clean 
catkin build
roslaunch pr2_moveit_config tabletop_object_recognition.launch 
cd /home/brky/workspaces/pr2_moveit_ws/build/mesh_core; catkin build --get-env mesh_core | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin_clean 
catkin build
roslaunch pr2_moveit_config tabletop_object_recognition.launch 
sudo apt-get install ros-kinetic-ecto-image-pipeline 
source ~/.bashrc
roslaunch pr2_moveit_config tabletop_object_recognition.launch 
cd workspaces/pr2_moveit_ws/src/
git clonehttps://github.com/wg-perception/tabletop.git
git clone https://github.com/wg-perception/tabletop.git
cd ..
catkin build
rosrun gazebo_ros gzclient 
catkin_clean 
catkin build
roslaunch pr2_moveit_config tabletop_object_recognition.launch 
rostopic list
roslaunch pr2_moveit_config demo.launch 
mkdir -p ./mastering_ros_ws/src
cd mastering_ros_ws/
catkin build
cd src/
git clone https://github.com/jocacace/seven_dof_arm_test.git
cd ..
catkin build
cd src/
git clone https://github.com/jocacace/seven_dof_arm_config.git
cd ..
catkin build
cd src/
git clone https://github.com/jocacace/mastering_ros_robot_description_pkg.git
cd ..
catkin build
svn co https://code.ros.org/svn/wg-ros-pkg/stacks/wg_robots_gazebo/trunk/pr2_doors_gazebo_demo/
grep -R "pr2_doors_gazebo_demo"
roslaunch seven_dof_arm_config demo.launch 
source ~/.bashrc
roslaunch seven_dof_arm_config demo.launch 
roslaunch seven_dof_arm_config demo.launch
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping.launch
source ~/.bashrc
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping.launch 
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping.launch
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping_gpd.launch 
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping.launch 
roslaunch seven_dof_arm_gazebo seven_dof_arm_world_grasping.launch 
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping.launch
roslaunch seven_dof_arm_gazebo seven_dof_arm_world.launch 
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping.launch
rosrun seven_dof_arm_test test_random_node 
rosrun seven_dof_arm_test test_custom_node 
cd workspaces/mastering_ros_ws/
catkin build
rosrun seven_dof_arm_test test_custom_node 
catkin clrna
catkin clean 
catkin build
source ~/.bashrc
rosrun seven_dof_arm_test test_custom_node 
catkin build
source ~/.bashrc
rosrun seven_dof_arm_test test_custom_node 
catkin build
source ~/.bashrc
rosrun seven_dof_arm_test test_custom_node 
rosrun seven_dof_arm_test add_collision_object
rosrun seven_dof_arm_test remove_collision_objct 
rosrun seven_dof_arm_test attach_detach_object 
rosrun seven_dof_arm_test check_collision
cd src/
git clone https://github.com/jocacace/seven_dof_arm_gazebo.git
cd ..
catkin build
source ~/.bashrc
source ~/.bashrc
roslaunch seven_dof_arm_config demo.launch 
source ~/.bashrc
roslaunch seven_dof_arm_config demo.launch 
tmux
rosrun rqt_graph rqt_graph 
rostopic list
rviz
rosrun rviz rviz -f base_link
roslaunch seven_dof_arm_config moveit_planning_execution.launch
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping.launch
cd workspaces/mastering_ros_ws/
catkin build
tmux
roslaunch seven_dof_arm_config moveit_planning_execution.launch
rosrun seven_dof_arm_test pick_place
source ~/.bashrc
rosrun seven_dof_arm_test pick_place
sudo apt-get install ros-kinetic-moveit-commander 
cd workspaces/mastering_ros_ws/
catkin clean
catkin build
source ~/.bashrc
rosrun seven_dof_arm_test pick_place
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping.launch
roslaunch seven_dof_arm_config demo.launch
git clone https://github.com/atenpas/gpd.git
sudo apt-get install libgflags-dev libprotobuf-dev liblmdb-dev
sudo apt-get install libgflags-dev libprotobuf-dev liblmdb-dev libleveldb-dev libsnappy-dev libatlas3-base
git clone https://github.com/BVLC/caffe.git
cd caffe/
mkdir build
cd build/
nvidia-smi 
cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr ..
make
make -j16
sudo make install
cd workspaces/mastering_ros_ws/
catkin build
catkin build gdb
catkin build gpd
git clone https://github.com/atenpas/gpd_ros.git
git clone https://github.com/atenpas/gpd2_ros
./configure 
make -j16
sudo make install
git clone https://github.com/atenpas/gpd2
git clone https://github.com/atenpas/gpd
cd gps
cd gpd
mkdir build
cd build/
cmake ..
cmake .
cmake ..
cmake
cmake --version
sudo apt remove cmake
cmake --version
cmake ..
make -j16
make -j
sudo make install
make . -j
make .. -j
make  -j
cd 
cd workspaces/mastering_ros_ws/src/gpd
make
make .
cd build/
cmake
cmake ..
sudo apt-get update
sudo apt-get install git build-essential linux-libc-dev
sudo apt-get install cmake cmake-gui 
sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev
sudo apt-get install mpi-default-dev openmpi-bin openmpi-common  
sudo apt-get install libflann1.8 libflann-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libvtk5.10-qt4 libvtk5.10 libvtk5-dev
sudo dpkg -i PCL-1.8.0-Linux.deb
sudo dpkg -r PCL-1.8.0-Linux.deb

sudo dpkg -remove PCL-1.8.0-Linux.deb
sudo dpkg --remove PCL-1.8.0-Linux.deb
sudo dpkg --remove PCL-1.8.0-Linux
sudo dpkg --remove PCL-1.8.0
sudo dpkg --remove PCL
mkdir build
cd build/
cmake ..
make .
make ..
cd ..
sudo dpkg -i pcl-1.9.0-darwin/
sudo dpkg -i pcl-pcl-1.9.0/
mkdir build
cd build/
cmake ..
make
make -j16
sudo make install
cmake ..
make -j16
sudo make install
cd ..
catkin build
./INSTALL 
mkdir build
cd build/
cmake ..
make -j8
sudo make install
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 3.4.4
cd ..
git clone https://github.com/opencv/opencv_contrib.git
git checkout 3.4.4
cd opencv_contrib/
git checkout 3.4.4
cd ..
cd opencv
mkdir build
cd build/
cmake -D CMAKE_BUILD_TYPE=RELEASE             -D CMAKE_INSTALL_PREFIX=$cwd/installation/OpenCV-"$cvVersion"             -D INSTALL_C_EXAMPLES=ON             -D INSTALL_PYTHON_EXAMPLES=ON             -D WITH_TBB=ON             -D WITH_V4L=ON             -D OPENCV_PYTHON3_INSTALL_PATH=$cwd/OpenCV-$cvVersion-py3/lib/python3.5/site-packages         -D WITH_QT=ON         -D WITH_OPENGL=ON         -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules         -D BUILD_EXAMPLES=ON ..
make -j16
cd .
cd ..
cvVersion="3.4.4"
rm -rf opencv/build
rm -rf opencv_contrib/build
mkdir installation
mkdir installation/OpenCV-"$cvVersion"
cwd=$(pwd)
sudo apt -y update
sudo apt -y upgrade
sudo apt -y remove x264 libx264-dev
sudo apt -y install build-essential checkinstall cmake pkg-config yasm
sudo apt -y install git gfortran
sudo apt -y install libjpeg8-dev libjasper-dev libpng12-dev
sudo apt -y install libtiff5-dev
sudo apt -y install libtiff-dev
sudo apt -y install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
sudo apt -y install libxine2-dev libv4l-dev
sudo apt -y install libprotobuf-dev protobuf-compiler
cd opencv
git checkout $cvVersion
cd ..
cd opencv_contrib/
git checkout $cvVersion
cd ..
cmake -D CMAKE_BUILD_TYPE=RELEASE             -D CMAKE_INSTALL_PREFIX=$cwd/installation/OpenCV-"$cvVersion"             -D INSTALL_C_EXAMPLES=ON             -D INSTALL_PYTHON_EXAMPLES=ON             -D WITH_TBB=ON             -D WITH_V4L=ON             -D OPENCV_PYTHON3_INSTALL_PATH=$cwd/OpenCV-$cvVersion-py3/lib/python3.5/site-packages         -D WITH_QT=ON         -D WITH_OPENGL=ON         -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
cd open
cd opencv
mkdir build
cd build/
cmake -D CMAKE_BUILD_TYPE=RELEASE             -D CMAKE_INSTALL_PREFIX=$cwd/installation/OpenCV-"$cvVersion"             -D INSTALL_C_EXAMPLES=ON             -D INSTALL_PYTHON_EXAMPLES=ON             -D WITH_TBB=ON             -D WITH_V4L=ON             -D OPENCV_PYTHON3_INSTALL_PATH=$cwd/OpenCV-$cvVersion-py3/lib/python3.5/site-packages         -D WITH_QT=ON         -D WITH_OPENGL=ON         -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules         -D BUILD_EXAMPLES=ON ..
make -j16
make
cd workspaces/mastering_ros_ws/
catkin build
make -j16
make 
grep -R "DBUILD_opencv_rgbd"
rm -rf build/
mkdir build
cd build/
cmake -D CMAKE_BUILD_TYPE=RELEASE             -D CMAKE_INSTALL_PREFIX=$cwd/installation/OpenCV-"$cvVersion"             -D INSTALL_C_EXAMPLES=ON             -D INSTALL_PYTHON_EXAMPLES=ON             -D WITH_TBB=ON             -D WITH_V4L=ON             -D OPENCV_PYTHON3_INSTALL_PATH=$cwd/OpenCV-$cvVersion-py3/lib/python3.5/site-packages         -D WITH_QT=ON         -D WITH_OPENGL=ON         -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules         -D BUILD_EXAMPLES=ON ..
cmake -D CMAKE_BUILD_TYPE=RELEASE             -D CMAKE_INSTALL_PREFIX=$cwd/installation/OpenCV-"$cvVersion"             -D INSTALL_C_EXAMPLES=ON             -D INSTALL_PYTHON_EXAMPLES=ON             -D WITH_TBB=ON             -D WITH_V4L=ON             -D OPENCV_PYTHON3_INSTALL_PATH=$cwd/OpenCV-$cvVersion-py3/lib/python3.5/site-packages         -D WITH_QT=ON         -D WITH_OPENGL=ON         -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules         -D BUILD_EXAMPLES=ON ..        -DWITH_EIGEN=OFF
make 
sudo make install
mkdir build && cd build
cmake ..
make -j
sudo make install
roslaunch gpd_ros ur5.launch 
sudo make uninstall
sudo make --uninstall
sudo make -n install
sudo make --uninstall
sudo rm libgpd.so 
sudo rm -rf gpd/
catkin build
cd ..
catkin_clean 
catkin build
git clone https://github.com/atenpas/gpg.git
cd gpg
mkdir build && cd build
cmake ..
make
sudo make install 
cd ..
cd src/
git clone https://github.com/atenpas/gpd.git -b forward
sudo rm -rf gpd
git clone https://github.com/atenpas/gpd.git -b forward
cd ..
catkin build
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping_gpd.launch
cd workspaces/mastering_ros_ws/
ls
cd src/
ls
rm -rf gpd2_ros/
git clone https://github.com/atenpas/gpd.git -b forward
cd ..
catkin build
cd /home/brky/workspaces/mastering_ros_ws/build/gpd; catkin build --get-env gpd | catkin env -si  /usr/local/bin/cmake /home/brky/workspaces/mastering_ros_ws/src/gpd --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/mastering_ros_ws/devel/.private/gpd -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/mastering_ros_ws/install; cd -
catkin_clean 
tmux
roslaunch gpd tutorial0.launch 
git clone https://github.com/atenpas/gpd.git
sudo apt-get install libgflags-dev libprotobuf-dev liblmdb-dev libleveldb-dev libsnappy-dev libatlas3-base
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping_gpd.launch
roslaunch seven_dof_arm_test gpd.launch
roslaunch seven_dof_arm_test gpd.launch 
roslaunch gpd tutorial0.launch 
roslaunch seven_dof_arm_config demo.launch 
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping.launch
roslaunch seven_dof_arm_config moveit_planning_execution.launch
mkdir build && cd build
cmake ..
make -j16
sudo make install
ls
./detect_grasps ../cfg/eigen_params.cfg ../tutorials/krylon.pcd
roslaunch gpd tutorial0.launch
tmux
rostopic list
rostopic hz /detect_grasps/clustered_grasps
rostopic echo /detect_grasps/clustered_grasps
rosrun rviz rviz -f base_lin
rostopic echo /detect_grasps/clustered_grasps 
rostopic echo /detect_grasps/grasps_rviz 
roslaunch seven_dof_arm_test gpd.launch 
rostopic list
roslaunch seven_dof_arm_config moveit_planning_execution.launch
roslaunch seven_dof_arm_gazebo seven_dof_arm_world_grasping.launch 
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping.launch
tmux
roslaunch seven_dof_arm_config moveit_planning_execution.launch
roslaunch seven_dof_arm_test gpd.launch
find gpd
roscd gpd
roslaunch seven_dof_arm_test gpd.launch
roslaunch seven_dof_arm_test gpd.launch --debug
roslaunch seven_dof_arm_test gpd.launch
roslaunch seven_dof_arm_gazebo seven_dof_arm_bringup_grasping.launch
roslaunch seven_dof_arm_test gpd.launch 
cd workspaces/
ls
mkdir -p grasping_ws/src
cd grasping_ws/src/
git clone git@bitbucket.org:robot-learning/grasp_type_planning_ral_2019.git
git clone https://bitbucket.org/robot-learning/grasp_type_planning_ral_2019.git
sudo apt-get install libcoin80-dev 
sudo apt-get install libsoqt4-dev
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
sudo apt-get install libqhull-dev
sudo apt-get install libeigen3-dev
mkdir -p graspIT_ws/src
cd graspIT_ws/
git clone git@github.com:graspit-simulator/graspit.git
cd graspit
export GRASPIT=$PWD
mkdir build
cd build/
cmake ..
sudo apt-get install libqt4-sql-psql
sudo apt-get install libqt4-opengl-dev
sudo apt-get install libqt4-dev
cmake ..
make -j16
sudo make install
htop
cd graspIT_ws/src/
cd ..
catkin build
cd src/
git clone https://github.com/graspit-simulator/graspit_interface.git
git clone https://github.com/graspit-simulator/graspit_commander.git
cd ..
catkin build
gedit ~/.bashrc
roslaunch graspit_interface graspit_interface.launch 
rostopic list
rosservice list
catkin_init_workspace . 
roslaunch graspit_interface graspit_interface.launch 
echo $PWD
rosrun graspit graspit_simulator 
python2
python
rosrun graspit graspit_simulator 
roslaunch graspit_interface graspit_interface.launch 
roscore
ssh airlab@192.168.66.2
graspit_simulator 
python
python3
grep -R "graspit.pro"
git clone git@github.com:bulletphysics/bullet3.git
cd bullet3
git checkout 2.83.7
mkdir build
cd build
ccmake ..
sudo apt install cmake-curses-gui
ccmake ..
cmake ..
make -j16
sudo make install
cd ..
cd graspit/build/
ccmake ..
cmake ..
make -j5
sudo make install
rosrun graspit graspit_simulator 
export GRASPIT=$PWD
rosrun graspit graspit_simulator 
source ~/.bashrc
rosrun graspit graspit_simulator 
roscore
grep -R "BUILD_SHARED_LIBS"
clear
grep -R "DYNAMICS_ENGINE"
grep -R "EigenGrasps"
grep -R "EigenGrasps\n"
grep -R "EigenGrasps "
grep -R "eiegn/barrett_eigen.egr"
grep -R "barrett_eigen.egr"
cd ~/workspaces/graspIT_ws/src/graspit-ros/graspit/graspit_source
export GRASPIT=$PWD
rosrun graspit graspit_simulator
roscore
qmake helloWorldPlugin.pro 
cd ..
export GRASPIT=$PWD
cd plugins/helloworld/
qmake helloWorldPlugin.pro 
make -j6
export GRASPIT_PLUGIN_DIR=./lib/
graspit -p libhelloWorldPlugin
graspit_simulator -p libhelloWorldPlugin
mkdir -p ./robotics_projects_ws/src
cd ro
cd robotics_projects_ws/
catkin build
cd src/
git clone https://github.com/introlab/find-object.git
src/find_object_2d
git clone https://github.com/introlab/find-object.git -b kinetic-devel
cd ..
catkin build
cd workspaces/robotics_projects_ws/src/
git clone https://github.com/ros-drivers/usb_cam.git
cd ..
catkin build
source ~/.bashrc
roslaunch usb_cam usb_cam-test.launch 
roslaunch realsense_camera r200_nodelet_
roslaunch realsense_camera r200_nodelet_rgbd.launch 
python
rosbash
roscore
rosrun rosbash 
rosrun rosbash rosbash
rosbash
rosboost-cfg 
sudo apt-get install ros-kinetic-cv-camera 
sudo apt-get install ros-kinetic-cv-camera
source ~/.bashrc
rosrun cv_camera cv_camera_node
roscore
rosnode ping
rosnode ping /rosout 
rosnode cleanup 
rosnode ping /rosout 
rosmsg show std_msgs/Header
rosmsg show nav_msgs/Odometry 
rosmsg list 
clear
rosnode cleanup 
roscore
rostopic list
rosrun rqt_bag rqt_bag 
cd ros
cd ..
cd ..,
cd ..
cd ros
catkin build
rosnode list
rosnode info /demo_topic_publisher 
kill -p 4350
kill  4350
rosnode info /demo_topic_publisher 
kill  4726
rosnode info /demo_topic_publisher 
rosnode info /demo_topic_subscriber 
clear
rostopic type /numbers 
rostopic type /statistics 
rostopic info /statistics 
rostopic echo /statistics 
rostopic bw /statistics 
rostopic bw /numbers 
roscore
rosrun rqt_graph rqt_graph 
rosrun mastering_ros_demo_pkg demo_topic_publisher 
rosrun mastering_ros_demo_pkg demo_topic_subscriber 
tmux
rosrun mastering_ros_demo_pkg demo_topic_publisher 
roscore
rosrun mastering_ros_demo_pkg demo_topic_publisher 
source ~/.bashrc
rosrun mastering_ros_demo_pkg demo_topic_publisher 
source ~/.bashrc
rosrun mastering_ros_demo_pkg demo_topic_publisher 
source ~/.bashrc
rosrun mastering_ros_demo_pkg demo_topic_publisher 
source ~/.bashrc
rosrun mastering_ros_demo_pkg demo_topic_publisher 
source ~/.bashrc
rosrun mastering_ros_demo_pkg demo_topic_publisher 
source ~/.bashrc
rosrun mastering_ros_demo_pkg demo_topic_publisher 
source ~/.bashrc
rosrun mastering_ros_demo_pkg demo_topic_publisher 
source ~/.bashrc
roscore
source ~/.bashrc
roscore
source ~/.bashrc
roscore
rosrun mastering_ros_demo_pkg demo_topic_subscriber 
source ~/.bashrc
rosrun mastering_ros_demo_pkg demo_topic_subscriber 
source ~/.bashrc
rosrun mastering_ros_demo_pkg demo_topic_subscriber 
catkin_build
cd ..
catkin build
source ~/.bashrc
rosmsg show mastering_ros_demo_pkg/demo_msg 
catkin build
tmux
catkin build
tmux
cd workspaces/mastering_ros_ws/
catkin build
cd ..
ls
cd getting_started_ws/
catkin build
mkdir -p getting_started_ws/src
cd getting_started_ws/
catkin_init_workspace 
rm CMakeLists.txt 
cd src/
catkin_init_workspace 
cd ..
catkin build
catkin clean 
catkin_make
catkin clean 
catkin build
gedit ~/.bashrc
source  ~/.bashrc
cd src/
catkin_create_pkg mastering_ros_demo_pkg roscpp std_msgs actionlib actionlib_msgs
cd ..
catkin build
rosrun rqt_graph rqt_graph 
tmux
roscore
cd workspaces/getting_started_ws/
catkin build
source  ~/.bashrc
rossrv show mastering_ros_demo_pkg/demo_srv 
catkin build
rosrun mastering_ros_demo_pkg demo_topic_subscriber 
source  ~/.bashrc
rosrun mastering_ros_demo_pkg demo_service_client
rosrun mastering_ros_demo_pkg demo_topic_publisher 
source  ~/.bashrc
rosrun mastering_ros_demo_pkg demo_topic_publisher 
source  ~/.bashrc
rosrun mastering_ros_demo_pkg demo_service_server
rosrun mastering_ros_demo_pkg demo_action_client 
rosrun mastering_ros_demo_pkg demo_action_client 2
rosrun mastering_ros_demo_pkg demo_action_client 2 10
source  ~/.bashrc
rosrun mastering_ros_demo_pkg demo_action_client 2 10
rosrun mastering_ros_demo_pkg demo_action_client 2000000000 1
rosrun mastering_ros_demo_pkg demo_action_server 
source  ~/.bashrc
rosrun mastering_ros_demo_pkg demo_action_server 
roscore
catkin build
catkin_clean 
catkin build
source  ~/.bashrc
tmux
cd workspaces/
cd getting_started_ws/
catkin build
roslaunch mastering_ros_demo_pkg demo_topic.launch 
rosnode list
rqt_console 
roscd mastering_ros_demo_pkg/
mkdir launch
cd launch/
gedit demo_topic.launch
roslaunch mastering_ros_demo_pkg demo_topic.launch 
tmux
rosrun pluginlib_calculator calculator_loader
cd workspaces/getting_started_ws/src/
catkin_create_pkg pluginlib_calculator roscpp std_msgs
cd ..
catkin_clean 
catkin build
clear
catkin build
catkin_clean
catkin build
rqt_console 
source  ~/.bashrc
rospack plugins --attrib=plugin pluginlib_calculator
tmux
rqt_console 
catkin build
source  ~/.bashrc
rospack plugins --attrib=plugin pluginlib_calculator
rosrun pluginlib_calculator calculator_loader
catkin_clean
catkin build
rostopic list
rosrun pluginlib_calculator calculator_loader
catkin build
rosrun pluginlib_calculator calculator_loader
catkin_create_pkg nodelet_hello_world nodelet roscpp std_msgs
rostopic list
roscore
rosrun nodelet nodelet load nodelet_hello_world/Hello nodelet_manager
htop
cd workspaces/getting_started_ws/
catkin build
roscore
source  ~/.bashrc
roscore
source  ~/.bashrc
roscore
rosrun nodelet nodelet load nodelet_hello_world/Hello nodelet_manager __name:=nodelet1
source  ~/.bashrc
rosrun nodelet nodelet load nodelet_hello_world/Hello nodelet_manager __name:=nodelet1
source  ~/.bashrc
rosrun nodelet nodelet load nodelet_hello_world/Hello nodelet_manager __name:=nodelet1
rosrun nodelet nodelet manager __name:=nodelet_manager
source  ~/.bashrc
rosrun nodelet nodelet manager __name:=nodelet_manager
source  ~/.bashrc
rosrun nodelet nodelet manager __name:=nodelet_manager
rostopic echo /nodelet1/msg_out 
rostopic list
clear
rostopic list
rostopic pub /nodelet1/msg_in std_msgs/String "Hello"
rosnode list
rosrun rqt_graph rqt_graph 
rosrun rqt_gui rqt_gui 
rostopic list
tmux
catkin build
source  ~/.bashrc
tmux
catkin build
tmux
source  ~/.bashrc
roslaunch nodelet_hello_world hello_world.launch
sudo apt-get install libgazebo7-dev
mkdir gazebo_basic_world_plugin
cd gazebo_basic_world_plugin/
nano hello_world.cc
subl hello_world.cc
gedit hello_world.cc
mkdir build
cd build/
cmake ../
make
gedit ~/.bashrc
source ~/.basrhc
source ~/.bashrc
gzserver hello_world --verbose
gzserver hello.world --verbose
gzserver hello.world --verbose
gzserver hello_world.cc --verbose
gzserver hello.world --verbose
htop
clear
htop
catkin build
source ~/.bashrc
rospack plugins --attrib=plugin controller_interface
catkin build
roslaunch my_controller my_controller.launch 
catkin_create_pkg my_controller roscpp pluginlib controller_interface
git clone https://github.com/jocacace/my_controller
cd ..
catkin_clean
cd build/
catkin build
rosservice call /controller_manager/list_controllers
rosservice call /controller_manager/switch_controller "start_controllers:
- ''
stop_controllers:
- ''
strictness: 0" 
rosservice call /controller_manager/switch_controller "start_controllers:
- ''
stop_controllers:
- 'my_controller_name'
strictness: 0" 
rosservice call /controller_manager/switch_controller "start_controllers:
- 'my_controller_name'
stop_controllers:
- ''
strictness: 0" 
rosservice call /controller_manager/switch_controller "start_controllers:
- ''
stop_controllers:
- 'my_controller_name'
strictness: 0" 
rosservice call /controller_manager/switch_controller "start_controllers:
- 'my_controller_name'
stop_controllers:
- ''
strictness: 0" 
rosservice call /controller_manager/switch_controller "start_controllers:
- ''
stop_controllers:
- 'my_controller_name'
strictness: 0" 
roslaunch my_controller my_controller.launch 
roslaunch my_controller my_controller.launch 
tmux
catkin_clean
catkin build
source ~/.bashrc
roslaunch my_controller my_controller.launch
catkin build
source ~/.bashrc
roslaunch my_controller my_controller.launch
catkin build
source ~/.bashrc
roslaunch my_controller my_controller.launch
catkin build
source ~/.bashrc
roslaunch my_controller my_controller.launch
catkin build
source ~/.bashrc
roslaunch my_controller my_controller.launch
catkin_clean 
catkin build
source ~/.bashrc
roslaunch my_controller my_controller.launch
catkin build
source ~/.bashrc
roslaunch my_controller my_controller.launch
catkin build
source ~/.bashrc
roslaunch my_controller my_controller.launch
catkin build
catkin_clean 
catkin build
source ~/.bashrc
roslaunch my_controller my_controller.launch
catkin build
source ~/.bashrc
roslaunch my_controller my_controller.launch
catkin build
source ~/.bashrc
roslaunch my_controller my_controller.launch
catkin build
source ~/.bashrc
roslaunch my_controller my_controller.launch
catkin build
source ~/.bashrc
roslaunch my_controller my_controller.launch
rostopic list
rostopic info /cmd_vel 
rostopic list
rostopic info /cmd_vel 
rostopic echo /cmd_vel 
rviz
catkin_create_pkg rviz_teleop_commander roscpp rviz std_msgs
cd rviz_teleop_commander/
cd .
cd ..
catkin build
roscore
mkdir -p ./air_pr2_ws/src
cd air_pr2_ws/
catkin_init_workspace src/
catkin build
cd src/
git clone https://github.com/PR2/pr2_simulator.git -b kinetic-devel
cd ..
catkin build
cd src/
git clone https://github.com/PR2/pr2_mechanism.git -b kinetic-devel
cd ..
catkin build
cd src/
git clone https://github.com/PR2/pr2_common.git -b kinetic-devel
cd .. & catkin biuld
cd .. && catkin biuld
cd .. && catkin build
cd .. & catkin build
cd ..
cd workspaces/
cd air_pr2_ws/
cd src/
cd .. ; catkin build
cd src/
git clone https://github.com/ros/convex_decomposition.git -b kinetic-devel
cd .. ; catkin build
cd src/
git clone https://github.com/ros/ivcon.git -b kinetic-devel
cd .. ; catkin build
catkin build
gedit ~/.bashrc
roslaunch pr2_gazebo pr2.launch
cd workspaces/air_pr2_ws/
cd src/
git clone https://github.com/PR2/pr2_controllers.git -b kinetic-devel
cd..
cd ..
catkin build
roslaunch gazebo_ros empty_world.launch
rostopic echo joint_states | less
rosmake pr2_teleop
roslaunch pr2_teleop teleop_joystick.launch 
roslaunch pr2_teleop teleop_keyboard.launch 
roslaunch pr2_gazebo pr2.launch
source ~/.bashrc
roslaunch pr2_gazebo pr2.launch
tmux
roslaunch pr2_teleop teleop_keyboard.launch
rosservice call laser_tilt_controller/set_periodic_cmd '{ command: { header: { stamp: 0 }, profile: "linear" , period: 3 , amplitude: 1 , offset: 0 }}'
rostopic list | grep r_gripper
rostopic info /r_gripper_controller/command
rosmsg show pr2_controllers_msgs/Pr2GripperCommand 
rostopic pub r_gripper_controller/command pr2_controllers_msgs/Pr2GripperCommand "position: 0.0
max_effort: 100.0" 
rostopic pub r_gripper_controller/command pr2_controllers_msgs/Pr2GripperCommand "position: 1.0
max_effort: 100.0" 
rostopic pub r_gripper_controller/command pr2_controllers_msgs/Pr2GripperCommand "position: 3.0
max_effort: 100.0" 
rostopic pub r_gripper_controller/command pr2_controllers_msgs/Pr2GripperCommand "position: 0.0
max_effort: 100.0" 
rosrun rviz rviz
ssh airlab@192.168.66.2
roscd gazebo_ros
ls
roslaunch pr2_gazebo pr2_empty_world.launch
cd workspaces/air_pr2_ws/src/
git clone https://github.com/PR2-prime/pr2_navigation.git -b kinetic-devel
cd ..
cd src/
git clone https://github.com/ros-planning/navigation.git -b kinetic-devel
cd ..
catkin build
roslaunch pr2_navigation pr2_nav_tutorial.launch
roscd pr2_navi
roslaunch pr2_navigation/launch/pr2_nav_tutorial.launch
tmux
cd workspaces/air_pr2_ws/src/
catkin build
roslaunch '/home/brky/workspaces/air_pr2_ws/src/pr2_navigation/pr2_navigation/launch/pr2_nav_tutorial.launch' 
cd workspaces/air_pr2_ws/src/
catkin build
roscd gazebo_ros
ls
mkdir Media
sudo -p mkdir Media/materials/textures/
sudo -p mkdir ./Media/materials/textures/
sudo mkdir -p ./Media/materials/textures/
ls
cd Media/materials/textures/
ls
sudo cp map_blank.png /opt/ros/kinetic/share/gazebo_ros/Media/materials/textures/
ls
git clone https://github.com/PR2/pr2_common_actions.git -b kinetic-devel
cd ..
catkin biuld
catkin build
roslaunch '/home/brky/workspaces/air_pr2_ws/src/pr2_navigation/pr2_navigation/launch/pr2_nav_tutorial.launch' 
cd workspaces/air_pr2_ws/src/
git clone https://github.com/PR2/pr2_apps.git -b kinetic-devel
cd .. ; catkin build
cd src/
git clone https://github.com/ros-drivers/joystick_drivers.git -b indigo-devel
cd .. ; catkin build
catkin build
cd air_pr2_ws/
catkin build
htop
roslaunch pr2_navigation_global rviz_move_base.launch
roslaunch '/home/brky/workspaces/air_pr2_ws/src/pr2_navigation/pr2_navigation/launch/pr2_nav_tutorial.launch' 
roslaunch pr2_gazebo pr2_empty_world.launch
tmux
catkin_create_pkg gazebo_worlds roscpp rospy gazebo
cd ..; catkin build
catkin_create_pkg gazebo_worlds roscpp rospy gazebo_ros
cd ..; catkin build
cd air_pr2_ws/; catkin build
cd workspaces/air_pr2_ws/src/
ls
chmod +x drawer.urdf 
rosrun gazebo_ros spawn_model -urdf -file drawer.urdf -model drawer1 -x 1.0
rospack find gazebo_worlds
rosrun gazebo spawn_model -urdf -file table.urdf -model table -x 1.0 -y 0.5 -z 0.3
rosrun gazebo_ros spawn_model -urdf -file table.urdf -model table -x 1.0 -y 0.5 -z 0.3
source ~/.bashrc
roslaunch pr2_gazebo pr2_empty_world.launch
rosclean 
rosclean purge 
roslaunch pr2_gazebo pr2_empty_world.launch
rosrun gazebo_ros spawn_model -urdf -file workspaces/air_pr2_ws/src/table.urdf -model table -x 1.0 -y 0.5 -z 0.3
rosrun gazebo_ros spawn_model -urdf -file workspaces/air_pr2_ws/src/table.urdf -model table 
rosrun gazebo_ros spawn_model -gazebo -file `rospack find gazebo_worlds`/objects/coffee_cup.model -model coffee_cup -x 1.2 -z 1
rosrun gazebo_ros spawn_model -model -file `rospack find gazebo_worlds`/objects/coffee_cup.model -model coffee_cup -x 1.2 -z 1
roslaunch gazebo_worlds table.launch 
sudo apt-get install ros-kinetic-gazebo-dev 
roslaunch gazebo_worlds table.launch 
roslaunch gazebo_worlds coffee_cup.launch 
roslaunch gazebo_worlds cylinder_object.launch 
rosrun rviz rviz
killall -9 rviz
roslaunch gazebo_worlds air_pr2.launch 
roslaunch pr2_gazebo pr2_table_
roslaunch pr2_gazebo pr2_table_object.launch 
tmux
ssh airlab@192.168.66.2
roslaunch pr2_gazebo pr2_empty_world.launch
cd workspaces/air_pr2_ws/src/
ls
roslaunch jtteleop.launch 
cd ..
ls
roslaunch jtteleop.launch 
rosrun robot_mechanism_controllers jt_cartesian_controller.cpp 
clear
rosrun robot_mechanism_controllers jt_cartesian_controller.cpp 
rospack find robot_mechanism_controllers 
roslaunch pr2_gazebo pr2_empty_world.launch
roscd pr2_controller_manager/
tmux
sendiso --help
rosrun pr2_controller_manager pr2_controller_manager list
clear
rosrun pr2_controller_manager pr2_controller_manager list
roslaunch pr2_gazebo pr2_empty_world.launch
rosrun pr2_controller_manager pr2_controller_manager list-joints
rqt_plot /joint_states/position[15]
rqt_plot /joint_states/position[16]
rosservice call laser_tilt_controller/set_periodic_cmd '{ command: { header: { stamp: 0 }, profile: "linear" , period: 3 , amplitude: 1 , offset: 0 }}'
rostopic echo /joint_states/positions[16]
rostopic echo /joint_states/position[16]
rosservice call laser_tilt_controller/set_periodic_cmd '{ command: { header: { stamp: 0 }, profile: "linear" , period: 3 , amplitude: 1 , offset: 0 }}'
clear
rosservice call laser_tilt_controller/set_periodic_cmd '{ command: { header: { stamp: 0 }, profile: "linear" , period: 3 , amplitude: 1 , offset: 0 }}'
tmux
typora 
catkin build
grep -R "machanism"
catkin_create_pkg my_controller_pkg pr2_controller_interface pr2_machanism_model pluginlip roscpp
cd ..
catkin build
rospack plugins --attrib=plugin pr2_controller_interface
cd workspaces/air_pr2_ws/
catkin build
rosparam list 
clear
rosparam set my_controller_name/type my_controller_pkg/MyControllerPlugin
rosparam get /my_controller_name/type 
rosrun pr2_controller_manager pr2_controller_manager list-joints
rosparam set my_controller_name/joint_name r_shoulder_pan_joint
rosparam get -p my_controller_name
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager load my_controller_name
rosrun pr2_controller_manager pr2_controller_manager list
rqt_console
rospack plugins --attrib=plugin pr2_controller_interface
clear
rosparam get /my_controller_name/joint_name 
rosparam get /my_controller_name/type 
rosparam get -p my_controller_name
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager stop base_controller
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager list-type
rosrun pr2_controller_manager pr2_controller_manager list-types
rosrun pr2_controller_manager pr2_controller_manager load my_controller_pkg/MyControllerPlugin
rosrun pr2_controller_manager pr2_controller_manager load my_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager load my_controller_name
cd workspaces/air_pr2_ws/
catkin clean my_controller_pkg
catkin build my_controller_pkg
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager list-types
rosrun pr2_controller_manager pr2_controller_manager load my_controller_name
source ~/.bashrc
rospack plugins --attrib=plugin pr2_controller_interface 
clear
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager list-types
rosparam get -p my_controller_name
clear
rosrun pr2_controller_manager pr2_controller_manager load my_controller_name
rosparam set my_controller_name/type my_controller_ns/MyControllerClass
rosrun pr2_controller_manager pr2_controller_manager list-types
rosparam get -p my_controller_name
rosrun pr2_controller_manager pr2_controller_manager load my_controller_name
rosrun pr2_controller_manager pr2_controller_manager list
source ~/.bashrc
rospack plugins --attrib=plugin pr2_controller_interface
rosrun rqt_console rqt_console 
cd workspaces/air_pr2_ws/
catkin builf
catkin build
catkin build my_controller_pkg
sudo rm -rf lost+found/
rosparam set my_controller_name/type my_controller_pkg/MyControllerPlugin
rosrun pr2_controller_manager pr2_controller_manager list-joints
rosparam set my_controller_name/joint_name r_shoulder_pan_joint
rosparam get -p my_controller_name
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager list-types
rosrun pr2_controller_manager pr2_controller_manager load my_controller_name
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager start my_controller_name
rosrun pr2_controller_manager pr2_controller_manager stop my_controller_name
rosrun pr2_controller_manager pr2_controller_manager unload my_controller_name
rosrun pr2_controller_manager pr2_controller_manager list
rosparam load my_controller.yaml
cd workspaces/air_pr2_ws/src/
rosparam load my_controller.yaml
cd my_controller_pkg/
rosparam load my_controller.yaml
rosrun pr2_controller_manager pr2_controller_manager spawn my_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_controller_name
roslaunch my_controller_pkg my_controller.launch 
roslaunch gazebo_worlds empty_world.launch
killall -9 gzserver
killall -9 gzclient
roslaunch gazebo_worlds empty_world.launch
roslaunch pr2_gazebo pr2.launch
tmux
htop
catkin build
source ~/.bashrc
rossrv show my_controller_pkg/SetAmplitude 
rosrun pr2_controller_manager pr2_controller_manager list
rosparam set my_controller_name/type my_controller_pkg/MyControllerPlugin
rosparam set my_controller_name/joint_name r_shoulder_pan_joint
rosparam get -p my_controller_name
rosrun pr2_controller_manager pr2_controller_manager list
roslaunch my_controller_pkg my_controller.launch 
rosrun pr2_controller_manager pr2_controller_manager reload-libraries
rosservice call /my_controller_name/set_amplitude 0.1
cd workspaces/air_pr2_ws/
catkin build
rosparam get list
rosparam list
rosparam set /my_controller_name/pid_parameters/d 1123.0
rosservice call /my_controller_name/set_amplitude 0.1
rosservice call /my_controller_name/set_amplitude 1.9
rosparam set /my_controller_name/pid_parameters/d 1123.0
rosparam set /my_controller_name/pid_parameters/p 1123.0
rosparam set /my_controller_name/pid_parameters/i 1123.0
rosrun rqt_reconfigure rqt_reconfigure 
rosrun pr2_controller_manager pr2_controller_manager reload-libraries
rosparam set my_controller_name/type my_controller_pkg/MyControllerPlugin
rosparam set my_controller_name/joint_name r_shoulder_pan_joint
rosparam get -p my_controller_name
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager list-types
rosrun pr2_controller_manager pr2_controller_manager load my_controller_name
rosrun pr2_controller_manager pr2_controller_manager start my_controller_name
rosservice call /my_controller_name/set_amplitude 0.1
rosservice call /my_controller_name/set_amplitude 2.9
rosservice call /my_controller_name/set_amplitude 0.4
rosservice call /my_controller_name/set_amplitude 0.5
rosservice call /my_controller_name/set_amplitude 0.6
rosservice call /my_controller_name/set_amplitude 0.7
rosservice call /my_controller_name/set_amplitude 1.7
rosservice call /my_controller_name/set_amplitude 3.7
rosservice call /my_controller_name/set_amplitude 1.99
source ~/.bashrc
rosparam set my_controller_name/type my_controller_pkg/MyControllerPlugin
rosparam set my_controller_name/joint_name r_shoulder_pan_joint
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager load my_controller_name
rosrun pr2_controller_manager pr2_controller_manager start my_controller_name
rosservice call /my_controller_name/set_amplitude 0.1
rosservice call /my_controller_name/set_amplitude 1.9
rosservice call /my_controller_name/set_amplitude 0.9
rosservice call /my_controller_name/set_amplitude 0.1
rosservice call /my_controller_name/set_amplitude 1.9999999999
rosservice call /my_controller_name/set_amplitude 11.9999999999
source ~/.bashrc
rosparam set my_controller_name/type my_controller_pkg/MyControllerPlugin
rosparam set my_controller_name/joint_name r_shoulder_pan_joint
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager load my_controller_name
rosparam load ~/workspaces/air_pr2_ws/src/my_controller_pkg/my_controller.yaml 
rosrun pr2_controller_manager pr2_controller_manager spawn my_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_controller_name
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager start r_arm_controller
rostopic pub /r_arm_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- ''
points:
- positions: [0]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /r_arm_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- ''
points:
- positions: [0]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 2, nsecs: 0}" 
rostopic pub /r_arm_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- ''
points:
- positions: [1]
  velocities: [1]
  accelerations: [0]
  effort: [1]
  time_from_start: {secs: 0, nsecs: 0}" 
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager spawn my_controller_name
roslaunch my_controller_pkg my_controller.launch 
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager spawn my_controller_name
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager spawn my_controller_name
rosparam set my_controller_name/type my_controller_pkg/MyControllerPlugin
rosrun pr2_controller_manager pr2_controller_manager spawn my_controller_name
rosparam load ~/workspaces/air_pr2_ws/src/my_controller_pkg/my_controller.yaml 
rosrun pr2_controller_manager pr2_controller_manager spawn my_controller_name
rosservice call /my_controller_name/set_amplitude 11.9999999999
rosservice call /my_controller_name/set_amplitude 1.9999999999
rosservice call /my_controller_name/set_amplitude 0.09999999999
roslaunch gazebo_worlds empty_world.launch
roslaunch pr2_gazebo pr2.launch
source ~/.bashrc
roslaunch pr2_gazebo pr2.launch
source ~/.bashrc
roslaunch pr2_gazebo pr2.launch
roslaunch gazebo_worlds empty_world.launch
source ~/.bashrc
roslaunch gazebo_worlds empty_world.launch
killall -9 gzserver
roslaunch gazebo_worlds empty_world.launch
source ~/.bashrc
roslaunch gazebo_worlds empty_world.launch
killall -9 gzserver
roslaunch gazebo_worlds empty_world.launch
killall -9 gzserver
roslaunch gazebo_worlds empty_world.launch
killall -9 gzserver
roslaunch gazebo_worlds empty_world.launch
tmux
grep -R "brkygkcn"
catkin build
rosparam set my_controller_name/type my_controller_pkg/MyControllerPlugin
rosparam load ~/workspaces/air_pr2_ws/src/my_controller_pkg/my_controller.yaml
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager load my_controller_name
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager start my_controller_name
rosservice call /my_controller_name/capture
rostopic echo /my_controller_name/mystate_topic
roslaunch pr2_gazebo pr2.launch
roslaunch gazebo_worlds empty_world.launch
tmux
catkin build
source ~/.bashrc
rostopic list
clear
source ~/.bashrc
rosparam load my_cart_controller.yaml
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager list
clear
rosrun pr2_controller_manager pr2_controller_manager list
roslaunch my_controller_pkg my_cart_controller.launch 
source ~/.bashrc
roslaunch my_controller_pkg my_cart_controller.launch 
rosparam load my_cart_controller.yaml
rosparam set my_controller_name/type my_controller_pkg/MyControllerPlugin
rosrun pr2_controller_manager pr2_controller_manager load my_controller_name
rosrun pr2_controller_manager pr2_controller_manager start my_controller_name
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
rosrun pr2_controller_manager pr2_controller_manager kill my_cart_controller_name
source ~/.bashrc
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
roscore
rosparam load my_cart_controller.yaml
rosrun pr2_controller_manager pr2_controller_manager spawn my_cart_controller_name
roslaunch my_controller_pkg my_controller.launch 
roslaunch gazebo_worlds empty_world.launch
source ~/.bashrc
roslaunch gazebo_worlds empty_world.launch
killall -9 gzserver
roslaunch gazebo_worlds empty_world.launch
roslaunch pr2_gazebo pr2.launch
rosparam load my_cart_controller.yaml
tmux
cd workspaces/air_pr2_ws/
catkin build
catkin_create_pkg my_controller_cart_pkg pr2_controller_interface pr2_mechanism_model pluginlib roscpp
catkin build
catkin clean my_controller_cart_pkg
catkin build my_controller_cart_pkg
catkin build
source ~/.bashrc
rospack plugins --attrib=plugin pr2_controller_interface
roslaunch pr2_gazebo pr2.launch
roslaunch gazebo_worlds empty_world.launch
killall -9 gzserver
roslaunch gazebo_worlds empty_world.launch
rosrun pr2_controller_manager pr2_controller_manager list-types
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager stop r_gripper_controller
rosrun pr2_controller_manager pr2_controller_manager list
roslaunch my_controller_cart_pkg my_controller_cart.launch 
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager start r_gripper_controller
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager start r_gripper_controller
rosrun pr2_controller_manager pr2_controller_manager start r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller
rosrun pr2_controller_manager pr2_controller_manager list
tmux
killall -9 rviz
roslaunch pr2_gazebo pr2_table_object.launch
rviz
grep -R "robot_mechanism_controllers/JTCartesianController"
grep -R "JTCartesianController"
grep -R "JTCartesianController"
cd ..
grep -R "robot_mechanism_controllers/JTCartesianController"
grep -R "<class name="robot_mechanism_controllers/JTCartesianController"
grep -R "robot_mechanism_controllers/JTCartesianController"
rospack find robot_mechanism_controllers
rosrun pr2_controller_manager pr2_controller_manager list
catkin_create_pkg teleop_arms geometry_msgs roscpp
roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch jtteleop.launch 
tmux
catkin biuld
catkin build
roscd teleop_arms/
rosrun pr2_controller_manager pr2_controller_manager list
rosrun teleop_arms teleop_pr2_arms_keyboard
cd workspaces/air_pr2_ws/
catkin build
source ~/.bashrc
rosrun teleop_arms teleop_pr2_arms_keyboard
rosrun teleop_arms teleop_pr2_arms
catkin build
cd /home/brky/workspaces/air_pr2_ws/build/teleop_arms; catkin build --get-env teleop_arms | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
source ~/.bashrc
rosrun teleop_arms teleop_pr2_arms_keyboard 
clear
rosrun teleop_arms teleop_pr2_arms_keyboard
rosrun pr2_controller_manager pr2_controller_manager list
roslaunch workspaces/air_pr2_ws/src/jtteleop.launch 
roslaunch pr2_gazebo pr2_empty_world.launch
tmux
export ROBOT=sim
roslaunch pr2_tabletop_manipulation_gazebo_demo pr2_tabletop_manipulation_demo.launch
rostopic echo /amcl_pose 
rostopic echo amcl_pose
rosparam set /fake_localization/odom_frame_id /odom_combined
rosparam set /fake_localization/base_frame_id /base_footprint
rosrun fake_localization fake_localization 
rosparam set /fake_localization/odom_frame_id odom_combined
rosparam set fake_localization/base_frame_id base_footprint
rosparam set fake_localization/odom_frame_id odom_combined
rosrun fake_localization fake_localization 
tmuc
tmux
ps -ef | grep 'gazebo/bin/gazebo'
localhost:~/ros/ros-pkg> gdb --pid 12473
~/ros/ros-pkg> gdb --pid 12473
./ros/ros-pkg> gdb --pid 12473
ls
cd ..
ls
cd ..
ls
cd opt/
ls
ros/ros-pkg> gdb --pid 12473
sudo ros/ros-pkg> gdb --pid 12473
sudo su
ps -ef | grep 'gazebo/bin/gazebo'
gdb 12861
gdb --pid 12861
ps -ef | grep 'gazebo/bin/gazebo'
gdb --pid 12875
ps -ef | grep 'gazebo/bin/gazebo'
ps -ef | grep 'gazebo*
gdb --pid 6123
sudo gdb --pid 6123
roslaunch pr2_gazebo pr2_table_object.launch
rostopic echo /base_controller/command 
rosrun rviz rviz 
roslaunch pr2_teleop teleop_keyboard.launch
roslaunch pr2_gazebo pr2.launch
roslaunch gazebo_worlds empty_world.launch
tmux
svn co https://code.ros.org/svn/wg-ros-pkg/stacks/wg_robots_gazebo/trunk/pr2_2dnav_gazebo/
svn co  https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/stacks/wg_robots_gazebo/pr2_2dnav_gazebo
roslaunch manipulation_worlds pr2_table_object.launch
cd workspaces/air_pr2_ws/src/
git clone https://github.com/ros-interactive-manipulation/pr2_object_manipulation.git -b groovy-devel
cd ..
catkin build
source ~/.bashrc
roscd manipulation_worlds
roscd /manipulation_worlds
cd src/
ls
cd pr2_object_manipulation/
ls
cd ..
rosmake pr2_object_manipulation
cd pr2_object_manipulation/
ls
cd applications/
ls
cd manipulation_worlds/
ls
rosmake Makefile
catkin_create_pkg manipulation_worlds
catkin build
source ~/.bashrc
roscd manipulation_worlds/
roscd pr2_tabletop_manipulation_launch
catkin_create_pkg pr2_tabletop_manipulation_launch
cd .. ; catkin build
cd src/
git clone https://github.com/ros-interactive-manipulation/household_objects_database_msgs.git
roslaunch pr2_tabletop_manipulation_launch pr2_tabletop_manipulation.launch
source ~/.bashrc
roslaunch pr2_tabletop_manipulation_launch pr2_tabletop_manipulation.launch
source ~/.bashrc
roslaunch pr2_tabletop_manipulation_launch pr2_tabletop_manipulation.launch
source ~/.bashrc
roslaunch pr2_tabletop_manipulation_launch pr2_tabletop_manipulation.launch
git clone https://github.com/ros-interactive-manipulation/household_objects_database.git
catkin build household_objects_database
git clone https://github.com/ros-interactive-manipulation/manipulation_msgs.git
catkin build household_objects_database
git clone https://github.com/ros-interactive-manipulation/sql_database.git
catkin build household_objects_database
catkin_create_pkg pr2_object_manipulation_launch
catkin_create_pkg pr2_object_manipulation_launch/
catkin build pr2_object_manipulation_launch
git clone https://github.com/ros-drivers/openni_camera.git
catkin build openni_launch 
roslaunch manipulation_worlds pr2_table_object.launch
export ROBOT=sim
roslaunch manipulation_worlds pr2_table_object.launch
killall -9 gzserver
roslaunch manipulation_worlds pr2_table_object.launch
roslocate svn pr2_gazebo_wg
roslocate svn pr2_gazebo_wg roslocate svn pr2_gazebo_wg
roslocate svn pr2_gazebo_wg
roslocate svn pr2_gazebo_wg https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/stacks/wg_robots_gazebo/pr2_gazebo_wg
roslocate svn pr2_gazebo_wg www https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/stacks/wg_robots_gazebo/pr2_gazebo_wg
roslocate www pr2_gazebo_wg https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/stacks/wg_robots_gazebo/pr2_gazebo_wg
roslocate www https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/stacks/wg_robots_gazebo/pr2_gazebo_wg
catkin build
export ROBOT=sim
roslaunch pr2_plugs_gazebo_demo pr2_plugs_demo.launch 
roscd pr2_plugs_gazebo_demo
roslaunch pr2_plugs_gazebo_demo pr2_plugs_demo.launch 
roscd pr2_plugs_executive
catkin_create_pkg pr2_plugs_gazebo_demo
roscd pr2_tuck_arms_action
sudo apt-get install libeigen3-dev 
chmod +x *
grep -R "bodies"
git clone https://github.com/PR2/pr2_object_manipulation.git
cd ..
catkin build
catkin clean pr2_tabletop_manipulation_launch
catkin build
catkin clean pr2_tabletop_manipulation_launch
catkin build
catkin clean manipulation_worlds
catkin build
cd /home/brky/workspaces/air_pr2_ws/build/tabletop_object_detector; catkin build --get-env tabletop_object_detector | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin biuld
catkin build
cd /home/brky/workspaces/air_pr2_ws/build/tabletop_object_detector; catkin build --get-env tabletop_object_detector | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
cd /home/brky/workspaces/air_pr2_ws/build/robot_self_filter_color; catkin build --get-env robot_self_filter_color | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cleaer
clear
cd /home/brky/workspaces/air_pr2_ws/build/robot_self_filter_color; catkin build --get-env robot_self_filter_color | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd workspaces/air_pr2_ws/
catkin build
catkin clean pr2_object_manipulation
catkin build pr2_object_manipulation
catkin build 
cd /home/brky/workspaces/air_pr2_ws/build/robot_self_filter_color; catkin build --get-env robot_self_filter_color | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build 
cd /home/brky/workspaces/air_pr2_ws/build/tabletop_object_detector; catkin build --get-env tabletop_object_detector | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd src/
git clone https://github.com/ros-planning/geometric_shapes.git -b hydro-devel
cd ..
catkin build
catkin build geometric_shapes
cd src/
git clone https://github.com/ros-planning/geometric_shapes.git 
cd ..
catkin build geometric_shapes
catkin build 
cd /home/brky/workspaces/air_pr2_ws/build/tabletop_object_detector; catkin build --get-env tabletop_object_detector | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin clean
catkin build
git clone https://github.com/PR2/pr2_object_manipulation.git
cd ..; catkin build 
cd src/
git clone https://github.com/PR2/pr2_object_manipulation.git -b hydro-devel
cd ..; catkin build 
cd src/
git clone https://github.com/PR2/pr2_object_manipulation.git
cd ..; catkin build 
cd air_pr2_ws/
cd src/
cd ..; catkin build 
cd air_pr2_ws/
catkin build
chmod +x ./*
catkin_create_pkg distance_field
catkin build tabletop_object_detector
catkin build household_objects_database_msgs
catkin build tabletop_object_detector
source ~/.bashrc
catkin build tabletop_object_detector
catkin build distance_field
source ~/.bashrc
cd src/
ls
catkin build distance_field
gedit ~/.bashrc
source ~/.bashrc
roscd distance_field
rospack find distance_field
source ~/.bashrc
gedit ~/.bashrc
catkin build distance_field
source ~/.bashrc
catkin build tabletop_object_detector
cd /home/brky/workspaces/air_pr2_ws/build/tabletop_object_detector; catkin build --get-env tabletop_object_detector | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
source ~/.bashrc
git clone https://github.com/ros-interactive-manipulation/tabletop_object_perception.git
cd ..
catkin build tabletop_object_detector
catkin clean tabletop_object_detector
catkin clean 
catkin build
cd src/
cd ..
catkin build
cd /home/brky/workspaces/air_pr2_ws/build/tf_throttle; catkin build --get-env tf_throttle | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
catkin build tf_throttle
catkin build
cd src/
git clone https://github.com/PR2/arm_navigation_msgs.git -b hydro-devel
cd ..
catkin build arm_navigation_msgs
catkin build
catkin build interactive_perception_msgs 
catkin build
source ~/.bashrc
catkin build
catkin build interactive_perception_msgs 
rosmsg show interactive_perception_msgs/ModelHypothesis
catkin build
catkin build interactive_perception_msgs 
rosmsg show interactive_perception_msgs/ModelHypothesis
rosmsg show arm_navigation_msgs/Shape 
git clone https://github.com/ros-interactive-manipulation/object_manipulation.git -b groovy-devel
git clone https://github.com/ros-interactive-manipulation/object_manipulation.git -b catkinize
catkin_create_pkg pr2_tabletop_manipulation_launch
catkin build
cd /home/brky/workspaces/air_pr2_ws/build/tabletop_collision_map_processing; catkin build --get-env tabletop_collision_map_processing | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
rospack find object_manipulation_msgs
catkin build object_manipulation_msgs
source ~/.bashrc
cd /home/brky/workspaces/air_pr2_ws/build/tabletop_collision_map_processing; catkin build --get-env tabletop_collision_map_processing | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build 
catkin clena
catkin clean 
catkin build
catkin build pr2_object_manipulation_launch
source ~/.bashrc
catkin build pr2_object_manipulation_launh
catkin build pr2_object_manipulation_launch
catkin_create_pkg object_manipulator
catkin build pr2_tabletop_manipulation_launch
catkin build
catkin build pr2_create_object_model
catkin build tabletop_collision_map_processing
catkin build object_manipulator 
cd src/
git clone https://github.com/PR2/kinematics_msgs.git -b hydro-devel
cd ..
catkin build kinematics_msgs
source ~/.bashrc
catkin build object_manipulator 
grep _SetInterpolatedIKMotionPlanParams.py | cut -c 5-
cut -c 5- _SetInterpolatedIKMotionPlanParams.py > new_file.txt
cut -c 6- _SetInterpolatedIKMotionPlanParams.py > new_file.txt
cut -c 7- _SetInterpolatedIKMotionPlanParams.py > new_file.txt
cut -c 7- ik_utilities.py > new_file.txt
cut -c 7- interpolated_ik_motion_planner.py > interpolated_ik_motion_planner.py
cut -c 7- interpolated_ik_motion_planner.py > interpolated_ik_motion_planner.pyas
cd ..
cd include/interpolated_ik_motion_planner/
cut -c 7- SetInterpolatedIKMotionPlanParams.h > SetInterpolatedIKMotionPlanParams.ha 
cd ..
cd scripts/
cut -c 7- test_interpolated_ik_motion_planner.py > test_interpolated_ik_motion_planner.pya
catkin_create_pkg interpolated_ik_motion_planner rospy
catkin_create_pkg arm_kinematics_constraint_aware roscpp
cut -c 7- create_ik_fast_plugin.py > create_ik_fast_plugina.py 
cut -c 7- arm_kinematics_constraint_aware.cpp > arm_kinematics_constraint_awarea.cpp
cut -c 7- arm_kinematics_constraint_aware_utils.cpp > arm_kinematics_constraint_aware_utilsa.cpp 
cut -c 7- arm_kinematics_solver_constraint_aware.cpp > arm_kinematics_solver_constraint_awarea.cpp 
cut -c 7- kdl_arm_kinematics_plugin.cpp > kdl_arm_kinematics_plugina.cpp 
cut -c 7- main.cpp > maina.cpp 
cut -c 7- arm_kinematics_constraint_aware.h > arm_kinematics_constraint_awarea.h 
cut -c 7- arm_kinematics_constraint_aware_utils.h > arm_kinematics_constraint_aware_utilsa.h 
cut -c 7- arm_kinematics_solver_constraint_aware.h > arm_kinematics_solver_constraint_awarea.h 
cut -c 7- kdl_arm_kinematics_plugin.h > kdl_arm_kinematics_plugina.h 
catkin_create_pkg kinematics_base
catkin build arm_kinematics_constraint_aware 
roscd kinematics_base
source ~/.bashrc
catkin build kinematics_base
gedit ~/.bashrc
source ~/.bashrc
catkin build kinematics_base 
catkin build arm_kinematics_constraint_aware 
source ~/.bashrc
catkin build arm_kinematics_constraint_aware 
roscd kinematics_base/
catkin build arm_kinematics_constraint_aware 
catkin build kinematics_base 
catkin build arm_kinematics_constraint_aware 
catkin_create_pkg planning_environment rospy
cut -c 7- * > *a
grep *
echo grep * 
cut -c 7- * > *
cut -c 7- * > *.txt
cut -c 7- collision_models (copy).cpp > collision_models.cpp
cut -c 7- "collision_models (copy).cpp" > collision_models.cpp
cut -c 7- "collision_models_interface (copy).cpp" > collision_models_interface.cpp 
cut -c 7- "collision_operations_generator (copy).cpp" > collision_operations_generator.cpp
cut -c 7- "collision_operations_generator_test (copy).cpp" > collision_operations_generator_test.cpp
cut -c 7- "collision_space_monitor (copy).cpp" > collision_space_monitor.cpp
cut -c 7- "construct_object (copy).cpp" > construct_object.cpp
cut -c 7- "default_joint_state_publisher (copy).cpp" > default_joint_state_publisher.cpp
cut -c 7- "environment_server (copy).cpp" > environment_server.cpp
cut -c 7- "filter_attached_objects (copy).cpp" > filter_attached_objects.cpp
cut -c 7- "joint_state_decumulator (copy).cpp" > joint_state_decumulator.cpp
cut -c 7- "joint_state_monitor (copy).cpp" > joint_state_monitor.cpp 
cut -c 7- "kinematic_model_state_monitor (copy).cpp" > kinematic_model_state_monitor.cpp 
cut -c 7- "kinematic_state_constraint_evaluator (copy).cpp" > kinematic_state_constraint_evaluator.cpp 
cut -c 7- "model_utils (copy).cpp" > model_utils.cpp 
cut -c 7- "monitor_utils (copy).cpp" > monitor_utils.cpp 
cut -c 7- "planning_description_configuration_wizard (copy).cpp" > planning_description_configuration_wizard.cpp 
cut -c 7- "planning_monitor (copy).cpp" > planning_monitor.cpp 
cut -c 7- "planning_scene_validity_server (copy).cpp" > planning_scene_validity_server.cpp 
cut -c 7- "remove_object_example (copy).cpp" > remove_object_example.cpp 
cut -c 7- "robot_models (copy).cpp" > robot_models.cpp 
cut -c 7- "test_collision_models (copy).cpp" > test_collision_models.cpp 
cut -c 7- "test_planning_monitor (copy).cpp" > test_planning_monitor.cpp 
cut -c 7- "test_robot_models (copy).cpp" > test_robot_models.cpp 
cut -c 7- "visualize_all_collisions (copy).cpp" > visualize_all_collisions.cpp 
cut -c 7- "visualize_collision_models (copy).cpp" > visualize_collision_models.cpp 
cut -c 7- "visualize_planning_scene (copy).cpp" > visualize_planning_scene.cpp 
cut -c 7- "collision_models (copy).h" > "collision_models.h"  
cut -c 7- "collision_models_interface (copy).h" > collision_models_interface.h 
cut -c 7- "collision_operations_generator (copy).h" > collision_operations_generator.h 
cut -c 7- "collision_space_monitor (copy).h" > collision_space_monitor.h 
cut -c 7- "construct_object (copy).h" > construct_object.h 
cut -c 7- "joint_state_monitor (copy).h" > joint_state_monitor.h 
cut -c 7- "kinematic_model_state_monitor (copy).h" > kinematic_model_state_monitor.h 
cut -c 7- "kinematic_state_constraint_evaluator (copy).h" > kinematic_state_constraint_evaluator.h 
cut -c 7- "model_utils (copy).h" > model_utils.h 
cut -c 7- "monitor_utils (copy).h" > monitor_utils.h 
cut -c 7- "planning_description_configuration_wizard (copy).h" > planning_description_configuration_wizard.h 
cut -c 7- "planning_monitor (copy).h" > planning_monitor.h 
cut -c 7- "robot_models (copy).h" > robot_models.h 
cut -c 7- "" >
cut -c 7- "add_attached_box (copy).py" > add_attached_box.py 
cut -c 7- "add_pole (copy).py" > add_pole.py 
cut -c 7- "fake_time (copy).py" > fake_time.py 
cut -c 7- "test_allowed_collision_operations (copy).py" > test_allowed_collision_operations.py 
cut -c 7- "test_alter_padding (copy).py" > test_alter_padding.py 
cut -c 7- "test_attached_object_collisions (copy).py" > test_attached_object_collisions.py 
cut -c 7- "test_collision_objects (copy).py" > test_collision_objects.py 
cut -c 7- "test_get_base_state_validity (copy).py" > test_get_base_state_validity.py 
cut -c 7- "test_get_current_state_validity (copy).py" > test_get_current_state_validity.py 
catkin_create_pkg planning_models roscpp
cut -c 7- "kinematic_model (copy).h" > kinematic_model.h 
cut -c 7- "kinematic_state (copy).h" > kinematic_state.h 
cut -c 7- "kinematic_model (copy).cpp" > kinematic_model.cpp 
cut -c 7- "kinematic_state (copy).cpp" > kinematic_state.cpp 
cut -c 7- "test_kinematic (copy).cpp" > test_kinematic.cpp 
git clone https://github.com/bulletphysics/bullet3.git
./build_cmake_pybullet_double.sh
rosmsg show object_manipulation_msgs/GraspResult 
catkin build object_manipulator 
roscd object_manipulation_msgs
catkin build object_manipulator 
catkin build object_manipulation_msgs 
catkin build object_manipulator 
catkin build interpolated_ik_motion_planner 
catkin build object_manipulator 
source ~/.bashrc
catkin build object_manipulator 
roscd interpolated_ik_motion_planner
catkin build object_manipulator 
catkin build planning_environment
roscd planning_models
catkin build planning_models 
grep *
echo grep *
sudo apt-get update
sudo apt-get install assimp-utils
catkin build planning_models 
cd workspaces/air_pr2_ws/
catkin build planning_models 
sudo apt-get install libassimp*
sudo apt-get install ros-kinetic-assimp-devel 
catkin build planning_models 
roscd assimp_devel/
ls
nano package.xml 
catkin build planning_models 
git clone https://github.com/assimp/assimp.git
cd assimp/
mkdir build
cd build/
cmake .. -G 'Unix Makefiles'
sudo make -j16
sudo make install
grep - R "bullet"
grep -R "bullet"
roscd assimp_devel/
cd ~/workspaces/air_pr2_ws/
catkin build planning_models 
sudo apt-get install -y assimp-utils
cd ..
catkin build planning_models 
cd /home/brky/workspaces/air_pr2_ws/build/planning_models; catkin build --get-env planning_models | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd ..
catkin build planning_models 
roscd geometric_shapes/
ls
nano cmake/
cd cmake/
ls
nano geometric_shapesConfig.cmake 
cd ..
ls
nano package.xml 
roscd geometric_shapes/
catkin_create_pkg geometric_shapes roscpp
cut -c 7- "bodies (copy).cpp" > bodies.cpp 
cut -c 7- "body_operations (copy).cpp" > body_operations.cpp 
cut -c 7- "shape_operations (copy).cpp" > shape_operations.cpp 
cut -c 7- "test_point_inclusion (copy).cpp" > test_point_inclusion.cpp 
catkin build planning_models 
cut -c 7- "bodies (copy).h" > bodies.h 
cut -c 7- "body_operations (copy).h" > body_operations.h 
cut -c 7- "shape_operations (copy).h" > shape_operations.h 
cut -c 7- "shapes (copy).h" > shapes.h 
grep -R "linear"
grep -R "LinearMath"
cd ..
grep -R "LinearMath"
catkin build geometric_shapes 
cd /home/brky/workspaces/air_pr2_ws/build/geometric_shapes; catkin build --get-env geometric_shapes | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build geometric_shapes 
roscd geometric_shapes/
catkin build planning_models 
sudo apt-get remove ros-kinetic-geometric-shapes 
sudo apt-get clean ros-kinetic-geometric-shapes 
sudo apt-get purge ros-kinetic-geometric-shapes 
catkin clean planning_models 
catkin build planning_models 
gsettings set org.gnome.desktop.background show-desktop-icons 
gsettings set org.gnome.desktop.background show-desktop-icons  true
nautilus -q
nautilus
sudo reboot
gsettings set org.gnome.desktop.background show-desktop-icons true
nautilus -q
sudo nautilus -q
sudo apt install gnome-tweak-tool
gsettings set org.gnome.desktop.background show-desktop-icons 
gsettings set org.gnome.desktop.background show-desktop-icons true
nautilus -q
nautilus
sudo reboot
git clone https://github.com/ormanli/run-length-encoding.git
python brky.py 
python3 code.py 
python code.py 
python brky.py 
python source/Main.py -e images/black\&white/lenna_BW.bmp -s R
python source/Main.py -e "images/black\&white/lenna_BW.bmp" -s R
cd source/
python3 Main.py -e lenna_BW.bmp 
python3 Main.py -e lenna_BW.bmp -s ZZ
python3 Main.py -e lenna_BW.bmp -s RR
python3 Main.py -e lenna_BW.bmp -s R
python3 Main.py -e lenna_BW.bmp -s C
python3 Main.py -e lenna_BW.bmp -s CR
python3 Main.py -e lena_gray.gif -s CR
python3 Main.py -e lenna_8bit.bmp -s CR
python3 Main.py -e lenna_8bit.bmp -s C
python3 Main.py -e lenna_8bit.bmp -s R
python3 Main.py -e lenna_8bit.bmp -s RR
python3 Main.py -e lenna_8bit.bmp -s ZZ
python3 Main.py -e lena_gray.gif -s ZZ
python3 Main.py -e lenna_BW.bmp -s ZZ
python3 Main.py -e filename.bmp -s ZZ
git init
git commit -m "first commit"
git remote add origin git@github.com:brkygokcen/DataCompHW1.git
git add *
git commit -m "first commit"
git push -u origin master
gedit ~/.bashrc
catkin build
catkin clean
catkin build
roslaunch pr2_moveit_config demo.launch
roslaunch moveit_tutorials move_group_interface_tutorial.launch
roslaunch pr2_moveit_tests pr2_plugin_test.launch 
roslaunch pr2_moveit_tests pr2_jacobian_tests.launch 
roslaunch pr2_moveit_config tabletop_object_recognition.launch 
roslaunch pr2_moveit_config demo.launch 
roslaunch manipulation_worlds empty_world.launch 
roslaunch manipulation_worlds pr2_table_object.launch
export ROBOT=sim
roslaunch manipulation_worlds pr2_table_object.launch
killall -9 gzserver
roslaunch manipulation_worlds pr2_table_object.launch
killall -9 gzserver
export ROBOT=sim
roslaunch manipulation_worlds pr2_table_object.launch
killall -9 gzserver
export ROBOT=sim
roslaunch manipulation_worlds pr2_table_object.launch
tmux
gedit ~/.bashrc
roslaunch manipulation_worlds pr2_table_object.launch
rosrun pr2_controller_manager pr2_controller_manager list
roscd pr2_gripper_action/
roscd pr2_mechanism_controllers/
roslaunch pr2_gazebo pr2_table_object.launch
killall -9 gzserver
roslaunch pr2_gazebo pr2_empty_world.launch 
roslaunch pr2_gazebo pr2_table_object.launch
killall -9 gzserver
tmux
catkin clean
catkin build 
catkin clean
catkin build 
cd /home/brky/workspaces/air_pr2_ws/build/pr2_navigation_self_filter; catkin build --get-env pr2_navigation_self_filter | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build 
catkin clean
catkin build 
cd /home/brky/workspaces/air_pr2_ws/build/planning_models; catkin build --get-env planning_models | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin clean
catkin build 
catkin clean
catkin build 
ssh airlab@192.168.66.2
rviz
rosrun rviz rviz
rostopic list
tmux
git clone https://github.com/jsk-ros-pkg/jsk_pr2eus.git
cd ..
catkin build
cd src/
git clone https://github.com/jsk-ros-pkg/jsk_model_tools.git
cd ..
catkin build
cd src/
git clone https://github.com/tork-a/euslisp-release.git -b debian/kinetic/xenial/euslisp
cd ..
catkin build
cd src/
git clone https://github.com/ros-drivers/audio_common.git
cd ..
catkin build
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/eusurdf; catkin build --get-env eusurdf | catkin env -si  /usr/local/bin/cmake /home/brky/workspaces/jsk_ros_pkg_ws/src/jsk_model_tools/eusurdf --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/jsk_ros_pkg_ws/devel/.private/eusurdf -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/jsk_ros_pkg_ws/install; cd -
cd src/
git clone https://github.com/jsk-ros-pkg/jsk_roseus.git 
cd ..
catkin build jsk_roseus 
cd src; git clone https://github.com/tork-a/jskeus-release.git -b debian/kinetic/xenial/euslisp
cd src; git clone https://github.com/tork-a/jskeus-release.git -b debian/kinetic/xenial/jskeus
cd ..
catkin build jsk_roseus 
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/roseus; catkin build --get-env roseus | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build jsk_roseus 
catkin build
acd /home/brky/workspaces/jsk_ros_pkg_ws/build/audio_play; catkin build --get-env audio_play | catkin env -si  /usr/local/bin/cmake /home/brky/workspaces/jsk_ros_pkg_ws/src/audio_common/audio_play --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/jsk_ros_pkg_ws/devel/.private/audio_play -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/jsk_ros_pkg_ws/install; cd -
sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-pulseaudio
cd ..
catkin build
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/audio_capture; catkin build --get-env audio_capture | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/audio_play; catkin build --get-env audio_play | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
sudo apt-get install libgstreamer-plugins-base1.0-dev 
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/audio_play; catkin build --get-env audio_play | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
gedit ~/.bashrc
mkdir -p ./jsk_ros_pkg_ws/src
cd jsk_ros_pkg_ws/
catkin build
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
cd workspaces/jsk_ros_pkg_ws/src/
git clone https://github.com/jsk-ros-pkg/jsk_recognition.git
cd ..
catkin build
cd src/
git clone https://github.com/jsk-ros-pkg/jsk_common_msgs.git
cd ..
catkin build
cd src/
git clone https://github.com/jsk-ros-pkg/jsk_common.git
cd .
cd ..
catkin build
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/jsk_recognition_utils; catkin build --get-env jsk_recognition_utils | catkin env -si  /usr/local/bin/cmake /home/brky/workspaces/jsk_ros_pkg_ws/src/jsk_recognition/jsk_recognition_utils --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/jsk_ros_pkg_ws/devel/.private/jsk_recognition_utils -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/jsk_ros_pkg_ws/install; cd -
pip install Cython
pip install --user Cython
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/jsk_recognition_utils; catkin build --get-env jsk_recognition_utils | catkin env -si  /usr/local/bin/cmake /home/brky/workspaces/jsk_ros_pkg_ws/src/jsk_recognition/jsk_recognition_utils --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/jsk_ros_pkg_ws/devel/.private/jsk_recognition_utils -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/jsk_ros_pkg_ws/install; cd -
catkin build
cd src/
git clone https://github.com/OctoMap/octomap_mapping.git
cd ..
catkin build
cd src/
git clone https://github.com/OctoMap/octomap_ros.git -b kinetic-devel
cd ..
catkin build
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/imagesift; catkin build --get-env imagesift | catkin env -si  /usr/local/bin/cmake /home/brky/workspaces/jsk_ros_pkg_ws/src/jsk_recognition/imagesift --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/jsk_ros_pkg_ws/devel/.private/imagesift -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/jsk_ros_pkg_ws/install; cd -
cd src/
git clone https://github.com/jsk-ros-pkg/jsk_3rdparty.git
cd ..
catkin build
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/downward; catkin build --get-env downward | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
sudo apt-get install flex
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/downward; catkin build --get-env downward | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
sudo apt-get install bison
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/downward; catkin build --get-env downward | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/ff; catkin build --get-env ff | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/collada_urdf_jsk_patch; catkin build --get-env collada_urdf_jsk_patch | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd src/
git clone https://github.com/ros/collada_urdf.git
catkin build collada_urdf
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/collada_urdf; catkin build --get-env collada_urdf | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
git clone https://github.com/ros/collada_urdf.git -b indigo-devel
catkin build collada_urdf
git clone https://github.com/ros/collada_urdf.git 
cd .
cd ..
catkin build
cd src/
git clone https://github.com/locusrobotics/catkin_virtualenv.git
cd ..
catkin build catkin_virtualenv 
catkin build
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/test_catkin_virtualenv_py3_isolated; catkin build --get-env test_catkin_virtualenv_py3_isolated | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
pip install --user packaging
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/collada_urdf; catkin build --get-env collada_urdf | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/jskeus; catkin build --get-env jskeus | catkin env -si  /usr/bin/make install; cd -
catkin build
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/test_catkin_virtualenv; catkin build --get-env test_catkin_virtualenv | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/sesame_ros; catkin build --get-env sesame_ros | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/collada_urdf; catkin build --get-env collada_urdf | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/julius; catkin build --get-env julius | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
clear
catkin build
clear
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/collada_urdf; catkin build --get-env collada_urdf | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd src/
git clone https://github.com/ros-planning/geometric_shapes.git -b kinetic-devel
catkin build geometric_shapes 
source ~/.bashrc
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/collada_urdf; catkin build --get-env collada_urdf | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
htop
ssh airlab@192.168.66.2
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
catkin build
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/sesame_ros; catkin build --get-env sesame_ros | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/collada_urdf; catkin build --get-env collada_urdf | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
sudo apt-get install ros-kinetic-collada-urdf
sudo apt-get remove ros-kinetic-collada-urdf
catkin clean
catkin build
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/test_catkin_virtualenv; catkin build --get-env test_catkin_virtualenv | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/collada_urdf_jsk_patch; catkin build --get-env collada_urdf_jsk_patch | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/jskeus; catkin build --get-env jskeus | catkin env -si  /usr/bin/make install; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/sesame_ros; catkin build --get-env sesame_ros | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
echo $PYTHONPATH
pip install virtualenv
pip install --user virtualenv
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/sesame_ros; catkin build --get-env sesame_ros | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
cd src/
git clone https://github.com/ros-perception/opencv_apps.git 
catkin build opencv_apps 
source ~/.bashrc
cd ..
catkin build
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/jsk_perception; catkin build --get-env jsk_perception | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
pip install --user gdown
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/jsk_perception; catkin build --get-env jsk_perception | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
git clone https://github.com/wg-perception/people.git -b kinetic
cd ..
catkin build people
cd src/
git clone https://github.com/DLu/wu_ros_tools.git -b kinetic
cd ..
catkin build people
roslaunch pr2_gazebo pr2_table_object.launch
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
htop
git clone https://github.com/jsk-ros-pkg/jsk_visualization.git
cd ..
catkin build
catkin build jsk_perception 
catkin build 
cd src/
git clone https://github.com/ros-visualization/view_controller_msgs.git -b hydro-devel
cd ..
catkin build 
rviz -d $(rospack find pr2eus_tutorials)/config/pr2_tabletop.rviz
rosrun jsk_robot_utils marker_msg_from_indigo_to_kinetic.py
rosrun topic_tools relay /bounding_box_interactive_marker/kinetic/feedback  /bounding_box_interactive_marker/feedback
rosrun pr2eus_tutorials pr2-tabletop-object-grasp.l
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
git clone https://github.com/ros-interactive-manipulation/tabletop_object_perception.git 
cd ..
catkin build 
rosrun pr2eus_tutorials pr2-tabletop-object-grasp.l
roscd pr2eus_tutorials/euslisp
roseus reach-object.l
roslaunch jsk_pcl_ros hsi_color_filter.launch INPUT:=/wide_stereo/points2 h_min:=75 s_min:=50
rviz -d `rospack find pr2eus_tutorials`/config/pr2_reach_object.rviz
roslaunch pr2eus_tutorials spawn_objects.launch
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
roslaunch pr2_gazebo pr2_empty_world.launch
rosrun pr2eus_tutorials pr2-tabletop-object-grasp.l
roslaunch tabletop_object_detector tabletop_object_recognition.launch 
roslaunch tabletop_object_detector tabletop_segmentation.launch 
roslaunch pr2eus_tutorials pr2_tabletop.launch
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch 
tmux
roscd tabletop_object_detector
cd ..
catkin clean tabletop_object_detector
rospack find tabletop_object_detector
catkin build 
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/roseus; catkin build --get-env roseus | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/jsk_pcl_ros; catkin build --get-env jsk_pcl_ros | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build 
roscd tabletop_object_detector
gedit ~/.bashrc
rospack find tabletop_object_detector
cd ..
catkin clean
source ~/.bashrc
rospack find tabletop_object_detector
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
rospack find tabletop_object_detector
roscd tabletop_object_detector
git clone https://github.com/PR2/pr2_simulator.git
cd ..
catkin build
cd src/
git clone https://github.com/PR2/pr2_mechanism.git -b kinetic-devel
cd ..
catkin build
cd src/
git clone https://github.com/PR2/pr2_common.git -b kinetic-devel
cd ..
catkin build
cd src/
git clone https://github.com/ros/convex_decomposition.git -b kinetic-devel
cd ..
catkin build
cd src/
git clone https://github.com/ros/ivcon.git -b kinetic-devel
cd ..
catkin build
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/jsk_pcl_ros; catkin build --get-env jsk_pcl_ros | catkin env -si  /usr/bin/make cmake_check_build_system; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/jsk_pcl_ros; catkin build --get-env jsk_pcl_ros | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/euslisp; catkin build --get-env euslisp | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/julius; catkin build --get-env julius | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/ivcon; catkin build --get-env ivcon | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/rosping; catkin build --get-env rosping | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/jskeus; catkin build --get-env jskeus | catkin env -si  /usr/bin/make install; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/pr2_controller_manager; catkin build --get-env pr2_controller_manager | catkin env -si  /usr/local/bin/cmake /home/brky/workspaces/jsk_ros_pkg_ws/src/pr2_mechanism/pr2_controller_manager --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/jsk_ros_pkg_ws/devel/.private/pr2_controller_manager -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/jsk_ros_pkg_ws/install; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/jsk_perception; catkin build --get-env jsk_perception | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/pr2eus; catkin build --get-env pr2eus | catkin env -si  /usr/bin/make cmake_check_build_system; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/pr2eus_moveit; catkin build --get-env pr2eus_moveit | catkin env -si  /usr/bin/make cmake_check_build_system; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/pr2eus; catkin build --get-env pr2eus | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/jskeus; catkin build --get-env jskeus | catkin env -si  /usr/bin/make install; cd -
git clone https://github.com/PR2/pr2_controllers.git -b kinetic-devel
cd ..
catkin build
grep -R "pr2_tabletop_object_detector"
grep -R "segmentation_decomposer/boxes"
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
source ~/.bashrc
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
source ~/.bashrc
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
source ~/.bashrc
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
killall -9 gzserver
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
killall -9 gzserver
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
killall -9 gzserver
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
killall -9 gzserver
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
rostopic info /segmentation_decomposer/boxes
rviz -d $(rospack find pr2eus_tutorials)/config/pr2_tabletop.rviz
rosrun pr2eus_tutorials pr2-tabletop-object-grasp.l
tmux
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
roslaunch pr2eus_tutorials pr2_tabletop_grasp_sim.launch
export ROBOT=sim
roslaunch pr2eus_tutorials pr2_tabletop_grasp_sim.launch
rostopic info /bounding_box_marker/selected_box
rostopic echo /bounding_box_marker/selected_box
rostopic info /grasp_state /
rostopic info /grasp_state 
rostopic echo /grasp_state 
grep -R "wait-interpolation debug"
rosrun pr2eus_tutorials pr2-tabletop-object-grasp.l
rviz -d $(rospack find pr2eus_tutorials)/config/pr2_tabletop.rviz
clear
rosservice list
rosservice info /bounding_box_marker/get_loggers 
rosservice info /bounding_box_marker/set_logger_level 
rosservice call /bounding_box_marker/set_logger_level "logger: ''
level: ''" 
rosservice call /bounding_box_marker/set_logger_level "logger: ''
level: 'VERBOSE'" 
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
tmux
./pr2_tabletop_grasp_sim.sh 
exit
docker ps
docker images
./jsk_recognition/docker/build.sh
sudo ./jsk_recognition/docker/build.sh
rosservice call /bounding_box_marker/set_logger_level "logger: ''
ssh airlab@192.168.66.2
roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
roseus tabletop-sample.l
rosrun pr2eus_tutorials pr2main.l 
rosrun pr2eus_tutorials tabletop-sample.l 
clear
rostopic pub /ObjectDetection posedetection_msgs/ObjectDetection "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '/base_link'
objects:
- pose:
    position:
      x: 0.4
      y: 0.1
      z: 0.6
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  type: ''" -1
rostopic pub /ObjectDetection posedetection_msgs/ObjectDetection "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '/base_link'
objects:
- pose:
    position:
      x: 0.4
      y: 0.1
      z: 0.6
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  type: ''" -1
rostopic pub /ObjectDetection posedetection_msgs/ObjectDetection "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '/base_link'
objects:
- pose:
    position:
      x: 0.4
      y: 0.1
      z: 0.3
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  type: ''" -1
rostopic pub /ObjectDetection posedetection_msgs/ObjectDetection "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '/base_link'
objects:
- pose:
    position:
      x: 0.4
      y: 0.2
      z: 0.6
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  type: ''" -1
rostopic pub /ObjectDetection posedetection_msgs/ObjectDetection "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '/base_link'
objects:
- pose:
    position:
      x: 0.4
      y: -0.2
      z: 0.4
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  type: ''" -1
rostopic pub /ObjectDetection posedetection_msgs/ObjectDetection "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '/base_link'
objects:
- pose:
    position:
      x: 0.4
      y: 0.2
      z: 0.6
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  type: ''" -1
rostopic pub /ObjectDetection posedetection_msgs/ObjectDetection "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '/base_link'
objects:
- pose:
    position:
      x: 0.4
      y: 0.2
      z: 0.6
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  type: ''" -1
rostopic pub /ObjectDetection posedetection_msgs/ObjectDetection "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '/base_link'
objects:
- pose:
    position:
      x: 0.4
      y: 0.1
      z: 0.3
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  type: ''" -1
rostopic list
rostopic echo /r_arm_controller/joint_trajectory_action/result
tmux
rosrun pr2eus_tutorials pr2-tabletop-object-grasp.l
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
rospack find pr2_interactive_manipulation
rosrun pr2eus_tutorials pr2-tabletop-object-grasp.l
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
htop
roslaunch pr2eus_tutorials spawn_objects.launch
rosservice list
roslaunch jsk_pcl_ros hsi_color_filter.launch INPUT:=/wide_stereo/points2 h_min:=75 s_min:=50
roscd pr2eus_tutorials/euslisp
roseus reach-object.l
rviz -d `rospack find pr2eus_tutorials`/config/pr2_reach_object.rviz
roslaunch pr2_gazebo pr2_empty_world.launch
tmux
htop
export ROBOT=sim
roslaunch pr2_interactive_manipulation pr2_interactive_manipulation_robot sim:=true
ssh airlab@192.168.66.2
ifconfig
ssh airlab@192.168.66.2
ifconfig
git clone https://github.com/scelesticsiva/Neural-Networks-for-Image-Compression.git
python3
python
htop
catkin_clean
catkin_debug
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/test_catkin_virtualenv_py3_isolated; catkin build --get-env test_catkin_virtualenv_py3_isolated | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin_debug
rosrun pr2eus_tutorials pr2-tabletop-object-grasp.l
rviz -d $(rospack find pr2eus_tutorials)/config/pr2_tabletop.rviz
tmux
grep -R "/bounding_box_marker/selected_box"
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
tmux
grep -R "/segmentation_decomposer/boxes"
rosrun pr2eus_tutorials pr2-tabletop-object-grasp.l
rostopic info /bounding_box_marker/selected_box
rosnode info /bounding_box_marker 
rostopic info /segmentation_decomposer/boxes
rostopic info /bounding_box_interactive_marker/feedback
rostopic info /segmentation_decomposer/boxes
grep -R "new"
grep -R "= new"
catkin_debug
catkin_debug jsk_pcl_ros
catkin_debug
rosrun pr2eus_tutorials pr2-tabletop-object-grasp.l
killall -9 gzserver
./pr2_tabletop_grasp_sim.sh 
killall -9 gzserver
killall -9 gzclient
./pr2_tabletop_grasp_sim.sh 
killall -9 gzclient
killall -9 gzserver
./pr2_tabletop_grasp_sim.sh 
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
./pr2_tabletop_grasp_sim.sh 
tmux
grep -R "jsk_pcl_ros::OrganizedMultiPlaneSegmentation::segmentFromNormals"
(wait-for-grasp-target)
roslaunch pr2_gazebo pr2_empty_world.launch
tmux
roseus tabletop-sample.l
rostopic pub /ObjectDetection posedetection_msgs/ObjectDetection "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '/base_link'
objects:
- pose:
    position:
      x: 0.4
      y: 0.1
      z: 0.6
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  type: ''" -1
rostopic pub /ObjectDetection posedetection_msgs/ObjectDetection "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '/base_link'
objects:
- pose:
    position:
      x: 0.4
      y: 0.1
      z: 0.6
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  type: ''" -1
rostopic info /ObjectDetection posedetection_msgs/ObjectDetection
rostopic info /ObjectDetection 
rosnode info /pr2_main_1586344533460877134 
roseus '/home/brky/workspaces/jsk_ros_pkg_ws/src/jsk_pr2eus/pr2eus_tutorials/euslisp/tabletop-sample.l' 
grep -R "/pr2_main_1586344533460877134/set_logger_level"
grep -R "/j_robotsound"
grep -R "j_robotsound"
roseus pr2-tabletop-demo.l 
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
roslaunch pr2eus_tutorials pr2_
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
mkdir -p ~/workspaces/udacity_ws/src
cd workspaces/udacity_ws/src/
git clone https://github.com/udacity/RoboND-Perception-Project.git
cd ..
catkin_debug
cd src/
git clone https://github.com/ros-planning/moveit_visual_tools.git -b kinetic-devel
cd .
cd ..
catkin_debug
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/models:$GAZEBO_MODEL_PATH
source ~/workspaces/udacity_ws/devel/setup.bash 
cd src/RoboND-Perception-Project/pr2_robot/scripts/
chmod u+x pr2_safe_spawner.sh 
./pr2_safe_spawner.sh 
catkin_clean
./pr2_safe_spawner.sh 
catkin_make
cd src/
git clone https://github.com/ros-planning/moveit_visual_tools.git -b kinetic-devel
cd ..
catkin_make
git clone https://github.com/mkhuthir/RoboND-Perception-Project.git
rostopic info /sensor_stick/point_cloud 
rostopic hz /sensor_stick/point_cloud 
rosrun sensor_stick object_recognition.py 
pip install --user sklearn
rosrun sensor_stick object_recognition.py 
pip install --user pcl
sudo apt-get install python-pcl
pip install --user python-pcl
rosrun sensor_stick object_recognition.py 
./pr2_safe_spawner.sh 
roslaunch pr2_robot pick_place_project.launch
roslaunch sensor_stick robot_spawn.launch
grep -R "sensor_stick/point_cloud"
rosrun sensor_stick object_recognition.py 
roslaunch sensor_stick robot_spawn.launch
rostopic info pr2/world/points
grep -R "def ros_to_pcl"
rosrun pr2_robot project.py 
tmux
roslaunch pr2_robot pick_place_project.launch 
roseus tabletop-sample.l
rosrun sensor_stick object_recognition.py 
tmux
roslaunch pr2_robot pick_place_project.launch
roslaunch sensor_stick robot_spawn.launch
./pr2_safe_spawner.sh 
killall -9 gzserver
killall -9 gzclient
rosrun pr2_controller_manager pr2_controller_manager list
clear
roslaunch pr2_robot pick_place_demo.launch
killall -9 gzserver
roslaunch pr2_robot pick_place_demo.launch
killall -9 gzserver
roslaunch pr2_robot pick_place_demo.launch
roslaunch pr2_moveit pr2_moveit.launch
rosnode list
rosrun pr2_robot pr2_motion
rosnode llisr
rosnode list
rosnode info /pr2_motion 
htop
roslaunch pr2_moveit pr2_moveit.launch
rosrun pr2_robot pr2_motion
roslaunch pr2_robot pick_place_demo.launch
roslaunch pr2_robot pick_place_project.launch 
rosnode list
tmux

clear
killall -9 gzserver
clear
tmux
roscd moveit_ros_move_group/
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin_build
catkin build
catkin_make
roslaunch pr2_moveit pr2_moveit.launch
rosrun pr2_robot pr2_motion
rostopic list
roslaunch pr2_robot pick_place_project.launch 

killall -9 gzserver
roslaunch pr2_robot pick_place_demo.launch
killall -9 gzserver
roslaunch pr2_robot pick_place_demo.launch
tmux
mkdir -p /fjnunes_ws/src
mkdir -p ./fjnunes_ws/src
cd fjnunes_ws/
catkin_make
git clone https://github.com/fjnunes/RoboND-Perception-Project.git
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin_make
catkin clean 
catkin_clean 
catkin build
catkin_clean 
git clone https://github.com/ros-planning/moveit.git -b kinetic-devel
cd ..
catkin_clean 
catkin build
catkin_make
gedit ./bashrc
gedit ~/.bashrc
cd src/
catkin_clean 
cd ..
catkin_clean --workspace fjnunes_ws
catkin_make
roslaunch pr2_robot pick_place_project.launch 
cd ..
catkin_make
roscd pr2_robot
tmux
roslaunch pr2_robot pick_place_project.launch 
source ~/.bashrc
roslaunch pr2_robot pick_place_project.launch 
./pr2_safe_spawner.sh 
wget http://download.qt.io/official_releases/qt/5.7/5.7.0/qt-opensource-linux-x64-5.7.0.run
chmod +x qt-unified-linux-x64-3.2.2-online.run 
./qt-unified-linux-x64-3.2.2-online.run 
sudo add-apt-repository ppa:levi-armstrong/qt-libraries-xenial
sudo add-apt-repository ppa:levi-armstrong/ppa
sudo apt-get update && sudo apt-get install qt57creator-plugin-ros libqtermwidget57-0-dev
sudo apt-get update && sudo apt-get install qt57creator-plugin-ros libqtermwidget57-0-dev libqtermwidget57-0
sudo apt-get update && sudo apt-get install qt57creator-plugin-ros libqtermwidget57-0-dev libqtermwidget57-0 qtermwidget57-data libqtermwidget59-0
sudo apt-get update && sudo apt-get install qt57creator-plugin-ros libqtermwidget57-0-dev libqtermwidget57-0 qtermwidget57-data libqtermwidget59-0 qtermwidget59-data
sudo apt-get install qtermwidget59-data
sudo apt-get install qt57creator-plugin-ros libqtermwidget57-0-dev
sudo apt-get install libqtermwidget57-0
sudo apt-get install qtermwidget57-data
sudo apt-get install libqtermwidget57-0
sudo apt-get install qt57creator-plugin-ros libqtermwidget57-0-dev
sudo apt-get install libqtermwidget59-0
sudo apt-get install qt57creator-plugin-ros libqtermwidget57-0-dev
sudo apt-get install qt57creator-plugin-ros 
sudo apt-get install libqtermwidget57-0-dev
sudo apt-get install libqtermwidget57-0
sudo apt-get install qtermwidget57-data
sudo apt-get install libqtermwidget57-0
sudo apt-get install libqtermwidget57-0-dev
sudo apt-get install qt57creator-plugin-ros libqtermwidget57-0-dev
sudo apt-get install libqtermwidget57-0-dev
sudo gedit /usr/lib/x86_64-linux-gnu/qt-default/qtchooser/default.conf 
git clone -b master https://github.com/BRKY/ros_qtc_plugins.git
./qtcreator-ros-xenial-latest-online-installer.run 
qtcreator-ros 
sudo gedit /etc/sysctl.d/10-ptrace.conf
sudo systemctl restart procps.service
sudo apt-get install clang-format-6.0
git clone https://github.com/Levi-Armstrong/gdb-7.7.1.git
cd gdb-7.7.1/
ls
./configure 
make
sudo checkinstall 
roscore
sudo apt install ppa-purge
sudo ppa-purge -o beineri
sudo ppa-purge levi-armstrong/qt-libraries-xenial
sudo ppa-purge levi-armstrong/ppa
./qtcreator-ros-xenial-latest-online-installer.run 
./qtcreator-ros-xenial-latest-online-installer.run --uninstall
./qtcreator-ros-xenial-latest-online-installer.run
qtcreator-ros
./MaintenanceTool 
htop
htop
./configure
make
sudo make install
qtdemo
./configure --help
./configure --reconf
echo $PATH
sudo make uninstall
./qt-creator-opensource-linux-x86_64-4.9.2.run 
PATH=/usr/local/Trolltech/Qt-4.8.7/bin:$PATH
export PATH
qtdeom
qtdemo
qtchooser 
qtconfig
qt3to4 
qt-faststart 
qtdemo 
./qtcreator-ros-xenial-latest-online-installer.run 
sudo reboot
./MaintenanceTool
sudo apt install ppa-purge
sudo ppa-purge -o beineri
sudo ppa-purge levi-armstrong/qt-libraries-xenial
sudo ppa-purge levi-armstrong/ppa
sudo apt install ppa-purge
sudo ppa-purge -o beineri
sudo ppa-purge levi-armstrong/qt-libraries-xenial
sudo ppa-purge levi-armstrong/ppa
qtcreator-ros 
catkin build 
catkin_make
./qtcreator-ros-xenial-latest-online-installer.run 
./pr2_safe_spawner.sh 
catkin_make
./pr2_safe_spawner.sh 
./qtcreator-ros-xenial-latest-online-installer.run 
history
./qtcreator-ros-xenial-latest-online-installer.run --uninstall
./pr2_safe_spawner.sh 
catkin_make
gedit ~/.bashrc
./pr2_safe_spawner.sh 
./pr2_safe_spawner.sh 
catkin_make
gedit ~/.bashrc
./pr2_safe_spawner.sh 
qtcreator-ros 
./MaintenanceTool 
sudo add-apt-repository ppa:levi-armstrong/qt-libraries-xenial
sudo apt install qt
sudo apt install qt57creator-plugin-ros*
sudo apt install qt57creator-plugin-ros
qtcreator-ros 
qtcreator-ros 
git clone https://github.com/ros-industrial/ros_qtc_plugin.git
catkin build
catkin_make
catkin build
catkin_make
gsettings set com.canonical.Unity.Launcher launcher-position Right
gsettings set com.canonical.Unity.Launcher launcher-position Buttom
gsettings set com.canonical.Unity.Launcher launcher-position Right
gsettings set com.canonical.Unity.Launcher launcher-position Buttom
qtcreator-ros 
qtcreator-ros 
gsettings set com.canonical.Unity.Launcher launcher-position Bottom
gsettings set com.canonical.Unity.Launcher launcher-position Right
gsettings set com.canonical.Unity.Launcher launcher-position Left
./pr2_safe_spawner.sh 
grep -R "world_joint_controller/command"
grep -R "command"
grep -R "world_joint_controller"
clear
grep -R "world_joint_controller"
roscd controller_manager
cd ~/workspaces/fjnunes_ws/src/
grep -R "world_joint_controller"
qtcreator-ros 
htop
rosrun pr2_robot pr2_motion 
./pr2_safe_spawner.sh 
rostopic info /pr2/world_joint_controller/command 
rostopic info /pr2/world_joint_controller/command -r
rostopic info /pr2/world_joint_controller/command -n 1
rostopic info --help
rostopic info -h
rostopic -h
rostopic info -h
rosclean
rosclean --purge
rosclean -h
rosclean purge
roscd pcl_ros/
ls
cd plugins/
ls
cd nodelet/
ls
rostopic info /pr2/world_joint_controller/command -n 1
rostopic info /pr2/world_joint_controller/command 
./pr2_safe_spawner.sh 
roscd moveit
grep -R "movegroupinterface"
grep -R "move_group_interface"
grep -R "move_action_server"
lsb_release -rs | sed 's/\.//'
./install-ompl-ubuntu.sh --python
roscd ompl/
source ~/workspaces/move_it_ws/devel/setup.bash 
roscd ompl/
./install-ompl-ubuntu.sh --python
sudo make install
catkin_make
roscd ompl/
catkin_make
roscd ompl/
sudo apt-get remove ros-kinetic-ompl 
git clone https://github.com/ompl/ompl
cd ompl/
wget https://raw.githubusercontent.com/ros-gbp/ompl-release/debian/kinetic/xenial/ompl/package.xml
cd ..
catkin_make
catkin build
catkin_clean
catkin build
cd /home/brky/workspaces/fjnunes_ws/build/moveit_planners_ompl; catkin build --get-env moveit_planners_ompl | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
htop
grep -R "++11"
grep -R "++14"
catkin_clean
catkin build
cd /home/brky/workspaces/fjnunes_ws/build/moveit_core; catkin build --get-env moveit_core | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
git clone https://github.com/ompl/ompl -b kinetic-devel
cd ompl/
wget https://raw.githubusercontent.com/ros-gbp/ompl-release/debian/kinetic/xenial/ompl/package.xml
cd ..
catkin build ompl
catkin build 
roscd ompl/
catkin build
cd /home/brky/workspaces/fjnunes_ws/build/moveit_planners_ompl; catkin build --get-env moveit_planners_ompl | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd src/
cd /home/brky/workspaces/fjnunes_ws/build/moveit_planners_ompl; catkin build --get-env moveit_planners_ompl | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin_clean
catkin_make
catkin_clean
catkin build
roscd ompl/
cd ..
catkin build
catkin_clean
catkin_make
catkin_make_isolated 
roscd ompl/
catkin build
./pr2_safe_spawner.sh 
catkin_clean 
catkin_clean
catkin_clean 
catkin_debug 
./pr2_safe_spawner.sh 
grep -R "Robot model parameter not found! Did you "
rosrun moveit_ros_move_group move_group 
roscore
roslaunch pr2_moveit pr2_moveit.launch 
roslaunch pr2_moveit move_group.launch 
roslaunch pr2_moveit pr2_moveit.launch 
roslaunch pr2_robot pick_place_demo.launch 
tmux
grep -R "You can start planning now!"
roslaunch pr2_moveit pr2_moveit.launch 
rosrun moveit_ros_move_group move_group 
roslaunch pr2_robot pick_place_demo.launch 
roscd moveit
cd ..
find -h
find --help
find robot_model*
find FILE robot_model*
find  robot_model_loader.h
rosrun moveit_ros_move_group move_group 
tmux
rosrun moveit_ros_move_group move_group 
roslaunch pr2_robot pick_place_demo.launch 
qtcreator-ros 
sudo apt-get remove ros-kinetic-moveit-core
grep -R "collision_detector"
grep -R "getSensorsList"
rosrun pr2_robot pr2_motion
rosrun moveit_ros_move_group move_group 
roslaunch pr2_moveit move_group.launch 
roslaunch pr2_moveit pr2_moveit.launch
roslaunch pr2_robot pick_place_demo.launch 
tmux
tmux
./pr2_safe_spawner.sh 
source ~/.bashrc
./pr2_safe_spawner.sh 
source ~/.bashrc
./pr2_safe_spawner.sh 
source ~/.bashrc
./pr2_safe_spawner.sh 
rosclean purge
./pr2_safe_spawner.sh 
catkin_clean
catkin build
source ~/.bashrc
./pr2_safe_spawner.sh 
./pr2_safe_spawner.sh 
catkin_clean 
catkin build
./pr2_safe_spawner.sh 
catkin_clean 
rosclean purge
grep -R "brkygkcn"
catkin build
./pr2_safe_spawner.sh 
catkin build
./pr2_safe_spawner.sh 
catkin_clean 
./pr2_safe_spawner.sh 
pip install coursera-dl
pip install --user coursera-dl
coursera-dl convolutional-neural-networks
coursera-dl --help
coursera-dl -u berkay.gokcen@altinay.com -p BERkay*389* convolutional-neural-networks
coursera-dl convolutional-neural-networks
pip uninstall coursera-dl
pip3 install --user coursera-dl
coursera-dl convolutional-neural-networks
coursera-dl -u berkay.gokcen@altinay.com -p BERkay*389* convolutional-neural-networks
coursera-dl -u berkay.gokcen@altinay.com -p BERkay*389* 
coursera-dl -u berkay.gokcen@altinay.com -p BERkay*389*  --cauth
$cauth = p_gS88tV2IG4wm3tJOnwXlVyPEY0-ISzJL3KEc_7mpX2kHALL7JS8HvTM1NKZ1CZ0VOvtojrAUcf7SQAEk1dzA.i6IaCtXgTeT6K_grx208VA.uH42UKsfSR9Cino1PzGqHJUlIS8mQZorDM1MSGV0I2gMx0B6vlikxT9-rPPpeL9dGzpwYk-PNLpgMLBHPLKwU2UKwFEVd25CBadE9dX74esC6rS_UmRMNzqvokUQcc1tdwV6qcKxLuS7bIzwpmsn0ROvPChE6B2rV4xxhyu_h4KLfGMYGiwLit6qpTqvPbee
cauth = p_gS88tV2IG4wm3tJOnwXlVyPEY0-ISzJL3KEc_7mpX2kHALL7JS8HvTM1NKZ1CZ0VOvtojrAUcf7SQAEk1dzA.i6IaCtXgTeT6K_grx208VA.uH42UKsfSR9Cino1PzGqHJUlIS8mQZorDM1MSGV0I2gMx0B6vlikxT9-rPPpeL9dGzpwYk-PNLpgMLBHPLKwU2UKwFEVd25CBadE9dX74esC6rS_UmRMNzqvokUQcc1tdwV6qcKxLuS7bIzwpmsn0ROvPChE6B2rV4xxhyu_h4KLfGMYGiwLit6qpTqvPbee
$cauth := p_gS88tV2IG4wm3tJOnwXlVyPEY0-ISzJL3KEc_7mpX2kHALL7JS8HvTM1NKZ1CZ0VOvtojrAUcf7SQAEk1dzA.i6IaCtXgTeT6K_grx208VA.uH42UKsfSR9Cino1PzGqHJUlIS8mQZorDM1MSGV0I2gMx0B6vlikxT9-rPPpeL9dGzpwYk-PNLpgMLBHPLKwU2UKwFEVd25CBadE9dX74esC6rS_UmRMNzqvokUQcc1tdwV6qcKxLuS7bIzwpmsn0ROvPChE6B2rV4xxhyu_h4KLfGMYGiwLit6qpTqvPbee
$cauth=p_gS88tV2IG4wm3tJOnwXlVyPEY0-ISzJL3KEc_7mpX2kHALL7JS8HvTM1NKZ1CZ0VOvtojrAUcf7SQAEk1dzA.i6IaCtXgTeT6K_grx208VA.uH42UKsfSR9Cino1PzGqHJUlIS8mQZorDM1MSGV0I2gMx0B6vlikxT9-rPPpeL9dGzpwYk-PNLpgMLBHPLKwU2UKwFEVd25CBadE9dX74esC6rS_UmRMNzqvokUQcc1tdwV6qcKxLuS7bIzwpmsn0ROvPChE6B2rV4xxhyu_h4KLfGMYGiwLit6qpTqvPbee
echo $cauth
echo cauth
export cauth = p_gS88tV2IG4wm3tJOnwXlVyPEY0-ISzJL3KEc_7mpX2kHALL7JS8HvTM1NKZ1CZ0VOvtojrAUcf7SQAEk1dzA.i6IaCtXgTeT6K_grx208VA.uH42UKsfSR9Cino1PzGqHJUlIS8mQZorDM1MSGV0I2gMx0B6vlikxT9-rPPpeL9dGzpwYk-PNLpgMLBHPLKwU2UKwFEVd25CBadE9dX74esC6rS_UmRMNzqvokUQcc1tdwV6qcKxLuS7bIzwpmsn0ROvPChE6B2rV4xxhyu_h4KLfGMYGiwLit6qpTqvPbee
export CAUTH=p_gS88tV2IG4wm3tJOnwXlVyPEY0-ISzJL3KEc_7mpX2kHALL7JS8HvTM1NKZ1CZ0VOvtojrAUcf7SQAEk1dzA.i6IaCtXgTeT6K_grx208VA.uH42UKsfSR9Cino1PzGqHJUlIS8mQZorDM1MSGV0I2gMx0B6vlikxT9-rPPpeL9dGzpwYk-PNLpgMLBHPLKwU2UKwFEVd25CBadE9dX74esC6rS_UmRMNzqvokUQcc1tdwV6qcKxLuS7bIzwpmsn0ROvPChE6B2rV4xxhyu_h4KLfGMYGiwLit6qpTqvPbee
echo $CAUTH
coursera-dl -u berkay.gokcen@altinay.com -p BERkay*389* -ca $CAUTH convolutional-neural-networks
coursera-dl -u berkay.gokcen@altinay.com -p BERkay*389* -ca $CAUTH "convolutional-neural-networks"
coursera-dl -u berkay.gokcen@altinay.com -p BERkay*389* -ca $CAUTH 
coursera-dl convolutional-neural-networks
coursera-dl
coursera-dl --list-courses
coursera-dl -u berkay.gokcen@altinay.com -p BERkay*389* -ca $CAUTH 
export CAUTH=p_gS88tV2IG4wm3tJOnwXlVyPEY0-ISzJL3KEc_7mpX2kHALL7JS8HvTM1NKZ1CZ0VOvtojrAUcf7SQAEk1dzA.i6IaCtXgTeT6K_grx208VA.uH42UKsfSR9Cino1PzGqHJUlIS8mQZorDM1MSGV0I2gMx0B6vlikxT9-rPPpeL9dGzpwYk-PNLpgMLBHPLKwU2UKwFEVd25CBadE9dX74esC6rS_UmRMNzqvokUQcc1tdwV6qcKxLuS7bIzwpmsn0ROvPChE6B2rV4xxhyu_h4KLfGMYGiwLit6qpTqvPbee
coursera-dl -u berkay.gokcen@altinay.com -p BERkay*389* -ca $CAUTH 
coursera-dl -u berkay.gokcen@altinay.com -p BERkay*389* -ca $CAUTH "convolutional-neural-networks"
coursera-dl -u berkay.gokcen@altinay.com -p BERkay*389* -ca $CAUTH convolutional-neural-networks
pip3 install --upgrade keyrings.alt
pip3 install --upgrade --user keyrings.alt
pip uninstall --user coursera-dl
pip uninstall  coursera-dl
pip3 uninstall  coursera-dl
coursera-dl convolutional-neural-networks
coursera-dl "convolutional-neural-networks"
coursera-dl --list-courses
python3 cmylmz.py 
pip3 install --user selenium
python3 cmylmz.py 
pip3 install --user geckodriver
wget https://github.com/mozilla/geckodriver/releases/download/v0.24.0/geckodriver-v0.24.0-linux64.tar.gz
tar -xvzf geckodriver*
tar -xvzf geckodriver-v0.24.0-linux64.tar.gz 
chmod +x geckodriver
export PATH=$PATH:/home/brky/Coursera - Deep Learning Specialization/geckodriver
export PATH=$PATH:"/home/brky/Coursera - Deep Learning Specialization/geckodriver"
echo $PATH
python3 cmylmz.py 
export PATH=$PATH:"/home/brky/Coursera - Deep Learning Specialization/"
python3 cmylmz.py 
grep -R "solve("
grep -R "SimpleActionServer<moveit_msgs::MoveGroupAction>"
grep -R "OMPLInterface"
grep -R "PlanningContextPtr"
grep -R "simplifySolutions("
grep -R "PlanningContext"
grep -R "solve(const planning_interface::MotionPlanResponse"
grep -R "solve(planning_interface::MotionPlanResponse"
grep -R "brkygkcn"
grep -R "solve(planning_interface::MotionPlanResponse"
grep -R "SimpleActionServer<moveit_msgs::MoveGroupAction>"
cd ..
grep -R "solve(planning_interface::MotionPlanResponse"
grep -R "brkygkcn:milestone"
grep -R "controller_list"
rosservice list 
grep -R "pr2/right_arm_controller"
grep -R "pr2/right_arm_controller/follow_joint_trajectory"
grep -R "new actionlib::SimpleActionServer"
grep -R "SimpleActionServer<T>(getActionName()"
grep -R "SimpleActionServer"
clear
grep -R "SimpleActionServer"
grep -R "controller_action_server_"
grep -R "controller_action_server"
./pr2_safe_spawner.sh 
grep -R "executePart("
grep -R "execute("
grep -R "moveit_msgs::ExecuteTrajectoryAction"
grep -R "moveit_msgs::MoveGroupAction"
clear
grep -R "moveit_msgs::MoveGroupAction"
grep -R "solve(planning_interface::MotionPlanResponse"
grep -R "kinematic"
grep -R "ik"
grep -R "kinematic"
cd ..
grep -R "kinematic"
grep -R "kinematics_solver"
git init
git commit -m "first commit"
git add *
git commit -m "first commit"
git remote add origin git@github.com:brkygokcen/DeepLearningSpecialization.git
git push -u origin master
rosrun jsk_robot_utils marker_msg_from_indigo_to_kinetic.py
roslaunch pr2eus_tutorials pr2_tabletop.launch
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
tmux
git clone https://github.com/jsk-ros-pkg/jsk_robot.git
cd ..
catkin build
cd src/
git clone https://github.com/fetchrobotics/fetch_ros.git -b indigo-devel
cd ..
catkin build fetch_ros
catkin build jsk_robot_utils
catkin build  
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/fetcheus; catkin build --get-env fetcheus | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin_clean fetch_ros
htop
catkin build jsk_robot_utils
catkin_clean
catkin build 
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/test_catkin_virtualenv_py3; catkin build --get-env test_catkin_virtualenv_py3 | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
sudo apt-get install python3-venv
cd /home/brky/workspaces/jsk_ros_pkg_ws/build/test_catkin_virtualenv_py3; catkin build --get-env test_catkin_virtualenv_py3 | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build 
rosrun pr2eus_tutorials pr2-tabletop-object-grasp.l
rviz -d $(rospack find pr2eus_tutorials)/config/pr2_tabletop.rviz
rosrun pr2eus_tutorials pr2-tabletop-object-grasp.l
rosrun topic_tools relay /bounding_box_interactive_marker/kinetic/feedback  /bounding_box_interactive_marker/feedback 
rosrun jsk_robot_utils marker_msg_from_indigo_to_kinetic.py
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
tmux
cd workspaces/air_pr2_ws/
catkin_clean
catkin build
cd /home/brky/workspaces/air_pr2_ws/build/my_controller_pkg; catkin build --get-env my_controller_pkg | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
roslaunch pr2_gazebo pr2_table_object.launch 
roslaunch pr2_moveit_config tabletop_object_recognition.launch 
grep -R "object_information_server"
catkin build
roslaunch pr2_moveit_config demo.launch 
source ~/workspaces/pr2_moveit_ws/devel/setup.bash
roslaunch pr2_moveit_config demo.launch 
roslaunch pr2_moveit_config ompl_planner.launch 
catkin build
cd /home/brky/workspaces/pr2_ws/build/pr2_navigation_self_filter; catkin build --get-env pr2_navigation_self_filter | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
roslaunch pr2_plugs_gazebo_demo pr2_plugs_demo.launch 
git clone https://github.com/PR2/pr2_doors.git -b hydro-devel
cd ..
catkin build
roslaunch pr2_doors_gazebo_demo pr2_doors_demo.launch 
grep -R "whole_arm"
grep -R "whole_body"
rostopcl list | grep kinect
rostopic list | grep "kinect"
rostopic list | grep kinect
rostopic list
rostopic info /pr2/world/points 
./pr2_safe_spawner.sh 
chmod +x web.py 
python3 web.py 
pip3 install --user BeautifulSoup
pip3 install --user beautifulsoup4
python3 web.py 
pip3 install --user pandas
python3 web.py 
gedit web.py
chmod +x geckodriver
python3 web.py 
grep -R "pcl_manager"
rostopic info /pr2/world/points 
rosnode info /pcl_manager
rosrun pr2_robot project_template.py 
roslaunch pr2_moveit pr2_moveit.launch
roslaunch pr2_robot pick_place_demo.launch
tmux
roslaunch sensor_stick training.launch 
roslaunch sensor_stick robot_spawn.launch 
rosrun sensor_stick segmentation.py 
grep -R "/feature_extractor/get_normals"
roslaunch pr2_moveit pr2_moveit.launch
rosrun pr2_robot project_template.py 
catkin build
cd ..
catkin_make
cd pick_and_place_ws/
catkin_make
catkin_clean 
catkin build
roscd pr2_robot/
./pr2_safe_spawner.sh 
catkin_create_pkg object_segmentation rospy roscpp 
cd ..
catkin build
catkin clean
cd src/
catkin_create_pkg air_object_segmentation rospy roscpp 
cd ..
catkin build
cd src/
catkin_create_pkg air_manipulation_demo
cd ..
catkin build
rosrun rqt_graph rqt_graph 
rostopic info /pr2/world/points 
rostopic info /point_cloud 
roslaunch air_manipulation_demo air_manipulation_demo.launch 
rostopic info /pr2/world/points 
rostopic info /point_cloud 
rostopic info /pr2/world/points 
rostopic info /point_cloud 
rostopic info /pr2/world/points 
rostopic info /point_cloud 
htop
roslaunch air_object_segmentation air_object_segmentation.launch 
tmux
catkin_create_pkg air_pick_and_place roscpp
cd ..
catkin build
grep -R "replanning"
catkin_create_pkg air_grasp_server rospy
catkin build air_grasp_server
catkin build air_pick_and_place 
catkin build
cd /home/brky/workspaces/pick_and_place_ws/build/air_pick_and_place; catkin build --get-env air_pick_and_place | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build air_pick_and_place 
catkin build 
roslaunch air_manipulation_demo air_manipulation_demo.launch 
grep -R "Pick And Place Server Ready"
cd ..
catkin build air_pick_and_place 
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_pick_and_place
catkin_create_pkg air_pcl_feature_extracter roscpp rospy pcl sensor_msgs
cd ..
catkin build
catkin_create_pkg air_pcl_feature_extracter roscpp rospy pcl_ros sensor_msgs
cd src/
cd ..
catkin build
roslaunch air_pick_and_place air_pick_and_place.launch 
roslaunch air_manipulation_demo air_manipulation_demo.launch 
cd workspaces/pick_and_place_ws/
catkin build air_object_segmentation
roslaunch air_manipulation_demo air_manipulation_demo.launch 
cd src/
git clone https://github.com/davidfischinger/haf_grasping.git -b kinetic
cd ..
catkin build haf_grasping 
cd ..
catkin build haf_grasping
catkin clean haf_grasping
catkin build haf_grasping
rostopic info /visualization_marker_array_grasp_params
rostopic echo /visualization_marker_array_grasp_params
rostopic pub /haf_grasping/input_pcd_rcs_path std_msgs/String "$(rospack find haf_grasping)/data/pcd2.pcd" -1
rostopic pub /haf_grasping/input_pcd_rcs_path std_msgs/String "$(rospack find haf_grasping)/data/pcd3.pcd" -1
rostopic pub /haf_grasping/input_pcd_rcs_path std_msgs/String "$(rospack find haf_grasping)/data/pcd3.pcd" 
rostopic pub /haf_grasping/input_pcd_rcs_path std_msgs/String "$(rospack find haf_grasping)/data/pcd1.pcd" 
rostopic pub /haf_grasping/input_pcd_rcs_path std_msgs/String "$(rospack find haf_grasping)/data/pcd6.pcd" 
rostopic pub /haf_grasping/input_pcd_rcs_path std_msgs/String "$(rospack find haf_grasping)/data/pcd2.pcd" 
clear
roslaunch haf_grasping haf_grasping_all.launch 
rostopic list
rostopic info /haf_grasping/depth_registered/single_cloud/points_in_lcs 
rostopic echoo /haf_grasping/depth_registered/single_cloud/points_in_lcs 
rostopic echo /haf_grasping/depth_registered/single_cloud/points_in_lcs 
roslaunch haf_grasping haf_grasping_all.launch 
roslaunch haf_grasping haf_grasping_all.launch
source ~/.bashrc
roslaunch haf_grasping haf_grasping_all.launch
roslaunch air_manipulation_demo air_manipulation_demo.launch 
tmux
catkin build air_pcl_feature_extracter 
catkin build air_pick_and_place 
grep -R extracter
catkin clean air_pcl_feature_extracter
catkin build air_pcl_feature_extractor
source ~/.bashrc
catkin build air_pcl_feature_extractor
catkin build 
catkin build air_manipulation_demo 
cd /home/brky/workspaces/pick_and_place_ws/build/air_manipulation_demo; catkin build --get-env air_manipulation_demo | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build air_manipulation_demo 
grep -R getparam
grep -R "getparam"
grep -R "getParam"
cd ..
catkin build
rostopic info /grasp_object 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
grep -R  "type="string""
grep -R  "type=\"string\""
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
roslaunch air_manipulation_demo air_manipulation_demo.launch 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
rosservice list 
rosservice list
rosrun rqt_graph rqt_graph 
catkin build air_manipulation_demo 
source ~/.bashrc
roslaunch air_manipulation_demo air_manipulation_demo.launch 
cd ..
catkin build
roslaunch air_manipulation_demo demo.launch 
roslaunch air_pick_and_place air_pick_and_place.launch 
roslaunch air_manipulation_demo air_manipulation_demo.launch 
rosrun air_pcl_feature_extractor air_pcl_feature_extractor
tmux
roslaunch air_manipulation_demo air_manipulation_demo.launch 
roslaunch air_manipulation_demo demo.launch 
catkin build
roslaunch air_manipulation_demo demo.launch 
roslaunch air_manipulation_demo air_manipulation_demo.launch 
catkin build
nvcc -V
nvidia-smi 
docker ps
docker ps -l
docker images ls
docker ps -aq
docker image ls
rostopic list
rostopic hz /turtle2/cmd_vel 
rostopic echo /turtle2/cmd_vel 
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist ---r 1 '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
"export ROS_HOSTNAME=$(hostname --ip-address)" >> ~/.bashrc
echo "export ROS_HOSTNAME=$(hostname --ip-address)" >> ~/.bashrc
echo "\nexport ROS_HOSTNAME=$(hostname --ip-address)" >> ~/.bashrc
echo "export ROS_HOSTNAME=$(hostname --ip-address)" >> ~/.bashrc
export ROS_HOSTNAME=$(hostname --ip-address) >> ~/.bashrc
gedit ~/.bashrc
ifconfig
rostopic list
roscore
rostopic pub /haf_grasping/input_pcd_rcs_path std_msgs/String "$(rospack find haf_grasping)/data/pcd2.pcd" 
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.2]'
rostopic list
rostopic echo /turtle1/cmd_vel 
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.2]'
rostopic echo /turtle1/cmd_vel 
docker build -t ros:1.0 .
docker ps
docker images ls
docker images -l
docker images ps
docker images list
docker images 
docker run -i -t ros:1.0 /bin/bash
docker run -i -t ros:1.0 --network host /bin/bash
docker run --network host -i -t ros:1.0 /bin/bash
docker build -t ros:1.1 .
docker run --network host -i -t ros:1.1 /bin/bash
docker build -t ros:1.1 .
docker run --network host -i -t ros:1.1 
docker run --network host -i -t ros:1.1 /bin/bash
docker images list
docker images ps
docker list
docker ps
docker ps -a
docker images -a
docker system prune -a
docker images -a
docker ps -a
docker list
docker container list
docker build -t ros:1.1 .
docker images -a
docker run --network host -i -t ros:1.1 /bin/bash
tmux
roscore
docker images
docker rmi dc068c2f3afc
docker rmi b7b4b5b08941
docker images
docker rmi 10c4bd25ca4d
docker rmi --force 10c4bd25ca4d
docker images
docker rmi --force 99b71052693d
docker rmi --force 619ccd521fc1
docker images
docker rmi --force 99b71052693d
docker images
sudo rm phidgets.list phidgets.list
sudo rm ./phidgets.list 
sudo rm phidgets.list.save 
docker load < graspnet-ros.tar 
docker images
docker load < ros.tar 
docker images
xhost +
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix brkygkcn/graspnet-ros:1.0
nvidia-smi 
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix brkygkcn/graspnet-ros:1.0
sudo apt-get update
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix brkygkcn/graspnet-ros:1.0
sudo apt-get install -y nvidia-container-toolkit
nvidia-smi 
sudo apt-get install nvidia-container-toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
xhost +
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix brkygkcn/graspnet-ros:1.0
docker run --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  ros:1.0 /bin/bash
gcc dummy.cpp -o dummy
gcc -std=cpp11 dummy.cpp -o dummy
gcc --std=cpp11 dummy.cpp -o dummy
gcc -std=c++11 dummy.cpp -o dummy
g++ -std=c++11 dummy.cpp -o dummy
./dummy 
g++ -std=c++11 dummy.cpp -o dummy
./dummy 
rostopic list
docker run --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  ros:1.0 /bin/bash
xhost +
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix brkygkcn/graspnet-ros:1.0
chmod +x *
sudo chmod +x *
sudo chomd +x -r *
sudo chmod +x -r *
sudo chmod +x *
sudo chmod +x -r *
sudo chmod -r +x *
git checkout dev
git pull origin dev
git pull 
git status
git commit -m "brkygkcn"
git push
git push origin dev
git push dev
git add *
git status
roslaunch realsense2_camera rs_camera.launch 
roscore
catkin_release 
catkin build
cd /home/brky/workspaces/catkin_ws/build/roboware_package; catkin build --get-env roboware_package | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin_clean 
catkin build
roslaunch realsense2_camera rs_camera.launch 
roscd realsense_camera/
source ~/.bashrc 
roslaunch realsense2_camera rs_camera.launch 
rosfind realsense2_camera
roscd realsense2_camera
roslaunch realsense2_camera rs_camera.launch 
rviz
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/graspnet-dev:/home/graspnet-dev brkygkcn/graspnet-ros:1.0
rostopic list
rostopic info /camera/depth/color/points
rviz
catkin_create_pkg air_grasp_demo rospy sensor_msgs std_msgs
cd workspaces/ca
cd workspaces/catkin_ws/
catkin build
source ~/.bashrc
rosrun air_grasp_demo air_grasp_demo.py 
roslaunch realsense2_camera rs_camera.launch 
roslaunch realsense2_camera rs_aligned_depth.launch 
roslaunch realsense2_camera demo_pointcloud.launch 
rostopic list
rosrun ./main.py
catkin_create_pkg rospy numpy tensorflow pcl
catkin_create_pkg graspnet_ros_module rospy numpy tensorflow pcl
cd ..
catkin build
catkin_create_pkg graspnet_ros_module rospy tensorflow pcl
catkin build
catkin_create_pkg graspnet_ros_module rospy
catkin build
source ~/.bashrc
rosrun graspnet_ros_module main.py 
roslaunch realsense2_camera demo_pointcloud.launch 
xhost +
docker run --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  ros:1.0 /bin/bash
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/graspnet-dev:/home/graspnet-dev brkygkcn/graspnet-ros:1.0
rosrun air_grasp_demo air_grasp_demo.py 
roslaunch realsense2_camera demo_pointcloud.launch 
htop
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/graspnet-dev:/home/graspnet-dev brkygkcn/graspnet-ros:1.0
tmux
docker images
docker images -a
docker rmi --force 355504b87f66
docker rmi  355504b87f66
catkin build
mkdir -p 6dof_grasp_ws/src
cd 6dof_grasp_ws/
catkin builf
catkin build
docker ps
docker images
docker ps
docker commit 4108f5ab3906 brkygkcn/graspnet-ros:1.0
docker images
docker ps
catkin_create_pkg air_6dof_grasp_module rospy
cd ..
catkin build
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v  brkygkcn/graspnet-ros:1.0
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix brkygkcn/graspnet-ros:1.0
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws brkygkcn/graspnet-ros:1.0
g++ -std=c++11 dummy.cpp -o dummy
./dummy 
g++ -std=c++11 dummy.cpp -o dummy
./dummy 
g++ -std=c++11 dummy.cpp -o dummy
./dummy 
g++ -std=c++11 dummy.cpp -o dummy
./dummy 
g++ -std=c++11 dummy.cpp -o dummy
./dummy 
g++ -std=c++11 dummy.cpp -o dummy
./dummy 
rpm --import package-signing-key.pub 
sudo apt install rpm
rpm --import package-signing-key.pub 
dpkg -i zoom_amd64.deb 
sudo dpkg -i zoom_amd64.deb 
sudo apt-get install libxcb-xtest0 
zoom 
zoom --help
zoom start
apt -f install
apt install ./zoom_amd64.deb
sudo apt-get update 
apt install ./zoom_amd64.deb
rm /var/lib/dpkg/lock 
sudo rm /var/lib/dpkg/lock 
apt install ./zoom_amd64.deb
sudo rm /var/lib/apt/lists/lock
sudo rm /var/cache/apt/archives/lock
sudo rm /var/lib/dpkg/lock
sudo dpkg --configure -a
apt install ./zoom_amd64.deb
sudo apt install ./zoom_amd64.deb
zoom
zoom asd
xhost +
rostopic list
python
python
pip install horovod
history
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws brkygkcn/graspnet-ros:1.0
roslaunch realsense2_camera demo_pointcloud.launch 
ifconfig
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws brkygkcn/graspnet-ros:1.0
rostopic hz /my_pcl_topic
rviz
xhost +
docker ps
docker commit 55fbd1cb0bed brkygkcn/graspnet-ros:1.0
docker images
docker ps
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws brkygkcn/graspnet-ros:1.0
clear
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
docker run --gpus all --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 -v /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
roslaunch realsense2_camera demo_pointcloud.launch 
rviz
chmod +x *
sudo chmod +x CMakeLists.txt 
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 -v /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
roslaunch realsense2_camera demo_pointcloud.launch 
ifconfig
xhost X
xhost +
rosmsg show Grasp*
rosmsg show GraspParam
catkin build
catkin clean
sudo catkin clean
catkin build
gedit ~/.bashrc 
source ~/.bashrc 
rosmsg show GraspParam
subl ~/.bashrc
gedit ~/.bashrc 
catkin_create_pkg air_6dof_grasp_module rospy 
catkin build
chmod -R +x ./*
sudo chmod -R +x ./*
source ~/.bashrc 
rostopic list
rostopic hz /graspnet_parameters
rosmsg show GraspParam
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 -v /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
docker rmi  d76584b10ec4
rostopic hz /graspnet_parameters
rostopic echo /graspnet_parameters
roslaunch realsense2_camera demo_pointcloud.launch 
docker images ps
docker ps
docker commit 0d663f80254b brkygkcn/graspnet-ros:1.0
docker images ps
docker images 
docker images -a
docker rmi  355504b87f66
docker rmi -force  355504b87f66
docker rmi --force  355504b87f66
docker rmi 355504b87f66 --force
docker rmi a88cf61d80e6 --force
docker rmi --force 99b71052693d
docker rmi --force a88cf61d80e6
rostopic echo /graspnet_parameters
xhost +
roslaunch realsense2_camera demo_pointcloud.launch 
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 -v /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
catkin build
rostopic hz /graspnet_parameters 
rostopic info /graspnet_parameters 
rostopic echo /graspnet_parameters 
rostopic hz /graspnet_parameters 
cd ~/workspaces/catkin_ws/
catkin build
rostopic hz /graspnet_parameters 
cd ~/workspaces/catkin_ws/
catkin build
rostopic hz /graspnet_parameters 
cd ~/workspaces/catkin_ws/
catkin build
xhost +
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 -v /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
rostopic hz /graspnet_parameters 
roslaunch realsense2_camera demo_pointcloud.launch 
rostopic list
cd workspaces/catkin_ws/
catkin build
cd workspaces/catkin_ws/
catkin build
rosmsg show GraspParam
rostopic hz /graspnet_parameters 
rostopic echo /graspnet_parameters/generated_grasp
rostopic echo /graspnet_parameters/generated_score
cd workspaces/6dof_grasp_ws/src/air_6dof_grasp_module/6dof-graspnet/
python -m demo.main_visualizer
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 -v /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
xhost +
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 -v /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
exit
catkin buil
catkin build 
source ~/.bashrc 
rossrv show air_6dof_grasp_module/GraspVisualization 
xhost +
rosservice list
rosservice info /grasp_visualizer 
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 -v /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
roslaunch realsense2_camera demo_pointcloud.launch 
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 -v /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
htop
nvidia-smi -1
nvidia-smi -h
nvidia-smi -l
nvidia-smi -lms
rviz
rostopic hz /my_pcl_topic 
rostopic echo /my_pcl_topic 
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 -v /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
rostopic echo /my_pcl_topic 
rostopic info /my_pcl_topic 
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 -v /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
rostopic echo /my_pcl_topic 
roscore
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 -v /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
xhost +
roslaunch realsense2_camera demo_pointcloud.launch 
docker run --gpus all --privileged --network host -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/workspaces/6dof_grasp_ws:/home/catkin_ws -v /usr/lib/nvidia-410:/usr/lib/nvidia-410 -v /usr/lib32/nvidia-410:/usr/lib32/nvidia-410 brkygkcn/graspnet-ros:1.0
sudo rm rf .git/
sudo rm -r .git/
htop
docker images
docker images -la
docker images -a
docker save brkygkcn/graspnet-ros graspnet-ros.tar
docker save brkygkcn/graspnet-ros >  graspnet-ros.tar
rosrun gazebo_ros gzserver 
rosrun gazebo_ros gzclient 
gazebo
catkin build
gedit ~/.bashrc 
catkin build
gedit ~/.bashrc 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/move_it_ws/franka_description.urdf -urdf -x 0 -y 0 -z 1 -model panda
rosrun gazebo_ros spawn_model -file /home/brky/1352_ELBMEC/ELBMEC/urdf/ELBMEC.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
source workspaces/catkin_ws/devel/setup.bash 
rosrun gazebo_ros spawn_model -file /home/brky/1352_ELBMEC/ELBMEC/urdf/ELBMEC.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
clear
rostopic list
rosrun gazebo_ros spawn_model -file /home/brky/1352_ELBMEC/ELBMEC/urdf/ELBMEC.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
source ~/.bashrc 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
source ~/.bashrc 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
source ~/.bashrc 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
roscore
roslaunch gazebo_ros empty_world.launch 
source workspaces/catkin_ws/devel/setup.bash 
roslaunch gazebo_ros empty_world.launch 
source ~/.bashrc 
roslaunch gazebo_ros empty_world.launch 
source ~/.bashrc 
roslaunch gazebo_ros empty_world.launch 
tmux
catkin build
cd ..
catkin build
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
source ~/.bashrc 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
tmux
roslaunch gazebo_ros empty_world.launch 
git branch
git clone git@github.com:brkygokcen/financeApp.git
git clone git@github.com:brkygokcen/financeApp.git -b berkay
cd ..
catkin build
[A
catkin build
grep -R "lower="-0.085""
grep -R "-0.085"
grep -R "-0.085" ./
grep -R ./ "-0.085"
grep -R "lower"
htop
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
source ~/.bashrc 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
source ~/.bashrc 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
source ~/.bashrc 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
source ~/.bashrc 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
source ~/.bashrc 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
source ~/.bashrc 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
cd ..
catkin clean EP
catkin build
catkin clean EP
catkin build
catkin clean EP
catkin build
catkin clean EP
catkin build
catkin clean EP
catkin build
roslaunch gazebo_ros empty_world.launch 
source ~/.bashrc 
roslaunch gazebo_ros empty_world.launch 
source ~/.bashrc 
roslaunch gazebo_ros empty_world.launch 
source ~/.bashrc 
roslaunch gazebo_ros empty_world.launch 
source ~/.bashrc 
roslaunch gazebo_ros empty_world.launch 
source ~/.bashrc 
roslaunch gazebo_ros empty_world.launch 
source ~/.bashrc 
roslaunch gazebo_ros empty_world.launch 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
source ~/.bashrc 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
catkin clean EP
catkin build
catkin clean EP
catkin build
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
catkin clean EP
catkin build
roslaunch gazebo_ros empty_world.launch 
htop
sudo apt-get upgrade librealsense2-udev-rules
sudo apt-get install librealsense2-udev-rules
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/catkin_ws/src/EP/urdf/EP.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
sudo cp ~/.99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger
sudo reboot
roslaunch gazebo_ros empty_world.launch 
catkin clean EP
cd ..
htop
catkin build
catkin clean EP
sudo apt-get purge librealsense2
dpkg -l | grep "realsense" | cut -d " " -f 3
realsense-viewer 
sudo apt-get purge realsense-viewer
realsense-viewer --uninstall
sudo cp ~/.99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger
sudo realsense-viewer 
sudo cp ~/.99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger
sudo reboot
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
modinfo uvcvideo | grep "version:"
sudo rm 60-librealsense2-udev-rules.rules 
realsense-viewer
ls /dev/video0
ls /dev/video2
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev
sudo rm 99-realsense-libusb.rules 
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ 
sudo udevadm control --reload-rules && udevadm trigger
./scripts/patch-realsense-ubuntu-lts.sh
cmake ../
mkdir build && cd build
cmake ../
sudo make uninstall && make clean && make && sudo make install
git pull origin berkay
git pull --force origin berkay
cd ..
git clone git@github.com:brkygokcen/financeApp.git
git clone git@github.com:brkygokcen/financeApp.git -b berkay
roslaunch moveit_setup_assistant setup_assistant.launch
roslaunch gazebo_ros empty_world.launch 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/air_arm_ws/src/E/urdf/E.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
catkin build
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/air_arm_ws/src/E/urdf/E.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
roslaunch gazebo_ros empty_world.launch 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/air_arm_ws/src/E/urdf/E.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
catkin clean E
cd ..
catkin build 
source ~/.bashrc 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/air_arm_ws/src/E/urdf/E.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
git add .
git commit -m "get_all_usdt_tickers added"
git push origin berkay
roslaunch gazebo_ros empty_world.launch 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/air_arm_ws/src/e/urdf/e.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
cd ..
catkin clean E
catkin build
roslaunch gazebo_ros empty_world.launch 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/air_arm_ws/src/e/urdf/e.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/air_arm_ws/src/e/urdf/delat.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/air_arm_ws/src/ros_delta_robot/delta_robot_support/urdf/delta_robot.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/air_arm_ws/src/ros_delta_robot/delta_robot_support/urdf/delta_robot.urdf -xacro -x 0 -y 0 -z 1 -model air_arm
source ~/.bashrc 
roslaunch delta_robot_support delta_robot_sim.launch 
git clone git@github.com:serrauvic/ros_delta_robot.git
cd ..
catkin build
source ~/.bashrc 
roslaunch gazebo_ros empty_world.launch 
roslaunch delta_robot_support delta_robot_sim.launch 
rosrun xacro xacro --inorder delta_robot.urdf.xacro > delta.urdf
roscore
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/air_arm_ws/src/ros_delta_robot/delta_robot_support/urdf/delta.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
roslaunch moveit_setup_assistant setup_assistant.launch
roslaunch gazebo_ros empty_world.launch 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/air_arm_ws/src/ros_delta_robot/delta_robot_support/urdf/delta.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
roslaunch moveit_setup_assistant setup_assistant.launch
roscore
rosrun rqt_gui rqt_gui 
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
git clone git@github.com:ros-simulation/gazebo_ros_demos.git
cd ..
catkin build
gedit ~/.bashrc 
rostopic pub -1 /rrbot/joint2_position_controller/command std_msgs/Float64 "data: 1.0"
rostopic pub -1 /rrbot/joint2_position_controller/command std_msgs/Float64 "data: 1.0"
rostopic pub -1 /rrbot/joint1_position_controller/command std_msgs/Float64 "data: 1.5"
rosrun rqt_gui rqt_gui
rosservice call /rrbot/controller_manager/load_controller "name: 'joint1_position_controller'"
rosservice call /rrbot/controller_manager/load_controller "name: 'joint2_position_controller'"
rosservice list
rosrun controller_manager controller_manager -list
rosrun controller_manager controller_manager --list
rosrun controller_manager controller_manager list
rosrun controller_manager list
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: ['joint1_position_controller','joint2_position_controller'], stop_controllers: [], strictness: 2}"
rosrun controller_manager list
rosrun controller_manager controller_manager list
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: ['joint1_position_controller','joint2_position_controller'], stop_controllers: [], strictness: 2}"
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['joint1_position_controller','joint2_position_controller'], strictness: 2}"
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: ['joint1_position_controller','joint2_position_controller'], stop_controllers: [], strictness: 2}"
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['joint1_position_controller','joint2_position_controller'], strictness: 2}"
rostopic pub -1 /rrbot/joint1_position_controller/command std_msgs/Float64 "data: 1.5"
rosservice call /rrbot/controller_manager/load_controller "name: 'joint1_position_controller'"
rosservice call /rrbot/controller_manager/load_controller "name: 'joint2_position_controller'"
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: ['joint1_position_controller','joint2_position_controller'], stop_controllers: [], strictness: 2}"
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['joint1_position_controller','joint2_position_controller'], strictness: 2}"
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: ['joint1_position_controller','joint2_position_controller'], stop_controllers: [], strictness: 2}"
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['joint1_position_controller','joint2_position_controller'], strictness: 2}"
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: ['joint1_position_controller','joint2_position_controller'], stop_controllers: [], strictness: 2}"
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['joint1_position_controller','joint2_position_controller'], strictness: 2}"
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: ['joint1_position_controller','joint2_position_controller'], stop_controllers: [], strictness: 2}"
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['joint1_position_controller','joint2_position_controller'], strictness: 2}"
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: ['joint1_position_controller','joint2_position_controller'], stop_controllers: [], strictness: 2}"
roslaunch rrbot_control rrbot_control.launch
roslaunch rrbot_gazebo rrbot_world.launch
tmux
roslaunch rrbot_gazebo rrbot_world.launch
roslaunch rrbot_control rrbot_control.launch
roslaunch rrbot_control rrbot_rqt.launch
rosservice call /rrbot/controller_manager/load_controller "name: 'joint1_position_controller'"
rosservice call /rrbot/controller_manager/load_controller "name: 'joint2_position_controller'"
rosrun rqt_gui rqt_gui 
rosservice call /rrbot/controller_manager/switch_controller "{start_controllers: ['joint1_position_controller','joint2_position_controller'], stop_controllers: [], strictness: 2}"
roslaunch rrbot_control rrbot_control.launch
ifconfig
grep -R "effort_controllers/JointTrajectoryController"
sudo poweroff
./pr2_safe_spawner.sh 
rostopic list
rostopic info /pr2/left_arm_controller/state
rosrun rqt_gui rqt_gui 
rostopic echo /pr2/left_arm_controller/state
rostopic echo /pr2/left_arm_controller/state -n1
rostopic echo /pr2/left_arm_controller/state 
rostopic echo /pr2/right_arm_controller/command
rostopic list
rostopic echo /pr2/left_gripper_controller/state
rostopic info /pr2/left_gripper_controller/state
rostopic list
./pr2_safe_spawner.sh 
rostopic list
rostopic echo /pr2/left_gripper_controller/command
rostopic echo /pr2/left_gripper_controller/state 
rostopic list
rostopic echo /pr2/left_gripper_controller/gains/left_left_gripper_finger_joint/parameter_descriptions
rostopic echo /pr2/left_gripper_controller/gains/left_left_gripper_finger_joint/parameter_updates 
rostopic echo /pr2/left_gripper_controller/follow_joint_trajectory/result 
grep -R "effort_controllers/JointPositionController"
./pr2_safe_spawner.sh 
rostopic list
rostopic echo /pr2/joint_states 
rostopic echo /pr2/joint_states/effort
rostopic echo /pr2/joint_states  -n1
rostopic echo /pr2/joint_states/effort
./pr2_safe_spawner.sh 
rostopic list
rostopic echo /joint_states 
rostopic echo /joint_states/effort
rostopic echo /pr2/joint_states 
rostopic echo /pr2/joint_states/effort
tmux
rostopic list
rostopic echo /joint_states
rostopic list
rostopic echo /move_group/trajectory_execution/parameter_updates 
rostopic echo /move_group/trajectory_execution/parameter_descriptions 
rostopic list
rostopic echo /trajectory_execution_event 
rostopic info /trajectory_execution_event 
rostopic info /move_group/status
rostopic echo /move_group/status
rostopic list
rostopic echo /move_group/fake_controller_joint_states 
rostopic list
rostopic echo /execute_trajectory/goal 
roslaunch panda_moveit_config demo.launch 
./pr2_safe_spawner.sh 
rostopic list
rostopic echo /execute_trajectory/goal
rostopic info /execute_trajectory/goal
rostopic echo /move_group/goal 
rostopic echo /move_group/feedback 
rostopic echo /move_group/display_planned_path 
rostopic list
rostopic echo /pr2/left_arm_controller/command
rostopic echo /pr2/left_arm_controller/follow_joint_trajectory/status 
rostopic echo /pr2/left_arm_controller/follow_joint_trajectory/goal 
rostopic echo /pr2/left_gripper_controller/state 
rostopic echo /pr2/left_gripper_controller/follow_joint_trajectory/goal 
rostopic eclist
rostopic list
rostopic echo /pr2/joint_states
rostopic echo /pr2/joint_states/effort
rostopic echo /pr2/left_arm_controller/state 
rosrun rqt_gui rqt_gui 
rosrun rqt_plot rqt_plot 
rosrun rqt_gui rqt_gui 
rostopic list
rostopic echo /pr2/left_arm_controller/command
rostopic echo /pr2/left_arm_controller/follow_joint_trajectory/goal
rostopic echo /pr2/left_arm_controller/state -n1
rostopic echo /joint_states/effort
rostopic list
tmux
./pr2_safe_spawner.sh 
clear
rostopic echo /move_group/result 
rostopic list
rosrun pr2_controller_manager pr2_controller_manager list
rosrun controller_manager controller_manager list
grep -R "-10"
grep -R "\-10"
grep -R "\-10 *"
grep -R "\-10 \*"
grep -R "tau"
grep -R "tau_ "
sudo apt-get install ros-kinetic-joint-trajectory-controller
sudo apt-get install ros-kinetic-effort-controllers
grep -R "right_arm_controller/follow_joint_trajectory"
cd ..
grep -R "right_arm_controller/follow_joint_trajectory"
./pr2_safe_spawner.sh 
grep -R "brkygkcn"
roslaunch pr2_robot pick_place_demo.launch
htop
./pr2_safe_spawner.sh 
rostopic list
rostopic echo /pr2/right_arm_controller/state
rostopic echo /pr2/right_arm_controller/command 
rostopic echo /pr2/right_arm_controller/state
./pr2_safe_spawner.sh 
killall -9 gzserver
killall -9 gzclient
htop
rostopic echo /pr2/right_arm_controller/state
killall -9 gzserver
./pr2_safe_spawner.sh 
htop
git clone git@github.com:ros-controls/ros_controllers.git
cd ..
catkin build
cd src/
git clone git@github.com:ros-controls/ros_controllers.git -b kinetic-devel
cd ..
catkin build
roslaunch pr2_robot pick_place_demo.launch
catkin build
./pr2_safe_spawner.sh 
rostopic list
rostopic echo /pr2/right_arm_controller/state
grep -R "BERKAY"
grep -R "effort"
rostopic echo /joint_states 
rostopic echo /joint_states/effort -n1
./pr2_safe_spawner.sh 
g++ -help
g++ --help
catkin build
rostopic list
rostopic echo /gazebo/model_states 
rostopic list
rostopic echo /gazebo/link_states 
rostopic list
catkin build
roslaunch gazebo_ros empty_world.launch 
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/air_arm_ws/src/air_wrist_ros/urdf/air_wrist_ros.urdf -urdf -x 0 -y 0 -z 1 -model air_arm
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch 
cd .. 
catkin build
rostopic list
rostopic echo /joint_states 
rostopic list
rostopic echo /move_group/fake_controller_joint_states
rostopic echo /tf
rosrun rqt_tf_tree rqt_tf_tree 
rostopic echo /joint_states 
roslaunch air_wrist_ros_moveit_config demo.launch 
ssh airlab@192.168.66.2
roslaunch air_wrist_ros display.launch 
rostopic echo /joint_states 
rostopic list
rostopic echo /joint_states 
roslaunch air_wrist_ros gazebo.launch 
roslaunch air_wrist_ros_moveit_config gazebo.launch 
roslaunch air_wrist_ros_moveit_config de
roslaunch air_wrist_ros_moveit_config demo.launch 
catkin clean air_wrist_ros_moveit_config
catkin build 
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build 
roslaunch air_wrist_ros_moveit_config demo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
rostopic echo /joint_states 
rostopic list
rostopic echo /move_group/fake_controller_joint_states 
cd ..
catkin build
source ~/.bashrc 
roslaunch air_wrist_ros_moveit_config demo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch air_wrist_ros_moveit_config demo.launch 
cd workspaces/fjnunes_ws/
catkin build
rostopic list
rostopic echo /move_group/result
rostopic echo /move_group/feedback 
rostopic echo /move_group/status 
rostopic echo /move_group/fake_controller_joint_states 
rostopic echo /joint_states 
rostopic echo /move_group/result
roslaunch air_wrist_ros_moveit_config demo.launch 
roscd ros_controllers/
grep -R "legacy"
cd ..
grep -R "legacy"
grep -R "legacyModeNS"
cd ..
cd air_arm_ws/src/
grep -R "legacyModeNS"
cd ..
catkin build
source ~/.bashrc 
roslaunch air_wrist_ros_moveit_config demo.launch 
roslaunch air_wrist_ros_moveit_config demo_gazebo.launch 
roslaunch air_wrist_ros_moveit_config air_wrist.launch 
catkin build
source ~/.bashrc 
catkin build
source ~/.bashrc 
roslaunch air_wrist_ros_moveit_config air_wrist.launch 
rostopic list
rostopic echo /joint_states 
rostopic list
rostopic info /joint_states 
rostopic info /gazebo/model_states 
rostopic echo /gazebo/model_states 
rostopic list
rostopic info /joint_states 
rostopic echo /joint_states 
rostopic list
rostopic echo /joint_states 
rostopic list
rostopic echo /trajectory_execution_event 
rostopic info /trajectory_execution_event 
rostopic list
rostopic echo /joint_states 
rostopic list
rostopic echo /joint_states 
rostopic list
rostopic echo /joint_states_relay 
rostopic list
rostopic echo /joint_states 
catkin clean 
catkin build
rosrun gazebo_ros spawn_model -file /home/brky/workspaces/fjnunes_ws/src/air_wrist_new.urdf -urdf -x 0 -y 0 -z 1 -model air_wrist
roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true
catkin build
roslaunch air_wrist_ros_moveit_config demo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch
roslaunch air_wrist_ros_moveit_config ros_controllers.launch 
rostopic list
rostopic echo /joint_states_desired
roslaunch air_wrist_ros_moveit_config demo.launch 
rostopic list
roslaunch air_wrist_ros_moveit_config demo.launch 
catkin build
rostopic list
rostopic echo /joint_states 
roslaunch air_wrist_ros_moveit_config demo.launch 
roslaunch air_wrist_ros_config demo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch air_wrist_ros_config ros_controllers.launch 
rostopic l
rostopic list 
rostopic echo /move_group/fake_controller_joint_states
rostopic list
rostopic echo /joint_states
rostopic list
roslaunch air_wrist_ros_config ros_controllers.launch 
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-ros-control
sudo apt-get install ros-kinetic-ros-control ros-kinetic-joint-state-controller ros-kinetic-effort-controllers ros-kinetic-position-controllers ros-kinetic-velocity-controllers ros-kinetic-ros-controllers ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-control
source ~/.basht
source ~/.bashrc 
roslaunch air_wrist_ros_config ros_controllers.launch 
roslaunch air_wrist_ros_config demo.launch 
rostopic list 
rostopic list
roslaunch air_wrist_ros_config air_wrist.launch 
roslaunch air_wrist_ros_config demo.launch 
rostopic list
roslaunch air_wrist_ros_config air_wrist.launch 
grep -R "controller_list:"
grep -R "controller_list:["
grep -R "controller_list:\["
grep -R "controller_list:"
rostopic list
rospack find controller_manager
rosrun controller_manager controller_manager list
rosservice list | grep controller_manager
cd workspaces/fjnunes_ws/
catkin build
source ~/.bashrc 
roslaunch air_wrist_ros_config demo.launch 
source ~/.bashrc 
roslaunch air_wrist_ros_config demo.launch 
gedit ~/.bashrc 
catkin clean air_wrist_ros_config
cd src/
catkin clean air_wrist_ros_config
catkin build air_wrist_ros_config
cd ..
catkin build air_wrist_ros_config
roslaunch air_wrist_ros_config demo.launch 
catkin build 
roslaunch air_wrist_ros_config demo_gazebo.launch 
killall -9 gzserver
killall -9 gzclient
roslaunch air_wrist_ros_config demo_gazebo.launch 
rosrun controller_manager controller_manager list
rosrun controller_manager controller_manager 
rosrun controller_manager controller_manager list-types
rosrun controller_manager controller_manager reload-libraries
rosrun controller_manager controller_manager start
rosrun controller_manager controller_manager list
clea
clear
roslaunch air_wrist_ros_config demo_gazebo.launch 
rostopicl ist
rostopic list
roslaunch air_wrist_ros_config gazebo.launch 
roslaunch air_wrist_ros_config demo_gazebo.launch
rostopic list
rostopic echo /air_wrist/air_wrist_controller/state 
roslaunch air_wrist_ros_config gazebo.launch 
rostopic list
rostopic echo /joint_states 
rostopic echo /air_wrist/joint_states 
grep -R "brkygkcn"
rosrun controller_manager controller_manager list
rosrun controller_manager controller_manager start
rosrun controller_manager controller_manager list
rostopic list
rostopic echo /joint_states 
rostopic echo /air_wrist/joint_states 
rosrun controller_manager controller_manager list
rostopic echo /air_wrist/air_wrist_controller/command 
rostopic echo /air_wrist/air_wrist_controller/state 
rostopic pub /air_wrist/air_wrist_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- ''
points:
- positions: [0]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /air_wrist/air_wrist_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- ''
points:
- positions: [0 0 0]
  velocities: [0 0 0]
  accelerations: [0 0 0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /air_wrist/air_wrist_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- ''
points:
- positions: [0]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /air_wrist/air_wrist_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- ''
points:
- positions: [0 0 0]
  velocities: [0 0 0]
  accelerations: [0 0 0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /air_wrist/air_wrist_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- ''
points:
- positions: [0 0 0]
  velocities: [0 0 0]
  accelerations: [0 0 0]
  effort: [0 0 0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /air_wrist/air_wrist_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'roll_j, pitch_j, yaw_j'
points:
- positions: [0 0 0]
  velocities: [0 0 0]
  accelerations: [0 0 0]
  effort: [0 0 0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /air_wrist/air_wrist_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'world'
joint_names:
- 'roll_j, pitch_j, yaw_j'
points:
- positions: [0 0 0]
  velocities: [0 0 0]
  accelerations: [0 0 0]
  effort: [0 0 0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /air_wrist/air_wrist_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'roll_j, pitch_j, yaw_j'
points:
- positions: [0 0 0]
  velocities: [0 0 0]
  accelerations: [0 0 0]
  effort: [0 0 0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /air_wrist/air_wrist_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "world"
joint_names:
- 'roll_j, pitch_j, yaw_j'
points:
- positions: [0 0 0]
  velocities: [0 0 0]
  accelerations: [0 0 0]
  effort: [0 0 0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /air_wrist/air_wrist_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "base_link"
joint_names:
- 'roll_j, pitch_j, yaw_j'
points:
- positions: [0 0 0]
  velocities: [0 0 0]
  accelerations: [0 0 0]
  effort: [0 0 0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /air_wrist/air_wrist_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 1
joint_names:
- 'roll_j, pitch_j, yaw_j'
points:
- positions: [0 0 0]
  velocities: [0 0 0]
  accelerations: [0 0 0]
  effort: [0 0 0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /air_wrist/air_wrist_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "1"
joint_names:
- 'roll_j, pitch_j, yaw_j'
points:
- positions: [0 0 0]
  velocities: [0 0 0]
  accelerations: [0 0 0]
  effort: [0 0 0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /air_wrist/air_wrist_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '1'
joint_names:
- 'roll_j, pitch_j, yaw_j'
points:
- positions: [0 0 0]
  velocities: [0 0 0]
  accelerations: [0 0 0]
  effort: [0 0 0]
  time_from_start: {secs: 0, nsecs: 0}" 
git clone https://bitbucket.org/AndyZe/pid.git
cd ..
catkin build
https://github.com/ros-controls/control_toolbox.git
git clone https://github.com/ros-controls/control_toolbox.git
cd ..
catkin build
cd src/
git clone https://github.com/ros-controls/control_toolbox.git -b kinetic-devel
cd ..
catkin build
roscd control_toolbox/
rostopic echo /joint_states 
rostopic echo /air_wrist/joint_states 
rostopic list
rostopic echo /joint_states 
roslaunch air_wrist_ros_config demo_gazebo.launch 
roslaunch air_wrist_ros_config gazebo.launch 
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch air_wrist_ros_config demo_gazebo.launch 
rostopic  echo /air_wrist/joint_states 
ifconfig 
rostopic list
rostopic echo /joint_states 
rostopic info /joint_states 
rostopic list
rostopic echo /gazebo/link_states 
rostopic list
rostopic info /air_wrist/air_wrist_controller/state 
rosservice list | grep controller_manager
rospack find joint_state_controller
rostopic list
rostopic info /air_wrist/air_wrist_controller/state 
rostopic info /air_wrist/air_wrist_controller/command
rostopic info /move_group/trajectory_execution/parameter_descriptions 
rostopic echo /move_group/trajectory_execution/parameter_descriptions 
rostopic echo /move_group/trajectory_execution/parameter_updates 
gedit ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
roslaunch panda_moveit_config demo.launch 
tmux
roslaunch panda_moveit_config demo.launch 
gedit ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
roslaunch air_wrist_ros_config demo.launch 
rosrun controller_manager controller_manager list
rostopic list
rostopic echo /pr2/right_arm_controller/state
rostopic echo /joint_states 
rostopic info /joint_states 
rostopic echo /joint_states 
rostopic echo /joint_states/effort 
rostopic echo /joint_states/effort
rostopic echo /
rostopic echo /pr2/joint_states
rostopic echo /pr2/joint_states/effort[-6]
rostopic echo /pr2/joint_states/effort[:-6]
rostopic echo /pr2/joint_states/effort[9:18]
rostopic echo /pr2/joint_states/effort
tmux
roslaunch pr2_moveit pr2_moveit.launch 
source ~/.bashrc 
roslaunch pr2_moveit pr2_moveit.launch 
roslaunch pr2_robot pick_place_demo.launch 
source ~/.bashrc 
roslaunch pr2_robot pick_place_demo.launch 
tmux
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch air_wrist_ros_config demo_gazebo.launch 
roslaunch pr2_robot robot_description.launch 
roslaunch pr2_robot pick_place_demo.launch 
roslaunch pr2_robot robot_description.launch 
roslaunch pr2_robot pick_place_
roslaunch pr2_robot pick_place_demo.launch 
roslaunch air_wrist_ros_config gazebo.launch 
roslaunch air_wrist_ros_config pick_place_demo.launch 
rosrun rqt_reconfigure rqt_reconfigure 
roslaunch air_wrist_ros_config demo_gazebo.launch 
rosrun rqt_reconfigure rqt_reconfigure 
rosrun rqt_plot rqt_plot 
rosrun rqt_gui rqt_gui 
python -m pip install -U matplotlib
pip install -U matplotlib
sudo pip install -U matplotlib
rosrun rqt_plot rqt_plot 
rqt_plot rqt_plot
sudo pip2 install --upgrade matplotlib
sudo pip install --upgrade matplotlib
pip2 install --upgrade matplotlib
sudo pip2 install --upgrade matplotlib
sudo pip2 uninstall  matplotlib
pip2 uninstall  matplotlib
sudo pip2 uninstall  matplotlib
sudo -H pip2 uninstall  matplotlib
sudo -H pip2.7 uninstall  matplotlib
pip
pip uninstall matplotlib
sudo pip uninstall matplotlib
pip uninstall matplotlib
pip uninstall --user matplotlib
pip --user uninstall matplotlib
roscore
rosrun rqt_gui rqt_gui 
rosdep install rqt_plot
rqt_plot /joint_states
pip install --user matplotlib
pip uninstall --user matplotlib
pip uninstall  matplotlib
sudo pip uninstall  matplotlib
sudo pip2 install  matplotlib
pip2 install  matplotlib
pip2 uninstall  matplotlib
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
rostopic list
rostopic echo /joint_states 
rostopic echo /air_wrist/air_wrist_controller/state 
rostopic echo /joint_states 
rostopic echo /joint_states/effort 
sudo pip install --upgrade matplotlib
rosrun rqt_plot rqt_plot 
sudo apt-get install ros-kinetic-rqt ros-kinetic-rqt-common-plugins
sudo apt-get install  ros-kinetic-rqt-robot-plugins
sudo apt-get install  ros-kinetic-rqt-pr2-dashboard 
sudo apt-get install  ros-kinetic-rqt-moveit 
sudo apt-get update
roscore
rosrun rqt_plot rqt_plot 
sudo apt-get remove ros-kinetic-rqt-plot 
sudo apt-get remove ros-kinetic-rqt*
sudo apt-get upgrade ros-kinetic-rqt*
rosrun rqt_plot rqt_plot 
pip install numpy
roscore
rqt_plot
python -m pip install -U pip
sudo python -m pip install -U pip
python -m pip install -U matplotlib 
rqt_plot
rosrun rqt_plot rqt_plot 
rosrun rqt_gui rqt_gui 
rostopic echo /joint_states 
rostopic echo /air_wrist/joint_states 
rqt_plot /joint_states
rqt_plot /joint_states/effort
rqt_plot /joint_states/effort[0]
rostopic info /joint_states 
source ~/.bashrc
killall -9 gzserver
killall -9 gzclient
roslaunch air_wrist_ros_config pick_place_demo.launch 
roslaunch air_wrist_ros_config demo_gazebo
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
rostopic list
clear
rostopic list
rostopic echo /air_wrist/air_wrist_controller/air_wrist_state 
clear
tmux
catkin build
roslaunch air_wrist_ros_config demo_gazebo.launch 
catkin build
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
git clone git@192.168.66.3:brkygkcn/my_tools.git
catkin build
rostopic echo /joint_states 
rosrun arm_planned_path_to_matlab joint_state_listener.py 
roslaunch air_wrist_ros_config demo_gazebo.launch 
rostopic echo /joint_states/velocity[0]
rostopic echo /joint_states/velocityy
rostopic echo /joint_states
rostopic echo /joint_states/velocity[0]
roslaunch air_wrist_ros_config demo_gazebo.launch 
rostopic echo /joint_states/velocity
roscd control_toolbox/
sudo apt-get remove ros-kinetic-control-toolbox 
catkin build
sudo apt-get install ros-kinetic-control-toolbox 
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
catkin build
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
roslaunch air_wrist_ros_config demo_gazebo.launch 
catkin_create_pkg graspnet_ros_module rospy tensorflow pcl
roslaunch air_wrist_ros_config demo_gazebo.launch 
cd workspaces/fjnunes_ws/
catkin build
rostopic echo /joint_states 
roslaunch air_wrist_ros_config demo_gazebo.launch 
roslaunch air_wrist_ros_config demo_gazebo.launch 
clear
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
rostopic echo /joint_states 
rostopic list
rostopic echo /move_group/goal
rostopic echo /move_group/result 
rosrun rqt_gui rqt_gui 
rosrun rqt_reconfigure rqt_reconfigure 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
rostopic list
rostopic echo /move_group/goal 
rostopic echo /move_group/result
rostopic echo /move_group/goal 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header: 
  seq: 21
  stamp: 
    secs: 671
    nsecs: 791000000
  frame_id: ''
goal_id: 
  stamp: 
    secs: 671
    nsecs: 791000000
  id: "/rviz_brky_AIR_17609_5591629224830779485-22-671.791000000"
goal: 
  request: 
    workspace_parameters: 
      header: 
        seq: 0
        stamp: 
          secs: 671
          nsecs: 791000000
        frame_id: "/world"
      min_corner: 
        x: -1.0
        y: -1.0
        z: -1.0
      max_corner: 
        x: 1.0
        y: 1.0
        z: 1.0
    start_state: 
      joint_state: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: "/world"
        name: [roll_j, pitch_j, yaw_j]
        position: [-0.33651135976812974, 0.2501033630752314, 0.45258107443207773]
        velocity: []
        effort: []
      multi_dof_joint_state: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: "/world"
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: False
    goal_constraints: 
      - 
        name: ''
        joint_constraints: 
          - 
            joint_name: "roll_j"
            position: -0.211479299035
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
          - 
            joint_name: "pitch_j"
            position: 0.232518325532
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
          - 
            joint_name: "yaw_j"
            position: -0.3826155738
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
        position_constraints: []
        orientation_constraints: []
        visibility_constraints: []
    path_constraints: 
      name: ''
      joint_constraints: []
      position_constraints: []
      orientation_constraints: []
      visibility_constraints: []
    trajectory_constraints: 
      constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    planning_scene_diff: 
      name: ''
      robot_state: 
        joint_state: 
          header: 
            seq: 0
            stamp: 
              secs: 0
              nsecs:         0
            frame_id: ''
          name: []
          position: []
          velocity: []
          effort: []
        multi_dof_joint_state: 
          header: 
            seq: 0
            stamp: 
              secs: 0
              nsecs:         0
            frame_id: ''
          joint_names: []
          transforms: []
          twist: []
          wrench: []
        attached_collision_objects: []
        is_diff: True
      robot_model_name: ''
      fixed_frame_transforms: []
      allowed_collision_matrix: 
        entry_names: []
        entry_values: []
        default_entry_names: []
        default_entry_values: []
      link_padding: []
      link_scale: []
      object_colors: []
      world: 
        collision_objects: []
        octomap: 
          header: 
            seq: 0
            stamp: 
              secs: 0
              nsecs:         0
            frame_id: ''
          origin: 
            position: 
              x: 0.0
              y: 0.0
              z: 0.0
            orientation: 
              x: 0.0
              y: 0.0
              z: 0.0
              w: 0.0
          octomap: 
            header: 
              seq: 0
              stamp: 
                secs: 0
                nsecs:         0
              frame_id: ''
            binary: False
            id: ''
            resolution: 0.0
            data: []
      is_diff: True
    plan_only: True
    look_around: False
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: False
    replan_attempts: 0
    replan_delay: 0.0"
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header: 
  seq: 21
  stamp: 
    secs: 671
    nsecs: 791000000
  frame_id: ''
goal_id: 
  stamp: 
    secs: 671
    nsecs: 791000000
  id: "/rviz_brky_AIR_17609_5591629224830779485-22-671.791000000"
goal: 
  request: 
    workspace_parameters: 
      header: 
        seq: 0
        stamp: 
          secs: 671
          nsecs: 791000000
        frame_id: "/world"
      min_corner: 
        x: -1.0
        y: -1.0
        z: -1.0
      max_corner: 
        x: 1.0
        y: 1.0
        z: 1.0
    start_state: 
      joint_state: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: "/world"
        name: [roll_j, pitch_j, yaw_j]
        position: [-0.33651135976812974, 0.2501033630752314, 0.45258107443207773]
        velocity: []
        effort: []
      multi_dof_joint_state: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: "/world"
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: False
    goal_constraints: 
      - 
        name: ''
        joint_constraints: 
          - 
            joint_name: "roll_j"
            position: -0.211479299035
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
          - 
            joint_name: "pitch_j"
            position: 0.232518325532
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
          - 
            joint_name: "yaw_j"
            position: -0.3826155738
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
        position_constraints: []
        orientation_constraints: []
        visibility_constraints: []
    path_constraints: 
      name: ''
      joint_constraints: []
      position_constraints: []
      orientation_constraints: []
      visibility_constraints: []
    trajectory_constraints: 
      constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    planning_scene_diff: 
      name: ''
      robot_state: 
        joint_state: 
          header: 
            seq: 0
            stamp: 
              secs: 0
              nsecs:         0
            frame_id: ''
          name: []
          position: []
          velocity: []
          effort: []
        multi_dof_joint_state: 
          header: 
            seq: 0
            stamp: 
              secs: 0
              nsecs:         0
            frame_id: ''
          joint_names: []
          transforms: []
          twist: []
          wrench: []
        attached_collision_objects: []
        is_diff: True
      robot_model_name: ''
      fixed_frame_transforms: []
      allowed_collision_matrix: 
        entry_names: []
        entry_values: []
        default_entry_names: []
        default_entry_values: []
      link_padding: []
      link_scale: []
      object_colors: []
      world: 
        collision_objects: []
        octomap: 
          header: 
            seq: 0
            stamp: 
              secs: 0
              nsecs:         0
            frame_id: ''
          origin: 
            position: 
              x: 0.0
              y: 0.0
              z: 0.0
            orientation: 
              x: 0.0
              y: 0.0
              z: 0.0
              w: 0.0
          octomap: 
            header: 
              seq: 0
              stamp: 
                secs: 0
                nsecs:         0
              frame_id: ''
            binary: False
            id: ''
            resolution: 0.0
            data: []
      is_diff: True
    plan_only: True
    look_around: False
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: False
    replan_attempts: 0
    replan_delay: 0.0"
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header: 
  seq: 21
  stamp: 
    secs: 671
    nsecs: 791000000
  frame_id: ''
goal_id: 
  stamp: 
    secs: 671
    nsecs: 791000000
  id: "/rviz_brky_AIR_17609_5591629224830779485-22-671.791000000"
goal: 
  request: 
    workspace_parameters: 
      header: 
        seq: 0
        stamp: 
          secs: 671
          nsecs: 791000000
        frame_id: "/world"
      min_corner: 
        x: -1.0
        y: -1.0
        z: -1.0
      max_corner: 
        x: 1.0
        y: 1.0
        z: 1.0
    start_state: 
      joint_state: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: "/world"
        name: [roll_j, pitch_j, yaw_j]
        position: [-0.33651135976812974, 0.2501033630752314, 0.45258107443207773]
        velocity: []
        effort: []
      multi_dof_joint_state: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: "/world"
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: False
    goal_constraints: 
      - 
        name: ''
        joint_constraints: 
          - 
            joint_name: "roll_j"
            position: -0.211479299035
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
          - 
            joint_name: "pitch_j"
            position: 0.232518325532
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
          - 
            joint_name: "yaw_j"
            position: -0.3826155738
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
        position_constraints: []
        orientation_constraints: []
        visibility_constraints: []
    path_constraints: 
      name: ''
      joint_constraints: []
      position_constraints: []
      orientation_constraints: []
      visibility_constraints: []
    trajectory_constraints: 
      constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    planning_scene_diff: 
      name: ''
      robot_state: 
        joint_state: 
          header: 
            seq: 0
            stamp: 
              secs: 0
              nsecs:         0
            frame_id: ''
          name: []
          position: []
          velocity: []
          effort: []
        multi_dof_joint_state: 
          header: 
            seq: 0
            stamp: 
              secs: 0
              nsecs:         0
            frame_id: ''
          joint_names: []
          transforms: []
          twist: []
          wrench: []
        attached_collision_objects: []
        is_diff: True
      robot_model_name: ''
      fixed_frame_transforms: []
      allowed_collision_matrix: 
        entry_names: []
        entry_values: []
        default_entry_names: []
        default_entry_values: []
      link_padding: []
      link_scale: []
      object_colors: []
      world: 
        collision_objects: []
        octomap: 
          header: 
            seq: 0
            stamp: 
              secs: 0
              nsecs:         0
            frame_id: ''
          origin: 
            position: 
              x: 0.0
              y: 0.0
              z: 0.0
            orientation: 
              x: 0.0
              y: 0.0
              z: 0.0
              w: 0.0
          octomap: 
            header: 
              seq: 0
              stamp: 
                secs: 0
                nsecs:         0
              frame_id: ''
            binary: False
            id: ''
            resolution: 0.0
            data: []
      is_diff: True
    plan_only: True
    look_around: False
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: False
    replan_attempts: 0
    replan_delay: 0.0"
clear
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: ''
      min_corner: {x: 0.0, y: 0.0, z: 0.0}
      max_corner: {x: 0.0, y: 0.0, z: 0.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: ''
        name: ['']
        position: [0]
        velocity: [0]
        effort: [0]
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: ''
        joint_names: ['']
        transforms:
        - translation: {x: 0.0, y: 0.0, z: 0.0}
          rotation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        twist:
        - linear: {x: 0.0, y: 0.0, z: 0.0}
          angular: {x: 0.0, y: 0.0, z: 0.0}
        wrench:
        - force: {x: 0.0, y: 0.0, z: 0.0}
          torque: {x: 0.0, y: 0.0, z: 0.0}
      attached_collision_objects:
      - link_name: ''
        object:
          header:
            seq: 0
            stamp: {secs: 0, nsecs: 0}
            frame_id: ''
          id: ''
          type: {key: '', db: ''}
          primitives:
          - type: 0
            dimensions: [0]
          primitive_poses:
          - position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          meshes:
          - triangles:
            - vertex_indices: [0, 0, 0]
            vertices:
            - {x: 0.0, y: 0.0, z: 0.0}
          mesh_poses:
          - position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          planes:
          - coef: [0.0, 0.0, 0.0, 0.0]
          plane_poses:
          - position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          operation: 0
        touch_links: ['']
        detach_posture:
          header:
            seq: 0
            stamp: {secs: 0, nsecs: 0}
            frame_id: ''
          joint_names: ['']
          points:
          - positions: [0]
            velocities: [0]
            accelerations: [0]
            effort: [0]
            time_from_start: {secs: 0, nsecs: 0}
        weight: 0.0
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: '', position: 0.0, tolerance_above: 0.0, tolerance_below: 0.0,
        weight: 0.0}
      position_constraints:
      - header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: ''
        link_name: ''
        target_point_offset: {x: 0.0, y: 0.0, z: 0.0}
        constraint_region:
          primitives:
          - type: 0
            dimensions: [0]
          primitive_poses:
          - position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          meshes:
          - triangles:
            - vertex_indices: [0, 0, 0]
            vertices:
            - {x: 0.0, y: 0.0, z: 0.0}
          mesh_poses:
          - position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        weight: 0.0
      orientation_constraints:
      - header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: ''
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        link_name: ''
        absolute_x_axis_tolerance: 0.0
        absolute_y_axis_tolerance: 0.0
        absolute_z_axis_tolerance: 0.0
        weight: 0.0
      visibility_constraints:
      - target_radius: 0.0
        target_pose:
          header:
            seq: 0
            stamp: {secs: 0, nsecs: 0}
            frame_id: ''
          pose:
            position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        cone_sides: 0
        sensor_pose:
          header:
            seq: 0
            stamp: {secs: 0, nsecs: 0}
            frame_id: ''
          pose:
            position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        max_view_angle: 0.0
        max_range_angle: 0.0
        sensor_view_direction: 0
        weight: 0.0
    path_constraints:
      name: ''
      joint_constraints:
      - {joint_name: '', position: 0.0, tolerance_above: 0.0, tolerance_below: 0.0,
        weight: 0.0}
      position_constraints:
      - header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: ''
        link_name: ''
        target_point_offset: {x: 0.0, y: 0.0, z: 0.0}
        constraint_region:
          primitives:
          - type: 0
            dimensions: [0]
          primitive_poses:
          - position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          meshes:
          - triangles:
            - vertex_indices: [0, 0, 0]
            vertices:
            - {x: 0.0, y: 0.0, z: 0.0}
          mesh_poses:
          - position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        weight: 0.0
      orientation_constraints:
      - header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: ''
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        link_name: ''
        absolute_x_axis_tolerance: 0.0
        absolute_y_axis_tolerance: 0.0
        absolute_z_axis_tolerance: 0.0
        weight: 0.0
      visibility_constraints:
      - target_radius: 0.0
        target_pose:
          header:
            seq: 0
            stamp: {secs: 0, nsecs: 0}
            frame_id: ''
          pose:
            position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        cone_sides: 0
        sensor_pose:
          header:
            seq: 0
            stamp: {secs: 0, nsecs: 0}
            frame_id: ''
          pose:
            position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        max_view_angle: 0.0
        max_range_angle: 0.0
        sensor_view_direction: 0
        weight: 0.0
    trajectory_constraints:
      constraints:
      - name: ''
        joint_constraints:
        - {joint_name: '', position: 0.0, tolerance_above: 0.0, tolerance_below: 0.0,
          weight: 0.0}
        position_constraints:
        - header:
            seq: 0
            stamp: {secs: 0, nsecs: 0}
            frame_id: ''
          link_name: ''
          target_point_offset: {x: 0.0, y: 0.0, z: 0.0}
          constraint_region:
            primitives:
            - type: 0
              dimensions: [0]
            primitive_poses:
            - position: {x: 0.0, y: 0.0, z: 0.0}
              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
            meshes:
            - triangles:
              - vertex_indices: [0, 0, 0]
              vertices:
              - {x: 0.0, y: 0.0, z: 0.0}
            mesh_poses:
            - position: {x: 0.0, y: 0.0, z: 0.0}
              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          weight: 0.0
        orientation_constraints:
        - header:
            seq: 0
            stamp: {secs: 0, nsecs: 0}
            frame_id: ''
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          link_name: ''
          absolute_x_axis_tolerance: 0.0
          absolute_y_axis_tolerance: 0.0
          absolute_z_axis_tolerance: 0.0
          weight: 0.0
        visibility_constraints:
        - target_radius: 0.0
          target_pose:
            header:
              seq: 0
              stamp: {secs: 0, nsecs: 0}
              frame_id: ''
            pose:
              position: {x: 0.0, y: 0.0, z: 0.0}
              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          cone_sides: 0
          sensor_pose:
            header:
              seq: 0
              stamp: {secs: 0, nsecs: 0}
              frame_id: ''
            pose:
              position: {x: 0.0, y: 0.0, z: 0.0}
              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          max_view_angle: 0.0
          max_range_angle: 0.0
          sensor_view_direction: 0
          weight: 0.0
    planner_id: ''
    group_name: ''
    num_planning_attempts: 0
    allowed_planning_time: 0.0
    max_velocity_scaling_factor: 0.0
    max_acceleration_scaling_factor: 0.0
  planning_options:
    planning_scene_diff:
      name: ''
      robot_state:
        joint_state:
          header:
            seq: 0
            stamp: {secs: 0, nsecs: 0}
            frame_id: ''
          name: ['']
          position: [0]
          velocity: [0]
          effort: [0]
        multi_dof_joint_state:
          header:
            seq: 0
            stamp: {secs: 0, nsecs: 0}
            frame_id: ''
          joint_names: ['']
          transforms:
          - translation: {x: 0.0, y: 0.0, z: 0.0}
            rotation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          twist:
          - linear: {x: 0.0, y: 0.0, z: 0.0}
            angular: {x: 0.0, y: 0.0, z: 0.0}
          wrench:
          - force: {x: 0.0, y: 0.0, z: 0.0}
            torque: {x: 0.0, y: 0.0, z: 0.0}
        attached_collision_objects:
        - link_name: ''
          object:
            header:
              seq: 0
              stamp: {secs: 0, nsecs: 0}
              frame_id: ''
            id: ''
            type: {key: '', db: ''}
            primitives:
            - type: 0
              dimensions: [0]
            primitive_poses:
            - position: {x: 0.0, y: 0.0, z: 0.0}
              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
            meshes:
            - triangles:
              - vertex_indices: [0, 0, 0]
              vertices:
              - {x: 0.0, y: 0.0, z: 0.0}
            mesh_poses:
            - position: {x: 0.0, y: 0.0, z: 0.0}
              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
            planes:
            - coef: [0.0, 0.0, 0.0, 0.0]
            plane_poses:
            - position: {x: 0.0, y: 0.0, z: 0.0}
              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
            operation: 0
          touch_links: ['']
          detach_posture:
            header:
              seq: 0
              stamp: {secs: 0, nsecs: 0}
              frame_id: ''
            joint_names: ['']
            points:
            - positions: [0]
              velocities: [0]
              accelerations: [0]
              effort: [0]
              time_from_start: {secs: 0, nsecs: 0}
          weight: 0.0
        is_diff: false
      robot_model_name: ''
      fixed_frame_transforms:
      - header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: ''
        child_frame_id: ''
        transform:
          translation: {x: 0.0, y: 0.0, z: 0.0}
          rotation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
      allowed_collision_matrix:
        entry_names: ['']
        entry_values:
        - enabled: [false]
        default_entry_names: ['']
        default_entry_values: [false]
      link_padding:
      - {link_name: '', padding: 0.0}
      link_scale:
      - {link_name: '', scale: 0.0}
      object_colors:
      - id: ''
        color: {r: 0.0, g: 0.0, b: 0.0, a: 0.0}
      world:
        collision_objects:
        - header:
            seq: 0
            stamp: {secs: 0, nsecs: 0}
            frame_id: ''
          id: ''
          type: {key: '', db: ''}
          primitives:
          - type: 0
            dimensions: [0]
          primitive_poses:
          - position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          meshes:
          - triangles:
            - vertex_indices: [0, 0, 0]
            vertices:
            - {x: 0.0, y: 0.0, z: 0.0}
          mesh_poses:
          - position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          planes:
          - coef: [0.0, 0.0, 0.0, 0.0]
          plane_poses:
          - position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          operation: 0
        octomap:
          header:
            seq: 0
            stamp: {secs: 0, nsecs: 0}
            frame_id: ''
          origin:
            position: {x: 0.0, y: 0.0, z: 0.0}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
          octomap:
            header:
              seq: 0
              stamp: {secs: 0, nsecs: 0}
              frame_id: ''
            binary: false
            id: ''
            resolution: 0.0
            data: [0]
      is_diff: false
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0"  >> goal.txt
rostopic echo /move_group/goal 
rostopic echo /move_group/goal 
rosrun rqt_reconfigure rqt_reconfigure  
rostopic echo /joint_states/position -n1
rostopic info /move_group/goal 
rostopic echo /move_group/goal 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.23648763234674064, 0.2694224637354212, -0.01679996973889164]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
      planner_id: "RRTConnect"
      group_name: "air_wrist"
      num_planning_attempts: 10
      allowed_planning_time: 5.0
      max_velocity_scaling_factor: 1.0
      max_acceleration_scaling_factor: 1.0
plan_only: true
look_around: false
look_around_attempts: 0
max_safe_execution_cost: 0.0
replan: false
replan_attempts: 0
replan_delay: 0.0" 
brky@brky-AIR:~$ rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.23648763234674064, 0.2694224637354212, -0.01679996973889164]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
      planner_id: "RRTConnect"
      group_name: "air_wrist"
      num_planning_attempts: 10
      allowed_planning_time: 5.0
      max_velocity_scaling_factor: 1.0
      max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: true
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.23648763234674064, 0.2694224637354212, -0.01679996973889164]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
      planner_id: "RRTConnect"
      group_name: "air_wrist"
      num_planning_attempts: 10
      allowed_planning_time: 5.0
      max_velocity_scaling_factor: 1.0
      max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: true
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.23648763234674064, 0.2694224637354212, -0.01679996973889164]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
    position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: true
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.23648763234674064, 0.2694224637354212, -0.01679996973889164]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: true
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.23648763234674064, 0.2694224637354212, -0.01679996973889164]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: true
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.23648763234674064, 0.2694224637354212, -0.01679996973889164]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: true
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.23648763234674064, 0.2694224637354212, -0.01679996973889164]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: true
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0"
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.0, 0.0, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.2, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.3, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: true
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0"
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.0, 0.0, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.2, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.3, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0"
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.0, 0.0, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.2, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.3, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0"
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.0, 0.0, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.2, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.3, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0"
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.2, 0.3, 0.4]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.0, 0.0, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.5, 0.0, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.5, 0.0, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.5, 0.5, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.5, -0.5, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.5, 0.5, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.5, -0.5, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.5, 0.5, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.5, 0.5, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.5, 0.5, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.5, 0.5, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.5, 0.5, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.7, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.2920810292488225, -0.2579157623083521, -0.023691718521936522]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.7, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.5029813185899545, 0.5235362936905856, 0.0005893636367613198]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [-0.5031861061966687, 0.5053355943075193, 0.49975783246958283]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.49327215415322545, 0.2258375227931646, -0.0721584421832473]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.0, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
roslaunch air_wrist_ros_config pick_place_demo.launch 
roslaunch air_wrist_ros_config demo_gazebo.launch 
rostopic echo /joint_states/position -n1
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.19483310408072896, 0.191906580200504, -0.2815892594498006]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [ 0.5,  0.5, 0.5]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
clear
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.19483310408072896, 0.191906580200504, -0.2815892594498006]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.19483310408072896, 0.191906580200504, -0.2815892594498006]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.19483310408072896, 0.191906580200504, -0.2815892594498006]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [0.19483310408072896, 0.191906580200504, -0.2815892594498006]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic echo /joint_states/positions
rostopic echo /joint_states/position
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [ 0.5109569217824506, 0.5129443495442478, 0.5000818176436788]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic echo /joint_states/position
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [ 0.5109569217824506, 0.5129443495442478, 0.5000818176436788]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: -0.5, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [ 0.19202131373195108, -0.16557397539263619, 0.0007850130856272841]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: -0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [ -0.4, -0.4, -0.4]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [ 0.4, 0.4, 0.4]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: -0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [ -0.4, -0.4, -0.4]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: -0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [ -0.4, 0.4, -0.4]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [ 0.4, 0.4, 0.4]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: -0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [ 0.4, 0.4, 0.4]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: -0.4, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [ -0.4, -0.4, -0.4]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: 0.45, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.45, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.45, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [ 0.45, 0.45, 0.45]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.45, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: -0.45, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: -0.45, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rostopic pub /move_group/goal moveit_msgs/MoveGroupActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  request:
    workspace_parameters:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: '/world'
      min_corner: {x: -1.0, y: -1.0, z: -1.0}
      max_corner: {x: 1.0, y: 1.0, z: 1.0}
    start_state:
      joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        name: ['roll_j', 'pitch_j', 'yaw_j']
        position: [ -0.45, -0.45, -0.45]
        velocity: []
        effort: []
      multi_dof_joint_state:
        header:
          seq: 0
          stamp: {secs: 0, nsecs: 0}
          frame_id: '/world'
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: false
    goal_constraints:
    - name: ''
      joint_constraints:
      - {joint_name: 'roll_j', position: -0.45, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'pitch_j', position: 0.45, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      - {joint_name: 'yaw_j', position: 0.45, tolerance_above: 0.0001, tolerance_below: 0.0001,
        weight: 1.0}
      position_constraints: []
    planner_id: "RRTConnect"
    group_name: "air_wrist"
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
  planning_options: 
    plan_only: false
    look_around: false
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: false
    replan_attempts: 0
    replan_delay: 0.0" 
rosrun rqt_reconfigure rqt_reconfigure 
rostopic echo /joint_states/position -n1
rosrun arm_planned_path_to_matlab joint_state_listener.py 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
sudo reboot
ssh airlab@192.168.66.2
myip="$(dig +short myip.opendns.com @resolver1.opendns.com)"
echo "My WAN/Public IP address: ${myip}"
dig +short myip.opendns.com @resolver1.opendns.com
dig TXT +short o-o.myaddr.l.google.com @ns1.google.com
python -m SimpleHTTPServer 8008
python -m SimpleHTTPServer 80
python -m SimpleHTTPServer 8008
ufw status
sudo ufw status
python -m SimpleHTTPServer 8008
python -m SimpleHTTPServer 46.106.235.155:8008
python -m SimpleHTTPServer http://46.106.235.155:8008/
ifconfig
sudo ufw allow 8008
sudo ufw status
sudo ufw activate
sudo ufw enable
sudo ufw status
sudo ufw delete 8008
sudo ufw disable  8008
sudo ufw deny 8008
sudo ufw status
sudo ufw allow 8008
sudo ufw status
sudo ufw reset
sudo ufw reload
sudo ufw reset
sudo ufw status
sudo ufw disable
sudo ufw status
sudo ufw allow 8008
sudo ufw status
sudo ufw reset
sudo ufw status
python -m SimpleHTTPServer 8008
rosrun rqt_reconfigure rqt_reconfigure 
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
rostopic list
rostopic echo /air_wrist/air_wrist_controller/raw_effort
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
rostopic echo /air_wrist/air_wrist_controller/raw_effort
rostopic info /air_wrist/air_wrist_controller/raw_effort 
source ~/.bashrc
roslaunch air_wrist_ros_config demo_gazebo.launch 
rosrun arm_planned_path_to_matlab joint_state_listener.py 
clear
rostopic echo /joint_states 
rosrun arm_planned_path_to_matlab joint_state_listener.py 
rostopic echo /air_wrist/air_wrist_controller/raw_effort 
rosrun arm_planned_path_to_matlab joint_state_listener.py 
ssh airlab@192.168.66.2
python edit.py 
ifconfşg
ifconfig 
ifconfig
ssh airlab@192.168.66.2
İFCONFİG
ifconfig
roslaunch moveit_setup_assistant setup_assistant.launch 
cd workspaces/fjnunes_ws/
catkin build
source ~/.bashrc
roslaunch ARMSim gazebo.launch
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build
source ~/.bashrc
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch armsim gazebo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
cd workspaces/fjnunes_ws/
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch 
mkdir -p arm/src
cd arm/
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch 
cd workspaces/arm/
catkin build
ls
catkin build
roslaunch arm_config demo_gazebo.launch 
cd workspaces/arm_ws/
catkin build
mkdir -p arm_ws/src
cd arm_ws/
catkin_init_workspace 
catkin_init_workspace src/
rostopic list
roslaunch arm_config demo_gazebo.launch 
roslaunch arm_config demo_gazebo.launch 
rostopic list
ssh airlab@192.168.66.2
roslaunch air_wrist_ros_config demo_gazebo.launch 
roslaunch air_wrist_ros_config demo_gazebo (copy).launch
roslaunch air_wrist_ros_config "demo_gazebo (copy)".launch
roslaunch arm_config demo_gazebo.launch 
rostopic list
rostopic info /arm/joint_states 
rostopic info /joint_states 
rostopic list
rostopic info /joint_states 
roslaunch arm_config demo.launch 
roslaunch arm_config demo_gazebo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
cd workspaces/arm_ws/
catkin build
rostopic list
rostopic info /joint_states 
rostopic list
rostopic info /joint_states 
rostopic echo /joint_states 
rostopic list
catkin build
cd src/
catkin build
cd ..
catkin build
roslaunch arm_config gazebo.launch 
roslaunch arm_config demo.launch 
roslaunch arm_config demo_gazebo.launch 
source ~/.bashrc
roslaunch arm_config demo_gazebo.launch 
cd workspaces/arm_ws/
catkin build
source ~/.bashrc
roslaunch arm_config demo_gazebo.launch 
mkdir -p fjnunes_ws/src
cd fjnunes_ws/
catkin_make
catkin build
catkin_clean 
catkin clean
catkin clean 
catkin_clean 
catkin_clean --workspace
source devel/setup.bash
catkin_clean 
catkin clean 
cd workspaces/fjnunes_ws/
catkin_clean
mkdir -p fjnunes_ws/src
cd fjnunes_ws/
catkin_init_workspace 
catkin build
source devel/setup.bash
roslaunch air_wrist_ros_config demo_gazebo.launch 
cd workspaces/fjnunes_ws/
catkin build
roslaunch arm_config demo_gazebo.launch 
roslaunch air_wrist_ros_config demo_gazebo.launch 
roslaunch arm_config demo.launch 
roslaunch arm_config demo_gazebo.launch 
rosrun rqt_reconfigure rqt_reconfigure 
catkin build
roscd arm_config/
cd ~/
roslaunch arm_config demo_gazebo.launch 
clear
chmod +x -R ./*
sudo chmod +x -R ./*
chmod +x -R ./*
sudo chmod +x -R ./*
sudo tar cvzf ws_03_12_2020.tar.gz workspaces/
roslaunch arm_config demo_gazebo.launch 
roscd arm_config/
cd ..
catkin build
roscd arm_config/
roslaunch arm_config demo_gazebo.launch 
rosrun rqt_tf_tree rqt_tf_tree 
rostopic list
rostopic info /arm/arm_controller/state 
rostopic info /arm/arm_controller/follow_joint_trajectory/result 
rostopic info /arm/arm_controller/follow_joint_trajectory/status 
rostopic echo /arm/arm_controller/follow_joint_trajectory/status 
rostopic echo /arm/arm_controller/follow_joint_trajectory/stre
rostopic echo /arm/arm_controller/follow_joint_trajectory/result 
roslaunch pr2_robot pick_place_project.launch 
source ~/workspaces/arm_ws/devel/setup.bash 
roslaunch arm_config demo_gazebo.launch 
source ~/workspaces/arm_ws/devel/setup.bash 
roslaunch arm_config demo_gazebo.launch 
source ~/workspaces/arm_ws/devel/setup.bash 
roslaunch arm_config demo_gazebo.launch 
rosrun rqt_graph rqt_graph 
roslaunch arm_config demo_gazebo.launch 
roslaunch air_wrist_ros_config demo_gazebo.launch 
catkin builf
catkin build
cd /home/brky/workspaces/fjnunes_ws/build/sensor_stick; catkin build --get-env sensor_stick | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin clean
catkin build
cd /home/brky/workspaces/fjnunes_ws/build/sensor_stick; catkin build --get-env sensor_stick | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
sudo apt-get install libvtk6-dev
sudo apt-get install libvtk6.2
sudo apt-get install libvtk6.2*
sudo apt-get remove libvtk6.2
sudo apt-get upgrade libvtk6.2*
roslaunch arm_config demo_gazebo.launch 
source ~/workspaces/fjnunes_ws/devel/setup.bash 
catkin build
sudo ln -s /usr/lib/aarch64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/libvtkproj4.so
sudo ln -s /usr/lib/aarch64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/aarch64-linux-gnu/libvtkproj4-6.2.so.6.2.0
ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/x86_64-linux-gnu/libvtkproj4-6.2.so.6.2.0
sudo ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/x86_64-linux-gnu/libvtkproj4-6.2.so.6.2.0
catkin build
cd /home/brky/workspaces/fjnunes_ws/build/sensor_stick; catkin build --get-env sensor_stick | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
ld -lvtkproj4-6.2 --verbose
sudo ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/x86_64-linux-gnu/libvtkproj4-6.2.so.6.2.0
ld -lvtkproj4-6.2 --verbose
sudo ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/x86_64-linux-gnu/libvtkproj4-6.2.so
cd /home/brky/workspaces/fjnunes_ws/build/sensor_stick; catkin build --get-env sensor_stick | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
rostopic list
roscd armsim/
roslaunch arm_config arm_demo.launch 
roscd armsim/
cd ..
roslaunch arm_config demo_gazebo.launch 
roslaunch air_wrist_ros_config demo_gazebo.launch 
roslaunch arm_config demo_gazebo.launch 
rostopic list
roslaunch arm_config arm_
roslaunch arm_config arm_demo.launch 
rostopic list
rostopic echo /arm/arm_controller/follow_joint_trajectory/result 
clear
rostopic echo /arm/arm_controller/follow_joint_trajectory/goal 
roslaunch arm_config arm_gazebo.launch 
tmux
roslaunch arm_config arm_demo.launch 
rosservice list | grep get_planning_scene
rosservice info /get_planning_scene 
roslaunch arm_config arm_gazebo.launch 
catkin clean
catkin build
roslaunch arm_config arm_demo.launch 
roscd arm_config/
roslaunch arm_config arm_demo.launch 
cd ..
catkin build
roslaunch arm_config arm_demo.launch 
cd workspaces/arm_ws/
catkin build
cd /home/brky/workspaces/arm_ws/build/moveit_core; catkin build --get-env moveit_core | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
source ~/workspaces/arm_ws/devel/setup.bash 
roslaunch arm_config arm_demo.launch 
catkin build
source ~/workspaces/arm_ws/devel/setup.bash 
catkin build
roslaunch arm_config arm_demo.launch 
rostopic list
rostopic info /joint_states 
rostopic info /arm/joint_states 
roswtf 
rosservice list
roslaunch arm_config moveit_rviz.launch 
roslaunch arm_config arm_gazebo.launch 
cd workspaces/arm_ws/
catkin build
roslaunch arm_config arm_demo.launch 
cd workspaces/arm_ws/
catkin build
roslaunch arm_config arm_demo.launch 
cd workspaces/arm_ws/
catkin build
roslaunch arm_config arm_demo.launch 
cd workspaces/arm_ws/
catkin build
roslaunch arm_config arm_demo.launch 
roslaunch arm_config moveit_rviz.launch 
rosrun rqt_reconfigure rqt_reconfigure 
roslaunch arm_config arm_demo.launch 
mkdir -p dummy_ws/src
cd dummy_ws/
catkin_init_workspace 
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch
roscd arm_config/
roslaunch arm_config demo_gazebo.launch 
catkin build
roscd armsim/
cd ..
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build
roslaunch armsim_config gazebo.launch 
roslaunch armsim_config demo.launch 
roslaunch armsim_config demo_gazebo.launch 
roslaunch arm_config demo_gazebo.launch 
roslaunch armsim_config gazebo.launch 
roslaunch armsim_config demo_gazebo.launch 
rostopic list
roslaunch arm_config demo_gazebo.launch 
cd workspaces/fjnunes_ws/
catkin clean
catkin build
roscd arm
roscd armsim
cd ..
roslaunch arm_config demo_gazebo.launch 
roscd arm_config/
source ~/.bashrc
cd ..
source ~/.bashrc
cd ~/workspaces/fjnunes_ws/
source ~/.bashrc
roslaunch armsim_config demo_gazebo.launch 
rosservice list | grep controller_manager
rosrun controller_manager controller_manager list
roslaunch armsim_config demo_gazebo.launch 
source ~/.bashrc
roslaunch armsim_config demo_gazebo.launch 
catkin_debug 
sudo apt-get install ros-kinetic-joint-state-publisher-gui 
rostopic list
rostopic echo /arm/joint_states 
rostopic echo /joint_states 
rostopic list
rostopic info /joint_states 
rostopic info /arm/joint_states 
rosrun rqt_tf_tree rqt_tf_tree 
rosrun rqt_graph rqt_graph 
roslaunch air_wrist_ros_config demo_gazebo.launch 
roslaunch armsim_config gazebo.launch 
roslaunch armsim_config demo_gazebo.launch 
source ~/.bashrc
roslaunch armsim_config demo_gazebo.launch 
roslaunch armsim_config demo.launch 
rosrun rqt_gui rqt_gui 
rostopic info /joint_states 
rostopic info /arm/joint_states 
rostopic info /joint_states 
rostopic info /ar/joint_states 
rostopic info /arm/joint_states 
rostopic list
rosnode machine 
rosnode info /move_group 
rosservice info /get_planning_scene
rosservice call /get_planning_scene "components:
  components: 0" 
rosservice call /get_planning_scene "components:
  components: 1" 
celar
clear
rostopic list
clear
rostopic list
clear
rostopic list
rostopic info /arm/joint_states 
rostopic info /joint_states 
rosnode info /joint_state_publisher 
rostopic echo /joint_states -n1
rostopic echo /arm/joint_states -n1
rostopic info /arm/joint_states 
rostopic info /joint_states 
rostopic echo /arm/joint_states 
clear
rostopic list
rostopic info /joint_states 
rostopic info /arm/joint_states 
rostopic info /joint_states 
rostopic echo /joint_states 
rostopic echo /arm/joint_states 
rosrun rqt_graph rqt_graph 
rostopic echo /joint_states
rostopic info /joint_states 
rostopic info /arm/joint_states 
rosrun rqt_tf_tree rqt_tf_tree 
rostopic echo /joint_states -n1
rostopic echo /arm/joint_states -n1
rostopic infoo /joint_states
rostopic info /joint_states
rostopic info /arm/joint_states
rostopic info /joint_states
clear
rostopic list
rostopic info /joint_states
rostopic info /arm/joint_states
rostopic info /joint_states
rostopic info /arm/joint_states
rostopic echo /joint_states 
rostopic echo /arm/joint_states 
rostopic echo /joint_states 
rostopic echo /arm/joint_states 
rostopic info /joint_states
rostopic echo /joint_states 
rostopic info /arm/joint_states
clear
rostopic info /arm/joint_states
rostopic info /joint_states
roslaunch armsim_config demo_gazebo.launch 
roslaunch armsim_config dem.launch 
roslaunch armsim_config demo.launch 
roslaunch armsim_config demo_gazebo.launch 
cd workspaces/fjnunes_ws/
catkin build
roslaunch armsim_config demo_gazebo.launch 
rosrun rqt_reconfigure rqt_reconfigure 
roslaunch armsim_config demo_gazebo.launch 
roslaunch armsim_config demo_gazebo.launch -d
roscore
roslaunch armsim_config demo_gazebo.launch -d
roslaunch armsim_config demo_gazebo.launch 
rosrun rosconsole 
rosrun rosconsole --debug
rosconsole lit
rosconsole -h
rosconsole set -h
rosconsole set debug
rosconsole list
rosconsole list /rosout 
rosconsole set /rosout ros* debug 
rosconsole set /rosout  debug 
rosconsole set /move_group ros debug
rosconsole set /arm/controller_spawner ros debug
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
rosrun rqt_console rqt_console 
roslaunch armsim_config demo_gazebo.launch 
killall -9 gzclient
killall -9 gzserver
roslaunch armsim_config demo_gazebo.launch 
source ~/.bashrc
roslaunch armsim_config demo_gazebo.launch 
rosrun joint_trajectory_controller joint_trajectory_controller
rosrun rqt_joint_trajectory_controller joint_trajectory_controller
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
git add ./
git commit -m "air arm - position controller implementation - done"
git remote add origin git@192.168.66.3:brkygkcn/fjnunes_ws.git
git push -u origin master
python3
roslaunch moveit_setup_assistant setup_assistant.launch 
rostopic list
rostopic echo /arm/arm_position_controller/command -n1
rostopic echo /arm/arm_position_controller/follow_joint_trajectory/feedback 
rostopic echo /arm/arm_position_controller/state 
rosrun rqt_reconfigure rqt_reconfigure 
rostopic list
roslaunch armsim_config demo_gazebo.launch 
docker ps
docker images ls
docker images -l
docker images ps
docker images list
nvidia-smi 
docker images -a
roslaunch pr2_robot pick_place_demo.launch 
roslaunch pr2_robot pick_place_project.launch 
roslaunch armsim_config demo_gazebo.launch 
htop
pip install ipynb-py-convert
ipynb-py-convert improved_space.ipynb improved_space.py
ipynb-py-convert ./improved_space.ipynb ./improved_space.py
export LANG=C.UTF-8
export PYTHONIOENCODING=utf-8
export PYTHONUTF8=1
ipynb-py-convert ./improved_space.ipynb ./improved_space.py
pip uninstall ipynb-py-convert
pip install ipynb-py-convert==0.4.5
ipynb-py-convert ./improved_space.ipynb ./improved_space.py
python improved_space.py 
python3 improved_space.py 
clear
python3 improved_space.py 
clear
python3 improved_space.py 
python3 Untitled.py 
clear
python3 improved_space.py 
python3 Untitled.py 
clear
python3 improved_space.py 
clear
python3 improved_space.py 
clear
python3 improved_space.py 
clear
htop
python3 improved_space.py 
roslaunch armsim_config demo_gazebo.launch 
python3 improved_space.py 
python3
pip install xgboost==1.1.1
pip3 install xgboost==1.1.1
pip3 install --ser xgboost==1.1.1
pip3 install --user xgboost==1.1.1
python3 improved_space.py 
python3 Untitled.py 
pip3 install mlxtend
pip3 install --user mlxtend
python3 Untitled.py 
pip3 uninstall --user mlxtend
pip3 uninstall  mlxtend
pip install mlxtend==0.17.0
pip3 install --ser mlxtend==0.17.0
pip3 install --user mlxtend==0.17.0
python3 Untitled.py 
python Untitled.py 
python3 Untitled.py 
pip3 uninstall  mlxtend
pip3 install --user mlxtend==0.13.0
python3 Untitled.py 
python3 improved_space.py 
htop
python3 improved_space.py 
htop
killall -9 kdenlive
htop
./files 
grep -R "gamma" ./
grep -R "lamda" ./
grep -R "similarity" ./
grep -R "gain" ./
grep -R " gain" ./
pip3 install xgboost
pip3 remove xgboost
pip3 uninstall xgboost
python3 setup.py install
sudo python3 setup.py install
python3
python3
python
git clone --recursive https://github.com/dmlc/xgboost-b release_1.1.0
git clone --recursive https://github.com/dmlc/xgboost -b release_1.1.0
cd xgboost/
mkdir build
cd build/
cmake ..
make -j$(nproc)
mkdir build
cd build/
cmake ..
git submodule update --init --recursive
python3 improved_space.py 
clear
python3 improved_space.py 
sudo python3 setup.py install
cd ..
cd build/
cmake ..
python3 improved_space.py 
source ~/.bashrc 
python3 improved_space.py 
grep -R "XGBoosterUpdateOneIter" ./*
grep -R " XGBoosterUpdateOneIter" ./*
cmake --version
clear
python3 improved_space.py 
sudo python3 setup.py install
python3 impr_random_forest.py 
python3
htop
python3 impr_random_forest.py 
sudo python3 setup.py install
python3 improved_space.py 
sudo python3 setup.py install
pip3 uninstall xgboost
python3 improved_space.py 
pip3 uninstall xgboost
sudo pip3 uninstall xgboost
python3
python3 improved_space.py 
python3 impr_random_forest.py 
./rar 
pip2 install numpy
python ros_ads.py 
python
python ros_ads.py 
ping 192.168.66.29
python ros_ads.py 
ifconfgi
ifconfig
python ros_ads.py 
ipconfig
ifconfig
ping 192.168.66.30
ping 192.168.66.29
python ros_ads.py 
python shoulder_ros_ads.py 
roscd armsim_config
grep -R "::bind" ./*
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 1
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 1
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 1
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 1
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 1
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 1
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 1
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 1
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 1
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 1
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 1
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
roscd pr2_robot
rostopic info /move_group/display_planned_path 
python arm_planned_path_to_matlab_scripts_arm_planned_path_to_matlab.py 
roslaunch armsim_config demo_gazebo.launch
cd Downloads/
python arm_planned_path_to_matlab_scripts_arm_planned_path_to_matlab.py 
grep -R "limit" ./*
cd ..
grep -R "limit" ./*
exit
rostopic list
rostopic echo /arm/arm_position_controller/command 
rostopic echo /pickup/feedback
clear
rostopic echo /arm/arm_position_controller/follow_joint_trajectory/result 
rostopic echo /execute_trajectory/feedback 
rostopic echo /place/feedback
clear
rostopic echo /arm/arm_position_controller/follow_joint_trajectory/goal 
rostopic echo /arm/arm_position_controller/follow_joint_trajectory/status 
rostopic echo /trajectory_execution_event 
clear
rostopic list
rostopic info /arm/arm_position_controller/command 
rostopic info /execute_trajectory/
rostopic info /execute_trajectory/status 
rostopic echo /execute_trajectory/status 
rostopic list
tmux
roslaunch armsim_config demo_gazebo.launch
python shoulder_ros_ads.py 1
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
python shoulder_ros_ads.py 2
python shoulder_ros_ads.py 
sudo apt-get install libsdl2-2.0
sudo apt-get install libsdl2-2.0-0 
cd /usr/local/include/
ls
cd..
cd .. 
cd include/
cd SDL2/
ls
cd /usr/lib/
ls
htop
roslaunch armsim_config demo_gazebo.launch
catkin build
catkin build move_group_interface
roscd move_group_interface
catkin_create_pkg air_move_group_interface roscpp moveit moveit_msgs moveit_visual_tools
cd ..
catkin build air_move_group_interface
cd src/
catkin_create_pkg air_move_group_interface roscpp moveit_core
cd ..
catkin build air_move_group_interface
cd /home/brky/workspaces/fjnunes_ws/build/air_move_group_interface; catkin build --get-env air_move_group_interface | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build air_move_group_interface
rosrun air_move_group_interface move_group_interface_node
rosrun air_move_group_interface move_group_interface.cpp 
rosrun air_move_group_interface air_move_group_interface_node 
source ~/.bashrc 
rosrun air_move_group_interface air_move_group_interface_node 
cd workspaces/fjnunes_ws/
catkin build air_move_group_interface
source ~/.bashrc 
rosrun air_move_group_interface air_move_group_interface_node 
catkin build 
./build_locally.sh
git clone -b kinetic-devel https://github.com/ros-planning/moveit_tutorials.git
cd 
cd workspaces/fjnunes_ws/
catkin build
catkin build moveit_tutorials-kinetic-devel
roslaunch armsim_config demo_gazebo.launch
source ~/.bashrc 
rosrun air_move_group_interface air_move_group_interface_node 
source ~/.bashrc 
rosrun air_move_group_interface air_move_group_interface_node 
source ~/.bashrc 
rosrun air_move_group_interface air_move_group_interface_node 
catkin build
cd /home/brky/workspaces/fjnunes_ws/build/moveit_ros_visualization; catkin build --get-env moveit_ros_visualization | catkin env -si  /usr/local/bin/cmake /home/brky/workspaces/fjnunes_ws/src/moveit/moveit_ros/visualization --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/fjnunes_ws/devel/.private/moveit_ros_visualization -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/fjnunes_ws/install; cd -
catkin build
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo_gazebo.launch
rosrun rqt_tf_tree rqt_tf_tree 
rostopic list
rostopic echo /move_group/display_planned_path 
rosnode info /move_group_interface_tutorial 
rostopic info /execute_trajectory/goal 
rostopic echo /execute_trajectory/goal 
rostopic echo /move_group/display_planned_path 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
cd workspaces/fjnunes_ws/
clear
catkin build moveit_tutorials 
tmux
roslaunch arm_config arm_gazebo.launch 
roslaunch armsim_config demo_gazebo.launch
rostopic echo /arm/joint_states 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
clear
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
catkin_create_pkg air_arm_ads rospy moveit_msgs
cd ..
catkin build
roscd moveit_msgs/
catkin build air_arm_ads 
python air_ads_node.py 
clear
catkin_create_pkg dummy rospy
catkin build air_arm_ads 
clear
cd ..
ls devel/share/air_arm_ads/
ls devel/share/air_arm_ads/msg/
catkin build air_arm_ads 
rosservice list
grep -R "MoveGroupInterface" /*
grep -R "MoveGroupInterface" ./*
grep -R "MoveGroupGoal" ./*
cd ..
cd move_it_ws/
cd src/
grep -R "MoveGroupGoal" ./*
cd ..
cd fjnunes_ws/src/
grep -R "actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction>" ./*
git clone https://github.com/ros/actionlib.git -b indigo-devel
cd ..
catkin build
grep -R "executeMoveCallback" ./*
grep -R "initGoal" ./*
rosservice list 
clear
rostopic list
rostopic info /execute_trajectory/goal 
rostopic info /move_group/goal 
clera
clear
rostopic list
clear
rostopic list
rostopic info /move_group/goal
rostopic info /move_group/brky_goal 
clear
rostopic list
rostopic info /arm/arm_position_controller/follow_joint_trajectory/action_topics
rostopic echo /arm/arm_position_controller/follow_joint_trajectory/action_topics
rostopic echo /arm/arm_position_controller/follow_joint_trajectory
rostopic e info /arm/arm_position_controller/follow_joint_trajectory
rostopic info /arm/arm_position_controller/follow_joint_trajectory
rostopic info /arm/arm_position_controller/follow_joint_trajectory/goal 
rosservice list
grep -R "goalCallback" ./*
grep -R "<ActionGoal>("goal", static_cast<uint32_t>" ./*
grep -R "<ActionGoal>(\"goal\", static_cast<uint32_t>" ./*
cd actionlib/include/actionlib/server/
clear
grep -R "goalCallback" ./*
cd ..
grep -R "**** sendGoalFunc" ./*
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
clear
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
rosrun rqt_graph rqt_graph 
clear
rostopic info /move_group/go
rostopic info /move_group/goal 
rostopic info /arm/arm_position_controller/follow_joint_trajectory/goal
rostopic echo /arm/arm_position_controller/follow_joint_trajectory/goal
rostopic echo /arm/arm_position_controller/follow_joint_trajectory/status 
rostopic echo /arm/arm_position_controller/follow_joint_trajectory/result 
roslaunch armsim_config demo_gazebo.launch
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch
rosrun rqt_graph rqt_graph 
tmux
htop
history
cd workspaces/fjnunes_ws/src/
grep -R "<ActionGoal>(\"goal\", static_cast<uint32_t>" ./*
grep -R "[BERKAY]  ----- goalCB: Received new action goal  ----- " ./*
grep -R "[BERKAY]  ----- goalCB: Received new action goal" ./*
grep -R "goalCB: Received new action goal" ./*
grep -R "createTimer(" ./*
grep -R "::createTimer(" ./*
grep -R "setAccepted(" ./*
roscd joint_trajectory_controller/
catkin_create_pkg air_arm_controller roscpp pluginlib controller_interface
cd ..
catkin build
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo_gazebo.launch
tmux
rospack plugins --attrib=plugin controller_interface
catkin build
rosrun rqt_controller_manager rqt_controller_manager 
rospack plugins --attrib=plugin controller_interface
roslaunch armsim_config demo_gazebo.launch
tmux
rospack plugins --attrib=plugin controller_interface
roslaunch armsim_config demo_gazebo.launch
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch
roscd hardware_interface/
roscd ros_control
roslaunch armsim_config ros_controllers.launch 
roslaunch armsim_config demo.launch 
rostopic list
rosservice call /controller_manager/list_controllers
roslaunch armsim_config ros_controllers.launch 
rosrun controller_manager controller_manager 
rosrun controller_manager spawner air_arm_controller
rosservice list
rosservice call /controller_manager/load_controller
roslaunch armsim_config ros_controllers.launch 
rosservice list | grep controller_manager
rosrun controller_manager controller_manager 
rosrun controller_manager controller_manager start arm/air_arm_controller
rosrun controller_manager controller_manager list
rosservice call /controller_manager/list_controllers
rospack plugins --attrib=plugin controller_interface
rosrun controller_manager controller_manager start /arm/arm_position_controller
rostopic list
rosnode list
roslaunch armsim_config ros_controllers.launch 
rosrun rqt_controller_manager rqt_controller_manager
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
rostopic list
clear
rostopic list
rostopic info /joint_states 
rostopic echo /joint_states -n1
rostopic echo /move_group/fake_controller_joint_states -n1
rostopic c info  move_group/fake_controller_joint_states -n1
rostopic info  move_group/fake_controller_joint_states -n1
rostopic info  move_group/fake_controller_joint_states
clear
rostopic list
rostopic info /arm/joint_states 
rostopic echo /arm/joint_states 
rostopic echo /joint_states 
rostopic info /joint_states 
rostopic info /arm/joint_states 
clear
rostopic list
rostopic info /joint_states 
roscore
source ~/.bashrc 
tmux
roslaunch armsim_config demo_gazebo.launch
cd workspaces/fjnunes_ws/src/
grep -R ""goal", static_cast<uint32_t>" ./*
grep -R "\"goal\", static_cast<uint32_t>" ./*
grep -R "include <action_server_imp.h" ./*
grep -R "include \<action_server_imp.h" ./*
grep -R "action_server_imp.h" ./*
grep -R "action_server.h" ./*
grep -R "actionlib/server/action_server.h" ./*
clear
grep -R "actionlib/server/action_server.h" ./*
grep -R "updateCommand(" ./*
roscd controller_interface/
roscd hardware_interface
rostopic list
clear
rostopic echo /move_group/goal 
rostopic info /move_group/goal 
rosrun rqt_graph rqt_graph 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo_gazebo.launch
tmux
grep -R "hardware_interface" ./*
grep -R "hw_iface_adapter_" ./*
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roscore
clear
roslaunch armsim_config demo_gazebo.launch
clear
exit
catkin build
grep -R "HardwareInterfaceAdapter" ./*
roscd controller_manager
cd workspaces/fjnunes_ws/src/
grep -R "JointTrajectoryController<" ./*
roslaunch armsim_config demo.launch 
roslaunch armsim_config demo_gazebo.launch
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
tmux
catkin_create_pkg air_arm_hardware_interface controller_manager hardware_interface 
cd ..
catkin build air_arm_hardware_interface 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo.launch
catkin_create_pkg air_arm_robot roscpp
cd ..
catkin build air_arm_robot 
cd /home/brky/workspaces/fjnunes_ws/build/air_arm_robot; catkin build --get-env air_arm_robot | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build air_arm_robot 
source ~/.bashrc 
roscd air_arm_robot/
catkin build air_arm_hardware_interface 
cd ..
catkin build 
cd workspaces/fjnunes_ws/src/
grep -R "xacro --inorder" ./*
exit
rostopic list
roslaunch armsim_config demo.launch 
clear
cd workspaces/fjnunes_ws/src/
grep -R "load_controller" ./*
grep -R "Waiting for service controller_manager/load_controller" ./*
rosservice list 
rosservice info /controller_manager/load_controller 
rosservice list 
rostopic list
rosservice list 
rosservice call /controller_manager/list_controller_types 
rosservice call /controller_manager/list_controllers
grep -R "loadController(" ./*
roscd gazebo_ros_control/
ls
roscd gazebo_ros_control/
ls
gedit robot_hw_sim_plugins.xml 
cd workspaces/fjnunes_ws/src/
grep -R "Controller Spawner couldn't find the expected controller_manager ROS interface." ./*
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
rostopic list
rostopic echo /joint_states -n1
rosservice list
rosservice call /controller_manager/list_controllers "{}" 
rosservice call /controller_manager/list_controllers
rosservice call /controller_manager/list_controller_types 
clear
rosservice call /controller_manager/list_controllers
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
rosrun controller_manager controller_manager list
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
clear
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
clear
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
source ~/.bashrc 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
source ~/.bashrc 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
source ~/.bashrc 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
source ~/.bashrc 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
source ~/.bashrc 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
roslaunch armsim_config demo.launch 
clear
roslaunch armsim_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
tmux
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
rostopic list
rostopic echo /joint_states 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
source ~/.bashrc 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
clear
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo.launch 
clear
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
catkin build
cd workspaces/fjnunes_ws/src/
grep -R "[BERKAY] ----- **** sendGoalFunc **** -----" ./*
grep -R " sendGoalFunc " ./*
rostopic echo /joint_states 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
tmux
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
cd workspaces/fjnunes_ws/
catkin build
rosrun controller_manager controller_manager list
rosservice call /controller_manager/list_controllers
rosservice call /controller_manager/list_controllers 
rosrun rqt_controller_manager rqt_controller_manager
rostopic list
rostopic info /joint_states 
rostopic echo /joint_states 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
tmux
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
cd /home/brky/workspaces/fjnunes_ws/build/sensor_stick; catkin build --get-env sensor_stick | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
source ~/.bashrc 
cd /home/brky/workspaces/fjnunes_ws/build/sensor_stick; catkin build --get-env sensor_stick | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo.launch 
cd workspaces/fjnunes_ws/
catkin build
roslaunch armsim_config demo.launch 
tmux
rostopic echo /joint_states 
rostopic info /joint_states 
rostopic echo /joint_states 
rostopic h< /joint_states 
rostopic hz /joint_states 
rostopic echo /joint_states 
rostopic list
rostopic info /joint_states 
clear
rostopic echo /joint_states 
clear
rostopic list
clear
rostopic list
rostopic echo /arm/arm_position_controller/state 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
rostopic list
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
rostopic list
clear
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
tmux
cd workspaces/fjnunes_ws/src/
grep -R "taking too long" ./*
grep -R "sendTrajectory(" ./*
clear
grep -R "sendTrajectory(" ./*
grep -R ": public moveit_controller_manager::MoveItControllerHandle" ./*
clear
grep -R ": public moveit_controller_manager::MoveItControllerHandle" ./*
cd workspaces/fjnunes_ws/src/
grep -R "controller_action_server_" ./*
clear
grep -R "new actionlib::SimpleActionServer<T>(getActionName()" ./*
grep -R "new actionlib::SimpleActionServer<T>" ./*
grep -R "new actionlib::SimpleActionServer" ./*
clera
clear
grep -R "control_msgs::FollowJointTrajectoryAction" ./*
clear
grep -R "control_msgs::FollowJointTrajectoryAction" ./*
grep -R "joint_trajectory_segment.h" ./*
grep -R "constraints" ./*
rostopic list
rostopic echo /arm/arm_position_controller/state 
rostopic echo /arm/arm_position_controller/follow_joint_trajectory/feedback 
rostopic echo /arm/arm_position_controller/follow_joint_trajectory/status 
rostopic echo /arm/arm_position_controller/follow_joint_trajectory/result 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
rosparam list
rosparam get /arm/arm_position_controller/constraints/ej
clear
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
tmux
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
clear
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
clear
roslaunch armsim_config demo.launch 
clear
source ~/.bashrc 
roslaunch armsim_config demo.launch 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
clear
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
clear
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
clear
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
clear
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
tmux
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo.launch 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
tmux
exit
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo.launch 
ping 192.168.66.29
ping 192.168.66.30
ping 192.168.66.29
ifconfig
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo.launch 
tmux
mkdir -p ./air_pick_and_place_ws/src
cd air_pick_and_place_ws/
catkin build
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev
sudo apt-get update
sudo apt-get upgrade
sudo apt-get dist-upgrade 
cd workspaces/air_pick_and_place_ws/src/
git clone https://github.com/IntelRealSense/librealsense.git
catkin build
rosrun rviz rviz 
tmux
roslaunch realsense2_camera rs_camera.launch 
rostopic list
rostopic echo /camera/color/camera_info 
rostopic info /camera/color/camera_info 
clear
rostopic list
rostopic hz /camera/color/image_raw/compressed
clear
tmux
https://github.com/UbiquityRobotics/fiducials.git -b kinetic-devel
gt clone https://github.com/UbiquityRobotics/fiducials.git -b kinetic-devel
git clone https://github.com/UbiquityRobotics/fiducials.git -b kinetic-devel
cd ..
catkin build
git clonde https://github.com/ros-perception/vision_msgs.git -b kinetic-devel
git clone https://github.com/ros-perception/vision_msgs.git -b kinetic-devel
cd ..
catkin build
rosrun aruco_detect create_markers.py 100 100 fiducials.pdf
sudo apt install python-cairosvg
rosrun aruco_detect create_markers.py 100 100 fiducials.pdf
rosrun aruco_detect create_markers.py 100 100 fiducials.pdf --paper-size a4
roslaunch aruco_detect aruco_detect.launch
rostopic list
clear
rostopic list
rostopic echo /fiducial_transforms 
roslaunch realsense2_camera rs_camera.launch 
rviz
rosrun rviz rviz
roslaunch aruco_detect aruco_detect.launch
tmux
catkin build
roslaunch aruco_detect aruco_detect.launch 
rosrun rqt_tf_tree rqt_tf_tree 
rostopic list
rostopic echo /fiducial_transforms -n1
roslaunch aruco_detect aruco_detect.launch 
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch realsense2_camera rs_camera.launch 
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch aruco_detect aruco_detect.launch 
roslaunch armsim_config demo.launch 
tmux
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch aruco_detect aruco_detect.launch 
roslaunch armsim_config demo.launch 
rostopic list
rostopic echo /rviz_visual_tools -n1
rostopic echo /rviz_visual_tools_gui -n1
rostopic echo /rviz_visual_tools -n1
rostopic echo /rviz_visual_tools 
clear
rostopic list
rostopic echo /visualization_marker
source ~/.bashrc 
clear
rostopic list
roslaunch aruco_detect aruco_detect.launch 
source ~/.bashrc 
roslaunch aruco_detect aruco_detect.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
tmux
roslaunch aruco_detect aruco_detect.launch 
source ~/.bashrc 
roslaunch aruco_detect aruco_detect.launch 
rostopic echo /move_group/goal 
rostopic echo /joint_states -n1
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo.launch 
tmux
source ~/.bashrc 
roslaunch armsim_config demo.launch 
roslaunch pr2_robot pick_place_project.launch 
source ~/.bashrc 
roslaunch pr2_robot pick_place_project.launch 
roslaunch pr2_robot pick_place_demo.launch 
tmux
roslaunch armsim_config demo.launch 
roslaunch aruco_detect aruco_detect.launch 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch aruco_detect aruco_detect.launch 
roslaunch armsim_config demo.launch 
tmux
roslaunch armsim_config demo.launch 
nvidia-smi 
sudo poweroff
rosclean purge 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo.launch 
tmux
roslaunch aruco_detect aruco_detect.launch 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
clear
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
clear
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
clear
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
clear
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
clear
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
source ~/.bashrc 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
tmux
roscd panda_moveit_config
catkin build
cat /usr/include/eigen3/Eigen/src/Core/util/Macros.h | grep VERSION
catkin build
cd /home/brky/workspaces/move_it_ws/build/moveit_kinematics; catkin build --get-env moveit_kinematics | catkin env -si  /usr/bin/make cmake_check_build_system; cd -
catkin build
roscd trac_ik_kinematics_plugin
catkin build
roslaunch panda_moveit_config demo.launch
tmux
git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git
cd ..
catkin build
catkin clean
catkin build
roslaunch panda_moveit_config demo.launch
roslaunch panda_moveit_config demo.launch rviz_tutorial:=true
rosrun moveit_tutorials pick_place_tutorial
tmux
catkin clean
catkin build
roslaunch panda_moveit_config demo.launch
sudo apt-get update
rosrun moveit_tutorials pick_place_tutorial
tmux
catkin clean
catkin build
cd /home/brky/workspaces/move_it_ws/build/moveit_core; catkin build --get-env moveit_core | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin clean
catkin build
cd /home/brky/workspaces/move_it_ws/build/moveit_core; catkin build --get-env moveit_core | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin clean
catkin build
cat /usr/include/eigen3/Eigen/src/Core/util/Macros.h | grep VERSION
cat /usr/include/eigen3/Eigen/src/Core/util/Macros.h |
cat /usr/include/eigen3/Eigen/src/Core/util/Macros.h 
sudo apt-get install libopenblas-dev
sudo wget https://github.com/eigenteam/eigen-git-mirror/archive/3.3.5.tar.gz
sudo tar -xzvf 3.3.5.tar.gz 
sudo mv eigen-git-mirror-3.3.5/ eigen-3.3.5/
cd eigen-3.3.5/
sudo mkdir buidl
sudo rmdir buidl
sudo mkdir build
sudo cmake ..
cd build/
sudo cmake ..
sudo make
sudo make install
sudo ldconfig -v
cmake .
make
./useEigen 
catkin clean
catkin build
cd /home/brky/workspaces/move_it_ws/build/moveit_core; catkin build --get-env moveit_core | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
sudo tar -xzv eigen-3.2.10.tar.gz 
sudo tar -xzvf eigen-3.2.10.tar.gz 
sudo mv eigen-3.2.10/ eigen-3.2.10/
cd eigen-3.2.10/
sudo mkdir build
cd build/
cmake ..
sudo cmake ..
sudo make
sudo make install
catkin clean
catkin build
cat /usr/include/eigen3/Eigen/src/Core/util/Macros.h | grep VERSION
catkin clean
catkin build
sudo rm -rf eigen-3.3.5/
cd Donw
cd Downloads/
sudo rm -rf eigen-3.2*
sudo rm -rf eigen-3.3.7*
sudo tar -xzvf eigen-3.2.8.tar.gz 
cd eigen-3.2.8/
cmake ..
mkdir build
sudo mkdir build
cd build/
cmake ..
sudo cmake ..
sudo make install
catkin clean
catkin build
catkin build moveit
sudo apt-get install ros-kinetic-catkin python-catkin-tools
sudo apt-get install clang-format-10 python3-rosdep 
sudo apt-get install clang-format python3-rosdep 
sudo apt-get install python-rosdep 
cd ..
cd move_it_ws/
sudo apt-get install libomp-dev 
source ~/.bashrc 
sudo apt-get install libompl-dev
sudo apt-get install libomp-dev
sudo apt-get install libomp*
sudo apt-get install libomp-dev 
sudo apt-get remove libomp-dev 
source ~/.bashrc 
sudo apt-get install libomp-dev 
catkin clean
catkin build
cd /home/brky/workspaces/move_it_ws/build/ompl; catkin build --get-env ompl | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
cd /home/brky/workspaces/move_it_ws/build/geometric_shapes; catkin build --get-env geometric_shapes | catkin env -si  /usr/local/bin/cmake /home/brky/workspaces/move_it_ws/src/geometric_shapes --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/move_it_ws/devel/.private/geometric_shapes -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/move_it_ws/install -DCMAKE_BUILD_TYPE=Release; cd -

cd /home/brky/workspaces/move_it_ws/build/geometric_shapes; catkin build --get-env geometric_shapes | catkin env -si  /usr/local/bin/cmake /home/brky/workspaces/move_it_ws/src/geometric_shapes --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/move_it_ws/devel/.private/geometric_shapes -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/move_it_ws/install -DCMAKE_BUILD_TYPE=Release; cd -
source ~/.bashrc 
cd /home/brky/workspaces/move_it_ws/build/geometric_shapes; catkin build --get-env geometric_shapes | catkin env -si  /usr/local/bin/cmake /home/brky/workspaces/move_it_ws/src/geometric_shapes --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/move_it_ws/devel/.private/geometric_shapes -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/move_it_ws/install -DCMAKE_BUILD_TYPE=Release; cd -
sudo apt-get install libqhull* libgtest-dev
source ~/.bashrc 
cd /home/brky/workspaces/move_it_ws/build/geometric_shapes; catkin build --get-env geometric_shapes | catkin env -si  /usr/local/bin/cmake /home/brky/workspaces/move_it_ws/src/geometric_shapes --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/brky/workspaces/move_it_ws/devel/.private/geometric_shapes -DCMAKE_INSTALL_PREFIX=/home/brky/workspaces/move_it_ws/install -DCMAKE_BUILD_TYPE=Release; cd -
roslaunch aruco_detect aruco_detect.launch 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
clear
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo.launch 
sudo apt-get remove libeigen3-dev 
sudo apt-get autoclean libeigen3-dev 
sudo apt-get update libeigen3-dev 
sudo apt-get upgrade libeigen3-dev 
cd eigen-3.2.8/
sudo make uninstall
make uninstall
cd ..
cd eigen-3.2.8/build/
sudo make uninstall
sudo make uninstall .
cd ..
sudo make uninstall .
cd ..
sudo rm -rf eigen-3.2.8*
sudo tar -xzvf eigen-3.0.7.tar.gz 
cd eigen-3.0.7/
mkdir build
sudo mkdir build
cd build/
cmake ..
sudo cmake ..
sudo make
sudo make install
cat /usr/include/eigen3/Eigen/src/Core/util/Macros.h | grep VERSION
sudo rm -rf eigen-3.0.7*
sudo tar -xzvf eigen-3.2.9.tar.gz 
cd eigen-3.2.9/
mkdir build
sudo mkdir build
cd build/
cmake .
cmake ..
sudo cmake ..
sudo make install
htop
./install-ompl-ubuntu.sh
source ~/.bashrc 
./install-ompl-ubuntu.sh
catkin clean
catkin build
cd /home/brky/workspaces/fjnunes_ws/build/ompl; catkin build --get-env ompl | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
catkin config --blacklist moveit_visual_tools
catkin build
cd /home/brky/workspaces/fjnunes_ws/build/moveit_ros_move_group; catkin build --get-env moveit_ros_move_group | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin config --no-blacklist
catkin build moveit_visual_tools
sudo rm -rf eigen3/
sudo rm -rf Eigen 
sudo mv eigen3 /usr/include/
sudo rm -rf eigen-3.2.9*
sudo rm -rf eigen3/
sudo tar -xzvf eigen-3.3.5.tar.gz 
cd eigen-3.3.5/
mkdir build
sudo mkdir build
cd build/
cmake ..
sudo cmake ..
sudo make install
catkin clean
catkin build
catkin config --blacklist moveit_visual_tools
catkin build
catkin config --blacklist moveit_visual_tools moveit_ros_move_group
catkin build
sudo apt-get install ros-kinetic-moveit-visual-tools 
catkin config --blacklist moveit_ros_move_group
catkin config --blacklist moveit_visual_tools moveit_ros_move_group
catkin build
sudo apt-get install ros-kinetic-eigen*
catkin build
cd /home/brky/workspaces/fjnunes_ws/build/moveit_setup_assistant; catkin build --get-env moveit_setup_assistant | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
catkin build moveit_visual_tools 
source ~/.bashrc 
catkin clean moveit_visual_tools
catkin build moveit_visual_tools 
catkin build 
catkin config --no-blacklist
catkin build 
catkin clean
catkin build 
catkin build moveit
sudo cp /usr/local/include/eigen3/ /usr/include/
sudo mv /usr/local/include/eigen3/ /usr/include/
catkin build moveit
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
catkin build moveit
sudo cp -R /usr/include/eigen3/ /usr/local/include/eigen3/
catkin build moveit
cd /home/brky/workspaces/fjnunes_ws/build/moveit_ros_move_group; catkin build --get-env moveit_ros_move_group | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
sudo apt-get remove ros-kinetic-moveit-msgs 
catkin clean
catkin build moveit
catkin build
catkin config --blacklist moveit_visual_tools
catkin build
catkin config --blacklist moveit_visual_tools moveit_tutorials
catkin build
catkin build moveit_visual_tools
catkin build rviz_visual_tools
catkin build moveit_visual_tools
catkin build 
sudo apt-get remove ros-kinetic-moveit-visual-tools 
catkin build 
sudo apt-get install ros-kinetic-moveit-visual-tools 
catkin build 
cd /home/brky/workspaces/fjnunes_ws/build/pr2_robot; catkin build --get-env pr2_robot | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin config --blacklist moveit_visual_tools moveit_tutorials pr2_robot
catkin build 
catkin config --no-blacklist
catkin build 
cd /home/brky/workspaces/fjnunes_ws/build/moveit_tutorials; catkin build --get-env moveit_tutorials | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
rosdep install --from-paths ./src --ignore-src --rosdistro=kinetic
cd /home/brky/workspaces/fjnunes_ws/build/moveit_tutorials; catkin build --get-env moveit_tutorials | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
sudo apt-get install libeigen3-dev
sudo apt-get upgrade libeigen3-dev
sudo apt-get remove libeigen3-dev
cd src/moveit_tutorials/
ls
./build_locally.sh 
sudo apt-get install ros-kinetic-rosdoc-lite
./build_locally.sh 
catkin build moveit_tutorials
source ~/.bashrc 
catkin build moveit_tutorials
source ~/.bashrc 
catkin build moveit_tutorials
cd ..
catkin build moveit_visual_tools
catkin config --no-blacklist
catkin build moveit_visual_tools
cd src/
git clone https://github.com/ros-planning/moveit_visual_tools.git -b kinetic-devel
cd ..
catkin build moveit_visual_tools
catkin clean moveit_visual_tools
catkin clean moveit_visual_tools-kinetic-devel
catkin build moveit_visual_tools
cd /home/brky/workspaces/fjnunes_ws/build/moveit_visual_tools; catkin build --get-env moveit_visual_tools | catkin env -si  /usr/bin/make cmake_check_build_system; cd -
catkin build moveit_visual_tools
catkin build
source ~/.bashrc 
catkin build
sudo apt-get remove ros-kinetic-moveit-visual-tools 
source ~/.bashrc 
sudo apt-get remove ros-kinetic-moveit-visual-tools 
catkin build
cd /home/brky/workspaces/fjnunes_ws/build/moveit_tutorials; catkin build --get-env moveit_tutorials | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
sudo apt-get install libeigen3-dev
sudo apt-get autoremove libeigen3-dev
sudo apt-get check libeigen3-dev
sudo apt-get clean libeigen3-dev
sudo apt-get check libeigen3-dev
sudo apt-get install libeigen3-dev
sudo ln -sf /usr/local/include/eigen3/Eigen /usr/include/Eigen
sudo ln -sf /usr/local/include/eigen3/Eigen /usr/include/eigen3
cd ~/
sudo tar -xzvf eigen-3.3.9.tar.gz 
cd eigen-3.3.9/
sudo mkdir build 
cd build/
cmake ..
sudo cmake ..
sudo make 
sudo make install
cd ..
cd local/include/
sudo rm -rf eigen3/
sudo rm -rf eigen3/
sudo rm -rf eigen-3.3.*
sudo cp -R ~/eigen3/ /usr/include/eigen3/
sudo rm -rf /usr/include/eigen3/
sudo cp -R ~/eigen3/ /usr/include/eigen3/
sudo mv ~/eigen3/ /usr/include/eigen3/
sudo mv /home/brky/eigen3/ /usr/include/
sudo cp /home/brky/eigen3/ /usr/include/eigen3/
sudo cp iR /home/brky/eigen3/ /usr/include/eigen3/
sudo cp -R /home/brky/eigen3/ /usr/include/eigen3/
sudo reboot
sudo cp -R /home/brky/eigen3/ /usr/include/eigen3/
sudo rmdir /usr/include/eigen3/
sudo rm /usr/include/eigen3/
sudo rm /usr/include/eigen3
sudo cp -R /home/brky/eigen3/ /usr/include/eigen3/
sudo reboot
sudo apt-get install libeigen3-dev 
sudo apt-get check libeigen3-dev 
sudo apt-get clean libeigen3-dev 
sudo apt-get upgrade libeigen3-dev 
grep -R "verison"
sudo chmod +x Macros.h 
sudo rm -rf new/
sudo rm -rf include/
catkin build moveit_visual_tools
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
sudo rm /usr/include/Eigen 
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
catkin build moveit_visual_tools
grep -R "DESTDIR" ./*
grep -R "local/include" ./*
sudo cmake -DCMAKE_INSTALL_PREFIX=/usr/include ..
grep -R "local/include" ./*
cd ..
sudo rm -rf build/
sudo mkdir build 
cd build/
sudo cmake -DCMAKE_INSTALL_PREFIX=/usr/include ..
grep -R "local/include" ./*
sudo make 
sudo make install
cd ..
sudo rm -rf build/
sudo mkdir build 
cd build/
sudo cmake -DCMAKE_INSTALL_PREFIX=/usr ..
sudo make 
sudo make install
cd ..
sudo chmod +x eigen3/
sudo chmod +x eigen3/*
sudo chmod +x -R eigen3
sudo chmod +x -R eigen3/*
sudo rm -rf eigen3/
sudo rm -rf Eigen 
sudo tar -xzvf eigen-3.3.9.tar.gz 
cd eigen-3.3.9/
sudo mkdir build
cd build/
cmake ..
sudo cmake ..
sudo make DESTDIR=./new/customized/path install
./configure --prefix=/usr/include
ls
sudo ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
catkin build moveit_visual_tools
catkin build pr2_robot 
cd /home/brky/workspaces/fjnunes_ws/build/pr2_robot; catkin build --get-env pr2_robot | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build moveit_visual_tools 
source ~/.bashrc 
cd /home/brky/workspaces/fjnunes_ws/build/pr2_robot; catkin build --get-env pr2_robot | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin clean moveit_visual_tools 
catkin build moveit_visual_tools 
catkin clean
catkin build
source ~/.bashrc 
catkin build
cd eigen
cd eigen-3.3.9/build/
cd ..
sudo rm -rf build/
mkdir build
sudo mkdir build
cd build/
sudo cmake ..
sudo make install
htop
./install-ompl-ubuntu.sh.in 
git clone https://github.com/ros-planning/moveit_visual_tools.git
cd ..
catkin build moveit_visual_tools 
git clone https://github.com/ros-planning/moveit_visual_tools.git -b indigo-devel
catkin build moveit_visual_tools 
catkin clean
catkin build
catkin clean
cd src/
git clone https://github.com/ros-planning/geometric_shapes.git -b kinetic-devel
cd ..
catkin build geometric_shape
catkin build geometric_shapes
catkin build 
sudo apt-get install ros-kinetic-pybind11-catkin 
catkin build 
catkin clean
catkin build 
catkin clean
catkin build 
cd /home/brky/workspaces/move_it_ws/build/moveit_visual_tools; catkin build --get-env moveit_visual_tools | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin clean
catkin build moveit_visual_tools
source ~/.bashrc 
catkin build moveit_visual_tools
catkin clean
catkin build 
catkin clean
catkin build 
cd /home/brky/workspaces/move_it_ws/build/moveit_planners_ompl; catkin build --get-env moveit_planners_ompl | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build 
sudo apt-get install ros-kinetic-moveit-visual-tools 
catkin build 
catkin build moveit_tutorials 
cd /home/brky/workspaces/move_it_ws/build/moveit_tutorials; catkin build --get-env moveit_tutorials | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
sudo apt-get remove ros-kinetic-moveit-visual-tools 
git clone -b kinetic-devel https://github.com/ros-planning/moveit_tutorials.git
git clone https://github.com/ros-planning/moveit_visual_tools.git -b kinetic-devel
catkin build moveit_visual_tools 
catkin build moveit_tutorials 
roscd moveit_visual_tools>
roscd moveit_visual_tools
source ~/.bashrc 
roscd moveit_visual_tools
catkin build moveit_tutorials 
git clone https://github.com/ros-planning/moveit_visual_tools.git -b melodic-devel
catkin build moveit_visual_tools 
roslaunch aruco_detect aruco_detect.launch 
roslaunch armsim_config demo.launch 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
tmux
sudo apt-get install libeigen3-dev
sudo dpkg -r --force-depends libeigen3-dev
source ~/.bashrc 
sudo apt-get install libeigen3-dev
catkin clean
catkin build
cd ..
cd fjnunes_ws/
catkin build
roslaunch aruco_detect aruco_detect.launch 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch armsim_config demo.launch 
tmux
catkin build
catkin clean
catkin build
catkin config --blacklist moveit_visual_tools 
catkin build
catkin config --blacklist moveit_visual_tools moveit_tutorials
catkin build
catkin config --blacklist moveit_tutorials
catkin build
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
grep -R "moveit_resources" ./
mkdir -p panda_pick_place/src
rmdir panda_pick_place/
rm -rf panda_pick_place/
mkdir -p panda_pick_place_ws/src
cd panda_pick_place_ws/
catkin build
cd..
cd ..
cd fjnunes_ws/src/
grep -R "find_package(moveit_resources REQUIRED)" ./*
cd ..
catkin build
catkin build moveit_resources
cd src/
grep -R "find_package(moveit_resources REQUIRED)" ./*
cd ..
catkin clean
catkin_make
catkin build
catkin_clean 
catkin build
catkin build moveit
catkin build moveit_resources
catkin build moveit_planners_ompl 
catkin_make
catkin build
catkin_clean
catkin clean
catkin build
cd src/
catkin_init_workspace
cd ..
catkin_make
catkin_clean
catkin clean
catkin clean --workspace
./install-ompl-ubuntu.sh
catkin build
cd /home/brky/workspaces/panda_pick_place_ws/build/moveit_planners_ompl; catkin build --get-env moveit_planners_ompl | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build
cd src/
git clone https://github.com/ros-planning/panda_moveit_config.git -b kinetic-devel
cd ..
catkin build
rosrun moveit_tutorials pick_place_tutorial
roslaunch panda_moveit_config demo.launch
tmux
catkin build
rosrun moveit_tutorials pick_place_tutorial
roslaunch panda_moveit_config demo.launch
tmux
catkin build
rosdep install --from-paths ./src --ignore-src --rosdistro=kinetic
catkin clean
catkin build
cd ..
cd panda_pick_place_ws/
catkin build
git clone https://github.com/ros-planning/moveit_tutorials.git -b kinetic-devel
cd ..
catkin build
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
tmux
roslaunch aruco_detect aruco_detect.launch 
roslaunch armsim_config demo.launch 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
htop
ipconfig
ifconfig
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
tmux
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
grep -R "actionlib::SimpleActionServer<moveit_msgs::PickupAction> " ./*
grep -R "namespace pick_place " ./*
grep -R "pick_place:: " ./*
grep -R "pick_place\:\: " ./*
grep -R "pick_place " ./*
grep -R "PickPlace " ./*
grep -R "PlacePlanPtr planPlace( " ./*
grep -R "PlacePlanPtr planPlace " ./*
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
source ~/.bashrc 
rosrun moveit_tutorials pick_place_tutorial
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
source ~/.bashrc 
roslaunch panda_moveit_config demo.launch
ifconfig
ipconfig
ifconfig
ping 192.168.66.2
ssh airlab@192.168.66.2
python
rosrun aruco_detect create_markers.py 100 112 fiducials.pdf
roscore
rosrun aruco_detect create_markers.py 100 112 fiducials.pdf
rosrun aruco_detect create_markers.py 100 102 fiducials.pdf --marker-size=10
rosrun aruco_detect create_markers.py 100 102 fiducials.pdf --marker-size=15
rosrun aruco_detect create_markers.py  123 fiducials.pdf --marker-size=15
rosrun aruco_detect create_markers.py  123 123 fiducials.pdf --marker-size=15
rosrun aruco_detect create_markers.py  167 167 fiducials.pdf --marker-size=15
rosrun aruco_detect create_markers.py  175 175 fiducials.pdf --marker-size=15
roslaunch fiducial_slam fiducial_rviz.launch
roslaunch aruco_detect aruco_detect.launch
rviz
rosrun rviz rviz 
roslaunch realsense2_camera rs_camera.launch 
tmux
roslaunch fiducial_slam fiducial_rviz.launch
roslaunch aruco_detect aruco_detect.launch
rosrun rqt_tf_tree  rqt_tf_tree 
rocore
roscore
ifconfig
python wrist_ros_ads.py 
python wrist_ros_ads.py 1
python wrist_ros_ads.py 2
python wrist_ros_ads.py 1
python wrist_ros_ads.py 
python wrist_ros_ads.py 1
python wrist_ros_ads.py 
python wrist_ros_ads.py 1
python wrist_ros_ads.py 
python wrist_ros_ads.py 1
python wrist_ros_ads.py 
python wrist_ros_ads.py 1
python wrist_ros_ads.py 
python wrist_ros_ads.py 1
python wrist_ros_ads.py 
python wrist_ros_ads.py 1
python wrist_ros_ads.py 
python wrist_ros_ads.py 2
python wrist_ros_ads.py 
ifconfig
rostopic list
rosrun rviz rviz 
source ~/.bashrc 
rosrun rviz rviz 
rostopic hz /camera/color/camera_info 
sudo gedit /etc/hosts
rostopic hz /camera/color/camera_info 
sudo reboot
roscore
rostopic list
rostopic hz /camera/color/camera_info 
python wrist_ros_ads.py 
catkin_create_pkg ik_vision roscpp tf2 tf2_ros
cd ik_vision/
roscd tf2_ros/
rosrun ik_vision analizer
python wrist_ros_ads.py 1
python wrist_ros_ads.py 2
python wrist_ros_ads.py 3
python wrist_ros_ads.py 4
python wrist_ros_ads.py 
python wrist_ros_ads.py 2
python wrist_ros_ads.py 3
python wrist_ros_ads.py 4
python wrist_ros_ads.py 
python wrist_ros_ads.py 2
python wrist_ros_ads.py 4
python wrist_ros_ads.py 
python wrist_ros_ads.py 4
python wrist_ros_ads.py 
python wrist_ros_ads.py 1
python wrist_ros_ads.py 2
python wrist_ros_ads.py 
python wrist_ros_ads.py 1
rosrun ik_vision ik_vision_node 
rosrun rqt_tf_tree rqt_tf_tree 
rostopic echo /fiducial_transforms -n1
rosrun tf tf_monitor
rosrun tf tf_monitor /fiducial_175 /world
rosrun tf tf_monitor /fiducial_175 /camera_link
rosrun tf tf_monitor
rosrun tf tf_monitor /fiducial_175 /world
rostopic list
python wrist_ros_ads.py 4
python wrist_ros_ads.py 2
python wrist_ros_ads.py 
python wrist_ros_ads.py 2
ping 192.168.66.29 
python wrist_ros_ads.py 1
python wrist_ros_ads.py 2
python wrist_ros_ads.py 
python wrist_ros_ads.py 4
python wrist_ros_ads.py 1
python wrist_ros_ads.py 
python wrist_ros_ads.py 1
python wrist_ros_ads.py 2
python wrist_ros_ads.py 3
python wrist_ros_ads.py 1
python wrist_ros_ads.py 
python wrist_ros_ads.py 4
python wrist_ros_ads.py 3
python wrist_ros_ads.py 2
python wrist_ros_ads.py 
python wrist_ros_ads.py 4
python wrist_ros_ads.py 
python wrist_ros_ads.py 3
python wrist_ros_ads.py 
python wrist_ros_ads.py 2
python wrist_ros_ads.py 
python wrist_ros_ads.py 2
python wrist_ros_ads.py 1
python wrist_ros_ads.py 3
python wrist_ros_ads.py 4
rosrun ik_vision ik_vision_node
source ~/.bashrc 
rosrun ik_vision ik_vision_node
source ~/.bashrc 
rosrun ik_vision ik_vision_node
source ~/.bashrc 
rosrun ik_vision ik_vision_node
source ~/.bashrc 
roslaunch ik_vision analizer.launch 
source ~/.bashrc 
roslaunch ik_vision analizer.launch 
source ~/.bashrc 
roslaunch ik_vision analizer.launch 
source ~/.bashrc 
roslaunch ik_vision analizer.launch 
source ~/.bashrc 
roslaunch ik_vision analizer.launch 
roslaunch aruco_detect aruco_detect.launch 
rosrun rviz rviz 
tmux
roscore
source ~/.bashrc 
roscore
tmux
python wrist_ros_ads.py 
python wrist_ros_ads.py 3
python wrist_ros_ads.py 
python wrist_ros_ads.py 4
python wrist_ros_ads.py 
python wrist_ros_ads.py 3
python wrist_ros_ads.py 4
python wrist_ros_ads.py 
python wrist_ros_ads.py 2
python wrist_ros_ads.py 1
python wrist_ros_ads.py 
python wrist_ros_ads.py 1
python wrist_ros_ads.py 
ping 192.168.66.29
python wrist_ros_ads.py 
roscore
roscore
tmux
rosrun rviz rviz 
htop
rosparam use_sim_time true
rosparam set use_sim_time true
rosrun rviz rviz 
cd workspaces/fjnunes_ws/src/ik_vision/bags/
roscore
htop
tmux
sudo reboot
rosbag play 2021-05-11-11-26-17_wrist_ik_camera.bag --clock
roslaunch aruco_detect aruco_detect.launch 
rviz
roscor
roscore
tmux
rosrun rviz rviz 
rosrun rviz rviz -d /home/brky/.rviz/ik_wrist.rviz
roscore
cd workspaces/fjnunes_ws/src/ik_vision/bags/
rosbag play 2021-05-11-11-26-17_wrist_ik_camera.bag --clock
roslaunch aruco_detect aruco_detect.launch 
rosrun rqt_tf_tree rqt_tf_tree 
rosrun rviz rviz -d /home/brky/.rviz/ik_wrist.rviz
cd bags/
rosbag play 2021-05-11-11-26-17_wrist_ik_camera.bag --clock
roscore
roslaunch aruco_detect aruco_detect.launch 
roslaunch ik_vision analizer.launch 
tmux
rosrun rviz rviz 
roslaunch ik_vision analizer.launch 
rosrun rviz rviz 
roscore
python
rosrun rqt_tf_tree rqt_tf_tree 
cd workspaces/
grep -R "use_sim_time" ./*
rosparam get /use_sim_time
rosrun rqt_tf_tree rqt_tf_tree 
ping 192.168.66.29
python wrist_ros_ads.py 
roslaunch ik_vision analizer.launch 
source ~/.bashrc 
roslaunch ik_vision analizer.launch 
source ~/.bashrc 
roslaunch ik_vision analizer.launch 
source ~/.bashrc 
roslaunch ik_vision analizer.launch 
source ~/.bashrc 
roslaunch ik_vision analizer.launch 
source ~/.bashrc 
roslaunch ik_vision analizer.launch 
sudo apt install ros-kinetic-plotjuggler-ros
sudo apt install ros-kinetic-plotjuggler*
source ~/.bashrc 
roslaunch ik_vision analizer.launch 
source ~/.bashrc 
roslaunch ik_vision analizer.launch 
rostopic list
rosrun rqt_plot rqt_plot 
rosrun rqt_gui rqt_gui 
source ~/.bashrc 
roscore
tmux
sudo apt-get install ros-kinetic-plotjuggler*
rosrun plotjuggler PlotJuggler 
roslaunch ik_vision analizer.launch 
python create_markers.py 
python create_markers.py  167 167 fiducial_167.pdf 
roslaunch ik_vision analizer.launch 
ifconfig
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch ik_vision analizer.launch 
python create_markers.py  175 175 fiducial_175.pdf 
roslaunch ik_vision analizer.launch 
python wrist_ros_ads.py 
ifconfig
ping 192.168.66.2
ssh airlab@192.168.66.2
roslaunch ik_vision analizer.launch
tmux
roslaunch ik_vision analizer.launch
ssh airlab@192.168.66.2
rostopic list
rostopic echo /tf
roslaunch ik_vision analizer.launch
htop
rostopic list
rostopic echo /qr_pose 
rosrun rqt_tf_tree rqt_tf_tree 
rosrun tf tf_echo /world /qr_reference
rosrun tf tf_echo /world /active_qr
rosrun tf tf_echo /world /camera_color_optical_frame
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
rosrun rqt_tf_tree rqt_tf_tree 
tmux
rostopic list
rostopic echo /joint_states -n1
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
roslaunch armsim_config demo.launch 
roslaunch armsim_config demo_gazebo.launch 
roslaunch armsim_config demo.launch 
catkin build
roslaunch armsim_config demo.launch 
roscd armsim
catkin build
cd /home/brky/workspaces/catkin_ws/build/armsim; catkin build --get-env armsim | catkin env -si  /usr/bin/make cmake_check_build_system; cd -
catkin clean armsim
catkin build
roslaunch armsim display.launch 
roscd armsim
roslaunch armsim display.launch 
roslaunch armsim gazebo.launch 
source ~/.bashrc 
rostopic list
rostopic echo /arm/arm_position_controller/state -n1
rostopic echo /joint_states -n1
rostopic list
rostopic echo /joint_states -n1
roslaunch armsim_config demo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch armsim_config demo.launch 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
rosrun tf tf_echo /world /reference_qr
rosrun tf tf_echo /world /qr_reference
rosrun tf tf_echo /world /f
rosrun tf tf_echo /world /qr_reference
rosrun tf tf_echo /world /fiducial_175
rosrun tf tf_echo /world /qr_reference
rosrun tf tf_echo /world /fiducial_175
rosrun tf tf_echo /world /qr_reference
rosrun tf tf_echo /world /active_qr
rosrun tf tf_echo /qr_reference /active_qr
rosrun tf tf_echo /world /active_qr
rosrun tf tf_echo /qr_reference /active_qr
catkin build ik_vision
rosrun tf tf_echo /qr_reference /active_qr
rosrun tf tf_echo /qrworld /active_qr
rosrun tf tf_echo /world /active_qr
rosrun tf tf_echo /qr_reference /active_qr
rosrun tf tf_echo /world /qr_reference
rosrun tf tf_echo / /world /active_qr
rosrun tf tf_echo /world /active_qr
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch ik_vision analizer.launch
catkin build ik_vision
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
roslaunch moveit_setup_assistant setup_assistant.launch 
python wrist_ros_ads.py 
roslaunch ik_vision analizer.launch
rosclean purge 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
roslaunch ik_vision analizer.launch
source ~/.bashrc 
rosrun plotjuggler PlotJuggler 
clear
rostopic list
rostopic echo /camera/color/camera_info 
tmux
catkin build ik_vision
rosrun ik_vision wrist_ros_ads_with_encoder.py 
catkin build ik_vision
cd workspaces/fjnunes_ws/
catkin build ik_vision
rosrun ik_vision wrist_ros_ads_with_encoder.py 
catkin build ik_vision
rosrun ik_vision dummy_threading.py
cd ros_ads/
python wrist_ros_ads.py 
python wrist_ros_ads_with_encoder.py 
ping 192.168.66.29
python wrist_ros_ads_with_encoder.py 
cd ..
roscore
htop
python dummy_threading.py 
htop
roscore
ping 192.168.66.29
python wrist_ros_ads_encoders.py 
ping 192.168.66.2
python wrist_ros_ads_encoders.py 
ping 192.168.66.2
python wrist_ros_ads_encoders.py 
python d.py 
htop
python d.py 
rosrun ik_vision wrist_ros_ads_with_encoder.py 
roscore
htop
python wrist_ros_ads.py 
python wrist_ros_ads_encoder.py 
cd workspaces/fjnunes_ws/src/ros_ads/
ls
python wrist_ros_ads_encoder.py 
htop
rostopic list
rostopic echo /wrist_encoder_rpy 
python wrist_ros_ads_encoder.py 
roscore
python wrist_ros_ads.py 
cd ..
catkin build
htop
python wrist_ros_ads.py 
roscore
rosrun ik_vision wrist_ros_ads_encoder.py 
rostopic echo /wrist_encoder_rpy 
python wrist_ros_ads.py 
htop
rostopic list
rosrun ik_vision wrist_ros_ads_encoder.py 
roslaunch ik_vision analizer.launch 
python wrist_ros_ads.py 
ping 192.168.66.29
python wrist_ros_ads.py 
rosrun tf tf_echo /fiducial_167 /fiducial_175
rosrun tf tf_echo /qr_reference /fidual_175
rosrun tf tf_echo /qr_reference /fiducial_175
roslaunch ik_vision analizer.launch 
./mtsdk_linux-x64_2020.0.2.sh 
sudo apt-get install sharutils
./mtsdk_linux-x64_2020.0.2.sh 
rostopic list
rostopic info /imu/data -n1
rostopic echo /imu/data -n1
rostopic info /im
rostopic echo /imu/data -n1
rostopic echo /filter/quaternion -n1
roslaunch xsens_mti_driver example.launch
roslaunch xsens_mti_driver xsens_mti_node.launch 
pushd src/xsens_ros_mti_driver/lib/xspublic && make && popd
pushd ./src/xsens_ros_mti_driver/lib/xspublic && make && popd
catkin build
source devel/setup.bash
roslaunch xsens_mti_driver display.launch
rostopic list
rostopic echo /imu/a
rostopic echo /imu/acceleration 
groups
ls -l /dev/ttyUSB0
roslaunch xsens_mti_driver display.launch 
python example_mti_receive_data.py 
python3 example_mti_receive_data.py 
make src_cpp
make xda_cpp
./example_mti_receive_data 
roslaunch xsens_mti_driver display.launch 
cd workspaces/fjnunes_ws/src/ros_
cd workspaces/fjnunes_ws/src/ros_ads/
ls
python wrist_ros_ads.py 
rosrun ik_vision wrist_ros_ads_encoder.py 
roscore
rostopic echo /imu/data 
rostopic list
rostopic info /tf
rostopic echo /tf -n1
rostopic echo /tf 
rostopic list
rostopic echo /imu/time_ref 
rostopic list
rostopic echo /filter/twist 
rostopic echo /filter/quaternion 
rostopic echo /imu/data 
rostopic info /imu/data 
rostopic echo /wrist_encoder_rpy 
cd workspaces/fjnunes_ws/
catkin build ik_vision 
rosrun ik_vision ik_wrist_csv_writer.py 
source devel/setup.bash
source ~/.bashrc 
clear
rosrun ik_vision ik_wrist_csv_writer.py 
roslaunch xsens_mti_driver example.launch
roslaunch xsens_mti_driver xsens_mti_node.launch 
roslaunch xsens_mti_driver display.launch 
source devel/setup.bash
source ~/.bashrc 
roslaunch xsens_mti_driver display.launch 
clear
roscore
rostopic list
rostopic echo /imu/data -n1
rostopic echo /filter/quaternion -n1
rostopic echo /temperature -n1
tmux
htop
rosrun ik_vision wrist_ros_ads_encoder.py 
roslaunch xsens_mti_driver xsens_mti_node.launch 
roslaunch xsens_mti_driver display.launch 
python wrist_ros_ads.py 
rostopic echo /imu/data 
rostopic echo /wrist_encoder_rpy 
rostopic list
source ~/.bashrc 
rostopic list
rostopic echo /wrist_encoder_rpy 
rostopic list
rostopic echo /wrist_encoder_rpy 
rostopic echo /imu/data 
rosnode info ik_wrist_csv_writer_node
rosnode list
rosrun ik_vision ik_wrist_csv_writer.py 
rosrun ik_vision ik_wrist_csv_writer.py 
rscore
tmux
roscore
rosbag record -a
rosbag play imu_encoder.bag --clock
rsocore
roscore
rosrun rviz rviz 
rostopic echo /wrist_encoder_rpy 
cd bags/imu_encoder/
rosbag play imu_encoder.bag --clock
clear
rosbag play imu_encoder.bag --clock
rostopic list
rostopic echo /wrist_encoder_rpy 
rosrun ik_vision ik_wrist_csv_writer.py 
clear
rosrun ik_vision ik_wrist_csv_writer.py 
clear
rosrun ik_vision ik_wrist_csv_writer.py 
roscore
cd ..
tmux
rostopic echo /imu/data 
rostopic echo /imu/data/linear_acceleration
roslaunch xsens_mti_driver display.launch 
rosrun ik_vision ik_wrist_csv_writer.py 
rosbag play imu_encoder.bag --clock
cd imu_encoder/
rosbag play imu_encoder.bag --clock
clear
rosbag play imu_encoder.bag --clock
tmux
roscore
rosrun ik_vision wrist_ros_ads_encoder.py 
source ~/.bashrc 
rosrun ik_vision wrist_ros_ads_encoder.py 
python wrist_ros_ads.py 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
rostopic echo /imu/data -n1
roslaunch armsim_config demo.launch 
catkin_create_pkg charuco rospy cv2
cd ..
catkin build
python
cd ..
catkin clean charuco
catkin_create_pkg charuco rospy
cd src/
catkin_create_pkg charuco rospy
cd ..
catkin build 
python3 charuco_creater.py 
python charuco_creater.py 
catkin build
python wrist_ros_ads.py 
catkin build ik_vision 
python wrist_ros_ads.py 
rosrun ik_vision qr_pose_listener.py 
python dummy.py 
roslaunch charuco_detector ChArUco-A4_5-7.launch 
python dummy.py 
git clone https://github.com/ItzMeJP/charuco_creator.git
cd ..
catkin build charuco_creator 
roslaunch charuco_detector ChArUco-A4_5-7.launch 
python dummy.py 
python wrist_ros_ads.py 
rosrun ik_vision qr_pose_listener.py 
rosrun tf tf_echo /camera_link /charuco
python wrist_ros_ads.py 
rosrun tf tf_echo /camera_link /charuco
rosrun rqt_tf_tree rqt_tf_tree 
python wrist_ros_ads.py 
rosrun ik_vision qr_pose_listener.py 
htop
rosrun ik_vision wrist_ros_ads_encoder.py 
roslaunch ik_vision charuco_analizer.launch 
source ~/.bashrc 
roslaunch ik_vision charuco_analizer.launch 
source ~/.bashrc 
roslaunch ik_vision charuco_analizer.launch 
tmux
roslaunch ik_vision charuco_analizer.launch 
rosrun ik_vision ik_wrist_csv_writer.py 
rostopic echo /wrist_encoder_rpy 
rosrun ik_vision ik_wrist_csv_writer.py 
rosrun plotjuggler PlotJuggler 
tmux 
catkin build ik_vision 
rosrun plotjuggler PlotJuggler 
roslaunch ik_vision charuco_analizer.launch 
python wrist_ros_ads.py 
rostopic list
rosrun ik_vision wrist_ros_ads_encoder.py 
rosrun rviz rviz 
rostopic echo /wrist_encoder_rpy 
rosbag play 2021-06-02-10-55-18_charuco_centered_encoder.bag --clock
roscore
rosrun plotjuggler PlotJuggler 
roslaunch ik_vision charuco_analizer.launch 
source ~/.bashrc 
roslaunch ik_vision charuco_analizer.launch 
source ~/.bashrc 
roslaunch ik_vision charuco_analizer.launch 
roslaunch ik_vision charuco_analizer.launch 
rosrun tf tf_echo /world /charuco
roslaunch ik_vision charuco_analizer.launch 
htop
rostopic echo /wrist_encoder_
rostopic echo /
rostopic echo /qr_pose 
rosrun ik_vision wrist_ros_ads_encoder.py 
rostopic echo /wrist_encoder_rpy 
rostopic echo /camera/color/image_raw_charuco_pose 
rosrun ik_vision qr_pose_listener.py 
rosrun ik_vision ik_vision_node 
roslaunch ik_vision charuco_analizer.launch 
tmux
python wrist_ros_ads.py 
htop
rosrun ik_vision wrist_ros_ads_encoder.py 
roslaunch ik_vision charuco_analizer.launch 
rosrun plotjuggler PlotJuggler 
roslaunch ik_vision charuco_analizer.launch 
htop
rostopic echo /wrist_encoder_rpy 
rostopic echo /qr_pose 
rosrun plotjuggler PlotJuggler 
rosrun ik_vision ik_wrist_csv_writer.py 
roslaunch ik_vision charuco_analizer.launch 
python dummy.py 
rostopic echo /wrist_encoder_rpy 
rostopic list
rostopic echo /imu/data 
roslaunch ik_vision imu_analizer.launch 
rostopic echo /imu/data 
rostopic echo /wrist_encoder_rpy 
roslaunch ik_vision imu_analizer.launch 
rostopic echo /imu/data 
tmux
python wrist_ros_ads.py 
rosrun ik_vision wrist_ros_ads_encoder.py 
htop
python wrist_ros_ads.py 
rostopic echo /wrist_encoder_rpy 
roslaunch ik_vision imu_analizer.launch 
python wrist_ros_ads.py 
ping 192.168.66.29
python wrist_ros_ads.py 
python vedat_bey_ik_wrist_csv_writer.py 
python wrist_ros_ads.py 
roslaunch armsim_config demo_gazebo.launch 
mkdir -p ./real_ws/src
cd air_gazebo_ws/src
catkin build
cd ..
mkdir -p ./air_gazebo_ws/src
cd air_gazebo_ws/
catkin build
cd ..
mkdir -p ./airarm_gazebo_ws/src
cd airarm_gazebo_ws/
catkin build
mkdir -p ./airarm_real_ws/src
cd ..
mkdir -p ./airarm_real_ws/src
cd airarm_real_ws/
catkin build
roslaunch armsim_config demo.launch 
roslaunch armsim_config demo_gazebo.launch 
roslaunch armsim gazebo.launch 
roslaunch armsim_config demo_gazebo.launch 
roslaunch pr2_robot pick_place_project.launch 
roslaunch armsim_config demo_gazebo.launch 
rostopic list
rostopic echo /arm/joint_states 
rosparam get /use_sim_time 
tmux
./scripts/patch-realsense-ubuntu-lts.sh
catkin build
htop
./scripts/patch-realsense-ubuntu-lts.sh
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release
sudo make install
cmake ..
cmake .
make -j12
cd ..
mkdir build
cd build/
cmake ..
make -j12
ping google.com
make -j12
sudo rm -rf build/
./scripts/patch-realsense-ubuntu-lts.sh
mkdir build && cd build
cmake ../
sudo make uninstall && make clean && make && sudo make install
make clean && make -j12 && sudo make install
sudo make uninstall && make clean && make -j8 && sudo make install
sudo make uninstall
mkdir build
cd build/
cmake ..
make -j12
sudo make uninstall
cmake ..
sudo make -j12
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense/
./scripts/patch-realsense-ubuntu-lts.sh
mkdir build
cd build/
cmake .
cmake ..
make -j12
sudo make install
mkdir build
cd build/
cmake ..
make -j12
catkin build
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense/
mkdir build
cd build/
cmae ..
cmake ..
make -j12
sudo make install
catkin build
roscd armsim_config 
roslaunch armsim_config demo_gazebo.launch 
rostopic echo /arm/arm_position_controller/state 
roscd armsim_config/
roslaunch armsim_config demo.launch 
rosrun tf tf_echo /s1 /s2
rosrun tf tf_echo /s3 /e
rosrun tf tf_echo /s1 /s2
catkin build
catkin clean
catkin build
grep -R "arm_produced.urdf.xacro" ./*
roscd armsim
tmux
roscd armsim
catkin_create_pkg ik_acc_vel_analizer rospy
catkin build ik_acc_vel_analizer
rostopic list
rostopic echo /move_group/goal 
rostopic list
rostopic echo /arm/arm_position_controller/follow_joint_trajectory/goal
rostopic echo /arm/arm_position_controller/command
clear
rostopic list
rostopic echo /move_group/display_planned_path
clear
rostopic echo /move_group/display_planned_path
roslaunch armsim_config demo_gazebo.launch 
rosrun tf tf_echo /s1 /s2
rosrun tf tf_echo /s2 /s3
rosrun tf tf_echo /s3 /e
rosrun tf tf_echo /s1 /s2
source ~/.bashrc 
rosrun ik_acc_vel_analizer ik_acc_vel_analizer.py
rosrun plotjuggler PlotJuggler 
rosrun ik_acc_vel_analizer ik_acc_vel_analizer.py
tmux
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin clean armsim
catkin clean armsim_config
catkin build
roscd armsim
rosrun tf tf_echo /s1 /s2
rosrun tf tf_echo /s2 /s3
rosrun tf tf_echo /s3 /e
rosrun tf tf_echo /s3 /roll
rosrun tf tf_echo /e /roll
rosrun tf tf_echo /e /pitch
rosrun tf tf_echo /pitch /yaw
rosrun tf tf_echo /s1 /s2
rosrun tf tf_echo /s2 /s3
rosrun tf tf_echo /s3 /roll
rosrun tf tf_echo /e /roll
rosrun tf tf_echo /pitch /roll
rosrun tf tf_echo /e /roll
rosrun tf tf_echo /pitch /yaw
roslaunch armsim_config demo_gazebo.launch 
rosrun tf tf_echo /s1 /s2
rosrun tf tf_echo /s2 /s3
rosrun tf tf_echo /s3 /se
rosrun tf tf_echo /s3 /3
rosrun tf tf_echo /s3 /e
rosrun tf tf_echo /e /roll
rosrun tf tf_echo /roll /yaw
catkin build
rosrun tf tf_echo /roll /yaw
rosrun tf tf_echo /s1 /s2
roslaunch armsim_config demo.launch 
rosrun tf tf_echo /s1 /s2
catkin clean armsim
catkin buil
catkin build
roslaunch armsim_config demo.launch 
rosrun tf tf_echo /s1 /s2
roscd  armsim
roslaunch armsim_config demo.launch 
catkin build
rosrun tf tf_echo /s1 /s2
rosrun tf tf_echo /s2 /s3
rosrun tf tf_echo /s3 /e
rosrun tf tf_echo /e /roo
rosrun tf tf_echo /e /roll
rosrun tf tf_echo /roll /yaw
roslaunch armsim_config demo.launch 
sudo apt install ./code_1.57.0-1623259737_amd64.deb 
grep -R "action_monitor_rate" ./*
cd ..
grep -R "action_monitor_rate" ./*
cd ..
grep -R "action_monitor_rate" ./*
grep -R "allow_partial_joints_goal" ./*
sudo apt uninstall ./code_1.57.0-1623259737_amd64.deb 
sudo apt remove ./code_1.57.0-1623259737_amd64.deb 
sudo apt remove code 
grep -R "state_publish_rate" ./*
cd ..
grep -R "state_publish_rate" ./*
roscd armsim
roslaunch armsim_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch 
source ~/.bashrc 
roslaunch armsim_config demo_gazebo.launch 
grep -R "[BERKAY] ----- **** hardware_interface::starting **** -----" ./*
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
roscd armsim_config 
roslaunch armsim_config demo.launch 
catkin build ADS
cd build/
cmake ..
make install
sudo make install
gcc example.cpp -o ex
git clone https://github.com/Beckhoff/ADS.git
cd ADS/
meson build
sudo apt install meson
meson build
mkdir build
meson build
meson --version
sudo apt upgrade meson
meson build
sudo apt purge meson
python3 -m pip install meson
sudo python3 -m pip install meson
python3 -m pip install ninja
sudo python3 -m pip install ninja
meson build
sudo python3 -m pip uninstall meson
sudo python3 -m pip install meson==0.40.1
meson build
sudo rm -rf build/
meson build
ninja -C build
sudo ninja -C build
sudo python3 -m pip uninstall ninja
pip install ninja==1.8.2
ninja -C build
sudo ninja -C build
source ~/.bashrc 
sudo ninja -C build
ninja -C build
sudo rm -rf ./build/
ls
meson build
ninja -C build
pip3 install --user empy
ninja -C build
pip3 uninstall --user empy
pip3 uninstall  empy
ninja build
ninja -C build
sudo rm -rf ./build/
mkdir build
cd build/
mkdir build
sudo rm -rf ./build/
ls
cd ..
ninja -C build
meson example/build example
sudo python3 -m pip uninstall meson
sudo python3 -m pip install meson==0.53.2
meson example/build example
ninja -C example/build
meson build -Dcpp_std=c++11 example/build example
meson -Dcpp_std=c++11 example/build example
cd ..
meson -Dcpp_std=c++11 example/build example
ninja -C example/build
./example/build/example
git clone https://github.com/Beckhoff/ADS.git
cd ADS/
mkdir build
cd build/
cmake ..
make -j12
sudo make install
make install
meson -Dcpp_std=c++11 src/arm.cpp 
meson -Dcpp_std=c++11 src/
cd src/
mkdir build
meson -Dcpp_std=c++11 ./build/
./example/build/example
ls
cd ..
ls
git clone https://github.com/Beckhoff/ADS.git
cd ADS/
mkdir build
cd build/
cmake ..
make -j12
cd ..
cd example/
mkdir build
cd build/
cmake ..
make -j12
rosrun air_arm_ads air_ads_node.py
roscore
cd ..
catkin build
source ~/.bashrc 
rosmsg show air_arm_ads/Float64ArrayStamped 
rosrun air_arm_ads air_ads_node.py 
rostopic echo /ads_ros_encoder 
roscore
rosrun air_arm_ads air_ads_node.py 
rostopic echo /ads_ros_encoder 
python3 cmake2meson.py ./
python3 cmake2meson.py ./CMakeLists.txt 
python cmake2meson.py ./CMakeLists.txt 
git clone https://github.com/Beckhoff/ADS.git
cd ADS/
meson -Dcpp_std=c++11 ./build/
ninja -C build
meson -Dcpp_std=c++11 ./example/build example
ninja -C example/build
./example/build/example
grep -R "AmsNetId" ./*
cd workspaces/airarm_real_ws/
catkin build air_arm_robot 
meson -Dcpp_std=c++11 ./example/build example
ninja -C example/build
./example/build/example
roslaunch armsim_config demo.launch 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
meson -Dcpp_std=c++11 ./build/
ninja -C build
meson -Dcpp_std=c++11 ./build/
ninja -C build
meson -Dcpp_std=c++11 ./example/build example
meson -Dcpp_std=c++11 ./build/
ninja -C example/build
meson -Dcpp_std=c++11 ./example/build example
meson --reconfigure -Dcpp_std=c++11 ./example/build example
ninja -C example/build
./example/build/example
catkin clean air_arm_robot
catkin clean air_arm_hardware_interface
catkin build air_arm_robot air_arm_hardware_interface
catkin build air_arm_robot 
catkin build air_arm_robot air_arm_hardware_interface
catkin clean air_arm_hardware_interface air_arm_robot
catkin clean air_arm_hardware_interface 
catkin clean air_arm_robot
catkin build air_arm_robot air_arm_hardware_interface
cd /home/brky/workspaces/airarm_real_ws/build/air_arm_hardware_interface; catkin build --get-env air_arm_hardware_interface | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
catkin build air_arm_robot air_arm_hardware_interface
catkin clean air_arm_robot air_arm_hardware_interface
catkin build air_arm_robot air_arm_hardware_interface
catkin build air_arm_robot 
catkin build air_arm_hardware_interface 
cd /home/brky/workspaces/airarm_real_ws/build/air_arm_hardware_interface; catkin build --get-env air_arm_hardware_interface | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
nm  -C -u libair_arm_robot.so
nm  -C -u /home/brky/workspaces/airarm_real_ws/devel/.private/air_arm_robot/lib/libair_arm_robot.so
catkin build air_arm_robot 
nm  -C -u /home/brky/workspaces/airarm_real_ws/devel/.private/air_arm_robot/lib/libair_arm_robot.so
catkin build air_arm_hardware_interface 
catkin build air_arm_robot 
catkin build air_arm_hardware_interface 
catkin build 
catkin build air_arm_robot 
catkin build 
catkin build air_arm_hardware_interface 
cd /home/brky/workspaces/airarm_real_ws/build/air_arm_hardware_interface; catkin build --get-env air_arm_hardware_interface | catkin env -si  /usr/bin/make --jobserver-fds=6,7 -j; cd -
cd build/
sudo make install
cmake ..
make -j12
catkin build air_arm_robot 
catkin build air_arm_hardware_interface 
source ~/.bashrc 
catkin build air_arm_hardware_interface 
mkdir build
cd build/
cmake ..
make -j12
make install
meson -Dcpp_std=c++11 ./build/
ninja -C build
rosrun air_arm_robot arm.cpp 
catkin build air_arm_robot
roscore
sudo make all
make test
sudo rm -rf obj/
make all
catkin build air_arm_robot
oscore
catkin build air_arm_robot
catkin build air_arm_hardware_interface 
catkin build air_arm_robot
catkin build air_arm_hardware_interface 
catkin build air_arm_robot
catkin build air_arm_hardware_interface 
catkin clean air_arm_robot
catkin build air_arm_robot
source ~/.bashrc 
catkin build air_arm_hardware_interface 
catkin build air_arm_robot
catkin clean air_arm_robot
catkin build air_arm_robot
catkin build air_arm_hardware_interface 
catkin build air_arm_robot
cd ..
./example/build/example
source ~/.bashrc 
roslaunch armsim_config demo.launch 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
rostopic list
rostopic echo /joint_states 
rostopic list
rostopic info /joint_states 
rostopic echo /joint_states 
rostopic list
rostopic echo /joint_states 
rostopic list
rostopic echo /arm/arm_position_controller/state 
rostopic echo /joint_states -n1
rosnode list
rostopic info /joint_states 
rostopic echo /joint_states -n1
htop
meson --reconfigure -Dcpp_std=c++11 ./example/build example
ninja -C example/build
./example/build/example
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
source ~/.bashrc 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
source ~/.bashrc 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
source ~/.bashrc 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
source ~/.bashrc 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
source ~/.bashrc 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
rostopic echo /joint_states -n1
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
rostopic echo /joint_states 
rosrun arm_planned_path_to_matlab arm_planned_path_to_matlab.py 
roslaunch armsim_config demo.launch 
grep -R "<trajectory_msgs::JointTrajectory>("command"" *
grep -R "<trajectory_msgs::JointTrajectory>("command"" ./*
grep -R "<trajectory_msgs::JointTrajectory>("command" ./*
grep -R "<trajectory_msgs::JointTrajectory>(\"command" ./*
grep -R "<trajectory_msgs::JointTrajectory>" ./*
cd air_pr2_ws/src/"subscribe("chatter","
cd air_pr2_ws/src/
grep -R "subscribe(\"command" ./*
grep -R "RTGoalHandleFollow::runNonRT" ./*
grep -R "runNonRT" ./*
grep -R "<trajectory_msgs::JointTrajectory>" ./* 
cd ..
grep -R "<trajectory_msgs::JointTrajectory>" ./* 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
rostopic echo /joint_states 
rostopic list
rostopic echo /arm/arm_position_controller/state
rostopic echo /joint_states 
rostopic list
rostopic echo /move_group/display_planned_path
grep -R "execution_duration_monitoring_" ./*
grep -R "::getControllerHandle(" ./*
grep -R "MoveItControllerManager" ./*
clear
grep -R "waitForExecution(" ./*
rostopic echo /move_group/display_planned_path 
rostopic echo /joint_states 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
sudo dpkg -i virtualbox-6.1_6.1.22-144080~Ubuntu~xenial_amd64.deb 
sudo apt-get remove virtualbox-6.0 
sudo dpkg -i virtualbox-6.1_6.1.22-144080~Ubuntu~xenial_amd64.deb 
htop
catkin clean
catkin build
python vel_shoulder_ros_ads.py 
clear
python vel_shoulder_ros_ads.py 
roslaunch armsim_config demo.launch 
roslaunch realsense2_camera rs_camera.launch 
roslaunch armsim_config demo.launch 
rostopic echo /joint_states 
roslaunch moveit_tutorials move_group_interface_tutorial.launch 
roslaunch aruco_detect aruco_detect.launch 
tmux
roslaunch ik_vision charuco_analizer.launch 
roslaunch pr2_moveit_config demo.launch 
roscd pr2_navigation_config/
roslaunch /home/brky/workspaces/air_pr2_ws/src/pr2_navigation/pr2_navigation/launch/pr2_nav_tutorial.launch
source ~/.bashrc 
roslaunch /home/brky/workspaces/air_pr2_ws/src/pr2_navigation/pr2_navigation/launch/pr2_nav_tutorial.launch
roslaunch pr2_moveit demo.launch 
roslaunch pr2_moveit pr2_moveit.launch 
roslaunch pr2_gazebo pr2_nav_tutorial.launch 
roscd pr2_moveit/
roslaunch pr2_moveit demo.launch 
roslaunch pr2_moveit 
roslaunch pr2_moveit pr2_moveit.launch 
roslaunch pr2_gazebo pr2.launch 
roslaunch '/home/brky/workspaces/air_pr2_ws/src/pr2_navigation/pr2_navigation/launch/pr2_nav_tutorial.launch' 
roslaunch pr2_navigation_global rviz_move_base.launch
roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch pr2_tabletop_manipulation_gazebo_demo pr2_tabletop_manipulation_demo.launch
roslaunch /home/brky/workspaces/air_pr2_ws/src/pr2_navigation/pr2_navigation/launch/pr2_nav_tutorial.launch
roslaunch pr2_gazebo pr2_empty_world.launch
roscd pr2_navigation_global/
tmux,
tmux
roscd pr2_gazebo
grep -R "ROBOT=sim" ./*
grep -R "ROBOT=sim" ./*/src/
grep -R "a" ./*/src/
grep -R "CmakeList" ./*/src/
grep -R "Cmakelist" ./*/src/
grep -R "CMakeLists" ./*/src/
grep -R "ROBOT=sim" ./*/src/
grep -R "ROBOT" ./*/src/
grep -R "ROBOT=sim" ./*/src/
roslaunch pr2_gazebo pr2.launch
roslaunch '/home/brky/workspaces/air_pr2_ws/src/pr2_navigation/pr2_navigation/launch/pr2_nav_tutorial.launch' 
roslaunch pr2_teleop teleop_keyboard.launch 
roslaunch pr2_navigation_global rviz_move_base.launch
roslaunch pr2_tabletop_manipulation_gazebo_demo pr2_tabletop_manipulation_demo.launch
roslaunch pr2_gazebo pr2.launch
roslaunch gazebo_ros empty_world.launch
roslaunch pr2_gazebo pr2_empty_world.launch
tmux
roslaunch pr2_moveit_config demo.launch 
roscd pr2_moveit_config
roslaunch pr2_moveit_config tabletop_object_recognition.launch 
roslaunch pr2_moveit_config demo.launch
roslaunch pr2_gazebo pr2_table_object.launch
tmux
source ~/.bashrc 
roslaunch pr2_moveit_config demo.launch
roslaunch pr2_moveit demo.launch 
roslaunch pr2_robot robot_spawn.launch 
tmux
roslaunch pr2_gazebo pr2_table_object.launch
roslaunch pr2_gazebo pr2_table_world.launch 
tmux
roslaunch pr2_moveit_config tabletop_object_recognition.launch 
roslaunch pr2_gazebo pr2.launch 
roslaunch pr2_gazebo pr2_table_object.launch 
tmux
roslaunch pr2_gazebo pr2_table_object.launch 
roslaunch pr2_gazebo pr2_empty_world.launch 
roslaunch pr2_gazebo pr2_no_controllers.launch 
roslaunch pr2_moveit_config demo.launch 
tmux
roslaunch pr2_tabletop_manipulation_launch pr2_tabletop_manipulation.launch
roslaunch baxter_sim_examples baxter_pick_and_place_demo.launch 
roslaunch baxter_gazebo baxter_world.launch 
roslaunch pr2eus_tutorials pr2_tabletop_sim.launch physics:=dart
roslaunch pr2eus_tutorials pr2_tabletop_grasp_sim.launch
roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch pr2_gazebo pr2_wg_world.launch 
python vel_shoulder_ros_ads.py 
cd ..
catkin build 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
source ~/.bashrc 
roslaunch armsim_config demo.launch 
rostopic lsit
rostopic list
rostopic echo /joint_states 
source ~/.bashrc 
rostopic echo /joint_states 
source ~/.bashrc 
rostopic echo /move_group/display_planned_path 
tmux
python vel_arm_ros_ads.py 
python dummy.py 
roslaunch armsim_config demo.launch 
rostopic echo /joint_states 
python vel_arm_ros_ads.py 
rostopic echo /joint_states 
roslaunch armsim_config demo.launch 
python dummy.py 
mkdir -p ~/dummy_ws/src
mkdir -p ./dummy_ws/src
cd dummy_ws/
catkin_init_workspace 
catkin build
catkin_init_workspace 
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin_create_pkg arm_07072021
catkin build
rostopic list
rostopic echo /joint_states 
rosrun tf tf_echo /roll /yaw
rosrun tf tf_echo /base_link /s1
rosrun tf tf_echo /base_link /s2
rosrun tf tf_echo /base_link /s3
rosrun tf tf_echo /s1 /s3
rosrun tf tf_echo /s2 /s3
rosrun tf tf_echo /base_link /e
rosrun tf tf_echo /base_link /s3
rosrun tf tf_echo /base_link /roll
rosrun tf tf_echo /base_link /yaw
rosrun tf tf_echo /base_link /s1
rosrun tf tf_echo /base_link /s2
rosrun tf tf_echo /base_link /s3
rosrun tf tf_echo /base_link /e
rosrun tf tf_echo /base_link /roll
rosrun tf tf_echo /base_link /pitch
rosrun tf tf_echo /base_link /yaw
roslaunch arm_07072021 display.launch 
python wrist_read_roll_pitch_joint_states.py 
catkin build
roscd armsim_080721_1021/
rosrun tf tf_echo /base_link /s1
rosrun tf tf_echo /base_link /s2
rosrun tf tf_echo /base_link /s3
rosrun tf tf_echo /base_link /e
rosrun tf tf_echo /base_link /roll
rosrun tf tf_echo /base_link /pitch
rosrun tf tf_echo /base_link /yaw
rostopic  echo /joint_states 
tmux
roslaunch armsim_080721_1021 display.launch 
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch 
python vel_arm_ros_ads.py 
roslaunch armsim_config demo.launch 
rostopic echo /joint_states -n1

rosrun tf tf_echo /base_link /yaw
roslaunch armsim display.launch 
roscd qb_hand_control/
rosservice list
rosservice call /communication_handler/activate_motors {"id: 1, max_repeats: 0"}
rosservice list
roslaunch qb_hand_control control.launch use_controller_gui:=true
ls /dev/
clera
clear
ls /dev/
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=0
ls /dev/
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=0
roslaunch qb_hand_control control.launch use_controller_gui:=true
roslaunch qb_device_bringup robot_description_bringup.launch 
rosservice call /communication_handler/activate_motors {"id: 1, max_repeats: 0"}
rosservice list
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1
roslaunch qb_device_bringup robot_description_bringup.launch 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=0
rosservice call /communication_handler/activate_motors {"id: 1, max_repeats: 0"}
rosservice list
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [0.5]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [0.5]" 
grep -R "activate_on_initialization" ./
grep -R "use_simulator_mode" ./
rosservice call /communication_handler/activate_motors {"id: 1, max_repeats: 0"}
grep -R "qbhand1_synergy_trajectory_conoller" ./*
grep -R "qbhand1_synergy_trajectory_controller" ./*
grep -R "trajectory_controller" ./*
grep -R "synergy_trajectory_controller" ./*
rosservice list
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: 0" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [0 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [0 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [0.1 0.1]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [0.1 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [40 0.8]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [40 0.8]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [40 10]" 
rosnode list
rosnode info /qbhand1/controller_gui
rostopic echo /qbhand1/control/qbhand1_synergy_trajectory_controller/command
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [10 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [10 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [10 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [40 40]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [0.40 0.40]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [1 1]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [1]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [100]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [100 100]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [100,100]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [10,10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [1,1]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [1,100]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [100,1]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [0,0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [1,1]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [1]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [1]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [1]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [10,10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [10 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [1,1]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [100,100]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [10,10,10,10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [10,10,10,10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [10,10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [0.10,0.10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [1,1]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [100]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [-100]" 
rosservice args /communication_handler/set_commands 
rosservice args /communication_handler/set_commands commands 
rosservice args /communication_handler/set_commands/commands 
rosservice args /communication_handler/set_commands
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: false
set_commands_async: true
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [10000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [10000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [10000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [10000, 10000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [10000, 10000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [1000, 1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [10000, 10000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [1000, 1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [1000, 1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [1000, 1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [1000, 1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: false
commands: [10000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [10000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [10000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 0
set_commands: true
set_commands_async: true
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: true
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [1000]" 
ls /dev/
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=0
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1
roslaunch qb_hand_control control.launch activate_on_initialization:=true device_id:=1
roslaunch qb_hand_control control.launch activate_on_initialization:=false device_id:=1
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=false device_id:=1
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false
source ~/.bashrc 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false
source ~/.bashrc 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false
grep -R "/communication_handler/set_commands" ./*
/home/brky/workspaces/qb_hand_ws/src/qbdevice-ros/qb_device_driver/src/qb_device_communication_handler.cpp
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false
source ~/.bashrc 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false
source ~/.bashrc 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false
source ~/.bashrc 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=false
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [100]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [100, 100]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [0.79]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [14000, 930]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 2
set_commands: true
set_commands_async: false
commands: [14000, 930]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 2
set_commands: true
set_commands_async: false
commands: [14000, 30]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 2
set_commands: true
set_commands_async: false
commands: [14000, 3000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 2
set_commands: true
set_commands_async: false
commands: [100, 3000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 2
set_commands: true
set_commands_async: false
commands: [100, 3000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 2
set_commands: true
set_commands_async: false
commands: [10, 3000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 10
set_commands: true
set_commands_async: false
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 10
set_commands: true
set_commands_async: false
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 10
set_commands: true
set_commands_async: false
commands: [1000 1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 10
set_commands: true
set_commands_async: false
commands: [1000, 1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 10
set_commands: true
set_commands_async: false
commands: [120]" 
rosservice call /communication_handler/get_measurements "{id: 1, max_repeats: 10, get_positions: true, get_currents: true, get_distinct_packages: false}" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 10
set_commands: true
set_commands_async: false
commands: [4024, 0, -4284]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 10
set_commands: true
set_commands_async: false
commands: [4024,-4284]" 
rosservice info /communication_handler/set_commands 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 10
set_commands: true
set_commands_async: false
commands: [4024,-4284]" 
rostopic echo  /qbhand1/control/qbhand1_synergy_trajectory_controller/result
rostopic list
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- ''
points:
- positions: [1000]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [1000]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [1000]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [1000]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 100}" 
ostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [10000]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 10}" 
ostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [10000]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 10}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [10000]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 10}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [8000]
  velocities: [0.2]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [8000]
  velocities: [20]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 0, nsecs: 0}" 
rostopic list
rostopic echo /qbhand1/control/qbhand1_synergy_trajectory_controller/command
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [0.9]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 2, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [0.9]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 2, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [0.1]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 2, nsecs: 0}" 
source ~/.bashrc 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
grep -R "serviceServer<qb_device_srvs::SetCommands>" ./*
grep -R "<qb_device_srvs::SetCommands>" ./*
grep -R "/communication_handler/set_commands" ./*
grep -R "advertiseService("/communication_handler/set_commands"" ./*
grep -R "advertiseService("/communication_handler/set_commands" ./*
grep -R "advertiseService(\"/communication_handler/set_commands\"" ./*
grep -R "setCommandsAsync(const int" ./*
grep -R "setCommandsAsync(" ./*
grep -R "qbhand_controllers.yaml" ./*
grep -R "_controllers.yaml" ./*
clear
grep -R "_controllers.yaml" ./*
grep -R "device_controllers_bringup.launch" ./*
rosrun controller_manager controller_manager 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [7500]" 
rosservice call /communication_handler/set_commands "id: 0
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [7500]" 
rosservice call /communication_handler/set_commands "id: 2
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [7500]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [7500]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [7500]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1500]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1500]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 0
commands: [1500]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1500, 100]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1500,]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 
commands: [1500,]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 0
commands: [1500,]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 0
commands: [1500,]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 0
commands: [1500,]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 0
commands: [1500,]" 
rosrun controller_manager controller_manager list
rosrun controller_manager controller_manager 
rosrun controller_manager controller_manager list
rosrun rqt_controller_manager rqt_controller_manager
catkin build
rostopic echo /qbhand1/control/qbhand1_synergy_trajectory_controller/
rostopic echo /qbhand1/control/qbhand1_synergy_trajectory_controller/result
rostopic echo /qbhand1/control/qbhand1_synergy_trajectory_controller/state
rostopic echo /qbhand1/control/qbhand1_synergy_trajectory_controller/follow_joint_trajectory/result 
rostopic echo /qbhand1/control/qbhand1_synergy_trajectory_controller/follow_joint_trajectory/status 
grep -R "serviceClient<qb_device_srvs::GetMeasurements>" ./*
grep -R "serviceAdvertise<qb_device_srvs::GetMeasurements>" ./*
grep -R "serviceAdvertise" ./*
grep -R "advertiseService<qb_device_srvs::GetMeasurements>" ./*
grep -R "advertiseServic" ./*
rostopic echo /tf
rostopic echo /qbhand1/joint_states 
rostopic echo /qbhand1/control/qbhand1_synergy_trajectory_controller/follow_joint_trajectory/command
clear
rostopic list
source ~/.bashrc 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
source ~/.bashrc 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
source ~/.bashrc 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
source ~/.bashrc 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
roslaunch qb_hand_control control.launch standalone:=false activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
source ~/.bashrc 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
roslaunch qb_hand_control control.launch standalone:=false activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=false 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=false  use_waypoints:=true
roslaunch qb_hand_control control.launch activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_waypoints:=true
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_waypoints:=true
roslaunch qb_hand_control control.launch activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_waypoints:=true
source ~/.bashrc 
roslaunch qb_hand_control control.launch activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
cd ..
catkin build
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 0
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 0
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 999993
set_commands: 1
set_commands_async: 0
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 999
set_commands: 1
set_commands_async: 1
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 999
set_commands: 1
set_commands_async: 1
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 999
set_commands: 1
set_commands_async: 0
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 999
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 0
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 0
set_commands_async: 0
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 0
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 0
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 0
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 0
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 0
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 0
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 0
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 0.1]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 1]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 333
set_commands: 1
set_commands_async: 1
commands: [1000, 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1000, 0]" 
rostopic list
clear
rostopic list
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- ''
points:
- positions: [0.5]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 3, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [0.5]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 3, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [500]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 3, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [15000]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 10, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [15000]
  velocities: [0.3]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 1, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [15000]
  velocities: [30]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 1, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [5500]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 3, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [5500]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 3, nsecs: 0}" 
rostopic echo /qbhand1/control/qbhand1_synergy_trajectory_controller/command 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [0.9]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 3, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [0.9]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 3, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [0.1]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 3, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [0.9]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 3, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [0.1]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 3, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [0.1]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 10, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [0.9]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 10, nsecs: 0}" 
rostopic pub /qbhand1/control/qbhand1_synergy_trajectory_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'qbhand1_synergy_joint'
points:
- positions: [0.1]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 10, nsecs: 0}" 
clear
rosservice list
clear
rosservice list
rosservice call /qbhand1/set_async_commands "id: 1
max_repeats: 0
set_commands: false
set_commands_async: false
commands: [1500]" 
rosservice call /qbhand1/set_async_commands "id: 1
max_repeats: 0
set_commands: 1
set_commands_async: false
commands: [1500]" 
rosservice call /qbhand1/set_async_commands "id: 1
max_repeats: 0
set_commands: 1
set_commands_async: 1
commands: [1500]" 
rosservice call /qbhand1/set_async_commands "id: 1
max_repeats: 0
set_commands: 1
set_commands_async: 1
commands: [1500,0]" 
rosservice call /qbhand1/set_async_commands "id: 1
max_repeats: 0
set_commands: 1
set_commands_async: 1
commands: [1500,0]" 
clear
rosservice list
clear
rosservice call /qbhand1/get_async_measurements "{id: 1, max_repeats: 0, get_positions: 1, get_currents: 1, get_distinct_packages: false}" 
clear
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: false
commands: [0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: false
commands: [0]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: false
commands: [1110]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1110]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1110]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1110]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1110]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1110]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1110]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1110]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1110]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1110]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1110]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 3
set_commands: 1
set_commands_async: 1
commands: [1110]" 
rostopic list | grep -o -P '^.*(?=/feedback)'
rostopic info /qbhand1/control/qbhand1_synergy_trajectory_controller/follow_joint_trajectory/command
rostopic info /qbhand1/control/qbhand1_synergy_trajectory_controller/follow_joint_trajectory/goal
tmux
rosrun qb_hand_control tubitak_demo.py 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
ping 192.168.66.29
rostopic echo /qbhand1/joint_states 
htop
rosrun qb_hand_control tubitak_demo.py 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=false
rosrun qb_hand_control tubitak_demo.py 
tmux
roscore
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
ifconfig
catkin_create_pkg air_qr_moveit_demo roscpp
cd ..
catkin build
rostopic hz /camera/depth/image_rect_raw
rostopic info /camera/depth/image_rect_raw
rostopic info /camera/color/image_raw
rostopic hz /camera/color/image_raw
rostopic hz /camera/color/image_raw/compressed
rostopic hz /camera/color/image_raw
rostopic hz /camera/depth/image_rect_raw/compressed
clear
tmux
rostopic list
rosrun rviz rviz 
roscore
catkin build air_qr_moveit_demo 
catkin build a
catkin build 
roslaunch armsim_config demo_gazebo.launch 
roscore
tmux
catkin clean air_qr_moveit_demo 
catkin build 
roscd armsim_config/
roslaunch armsim_config demo_gazebo.launch 
cd ..
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
rosrun rqt_tf_tree rqt_tf_tree 
roscore
roslaunch armsim_config demo_gazebo.launch 
tmux
rosrun rviz rviz 
rosparam get /use_sim_time
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
rosrun rviz rviz 
roslaunch armsim_config demo_gazebo.launch 
roscore
rumux
tmux
rosrun rqt_tf_tree rqt_tf_tree 
grep -R ./* "use_sim"
grep -R "use_sim" ./*
cd ..
grep -R "use_sim" ./*
cd ..
grep -R "use_sim" ./*
grep -R "use_sim_time" ./*
rosparam get /use_sim_time
cd workspaces/airarm_gazebo_ws/src/armsim_config/
grep -R "/use_sim_time" 
grep -R "/use_sim_time" ./*
grep -R "use_sim_time" ./*
cd ..
grep -R "/use_sim_time" ./*
rosparam get /use_sim_time
cd workspaces/airarm_gazebo_ws/src/
grep -R "/use_sim_time" ./*
rosparam set /use_sim_time False
grep -R "/use_sim_time" ./*
rosparam get /use_sim_time
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roscore
rosclean purge
roscore
tmux
roslaunch armsim_config demo_gazebo.launch 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roscore
tmux
roscore
catkin clena
catkin clean
catkin clean armsim_config
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build
roslaunch airarm_moveit_config demo_gazebo.launch 
catkin build
rosparam get /use_sim_time
rosparam set /use_sim_time false
catkin build
roslaunch airarm_moveit_config demo_gazebo.launch 
rosrun rviz rviz 
clear
rosparam get /use_sim_time
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
clear
roscore
tmux
ssh airlab@192.168.66.201
sudo ufw status
sudo ufw 
sudo ufw status
sudo ufw enable
sudo ufw status verbose
sudo ufw disable
sudo ufw status
sudo ufw status verbose
ssh airlab@192.168.66.201
ipconfig
ifconfig
sudo ufw enable
sudo reboot
sudo ufw status
ssh airlab@192.168.66.201
sudo ufw stat
sudo ufw disable
sudo gedit /etc/ssh/sshd_config 
sudo service sshd restart
ssh airlab@192.168.66.201
ssh brkygkcn@192.168.66.201
sudo gedit /etc/ssh/sshd_config 
sudo service sshd restart
ssh brkygkcn@192.168.66.201
roscore
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roslaunch moveit_setup_assistant setup_assistant.launch 
rostopic list
rostopic echo /move_group/goal
rostopic echo /arm/joint_states -n1
roscore
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roscore
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roslaunch airarm_moveit_config demo_gazebo.launch 
tmux
rosrun tf tf_echo /base_link /s1
rosrun tf tf_echo /base_link /camera_link
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
ssh brkygkcn@192.168.66.201
roslaunch airarm_moveit_config demo_gazebo.launch 
roscore
tmux
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=false
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
roslaunch airarm_moveit_config demo_gazebo.launch 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
tmux
rosrun qb_hand_control tubitak_demo.py 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=false
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
rosrun qb_hand_control tubitak_demo.py 
rosservice call /communication_handler/set_commands "id: 0
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [0]" 
rosservice call /communication_handler/set_commands "id: 0
max_repeats: 1
set_commands: false
set_commands_async: false
commands: [0]" 
rosservice call /communication_handler/set_commands "id: 0
max_repeats: 1
set_commands: false
set_commands_async: false
commands: [10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: true
set_commands_async: false
commands: [10]" 
rosservice call /communication_handler/set_commands "id: 1
max_repeats: 1
set_commands: false
set_commands_async: false
commands: [10]" 
htop
rosrun rqt_gui rqt_gui 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=false
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=false
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=false
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
rosrun qb_hand_control tubitak_demo.py 
htop
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=false
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
rosrun qb_hand_control tubitak_demo.py 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
rosrun qb_hand_control tubitak_demo.py 
[A
rosrun qb_hand_control tubitak_demo.py 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
roscore
tmux
roscore
htop
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
roslaunch airarm_moveit_config demo_gazebo.launch 
roscore
rosclean purge 
tmux
htop
catkin build
rosservice list
clera
clear
rostopic list
rostopic info /qbhand1/control/qbhand1_synergy_trajectory_controller/command
rostopic type /qbhand1/control/qbhand1_synergy_trajectory_controller/command
rostopic bw --help
rostopic  --help
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=false
roslaunch airarm_moveit_config demo_gazebo.launch 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
roslaunch airarm_moveit_config demo_gazebo.launch 
source ~/.bashrc
roslaunch airarm_moveit_config demo_gazebo.launch 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
ssh brkygkcn@192.168.66.201
roscore
source ~/.bashrc
roscore
source ~/.bashrc
roscore
htop
rosrun rqt_tf_tree rqt_tf_tree 
ssh brkygkcn@192.168.66.201
roslaunch airarm_moveit_config demo_gazebo.launch 
roslaunch panda_moveit_config demo.launch
roscore
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
rosrun moveit_tutorials pick_place_tutorial.cpp 
rosrun moveit_tutorials pick_place_tutorial
rosrun moveit_tutorials pick_place_tutorial.cpp 
roscd moveit_tutorials/
cd ..
catkin build moveit_tutorials
source ~/.bashrc
rosrun moveit_tutorials pick_place_tutorial
catkin build moveit_tutorials
source ~/.bashrc
rosrun moveit_tutorials pick_place_tutorial
tmux
rosrun moveit_tutorials pick_place_tutorial
roslaunch panda_moveit_config demo.launch
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
roslaunch airarm_moveit_config demo_gazebo.launch 
ssh brkygkcn@192.168.66.201
roscore
htop
tmux
roslaunch moveit_setup_assistant setup_assistant.launch 
htop
ssh brkygkcn@192.168.66.201
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
roslaunch airarm_moveit_config demo_gazebo.launch 
roscore
tmux
catkin build
rostopic list
rostopic echo /move_group/goal
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
rostopic echo /move_group/goal
ssh brkygkcn@192.168.66.201
roslaunch airarm_moveit_config demo_gazebo.launch 
roslaunch armsim_config demo_gazebo.launch 
roslaunch /home/brky/workspaces/airarm_gazebo_ws/src/armsim_config/launch/demo_gazebo.launch
roscore
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build
roslaunch airarm_moveit_config demo_gazebo.launch 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
ssh brkygkcn@192.168.66.201
rosrun rqt_tf_tree rqt_tf_tree 
roscore
tmux
catkin build
catkin cleav
catkin clean
catkin build
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
rosrun rqt_tf_tree rqt_tf_tree 
ssh brkygkcn@192.168.66.201
roscore
roslaunch airarm_moveit_config demo_gazebo.launch 
roslaunch airarm_config demo_gazebo.launch 
roslaunch armsim_config demo_gazebo.launch 
tmux
htop
catkin clean
catkin build
roslaunch airarm_moveit_config demo_gazebo.launch 
catkin clean airarm_moveit_config
rosrun air_qr_moveit_demo air_qr_moveit_demo_node
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
cd workspaces/airarm_gazebo_ws/
catkin clean air_qr_moveit_demo
catkin build  air_qr_moveit_demo
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roscore
ssh brkygkcn@192.168.66.201
tmux
roslaunch moveit_setup_assistant setup_assistant.launch 
cd ..
catkin build
roslaunch airarm_moveit_config demo_gazebo.launch 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roscore
tmux
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roslaunch airarm_moveit_config demo_gazebo.launch 
roscore
ssh brkygkcn@192.168.66.201
catkin clean airarm_moveit_config
roslaunch moveit_setup_assistant setup_assistant.launch 
rosclean purge
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch air_right_arm_config demo_gazebo.launch 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
ssh brkygkcn@192.168.66.201
tmux
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roslaunch air_right_arm_config demo_gazebo.launch 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
clear
rostopic list
rostopic hz /move_group/filtered_cloud
rostopic echo /camera/depth/image_rect_raw
rostopic echo /camera/depth/image_rect_raw/header
rostopic echzcamera/depth/image_rect_raw/header
rostopic echz camera/depth/image_rect_raw/header
rostopic hz  camera/depth/image_rect_raw/header
clear
rostopic list
rostopic echo /move_group/filtered_cloud
rostopic hz /camera/depth/image_rect_raw/compressed
rostopic list
ssh brkygkcn@192.168.66.201
roscor
roscore
roslaunch moveit_tutorials obstacle_avoidance_demo.launch
roslaunch panda_moveit_config demo.launch
roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch
tmux
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch moveit_tutorials obstacle_avoidance_demo.launch 
roslaunch moveit_tutorials obstacle_avoidance_demo.launch
tmux
rostopic list
rostopic echo /camera/depth_registered/points/header -n1
rosrun rqt_tf_tree rqt_tf_tree 
ssh brkygkcn@192.168.66.201
rostopic echo /camera/depth_registered/points/header -n1
rostopic list
clear
rostopic list
rostopic hz /camera/depth/color/points
rostopic echo /camera/depth/color/points/header -n1
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
rostopic list
rostopic hz /camera/depth_registered/image_raw
rostopic hz /camera/depth_registered/points 
ssh brkygkcn@192.168.66.201
rosrun rqt_tf_tree rqt_tf_tree 
source ~/.bashrc
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch moveit_tutorials obstacle_avoidance_demo.launch 
rocore
roscore
source ~/.bashrc
roscore
roslaunch moveit_tutorials obstacle_avoidance_demo.launch 
source ~/.bashrc
roslaunch air_right_arm_config demo_gazebo.launch 
roslaunch moveit_tutorials obstacle_avoidance_demo.launch
tmux
ssh brkygkcn@192.168.66.201
htop
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roslaunch air_right_arm_config demo_gazebo.launch 
roscore
tmux
cd ..
catkin build
ifconfig
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch 
roscd armtubitak/
roslaunch moveit_setup_assistant setup_assistant.launch 
cd ..
catkin build
roslaunch armtubitak gazebo.launch 
roslaunch armtubitak display.launch 
rosrun tf tf_echo /world /base_link
rostopic echo /tf
rostopic info /tf
rostopic list
rosrun tf tf_echo /base_link /s1
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch armtubitak display.launch 
roslaunch armtubitak gazebo.launch 
roslaunch armtubitak gazebo.launch 
tmux
rosrun tf tf_echo /base_link /s1
rosrun tf tf_echo /base_link /s2
rosrun tf tf_echo /base_link /s3
rosrun tf tf_echo /base_link /e
rosrun tf tf_echo /base_link /roll
rosrun tf tf_echo /base_link /pitch
rosrun tf tf_echo /base_link /yaw
ping 192.168.66.201
ping 192.168.66.1
ping 192.168.66.201
ssh brkygkcn@192.168.66.201
ping 192.168.66.201
ping 192.168.66.200
roscore
htop
ping 192.168.66.1
ifconfig
ping 192.168.66.201
ping 192.168.66.222
ping 192.168.66.200
ping 192.168.66.201
roscore
rosrun rqt_gui rqt_gui 
ping 192.168.6.29
roscore
sudo poweroff
ssh brkygkcn@192.168.66.201
mkdir -p agv_ws/src
cd agv_ws/
catkin_release 
roslaunch moveit_setup_assistant setup_assistant.launch 
roscd controller_manager
catkin_release 
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin_release
ping 192.168.6.29
rostopic list
roslaunch armtubitak_config demo.launch 
rosrun controller_manager controller_manager list
rosrun controller_manager controller_manager 
rosrun controller_manager controller_manager list
roscd realtime_tools/
roslaunch armtubitak_config demo.launch 
roscore
tmux
roscd armsim_config/
roslaunch armsim_config demo.launch 
ifonfig
ifconfig
ping 192.168.6.29
python beckhoff_add_route.py 
catkin_release
roslaunch armsim_config demo.launch 
roslaunch armtubitak_config demo.launch 
roslaunch air_right_arm_config demo_gazebo.launch 
catkin build
roslaunch armtubitak_config demo.launch 
roslaunch armsim_config demo.launch 
catkin_clean 
catkin build
roslaunch armsim_config demo.launch 
htop
roslaunch armsim_config demo.launch 
source ~/.bashrc
roslaunch armsim_config demo.launch 
roslaunch air_arm_hardware_interface air_arm_position_trajectory_controllers.launch 
roslaunch armsim_config demo.launch 
rostopic list
rostopic echo /move_group/goal
source ~/.bashrc
roslaunch armsim_config demo.launch 
source ~/.bashrc
roslaunch armsim_config demo.launch 
python arm_planned_path_to_matlab.py 
roslaunch armsim_config demo.launch 
ifconfig
ping 192.168.6.29
roslaunch armsim_config demo.launch 
source ~/.bashrc
roslaunch armsim_config demo.launch 
source ~/.bashrc
roslaunch armsim_config demo.launch 
source ~/.bashrc
roslaunch armsim_config demo.launch 
roscd armsim_config/
roslaunch armsim_config demo.launch 
cd ..
grep -R "----- plan_only" ./*
grep -R " plan_only" ./*
roslaunch armsim_config demo.launch 
grep -R "sendGoalFunc" ./*
roslaunch armtubitak_config demo.launch 
ifconfig
ping 192.168.6.29
ping 192.168.66.29
ifconfig
ping 192.168.6.29
ifconfig
ping 192.168.6.29
python beckhoff_add_route.py 
grep -R "virtual_joint" ./*
cd ..
grep -R "virtual_joint" ./*
roslaunch armtubitak_config demo.launch 
catkin clean armtubitak_config
roslaunch moveit_setup_assistant setup_assistant.launch 
ping 192.168.6.29
python arm_planned_path_to_matlab.py 
roslaunch armtubitak_config demo.launch 
source ~/.bashrc
roslaunch armtubitak_config demo.launch 
dpkg -i zoom_amd64(1).deb
dpkg -i zoom_amd64.deb 
sudo dpkg -i zoom_amd64.deb 
htop
python arm_planned_path_to_matlab.py 
rostopic echo 
rostopic list
rostopic echo /joint_states 
rostopic list
rostopic echo /joint_states
rostopic info /joint_states
rosrun rqt_logger_level rqt_logger_level 
rosrun rqt_console rqt_console 
cd ..
catkin build
roslaunch armtubitak_config demo.launch 
source ~/.bashrc
roslaunch armtubitak_config demo.launch 
source ~/.bashrc
roslaunch armtubitak_config demo.launch 
roscore
roslaunch armtubitak_config demo.launch 
python arm_planned_path_to_matlab.py 
rostopic echo /rosout
python arm_planned_path_to_matlab.py 
roslaunch armtubitak_config demo.launch 
source ~/.bashrc
roslaunch armtubitak_config demo.launch 
source ~/.bashrc
roslaunch armtubitak_config demo.launch 
python arm_planned_path_to_matlab.py 
grep -R "E0017" ./*
cd air_right_arm/
grep -R "E0017" ./*
[A
grep -R "E0017" ./*
catkin build
grep -R "51021" ./*
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build
roslaunch air_right_arm_config demo.launch 
catkin clean
cd workspaces/airarm_real_ws/
catkin clean
catkin build
roslaunch air_right_arm_config demo.launch 
source ~/.bashrc
roslaunch air_right_arm_config demo.launch 
source ~/.bashrc
roslaunch air_right_arm_config demo.launch 
source ~/.bashrc
roslaunch air_right_arm_config demo.launch 
roscore
tmux
python beckhoff_add_route.py 
rostopic list
roslaunch air_right_arm_config demo.launch 
rostopic echo /joint_states 
rosrun rqt_reconfigure rqt_reconfigure 
roscore
rosrun rviz rviz 
tmux
sudo poweroff
htop
roslaunch air_right_arm_config demo.launch 
rosrun rqt_tf_tree rqt_tf_tree 
rostopic list
rostopic info /move_base/goal
rostopic echo /move_base/goal
rostopic infoo /move_base/goal
rostopic info /move_base/goal
rosrun rviz rviz 
rosrun rqt_reconfigure rqt_reconfigure 
rosrun rqt_tf_tree rqt_tf_tree 
roscore
tmux
ssh brkygkcn@192.168.66.201
roslaunch air_right_arm_config demo.launch 
rosrun rviz rviz 
roscore
tmux
catkin build
roslaunch airurdf07Dec1422 display.launch 
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch air_tubitak display.launch 
roslaunch air_right_arm display.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch air_tubitak gazebo.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch air_right_arm display.launch 
catkin build
roslaunch armsim display.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch airtubitak display.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
rosrun tf tf_echo /base_link /s1
rosrun rqt_tf_tree rqt_tf_tree 
rosrun tf tf_echo /base_link /s1
rosrun tf tf_echo /base_link /s2
roslaunch airtubitak gazebo.launch 
roslaunch airtubitak display.launch 
roslaunch moveit_setup_assistant setup_assistant.launch 
roslaunch airtubitak display.launch 
roslaunch airtubitak demo.la
catkin build 
roslaunch airtubitak_config demo.launch 
ping 192.168.6.29
roslaunch airtubitak_config demo.launch 
ifconfig
cd workspaces/
python beckhoff_add_route.py 
roslaunch airtubitak_config demo.launch 
catkin build 
roslaunch airtubitak_config demo.launch 
roscore
tmux
ssh brkygkcn@192.168.66.201
roscore
roslaunch airtubitak_config demo_robot_server.launch 
tmux
rosrun rviz rviz 
rosrun rqt_gui rqt_gui 
python qb_hand_tubitak_demo.py 
roslaunch airtubitak_config demo_robot_server.launch 
tmux
rsocore
roscor
roscore
rosrun rqt_reconfigure rqt_reconfigure 
roslaunch airtubitak_config demo_robot_server.launch 
tmux
ssh brkygkcn@192.168.66.201
htop
htop
roslaunch airtubitak_config demo_robot_server.launch 
rosrun rqt_tf_tree rqt_tf_tree 
rosrun rqt_reconfigure rqt_reconfigure 
roscore
tmux
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build 
roslaunch airtubitak_config demo_robot_server.launch 
rosrun rqt_tf_tree rqt_tf_tree 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roscore
ssh brkygkcn@192.168.66.201
tmux
catkin clean 
catkin_release 
roslaunch airtubitak_config demo_robot_server.launch 
roscore
tmux
ssh brkygkcn@192.168.66.201
catkin clean 
catkin build
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roslaunch moveit_setup_assistant setup_assistant.launch 
ssh brkygkcn@192.168.66.201
rostopic echo /move_group/display_planned_path 
roslaunch airtubitak_config demo_robot_server.launch 
roscore
tmux
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roslaunch moveit_setup_assistant setup_assistant.launch 
ssh brkygkcn@192.168.66.201
roscore
ssh brkygkcn@192.168.66.201
roslaunch airtubitak_config demo_robot_server.launch 
tmux
ssh brkygkcn@192.168.66.201
grep -R "read_by_name" ./*
ssh brkygkcn@192.168.66.201
rosrun rqt_gui rqt_gui 
roscore
tmux
ssh brkygkcn@192.168.66.201
ls
cd workspaces/
ls
rostopic list
rostopic echo /wheel_odom 
rosrun rqt_gui rqt_gui 
rosrun rqt_tf_tree rqt_tf_tree 
rostopic list
rostopic echo /wheel_encoders_rl 
ssh brkygkcn@192.168.66.201
ssh brkygkcn@192.168.66.201
roscore
rostopic info /odom 
rostopic echo /odom -n1
rostopic list
clear
rostopic list
rostopic info /pose2D 
rostopic info /pose_with_covariance_stamped 
ssh brkygkcn@192.168.66.201
rostopic list
rostopic info /odom
rostopic echo /wheel_odom -n1
rostopic echo /lsm_odom -n1
rosrun rqt_tf_tree rqt_tf_tree 
roscore
rosrun rviz rviz 
rosrun rqt_gui rqt_gui 
tmux
rosrun rqt_tf_tree rqt_tf_tree 
rosrun tf tf_monitor 
roscore
rosrun rviz rviz 
rosrun rqt_tf_tree 
rosrun rqt_tf_tree rqt_tf_tree 
rosrun rqt_gui rqt_gui 
ssh brkygkcn@192.168.66.201
rosrun rqt_tf_tree rqt_tf_tree 
rosrun rviz rviz 
roscore
tmux
ssh brkygkcn@192.168.66.201
rosrun rviz rviz 
roscore
roslaunch airtubitak display.launch 
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
source ~/.bashrc
roslaunch qb_hand_control control.launch standalone:=true activate_on_initialization:=true device_id:=1 use_simulator_mode:=false use_controller_gui:=true
roslaunch airtubitak display.launch 
ping 192.168.66.103
ifconfig
ping 192.168.0.109
ping 192.168.66.1
ping 192.168.66.103
ifconfig
ping 192.168.66.103
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build
grep -R "SLDASM" ./*
catkin build
roslaunch moveit_setup_assistant setup_assistant.launch 
source ~/.bashrc
roslaunch moveit_setup_assistant setup_assistant.launch 
catkin build
roscore
ssh brkygkcn@192.168.66.201
roslaunch arm_config demo_robot_server.launch 
rocore
roscore
tmux
roscore
rosclean purge 
clear
tmux
ifconfig
ping 192.168.6.29
python arm_planned_path_to_matlab.py 
roslaunch arm_config demo_full.launch 
ping 192.168.6.29
roslaunch arm_config demo_full.launch 
grep -R "lower=\"-1.3\""
grep -R "lower=\"-1.3\"" ./*
grep -R "lower=\"-1.4\"" ./*
catkin build
python arm_planned_path_to_matlab.py 
roslaunch arm_config demo_full.launch 
catkin build
python arm_planned_path_to_matlab.py 
roslaunch arm_config demo_full.launch 
catkin build
python arm_planned_path_to_matlab.py 
roslaunch arm_config demo_full.launch 
catkin build
roslaunch arm_config demo_full.launch 
rosrun rqt_tf_tree rqt_tf_tree 
catkin build
ssh brkygkcn@192.168.66.201
roscorae
roscore
ping 192.168.6.29
roslaunch arm_config demo_full.launch 
roslaunch arm_config demo_robot_server.launch 
roscore
tmux
rosrun tf tf_echo /base_link /fiducial_100
rosrun tf tf_echo /base_link /base_link
cd ..
catkin build
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roslaunch arm_config demo_robot_server.launch 
roscore
ssh brkygkcn@192.168.66.201
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
rosrun tf tf_echo /fiducial_100 /fiducial_175
rosrun tf tf_echo /yaw /fiducial_175
rosrun tf tf_echo /fiducial_100 /fiducial_175
rosrun tf tf_echo /base_link /fiducial_100
rosrun tf tf_echo /fiducial_100 /fiducial_175
rosrun tf tf_echo /yaw /fiducial_100
rosrun tf tf_echo /base_link /yaw
rosrun tf tf_echo /base_link /fiducial_100
rosrun tf tf_echo /fiducial_100 /fiducial_175
rosrun tf tf_echo /base_link /fiducial_175
roslaunch arm_config demo_robot_server.launch 
roscore
tmux
roscd moveit_ros_planning_interface/
roscd moveit_ros
roscd moveit
rosls moveit_ros
rosls moveit_ros_planning_interface/
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roscore
ssh brkygkcn@192.168.66.201
rosrun rqt_gui rqt_gui 
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
rosrun tf tf_echo /fiducial_100 /fiducial_175
rosrun tf tf_echo /fibase_linkfiducial_175
rosrun tf tf_echo /fibase_link fiducial_175
rosrun tf tf_echo /base_link /fiducial_175
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
roscore
tmux
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
rosrun tf tf_echo /base_link /fiducial_175
rosrun tf tf_echo /fiducial_100 /fiducial_175
rosrun tf tf_echo /base_link /fiducial_100
rosrun tf tf_echo /base_link /fiducial_175
rosrun tf tf_echo /base_link /fiyaw
rosrun tf tf_echo /base_link /yaw
rosrun tf tf_echo /base_link /fiducial_175
rosrun tf tf_echo /bayaw /fiducial_175
rosrun tf tf_echo /yaw /fiducial_175
roslaunch arm_config demo_robot_server.launch 
ssh brkygkcn@192.168.66.201
roscore
rosclean purge 
roscore
ssh brkygkcn@192.168.66.201
roslaunch arm_config demo_robot_server.launch 
roscore
source ~/.bashrc
roscore
roslaunch arm_config demo_robot_server.launch 
source ~/.bashrc
roslaunch arm_config demo_robot_server.launch 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
rosrun tf tf_echo /s1 /fiducial_100
tmux
ssh brkygkcn@192.168.66.201
rosrun tf tf_echo /base_link /yaw
ssh brkygkcn@192.168.66.201
roscore
roslaunch arm_config demo_robot_server.launch 
clear
roslaunch arm_config demo_robot_server.launch 
clear
roslaunch arm_config demo_robot_server.launch 
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
tmux
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
clear
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
sour
roslaunch arm_config demo_robot_server.launch 
rostopic echo /joint_states -n1
ssh brkygkcn@192.168.66.201
roscore
rosclean purge 
clear
roscore
tmux
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
tmux
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
source ~/.bashrc
rosrun air_qr_moveit_demo air_qr_moveit_demo_node 
ssh brkygkcn@192.168.66.201
roslaunch arm_config demo_robot_server.launch 
roscore
ssh brkygkcn@192.168.66.201
catkin build
rostopic list
clear
rostopic list
rosrun rviz rviz 
roscore
rostopic echo /camera/align_to_color/parameter_descriptions -n1
rostopic echo /camera/align_to_color/parameter_updates -n1
clear
rostopic info /camera/aligned_depth_to_color/camera_info -n1
rostopic echo /camera/aligned_depth_to_color/camera_info -n1
rostopic info /camera/aligned_depth_to_color/image_raw/compressed
rostopic info /camera/aligned_depth_to_color/image_raw
rostopic info /camera/depth/color/points 
rostopic info /camera/depth/image_rect_raw
rostopic echo /camera/depth/image_rect_raw
rostopic echo /camera/depth/image_rect_raw/header -n1
rostopic echo /camera/depth/image_rect_raw/height -n1
rostopic echo /camera/depth/image_rect_raw/width -n1
rostopic echo /camera/depth/image_rect_raw/step -n1
rostopic echo /camera/depth/camera_info -n1
rostopic info /camera/depth/color/points -n1
rostopic info /camera/depth/color/points 
rostopic echo /camera/depth/color/points/height -n1 
rostopic echo /camera/depth/color/points/width -n1 
rostopic echo /camera/depth/color/points/fields -n1 
rostopic echo /camera/depth/color/points/is_dense -n1 
rostopic echo /camera/depth/color/points/point_step -n1 
rostopic echo /camera/depth/color/points/row_step -n1 
rostopic info /camera/depth/color/points
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
tmux
catkin build air_object_segmentation 
catkin build
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
rossrv show air_object_segmentation/PcTransform 
catkin build
catkin clean air_object_segmentation 
catkin build air_object_segmentation 
catkin clean air_object_segmentation 
catkin build air_object_segmentation 
catkin clean air_object_segmentation 
catkin build air_object_segmentation 
rosservice list
rosservice info /transform_pc_srv 
catkin clean air_object_segmentation 
cd workspaces/airarm_real_ws/
catkin clean air_object_segmentation 
catkin build air_object_segmentation 
rostopic hz /pcl_passed 
rostopic info /pcl_passed 
rostopic hz /pcl_passed 
rostopic hz /pcl_objects 
rostopic echo /pcl_objects 
rostopic echo /pcl_outliers_removed 
rostopic hz /pcl_outliers_removed 
roslaunch air_object_segmentation air_object_segmentation.launch 
ssh brkygkcn@192.168.66.201
rosrun rqt_tf_tree rqt_tf_tree 
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
rosrun rviz rviz 
roscore
ssh brkygkcn@192.168.66.201
rosrun rviz rviz 
roscore
rostopic hz /airarm/goal 
rosrun rqt_tf_tree rqt_tf_tree 
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
source ~/.bashrc
rosrun air_qr_moveit_demo air_pc_demo 
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
source ~/.bashrc
roslaunch air_object_segmentation air_object_segmentation.launch 
roslaunch arm_config demo_robot_server.launch 
tmux
ssh brkygkcn@192.168.66.201
ssh airlab@192.168.66.2
