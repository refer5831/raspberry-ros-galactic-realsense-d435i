# raspberry-ros-galactic-realsense-d435i
#use Realsense D435i on Raspberry PI4 
| divce | ubuntu   |  ros  | camera  |
| :------:| :-----:  | :----:  | :----:  |
| raspberry pi4   | 20.04   |   galactic     | realsense d435i |
## If you want to get the most out of realsense in raspberry, use ubuntu20.04!           This is a tutorial designed for people starting from scratch

## Preparation before begin
###  Download ubuntu20.04 server
### Change the wifi name and password
```javascript
sudo nano /etc/netplan/50-cloud-init.yaml
```
Save as ^x -> y ->enter
Save and run in sequence
```javascript
sudo netplan --debug try
sudo netplan --debug apply
```
### Modifying mirror source
```javascript
wget http://fishros.com/install -O fishros && . fishros
```
Select *5 -> 2 ->2* in turn
```javascript
sudo apt update
sudo apt upgrade
```
### Install desktop
```javascript
sudo apt install ubuntu-desktop
```
Restart after installation

### Install others
```javascript
sudo apt install vim
sudo apt install git
sudo apt install openssh-server
sudo systemctl status ssh
```
### Creat super user
```javascript
sudo passwd root
```
## Install ros2-galactic
```javascript
wget http://fishros.com/install -O fishros && . fishros
```
## Install Realsense SDK
```javascript
sudo apt install ros-galactic-librealsense2*
```
```javascript
sudo apt install ros-galactic-realsense2-*
```
Then you can  start the realsense node
```javascript
ros2 launch realsense2_camera rs_launch.py
# ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
```
## Install yolov5
This way has conflict with foxy
```javascript
sudo apt update
sudo apt install python3-pip ros-galactic-vision-msgs
pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple yolov5  
```
The last command will last about an hour
```javascript
mkdir -p yolov5_ws/src
cd yolov5_ws/src
git clone https://github.com/fishros/yolov5_ros2.git
cd yolov5_ws
colcon build
source install/setup.bash
```
If you have some errors during the build process,you may need to reduce setuptools' version
```javascript
sudo apt install setupyools==50.0.0
```
Then *colcon build* in yolo's workspace
## Check yolov5 and reduce huggingface-hub
You must check yolov5,find it and open its python files,because those may be blank.And if this has already happened,you can delect it and install again.
Then,you must reduce the huggingface-hub,such as 0.24.7.
## RUN IT
In first terminal ,run
```javascript
ros2 launch realsense2_camera rs_launch.py
```
In second terminal,run
```javascript
cd yolov5_ws
source install/setup.bash
ros2 run yolov5_ros2 yolo_detect_2d --ros-args -p device:=cpu -p image_topic:=/camera/color/image_raw -p show_result:=True -p pub_result_img:=True
```
In third terminal,run 
```javascript
ros2 topic echo /yolo_result
```
This will show you the test results
### End
