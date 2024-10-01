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
sudo apt install setupyools==58.0.0
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

## How can the Raspberry PI results be applied to other boards?
### Communication using different levels
Since I forgot the interface to prevent serial communication on the esp32-wroom-32e and stm32g431cbu6 boards, and my motor control interface happened to have two Hall encoder pulse return values, I used high and low levels to communicate between the boards
#### On raspberry
You must install package which will be used
```javascript
sudo apt update
sudo apt install python3-rpi.gpio
```
Create workspace
```javascript
mkdir -p ~/ros2_gpio_ws/src
cd ~/ros2_gpio_ws/src
```
Create pacakge in src
```javascript
cd src
ros2 pkg create --build-type ament_python gpio_control
```
Write source code
```javascript
cd gpio_control
mkdir gpio_control
touch gpio_control/gpio_node.py
vim gpio_node.py
```
gpio_node.py:
```javascript
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from time import sleep

class GPIOControlNode(Node):
    def __init__(self):
        super().__init__('gpio_control_node')
        
        # 设置GPIO模式
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT)  # GPIO 18 is output

        self.timer = self.create_timer(1.0, self.toggle_gpio)  # change time
        self.state = False

    def toggle_gpio(self):
        self.state = not self.state
        GPIO.output(18, self.state)
        self.get_logger().info(f'GPIO 18 is {"HIGH" if self.state else "LOW"}')

    def destroy_node(self):
        GPIO.cleanup()  # 清理GPIO状态
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPIOControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Update setup.py
```javascript
# In ws/src/gpio_control/setup.py
vim setup.py
```
Add node information as
```javascript
entry_points={ # exist
    'console_scripts': [ # exist 
        'gpio_control = gpio_control.gpio_node:main', # you add
    ], # exist
}, # exist
```
Build it
```javascript
# In your ws
colcon build
```
In this step,you may need to tier down the setuptools just like sudo apt install setupyools==58.0.0

Then,you can run it
In your ws
```javascript
source install/setup.bash
ros2 run gpio_control gpio_control
```
If you have some trouble like "RuntimeError: Not running on a RPi!"
just run su!,source ws and run again.
#### On Esp32
just choose your pin
```javascript
from machine import Pin
import time

input_pin = Pin(18, Pin.IN)

def read_gpio():
    while True:
        value = input_pin.value()
        print("GPIO Input Value:", value)
        time.sleep(1)

if __name__ == "__main__":
    read_gpio()

```
Connect esp32 and raspberry,you can certificate this work successful or not.
### End
