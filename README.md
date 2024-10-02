# raspberry-ros-galactic-realsense-d435i
#use Realsense D435i on Raspberry PI4 
| divce | ubuntu   |  ros  | camera  |
| :------:| :-----:  | :----:  | :----:  |
| raspberry pi4   | 20.04   |   galactic     | realsense d435i |
## If you want to get the most out of realsense in raspberry, use ubuntu20.04!           This is a tutorial designed for people starting from scratch

## Preparation before begin
###  Download ubuntu20.04 server
### Change the wifi name and password
```
sudo nano /etc/netplan/50-cloud-init.yaml
```
Save as ^x -> y ->enter
Save and run in sequence
```
sudo netplan --debug try
sudo netplan --debug apply
```
### Modifying mirror source
```
wget http://fishros.com/install -O fishros && . fishros
```
Select *5 -> 2 ->2* in turn
```
sudo apt update
sudo apt upgrade
```
### Install desktop
```
sudo apt install ubuntu-desktop
```
Restart after installation

### Install others
```
sudo apt install vim
sudo apt install git
sudo apt install openssh-server
sudo systemctl status ssh
```
### Creat super user
```
sudo passwd root
```
## Install ros2-galactic
```
wget http://fishros.com/install -O fishros && . fishros
```
## Install Realsense SDK
```
sudo apt install ros-galactic-librealsense2*
```
```
sudo apt install ros-galactic-realsense2-*
```
Then you can  start the realsense node
```
ros2 launch realsense2_camera rs_launch.py
# ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
```
## Install yolov5
This way has conflict with foxy
```
sudo apt update
sudo apt install python3-pip ros-galactic-vision-msgs
pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple yolov5  
```
The last command will last about an hour
```
mkdir -p yolov5_ws/src
cd yolov5_ws/src
git clone https://github.com/fishros/yolov5_ros2.git
cd yolov5_ws
colcon build
source install/setup.bash
```
If you have some errors during the build process,you may need to reduce setuptools' version
```
sudo apt install setupyools==58.0.0
```
Then *colcon build* in yolo's workspace
## Check yolov5 and reduce huggingface-hub
You must check yolov5,find it and open its python files,because those may be blank.And if this has already happened,you can delect it and install again.
Then,you must reduce the huggingface-hub,such as 0.24.7.
## RUN IT
In first terminal ,run
```
ros2 launch realsense2_camera rs_launch.py
```
In second terminal,run
```
cd yolov5_ws
source install/setup.bash
ros2 run yolov5_ros2 yolo_detect_2d --ros-args -p device:=cpu -p image_topic:=/camera/color/image_raw -p show_result:=True -p pub_result_img:=True
```
In third terminal,run 
```
ros2 topic echo /yolo_result
```
This will show you the test results

## How can the Raspberry PI results be applied to other boards?
### Communication using different levels
Since I forgot the interface to prevent serial communication on the esp32-wroom-32e and stm32g431cbu6 boards, and my motor control interface happened to have two Hall encoder pulse return values, I used high and low levels to communicate between the boards
#### On raspberry
You must install package which will be used
```
sudo apt update
sudo apt install python3-rpi.gpio
```
Create workspace
```
mkdir -p ~/ros2_gpio_ws/src
cd ~/ros2_gpio_ws/src
```
Create pacakge in src
```
cd src
ros2 pkg create --build-type ament_python gpio_control
```
Write source code
```
cd gpio_control
mkdir gpio_control
touch gpio_control/gpio_node.py
vim gpio_node.py
```
gpio_node.py:
```python
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
```
# In ws/src/gpio_control/setup.py
vim setup.py
```
Add node information as
```
entry_points={ # exist
    'console_scripts': [ # exist 
        'gpio_control = gpio_control.gpio_node:main', # you add
    ], # exist
}, # exist
```
Build it
```
# In your ws
colcon build
```
In this step,you may need to tier down the setuptools just like sudo apt install setupyools==58.0.0

Then,you can run it
In your ws
```
source install/setup.bash
ros2 run gpio_control gpio_control
```
If you have some trouble like "RuntimeError: Not running on a RPi!"
just run su!,source ws and run again.
#### On Esp32
just choose your pin
```python
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

## How can I use yolo result to control what signals are output
### First,you must create a listener package and node
Create a package
```
mkdir -p listen/src
cd listen/src
ros2 pkg create --build-type ament_python listen
cd listen/listen
touch listen_node.py
vim listen_node.py
```
In your node pyfiles
```python
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String

class YoloResultListener(Node):
    def __init__(self):
        super().__init__('yolo_result_listener')


        self.subscription = self.create_subscription(
            Detection2DArray,
            'yolo_result',
            self.listener_callback,
            10
        )
        self.subscription  


        self.publisher_ = self.create_publisher(String, 'yolo_result_feedback', 10)
        

        self.timer = self.create_timer(1.0, self.timer_callback)


        self.last_message = None
        self.topic_detected = False

        self.get_logger().info('Yolo Result Listener Node has been started')

    def listener_callback(self, msg):

        if len(msg.detections) > 0:
            detection_info = []
            for detection in msg.detections:
                for result in detection.results:
                    class_id = result.hypothesis.class_id
                    score = result.hypothesis.score
                    bbox_center_x = detection.bbox.center.x
                    bbox_center_y = detection.bbox.center.y
                    bbox_size_x = detection.bbox.size_x
                    bbox_size_y = detection.bbox.size_y

                    detection_info.append(f"Class: {class_id}, Score: {score:.2f}, BBox center: ({bbox_center_x}, {bbox_center_y}), Size: ({bbox_size_x}, {bbox_size_y})")
            

            self.last_message = "; ".join(detection_info)
        else:
            self.last_message = 'no message listen'


        self.topic_detected = True
        self.get_logger().info(f'Heard yolo_result message: "{self.last_message}"')

    def timer_callback(self):
        msg = String()

        if not self.topic_detected:

            msg.data = 'no detect topic'
        elif self.last_message is None:

            msg.data = 'no message listen'
        else:

            msg.data = f'Yolo result: {self.last_message}'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.last_message = None


def main(args=None):
    rclpy.init(args=args)


    node = YoloResultListener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
Change setup.py as
```python
    entry_points={
        'console_scripts': [
            'listen = listen.listen_node:main',
        ],

```
Then,you can run it.
```
cd
cd listen
source install/setup.bash
ros2 run listen listen
```
### End
