# Aceinna OpenRTK ROS Driver

Overview
--------
This is the ROS driver for Aceinna OpenRTK series GNSS/INS integrated navigation module products, with support for **Serial (UART)** port and **Ethernet** port. 

The ROS driver source files are located in the subfolder "openrtk_ros".

Several primary messages output by OpenRTK series products are defined in the subfolder "openrtk_msg", which are composed of ROS "std_msg" header and proprietary OpenRTK message content:

- openrtk_gnss.msg: GNSS solution message
- openrtk_imu.msg: raw IMU data message
- openrtk_ins.msg: INS solution message

  Note: User could modify the content of the messages that are defined in the openrtk_msg/msg folder, and the modified messages should comply with ROS definitions.



Usage
--------

### Prerequisites

- PC with Ubuntu 18.04
- Python 3.x
- Install ROS Melodic, please refer to http://wiki.ros.org/melodic/Installation/Ubuntu
- OpenRTK330LI EVK (FW v23.05), please refer to https://openrtk.readthedocs.io/en/latest/firmware_upgrade.html for EVK upgrade steps



### Build

The following are steps to build the ROS driver from source code in your local development environment:

1. Go to your ROS workspace

   ​	`cd ~/catkin_ws`   

2. Copy folders to your ROS workspace 

   ​	`cp openrtk_ros openrtk_msg ./src`

3. Compile the code

   ​	`catkin_make`

**Note**:   This ROS driver supports for serial port and Ethernet port. The messages contents output by the two ports are the same. To switch between the two type of ports, go to *line 122* of /ros_rtk/src/driver/driver.cpp  and follow the operation in the comments.

When you choose the Ethernet port interface, follow the few steps below to configure the device IP:

1. Get the local network IP of your Ubuntu system (e.g. 192.168.xxx.xxx)	

2. Modify *LINE 9* of  the python script "netbios.py" inside the folder "openrtk_ros" to let OpenRTK330LI device get your Ubuntu system IP

3. Run the "netbios.py" script to complete the config, and if the console prints ".........   true ", it shows OpenRTK330LI has obtained your system IP address successfully, then the ROS driver takes effective. 

   


### Operation

1. Launch the node

   ​	`roslaunch openrtk_ros run.launch`	

2. List topics

   ​	`rostopic list`

3. Message echo (xxx = imu, ins, gnss)

   ​	`rostopic echo /openrtk/topic_rtk_xxx`

   ​	                

## License

The source code is licensed under the MIT license --- refer to the LICENSE file for details.

