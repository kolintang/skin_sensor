#  Skin Sensors (Linux)


## File explaination
* **/sensor_devel:**<br />
socketcan_dev_v1.0: slight modification to original. <br />
socketcan_dev_v2.0: output one patch to csv. <br />
socketcan_dev_v3.0: rostopic publication added. <br />
socketcan_dev_v4.0: write to csv moved to subscriber node <br />
socketcan_dev_v5.0: reading multiple patches <br />
joint_tactile_subscriber: ros node for subscribing and output csv file<br />

* **/sensor_rosnode:**<br />
joint_tactile_subscriber_v1: ros node for subscribing and output csv file.<br />
joint_tactile_subscriber_v2: added TCP/IP reading.<br />
joint_tactile_subscriber_v3: rewrite csv row head initilisation.<br />
server.py: mock windows server sending sensor readings over TCP/IP.<br />

* **/sensor_windows:**<br />
main4: tito's original code.<br />
main5: modification to communicate with linux over TCP/IP.<br />
client.py: mock linux client acts as rosnode subscriber over TCP/IP.<br />

* **/socketcan_test:**<br />
Socketcan communication scripts for testing purpose.<br />

* **/utilities:**<br />
Command sensors using bash or pytho.<br />
virtual sensor for testing purpose.<br />


## Prerequisites

Install NTCANDrivers by following instruction in
```
NTCANDrivers/Linux/SocketCAN/ntcanSckPlugin64-2.1.0-ntcan-3.5.1
```

Install module assistant 'socketcan_1-1-92-0_20150508.zip'
```
unzip /additional_driver/socketcan_1-1-92-0_20150508.zip
make
sudo make install
```

Install can-utils
```
sudo apt-get install can-utils -y
```

Install ROS socketcan_interface
```
sudo apt-get install socketcan_interface
```

To use ROS package in Python3
```
sudo pip3 install rospkg, catkin_pkg
```


## For real sensor communication (can0)

Load esd_usb2 kernel driver
```
sudo modprobe esd_usb2
```

Initialize NIC, and setup can0
```
sudo ip link set can0 type can bitrate 100000
sudo ip link set can0 up
```

Use socketcan_dump, or you can use can-util function
```
rosrun socketcan_interface socketcan_dump can0
candump can0
```

To check netlink status
```
ip -details -statistics link show can0
```

Send MTB trigger command (20x#07.00 depending on which MTB used)<br />
Send MTB stop command (70x#17.00 depending on which MTB used)
```
bash | cansend can0 20x#07.00
bash | cansend can0 70x#17.00
```

To use sensor_dev_vx.0.py to pre-process sensor data<br />
(For specific version to use, please refer to file description)
```
python3 sensor_dev_vx.0.py listen can0
```

To use mock sensor
```
./virtual_sensor.sh
```


## For virtual sensor commnucation (vcan0)

Load driver
```
sudo modprobe vcan # kernel driver
```

Initialize NIC
```
sudo ip link add dev vcan0 up type vcan
```

Socketcan_dump
```
rosrun socketcan_interface socketcan_dump vcan0
```

Netlink status
```
ip -details -statistics link show vcan0
```

To use socketcan program to record sensor date to CSV
```
python3 socketcan_dev_v3.0.py listen vcan0
```

To use mock sensor
```
./virtual_sensor.sh
```
