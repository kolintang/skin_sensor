
-----------------------------------------------------------
File explained
-----------------------------------------------------------

1. Sensor Development:
The tactile sensor reading pre-processing program using socketcan.
socketcan_dev_Example: being the original script.
socketcan_dev_v1.0: slight modification.
socketcan_dev_v2.0: output one patch to csv.
socketcan_dev_v3.0: rostopic publication added.
socketcan_dev_v4.0: write to csv moved to subscriber node.
socketcan_dev_v5.0: reading multiple patches.

2. Sensor ROSNode:
joint_tactile_subscriber_v1: ros node for subscribing and output csv file.
joint_tactile_subscriber_v2: added TCP/IP reading.
joint_tactile_subscriber_v3: rewrite csv row head initilisation.
server.py: mock windows server sending sensor readings over TCP/IP.

3. Sensor Windows:
main4: tito's original code.
main5: modification to communicate with linux over TCP/IP.
client.py: mock linux client acts as rosnode subscriber over TCP/IP.

3. Socketcan Test:
Other socketcan communication scripts for testing purpose.

4. Utilities:
Command sensors using bash or pytho.
virtual sensor for testing purpose.

-----------------------------------------------------------
Requirements
-----------------------------------------------------------

# Install NTCANDrivers
follow instruction in:
/NTCANDrivers/Linux/SocketCAN/ntcanSckPlugin64-2.1.0-ntcan-3.5.1

# Install module assistant 'socketcan_1-1-92-0_20150508.zip'
unzip /additional_driver/socketcan_1-1-92-0_20150508.zip
make
sudo make install

# Install can-utils
sudo apt-get install can-utils -y

# Install ROS Socketcan_interface
sudo apt-get install socketcan_interface

# To use ROS package in Python3
sudo pip3 install rospkg catkin_pkg


-----------------------------------------------------------
For physical sensor communication using can0
-----------------------------------------------------------

# Load driver
sudo modprobe esd_usb2 # kernel driver

# Initialize NIC
sudo ip link set can0 type can bitrate 100000

# Initialize can0 channel
sudo ip link set can0 up

# Socketcan_dump
rosrun socketcan_interface socketcan_dump can0

# or you can use can-utils function
candump can0

# Netlink status
ip -details -statistics link show can0

# Send MTB trigger command (20x#07.00 depending on which MTB used)
bash | cansend can0 20x#07.00

# Send MTB stop command (70x#17.00 depending on which MTB used)
bash | cansend can0 70x#17.00

#To use sensor_dev_vx.0.py to pre-process sensor data
(For specific version to use, please refer to file description)
python3 sensor_dev_vx.0.py listen can0


-----------------------------------------------------------
For virtual sensor communication using vcan0
-----------------------------------------------------------

# Load kernel driver
sudo modprobe vcan

# Initialize NIC
sudo ip link add dev vcan0 up type vcan

# Socketcan_dump
rosrun socketcan_interface socketcan_dump vcan0

# or you can use can-utils function
candump vcan0

# Netlink status
ip -details -statistics link show vcan0

#To use sensor_dev_vx.0.py to pre-process sensor data
(For specific version to use, please refer to file description)
python3 sensor_dev_vx.0.py listen can0

# To use mock sensor
./virtual_sensor.sh
