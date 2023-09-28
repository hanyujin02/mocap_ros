# Optitrack Motion Capture for CERLAB Drone Cage
### Installation
This step only applies for robot's onboard computer.
```
# ALL the operations are on the robot's onboard computer!
# Install the require package
sudo apt install ros-noetic-vrpn-client-ros

# Install this package
cd ~catkin_ws/src
git clone https://github.com/Zhefan-Xu/mocap_ros.git
cd ~catkin_ws
catkin_make

# move the mocap_px4.sh to the home directory of the robot's onboard computer (optional, for convenience)
mv ~/catkin_ws/src/mocap_ros/scripts/mocap_px4.sh ~/
```

### Run
a. Turn on the OptiTrack desktop computer and open the software ```motive``` on the desktop. DO NOT CHANGE ANYTHING INSIDE.

b. Connect the desktop computer to ```CERLAB_WIFI```. Password: ```00000000```

c. To obtain localization and fly the drone:
```
sh mocap_px4.sh # this will be the same as vins_px4.sh
roslaunch autonomous_flight navgation.launch # use navigation for exmaple
```
