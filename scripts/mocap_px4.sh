roslaunch realsense2_camera rs_d435i.launch &
sleep 3
roslaunch mavros px4.launch &
sleep 3
roslaunch mocap_ros mocap.launch