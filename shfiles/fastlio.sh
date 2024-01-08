source devel/setup.bash;
roslaunch livox_ros_driver2 msg_MID360.launch & sleep 5;
roslaunch fast_lio mapping_mid360.launch & sleep 5;
wait;