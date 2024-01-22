rosbag record --tcpnodelay \
/robot/dlio/odom_node/odom \
/robot/dlio/odom_node/pointcloud/deskewed \
/odometry \
/particle_map/future_map \
/particle_map/particle_map \
/particle_map/inflate_map \
/odom_visualization/path \
/drone_node/init_list \
/drone_node/optimal_list \
/drone_node/control_point \
/odom_visualization/robot \
/drone_node/goal_point \
-o ~/real_world.bag
