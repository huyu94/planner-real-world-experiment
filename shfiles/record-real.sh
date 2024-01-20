rosbag record --tcpnodelay \
/robot/dlio/odom_node/odom \
/robot/dlio/odom_node/pointcloud/deskewed \
/particle_map/future_map \
/particle_map/particle_map \
/particle_map/inflate_map \
/drone_0_odom_visualization/path \
/drone_0_node/topo_list \
/drone_0_node/optimal_list \
/drone_0_odom_visualization/robot \
/drone_0_odometry \
/drone_0_node/goal_point \
-o ~/real_world.bag
