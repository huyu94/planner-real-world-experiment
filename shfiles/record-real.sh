rosbag record --tcpnodelay \
/robot/dlio/odom_node/odom \
/robot/dlio/odom_node/pointcloud/deskewed \
/particle_map/future_map \
/particle_map/particle_map \
/particle_map/inflate_map \
/odom_visualization/path \
/drone_node/topo_list \
/drone_node/optimal_list \
/odom_visualization/robot \
/odometry \
/drone_node/goal_point \
/drone_node/control_point \
-o ~/real_world.bag
