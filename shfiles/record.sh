rosbag record --tcpnodelay \
/Odometry \
/cloud_registered \
/particle_map/future_map \
/particle_map/particle_map \
/drone_0_odom_visualization/path \
/drone_0_node/topo_list \
/drone_0_node/optimal_list \
/drone_0_odom_visualization/robot \
/dynamic_map_generator/global_cylinders_state \
/drone_0_odometry \
/drone_0_node/goal_point \
-o ~/real_world.bag
