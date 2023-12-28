#include <path_searching/topo_prm.h>
#include <plan_env/dsp_map.h>
#include <nav_msgs/Odometry.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/polynomial_traj.h>
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>

nav_msgs::Odometry init_odom_;
unique_ptr<TopoPRM> topo_prm;
PlanningVisualization::Ptr visualization_;
BsplineOptimizer optimizer_;


void waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    if(msg->pose.position.z < -0.1)
    {
        ROS_ERROR("Invalid waypoint");
        return;
    }
    std::cout << "Triggered" << std::endl;

    Eigen::Vector3d end_pt(msg->pose.position.x , msg->pose.position.y , 1.0);
    Eigen::Vector3d start_pt(init_odom_.pose.pose.position.x , init_odom_.pose.pose.position.y , init_odom_.pose.pose.position.z);

    ROS_INFO("[Topo]: ----------");
    list<GraphNode::Ptr> graph;
    vector<vector<Eigen::Vector3d>> raw_paths,filtered_paths,select_paths;
    vector<Eigen::Vector3d> start_pts,end_pts;
    topo_prm->findTopoPaths(start_pt, end_pt, start_pts, end_pts, graph, raw_paths, filtered_paths, select_paths);

    if(select_paths.size() == 0)
    {
        ROS_ERROR("No path found");
        return;
    }
    else{
        visualization_->displayAStarList(select_paths,0);
        visualization_->displayGoalPoint(end_pt,Eigen::Vector4d(0, 0.5, 0.5, 1),0.3,0);
        visualization_->displayGoalPoint(start_pt,Eigen::Vector4d(0.5,0.5,0,1),0.3,1);
        int id = 2;
        for(auto t = graph.begin();t != graph.end(); t++)
        {
            visualization_->displayGoalPoint((*t)->pos_,Eigen::Vector4d(0.5,0.5,0.5,1),0.1,id++);
        }
        ROS_INFO("Found %d paths", select_paths.size());
    }

    /* minimum snap */
    Eigen::Vector3d start_vel(0,0,0);
    Eigen::Vector3d end_vel(0,0,0);
    Eigen::Vector3d start_acc(0,0,0);
    Eigen::Vector3d end_acc(0,0,0);
        
    for(int i=0;i<select_paths.size();i++)
    {
        PolynomialTraj traj_;
        std::cout << 0 << std::endl;
        Eigen::MatrixXd pos = Eigen::MatrixXd::Zero(3,select_paths[i].size());
        Eigen::VectorXd time = Eigen::VectorXd::Ones(select_paths[i].size()-1) * 2;
        if(select_paths[i].size() == 2)
        {
            pos(0,0) = select_paths[i][0][0];
            pos(1,0) = select_paths[i][0][1];
            pos(2,0) = select_paths[i][0][2];
            pos(0,1) = select_paths[i][1][0];
            pos(1,1) = select_paths[i][1][1];
            pos(2,1) = select_paths[i][1][2];
            traj_ = PolynomialTraj::one_segment_traj_gen(select_paths[i][0],start_vel,start_acc,select_paths[i][1],end_vel,end_acc,2);
        }
        else{
            for(int j=0;j<select_paths[i].size();j++)
            {
                pos(0,j) = select_paths[i][j][0];
                pos(1,j) = select_paths[i][j][1];
                pos(2,j) = select_paths[i][j][2];
                // time(j) = 2;
            }
            traj_ = PolynomialTraj::minSnapTraj(pos,start_vel,end_vel,start_acc,end_acc,time);

        }

        std::cout << pos.array() << std::endl;
        
        traj_.init();
        visualization_->displayInitPathList(traj_.getTraj(),0.2,i);
    }
    

}                                                                                                                                          

int main(int argc,char **argv)
{
    ros::init(argc, argv, "topo_prm_test");
    ros::NodeHandle nh("~");

    nh.param("topo_prm/init_x", init_odom_.pose.pose.position.x, 0.0);
    nh.param("topo_prm/init_y", init_odom_.pose.pose.position.y, 0.0);
    nh.param("topo_prm/init_z", init_odom_.pose.pose.position.z, 0.0);
    ROS_INFO("Init odom: %f, %f, %f", init_odom_.pose.pose.position.x, init_odom_.pose.pose.position.y, init_odom_.pose.pose.position.z);
    ros::Subscriber waypoint_sub = nh.subscribe("/move_base_simple/goal", 1, waypointCallback);

    DspMap::Ptr map;
    map.reset(new DspMap);
    map->initMap(nh);

    topo_prm.reset(new TopoPRM);
    topo_prm->setEnvironment(map);
    topo_prm->init(nh);

    visualization_.reset(new PlanningVisualization(nh));



    ros::spin();

    return 0;
}