
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <plan_manage/planner_manager.h>

// #include <plan_manage/ego_replan_fsm.h>

// using namespace ego_planner;

PlanningVisualization::Ptr visualization_;
PlannerManager::Ptr planner_manager_;
Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state 
Eigen::Quaterniond odom_orient_;
bool have_odom_;


void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
}

void waypointCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if(msg->pose.position.z < -0.1)
    {
        ROS_ERROR("Invalid waypoint");
        return;
    }
    std::cout << "Triggered" << std::endl;

    if(!have_odom_)
    {
        ROS_ERROR("No odom");
        return;
    }

    Eigen::Vector3d end_pt(msg->pose.position.x , msg->pose.position.y , 1.0);

    // bool success = false;
    // success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, odom_acc_, end_pt, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    // visualization_->displayGoalPoint(end_pt,Eigen::Vector4d(0, 0.5, 0.5, 1),0.3,0);
    // if(success)
    // {   
    //     ROS_INFO("Global planning success");
    //     /*** display ***/
    //     constexpr double step_size_t = 0.1;
    //     int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    //     vector<Eigen::Vector3d> gloabl_traj(i_end);
    //     for (int i = 0; i < i_end; i++)
    //     {
    //         gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    //     }

    //     visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    // }
    // else
    // {
    //     ROS_ERROR("Global planning failed");
    //     return ;
    // }

    bool success = planner_manager_->reboundTest(odom_pos_, odom_vel_, odom_acc_, end_pt, Eigen::Vector3d::Zero(), false , false);
    if(success)
    {
        ROS_INFO("Topo replan success");
        /*** display ***/
    }
    else
    {
        ROS_ERROR("Topo replan failed");
        return ;
    }


}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "ego_planner_node");
    ros::NodeHandle nh("~");

    ros::Subscriber odom_sub = nh.subscribe("/odometry",1,odomCallback);
    ros::Subscriber waypoint_sub = nh.subscribe("/move_base_simple/goal", 1, waypointCallback);


    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new PlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);



    // ros::Duration(1.0).sleep();

    // ros::MultiThreadedSpinner spinner(4);
    // spinner.spin();
    ros::spin();

    return 0;
}
