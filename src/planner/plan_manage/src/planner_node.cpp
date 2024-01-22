#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/topo_replan_fsm.h>

// using namespace ego_planner;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  TopoReplanFSM rebo_replan;

  rebo_replan.init(nh);

  // PlannerManager planner_manager_;
  // planner_manager_

  // ros::Duration(1.0).sleep();
  
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  // ros::AsyncSpinner async_spinner(4);
  // async_spinner.start();
  // ros::waitForShutdown();
  // ros::spin();
  
  return 0;
}

// #include <ros/ros.h>
// #include <csignal>
// #include <visualization_msgs/Marker.h>

// #include <plan_manage/ego_replan_fsm.h>

// using namespace ego_planner;

// void SignalHandler(int signal) {
//   if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
//     ros::shutdown();
//   }
// }

// int main(int argc, char **argv) {

//   signal(SIGINT, SignalHandler);
//   signal(SIGTERM,SignalHandler);

//   ros::init(argc, argv, "ego_planner_node", ros::init_options::NoSigintHandler);
//   ros::NodeHandle nh("~");

//   EGOReplanFSM rebo_replan;

//   rebo_replan.init(nh);

//   // ros::Duration(1.0).sleep();
//   ros::AsyncSpinner async_spinner(4);
//   async_spinner.start();
//   ros::waitForShutdown();

//   return 0;
// }