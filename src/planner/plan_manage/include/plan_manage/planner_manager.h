#ifndef _PLANNER_MANGER_H_
#define _PLANNER_MANGER_H_


#include <cstdlib>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/dsp_map.h>
#include <plan_env/obj_predictor.h>
#include <path_searching/topo_prm.h>
#include <plan_manage/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>

using std::vector;
class PlannerManager
{
public:

    PlannerManager(){}
    ~PlannerManager(){}


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */

    bool reboundReplan(     Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                            Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                            Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj); 
    bool reboundTest(       Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                            Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                            Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    void clearData();

    void sampleTrajs(Eigen::Vector3d &start_vel, Eigen::Vector3d &start_acc, Eigen::Vector3d &end_vel, Eigen::Vector3d &end_acc);
    void sampleTraj(int traj_id, Eigen::Vector3d &start_vel, Eigen::Vector3d &start_acc, Eigen::Vector3d &end_vel, Eigen::Vector3d &end_acc);
    void reboundTrajs();
    bool reboundTraj(int traj_id);
    bool refineTraj(int traj_id);

    // bool topoReplan(Eigen::Vector3d )
    bool emergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(   const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                    const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    bool sampleControlPoints(   Eigen::Vector3d &start_pt, Eigen::Vector3d &start_vel, Eigen::Vector3d &start_acc,
                                Eigen::Vector3d &end_pt, Eigen::Vector3d &end_vel,
                                std::vector<Eigen::Vector3d>& point_set, std::vector<Eigen::Vector3d> &start_end_derivatives,double &ts);

    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);


    void sortTopoTrajs(std::vector<UniformBspline> &trajs);
    double getTrajRisk(UniformBspline &traj);
    double getTrajRisk(Eigen::MatrixXd &ctrl_pts);
    double calcTrajCost(UniformBspline &traj);

    void drawSampleBox();

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    // TopoTrajData topo_data_;
    MidPlanData plan_data_;
    
    DspMap::Ptr dsp_map_;


    
    // int best_traj_id_;
    



private:
    PlanningVisualization::Ptr visualization_;

    unique_ptr<Astar> astar_;
    unique_ptr<TopoPRM> topo_prm_;
    // BsplineOptimizer::Ptr bspline_optimizer_rebound_;
    std::vector<BsplineOptimizer::Ptr> bspline_optimizer_rebound_;
    
    int continuous_failures_count_{0};



    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);
    
    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,double &time_inc);

    void optimizeTopoBspline(double start, double duration, vector<Eigen::Vector3d> guide_path, int traj_id);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);


    Eigen::MatrixXd reparamLocalTraj(double start_t, double& dt, double& duration);
    Eigen::MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double &dt);

    // topo optimize

    // void optimizeTopoBspline(double start, double duration, vector<Eigen::Vector3d> guide_path,int traj_id);

public:
    typedef unique_ptr<PlannerManager> Ptr;
};   


#endif

