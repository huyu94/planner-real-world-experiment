#include <plan_manage/planner_manager.h>
#include <thread>

void PlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
{

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/topo_max_num", pp_.topo_max_num_, 10);
    nh.param<double>("manager/risk_weight",pp_.risk_weight_,1.0);
    nh.param<double>("manager/traj_risk_thresh",pp_.traj_risk_thresh_,-1.0);

    local_data_.traj_id_ = 0;
    dsp_map_.reset(new DspMap);
    dsp_map_->initMap(nh);

    topo_prm_.reset(new TopoPRM);
    topo_prm_->setEnvironment(dsp_map_);
    topo_prm_->init(nh);
    visualization_ = vis;

    bspline_optimizer_rebound_.resize(10);
    for(int i=0; i < 10; i++)
    {
        bspline_optimizer_rebound_[i].reset(new BsplineOptimizer);
        bspline_optimizer_rebound_[i]->setParam(nh);
        bspline_optimizer_rebound_[i]->setEnvironment(dsp_map_);
        bspline_optimizer_rebound_[i]->a_star_.reset(new Astar);
        bspline_optimizer_rebound_[i]->a_star_->initGridMap(dsp_map_, Eigen::Vector3i(100, 100, 100));
    }

    // for (int i = 0; i < pp_.topo_max_num_; i++)
    // {
    //     topo_traj_set_.emplace_back(TopoTraj());
    //     topo_traj_set_[i].optimizer_.reset(new BsplineOptimizer);
    //     topo_traj_set_[i].optimizer_->setParam(nh);
    //     topo_traj_set_[i].optimizer_->setEnvironment(dsp_map_);
    //     topo_traj_set_[i].optimizer_->a_star_.reset(new Astar);
    //     topo_traj_set_[i].optimizer_->a_star_->initGridMap(dsp_map_, Eigen::Vector3i(100, 100, 100));
    // }

    ROS_INFO("Planner manager initialized");
}


bool PlannerManager::reboundReplan(     Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
{
    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
        cout << "Close to goal" << endl;
        continuous_failures_count_++;
        return false;
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
        point_set.clear();
        start_end_derivatives.clear();
        flag_regenerate = false;

        if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
        {
            flag_first_call = false;
            flag_force_polynomial = false;

            PolynomialTraj gl_traj;

            double dist = (start_pt - local_target_pt).norm();
            double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

            if (!flag_randomPolyTraj)
            {
                std::cout << "one segment traj" << std::endl;
                gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
                std::cout << "one segment traj end" << std::endl;
            }
            else
            {
                std::cout << "mini snap traj " << std::endl;
                Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
                Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
                Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                                     (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                                     (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
                Eigen::MatrixXd pos(3, 3);
                pos.col(0) = start_pt;
                pos.col(1) = random_inserted_pt;
                pos.col(2) = local_target_pt;
                Eigen::VectorXd t(2);
                t(0) = t(1) = time / 2;
                gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
                std::cout << "mini snap traj end" << std::endl;
            }

            double t;
            bool flag_too_far;
            ts *= 1.5; // ts will be divided by 1.5 in the next
            std::cout << "start expand control points " << std::endl;
            do
            {
                ts /= 1.5;
                point_set.clear();
                flag_too_far = false;
                Eigen::Vector3d last_pt = gl_traj.evaluate(0);
                for (t = 0; t < time; t += ts)
                {
                    Eigen::Vector3d pt = gl_traj.evaluate(t);
                    if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
                    {
                        flag_too_far = true;
                        break;
                    }
                    last_pt = pt;
                    point_set.push_back(pt);
                }
            } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
            std::cout << "end expand control points " << std::endl;
            t -= ts;
            start_end_derivatives.push_back(gl_traj.evaluateVel(0));
            start_end_derivatives.push_back(local_target_vel);
            start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
            start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
        }
        else // Initial path generated from previous trajectory.
        {

            double t;
            double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();
            std::cout << "start pseudo_arc_length" << std::endl;
            vector<double> pseudo_arc_length;
            vector<Eigen::Vector3d> segment_point;
            pseudo_arc_length.push_back(0.0);
            for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
            {
                segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
                if (t > t_cur)
                {
                    pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
                }
            }
            t -= ts;

            double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
            std::cout << "start one_segment_traj_gen" << std::endl;

            if (poly_time > ts)
            {
                PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                              local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                              local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                              local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

                for (t = ts; t < poly_time; t += ts)
                {
                    if (!pseudo_arc_length.empty())
                    {
                        segment_point.push_back(gl_traj.evaluate(t));
                        pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
                    }
                    else
                    {
                        ROS_ERROR("pseudo_arc_length is empty, return!");
                        continuous_failures_count_++;
                        return false;
                    }
                }
            }
            std::cout << "end one_segment_traj_gen" << std::endl;

            double sample_length = 0;
            double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
            size_t id = 0;
            do
            {
                cps_dist /= 1.5;
                point_set.clear();
                sample_length = 0;
                id = 0;
                while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
                {
                    if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
                    {
                        point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                            (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
                        sample_length += cps_dist;
                    }
                    else
                        id++;
                }
                point_set.push_back(local_target_pt);
            } while (point_set.size() < 7); // If the start point is very close to end point, this will help

            std::cout << "end expand control points" << std::endl;  
            start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
            start_end_derivatives.push_back(local_target_vel);
            start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
            start_end_derivatives.push_back(Eigen::Vector3d::Zero());

            if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
            {
                flag_force_polynomial = true;
                flag_regenerate = true;
            }
        }
    } while (flag_regenerate);
    cout << "initialize control point success " << endl;

    visualization_->displayControlPoints(point_set, 0.2,Eigen::Vector4d(0,0.5,0.5,1.0), 0);

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts,point_set,start_end_derivatives,ctrl_pts);

    // plan_data_.clearTopoPaths();
    list<GraphNode::Ptr>            graph;
    vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
    vector<Eigen::Vector3d> start_pts, end_pts;

    topo_prm_->findTopoPaths(start_pt, local_target_pt, start_pts, end_pts, graph,
                            raw_paths, filtered_paths, select_paths);

    // ROS_INFO("find topo paths finished !");
    vector<vector<Eigen::Vector3d>> guide_pts;
    // ROS_INFO("topo path size : %d", select_paths.size());

    vector<UniformBspline> topo_trajs;
    // if(pp_.optimize_parallel_)
    // {
        
    // }
    // else
    // {

    // }
    for(int i=0;i<select_paths.size();i++)
    {
        Eigen::MatrixXd temp_ctrl_pts = ctrl_pts;

        guide_pts.push_back(topo_prm_->pathToGuidePts(select_paths[i], temp_ctrl_pts.cols() - 2));
        visualization_->displayControlPoints(guide_pts.back(), 0.2,Eigen::Vector4d(0.5,0.5,0.5,1.0), i + 1);
        guide_pts[i].pop_back();
        guide_pts[i].pop_back();
        guide_pts[i].erase(guide_pts[i].begin(),guide_pts[i].begin() + 2);
        if (guide_pts[i].size() != int(temp_ctrl_pts.cols()) - 6)
        {
            ROS_WARN("what guide");
            ROS_WARN("guide_pts size : %d", guide_pts[i].size());
            ROS_WARN("ctrl_pts size : %d", temp_ctrl_pts.cols());            
        } 
        bspline_optimizer_rebound_[i]->setGuidePath(guide_pts[i]);
        bspline_optimizer_rebound_[i]->initControlPoints(temp_ctrl_pts,true);

        bool success = bspline_optimizer_rebound_[i]->BsplineOptimizeTrajRebound(temp_ctrl_pts, ts);
        if(success)
        {
            // visualization_->displayOptimalList(temp_ctrl_pts, i + 20);
            visualization_->displayTopoPathList(temp_ctrl_pts,i);
            topo_trajs.emplace_back(temp_ctrl_pts,3,ts);
        }
        else
        {
            ROS_ERROR("rebound fail");
        }
    }
    // ROS_INFO("optimize finished ! start sort ");
    if(topo_trajs.size() == 0)
    {
        ROS_WARN("no trajectory!");
        return false;
    }
    sortTopoTrajs(topo_trajs);
    // ROS_INFO("finish sort , start refine");

    UniformBspline pos = topo_trajs[0];
    pos.setPhysicalLimits(pp_.max_vel_,pp_.max_acc_,pp_.feasibility_tolerance_);
    double ratio;
    bool flag_step_2_success = true;
    if(!pos.checkFeasibility(ratio,false))
    {
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
        {
            pos = UniformBspline(optimal_control_points, 3, ts);
        }
    }

    if(!flag_step_2_success)
    {
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continuous_failures_count_++;
        return false;
    }

    // ROS_INFO("finish refine, start update traj info");

    updateTrajInfo(pos,ros::Time::now());

    // ROS_INFO("finish update !");

    continuous_failures_count_ = 0;

    return true;
}



void PlannerManager::sortTopoTrajs(std::vector<UniformBspline> &trajs)
{
    sort(trajs.begin(),trajs.end(),
        [&](UniformBspline& tj1, UniformBspline& tj2){
            return tj1.getJerk() + getTrajRisk(tj1) < tj2.getJerk() + getTrajRisk(tj2);
            });
}



bool PlannerManager::reboundTest(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt, Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
{
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;

    PolynomialTraj gl_traj;

    double dist = (start_pt - local_target_pt).norm();
    double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

    if (!flag_randomPolyTraj)
    {
        gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
    }
    else
    {
        Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
        Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                                (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                                (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
        Eigen::MatrixXd pos(3, 3);
        pos.col(0) = start_pt;
        pos.col(1) = random_inserted_pt;
        pos.col(2) = local_target_pt;
        Eigen::VectorXd t(2);
        t(0) = t(1) = time / 2;
        gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
    }

    double t;
    bool flag_too_far;
    ts *= 1.5; // ts will be divided by 1.5 in the next
    do
    {
        ts /= 1.5;
        point_set.clear();
        flag_too_far = false;
        Eigen::Vector3d last_pt = gl_traj.evaluate(0);
        for (t = 0; t < time; t += ts)
        {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
                flag_too_far = true;
                break;
            }
            last_pt = pt;
            point_set.push_back(pt);
        }
    } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
    t -= ts;
    start_end_derivatives.push_back(gl_traj.evaluateVel(0));
    start_end_derivatives.push_back(local_target_vel);
    start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
    start_end_derivatives.push_back(gl_traj.evaluateAcc(t));

    visualization_->displayControlPoints(point_set, 0.2,Eigen::Vector4d(0,0.5,0.5,1.0), 0);

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts,point_set,start_end_derivatives,ctrl_pts);

    // plan_data_.clearTopoPaths();
    list<GraphNode::Ptr>            graph;
    vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
    vector<Eigen::Vector3d> start_pts, end_pts;

    topo_prm_->findTopoPaths(start_pt, local_target_pt, start_pts, end_pts, graph,
                            raw_paths, filtered_paths, select_paths);

    vector<vector<Eigen::Vector3d>> guide_pts;
    for(int i=0;i<select_paths.size();i++)
    {
        guide_pts.push_back(topo_prm_->pathToGuidePts(select_paths[i], ctrl_pts.cols() - 2));
        visualization_->displayControlPoints(guide_pts.back(), 0.2,Eigen::Vector4d(0.5,0.5,0.5,1.0), i + 1);
        guide_pts[i].pop_back();
        guide_pts[i].pop_back();
        guide_pts[i].erase(guide_pts[i].begin(),guide_pts[i].begin() + 2);
        if (guide_pts[i].size() != int(ctrl_pts.cols()) - 6)
        {
            ROS_WARN("what guide");
            ROS_WARN("guide_pts size : %d", guide_pts[i].size());
            ROS_WARN("ctrl_pts size : %d", ctrl_pts.cols());            
        } 
        bspline_optimizer_rebound_[i]->setGuidePath(guide_pts[i]);
        bspline_optimizer_rebound_[i]->initControlPoints(ctrl_pts,true);

        bool success = bspline_optimizer_rebound_[i]->BsplineOptimizeTrajRebound(ctrl_pts, ts);
        if(success)
        {
            // visualization_->displayOptimalList(ctrl_pts, i + 20);
        }
        else
        {
            ROS_ERROR("rebound fail");
        }
    }

    return true;

}



// void PlannerManager::clearData()
// {
//     for (int i = 0; i < pp_.topo_max_num_; i++)
//     {
//         topo_traj_set_[i].clear();
//     }
// }

// void PlannerManager::sampleTrajs(Eigen::Vector3d &start_vel, Eigen::Vector3d &start_acc, Eigen::Vector3d &end_vel, Eigen::Vector3d &end_acc)
// {
//     for (int i = 0; i < topo_path_num_; i++)
//     {
//         sampleTraj(i, start_vel, start_acc, end_vel, end_acc);
//     }
// }

// void PlannerManager::sampleTraj(int traj_id, Eigen::Vector3d &start_vel, Eigen::Vector3d &start_acc, Eigen::Vector3d &end_vel, Eigen::Vector3d &end_acc)
// {
//     if (topo_traj_set_[traj_id].topo_select_paths_.size() < 2) // straint line
//     {
//         ROS_ERROR("topo path [%d] size < 2", traj_id);
//         return;
//     }

//     Eigen::Vector3d start_pt = topo_traj_set_[traj_id].topo_select_paths_[0];
//     Eigen::Vector3d local_target_pt = topo_traj_set_[traj_id].topo_select_paths_.back();

//     topo_traj_set_[traj_id].ts_ = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits

//     PolynomialTraj gl_traj;
//     double dist = (start_pt - local_target_pt).norm();
//     double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;
//     // ROS_INFO("time : %f" , time);
//     /* straight line */
//     topo_traj_set_[traj_id].point_set_ = topo_prm_->pathToGuidePts(topo_traj_set_[traj_id].topo_select_paths_, int(time / topo_traj_set_[traj_id].ts_) + 1);
//     topo_traj_set_[traj_id].start_end_derivatives_.push_back(start_vel);
//     topo_traj_set_[traj_id].start_end_derivatives_.push_back(end_vel);
//     topo_traj_set_[traj_id].start_end_derivatives_.push_back(start_acc);
//     topo_traj_set_[traj_id].start_end_derivatives_.push_back(end_acc);
//     // ROS_INFO("start_end_derivative size : %d", start_end_derivatives_[traj_id].size());
//     visualization_->displayInitPathList(topo_traj_set_[traj_id].point_set_, 0.2, traj_id);
//     visualization_->displayControlPoints(topo_traj_set_[traj_id].point_set_, 0.2, Eigen::Vector4d(0, 0.2, 0.3, 1), traj_id);
// }

// void PlannerManager::reboundTrajs()
// {
//     for (int i = 0; i < topo_path_num_; i++)
//     {
//         reboundTraj(i);
//     }
// }

// bool PlannerManager::reboundTraj(int traj_id)
// {
//     auto &point_set = topo_traj_set_[traj_id].point_set_;
//     auto &ctrl_pts = topo_traj_set_[traj_id].ctrl_pts_;
//     auto &start_end_derivative = topo_traj_set_[traj_id].start_end_derivatives_;
//     auto &optimizer = topo_traj_set_[traj_id].optimizer_;
//     auto &traj = topo_traj_set_[traj_id].spline_;
//     auto &ts = topo_traj_set_[traj_id].ts_;
//     auto &a_star_pathes = topo_traj_set_[traj_id].a_star_pathes_;
//     UniformBspline::parameterizeToBspline(ts,
//                                           point_set,
//                                           start_end_derivative,
//                                           ctrl_pts);
//     // a_star_pathes = optimizer->initControlPoints(topo_traj_set_[traj_id])
//     a_star_pathes = optimizer->initControlPoints(topo_traj_set_[traj_id].ctrl_pts_, true);
//     visualization_->displayAStarList(a_star_pathes, traj_id);

//     bool flag_optimize_succes = false;
//     flag_optimize_succes = optimizer->BsplineOptimizeTrajRebound(ctrl_pts, ts);
//     if (!flag_optimize_succes)
//     {
//         ROS_WARN("topo traj [%d] fail!", traj_id);
//         return false;
//     }
//     visualization_->displayOptimalList(ctrl_pts, traj_id);
//     traj = UniformBspline(ctrl_pts, 3, ts);

//     return true;
// }

// bool PlannerManager::refineTraj(int traj_id)
// {

//     auto &ctrl_pts = topo_traj_set_[traj_id].ctrl_pts_;
//     auto &start_end_derivative = topo_traj_set_[traj_id].start_end_derivatives_;
//     auto &optimizer = topo_traj_set_[traj_id].optimizer_;
//     auto &traj = topo_traj_set_[traj_id].spline_;
//     auto &ts = topo_traj_set_[traj_id].ts_;

//     traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
//     double ratio;
//     bool flag_refine_success = false;
//     if (!traj.checkFeasibility(ratio, false))
//     {
//         cout << "Need to reallocate time." << endl;

//         Eigen::MatrixXd optimal_control_points;

//         double t_inc;

//         double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
//         optimizer->ref_pts_.clear();
//         for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
//         {
//             optimizer->ref_pts_.push_back(traj.evaluateDeBoorT(t));
//         }
//         bool success = optimizer->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

//         if (!success)
//         {
//             ROS_WARN("refine fail");
//             return false;
//         }
//     }

//     return true;
// }

bool PlannerManager::emergencyStop(Eigen::Vector3d stop_pos)
{
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
        control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
}

bool PlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                             const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
{
    // generate global reference trajectory
    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
        points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
        total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
        inter_points.push_back(points.at(i));
        double dist = (points.at(i + 1) - points.at(i)).norm();

        if (dist > dist_thresh)
        {
            int id_num = floor(dist / dist_thresh) + 1;

            for (int j = 1; j < id_num; ++j)
            {
                Eigen::Vector3d inter_pt =
                    points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                inter_points.push_back(inter_pt);
            }
        }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
        pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
        time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
        gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
        gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
        return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
}

bool PlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                    const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
{
    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
        inter_points.push_back(points.at(i));
        double dist = (points.at(i + 1) - points.at(i)).norm();

        if (dist > dist_thresh)
        {
            int id_num = floor(dist / dist_thresh) + 1;

            for (int j = 1; j < id_num; ++j)
            {
                Eigen::Vector3d inter_pt =
                    points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                inter_points.push_back(inter_pt);
            }
        }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
        pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
        time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
        gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
        gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
        return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
}

Eigen::MatrixXd PlannerManager::reparamLocalTraj(double start_t, double &dt, double &duration)
{
    vector<Eigen::Vector3d> point_set;
    vector<Eigen::Vector3d> start_end_derivative;

    global_data_.getTrajByRadius(start_t, pp_.planning_horizen_, pp_.ctrl_pt_dist, point_set, start_end_derivative, dt, duration);

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
    plan_data_.local_start_end_derivative_ = start_end_derivative;
    return ctrl_pts;
}

Eigen::MatrixXd PlannerManager::reparamLocalTraj(double start_t, double duration, int seg_num, double &dt)
{
    vector<Eigen::Vector3d> point_set;
    vector<Eigen::Vector3d> start_end_derivative;

    global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
    plan_data_.local_start_end_derivative_ = start_end_derivative;

    /* parameterization of B-spline */
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);

    return ctrl_pts;
}

void PlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
{
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
}

void PlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
{
    double time_origin = bspline.getTimeSum();

    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
        point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
}

void PlannerManager::optimizeTopoBspline(double start, double duration, vector<Eigen::Vector3d> guide_path, int traj_id)
{
    ros::Time t1;
    double tm1, tm2, tm3;

    t1 = ros::Time::now();

    int seg_num = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
    Eigen::MatrixXd ctrl_pts;
    double dt;

    ctrl_pts = reparamLocalTraj(start, duration, seg_num, dt); // 重新参数化了未执行的局部轨迹的控制点

    // 把从当前位置拓扑规划的轨迹的控制点，插入到局部轨迹的控制点中
    vector<Eigen::Vector3d> guide_pt;
    guide_pt = topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2); // std::vector<Eigen::Vector3d> guide_pt -> Eigen::MatrixXd ctrl_pts;
    // 去除前两个和后两个控制点
    guide_pt.pop_back();
    guide_pt.pop_back();
    guide_pt.erase(guide_pt.begin(), guide_pt.begin() + 2);
    if (guide_pt.size() != int(ctrl_pts.rows()) - 6)
    {
        ROS_WARN("what guide");
    }

    tm1 = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // bspline_optimizers_[traj_id]->setGuidePath(guide_pt);
    // Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[traj_id]->BsplineOptimizeTrajRebound(ctrl_pts, )
}

bool PlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
{
    double t_inc;

    Eigen::MatrixXd ctrl_pts;

    reparamBspline(traj,start_end_derivative,ratio,ctrl_pts,ts,t_inc);

    traj = UniformBspline(ctrl_pts,3,ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_[0]->ref_pts_.clear();
    for(double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
    {
        bspline_optimizer_rebound_[0]->ref_pts_.push_back(traj.evaluateDeBoorT(t));
    }

    bool success = bspline_optimizer_rebound_[0]->BsplineOptimizeTrajRefine(ctrl_pts,ts,optimal_control_points);

    return success;
}

double PlannerManager::getTrajRisk(UniformBspline &traj)
{
    double risk = 0;
    Eigen::MatrixXd ctrl_pts = traj.getControlPoint();
    for(int i = 0; i < ctrl_pts.cols(); i ++)
    {
        Eigen::Vector3d ps = ctrl_pts.col(i);
        risk += dsp_map_->getVoxelFutureRisk(ps);
    }
    return risk * pp_.risk_weight_;
}