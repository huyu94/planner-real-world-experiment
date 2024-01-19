#ifndef _DSP_MAP_H_
#define _DSP_MAP_H_
// #pragma once 
#include <iostream>
#include <random>
#include <queue>
#include <array>
#include <thread>
#include <memory>
#include "munkres.h"

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>


#define logix(x) (log((x) / (1 - (x))))
#define GRID_MAP_OBS_FLAG 32767
#define GRID_MAP_NEW_PLATFORM_TEST false
#define LIMIT_MOVEMENT_IN_XY_PLANE 1 // 是否限制障碍物预测在XY平面的运动
#define O_MAKE_VALID 1
#define O_MAKE_INVALID 0


using namespace std;
using Vector3d = Eigen::Vector3d;
using Vector3i = Eigen::Vector3i;
using Vector3f = Eigen::Vector3f;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Quaterniond = Eigen::Quaterniond;

struct Particle
{
    Vector3d position;
    Vector3d velocity;
    float weight;
    int voxel_index;
};

struct ClusterFeature{
    Vector3d center = {0.0,0.0,0.0};
    int point_num = 0;
    int match_cluster_seq = -1;
    Vector3d velocity = {-10000.0f,-10000.0f,-10000.0f};
    float velocityNorm = 0.f;
    float intensity = 0.f;
};

struct MappingParamters
{
    bool have_initialized_ = false;
    Vector3d local_update_range3d_; // half of update range double 
    Vector3i local_update_range3i_; // half of update range int


    string frame_id_; // 
    int pose_type_; // 1. poseStamped; 2. odometry
    int sensor_type_; // 1. lidar; 2. camera
    bool enable_virtual_wall_; // 是否允许虚拟墙体出现
    double virtual_ceil_, virtual_ground_; // 虚拟天花板、虚拟地面
    
    /* enable future prediction */
    bool enable_future_prediction_;
    double prediction_z_height_;
    double prediciton_y_offset_;

    
    /* time */
    double odom_lidar_timeout_;
    

    /* visualization  and computation time display */
    bool show_occ_time_;

    /* particle map */
    /* resolution */
    float voxel_resolution_;// 体素分辨率
    float voxel_resolution_inv_;
    float angle_resolution_; // 角度分辨率
    float angle_resolution_inv_;
    double voxel_filter_resolution_; // 体素滤波的分辨率
    // float angle_resolution_rad_; //角度分辨率-弧度制
    // float half_angle_resolution_rad_; //角度分辨率的一一半-弧度制
    const float one_degree_rad_ = M_PI / 180;
    const float one_rad_degree_ = 180 / M_PI;
    // int half_fov_horizontal_;
    int fov_horizontal_lowbound_;
    int fov_horizontal_upbound_;
    float fov_horizontal_lowbound_rad_;
    float fov_horizontal_upbound_rad_;
    // int half_fov_vertical_;
    int fov_vertical_lowbound_;
    int fov_vertical_upbound_;
    float fov_vertical_lowbound_rad_;
    float fov_vertical_upbound_rad_;


    /* voxel subspace  */ 
    // int voxel_num_; 
    int max_particle_num_in_voxel_;
    int safe_particle_num_in_voxel_;
    int safe_particle_num_in_pyramid_;// fov空间中最大的粒子数量
    int voxel_objects_number_dimension; // voxel object number 第二维的维度

    // int max_particle_num_in_voxel_; // 每个体素子空间中粒子的最大数量
    
    /* pyramid subspace */
    int pyramid_num_;
    int observation_pyramid_num_;
    int observation_max_points_num_one_pyramid_; // fov空间中最大的观测点数量
    int pyramid_neighbor_one_dimension_; // 单方向上的邻居个数
    int pyramid_neighbor_num_; // 单个金字塔空间邻居数量最大值
    int observation_pyramid_num_horizontal_; 
    int observation_pyramid_num_vertical_; 

    /* prediction future */
    int prediction_time_; // 预测时间区间
    std::vector<float> prediction_future_time_;

    /* velocity estimation */
    int dynamic_cluster_max_point_num_;
    float dynamic_cluster_max_center_height_;
    float distance_gate_;
    float point_num_gate_;
    float maximum_velocity_;
    
    /* camera */
    double cx_, cy_, fx_, fy_;
    double k_depth_scaling_factor_;
    int skip_pixel_;
    bool use_depth_filter_;
    /* non const */

    /* particle output */
    bool if_record_particle_csv;
    string particle_save_folder;

    /* map upadte parameters */
    double position_prediction_stddev; //位置估计方差
    double velocity_prediction_stddev; //速度估计方差
    double kappa;
    double sigma_ob;
    double P_detection; //被检测到的概率
    double new_born_particle_weight_;
    int new_born_particle_number_each_point_;
    int position_guassian_random_seq_;
    int velocity_gaussian_random_seq_;

    /* new born */
    float expected_new_born_objects_;
    float new_born_each_object_weight_;

    /* random param*/
    const int guassian_random_num_ = 1000000;
    vector<float> p_gaussian_randoms;
    vector<float> v_gaussian_randoms;
    const int standard_gaussian_pdf_num_ = 20000;
    vector<float> standard_gaussian_pdf;

/*===============================================================*/

    const int max_point_num_ = 5000; // input cloud max size 


    /* occupancy param */
    double occupancy_thresh_;  // 占据阈值
    double risk_thresh_; // 风险阈值
    int risk_point_gate_; // 非对称障碍物中心点数量阈值

    /* inflation */
    int inf_grid_; // inflate grid number 
    double obstacles_inflation_; // inflat grid size double 



};


struct MappingData
{
    /* center and origin */
    Vector3i center_last3i_;
    /* ringbuffer */
    Vector3i ringbuffer_origin3i_;
    Vector3d ringbuffer_lowbound3d_;
    Vector3i ringbuffer_lowbound3i_;
    Vector3d ringbuffer_upbound3d_;
    Vector3i ringbuffer_upbound3i_;
    Vector3i ringbuffer_size3i_;
    int buffer_size_;
    /* inflation */
    Vector3i ringbuffer_inf_origin3i_;
    Vector3d ringbuffer_inf_lowbound3d_;
    Vector3i ringbuffer_inf_lowbound3i_;
    Vector3d ringbuffer_inf_upbound3d_;
    Vector3i ringbuffer_inf_upbound3i_;
    Vector3i ringbuffer_inf_size3i_;
    int inf_buffer_size_;

    /* particle map */
    //根据体素存储对应粒子，[VOXEL_NUM]体素数量，SAFE_PARTICLE_NUM_VOXEL体素内最大粒子数。
    // 1. flag: 0.0f invalid, 1.0f valid but not newborn 3.0f valid newborn 7.0f recently predicted 
    // 2.vx 3.vy 4.vz 5.px 6.py 7.pz
    // 8.weight 9.update time
    // flag 要不要改？ 不确定
    vector<vector<vector<float>>> voxels_with_particles;
    // 对应体素的信息 : 0. objects sum weights; 1-3. Avg vx, vy, vz; 4-8. Future objects number  9. 是否这个子空间后来被更新过，也就是mapmove出去的地图空间要置0.f
    // static float voxels_objects_number[VOXEL_NUM][voxels_objects_number_dimension];
    vector<vector<float>> voxels_objects_number; 
    // list for the future status of voxels
    // 改了一下，直接计算区间内的未来风险
    vector<float> future_status;
    // list for pyramids in fov，用来存放金字塔空间中的粒子
    // observation_pyramid_num fov视角里金字塔子空间的数量，SAFE_PARTICLE_NUM_PYRAMID 每个金字塔子空间中粒子的最大数量
    // [pyramid num][safe particle in each pyramid][0-1-2]
    // 0.flag:{0 invalid;1:valid}
    // 1.particle voxel index  2.particle index inside voxel，在金字塔子空间中的粒子个数
    vector<vector<vector<int>>> pyramids_in_fov;
    // 金字塔子空间的邻居数组，1维是子空间下标，2维{0:邻居数，1-end:邻居下标}
    vector<vector<int>> observation_pyramid_neighbours; 
    // list for observed point cloud in pyramid of fov, 用来存放金字塔空间中的观测点 
    // 每个金字塔空间中的观测点的情况，即为观测点，point_cloud_in_pyramid
    // 0.px, 1.py, 2.pz 3.acc 4.length for later usage
    // row:金字塔体素数量，column:每个金字塔空间最大观测点数
    vector<vector<vector<float>>> point_cloud_in_pyramid;
    vector<int> point_num_in_pyramid; // 用来fov金字塔空间中的观测点数量
    // list for the max depth of pyramid of fov 
    
    vector<float> input_points_; // odom+lidar-callback input points
    // list for the number of observed point cloud in pyramid of fov
    vector<float> max_depth_in_pyramid; // 用来fov金字塔空间中的深度
    // inflation 
    vector<uint16_t> occupancy_buffer_inflate_;

    /* velocity estimation */
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_; // 传进来的全局点云 
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud_with_velocity_; // 速度估计后的点云
    std::vector<ClusterFeature> clusters_last_;   // 上一帧的分类的cluster                                                            

    /* variables */

    int update_time_update_counter_; //参数update_time_的更新次数
    int update_times_; // 方法mapUpdate的更新次数



    /* time */
    ros::Time start_time_;
    ros::Time last_update_time_;
    ros::Time current_update_time_;



    // camera position and pose data
    // Vector3d lidar_position_, last_lidar_position_;
    // Quaterniond lidar_rotation_, last_lidar_rotation_;
    // Eigen::Affine3d lidar2world_,last_lidar2world_;
    Eigen::Affine3d body2world_,last_body2world_;
    Eigen::Affine3d camera2body_;
    // Matrix4d lidar2body_;

    /* camera */
    cv::Mat depth_image_, last_depth_image_;


    // flags of map state
    bool occ_need_update_, local_updated_;
    bool has_first_lidar_;
    bool has_odom_;

    // odom_depth_timeout
    bool flag_lidar_odom_timeout_;
    bool flag_have_ever_received_lidar_;

    // inflation
    int inf_grid_;
    double obstacles_inflation_;

    // queue<Eigen::Vector3i> rising_queue_;
    // queue<Eigen::Vector3i> falling_queue_;




    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class DspMap
{
public: 
    DspMap(){}
    ~DspMap() {}

    void initMap(ros::NodeHandle &nh);
    bool getOccupancy(const Vector3d& pos);
    bool getOccupancy(const Vector3i &idx);
    bool getInflateOccupancy(const Vector3d &pos);
    bool getInflateOccupancy(const Vector3i &idx);
    double getVoxelFutureRisk(const Vector3d &pos);
    double getVoxelFutureRisk(const Vector3i &idx);
    bool getVoxelFutureDangerous(const Vector3d &pos);
    bool getVoxelFutureDangerous(const Vector3i &idx);
    inline double getResolution();
    inline Vector3d getOrigin();
    bool getOdomLidarTimeout(){return md_.flag_lidar_odom_timeout_;}

    typedef std::shared_ptr<DspMap> Ptr;


    inline Vector3d globalIdx2Pos(const Vector3i& idx);
    inline Vector3i pos2GlobalIdx(const Vector3d& pos);
    inline int globalIdx2BufIdx(const Vector3i& id);
    inline int globalIdx2InfBufIdx(const Vector3i& id);
    inline Vector3i bufIdx2GlobalIdx(size_t address);
    inline Vector3i infBufIdx2GlobalIdx(size_t address);
    inline bool isInBuf(const Vector3d &pos);
    inline bool isInBuf(const float x, const float y, const float z);
    inline bool isInBuf(const Vector3i &idx);
    inline bool isInInfBuf(const Vector3d &pos);
    inline bool isInInfBuf(const Vector3i &idx);


    /* inflation */
    inline void changeInfBuf(const bool dir, const int inf_buf_idx, Eigen::Vector3i global_idx);
    // inline void inflatePoint(const Eigen::Vector3i &pt, int inf_step, vector<Eigen::Vector3i> &inf_pts);
    


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    MappingParamters mp_;
    MappingData md_;


    void publishMap();
    void publishFutureStatus();
    void publishMapInflate();
    void publishBoundary();
    void publishCloud();
    /* receive callback */
    void lidarOdomCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                           const nav_msgs::OdometryConstPtr &odom_msg);
    void lidarPoseCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                           const geometry_msgs::PoseStampedConstPtr &pose_msg);
    void DepthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                           const nav_msgs::OdometryConstPtr &odom_msg);
    void projectDepthImage();
    void DepthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                           const geometry_msgs::PoseStampedConstPtr &pose_msg);
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg);
    /* publish callback */
    void visCallback(const ros::TimerEvent &e);
    void updateOccupancyCallback(const ros::TimerEvent &e);


    /* ring buffer */
    void clearBuffer(char casein, int bound);
    void moveRingBuffer();
    
    /* ====================================================================================== */

    /* particle core */
    bool addAParticle(shared_ptr<Particle> p, int voxel_index) ;
    void addRandomParticles(int particle_num, float avg_weight);

    int moveAParticle(int new_voxel_index, int current_v_index, int current_v_inner_index);
    // int moveAParticle(const int& new_voxel_index, const int current_v_index, int current_v_inner_index, float *ori_particle_flag_ptr);
    
    void mapPrediction();
    void mapUpdate();
    void mapAddNewBornParticleByObservation();
    void mapOccupancyCalculationAndResample();
    void mapCalculateFutureStatus();
    void clearOccupancyMapPrediction();
    // generate random number;
    void generateGaussianRandomsVectorZeroCenter();
    float getPositionGaussianZeroCenter();
    float getVelocityGaussianZeroCenter();
    bool inPyramidsAreaInSensorFrame(float x,float y,float z);
    int findPointPyramidHorizontalIndexInSensorFrame(float x,float y,float z);
    int findPointPyramidVerticalIndexInSensorFrame(float x,float y,float z);
    void getKMClusterResult(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cluster_cloud);



    inline int setCacheOccupancy(Vector3d pos, int occ);
    Vector3d closestPointInMap(const Vector3d &pt, const Vector3d &center);
    void testIndexingCost();
    bool checkLidarOdomNeedUpdate();
    void initMapBoundary();

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyLidarOdom;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> SyncPolicyLidarPose;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> SyncPolicyDepthOdom;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> SyncPolicyDepthPose;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyLidarOdom>> SynchronizerLidarOdom;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyLidarPose>> SynchronizerLidarPose;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyDepthOdom>> SynchronizerDepthOdom;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyDepthPose>> SynchronizerDepthPose;

    ros::NodeHandle node_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> lidar_sub_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;

    SynchronizerLidarOdom sync_lidar_odom_;
    SynchronizerLidarPose sync_lidar_pose_;
    SynchronizerDepthOdom sync_depth_odom_;
    SynchronizerDepthPose sync_depth_pose_;

    ros::Subscriber indep_cloud_sub_, indep_odom_sub_, indep_pose_sub_;
    ros::Publisher map_pub_, map_inflate_pub_,map_future_pub_,pose_pub_,aabb_pub_,cloud_pub_ ;
    ros::Publisher map_boundary_pub_;
    ros::Timer occ_update_timer_, vis_timer_;

    uniform_real_distribution<double> rand_noise_;
    normal_distribution<double> rand_noise2_;
    default_random_engine rand_engine_;

    void velocityEstimationThread();
    void setOriginVoxelFilterResolution(float res);
    void findPyramidNeighborIndexInFOV(int index);
    void removeParticle(int voxel_index,int voxel_inner_index);
    float standardNormalPDF(float value);
    void calculateNormalPDFBuffer();
    float queryNormalPDF(float x,float mu, double sigma);
    void transformParticleToSensorFrame(const Vector3d &oriPoint,Vector3d& transformPoint);
    static float clusterDistance(ClusterFeature &c1, ClusterFeature &c2);

    static float generateRandomFloat(float min, float max);
    static void colorAssign(int &r,int &g,int &b,float v,float value_min,float value_max,int reverse_color);

    inline void setPredictionVariance(float p_stddev, float v_stddev);
    inline void setObservationStdDev(float ob_stddev);
    inline void setNewBornParticleWeight(float weight);
    inline void setNewBornParticleNumberofEachPoint(int num);
};

/* =================================================== definition of inline function 
 * =============================================*/

// inline int DspMap::setCacheOccupancy(Vector3d pos, int occ)
// {
//     if(occ != 1 && occ != 0)
//         return INVALID_IDX;
    
//     Vector3i id = pos2GlobalIdx(pos); //  pos -> global index
//     int idx_ctns = globalIdx2BufIdx(id); // global index -> buffer address 
    
//     md_.count_hit_and_miss_[idx_ctns] += 1;
//     if(md_.count_hit_and_miss_[idx_ctns] == 1)
//     {
//         md_.cache_voxel_[md_.cache_voxel_cnt_++] = id;
//     }
    
//     if(occ == 1)
//     {
//         md_.count_hit_[idx_ctns] += 1;
//     }
//     return idx_ctns;
// }

/* particle map ==========================================================*/
/// @brief 设置位置预测和速度预测的方差
/// @param p_stddev 传入的位置方差
/// @param v_stddev 传入的速度方差
inline void DspMap::setPredictionVariance(float p_stddev,float v_stddev){
    mp_.position_prediction_stddev = p_stddev;
    mp_.velocity_prediction_stddev = v_stddev;
    // regenerate randoms
    generateGaussianRandomsVectorZeroCenter();
}

/// @brief 设置观测方差
/// @param ob_stddev 
inline void DspMap::setObservationStdDev(float ob_stddev){
    mp_.sigma_ob = ob_stddev;
}

/// @brief 设置新产生粒子的权重
inline void DspMap::setNewBornParticleWeight(float weight){
    mp_.new_born_particle_weight_ = weight;
}


inline void DspMap::setNewBornParticleNumberofEachPoint(int num){
    mp_.new_born_particle_number_each_point_ = num;
}


/* ============================================================= */



inline Vector3d DspMap::getOrigin()
{
    // ROS_WARN("STILL NOT IMPLEMENTED");
    return md_.ringbuffer_lowbound3d_;
    // return md_.ringbuffer_origin3d_;
}


inline Vector3i DspMap::pos2GlobalIdx(const Vector3d &pos)
{
    return (pos * mp_.voxel_resolution_inv_).array().floor().cast<int>(); // more than twice faster than std::floor()
}

inline Vector3d DspMap::globalIdx2Pos(const Vector3i &idx)
{
    return Vector3d((idx(0) + 0.5) * mp_.voxel_resolution_, (idx(1) + 0.5) * mp_.voxel_resolution_, (idx(2) + 0.5) * mp_.voxel_resolution_);
}

/* global idx ------> buffer idx*/
inline int DspMap::globalIdx2BufIdx(const Vector3i &id)
{
    int x_buffer = (id(0) - md_.ringbuffer_origin3i_(0)) % md_.ringbuffer_size3i_(0);
    int y_buffer = (id(1) - md_.ringbuffer_origin3i_(1)) % md_.ringbuffer_size3i_(1);
    int z_buffer = (id(2) - md_.ringbuffer_origin3i_(2)) % md_.ringbuffer_size3i_(2);
    if(x_buffer < 0){
        x_buffer += md_.ringbuffer_size3i_(0);
    }
    if(y_buffer < 0){
        y_buffer += md_.ringbuffer_size3i_(1);
    }
    if(z_buffer < 0){
        z_buffer += md_.ringbuffer_size3i_(2);
    }

    return md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1) * z_buffer + md_.ringbuffer_size3i_(0) * y_buffer + x_buffer;
}

/* global idx ------> inflate buffer idx */
inline int DspMap::globalIdx2InfBufIdx(const Vector3i &id)
{
    int x_buffer = (id(0) - md_.ringbuffer_inf_origin3i_(0)) % md_.ringbuffer_inf_size3i_(0);
    int y_buffer = (id(1) - md_.ringbuffer_inf_origin3i_(1)) % md_.ringbuffer_inf_size3i_(1);
    int z_buffer = (id(2) - md_.ringbuffer_inf_origin3i_(2)) % md_.ringbuffer_inf_size3i_(2);
    if(x_buffer < 0){
        x_buffer += md_.ringbuffer_inf_size3i_(0);
    }
    if(y_buffer < 0){
        y_buffer += md_.ringbuffer_inf_size3i_(1);
    }
    if(z_buffer < 0){
        z_buffer += md_.ringbuffer_inf_size3i_(2);
    }

    return md_.ringbuffer_inf_size3i_(0) * md_.ringbuffer_inf_size3i_(1) * z_buffer + md_.ringbuffer_inf_size3i_(0) * y_buffer + x_buffer;
}

inline Vector3i DspMap::bufIdx2GlobalIdx(size_t address)
{
    const int ringbuffer_xysize = md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1);
    int zid_in_buffer = address / ringbuffer_xysize;
    address %= ringbuffer_xysize;
    int yid_in_buffer = address / md_.ringbuffer_size3i_(0);
    int xid_in_buffer = address % md_.ringbuffer_size3i_(0);

    int xid_global = xid_in_buffer + md_.ringbuffer_origin3i_(0);
    if(xid_global > md_.ringbuffer_upbound3i_(0))
    {
        xid_global -= md_.ringbuffer_size3i_(0);
    }
    int yid_global = yid_in_buffer + md_.ringbuffer_origin3i_(1);
    if(yid_global > md_.ringbuffer_upbound3i_(1))
    {
        yid_global -= md_.ringbuffer_size3i_(1);
    }
    int zid_global = zid_in_buffer + md_.ringbuffer_origin3i_(2);
    if(zid_global > md_.ringbuffer_upbound3i_(2))
    {
        zid_global -= md_.ringbuffer_size3i_(2);
    }
    
    return Vector3i(xid_global,yid_global,zid_global);
}


inline Vector3i DspMap::infBufIdx2GlobalIdx(size_t address)
{
    const int ringbuffer_xysize = md_.ringbuffer_inf_size3i_(0) * md_.ringbuffer_inf_size3i_(1);
    int zid_in_buffer = address / ringbuffer_xysize;
    address %= ringbuffer_xysize;
    int yid_in_buffer = address / md_.ringbuffer_inf_size3i_(0);
    int xid_in_buffer = address % md_.ringbuffer_inf_size3i_(0);

    int xid_global = xid_in_buffer + md_.ringbuffer_inf_origin3i_(0);
    if(xid_global > md_.ringbuffer_inf_upbound3i_(0))
    {
        xid_global -= md_.ringbuffer_inf_size3i_(0);
    }
    int yid_global = yid_in_buffer + md_.ringbuffer_inf_origin3i_(1);
    if(yid_global > md_.ringbuffer_inf_upbound3i_(1))
    {
        yid_global -= md_.ringbuffer_inf_size3i_(1);
    }
    int zid_global = zid_in_buffer + md_.ringbuffer_inf_origin3i_(2);
    if(zid_global > md_.ringbuffer_inf_upbound3i_(2))
    {
        zid_global -= md_.ringbuffer_inf_size3i_(2);
    }

    return Vector3i(xid_global,yid_global,zid_global);
}


inline void DspMap::changeInfBuf(const bool dir, const int inf_buf_idx, Eigen::Vector3i global_idx)
{
    int inf_grid = mp_.inf_grid_;
    // if(dir)
    // {
    //     md_.occupancy_buffer_inflate_[inf_buf_idx] += GRID_MAP_OBS_FLAG;
    // }
    // else
    // {
    //     md_.occupancy_buffer_inflate_[inf_buf_idx] -= GRID_MAP_OBS_FLAG;
    // }

    for (int x_inf = -inf_grid; x_inf <= inf_grid; ++x_inf)
    {
        for (int y_inf = -inf_grid; y_inf <= inf_grid; ++y_inf)
        {
            for (int z_inf = -inf_grid; z_inf <= inf_grid; ++z_inf)
            {
                Eigen::Vector3i id_inf(global_idx + Eigen::Vector3i(x_inf,y_inf,z_inf));

                int id_inf_buf = globalIdx2InfBufIdx(id_inf);
                if(dir)
                {   
                    ++md_.occupancy_buffer_inflate_[id_inf_buf];
                }
                else
                {
                    if(md_.occupancy_buffer_inflate_[id_inf_buf] > 0)
                    {
                        --md_.occupancy_buffer_inflate_[id_inf_buf];
                    }
                    if(md_.occupancy_buffer_inflate_[id_inf_buf] > 65000)
                    {
                        ROS_ERROR("A negtive value of nearby obstacle number! reset the map.");
                    }
                }

            }

        }

    }

}

// inline void DspMap::inflatePoint(const Eigen::Vector3i &pt, int inf_step, vector<Eigen::Vector3i> &pts)
// {
//     // int num = 0;
//     for(int x = -inf_step; x <= inf_step; ++x)
//     {
//         for(int y = -inf_step; y <= inf_step; ++y)
//         {
//             for(int z = -inf_step; z <= inf_step; ++z)
//             {
//                 Eigen::Vector3i inf_pt{pt(0) + x, pt(1) + y, pt(2) + z};
//                 pts.emplace_back(inf_pt);
//             }
//         }
//     }
// }


inline bool DspMap::isInBuf(const Vector3d &pos)
{
    if(pos(0) < md_.ringbuffer_lowbound3d_(0) || pos(1) < md_.ringbuffer_lowbound3d_(1) || pos(2) < md_.ringbuffer_lowbound3d_(2))
    {
        return false;
    } 
    if(pos(0) > md_.ringbuffer_upbound3d_(0) || pos(1) > md_.ringbuffer_upbound3d_(1) || pos(2) > md_.ringbuffer_upbound3d_(2))
    {
        return false;
    }
    return true;
}

inline bool DspMap::isInBuf(const float x, const float y, const float z)
{
    if(x < md_.ringbuffer_lowbound3d_(0) || y < md_.ringbuffer_lowbound3d_(1) || z < md_.ringbuffer_lowbound3d_(2))
    {
        return false;
    }
    if(x > md_.ringbuffer_upbound3d_(0) || y > md_.ringbuffer_upbound3d_(1) || z > md_.ringbuffer_upbound3d_(2))
    {
        return false;
    }
    return true;
}


inline bool DspMap::isInBuf(const Vector3i &idx)
{
    if(idx(0) < md_.ringbuffer_lowbound3i_(0) || idx(1) < md_.ringbuffer_lowbound3i_(1) || idx(2) < md_.ringbuffer_lowbound3i_(2))
    {
        return false;
    }
    if(idx(0) > md_.ringbuffer_upbound3i_(0) || idx(1) > md_.ringbuffer_upbound3i_(1) || idx(2) > md_.ringbuffer_upbound3i_(2))
    {
        return false;
    }
    return true;
}


inline bool DspMap::isInInfBuf(const Vector3d &pos)
{
    if(pos(0) < md_.ringbuffer_inf_lowbound3d_(0) || pos(1) < md_.ringbuffer_inf_lowbound3d_(1) || pos(2) < md_.ringbuffer_inf_lowbound3d_(2))
    {
        return false;
    }
    if(pos(0) > md_.ringbuffer_inf_upbound3d_(0) || pos(1) > md_.ringbuffer_inf_upbound3d_(2) || pos(2) > md_.ringbuffer_inf_upbound3d_(2))
    {
        return false;
    }
    return true;
}

inline bool DspMap::isInInfBuf(const Vector3i &idx)
{
    if(idx(0) < md_.ringbuffer_inf_lowbound3i_(0) || idx(1) < md_.ringbuffer_inf_lowbound3i_(1) || idx(2) < md_.ringbuffer_inf_lowbound3i_(2))
    {
        return false;
    }
    if(idx(0) > md_.ringbuffer_inf_upbound3i_(0) || idx(1) > md_.ringbuffer_inf_upbound3i_(1) || idx(2) > md_.ringbuffer_inf_upbound3i_(2))
    {
        return false;
    }
    return true;
}






inline double DspMap::getResolution()
{
    return mp_.voxel_resolution_;
}




#endif
