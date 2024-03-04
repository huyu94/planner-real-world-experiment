#ifndef _MAP_VISUALIZER_H_
#define _MAP_VISUALIZER_H_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using std::vector;

class Color : public std_msgs::ColorRGBA {
 public:
  Color() : std_msgs::ColorRGBA() {}
  Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
  Color(double red, double green, double blue, double alpha) : Color() {
    r = red;
    g = green;
    b = blue;
    a = alpha;
  }

  static const Color White() { return Color(1.0, 1.0, 1.0); }
  static const Color Black() { return Color(0.0, 0.0, 0.0); }
  static const Color Gray() { return Color(0.5, 0.5, 0.5, 0.5); }
  static const Color Red() { return Color(1.0, 0.0, 0.0); }
  static const Color Green() { return Color(0.0, 0.7, 0.0, 0.5); }
  static const Color Blue() { return Color(0.0, 0.0, 1.0); }
  static const Color SteelBlue() { return Color(0.4, 0.7, 1.0); }
  static const Color Yellow() { return Color(1.0, 1.0, 0.0, 0.5); }
  static const Color Orange() { return Color(1.0, 0.5, 0.0, 0.8); }
  static const Color Purple() { return Color(0.5, 0.0, 1.0,0.5); }
  static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color Teal() { return Color(0.0, 1.0, 1.0); }
  static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};



struct VisualCluster
{
  Vector3d centroid;
  Vector3d length;
  Vector3d min_bound,max_bound;
  Vector3d velocity;

  VisualCluster(Vector3d c,Vector3d l,Vector3d minb,Vector3d maxb) : 
                    centroid(c), length(l), velocity(Vector3d::Zero()),
                    min_bound(minb), max_bound(maxb) {}

  VisualCluster(Vector3d c,Vector3d l,Vector3d minb,Vector3d maxb,Vector3d vel) : 
                centroid(c), length(l),velocity(vel),
                min_bound(minb), max_bound(maxb) {} 
};



struct VisualKalmanTracker
{
  Vector3d pos;
  Vector3d vel;
  Vector3d len;
  int id;
  VisualKalmanTracker(Vector3d p,Vector3d v,Vector3d l,int i):pos(p),vel(v),len(l),id(i){}

};

struct VisualizeSlideBox
{
  Vector3d center;
  Vector3d length;
  Matrix3d rotation;
  int id;
  VisualizeSlideBox(Vector3d c,Vector3d l,Matrix3d r,int i) : center(c),length(l),rotation(r), id(i){}
};


visualization_msgs::Marker generateEllipse(const Vector3d &pos, const Vector3d &len, int id);
visualization_msgs::Marker generateArrows(const Vector3d &pos, const Vector3d &vel, int id);
visualization_msgs::Marker generateBBox(const Vector3d &min_point, const Vector3d &max_point, int id);
visualization_msgs::Marker generateCube(const Vector3d &center, const Vector3d &length, const Matrix3d &rotation, int id);

class MapVisualizer
{
public:
    MapVisualizer();
    MapVisualizer(const ros::NodeHandle& node);

    void visualizeReceiveCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void visualizeClusterResult(std::vector<VisualCluster> &visual_clusters);
    void visualizeSegmentationResult(std::vector<VisualCluster> &visual_clusters);
    // void visualizeKMResult(std::vector<VisualKMResult> &kmresult); // 滑匹配的线
    void visualizeStaticPoint(vector<Eigen::Vector3d> &static_points);
    void visualizeKalmanTracker(std::vector<VisualKalmanTracker> &visual_trackers);
    void visualizeSlideBox(std::vector<VisualizeSlideBox> &visual_slideboxes);
    void visualizeMovingObjectBox();
    void visualizeMovingObjectTraj();


    typedef std::shared_ptr<MapVisualizer> Ptr;

private:
    ros::NodeHandle node_;
    ros::Publisher cluster_result_pub_;
    ros::Publisher segmentation_result_pub_;
    ros::Publisher km_result_pub_;
    ros::Publisher kalman_tracker_pub_;
    ros::Publisher static_point_pub_;
    ros::Publisher moving_object_box_pub_;
    ros::Publisher moving_object_traj_pub_;
    ros::Publisher slide_box_pub_;
    ros::Publisher receive_cloud_pub_;

};



#endif