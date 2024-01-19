#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>

using Eigen::Quaternionf, Eigen::AngleAxisf, Eigen::Translation3f, Eigen::Affine3f, Eigen::Vector3f, Eigen::Vector4f;
ros::Publisher cloud_pub;
ros::Publisher odometry_pub;
Eigen::Matrix3f cc = Eigen::Matrix3f::Zero();
nav_msgs::Odometry odom_ ;
// Quaternionf lidar2body = AngleAxisf(-M_PI_2,Vector3f::UnitZ()) *
//                             AngleAxisf(0, Vector3f::UnitY()) *
//                             AngleAxisf(-M_PI_2, Vector3f::UnitX());


void callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    // std::cout << "11" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *cloud_in);


    Eigen::Translation3f translate(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    Eigen::Quaternionf rotate(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
    Eigen::Affine3f body2world;
    body2world.linear() = rotate.toRotationMatrix();
    body2world.translation() = translate.vector();
    // Eigen::Quaternionf lidar2body = Eigen::Quaternionf(cc);
    Eigen::Affine3f lidar2body;
    lidar2body.linear() = cc;
    lidar2body.translation() = Eigen::Vector3f(0,0,0);
    Eigen::Affine3f lidar2world =  body2world * lidar2body;

    pcl::transformPointCloud(*cloud_in, *cloud_out, lidar2world);
    // cloud_out = cloud_in;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_out, cloud_msg);
    cloud_msg.header.frame_id = "world";

    cloud_pub.publish(cloud_msg);
    odom_= *odom;
    odom_.header.frame_id = "world";
    odometry_pub.publish(odom_);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sync_node");

    ros::NodeHandle nh;
    cc(0,2) = 1.0;
    cc(1,0) = -1.0;
    cc(2,1) = -1.0;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> Sync;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/iris_0/mavros/local_position/odom", 1));
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/iris_0/realsense/depth_camera/depth/points", 1));
    
    Sync sync(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(1), *odom_sub, *cloud_sub));
    sync->registerCallback(boost::bind(&callback, _1, _2));

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 1);
    odometry_pub = nh.advertise<nav_msgs::Odometry>("/transformed_odom", 1);
    ros::spin();


}