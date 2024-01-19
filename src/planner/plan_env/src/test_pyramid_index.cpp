
#include <iostream>
#include <Eigen/Eigen>
#include <munkres.h>
#include <time.h>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <random>
#include <cmath>
using namespace std;

double one_degree_rad_ = M_PI / 180;
double one_degree_rad_inv_ = 180 / M_PI;
int angle_resolution_ = 1;

int half_fov_vertical_ = 50;
int fov_vertical_lowbound_ = -40;
int fov_vertical_upbound_ = 40;
int fov_horizontal_lowbound_ = -90;
int fov_horizontal_upbound_ = 90;
double fov_vertical_lowbound_rad_ = fov_vertical_lowbound_ * one_degree_rad_;
double fov_vertical_upbound_rad_ = fov_vertical_upbound_ * one_degree_rad_;
double fov_horizontal_lowbound_rad_ = fov_horizontal_lowbound_ * one_degree_rad_;
double fov_horizontal_upbound_rad_ = fov_horizontal_upbound_ * one_degree_rad_;
int observation_pyramid_num_vertical_ = (fov_vertical_upbound_ - fov_vertical_lowbound_) / angle_resolution_;
int observation_pyramid_num_horizontal_ = (fov_horizontal_upbound_ - fov_horizontal_lowbound_) / angle_resolution_;


bool inPyramidsAreaInSensorFrame(float x, float y, float z)
{

    float vertical_rad = atan2(z,sqrtf(powf(x,2) + powf(y,2)));

    if(vertical_rad <= fov_vertical_lowbound_rad_ || vertical_rad > fov_vertical_upbound_rad_)
    {
        return false;
    }
    float horizontal_rad = atan2(y,x);
    if(horizontal_rad <= fov_horizontal_lowbound_rad_ || horizontal_rad > fov_horizontal_upbound_rad_)
    {
        return false;
    }
    return true;


}

int findPointPyramidHorizontalIndexInSensorFrame(float x, float y, float z)
{

    float horizontal_rad = atan2(y,x);
    int horizontal_index = floor(horizontal_rad / one_degree_rad_ - fov_horizontal_lowbound_);
    if(horizontal_index >= 0 && horizontal_index < observation_pyramid_num_horizontal_)
    {
        return horizontal_index;
    }
    ROS_WARN("horizontal index : %d",horizontal_index);
    ROS_WARN("!!!!!! Please use Function ifInPyramidsArea() to filter the points first before using findPointPyramidHorizontalIndex()");
    return -1;
}

int findPointPyramidVerticalIndexInSensorFrame(float x,float y,float z)
{
    float vertical_rad = atan2(z,sqrtf(powf(x,2) + powf(y,2)));
    // int vertical_index = floor((vertical_rad - mp_.fov_vertical_lowbound_rad_) / mp_.one_degree_rad_);
    int vertical_index = floor(vertical_rad / one_degree_rad_ - fov_vertical_lowbound_);
    if(vertical_index >= 0 && vertical_index < observation_pyramid_num_vertical_ )
    {
        return vertical_index;
    }
    ROS_WARN("vertical index : %d",vertical_index);
    ROS_WARN("!!!!!! Please use Function ifInPyramidsAreaInSensorFrame() to filter the points first before using findPyramidVerticalIndexInSensorFrame()");
    return -1;
}


int main()
{

    /* test vertical idnex*/
    // int radius = 5;
    // Eigen::Vector2d center(0,0);
    // for(double i=-M_PI;i<M_PI;i+=M_PI/1000)
    // {
    //     double x = center[0] + radius * cos(i);
    //     double y = center[1] + radius * sin(i);
    //     // int horizontal_index = std::floor(fmod(atan2(y,x) + 2 * M_PI, 2*M_PI) / one_degree_rad_);
    //     if(!inPyramidsAreaInSensorFrame(x,y,0))
    //     {
    //         continue;
    //     }
    //     int vertical_index = findPointPyramidHorizontalIndexInSensorFrame(x,y,0);
    //     if(vertical_index < 0 || vertical_index >= 360)
    //     {
    //         std::cout << "horizontal index: " << vertical_index << std::endl;
    //         std::cout << "i : " << (i) / M_PI * 180 << std::endl;
    //         // std::cout << "x : " << x << " " << "y: " << y << endl;
    //     }
    //     // std::cout << i << " " << horizontal_index << std::endl;
    // }

        /* test horizontal index */
    // int radius = 5;
    // Eigen::Vector2d center(0,0); 
    // for(double i=-M_PI;i<M_PI;i+=M_PI/10000)
    // {
    //     double x = center[0] + radius * cos(i);
    //     double y = center[1] + radius * sin(i);
    //     // int horizontal_index = std::floor(fmod(atan2(y,x) + 2 * M_PI, 2*M_PI) / one_degree_rad_);
    //     if(!inPyramidsAreaInSensorFrame(x,y,0))
    //     {
    //         continue;
    //     }
    //     int horizontal_index = findPointPyramidHorizontalIndexInSensorFrame(x,y,0);
    //     if(horizontal_index < 0 || horizontal_index >= 360)
    //     {
    //         std::cout << "horizontal index: " << horizontal_index << std::endl;
    //         std::cout << "i : " << (i) / M_PI * 180 << std::endl;
    //         // std::cout << "x : " << x << " " << "y: " << y << endl;
    //     }
    //     // std::cout << i << " " << horizontal_index << std::endl;
    // }
    // int h_idx = findPointPyramidHorizontalIndexInSensorFrame(1,0,0);
    // cout << h_idx << endl;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> xdis(-4,4);
    std::uniform_real_distribution<> ydis(-4,4);
    std::uniform_real_distribution<> zdis(-4,4);

    vector<Eigen::Vector3d> cev;
    for(int i=0;i<1000000;i++)
    {
        double x = xdis(gen);
        double y = ydis(gen);
        double z = zdis(gen);
        // cev.emplace_back(x(gen),y(gen),z(gen));
        // std::cout << "x : " << x << " " << "y: " << y << "z: " << z << endl;

        if(inPyramidsAreaInSensorFrame(x,y,z))
        {
            int horizontal_index = findPointPyramidHorizontalIndexInSensorFrame(x,y,z);
            if(horizontal_index < 0 || horizontal_index >= observation_pyramid_num_horizontal_)
            {
                std::cout << "horizontal index: " << horizontal_index << std::endl;
                std::cout << "x : " << x << " " << "y: " << y << "z: " << z << endl;
            }
            int vertical_index = findPointPyramidVerticalIndexInSensorFrame(x,y,z);
            if(vertical_index < 0 || vertical_index >= observation_pyramid_num_vertical_)
            {
                std::cout << "horizontal index: " << vertical_index << std::endl;
            }
        }
    }



    
    return 0;
}