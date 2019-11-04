#include "ros/ros.h"

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/Imu.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "pcl_conversions/pcl_conversions.h"

#include <iostream>
#include <sstream>
#include <cstdio>
#include <sstream>	
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include <iostream>

#include <eigen3/Eigen/Eigen>

// pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMsg (new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::Imu imuRaw;
pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

int cnt = 0;
void  imu_cb (const sensor_msgs::Imu::ConstPtr& msg)
{
    imuRaw = *msg;   
}

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    

}

pcl::PointXYZ normalization(pcl::PointXYZ p) {
    pcl::PointXYZ ret;
    float len = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    ret.x = p.x / len;
    ret.y = p.y / len;
    ret.z = p.z / len;
    return ret;
}

Eigen::Matrix4f rotationMatFromX(float angX) {
    Eigen::Matrix4f ret;
    ret << 1,         0,          0, 0,
           0, cos(angX), -sin(angX), 0,
           0, sin(angX),  cos(angX), 0,
           0,         0,          0, 1;

    return ret;  
}

Eigen::Matrix4f rotationMatFromY(float angY) {
    Eigen::Matrix4f ret;
    ret <<  cos(angY), 0, sin(angY), 0,
                    0, 1,         0, 0,
           -sin(angY), 0, cos(angY), 0,
                    0, 0,         0, 1;
    return ret;  
}

int main(int argc, char** argv) {
    ros::init (argc, argv, "imuviz");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe ("/os1_cloud_node/imu", 1, imu_cb);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    pcl::PointXYZ p1, p2, p3;
    p1.x = 0; p1.y = 0; p1.z = 0;
    int cnt = 0;
    while (nh.ok()) {
        p2.x = imuRaw.linear_acceleration.x;
        p2.y = imuRaw.linear_acceleration.y;
        p2.z = imuRaw.linear_acceleration.z;
        p2 = normalization(p2);

        float angX = atan2(p2.y, p2.z);
        float angY = -atan2(p2.x, sqrt(p2.y * p2.y + p2.z * p2.z));

        Eigen::Matrix4f rotMatrix;
        rotMatrix = rotationMatFromY(angY) * rotationMatFromX(angX);
        Eigen::Vector4f tmp;
        tmp << p2.x, p2.y, p2.z, 1;
        std::cerr << rotMatrix << std::endl; 
        tmp = rotMatrix * tmp;
        tmp = tmp / tmp(3);
        p3.x = tmp(0);
        p3.y = tmp(1);
        p3.z = tmp(2);
        p3 = normalization(p3);
        std::cerr << "xxxxxxxx : " << p3 << std::endl;
        std::string name = "line" + std::to_string(cnt);

        cnt++;
        std::cerr << "Accel : " << p2.x << " " << p2.y <<  " " << p2.z << std::endl;

        std::cerr << "-----------------------------------" << std::endl;
        viewer->addLine(p1, p2, 0.5, 0.5, 0.5, name);
        viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, name);
        ros::spinOnce();
        viewer->spinOnce (1);
    }
}
