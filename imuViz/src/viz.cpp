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
#include <iostream>
#include <chrono>
#include <thread>
#include <iostream>


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

int main(int argc, char** argv) {
    ros::init (argc, argv, "imuviz");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe ("/os1_cloud_node/imu", 1, imu_cb);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (nh.ok()) {
        ROS_INFO("Imu accelerometer : x : %.3f y : %.3f z : %.3f", imuRaw.linear_acceleration.x, imuRaw.linear_acceleration.y, imuRaw.linear_acceleration.z);
        ros::spinOnce();
        viewer->spinOnce (1);
    }
}
