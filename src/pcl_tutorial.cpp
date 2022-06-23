#include "ros/init.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

ros::Publisher pub;
std::string camera;
std::string output;
float z_min_filter_limit, z_max_filter_limit;
float d_threshold;
std::string filterFieldName;
float pointColorThreshold;
float regionColorThreshold;
float minClusterSize;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *raw_cloud);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (raw_cloud);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (*transformed_cloud);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (raw_cloud);
    pass.setFilterFieldName (filterFieldName);
    pass.setFilterLimits (z_min_filter_limit, z_max_filter_limit);
    pass.filter (*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (raw_cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (d_threshold);
    reg.setPointColorThreshold (pointColorThreshold);
    reg.setRegionColorThreshold (regionColorThreshold);
    reg.setMinClusterSize (minClusterSize);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    transformed_cloud = reg.getColoredCloud(); 

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*transformed_cloud, output_msg);
    
    output_msg.header.frame_id = cloud_msg->header.frame_id;
    output_msg.header.stamp = cloud_msg->header.stamp;

    pub.publish(output_msg);
}

int main(int argc, char **argv) {

    //initalize ROS
    ros::init(argc, argv, "pcl_tutorial");
    ros::NodeHandle n("~");
    
    n.param<std::string>("camera_param", camera, "/camera/depth/points");
    n.param<std::string>("output_param", output, "/output");

    //Create a ROS Subscriber for input point cloud
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, cloud_cb);
    
    //Create a ROS Publisher for the output point cloud
    pub = n.advertise<sensor_msgs::PointCloud2>("output", 1);
    
    ros::Rate rate(10);
    while (ros::ok()) {

    n.param<float>("z_min_filter_limit", z_min_filter_limit, 0.0);
    n.param<float>("z_max_filter_limit", z_max_filter_limit, 1.0);
    n.param<float>("d_threshold", d_threshold, 10.0);
    n.param<std::string>("filterFieldName", filterFieldName, "z");
    n.param<float>("pointColorThreshold", pointColorThreshold, 6.0);
    n.param<float>("regionColorThreshold", regionColorThreshold, 5.0);
    n.param<float>("minClusterSize", minClusterSize, 600.0);

        ros::spinOnce();
        rate.sleep();
    }
}