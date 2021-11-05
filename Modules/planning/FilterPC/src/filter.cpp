#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/conversions.h>
#include<pcl_ros/transforms.h>
//#include<pcl/filters/voxel_grid.h>
#include<sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

using namespace std;

class Filter_Cloud
{
    public:
    Filter_Cloud(){
        sub_point_cloud_=nh.subscribe("/prometheus/sensors/pcl2",10,&Filter_Cloud::point_cb,this);
        pub_filtered_points_=nh.advertise<sensor_msgs::PointCloud2>("/filtered_points",10);
    }

    void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr){
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*in_cloud_ptr,*current_pc_ptr);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;
    pcFilter.setInputCloud(current_pc_ptr);
    pcFilter.setRadiusSearch(0.3);
    pcFilter.setMinNeighborsInRadius(12);
    pcFilter.filter(*filtered_pc_ptr);

    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg(*filtered_pc_ptr,pub_pc);

    pub_pc.header=in_cloud_ptr->header;
    pub_filtered_points_.publish(pub_pc);
  }

  private:
  ros::NodeHandle nh;
  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_filtered_points_;
};

int main(int argc,char **argv){
    ros::init(argc ,argv ,"filter_node");
    Filter_Cloud filter_cloud;
    ros::spin();

    return 0;
}
