#include "ros/ros.h"
#include "prometheus_msgs/GetOffsetpose.h"
#include "sensor_msgs/NavSatFix.h"

std::vector<sensor_msgs::NavSatFix> uavs_global_position(5);

bool getoffsetpose(prometheus_msgs::GetOffsetpose::Request &req, prometheus_msgs::GetOffsetpose::Response &res)
{
    if(req.uav_num == 1)
    {
        return false;
    }
    int x = (uavs_global_position[req.uav_num-1].longitude - uavs_global_position[0].longitude) * 1000000;
    int y = (uavs_global_position[req.uav_num-1].latitude - uavs_global_position[0].latitude) * 1000000;
    res.offset_pose_x = x/10.0;
    res.offset_pose_y = y/10.0;

    std::cout << res.offset_pose_x << std::endl;
    std::cout << res.offset_pose_y << std::endl;

}

void uav1_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[0] = *msgs;
}

void uav2_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[1] = *msgs;
}

void uav3_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[2] = *msgs;
}

void uav4_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[3] = *msgs;
}

void uav5_global_position(const sensor_msgs::NavSatFix::ConstPtr & msgs)
{
    uavs_global_position[4] = *msgs;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_offset_pose");
    ros::NodeHandle n;
    ros::ServiceServer srv = n.advertiseService("/get_offset_pose", getoffsetpose);
    ros::Subscriber uav1_global_sub = n.subscribe("/uav1/mavros/global_position/global", 10, uav1_global_position);
    ros::Subscriber uav2_global_sub = n.subscribe("/uav2/mavros/global_position/global", 10, uav2_global_position);
    ros::Subscriber uav3_global_sub = n.subscribe("/uav3/mavros/global_position/global", 10, uav3_global_position);
    ros::Subscriber uav4_global_sub = n.subscribe("/uav4/mavros/global_position/global", 10, uav4_global_position);
    ros::Subscriber uav5_global_sub = n.subscribe("/uav5/mavros/global_position/global", 10, uav5_global_position);

    ros::spin();
}
