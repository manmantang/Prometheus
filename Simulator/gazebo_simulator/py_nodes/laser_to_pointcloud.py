#!/usr/bin/env python

from numpy.core.numeric import Inf
import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan 
from laser_geometry import LaserProjection

class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/prometheus/sensors/pcl2", pc2, queue_size=1)
        self.laserSub = rospy.Subscriber("/prometheus/sensors/2Dlidar_scan", LaserScan, self.laserCallback) 

    def laserCallback(self,data):
        #cc.header = data.header
        #cc.angle_max = data.angle_max
        #cc.angle_min = data.angle_min
        #cc.angle_increment = data.angle_increment
        #cc.time_increment = data.time_increment
        #cc.scan_time = data.scan_time
        #cc.range_max = data.range_max
        #cc.range_min = data.range_min
        #cc.ranges = data.ranges
        #cc.intensities = data.intensities
        a = len(data.ranges)
        data.ranges = list(data.ranges)
        for i in range(0,a):
             if data.ranges[i] != float("inf"):
                 if data.ranges[i] < 0.15:
                     data.ranges[i] = float("inf")
        cloud_out = self.laserProj.projectLaser(data)
        self.pcPub.publish(cloud_out)

if __name__ == '__main__':
    rospy.init_node("laser2PointCloud")
    l2pc = Laser2PC()
    rospy.spin()
