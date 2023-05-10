#include <map>
#include <iostream>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>


using namespace hierarchicalMap;

class surfaceGen{
    private:
        double cutRadius; //the abstractiojn level of segmentation
        double r;//radius
        double q;//the number of neighboring points
        double m;//the number of planes

        void segmentation(sensor_msgs::PointCloud2 cloud);
        void preProcess();
        void initialization();
}