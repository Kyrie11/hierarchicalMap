//输入系列点云生成三维b样条曲面
#include<ros/ros.h>
#include<pcl/point_cloud.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "surface_gen");
    ROS_INFO("Calculate 3d surface via Swarm Algorithm.....");

    ros::NodeHandle nh;
    
    ros::Subscriber pointCloudSub = nh.subscribe("/camera/pointcloud", 10, &)
}

/*
*点云分割
*/
void surfaceGen::segmentation(sensor_msgs::PointCloud2ConstPtr& inCloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    // pcl::fromROSMsg(*inCloud, *cloud); 当数据量过大时，这种方式比较耗时
    // 通过直接复制数据地址的方式来进行数据类型转换
    for(int i=0; i<inCloud->width*inCloud->height; i++)
    {
        pcl::PointXYZRGBA p;
        std::memcpy(&p.x, &inCloud->data[32*i], 4);
        std::memcpy(&p.y, &inCloud->data[32*i+4], 4);
        std::memcpy(&p.z, &inCloud->data[32*i+8], 4);
        std::memcpy(&p.rgba, &inCloud->data[32*i+16], 4);
        cloud->points.push_back(p);
    }
}

void surfaceGen::preProcess()
{

}

void surfaceGen::initialization()
{

}

