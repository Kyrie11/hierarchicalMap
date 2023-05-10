//输入系列点云生成三维b样条曲面
#include<ros/ros.h>
#include<pcl/point_cloud.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "surface_gen");
    ROS_INFO("Calculate 3d surface via Swarm Algorithm.....");

    ros::NodeHandle nh;
    
    ros::Subscriber pointCloudSub = nh.subscribe("/camera/pointcloud", 10, &this::segmentation);
}

/*
* segement the point cloud to different parts
* the first step is to segment several point cloud segmentations
*/
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr[] surfaceGen::segmentation(sensor_msgs::PointCloud2ConstPtr& inCloud)
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

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud;

    filterCloud = subSample(cloud, this.r);

    
} 

/*
* 对点云进行降采样，固定分辨率内只有一个点云，并且用一个体素来包裹
*/
pcl::VoxelGrid surfaceGen::subSample(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, double r)
{
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxels;
    pcl::PCLPointCloud<pcl::PointXYZRGBA>::Ptr cloudFiltered;
    voxels.setInputCloud(cloud);
    voxels.setLeafSize(0.1*r, 0.1*r, 0.1*r);
    voxels.filter(*cloudFiltered);
    
    //用来显示点云滤波以后结果是否正确
    // pcl::visualization::PCLVisualizer viewer("voxel_grid_filter");
    // int v1(1);
    // int v2(2);
    // viewer.createViewPort(0, 0, 0.5, 1, v1);
    // viewer.createViewPort(0.5, 0, 1, 1, v2);
    // viewer.addPointCloud(cloud, "raw_cloud", v1);
    // viewer.addPointCloud(cloudFiltered, "cloud_filtered", v2);

    return cloudFiltered;
}


map<int, std::vector<int>> surfaceGen::calcNeighorCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, double radius)
{
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud);
    
    map<int, std::vector<int>> neighborClouds;  
    int size = cloud->size();
    for(int i=0; i<size; i++)
    {
        auto& point = cloud->points.at(i);
        std::vector<int> pointIdxRadiSearch(this.q);
        std::vector<float> pointRadiSqDis(this.q);
        if(tree.radiusSearch(point, radius, pointIdxRadiSearch, pointRadiSqDis)>0)
        {
            if(tree.radiusSearch.size() > this.q)
            {
                vector<int>::iterator iter = pointIdxRadiSearch.begin();
                *iter+=this.q-1;
                pointIdxRadiSearch.erase(iter, pointIdxRadiSearch.end());//delete the superfluous point clouds 

            }
            neighborClouds.insert(i, pointIdxRadiSearch);//for existing neighbor clouds, save the index
        }
    }

    return neighborClouds;
}


/*
* multi scale feature detection
*/
void surfaceGen::featureDet(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rawCloud, map<int, std::vector<int>> neighborClouds)
{
     map<int, std::vector<int>> neighborClouds =  calcNeighborCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    //计算点pi附近q个最近点云
    Eigen::Vector3d surfNormal = surfNormal(neighborCloud, d);
    for(const auto &[iter, clouds] : m){
        auto p = rawCloud.at(iter);
        for(auto point:clouds){
            
        }
    }
    std::vector<Eigen::vector3d> planes = new std::vector<Eigen::Vector3d>(this.m); // planes evenly distributed around the normal line
    
}


/*
* 通过邻近点云基于PCA计算平面的法向量
*/
pcl::PointCloud<pcl::Normal>::Ptr surfaceGen::surfNormal(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{

    int size = cloud->size();
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    double normal_x, normal_y, normal_z = 0;

    double centerX = 0, centerY = 0, centerZ = 0;
    for(int i=0; i<size; i++)
    {
        centerX += cloud->points[i].x;
        centerY += cloud->points[i].y;
        centerZ += cloud->points[i].z;
    }

    centerX /= size;
    centerY /= size;
    centerZ /= size;

    double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
    for(int i=0;i<size;i++)
    {
        xx += (cloud->points[i].x - centerX) * (cloud->points[i].x - centerX);
        xy += (cloud->points[i].x - centerX) * (cloud->points[i].y - centerY);
        xz += (cloud->points[i].x - centerX) * (cloud->points[i].z - centerZ);
        yy += (cloud->points[i].y - centerY) * (cloud->points[i].y - centerY);
        yz += (cloud->points[i].y - centerY) * (cloud->points[i].z - centerZ);
        zz += (cloud->points[i].z - centerZ) * (cloud->points[i].z - centerZ);
    }

    Eigen::Matrix3f covMat(3, 3);
    covMat(0, 0) = xx / size;
    covMat(0, 1) = covMat(1, 0) = xy / size;
    covMat(0, 2) = covMat(2, 0) = xz / size;
    covMat(1, 1) = yy / size;
    covMat(1, 2) = covMat(2, 1) = yz / size;
    covMat(2, 2) = zz / size;

    //求特征值与特征向量
    Eigen::EigenSolver<Eigen::Matrix3f> es(covMat);
    Eigen::Matrix3f val = es.pseudoEigenValueMatrix();
    Eigen::Matrix3f vec = es.pseudoEigenvectors();

    //找到最小特征值
    double t1 = val(0, 0);
    int ii = 0;
    if(t1 > val(1, 1))
    {
        ii = 1;
        t1 = val(1, 1);
    }
    if(t1 > val(2, 2))
    {
        ii = 2;
        t2 = val(2, 2);
    }

    //最小特征值对应的特征向量
    Eigen::Vector3f v(vec(0, ii), vec(1, ii), vec(2, ii));
    //特征向量单位化
    v /= v.norm();
    for(int i=0; i<size; i++)
    {
        normals->points[i].normal_x = v(0);
        normals->points[i].normal_y = v(1);
        normals->points[i].normal_z = v(2);
        normals->points[i].curvature = t1 / (val(0, 0) + val(1, 1) + val(2, 2));
    }

    return new pcl::PointCloud<pcl::Normal>(v(0), v(1), v(2));
}


void surfaceGen::preProcess(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    
}

void surfaceGen::initialization()
{

}


void surfaceGen::mergeNURBS()
{

}
