//输入系列点云生成三维b样条曲面
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>

#include<cmath>
#include<vector>
#include<set>

#include<Eigen/Eigenvalues>

#define r 1
#define Q 500 //设定的临近点云数量

#define DEGREE 3 //曲线阶数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "surface_gen");
    ROS_INFO("Calculate 3d surface via Swarm Algorithm.....");

    ros::NodeHandle nh;
    
    ros::Subscriber pointCloudSub = nh.subscribe("/camera/pointcloud", 10, &this::segmentation);
}

struct FeatureResult
{
    double curvature;
    Eigen::Vector2d center;
    double fitErrorMean;
    double fitErrorStddev;
}

/*
* segement the point cloud to different parts
* the first step is to segment several point cloud segmentations
*/
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr[] surfaceGen::segmentation(sensor_msgs::PointCloud2ConstPtr& inCloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*inCLoud, *cloud);


    // pcl::fromROSMsg(*inCloud, *cloud); 当数据量过大时，这种方式比较耗时
    // 通过直接复制数据地址的方式来进行数据类型转换
    // size_t numPoints = inCloud->width * inCloud->height;
    // std::memcpy(cloud->points.data(), inCloud->data.data(), inCloud->data.size());
    // cloud->width = inCloud->width;
    // cloud->height = inCloud->height;


    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.1 * r, 0.1 * r, 0.1 * r);

    //点云降采样
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_grid.filter(*downsampledCloud);

    //随机生成一个点
    pcl::common::UniformGenerator<int> indexGenerator(0, downsampledCloud->size()-1);
    int randomIndex = indexGenerator.run(0);
    pcl::PointXYZRGB randomPoint = downsampledCloud->points[randomIndex];

    //计算附近点云组成的平面的法线
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighbor50 = calcNeighborCloud(downsampledCloud, r, 50, randomPoint);
    pcl::Normal normal = computePlaneNormal(neighbor50);


    std::vector<Eigen::Vector3f> planeNormals;
    int m = 40;

    float angleInterval = 2 * M_PI / m;

    Eigen::Vector3f orthogonalVec1;
    if(std::abs(normal.x()) < std::abs(normal.y()))
        orthogonalVec1 << 0, -normal.z(), normal.y();
    else
        orthogonalVec1 << -normal.z(), 0, normal.x();
    
    orthogonalVec1.normalize();

    Eigen::Vector3f orthogonalVec2 = normal.cross(orthogonalVec1);
    orthogonalVec2.normalize();

    Eigen::Matrix3f rotationMatrix;
    rotationMatrix << normal, orthogonalVec1, orthogonalVec2;

    for(int i=0; i<m; i++)
    {
        Eigen::Vector3f rotatedNormal = rotationMatrix * normal;
        rotatedNormal.normalize();

        planeNormals.push_back(rotatedNormal);

        rotationMatrix = Eigen::AngleAxisf(angleInterval, normal) * rotationMatrix;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighbor500 = calcNeighborCloud(downsampledCloud, r, 50, randomPoint);

    std::vector<Eigen::Vector2d> points;

    for(Eigen::Vector3f plane : planeNormals)
    {
        for(size t= 0; t<neighbor500->size(); t++)
        {
            Eigen::Vector3d point(neighbor500->points[t].x, neighbor500->points[t].y, neighbor500->points[t].z);
            double distanceToPlane = std::abs((point - neighbor500->points[t].getVector3fMap().dot(plane)));
            if(distanceToPlane < 0.1 * r)
            {
                Eigen::Vector2d projectedPoint;
                projectedPoint << (point - neighbor500->points[t].getVector3fMap().dot(plane).norm(), point.z());
                points.push_back(projectedPoint);            
            }
        }


    }


} 

/*
 * 二维空间circle fit
*/
Circle fitCircle(const std::vector<Eigen::Vector2d> &points)
{
    Circle circle;
    Eigen::MatrixXd A(points.size(), 3);
    
}

/*
 * 计算点云法线
*/
pcl::Normal computePlaneNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PointXYZRGB centroid;
    for(const pcl::PointXYZRGB& piont : cloud->points)
    {
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
    }

    centroid.x /= cloud->size();
    centroid.y /= cloud->size();
    centroid.z /= cloud->size();

    //Compute the covariance matrix
    Eigen::Matrix3f covariance_matrix;
    covariance_matrix.setZero();
    for(const pcl::PointXYZ& point : cloud->points)
    {
        Eigen::Vector3f diff;
        diff[0] = point.x - centroid.x;
        diff[1] = point.y - centroid.y;
        diff[2] = point.z - centroid.z;
        covariance_matrix += diff * diff.transpose();
    }
    covariance_matrix /= cloud->size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covariance_matrix);
    if(eigenSolver.info() != Eigen::Success)
    {
        std::cerr <<"Eigenvalue decomposition failed!" << std::endl;
        exit(EXIT_FAILURE);
    }

    Eigen:Vector3d normal = eigenSolver.eigenvectors().col(0);

    pcl::Normal planeNormal;
    planeNormal.normal_x = normal[0];
    planeNormal.normal_y = normal[1];
    planeNormal.normal_z = normal[2];
    planeNormal.curvature = 0.0;

    return planeNormal;

}

/*
 * 为每个点找寻临近点云
 * radius:半径
 * cloud:原始点云 ，参数类型是pcl::PointCloud
*/

map<int, std::vector<int>> surfaceGen::calcNeighorCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, double radius, int q, pcl::PointXYZRGB referencePoint)
{
    // pcl::PointXYZRGB referencePoint = cloud->points[index];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighborClouds(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<pcl::PointXYZRGB> tempPoints;
    for(const pcl::PointXYZRGB& point : cloud->points)
    {
        double distance = pcl::euclideanDistance(referencePoint, point);
        if(distance < radius)
        {
            tempPoints.push_back(point);
        }
    }

    if(tempPoints.size() <= q)
    {
        neighborCLouds->points = tempPoints;
        neighborClouds->width  = tempPoints.size();
        neighborClouds->height = 1;
        return neighborClouds;
    }

    int step = tempPoints.size() / q;
    int index = 0;
    for(int i=0; i<q; i++)
    {
        neighborClouds->push_back(tempPoints[index]);
        index += step;
    }

    neighborClouds->width = q;
    neighborClouds->height = 1;

    return neighborClouds;
}

/*
 * 为每个点找寻临近点云
 * radius:半径
 * cloud:原始点云 ，参数类型是std::vector
*/

map<int, std::vector<int>> surfaceGen::calcNeighorCloud(std::vector<pcl::PointXYZRGBA> cloudVector, double radius, int index)
{
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    for(int i=0; i<cloudVector.size(); i++)
    {   
        pcl::PointXYZRGBA point = cloudVector[i];
        cloud.push_back(point);
    }
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud);
    
    //map<int, std::vector<int>> neighborClouds;  
    std::vector<std::vector<int>> neighborClouds;
    int size = cloud->size();
    auto& point = cloud->points.at(index);
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
        neighborClouds.push_back(pointIdxRadiSearch);//for existing neighbor clouds, save the index
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
    for(auto [index, cloud] : neighborClouds)
    {
        Eigen::Vector3d surfNormal = surfNormal(cloud);
        pcl::PointXYZRGBA point = rawCloud->points.at(index);
        double normalLength = sqrt(surfNormal[0]*surfNormal[0] + surfNormal[1]+surfNormal[1] + surfNormal[2]*surfNormal[2]);
        surfNormal[0] /= normalLength;
        surfNormal[1] /= normalLength;
        surfNormal[2] /= normalLength;
        
        double maxCurvature = 0;
        int index = 0;
        double totalFeature[numOfPlanes][3];
        std::vector<Eigen::vector3d> initPlane = new std::vector<Eigen::Vector3d>(-surfNormal[2], surfNormal[2], surfNormal[0]-surfNormal[1]); // planes evenly distributed around the normal line
        int unit = 360/this.numOfPlanes;
        for(int i=0;i<numOfPlanes;i++){
            int theta = unit * i;
            std::vector<Eigen::vector3d> plane;
            double rad = theta * M_PI / 180.0;
            double cosA = cos(rad);
            double sinA = sin(rad);
            
            double u = surfNormal[0];
            double v = surfNormal[1];
            double w = surfNormal[2];
            double cosT = cosA;
            double sinT = sinA;
            double oneMinusCosT = 1.0 - cosT;
            double rotMat[3][3];
            rotMat[0][0] = u*u + (1-u*u)*cosT;
            rotMat[0][1] = u*v*(oneMinusCosT) - w*sinT;
            rotMat[0][2] = u*w*(oneMinusCosT) + v*sinT;
            rotMat[1][0] = u*v*(oneMinusCosT) + w*sinT;
            rotMat[1][1] = v*v + (1-v*v)*cosT;
            rotMat[1][2] = v*w*(oneMinusCosT) - u*sinT;
            rotMat[2][0] = u*w*(oneMinusCosT) - v*sinT;
            rotMat[2][1] = v*w*(oneMinusCosT) + u*sinT;
            rotMat[2][2] = w*w + (1-w*w)*cosT;

            vector<double> result(3);
            result[0] = rotMat[0][0]*v[0] + rotMat[0][1]*v[1] + rotMat[0][2]*v[2];
            result[1] = rotMat[1][0]*v[0] + rotMat[1][1]*v[1] + rotMat[1][2]*v[2];
            result[2] = rotMat[2][0]*v[0] + rotMat[2][1]*v[1] + rotMat[2][2]*v[2];


            //以下是求所有在平面附近距离小于0.1r的点的集合
            double D = -(result[0]*point.x + result[1]*point.y + result[2]*point.z);
            pcl::PCLPointCloud<pcl::PointXYZRGBA> tempCloud;
            for(auto [i, p] : neighborClouds)
            {
                pcl::PointXYZRGBA neighborPoint;
                
                double numerator = std::abs(result[0]*neighborPoint.x + result[1]*neighborPoint.y + result[2]*neighborPoint.z);
                double distance = numerator / denominator;
                if(distance <= 0.1*r){
                    tempCloud.push_back(point);
                }
                double[] feature = calcCurvatureCenter(tempCloud);
                totalFeature[i] = feature;
                
                //to store the max curvature
                if(abs(feature[0]) > maxCurvature)
                {   
                    index = i;
                    maxCurvature = abs(feature[0]);
                }
            }
        }

        int supplyIndex = abs(90 - index * unit) / unit;
        minCurvature = totalFeature[supplyIndex];

        double halfSum = (maxCurvature + minCurvature) / 2;
        
    }
    
}

/*
 * 返回avgDis也就是roughness
 */
double[] surfaceGen::calcCurvatureCenter(pcl::PointCloud<pcl::PointXYZRGBA> cloud, double radius)
{
    double minAvgDis = 1000;
    int size = cloud.size();
    int index = 0;
    double result[5];//store the curvature & center & avgDis & stdDis

    for(int i=0; i<size; i++)
    {
        pcl::PointXYZRGBA point = cloud.at(i);
        pcl::PointCloud<pcl::PointXYZRGBA> neighborCloud = calcNeighborCloud(cloud, radius, i);//得到所有的邻近点云

        double sum_x = 0, sum_y = 0, sum_r2 = 0, sum_x2y2 = 0, sum_x2 = 0, sum_y2 = 0;
        double sumDist1, sumDist2 = 0;
        double avgDis, stdDis;//点到圆的平均距离和标准差距离

        int neighborSize = neighborCloud.size();

        for(int j=0; j<neighborSize; j++)
        pcl::PointCloud<pcl::PointXYZRGBA> neighborCl
        {
            pcl::PointXYZRGBA p = neighborCloud[j];
            double x = p.x;
            double y = p.y;
            sum_x += x;
            sum_y += y;
            sum_r2 += x*x + y*y;
            sum_x2y2 += =x*x + y*y;
            sum_x2 += x*x;
            sum_y2 += y*y;
        }

        double Cx = sum_x / neighborSize;
        double Cy = sum_y / neighborSize;

        //计算拟合误差
        for(int j=0; j<neighborSize; j++)
        {
            pcl::PointXYZRGBA p = neighborCloud[j];
            double x = p.x;
            double y = p.y;
            double dx = x - Cx;
            double dy = y - Cy;
            double dist = sqrt(dx*dx + dy*dy);
            sumDist1 += dist;
        }
        avgDis = sumDist1 / neighborSize;

        if(avgDis < minAvgDis)
            index = i;
        
        //计算标准差距离
        for(int j=0; j<neighborSize; j++)
        {
            pcl::PointXYZRGBA p = neighborCloud[j]
            
            ;
            double x = p.x;
            double y = p.y;
            double dx = x - Cx;
            double dy = y - Cy;
            double dist = sqrt(dx*dx + dy*dy);
            sumDist2 += (dist - avgDis) * (dist - avgDis);
        }
    }

    stdDis = sqrt(sumDist2/n);

    double A = sum_x2y2 - n * Cx * Cx - n * Cy * Cy;
    double B = sum_x2 - sum_y2;
    double C = sum_r2 - n * Cx * Cx - n * Cy * Cy;

    double R = sqrt((B * B + A * A) / (4 * A * A + 4 * B * B));
    double curvature = 1.0 / R;
    


    result[0] = curvature;
    result[1] = Cx;
    result[2] = Cy;
    result[3] = avgDis;
    result[4] = stdDis; 
    result[5] = index;
    return result;
}
/*
 * 返回avgDis也就是roughness
 * 参数是vector
 */
double[] surfaceGen::calcCurvatureCenter(std::vector<pcl::PointXYZRGBA> cloud, double radius)
{
    double minAvgDis = 1000;
    int size = cloud.size();
    int index = 0;
    double result[5];//store the curvature & center & avgDis & stdDis

    for(int i=0; i<size; i++)
    {
        pcl::PointXYZRGBA point = cloud[i];
        pcl::PointCloud<pcl::PointXYZRGBA> neighborCloud = calcNeighborCloud(cloud, radius, i);//得到所有的邻近点云

        double sum_x = 0, sum_y = 0, sum_r2 = 0, sum_x2y2 = 0, sum_x2 = 0, sum_y2 = 0;
        double sumDist1, sumDist2 = 0;
        double avgDis, stdDis;//点到圆的平均距离和标准差距离

        int neighborSize = neighborCloud.size();

        for(int j=0; j<neighborSize; j++)
        pcl::PointCloud<pcl::PointXYZRGBA> neighborCl
        {
            pcl::PointXYZRGBA p = neighborCloud[j];
            double x = p.x;
            double y = p.y;
            sum_x += x;
            sum_y += y;
            sum_r2 += x*x + y*y;
            sum_x2y2 += =x*x + y*y;
            sum_x2 += x*x;
            sum_y2 += y*y;
        }

        double Cx = sum_x / neighborSize;
        double Cy = sum_y / neighborSize;

        //计算拟合误差
        for(int j=0; j<neighborSize; j++)
        {
            pcl::PointXYZRGBA p = neighborCloud[j];
            double x = p.x;
            double y = p.y;
            double dx = x - Cx;
            double dy = y - Cy;
            double dist = sqrt(dx*dx + dy*dy);
            sumDist1 += dist;
        }
        avgDis = sumDist1 / neighborSize;

        if(avgDis < minAvgDis)
            index = i;
        
        //计算标准差距离
        for(int j=0; j<neighborSize; j++)
        {
            pcl::PointXYZRGBA p = neighborCloud[j]
            
            ;
            double x = p.x;
            double y = p.y;
            double dx = x - Cx;
            double dy = y - Cy;
            double dist = sqrt(dx*dx + dy*dy);
            sumDist2 += (dist - avgDis) * (dist - avgDis);
        }
    }

    stdDis = sqrt(sumDist2/n);

    double A = sum_x2y2 - n * Cx * Cx - n * Cy * Cy;
    double B = sum_x2 - sum_y2;
    double C = sum_r2 - n * Cx * Cx - n * Cy * Cy;

    double R = sqrt((B * B + A * A) / (4 * A * A + 4 * B * B));
    double curvature = 1.0 / R;
    


    result[0] = curvature;
    result[1] = Cx;
    result[2] = Cy;
    result[3] = avgDis;
    result[4] = stdDis; 
    result[5] = index;
    return result;
}
/*
* 通过邻近点云基于PCA计算平面的法向量
*/
pcl::PointCloud<pcl::Normal> surfaceGen::surfNormal(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{

    int size = cloud->size();
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

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
    // for(int i=0; i<size; i++)
    // {
    //     normals->points[i].normal_x = v(0);
    //     normals->points[i].normal_y = v(1);
    //     normals->points[i].normal_z = v(2);
    //     normals->points[i].curvature = t1 / (val(0, 0) + val(1, 1) + val(2, 2));
    // }

    return new pcl::PointCloud<pcl::Normal>(v(0), v(1), v(2));
}

/*
* index: curvature最小的点
*/
pcl::PointCloud<pcl::PointXYZRGBA> surfaceGen::regionGrowth(pcl::PointCloud<pcl::PointXYZRGBA> rawCloud)
{
    int size = rawCloud.size();
    char[] status = new char[size];//记录点云的状态:accepted、reevaluated、rejected，分别用a、r、n来表示

    double epsilon = 0.01 * r;
    int i = 0;
    double radius = this.ds[i]
    int gamma = 0;
    double[] result = calcCurvatureCenter(rawCloud, radius);
    while(result[3] > epsilon)
    {
        i++;
        if(i>5)
        {
            if(epsilon>=r)
            {
                ROS_ERROR("Can't find a proper radius to segment!!!");
                return;
            }
            else{
                i=0;
                epsilon *= 2;
            }
        }
        radius = this.ds[i]; 
        result = calcCurvatureCenter(radius);
    }
    
    double uSeg =  result[3];//the roughness of the segment
    double tSeg =  result[4];// the standard distance of segment
    int index = (int)result[5];
    
    pcl::PointXYZRGBA seed = cloud.at(index);

    pcl::PointCloud<pcl::PointXYZRGBA> curRegion;
    std::vector<pcl::PointXYZRGBA> candidatesSet;
    std::vector<pcl::PointXYZRGBA> reevaluatedSet;
    curRegion.insert(seed);
    pcl::PointCloud<pcl::PointXYZRGBA> neighborCloud = calcNeighborCloud(rawCloud, radius, index);
    for(int i=0; i<neighborCloud.size(); i++)
        candidatesSet.push_back(neighborCloud.at(i));

    while(candidatesSet.size()>0)
    {
        pcl::PointXYZRGBA candiPoint = candidatesSet.pop_back();
        int candiIndex = std::find(rawCloud.begin(), rawCloud.end(), candiPoint);
        if(status[candiIndex])
            continue;
        pcl::PointCloud<pcl::PointXYZRGBA> candiNeighbor = calcNeighborCloud(rawCloud, radius, candiIndex);
        double[] candiRes = calcCurvatureCenter(candiNeighbor, radius);
        if(result[3] <= uSeg + 2*tSeg)
        {
            curRegion.push_back(candiPoint);
            result = calcCurvatureCenter(curRegion, radius);//重新计算Rc的roughness
            int candiIndex = std::find(cloud.begin(), cloud.end()), candiPoint);
            pcl::PointCloud<pcl::PointXYZRGBA> candiNeighbor = calcNeighborCloud(cloud, radius, candiIndex);
            curRegion.insert(candiPoint);
            status[candiIndex] = 'a';//accepted
            candidates.pop_back();//检测合格就删除
            for(int i=0; i<candiNeighborCloud.size(); i++)
            {
                candiIndex = std::find()
                candidatesSet.insert(candiNeighborCloud.at(i));
            }
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZRGBA> interPoints;//邻居点云和Rc的交集
            for(int j=0; j<neighborCloud.size(); j++)
            {
                pcl::PointXYZRGBA point = neighborCloud.at(j);
                auto iter = std::find(curRegion.begin(), curRegion.end(), point);
                if(iter != curRegion.end())
                interPoints.insert(point);
            }
            double newSurfRes = calcCurvatureCenter(interPoints, radius);
            if(newSurfRes[3] <= result[3] + 2*result[4])
            {
                curRegion.insert(candiPoint);
            }
            else{
                //设置为rejected
                auto iter = std::find(cloud.begin(), cloud.end(), point)
                status[(int)iter] = "n";//rejected
            }    
        }            
         
    }
    return curRegion;
}  



/*
 * calculate rational basis function
 */
double RationalBasisValue(int order, Eigen::Vector3d u, int index, std::vector<double> knots)
{
    Eigen::Vector3d uStart = knots[index];
    Eigen::Vector3d uEnd   = knots[index+1];

    if(order == 0)
        if(u<uEnd && u >= uStart)
            return 1;
        else 
            return 0;
    
    return (u - uStart)/(knots[index + DEGREE] - uStart) * RationalBasisValue(DEGREE - 1, index, knots) + 
    (knots[start+DEGREE+1] - u)/(knots[start+DEGREE+1] - uEnd) * RationalBasisValue(DEGREE-1, start+1, end+1);
}


void IR_BFS(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointXYZRGBA targetPoint)
{
    std::vector<Eigen::Vector3d> splinePoints;


    for(const pcl::PointXYZRGBA& point : cloud->points)
    {
        Eigen::Vector3d eigenPoint(point.x, point.y, point.z);
        splinePoints.push_back(eigenPoint);
    }

    size_t n = cloud->size();
    size_t knot_size = n+DEGREE+1;
    
    //calculate ubar
    std::vector<double> ubar(n);
    ubar[0] = 0;
    ubar[n-1] = 1;
    

    std::vector<double> distances(n-1);

    double totalDis = 0;
    for(int i=0; i<n; i++)
    {
        Eigen::Vector3d diff = abs(splinePoints[i+1] - splinePoints[i]);
        double squaredDistance = diff.squaredNum();
        double distance = std::sqrt(suqaredDistance);
        distances[i] = distance;

        totalDis += distance;
    }

    for(int i=1; i<n-1; i++)
        ubar[i] = ubar[i-1] + distances[i-1]/totalDis;
    //calculate bar


    /*calculate knot vector U*/

    std::vector<double> U(n+DEGREE+2);//一共n+p+2个点
    for(int i=0; i<=DEGREE; i++)
        U[i] = 0;
    for(int i=1+DEGREE; i<=n; i++)
        U[i] = 1/DEGREE * std::accumulate(ubar.begin()+i, ubar.begin()+i, ubar.begin()+i+DEGREE-1, 0.0);

    for(int i=n+1; i<DEGREE+n+1; i++)
        U[i] = 1;
    /*calculate knot vector U*/


    /*Interpolation of NURBS Curves*/
    std::vector<std::vector<double>> coefficients(n+1, std::vector<double>(n+1, 0.0)); //系数矩阵
    
    for(int k=0; k<=n; k++){
        for(int i=0; i<=n; i++){
            coefficients[k][i] = RationalBasisValue(DEGREE, ubar[k], )
        }
    }
    /*Interpolation of NURBS Curves*/


    /*Inversion Algorithm of NURBS Curves*/

    /*Inversion Algorithm of NURBS Curves*/

    
}

