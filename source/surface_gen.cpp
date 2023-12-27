//输入系列点云生成三维b样条曲面
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/kdtree/kdtree_flann.h>

#include<cmath>
#include<vector>
#include<set>
#include<limits>

#include<Eigen/Eigenvalues>

#define r 1
#define Q 500 //设定的临近点云数量

#define DEGREE 3 //曲线阶数

//存储划分后的不同segment
std::vector<std::vector<pcl::PointXYZRGB>> segments;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "surface_gen");
    ROS_INFO("Calculate 3d surface via Swarm Algorithm.....");

    ros::NodeHandle nh;
    
    ros::Subscriber pointCloudSub = nh.subscribe("/camera/pointcloud", 10, &segmentation);

}

struct Edge
{
    int source;
    int target;
    double weight;
};

/*
* segement the point cloud to different parts
* the first step is to segment several point cloud segmentations
*/
void  segmentation(sensor_msgs::PointCloud2ConstPtr& inCloud)
{
    segments.clear();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*inCLoud, *cloud);

    // pcl::fromROSMsg(*inCloud, *cloud); 当数据量过大时，这种方式比较耗时
    // 通过直接复制数据地址的方式来进行数据类型转换
    // size_t numPoints = inCloud->width * inCloud->height;
    // std::memcpy(cloud->points.data(), inCloud->data.data(), inCloud->data.size());
    // cloud->width = inCloud->width;
    // cloud->height = inCloud->height;


    //点云降采样  PREPROCESS({P}, r)
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.1 * r, 0.1 * r, 0.1 * r);    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_grid.filter(*downsampledCloud);

    //寻找seedPoint
    FeatureResult* seedFeature;
    pcl::PointXYZRGB seedPoint = findSeed(downSampmledCloud, seedFeature);
    while(seedPoint != nullptr && downsampledCloud->size() != 0)
    {
        std::vector<pcl::PointXYZRGB> currRegion;
        std::vector<pcl::PointXYZRGB> candidates;
        currRegion.push_back(seedPoint);
        FeatureResult estimatedFeature = *seedFeature;
        double uRc = estimatedFeature.avgDis;
        double tRc = estimatedFeature.stdDis;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighborPoints = findNeighbor(downsampledCloud, seedPoint, r, 500);
        for(std::size_t i=0; i<neighborPoints->size(); i++)
        {
            candidates.push_back((*neighborPoints)[i]);
        }

        double Fc;
        while(candidates.size() != 0)
        {
            auto pointIt = candidates.begin();
            while(pointIt != candidates.end)
            {
                pcl::PointXYZRGB localPoint = *pointIt;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr localCloud = findNeighbor(downsampledCloud, localPoint, r, 500);
                FeatureResult localFeature = fitCircle(localCloud);
                if(localFeature.avgDis <= uRc + 2 * tRc)
                {
                    currRegion.push_back(localPoint)
                    for(size_t i=0; i<localCloud->size(); i++)
                        candidates.push_back((*localCloud)[i]);
                }
                else
                    Fc = fitCircle(candidates);
                
                if(localFeature.stdDis <= uRc + 2 * tRc)
                {
                    currRegion.push_back(*pointIt);
                    for(size_t i=0; i<localCloud->size(); i++)
                        candidates.push_back((*localCloud)[i]);
                }
                pointIt = candidates.erase(pointIt);
            }
        }

        double uRcSum = 0;
        double tRcSum = 0;

        std::vector<int> indices;

        for(size_t i=0; i<currRegion.size(); i++)
        {
            pcl::PointXYZRGB localPoint = currRegion[i];
            pcl::PointCloud<pcl::PointXYZRGB> localCloud = findNeighbor(downsampledCloud, localPoint, r, 500);
            FeatureResult localFeature = fitCircle(localCloud);
            uRcSum += localFeature.avgDis; 
            tRcSum += localFeature.stdDis;

            int index = downsampledCloud->findNearestPoint(localPoint);
            if(index != -1)
                indices.push_back(index);
        }

        downsampledCloud->erase(indices);

        uRc = uRcSum/currRegion.size();
        tRc = tRcSum/currRegion.size();

        segments.push_back(currRegion);

        seedPoint = findSeed(downSampmledCloud, seedFeature);
    }
} 

/*
 * 寻找邻域点
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr findNeighbor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointXYZRGB& point, double radius, std::size_t maxPoints)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighborPoints(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<int> pointIndices;
    std::vector<float> pointDistances;
    kdtree.radiusSearch(point, radius, pointIndices, pointDistances);

    std::size_t count = 0;
    for(std::size_t i=0; i<pointIndices.size() && count < maxPoints; i++)
    {
        pcl::PointXYZRGB neighborPoint = (*cloud)[pointIndices[i]];
        neighborPoints->push_back(neighborPoint);
        count++;
    }   
    return neighborPoints;
}

/*
 *计算
*/
std::vector<Eigen::Vector3d> computeRotatedNormals(pcl::PointXYZ seedPoint, pcl::PointCloud<pcl::PointXYZ> cloud)
{
    //计算附近点云组成的平面的法线
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighbor50 = calcNeighborCloud(cloud, r, 50, seedPoint);
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
    return planeNormals;
}
/*
 * 寻找种子点，对种子点进行扩充从而形成一个segment
*/
pcl::PointXYZRGB findSeed(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, FeatureResult* seedFeature)
{
    std::vector<double> radiSet = {r, 0.72*r, 0.52*r, 0.37*r, 0.27*r, 0.2*r};
    double tau = 0.01 * r; //代表表面roughness
    int gamma = 0;
    double radius = r;

    pcl::PointXYZRGB seedPoint;
    
    while(tau <= r)
    {
        while(gamma <= radiSet.size() - 1)
        {
            radius = radiSet[gamma];
            int randomIndex = rand() % cloud->size();
            seedPoint = (*cloud)[randomIndex];

            //计算附近点云组成的平面的法线
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighbor50 = calcNeighborCloud(downsampledCloud, r, 50, seedPoint);
            pcl::Normal normal = computePlaneNormal(neighbor50);

            std::vector<Eigen::Vector3f> planeNormals;
            planeNormals = computeRotatedNormals(seedPoint, cloud);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighbor500 = calcNeighborCloud(downsampledCloud, radius, 500, seedPoint);

            std::vector<Eigen::Vector2d> points; 

            FeatureResult minPlaneFeature;
            minPlaneFeature.curvature = (double)std::numeric_limits<int>::max();//记录拥有最小curvature的平面
            //离平面距离小于0.1*r的所有点构成平面，再进行circle fitting
            for(Eigen::Vector3f plane : planeNormals)
            {
                for(size_t i= 0; i<neighbor500->size(); i++)
                {
                    Eigen::Vector3d point((*neighbor500)[i].x, (*neighbor500)[i].y, (*neighbor500)[i].z);
                    double distanceToPlane = std::abs((point - (*neighbor500)[i].getVector3fMap().dot(plane)));
                    if(distanceToPlane < 0.1 * r)
                    {
                        Eigen::Vector2d projectedPoint;
                        projectedPoint << (point - (*neighbor500)[i].getVector3fMap().dot(plane).norm(), point.z());
                        points.push_back(projectedPoint);            
                    }
                }
                
                FeatureResult feature;
                feature = fitCircle(points);
                //寻找最小curvature
                if(feature.curvature < tau){
                    *seedFeature = feature;
                    return seedPoint;
                }
                points.clear();//每次清空放下一个平面的点
            }
            gamma++;
        }
       tau *= 2;
       gamma = 0;
    }
    return nullptr; 
}

struct FeatureResult
{
    double curvature;
    Eigen::Vector2d center;
    double meanFitError;
    double stdFitError;
}
/*
 * 二维空间circle fit
*/
FeatureResult fitCircle(const std::vector<Eigen::Vector2d> &points)
{
    FeatureResult circle;
    Eigen::MatrixXd A(points.size(), 3);
    Eigen::VectorXd b(points.size());

    for(size_t i=0; i < points.size(); i++)
    {
        A(i, 0) = 2 * points[i](0);
        A(i, 1) = 2 * points[i](1);
        A(i, 2) = -1;
        b(i) = points[i].squaredNorm();
    }

    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
    circle.center << x(0), x(1);
    circle.curvature = std::sqrt(x(0) * x(0) + x(1) * x(1) - x(2));

    Eigen::VectorXd distances(points.size());
    for(size_t i=0; i<points.size(0); i++) {
        distances(i) = (points[i] - circle.center).norm() - circle.curvature;
    }
    
    circle.meanFitError = distances.mean();
    circle.stdFitError = distances.array().abs().matrix().mean();

    return circle;
}

/*
 * 计算一堆点云的法线
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
 * 文中定义的能量函数
*/
double EnergyFunction(Eigen::Vector3d plane, pcl::PointXYZ planePoint, pcl::PointCloud<pcl::PointXYZ> cloud)
{
    double posDenominator, posNumerator = 0;//表示正距离的点
    double negDenominator, negNumerator = 0;//表示负距离的点
    int negPoints, posPoints = 0;//记录满足的点
    for(int i=0; i<cloud.size(); i++)
    {
        pcl::PointXYZ point = cloud[i];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighbor50 = calcNeighborCloud(cloud, r, 50, point);
        pcl::Normal normal = computePlaneNormal(neighbor50);
        
        double distance = (plane.x() * point.x + plane.y() * point.y + plane.z() * point.z - 
                plane.x() * planePoint.x - plane.y() * planePoint.y - plane.z() * planePoint.z) / 
                std::sqrt(plane.x()*plane.x() + plane.y()*plane.y() + plane.z()*plane.z());

        if(std::abs(distance)<=r)
        {
            double numerator = (1 - std::abs(plane.x()*normal.normal_x + plane.y()*normal.normal_y + plane.z()*normal.normal_z))
                        * std::exp(-3*r*distance*distance);
            double denominator = std::exp(-3*r*distance*distance);
            if(distance<0)
            {
                negPoints++;
                negDenominator += denominator;
                negNumerator += numerator;
            }
            else
            {
                posPoints++;
                posDenominator += denominator;
                posNumerator += numerator;
            }
        }
    }

    double posScore = posNumerator / posDenominator;
    double negScore = negNumerator / negDenominator;
    if(negPoints >= 50 && posPoints >= 50)
    {
        return min(posScore, negScore);
    }
    else if(negPoints >= 50)
        return negScore;
    else if(posPoints >= 50)
        return posScore;
    else
        return nullptr;
}
/*
 * 使用nurbs对不同segment进行fit
*/
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;
std::vector<Eigen::Vector3d> nurbsFitting()
{
    for(std::vector<pcl::PointXYZ> segment : segments)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        cloud->points = segment;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        //initialization
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dist(0, segment.size()-1);
        int random_index = dist(gen);
        pcl::PointXYZ seedPoint = segment[random_index];

        std::vector<Eigen::Vector3d> planeNormals = computeRotatedNormals(seedPoint, cloud);

        Eigen::Vector3d bestPlane(0,0,0);
        double maxEnergy = (double)std::numeric_limits<int>::min();
        for(Eigen::Vector3d plane : planeNormals)
        {
            double energy = EnergyFunction(plane, seedPoint, cloud);
            if(energy != nullptr && energy > maxEnergy)
            {
                bestPlane = plane;
                maxEnergy = energy;
            }
        }

        if(maxEnergy == (double)std::numeric_limits<int>::min()) //no cs found
            return;

        //将点云投影到选择的二维平面然后b-spline fitting

    }
}

double computePointScore(const pcl::PointCloud& point, const std::vector<pcl::PointXYZ>& neighbors, const Eigen::Vector3d& normal)
{
    double score = 0.0;
    for(const auto& neighbor : neighbors)
    {
        Eigen::Vector3d v1 = point.
    }
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

