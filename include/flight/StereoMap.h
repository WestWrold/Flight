#include<flight/common_include.h>

#include<pcl/point_cloud.h>
#include<pcl/octree/octree.h>

#define OCTREE_RESOLUTION 128.0f
class StereoMap{

public:

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *current_octree_;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *building_octree_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr building_cloud_;
    std::vector<Point3f> cameraHitPoints;
    std::vector<Point3f> worldHitPoints;
    ros::Time   current_octree_timestamp_, building_octree_timestamp_;
    bool flag_firstStart = true;
    void cameraToWorld();
    void RemoveOldPoints(ros::Time ros_time);
    void InsertPointsIntoOctree(vector<Point3f> hitPointsWorld);

};