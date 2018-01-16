#include<flight/StereoMap.h>

StereoMap::StereoMap()
{
    current_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    building_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());


    current_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
    current_octree_->setInputCloud((pcl::PointCloud<pcl::PointXYZ>::Ptr)current_cloud_);

    building_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
    building_octree_->setInputCloud(building_cloud_);


}
void StereoMap::RemoveOldPoints(ros::Time ros_time){
    if(flag_firstStart == true) {
        current_octree_timestamp_ = ros_time;
        building_octree_timestamp_ = ros_time + ros::Duration(2);
        flag_firstStart = false;
    } else if (current_octree_timestamp_ > ros_time)
    {   //时间戳出错
        delete current_octree_;
        delete building_octree_;
    

        current_octree_timestamp_ = ros_time;
        building_octree_timestamp_ = ros_time + ros::Duration(2);

        current_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        building_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        
        //重新建图
        current_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
        current_octree_->setInputCloud((pcl::PointCloud<pcl::PointXYZ>::Ptr)current_cloud_);
        
        building_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
        building_octree_->setInputCloud((pcl::PointCloud<pcl::PointXYZ>::Ptr)building_cloud_);

        std::cout << std::endl << "swapping octrees because jump back in time" << std::endl;

    }else if (current_octree_timestamp_ + ros::Duration(4) < ros_time)
    {
        //std::cout << "swapping octrees (old: " << (current_octree_timestamp_ - last_msg_time) / 1000000.0f << ", new: " << (building_octree_timestamp_ - last_msg_time) / 1000000.0f << ")" << std::endl;
        //新老地图交替
       // pcl::io::savePCDFile("./result.pcd", *current_cloud_);
        delete current_octree_;
        current_octree_ = building_octree_;
        current_cloud_ = building_cloud_;
        current_octree_timestamp_ = building_octree_timestamp_;
        building_octree_timestamp_ = ros_time;
        //构建新地图
        building_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        building_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
       
        
        
    }   
}

void StereoMap::InsertPointsIntoOctree(vector<Point3f> hitPointsWorld)
{   
    if(hitPointsWorld.size()>0)
    {
    for(int i = 0; i<hitPointsWorld.size(); i++)
        {
    pcl::PointXYZ obstaclePoints(hitPointsWorld[i].x,hitPointsWorld[i].y,hitPointsWorld[i].z);
    current_cloud_->push_back(obstaclePoints);
    building_cloud_->push_back(obstaclePoints);
        }
    }
    else{
        cout << "hitPointsWorld is empty" <<endl;
    }

    cout << "number of obstaclePoints in current cloud:  " << current_cloud_->size()<<endl;
    cout << "number of obstaclePoints in building cloud:  " << building_cloud_->size()<<endl;

}

void StereoMap::stereoMapStatus()
{

}