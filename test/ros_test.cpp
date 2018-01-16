#include<ros/ros.h>
#include<std_msgs/String.h>
#include<flight/ImgPro.h>
#include<flight/Frame.h>
#include<flight/StereoMap.h>
#include"ros_traj_plan/searchdistance.h"
#include"ros_traj_plan/uavtrans.h"
//#include<flight/Config.h>
using namespace std;

int main(int argc, char **argv)
{   
   // myslam::Config::setParameterFile("default.yaml");
    ros::init(argc,argv,"test_ros");
    imgPro imp("default.yaml");
    Frame framePro(imp.matQ);
    StereoMap map_world;
    ros::NodeHandle nh;
    Uav_trajplannimg traj_planning;
    traj_planning.fly_init();//飞机初始化
    traj_planning.fly_takeoff(map_world.current_octree_);//飞机起飞(需要一个地图，里面可以没有点)
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subLeftImg = it.subscribe("/zed/left/image_rect_color",1,&imgPro::getImgLeft,&imp);
    image_transport::Subscriber subRightImg = it.subscribe("/zed/right/image_rect_color",1,&imgPro::getImgRight,&imp);
    ros::Subscriber odom = nh.subscribe("/zed/odom",1,&Frame::getOdom,&framePro);
    framePro.imageResultleft = it.advertise("obstacle_points",10);
    
    ros::Rate loop_rate(10);    
    //imp.image_pub_left = it.advertise("show_result",1);
  //  imp.image_pub_right = it.advertise("show_result",1);
    
  while (ros::ok()&&!traj_planning.arrive_destination(traj_planning.get_trans_msg_a(),traj_planning.get_destinate_coor()))
    {   
        vector<Point3f> hitPointsCamera;
        vector<Point3i> hitPointsPixel;
        imp.HitPoints(hitPointsCamera,hitPointsPixel);
        framePro.visualizaFrame(imp.imgLeft,hitPointsPixel,imp.blockSize);
        framePro.pixelToCamera(hitPointsCamera);
        framePro.cameraToWorld();
        map_world.InsertPointsIntoOctree(framePro.hitPointsWorld);
        framePro.flashFrame();
        map_world.RemoveOldPoints(ros::Time::now());
         traj_planning.fly_traj_plan(map_world.current_octree_);
        
        ros::spinOnce();
        loop_rate.sleep();

        
        
    }

    return 0;
}