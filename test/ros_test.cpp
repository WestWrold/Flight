#include<ros/ros.h>
#include<std_msgs/String.h>
#include<flight/ImgPro.h>
#include<flight/Frame.h>
//#include<flight/Config.h>
using namespace std;

int main(int argc, char **argv)
{   
   // myslam::Config::setParameterFile("default.yaml");
    ros::init(argc,argv,"test_ros");
    imgPro imp("default.yaml");
    Frame framePro(imp.matQ);
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subLeftImg = it.subscribe("/zed/left/image_rect_color",1,&imgPro::getImgLeft,&imp);
    image_transport::Subscriber subRightImg = it.subscribe("/zed/right/image_rect_color",1,&imgPro::getImgRight,&imp);
    framePro.imageResultleft = it.advertise("obstacle_points",10);
    
    ros::Rate loop_rate(10);    
    //imp.image_pub_left = it.advertise("show_result",1);
  //  imp.image_pub_right = it.advertise("show_result",1);
    
    while (ros::ok())
    {   
        vector<Point3f> hitPointsCamera;
        vector<Point3i> hitPointsPixel;
        imp.HitPoints(hitPointsCamera,hitPointsPixel);
        framePro.visualizaFrame(imp.imgLeft,hitPointsPixel,imp.blockSize);
        

       if(imp.imgLeft.cols >0 && imp.imgLeft.rows > 0)
       {
       cout << "got you!" <<endl;
       }
       else{
           cout << "img not right " <<endl; 
           cout << imp.imgLeft.cols <<","<<imp.imgLeft.rows<<endl;
       } 
        
        ros::spinOnce();
        loop_rate.sleep();

        
        
    }

    return 0;
}