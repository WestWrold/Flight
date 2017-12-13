#include<ros/ros.h>
#include<std_msgs/String.h>
#include<flight/ImgPro.h>
//#include<flight/Config.h>
using namespace std;

int main(int argc, char **argv)
{   
   // myslam::Config::setParameterFile("default.yaml");

    ros::init(argc,argv,"test_ros");
    ros::start();
    startWindowThread();
    
    ros::Rate loop_rate(10);
    imgPro imp("default.yaml");
    imp.Run();
    
    
    /* imp.blockSize = myslam::Config::get<int>("blockSize");
    imp.disparity = myslam::Config::get<int>("disparity");
    imp.INVARIANCE_CHECK_HORZ_OFFSET_MAX = myslam::Config::get<int>("INVARIANCE_CHECK_HORZ_OFFSET_MAX");
    imp.INVARIANCE_CHECK_HORZ_OFFSET_MIN = myslam::Config::get<int>("INVARIANCE_CHECK_HORZ_OFFSET_MIN");
    imp.INVARIANCE_CHECK_VERT_OFFSET_INCREMENT = myslam::Config::get<int>("INVARIANCE_CHECK_VERT_OFFSET_INCREMENT");
    imp.INVARIANCE_CHECK_VERT_OFFSET_MAX = myslam::Config::get<int>("INVARIANCE_CHECK_VERT_OFFSET_MAX");
    imp.INVARIANCE_CHECK_VERT_OFFSET_MIN = myslam::Config::get<int>("INVARIANCE_CHECK_VERT_OFFSET_MIN");
    imp.zero_disparity = myslam::Config::get<int>("zero_disparity");
    imp.sobelLimit = myslam::Config::get<int>("sobelLimit");
    imp.sadThreshold = myslam::Config::get<int>("sadThreshold");

    cout << imp.blockSize <<endl;
    cout << imp.disparity <<endl;
    cout << imp.INVARIANCE_CHECK_HORZ_OFFSET_MAX <<endl;
    cout << imp.INVARIANCE_CHECK_HORZ_OFFSET_MIN <<endl;
    cout << imp.INVARIANCE_CHECK_VERT_OFFSET_INCREMENT <<endl;
    cout << imp.INVARIANCE_CHECK_VERT_OFFSET_MAX <<endl;
    cout << imp.INVARIANCE_CHECK_VERT_OFFSET_MIN <<endl;
    cout << imp.zero_disparity <<endl;
    cout << imp.sobelLimit <<endl;
    cout << imp.sadThreshold <<endl;*/
    

    int count  = 0;
    while (ros::ok())
    {
        cout << count++ <<endl;
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello wrold " <<count;
       if(imp.img.cols >0 && imp.img.rows > 0)
       {
        imshow("image",imp.img);
       }
       else{
           cout << "img not right " <<endl; 
       } 
        
        ros::spinOnce();
        loop_rate.sleep();
        
        //cout << imp.blockSize <<endl;
        
    }

    return 0;
}