#include<ros/ros.h>
#include<std_msgs/String.h>
#include<flight/ImgPro.h>
//#include<flight/Config.h>
using namespace std;

int main(int argc, char **argv)
{   
   // myslam::Config::setParameterFile("default.yaml");
    int flag ;
    ros::init(argc,argv,"test_ros");
    
    
    flag = 4;
    cout << flag <<endl;
    imgPro imp("default.yaml");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subLeftImg = it.subscribe("/zed/left/image_rect_color",1,&imgPro::getImgLeft,&imp);
    image_transport::Subscriber subRightImg = it.subscribe("/zed/right/image_rect_color",1,&imgPro::getImgRight,&imp);
    imp.image_pub_right = it.advertise("/image_converter/right", 1);
    imp.image_pub_left = it.advertise("/image_converter/left", 1);   
    
   // flag = 3;
    ros::Rate loop_rate(10);
   // imshow("image",imp.img);

    
    
   vector<Point3f> hitpoints;

    int count  = 0;
    while (ros::ok())
    {
        hitpoints = imp.hitPoints();
        cout << count++ <<endl;
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello wrold " <<count;
        if(imp.imgLeft.cols >0 && imp.imgLeft.rows > 0)
        {
        // imshow("image",imp.imgLeft);
        cout << "got you!" <<endl;
        }
        else{
            cout << "img not right " <<endl; 
            cout << imp.imgLeft.cols <<","<<imp.imgLeft.rows<<endl;
        } 
        ros::spinOnce();       
        loop_rate.sleep();
        //cout << imp.blockSize <<endl;
        
    }

    return 0;
}