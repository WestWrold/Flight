#include<ros/ros.h>
#include<std_msgs/String.h>
#include<flight/ImgPro.h>
#include<flight/Config.h>
using namespace std;

int main(int argc, char **argv)
{   
    myslam::Config::setParameterFile("default.yaml");

    ros::init(argc,argv,"test_ros");
    ros::NodeHandle nh;
    ros::Publisher test = nh.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);
    imgPro imp;
    imp.blockSize =myslam::Config::get<int>("blockSize");
    cout << imp.blockSize <<endl;

    int count  = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello wrold " <<count;
        ROS_INFO( "%s", msg.data.c_str());
        test.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        cout << imp.blockSize <<endl;
        
    }

    return 0;
}