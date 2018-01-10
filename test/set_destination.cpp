#include<ros/ros.h>
#include"Flight/destinate.h"
#include<stdlib.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "set_destination");
    if (argc!=4)
    {
        ROS_INFO("please input destination coordinate");
        return 1;
    }
    ros::NodeHandle nh4;
    ros::ServiceClient des_client=nh4.serviceClient<Flight::destinate>("set_destination");
    Flight::destinate des_srv;
    des_srv.request.x=atoll(argv[1]);
    des_srv.request.y=atoll(argv[2]);
    des_srv.request.z=atoll(argv[3]);

    if (des_client.call(des_srv))
    {
        ROS_INFO("set destination successfully");
    }
    else
    {
        ROS_ERROR("failed to set destination");
        return 1;
    }
    return 0;
}