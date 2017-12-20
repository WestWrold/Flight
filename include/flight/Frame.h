#include<flight/common_include.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
class Frame
{
public:
    cv::Mat matQ;
public:
    void PublishFrame();
    void toRosMsg();
    sensor_msgs::ImagePtr imageToROSmsg();
};