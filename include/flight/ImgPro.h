#include<common_include.h>
#include<image_transport/image_ransport.h>

class imgPro
{
public:
    void getImgRight(const sensor_msg::ImageConstPtr& msg);
    void getImgLeft(const sensor_msg::ImageConstPtr& msg);
    int  getSAD(Mat& leftImage,Mat& rightImage, int pxX, int pxY);
    bool checkHorizontalInvariance(Mat& leftImage, Mat& rightImage, int pxX, int pxY );

};