#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<flight/common_include.h>
#include<flight/Config.h>


using namespace std;
class imgPro
{
public:
    //checkHorizontalInvariance
    int zero_disparity,sobelLimit,disparity;
    bool check_horizontal_invariance = true;
    int INVARIANCE_CHECK_HORZ_OFFSET_MIN,INVARIANCE_CHECK_HORZ_OFFSET_MAX,INVARIANCE_CHECK_VERT_OFFSET_INCREMENT;
    cv::Mat imgLeft,imgRight,laplacianLeft,laplacianRight;
    int flag;
    int sadThreshold;
    int blockSize,INVARIANCE_CHECK_VERT_OFFSET_MIN,INVARIANCE_CHECK_VERT_OFFSET_MAX,horizontalInvarianceMultiplier;
    image_transport::Publisher image_pub_left,image_pub_right;

    void getImgRight(const sensor_msgs::ImageConstPtr& msg);
    void getImgLeft(const sensor_msgs::ImageConstPtr& msg);
    int  getSAD(Mat& leftImage, Mat& rightImage, Mat& laplacianL, Mat& laplacianR, int pxX, int pxY);
    bool checkHorizontalInvariance(Mat& leftImage, Mat& rightImage, Mat& sobelL, Mat& sobelR, int pxX, int pxY);
    imgPro(string strSettingPath);
    vector<Point3f> hitPoints();
    void LaplacianPro(cv::Mat& src_left,cv::Mat& src_right,cv::Mat& dst_left,cv::Mat& dst_right);
    void LaplacianPro();
};