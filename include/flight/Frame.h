#include<flight/common_include.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>

#include <nav_msgs/Odometry.h>
class Frame
{
public:
    cv::Mat matQ;
    Eigen::Isometry3d T ;
    std::vector<Point3f> hitPointsCamera;
    std::vector<Point3f> hitPointsWorld;
    bool flag_simoutanous;
    image_transport::Publisher imageResultleft;
public:
    void flashFrame();
    ~Frame ()
    {
        hitPointsCamera.clear();
        vector<Point3f>(hitPointsCamera).swap(hitPointsCamera);


        hitPointsWorld.clear();
        vector<Point3f>(hitPointsWorld).swap(hitPointsWorld);
        
        cout<< "Frame points has been removed" <<endl;

    }
    Frame(cv::Mat Q);
    void getOdom(const nav_msgs::Odometry& msg);
    sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType,std::string frameId, ros::Time t);
    void visualizaFrame(cv::Mat& displayL, vector<Point3i> pointVector2d,int blockSize);
    void pixelToCamera(std::vector<Point3f> hitPointsPixel);
    void cameraToWorld();
};