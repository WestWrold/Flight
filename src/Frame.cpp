#include<flight/Frame.h>

Frame::Frame(cv::Mat Q)
{
   // myslam::Config::setParameterFile(settingFile);
    matQ = Q;
    T = Eigen::Isometry3d::Identity();
    cv::namedWindow("image window");
    flag_simoutanous = false;
    
}

sensor_msgs::ImagePtr Frame::imageToROSmsg(cv::Mat img, const std::string encodingType,std::string frameId, ros::Time t){
    sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    sensor_msgs::Image& imgMessage = *ptr;
    imgMessage.header.stamp = t;
    imgMessage.header.frame_id = frameId;
    imgMessage.height = img.rows;
    imgMessage.width = img.cols;
    imgMessage.encoding = encodingType;
    int num = 1; //for endianness detection
    imgMessage.is_bigendian = !(*(char *)&num == 1);
    imgMessage.step = img.cols * img.elemSize();
    size_t size = imgMessage.step * img.rows;
    imgMessage.data.resize(size);

    if (img.isContinuous())
        memcpy((char*)(&imgMessage.data[0]), img.data, size);
    else {
        uchar* opencvData = img.data;
        uchar* rosData = (uchar*)(&imgMessage.data[0]);
        for (unsigned int i = 0; i < img.rows; i++) {
            memcpy(rosData, opencvData, imgMessage.step);
            rosData += imgMessage.step;
            opencvData += img.step;
        }
    }
    return ptr;
}

void Frame::getOdom(const nav_msgs::Odometry& msg)
{   
    ROS_INFO_STREAM(setprecision(6)<< fixed << "position = ("<<msg.pose.pose.position.y*10<<","<<msg.pose.pose.position.z*10<<","<<msg.pose.pose.position.x*10);
    ROS_INFO_STREAM(setprecision(6)<< fixed << "orientation = ("<<msg.pose.pose.orientation.x<<","<<msg.pose.pose.orientation.y<<","<<msg.pose.pose.orientation.z<<","<<msg.pose.pose.orientation.w);
    // T = Eigen::Isometry3d::Identity();
    
    //Eigen::Quaterniond q = Eigen::Quaterniond(-msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, -msg.pose.pose.orientation.x, msg.pose.pose.orientation.w);
    Eigen::Quaterniond q = Eigen::Quaterniond(1, 0, 0, 0);
    
    Eigen::Vector3d translate(msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.position.x*10);
    Eigen::Matrix3d R;
    R = q ;
    cout << translate <<endl;
    cout << R <<endl;
    //T.block(0,0,3,3) = R ;
    T.rotate(q);
    T.pretranslate(translate);
    ROS_INFO_STREAM(setprecision(8)<< fixed << "T = "<<T.matrix());
    flag_simoutanous = true;
}

void Frame::visualizaFrame(cv::Mat& displayL, vector<Point3i> pointVector2d,int blockSize)
{
    if (displayL.cols >9 && displayL.rows>0){
    for(unsigned int i=0;i<pointVector2d.size();i++){
        int x2=pointVector2d[i].x;
        int y2=pointVector2d[i].y;
  
	    rectangle(displayL,Point(x2,y2),Point(x2+blockSize,y2+blockSize),255);
	    rectangle(displayL,Point(x2+1,y2+1),Point(x2+blockSize-1,y2-1+blockSize),255);
        
        }
        
   // cv::imshow("image window", displayL);
   // cv::waitKey(1);
    //imageResultleft.publish(imageToROSmsg(displayL,sensor_msgs::image_encodings::BGR8,a,ros::Time::now()));
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", displayL).toImageMsg();
    imageResultleft.publish(msg);   
}
}

void Frame::pixelToCamera(std::vector<Point3f> hitPointsPixel){
    if(hitPointsPixel.size()>0){
    perspectiveTransform(hitPointsPixel,hitPointsCamera,matQ);
    }
    else{
        cout <<" pixelToCamera is Empty" << endl;
    }
}
void Frame::cameraToWorld()
{
    if(hitPointsCamera.size()>0)
    {
    for(int i = 0; i< hitPointsCamera.size(); i++)
        {
        Eigen::Vector3d v(hitPointsCamera[i].x,hitPointsCamera[i].y,hitPointsCamera[i].z);
        v = T.inverse()*v;
        Point3f temp(v(2,0),v(0,0),v(1,0));
        hitPointsWorld.push_back(temp);
        
        }
    }
    else {
        cout << "pointsCamera is empty" <<endl;
    }
   // return T.inverse() *hitPointsCamera;

}