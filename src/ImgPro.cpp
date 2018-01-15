#include"flight/ImgPro.h"
//
    //read Parameter from parameterfile
//
imgPro::imgPro(string strSettingPath)
{
    myslam::Config::setParameterFile(strSettingPath);

    blockSize = myslam::Config::get<int>("blockSize");
    disparity = myslam::Config::get<int>("disparity");
    INVARIANCE_CHECK_HORZ_OFFSET_MAX = myslam::Config::get<int>("INVARIANCE_CHECK_HORZ_OFFSET_MAX");
    INVARIANCE_CHECK_HORZ_OFFSET_MIN = myslam::Config::get<int>("INVARIANCE_CHECK_HORZ_OFFSET_MIN");
    INVARIANCE_CHECK_VERT_OFFSET_INCREMENT = myslam::Config::get<int>("INVARIANCE_CHECK_VERT_OFFSET_INCREMENT");
    INVARIANCE_CHECK_VERT_OFFSET_MAX = myslam::Config::get<int>("INVARIANCE_CHECK_VERT_OFFSET_MAX");
    INVARIANCE_CHECK_VERT_OFFSET_MIN = myslam::Config::get<int>("INVARIANCE_CHECK_VERT_OFFSET_MIN");
    zero_disparity = myslam::Config::get<int>("zero_disparity");
    sobelLimit = myslam::Config::get<int>("sobelLimit");
    sadThreshold = myslam::Config::get<int>("sadThreshold");

    double matQ_03,matQ_13,matQ_23,matQ_32;
    matQ_03 = myslam::Config::get<double>("matQ[0][3]");
    matQ_13 = myslam::Config::get<double>("matQ[1][3]");
    matQ_23 = myslam::Config::get<double>("matQ[2][3]");
    matQ_32 = myslam::Config::get<double>("matQ[3][2]");

    matQ=(Mat_<double>(4,4) <<1, 0, 0, matQ_03, 0, 1, 0, matQ_13, 0, 0, 0, matQ_23, 0, 0,matQ_32, 0);
    
    cout <<"matQ : " << matQ <<endl;
    cout <<"blockSize : " <<blockSize <<endl;
    cout <<"disparity : " <<disparity <<endl;
    cout <<"INVARIANCE_CHECK_HORZ_OFFSET_MAX : " <<INVARIANCE_CHECK_HORZ_OFFSET_MAX <<endl;
    cout <<"INVARIANCE_CHECK_HORZ_OFFSET_MIN : " <<INVARIANCE_CHECK_HORZ_OFFSET_MIN <<endl;
    cout <<"INVARIANCE_CHECK_VERT_OFFSET_INCREMENT : " <<INVARIANCE_CHECK_VERT_OFFSET_INCREMENT <<endl;
    cout <<"INVARIANCE_CHECK_VERT_OFFSET_MAX : " <<INVARIANCE_CHECK_VERT_OFFSET_MAX <<endl;
    cout <<"INVARIANCE_CHECK_VERT_OFFSET_MIN : " <<INVARIANCE_CHECK_VERT_OFFSET_MIN <<endl;
    cout <<"zero_disparity : " <<zero_disparity <<endl;
    cout <<"sobelLimit　: " <<sobelLimit <<endl;
    cout <<"sadThreshold　: " <<sadThreshold <<endl;


}
void imgPro::HitPoints(vector<Point3f>& localHitPoints, vector<Point3i>& pointVector2d)
{   

    if(imgRight.cols > 0 && imgRight.rows > 0)
    {
    LaplacianPro();
    int rows=imgRight.rows;
    int cols=imgLeft.cols; 
   // vector<uchar> pointColors;
    int hitCounter = 0;
    int startJ=0;
    int stopJ=cols-(disparity+blockSize);
    if(disparity<0){
        startJ=-disparity;
        stopJ=cols-blockSize;
    }
    for(int i=0;i<rows;i+=blockSize){
    for(int j=startJ;j<stopJ;j+=blockSize){
        int sad=getSAD(imgLeft,imgRight,laplacianLeft,laplacianRight,j,i);
        if(sad<sadThreshold && sad>=0){
            if(!check_horizontal_invariance || checkHorizontalInvariance(imgLeft,imgRight,laplacianLeft,laplacianRight,j,i)==false){
            localHitPoints.push_back(Point3f(j+blockSize/2.0,i+blockSize/2.0,-disparity));
            unsigned char pxL=imgLeft.at<unsigned char>(i,j);
           // pointColors.push_back(pxL);
            hitCounter ++;
            pointVector2d.push_back(Point3i(j,i,sad));
                }
            }
        }
    }
    }
    
}
bool imgPro::checkHorizontalInvariance(Mat& leftImage, Mat& rightImage, Mat& sobelL, Mat& sobelR, int pxX, int pxY)
{
    int startX = pxX;
    int startY = pxY;
    int endX = pxX + blockSize - 1;
    int endY = pxY + blockSize - 1;

    if(startX + zero_disparity + INVARIANCE_CHECK_HORZ_OFFSET_MIN < 0 || endX+zero_disparity+INVARIANCE_CHECK_HORZ_OFFSET_MAX>rightImage.cols)
    {
        return true;
    }
    if(startY+INVARIANCE_CHECK_VERT_OFFSET_MIN<0 || endY+INVARIANCE_CHECK_VERT_OFFSET_MAX>rightImage.rows){
        return true;
        }
    
        int leftVal=0;
        int right_val_array[400];
        int sad_array[400];
        int sobel_array[400];
    
        for(int i=0;i<400;i++){
        right_val_array[i]=0;
        sad_array[i]=0;
        sobel_array[i]=0;
        }
        int counter=0;
        for(int i=startY;i<=endY;i++){
        for(int j=startX;j<=endX;j++){
            unsigned char pxL=leftImage.at<unsigned char>(i,j);
            unsigned char pxR_array[400],sR_array[400];
            counter=0;
            for(int vert_offset=INVARIANCE_CHECK_VERT_OFFSET_MIN;vert_offset<=INVARIANCE_CHECK_VERT_OFFSET_MAX;
    vert_offset+=INVARIANCE_CHECK_VERT_OFFSET_INCREMENT){
            for(int horz_offset=INVARIANCE_CHECK_HORZ_OFFSET_MIN;horz_offset<=INVARIANCE_CHECK_HORZ_OFFSET_MAX;
    horz_offset++){
                pxR_array[counter]=rightImage.at<unsigned char>(i+vert_offset,j+zero_disparity+horz_offset);
                sR_array[counter]=sobelR.at<unsigned char>(i+vert_offset,j+zero_disparity+horz_offset);
                right_val_array[counter]+=sR_array[counter];
                sad_array[counter]+=abs(pxL-pxR_array[counter]);
                counter ++;
            }
            }
            unsigned char sL=sobelL.at<unsigned char>(i,j);
            leftVal += sL;
        }
        }
        for(int i=0;i<counter;i++){
        sobel_array[i]=leftVal+right_val_array[i];
        if(right_val_array[i]>=860&&333*horizontalInvarianceMultiplier*(float)sad_array[i]/((float)sobel_array[i])<sadThreshold){
            return true;
        }
        }
        return false;
}

int imgPro::getSAD(Mat& leftImage, Mat& rightImage, Mat& laplacianL, Mat& laplacianR, int pxX, int pxY)
{
    int startX=pxX;
    int startY=pxY;
    int endX=pxX+blockSize-1; 
    int endY=pxY+blockSize-1;
    
    int leftVal=0,rightVal=0;
    int sad=0;

    for(int i=startY;i<=endY;i++){
	    unsigned char *this_rowL=leftImage.ptr<unsigned char>(i);
        unsigned char *this_rowR=rightImage.ptr<unsigned char>(i);
        unsigned char *this_row_laplacianL=laplacianL.ptr<unsigned char>(i);
        unsigned char *this_row_laplacianR=laplacianR.ptr<unsigned char>(i);
    	    for(int j=startX;j<=endX;j++){
                unsigned char sL=this_row_laplacianL[j];
  		unsigned char sR=this_row_laplacianR[j+disparity];
		
		leftVal+=sL;
 		rightVal+=sR;

 		unsigned char pxL=this_rowL[j];
		unsigned char pxR=this_rowR[j+disparity];

		sad+=abs(pxL-pxR);
            }
    }
    int laplacian_value=leftVal+rightVal;
    if(leftVal<sobelLimit || rightVal<sobelLimit){
  	return -1;
    }
    return 333*(float)sad/(float)laplacian_value;
}

void imgPro::getImgRight(const sensor_msgs::ImageConstPtr& msg)
{
    flag = 2;
    cout << flag<<endl;
    
        // Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    
        ROS_ASSERT(cv_ptr->image.channels()==3 || cv_ptr->image.channels()==1);
    
        if(cv_ptr->image.channels()==3)
        {
           
            cvtColor(cv_ptr->image, imgRight, CV_RGB2GRAY);
         
        }
        else if(cv_ptr->image.channels()==1)
        {
            cv_ptr->image.copyTo(imgRight);
        }
       // imshow("image",imgRight);
     //  image_pub_right.publish(cv_ptr->toImageMsg());
       
        
}
void imgPro::getImgLeft(const sensor_msgs::ImageConstPtr& msg)
{
    flag = 1;    
    cout << 1 <<endl;
    
        // Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    
        ROS_ASSERT(cv_ptr->image.channels()==3 || cv_ptr->image.channels()==1);
    
        if(cv_ptr->image.channels()==3)
        {
            
            cvtColor(cv_ptr->image, imgLeft, CV_RGB2GRAY);
        }
        else if(cv_ptr->image.channels()==1)
        {
            cv_ptr->image.copyTo(imgLeft);
        }
      //  image_pub_left.publish(cv_ptr->toImageMsg());
        
}
void imgPro::LaplacianPro()
{
    
        cv::Laplacian(imgLeft,laplacianLeft,-1,3,1,0,BORDER_DEFAULT);
        cv::Laplacian(imgRight,laplacianRight,-1,3,1,0,BORDER_DEFAULT);   
    
}