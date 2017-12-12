#include<flight/ImgPro.h>

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

void imgPro::getImgRight(const sensor_msg::ImageConstPtr& msg)
{
    
    
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
           
            cvtColor(cv_ptr->image, img, CV_RGB2GRAY);
         
        }
        else if(cv_ptr->image.channels()==1)
        {
            cv_ptr->image.copyTo(img);
        }
}
void imgPro::getImgLeft(const sensor_msg::ImageConstPtr& msg)
{
    
    
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
            
            cvtColor(cv_ptr->image, img, CV_RGB2GRAY);
        }
        else if(cv_ptr->image.channels()==1)
        {
            cv_ptr->image.copyTo(img);
        }
}