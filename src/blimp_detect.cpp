#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
//#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include <Eigen/Core>
#include <algorithm>

using namespace cv;
using namespace Eigen;
namespace enc = sensor_msgs::image_encodings;

const double PI = 3.141592653589793;

bool compareContourAreas ( vector<Point> contour1, vector<Point> contour2 ) {
    double i = contourArea(Mat(contour1));
    double j = contourArea(Mat(contour2));
    return ( i > j );
}

class BlimpTracker {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    
    geometry_msgs::PointStamped center_;
    
    ros::Publisher center_pub_;
    
    bool is_bg_built;
    Mat background;
    Mat foreground;
    int bg_count;
    int bg_limit;
    
    Mat img_gray;
    Mat temp;
    
    Mat show;
    Mat bg_show;
    Mat contour_show;
    float scale;
    
    public:
        BlimpTracker()
            : it_(nh_){
            //ROS_INFO("Tracker created.");
            std::string camera (nh_.resolveName("camera"), 1, 5);
            image_pub_ = it_.advertise("/cam_"+camera+"/blimp_image", 1);
            image_sub_ = it_.subscribe("/cam_"+camera+"/raw_video", 1, &BlimpTracker::imageCallback, this);
            
            center_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/blimp_center", 1);
            is_bg_built = FALSE;
            bg_count = 0;
            bg_limit = 25;      // 1 s
            scale = 0.5;
        }
        
        void imageCallback (const sensor_msgs::Image::ConstPtr& msg) {
            ros::Time begin = ros::Time::now();
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            cvtColor(cv_ptr->image, img_gray, CV_BGR2GRAY);
            if(!is_bg_built){
                // Background is not yet created
                if(bg_count == 0){
                    // First frame, create background with the same size
                    temp = Mat::zeros(img_gray.rows, img_gray.cols, CV_16U);
                    background = Mat(img_gray.rows, img_gray.cols, CV_8U);
                }
                add(temp, img_gray, temp, noArray(), CV_16U);
                bg_count++;
                ROS_INFO("%d", bg_count);
                if(bg_count >= bg_limit){
                    background = Mat_<unsigned char>(temp/(float)bg_limit);
                    is_bg_built = TRUE;
                }
                return;
            }
            absdiff(background, img_gray, foreground);
            threshold(foreground, foreground, 25, 255, THRESH_BINARY);
            morphologyEx(foreground, foreground, MORPH_OPEN, Mat::ones(3,3,CV_8U), Point(-1,-1), 1);
            morphologyEx(foreground, foreground, MORPH_CLOSE, Mat::ones(3,3,CV_8U), Point(-1,-1), 3);
            morphologyEx(foreground, foreground, MORPH_OPEN, Mat::ones(3,3,CV_8U), Point(-1,-1), 3);
            
            vector<vector<Point> > contours;
            findContours(foreground, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            if(contours.size() > 1){
                std::sort(contours.begin(), contours.end(), compareContourAreas);
                //drawContours(cv_ptr->image, contours, 0, Scalar(128,255,255), 3, CV_AA);
                //drawContours(cv_ptr->image, contours, 1, Scalar(255,0,0), 3, CV_AA);
            }
            else if (contours.size()){
                // Only one
                //drawContours(cv_ptr->image, contours, -1, Scalar(128,255,255), 3, CV_AA);
            }
            //resize(foreground, show, Size(), scale, scale);
            //resize(background, bg_show, Size(), scale, scale);
            //resize(cv_ptr->image, contour_show, Size(), scale, scale);
            //imshow("Background", bg_show);
            //imshow("Foreground", show);
            //imshow("contours", contour_show);
            double loop_time = (ros::Time::now() - begin).toSec();
            ROS_INFO("%.6f", loop_time);
            waitKey(1);
        }
                
};


int main (int argc, char **argv) {
    ros::init(argc, argv, "blimp_tracker", ros::init_options::AnonymousName);
    ros::start();
    BlimpTracker tracker = BlimpTracker();
    ROS_INFO("START");
    ros::spin();
    
    return 0;
}
