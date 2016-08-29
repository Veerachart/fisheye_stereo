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

using namespace cv;
using namespace Eigen;
namespace enc = sensor_msgs::image_encodings;

const double PI = 3.141592653589793;

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
    
    public:
        BlimpTracker()
            : it_(nh_){
            ROS_INFO("Tracker created.");
            image_pub_ = it_.advertise("/cam_left/blimp_image", 1);
            image_sub_ = it_.subscribe("/cam_left/raw_video", 1, &BlimpTracker::imageCallback, this);
            
            center_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/blimp_center", 1);
            is_bg_built = FALSE;
            bg_count = 0;
            bg_limit = 25;      // 1 s
        }
        
        void imageCallback (const sensor_msgs::Image::ConstPtr& msg) {
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
            imshow("Foreground", foreground);
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
