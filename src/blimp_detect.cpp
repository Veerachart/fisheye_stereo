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
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
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
    
    ros::Publisher center_pub_;
    
    bool is_bg_built;
    Mat background;
    Mat foreground;
    int bg_count;
    int bg_limit;
    
    Mat img_gray;
    Mat temp;
    
    Mat show;
    Mat contour_show;
    float scale;
    
    double u0, v0;
    
    double area_threshold;
    
    geometry_msgs::PolygonStamped polygon;
    geometry_msgs::Polygon detected_points;
    geometry_msgs::Point32 point;
    
    public:
        BlimpTracker()
            : it_(nh_){
            //ROS_INFO("Tracker created.");
            std::string camera (nh_.resolveName("camera"), 1, 5);
            if(camera == "left"){
                u0 = 761.98;
                v0 = 772.98;
            }
            else if(camera == "right"){
                u0 = 760.76;
                v0 = 770.55;
            }
            image_pub_ = it_.advertise("/cam_"+camera+"/blimp_image", 1);
            image_sub_ = it_.subscribe("/cam_"+camera+"/raw_video", 1, &BlimpTracker::imageCallback, this);
            
            center_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/cam_"+camera+"/blimp_center", 1);
            is_bg_built = FALSE;
            bg_count = 0;
            bg_limit = 25;      // 1 s
            scale = 0.5;
            area_threshold = 2500;
        }
        
        void imageCallback (const sensor_msgs::Image::ConstPtr& msg) {
            ros::Time begin = ros::Time::now();
            detected_points = geometry_msgs::Polygon();
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
            threshold(foreground, foreground, 20, 255, THRESH_BINARY);
            morphologyEx(foreground, foreground, MORPH_OPEN, Mat::ones(3,3,CV_8U), Point(-1,-1), 1);
            morphologyEx(foreground, foreground, MORPH_CLOSE, Mat::ones(5,5,CV_8U), Point(-1,-1), 2);
            morphologyEx(foreground, foreground, MORPH_OPEN, Mat::ones(5,5,CV_8U), Point(-1,-1), 2);
            
            vector<vector<Point> > contours;
            findContours(foreground.clone(), contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            if(contours.size() > 0){
                std::sort(contours.begin(), contours.end(), compareContourAreas);
                //drawContours(cv_ptr->image, contours, 0, Scalar(128,255,255), 3, CV_AA);
                for(int i = 0; i < contours.size(); i++){
                    if(contourArea(contours[i]) < area_threshold){        // Too small contour
                        continue;
                    }
                    RotatedRect rect;
                    rect = minAreaRect(contours[i]);
                    Point2f rect_points[4];
                    rect.points(rect_points);
                    
                    //Find saturation_mean
                    float sat;
                    //ros::Time begin = ros::Time::now();
                    Rect roi = rect.boundingRect();
                    roi.x = max(roi.x, 0);
                    roi.y = max(roi.y, 0);
                    roi.width = min(cv_ptr->image.cols - roi.x, roi.width);
                    roi.height = min(cv_ptr->image.rows - roi.y, roi.height);
                    Mat crop(cv_ptr->image, roi);
                    Mat temp = Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8U);
                    drawContours(temp, contours, i, Scalar(255,255,255), CV_FILLED);
                    Mat mask(temp, roi);
                    Mat hsv;
                    cvtColor(crop, hsv, CV_BGR2HSV);
                    //cvtColor(cv_ptr->image, hsv, CV_BGR2HSV);
                    //double loop_time = (ros::Time::now() - begin).toSec();
                    //ROS_INFO("%.6f", loop_time);
                    sat = mean(hsv, mask)[1];
                    //std::cout << sat << std::endl;
                    char text[50];
                    sprintf(text, "%.2f", sat);
                    
                    double area = contourArea(Mat(contours[i]));
                    double rectArea = rect.size.width*rect.size.height;
                    
                    if(rect.size.width/rect.size.height >= 0.85 && rect.size.width/rect.size.height <= 1.15){
                        // Almost a circle --> close to the center of the image
                        // --> fitted rectangle will probably have wrong angle
                        if(sat <= 60 && area/rectArea > 0.5 && area/rectArea < 0.85){
                            // Blimp has low saturation in HSV space and area = pi/4 = 0.785
                            drawContours(cv_ptr->image, contours, i, Scalar(0,0,255), 3, CV_AA);        // Draw in red
                            for(int j = 0; j < 4; j++)
                                line( cv_ptr->image, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255),2,8);
                            //putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 2, Scalar(0,0,255),2);
                            point.x = rect.center.x;
                            point.y = rect.center.y;
                            detected_points.points.push_back(point);
                        }
                        else{
                            drawContours(cv_ptr->image, contours, i, Scalar(255,255,255), 1, CV_AA);        // Draw in white
                            for(int j = 0; j < 4; j++)
                                line( cv_ptr->image, rect_points[j], rect_points[(j+1)%4], Scalar(255,255,255),1,8);
                            //putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 2, Scalar(255,255,255),2);
                        }
                    }
                    else{
                        float angle;
                        if(rect.size.width < rect.size.height)
                            angle = abs(atan2(rect.center.y-v0, rect.center.x-u0)*180.0/PI - 90.0 - rect.angle);
                        else
                            angle = abs(atan2(rect.center.y-v0, rect.center.x-u0)*180.0/PI - rect.angle);
                        if(abs(angle - 90) < 20){
                            // Considered as blimp
                            // Check area: Ellipse = pi*a*b, rectangle = (2a)*(2b)
                            // Therefore Ellipse/rectangle = pi/4 --> 0.5-0.85
                            if(area/rectArea < 0.5 || area/rectArea > 0.85 || sat > 60){
                                // May not be the blimp
                                //ROS_INFO("%.2f", area/rectArea);
                                drawContours(cv_ptr->image, contours, i, Scalar(255,255,255), 1, CV_AA);        // Draw in white
                                for(int j = 0; j < 4; j++)
                                    line( cv_ptr->image, rect_points[j], rect_points[(j+1)%4], Scalar(255,255,255),1,8);
                                //putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 2, Scalar(255,255,255),2);
                            }
                            else{
                                drawContours(cv_ptr->image, contours, i, Scalar(0,0,255), 3, CV_AA);        // Draw in red
                                for(int j = 0; j < 4; j++)
                                    line( cv_ptr->image, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255),2,8);
                                //putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 2, Scalar(0,0,255),2);
                                point.x = rect.center.x;
                                point.y = rect.center.y;
                                detected_points.points.push_back(point);
                            }
                        }
                        else if (angle < 20){
                            // Considered as standing human
                            drawContours(cv_ptr->image, contours, i, Scalar(0,255,0), 3, CV_AA);        // Draw in green
                            for(int j = 0; j < 4; j++)
                                line( cv_ptr->image, rect_points[j], rect_points[(j+1)%4], Scalar(0,255,0),2,8);
                            //putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 2, Scalar(0,255,0),2);
                        }
                        else{
                            // Unclassified // TODO
                            drawContours(cv_ptr->image, contours, i, Scalar(255,255,255), 1, CV_AA);        // Draw in white
                            for(int j = 0; j < 4; j++)
                                line( cv_ptr->image, rect_points[j], rect_points[(j+1)%4], Scalar(255,255,255),1,8);
                            //putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 2, Scalar(255,255,255),2);
                        }
                    }
                }
            }
            //resize(foreground, show, Size(), scale, scale);
            //resize(cv_ptr->image, contour_show, Size(), scale, scale);
            //imshow("Foreground", show);
            //imshow("contours", contour_show);
            image_pub_.publish(cv_ptr->toImageMsg());
            polygon.header.stamp = ros::Time::now();
            polygon.polygon = detected_points;
            center_pub_.publish(polygon);
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
