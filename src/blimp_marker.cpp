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
#include "geometry_msgs/PointStamped.h"
#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <time.h>

using namespace cv;
using namespace Eigen;
namespace enc = sensor_msgs::image_encodings;

const double PI = 3.141592653589793;

int dist (Point& a, Point& b){
    return pow(a.x - b.x,2) + pow(a.y - b.y,2);
};

class BlimpMarker {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    
    ros::Publisher center_pub_;
    
    Mat imgHSV;
    Mat imgThresholded_y, imgThresholded_b, imgThresholded;
    double u0, v0;
    
    geometry_msgs::PointStamped point_msg;
    
    std::ofstream *logfile;
    //double t_zero;
    
    // Blue
    int iLowH_1;
    int iHighH_1;

    int iLowS_1;
    int iHighS_1;

    int iLowV_1;
    int iHighV_1;
    
    // Yellow    
    int iLowH_2;
    int iHighH_2;

    int iLowS_2; 
    int iHighS_2;

    int iLowV_2;
    int iHighV_2;
    
    int dilation_size;
    
    public:
        BlimpMarker()
            : it_(nh_){
            //ROS_INFO("Tracker created.");
            std::string camera (nh_.resolveName("camera"), 1, 5);
            if(camera == "left"){
                u0 = 761.98/2.;
                v0 = 772.98/2.;
            }
            else if(camera == "right"){
                u0 = 760.76/2.;
                v0 = 770.55/2.;
            }
            image_pub_ = it_.advertise("/cam_"+camera+"/blimp_image", 1);
            image_sub_ = it_.subscribe("/cam_"+camera+"/raw_video", 1, &BlimpMarker::imageCallback, this);
            
            center_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/cam_"+camera+"/blimp_center", 1);
            
            iLowH_1 = 90;
            iHighH_1 = 110;

            iLowS_1 = 100;
            iHighS_1 = 255;

            iLowV_1 = 60;
            iHighV_1 = 240;
 
            //iLowH_2 = 22;
            //iHighH_2 = 34;

            //iLowS_2 = 80; 
            //iHighS_2 = 200;

            //iLowV_2 = 75;
            //iHighV_2 = 255;
            
            //dilation_size = 21;
            
            namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
            
            cvCreateTrackbar("LowH1", "Control", &iLowH_1, 179); //Hue (0 - 179)
            cvCreateTrackbar("HighH1", "Control", &iHighH_1, 179);

            cvCreateTrackbar("LowS1", "Control", &iLowS_1, 255); //Saturation (0 - 255)
            cvCreateTrackbar("HighS1", "Control", &iHighS_1, 255);

            cvCreateTrackbar("LowV1", "Control", &iLowV_1, 255); //Value (0 - 255)
            cvCreateTrackbar("HighV1", "Control", &iHighV_1, 255);
             
            //cvCreateTrackbar("LowH2", "Control", &iLowH_2, 179); //Hue (0 - 179)
            //cvCreateTrackbar("HighH2", "Control", &iHighH_2, 179);

            //cvCreateTrackbar("LowS2", "Control", &iLowS_2, 255); //Saturation (0 - 255)
            //cvCreateTrackbar("HighS2", "Control", &iHighS_2, 255);

            //cvCreateTrackbar("LowV2", "Control", &iLowV_2, 255); //Value (0 - 255)
            //cvCreateTrackbar("HighV2", "Control", &iHighV_2, 255);
            
            //cvCreateTrackbar("Dilation size", "Control", &dilation_size, 50);
            //time_t now = time(0);
            //struct tm* timeinfo;
            //timeinfo = localtime(&now);
            //char buffer[80];
            //char videoName[80];
            //if (camera == "left") {
            //    strftime(buffer,80,"/home/otalab/logdata/%Y%m%d-%H%M_left.csv", timeinfo);
            //    strftime(videoName,80,"/home/otalab/logdata/%Y%m%d-%H%M_left.avi", timeinfo);
            //}
            //else if (camera == "right") {
            //    strftime(buffer,80,"/home/otalab/logdata/%Y%m%d-%H%M_right.csv", timeinfo);
            //    strftime(videoName,80,"/home/otalab/logdata/%Y%m%d-%H%M_right.avi", timeinfo);
            //}
            //logfile = new std::ofstream(buffer);
            //t_zero = ros::Time::now().toSec();
            //t_zero = (int)(floor(t_zero/100.0)) * 100.0;
        }
        
        void imageCallback (const sensor_msgs::Image::ConstPtr& msg) {
            double begin = ros::Time::now().toSec();
            //detected_points = geometry_msgs::Polygon();
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
            cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
            //vector<Mat> hsv_planes;
            //split( imgHSV, hsv_planes );
            //imshow("H", hsv_planes[0]);
            //imshow("S", hsv_planes[1]);
            //imshow("V", hsv_planes[2]);
            //double t1 = ros::Time::now().toSec();
            //std::cout << t1-begin << std::endl;
            
            inRange(imgHSV, Scalar(iLowH_1, iLowS_1, iLowV_1), Scalar(iHighH_1, iHighS_1, iHighV_1), imgThresholded_b); //Threshold the image, Blue
            //inRange(imgHSV, Scalar(iLowH_2, iLowS_2, iLowV_2), Scalar(iHighH_2, iHighS_2, iHighV_2), imgThresholded_y); //Threshold the image, Yellow
            
            
            morphologyEx(imgThresholded_b, imgThresholded_b, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
            morphologyEx(imgThresholded_b, imgThresholded_b, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

            //morphologyEx(imgThresholded_y, imgThresholded_y, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
            //morphologyEx(imgThresholded_y, imgThresholded_y, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
            
            Mat temp_b, temp_y;
            //cv_ptr->image.copyTo(temp_y, imgThresholded_y);
            cv_ptr->image.copyTo(temp_b, imgThresholded_b);
            //imshow("Yellow", temp_y);
            imshow("Blue", temp_b);
            
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            //Mat dilation_b, dilation_y, dilation_result;
            //morphologyEx(imgThresholded_b, dilation_b, MORPH_DILATE, getStructuringElement(MORPH_ELLIPSE, Size(21, 21)));
            //morphologyEx(imgThresholded_y, dilation_y, MORPH_DILATE, getStructuringElement(MORPH_ELLIPSE, Size(21, 21)));
            //bitwise_and(dilation_y, dilation_b, dilation_result);
            findContours(imgThresholded_b, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0,0));
            
            //vector<Point2f> centers;
            Point2f blimp_center(0,0);
            double area_max = 0;
            
            for( int i = 0; i< contours.size(); i++ )
            {
                double area = contourArea(contours[i]);
                Moments mu = moments(contours[i], true);
                if (area < 40)
                    continue;           // Too small
                if (area > area_max){
                    blimp_center.x = mu.m10/mu.m00;
                    blimp_center.y = mu.m01/mu.m00;
                    area_max = area;
                }
                circle(cv_ptr->image, Point(mu.m10/mu.m00, mu.m01/mu.m00), 4, Scalar(0,255,0));
            }
            
            //for (int i = 0; i < centers.size(); i++)
            //{
            //    blimp_center += centers[i];
            //    circle(cv_ptr->image, centers[i], 4, Scalar(0,255,0));
            //}
            
            //if (centers.size())
            //{
            //    double divisor = 1.f/(double)centers.size();
            //    blimp_center = divisor*blimp_center;
            //    circle(cv_ptr->image, blimp_center, 6, Scalar(0,0,255));
            //}
            if (area_max > 0)
                circle(cv_ptr->image, blimp_center, 6, Scalar(0,0,255));
            
            image_pub_.publish(cv_ptr->toImageMsg());
            
            point_msg.header.stamp = ros::Time::now();
            point_msg.point.x = blimp_center.x;
            point_msg.point.y = blimp_center.y;
            center_pub_.publish(point_msg);
            double loop_time = ros::Time::now().toSec() - begin;
            ROS_INFO("%.6f", loop_time);
            waitKey(1);
        }
};

int main (int argc, char **argv) {
    ros::init(argc, argv, "blimp_marker", ros::init_options::AnonymousName);
    ros::start();
    BlimpMarker blimp = BlimpMarker();
    ROS_INFO("START");
    ros::spin();
    
    return 0;
}
