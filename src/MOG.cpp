#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
//#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <time.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

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
    ros::Publisher human_pub_;
    
    Mat fgMaskMOG2;
    BackgroundSubtractorMOG2 pMOG2;
    
    Mat img_gray;
    Mat img_hsv;
    Mat temp;
    Mat img_thresholded_b, img_thresholded;
    
    Mat show;
    Mat contour_show;
    float scale;
    
    double u0, v0;
    
    double area_threshold;
    
    geometry_msgs::PolygonStamped polygon;
    geometry_msgs::Polygon detected_points;
    geometry_msgs::Point32 point;
    geometry_msgs::PointStamped point_msg;
    
    std::ofstream *logfile;
    double t_zero;
    
    double f1,f2,f3;
    
    // Blue
    int iLowH_1;
    int iHighH_1;

    int iLowS_1;
    int iHighS_1;

    int iLowV_1;
    int iHighV_1;
    
    // Skin
    int iLowH_skin;
    int iHighH_skin;

    int iLowS_skin;
    int iHighS_skin;

    int iLowV_skin;
    int iHighV_skin;
    
    int dilation_size;
    
    VideoWriter outputVideo;
    bool save_video;
    
    public:
        BlimpTracker()
            : it_(nh_){
            //ROS_INFO("Tracker created.");
            std::string camera (nh_.resolveName("camera"), 1, 5);
            if (nh_.resolveName("save_video") == "/true")
                save_video = true;
            if(camera == "left"){
                u0 = 385.21;
                v0 = 385.24;
            }
            else if(camera == "right"){
                u0 = 385.14;
                v0 = 385.32;
            }
            image_pub_ = it_.advertise("/cam_"+camera+"/detection_image", 1);
            image_sub_ = it_.subscribe("/cam_"+camera+"/raw_video", 1, &BlimpTracker::imageCallback, this);
            
            center_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/cam_"+camera+"/blimp_center", 1);
            human_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/cam_"+camera+"/human_center", 1);
            
            area_threshold = 300;
            
            time_t now = time(0);
            struct tm* timeinfo;
            timeinfo = localtime(&now);
            char buffer[80];
            char videoName[80];
            if (camera == "left") {
                strftime(buffer,80,"/home/otalab/logdata/%Y%m%d-%H%M_left.csv", timeinfo);
                strftime(videoName,80,"/home/otalab/logdata/%Y%m%d-%H%M_left.avi", timeinfo);
            }
            else if (camera == "right") {
                strftime(buffer,80,"/home/otalab/logdata/%Y%m%d-%H%M_right.csv", timeinfo);
                strftime(videoName,80,"/home/otalab/logdata/%Y%m%d-%H%M_right.avi", timeinfo);
            }
            //logfile = new std::ofstream(buffer);
            //*logfile << "time,f1,f2,f3,ftot\n";
            
            // HSV color detect
            // Control window for adjusting threshold values
            iLowH_1 = 100;
            iHighH_1 = 110;

            iLowS_1 = 100;
            iHighS_1 = 255;

            iLowV_1 = 60;
            iHighV_1 = 240;
            
            iLowH_skin = 2;
            iHighH_skin = 20;

            iLowS_skin = 40;
            iHighS_skin = 230;

            iLowV_skin = 30;
            iHighV_skin = 200;
            
            namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
            
            cvCreateTrackbar("LowH1", "Control", &iLowH_1, 179); //Hue (0 - 179)
            cvCreateTrackbar("HighH1", "Control", &iHighH_1, 179);

            cvCreateTrackbar("LowS1", "Control", &iLowS_1, 255); //Saturation (0 - 255)
            cvCreateTrackbar("HighS1", "Control", &iHighS_1, 255);

            cvCreateTrackbar("LowV1", "Control", &iLowV_1, 255); //Value (0 - 255)
            cvCreateTrackbar("HighV1", "Control", &iHighV_1, 255);
            
            if (save_video) {
                outputVideo.open(videoName, CV_FOURCC('D','I','V','X'), 10, Size(768, 768), true);
                if (!outputVideo.isOpened()) {
                    ROS_ERROR("Could not write video.");
                    return;
                }
            }
            
            pMOG2 = BackgroundSubtractorMOG2(1000, 16, true);
        }
            
        void imageCallback (const sensor_msgs::Image::ConstPtr& msg) {
            double begin = ros::Time::now().toSec();
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
            Mat original_img;
            cv_ptr->image.copyTo(original_img);
            cvtColor(cv_ptr->image, img_hsv, CV_BGR2HSV);
            cvtColor(cv_ptr->image, img_gray, CV_BGR2GRAY);
            
            inRange(img_hsv, Scalar(iLowH_1, iLowS_1, iLowV_1), Scalar(iHighH_1, iHighS_1, iHighV_1), img_thresholded_b); //Threshold the image, Blue
            morphologyEx(img_thresholded_b, img_thresholded_b, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
            morphologyEx(img_thresholded_b, img_thresholded_b, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
            
            /*Mat frame;
            cv_ptr->image.copyTo(frame, img_thresholded_b);
            imshow("Frame", frame);*/
            
            vector<vector<Point> > contours_blimp;
            vector<Vec4i> hierarchy;
            findContours(img_thresholded_b, contours_blimp, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0,0));
            
            //vector<Point2f> centers;
            Point2f blimp_center(0,0);
            double area_max = 0;
            int blimp_contour_idx = -1;
            
            for( int i = 0; i< contours_blimp.size(); i++ )
            {
                double area = contourArea(contours_blimp[i]);
                Moments mu = moments(contours_blimp[i], true);
                if (area < 100)
                    continue;           // Too small
                if (area > area_max){
                    blimp_center.x = mu.m10/mu.m00;
                    blimp_center.y = mu.m01/mu.m00;
                    area_max = area;
                    blimp_contour_idx = i;
                }
                circle(cv_ptr->image, Point(mu.m10/mu.m00, mu.m01/mu.m00), 4, Scalar(0,255,0));
            }
            
            if (area_max > 0)
            {
                circle(cv_ptr->image, blimp_center, 6, Scalar(0,0,255));
                
                point_msg.header.stamp = ros::Time::now();
                point_msg.point.x = blimp_center.x;
                point_msg.point.y = blimp_center.y;
                center_pub_.publish(point_msg);
            }
            
            double start = ros::Time::now().toSec();
            pMOG2(img_gray, fgMaskMOG2);
            double stop = ros::Time::now().toSec();
            ROS_INFO("%.6f", stop-start);
            threshold(fgMaskMOG2, fgMaskMOG2, 128, 255, THRESH_BINARY);
                        
            morphologyEx(fgMaskMOG2, fgMaskMOG2, MORPH_OPEN, Mat::ones(3,3,CV_8U), Point(-1,-1), 1);
            morphologyEx(fgMaskMOG2, fgMaskMOG2, MORPH_CLOSE, Mat::ones(5,5,CV_8U), Point(-1,-1), 2);
            morphologyEx(fgMaskMOG2, fgMaskMOG2, MORPH_OPEN, Mat::ones(5,5,CV_8U), Point(-1,-1), 1);
            //imshow("FG Mask MOG 2", fgMaskMOG2);
            vector<vector<Point> > contours_foreground;
            findContours(fgMaskMOG2.clone(), contours_foreground, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            Mat human_show;
            Mat mask = Mat::zeros(img_gray.rows, img_gray.cols, CV_8U);
            if(contours_foreground.size() > 0){
                //std::sort(contours.begin(), contours.end(), compareContourAreas);
                //drawContours(cv_ptr->image, contours, 0, Scalar(128,255,255), 3, CV_AA);
                for(int i = 0; i < contours_foreground.size(); i++){
                    if(contourArea(contours_foreground[i]) < area_threshold){        // Too small contour
                        continue;
                    }
                    RotatedRect rect;
                    rect = minAreaRect(contours_foreground[i]);
                    Point2f rect_points[4];
                    rect.points(rect_points);
                    for(int j = 0; j < 4; j++)
                        line( cv_ptr->image, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255),2,8);
                    
                    char text[50];
                    double area = contourArea(contours_foreground[i]);
                    //double rectArea = rect.size.width*rect.size.height;
                    
                    //double membershipValue;
                    
                    // Intersection between blimp's contour and human's contour
                    // To avoid including blimp as human
                    Mat intersect = Mat::zeros(img_gray.rows, img_gray.cols, CV_8U);
                    Mat blimp_mask = Mat::zeros(img_gray.rows, img_gray.cols, CV_8U);
                    Mat foreground_mask = Mat::zeros(img_gray.rows, img_gray.cols, CV_8U);
                    if (blimp_contour_idx >= 0)
                        drawContours(blimp_mask, contours_blimp, blimp_contour_idx, Scalar(255), CV_FILLED);
                    drawContours(foreground_mask, contours_foreground, i, Scalar(255), CV_FILLED);
                    intersect = blimp_mask & foreground_mask;
                    vector<vector<Point> > contours_intersect;
                    findContours(intersect.clone(), contours_intersect, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
                    double intersect_area = 0;
                    if (contours_intersect.size()) {
                        for (int j = 0; j < contours_intersect.size(); j++) {
                            intersect_area += contourArea(contours_intersect[j]);
                        }
                    }
                    //morphologyEx(foreground_mask, foreground_mask, MORPH_DILATE, Mat::ones(3,3,CV_8U), Point(-1,-1), 1);
                    
                    if (intersect_area < 0.4*area) {
                        // The overlap of the foreground blob is less than 40% of the blimp (now arbitrary # TODO get better number
                        //Moments m = moments(contours_foreground[i]);
                        if (fabs(rect.center.x - u0) + fabs(rect.center.y - v0) < 20) {
                            // Around the center, the orientation of the ellipse can be in any direction, depending on the direction the person is looking to
                            // TODO
                            point.x = rect.center.x;
                            point.y = rect.center.y;
                            point.z = 0.f;
                            //point.z = (float)membershipValue;
                            detected_points.points.push_back(point);
                        }
                        else {
                            double angle, diff_angle, azimuth_angle, height, width;
                            azimuth_angle = atan((rect.center.y-v0)/(rect.center.x-u0))*180.0/PI;
                            
                            if(rect.size.width < rect.size.height) {
                                //angle = acos(fabs(((rect.center.x-u0)*cos((rect.angle-90.0)*PI/180.0) + (rect.center.y-v0)*sin((rect.angle-90.0)*PI/180.0))/sqrt(std::pow(rect.center.x-u0,2) + std::pow(rect.center.y-v0,2)))) * 180.0/PI;
                                angle = rect.angle;
                                if (angle < 0.0)
                                    angle += 90.0;
                                else
                                    angle -= 90.0;
                                height = rect.size.height;
                                width = rect.size.width;
                            }
                            else {
                                //angle = acos(fabs(((rect.center.x-u0)*cos(rect.angle*PI/180.0) + (rect.center.y-v0)*sin(rect.angle*PI/180.0))/sqrt(std::pow(rect.center.x-u0,2) + std::pow(rect.center.y-v0,2)))) * 180.0/PI;
                                angle = rect.angle;
                                height = rect.size.width;
                                width = rect.size.height;
                            }
                            diff_angle = angle - azimuth_angle;
                            if (diff_angle > 150.0)
                                diff_angle -= 180.0;
                            else if (diff_angle < -150.0)
                                diff_angle += 180.0;
                            
                            // Writing on image for debug
                            /*sprintf(text, "%.2lf", diff_angle);
                            putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,0),2);
                            sprintf(text, "%.2lf %.2lf", rect.angle, rect.size.width/rect.size.height);
                            putText(cv_ptr->image, text, Point(rect.center.x, rect.center.y+30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,255),2);
                            sprintf(text, "%.2lf", atan((rect.center.y-v0)/(rect.center.x-u0))*180.0/PI);
                            putText(cv_ptr->image, text, Point(rect.center.x, rect.center.y+60), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(127,255,0),2);*/
                            //
                                
                            if (fabs(diff_angle) < 30.0) {
                                // orientation less than 15 degree from the radial direction -- supposed to be human
                                Point2f head_center = Point(rect.center.x + 3.*height/8.*sgn(rect.center.x-u0)*cos(fabs(angle)*PI/180.), rect.center.y + 3.*height/8.*sgn(rect.center.y-v0)*sin(fabs(angle)*PI/180.));
                                RotatedRect ROI(head_center, Size(height/4., 3.*width/4.), angle);
                                //Point2f rect_points[4];
                                ROI.points(rect_points);
                                Point points[4];
                                for(int j = 0; j < 4; j++) {
                                    line( cv_ptr->image, rect_points[j], rect_points[(j+1)%4], Scalar(255,255,255),2,8);
                                    points[j] = rect_points[j];
                                }
                                Mat temp_mask = Mat::zeros(img_gray.rows, img_gray.cols, CV_8U);
                                Rect ROI_rect = ROI.boundingRect();
                                Rect head_matrix_bound(Point(std::max(0,ROI_rect.x), max(0,ROI_rect.y)), Point(std::min(img_gray.cols, ROI_rect.x+ROI_rect.width), std::min(img_gray.cols, ROI_rect.y+ROI_rect.height)));
                                //rectangle(temp_mask, head_matrix_bound, Scalar(255), -1);
                                fillConvexPoly(temp_mask, points, 4, Scalar(255));
                                rectangle(cv_ptr->image, head_matrix_bound, Scalar(255), 1);
                                temp_mask = temp_mask & foreground_mask;
                                float head_area = sum(temp_mask)[0]/255.0;
                                Mat temp_head(original_img, head_matrix_bound);
                                Mat temp_head_hsv;
                                img_hsv.copyTo(temp_head_hsv, temp_mask);
                                Mat head_hsv(temp_head_hsv, head_matrix_bound);
                                //img_hsv.copyTo(temp_head_hsv, temp_mask);
                                Mat face_mat;//, hair_mat;
                                drawContours(cv_ptr->image, contours_foreground, i, Scalar(0,255,0), 2, CV_AA);        // Draw in green
                                inRange(head_hsv, Scalar(iLowH_skin, iLowS_skin, iLowV_skin), Scalar(iHighH_skin, iHighS_skin, iHighV_skin), face_mat); //Threshold the image, skin
                                //inRange(head_hsv, Scalar(0, 2, 2), Scalar(180, 180, 80), hair_mat); //Threshold the image, hair
                                morphologyEx(face_mat, face_mat, MORPH_CLOSE, Mat::ones(3,3,CV_8U), Point(-1,-1), 1);
                                //morphologyEx(hair_mat, hair_mat, MORPH_CLOSE, Mat::ones(3,3,CV_8U), Point(-1,-1), 1);
                                
                                //Point2f face_center;//, hair_center;
                                bool face_found = false;
                                double face_area;//, hair_area;
                                
                                vector<vector<Point> > contours;
                                findContours(face_mat.clone(), contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
                                if (contours.size() > 0) {
                                    std::sort(contours.begin(), contours.end(), compareContourAreas);
                                    //Moments mu = moments(contours[0], true);
                                    //face_center = Point(mu.m10/mu.m00, mu.m01/mu.m00);
                                    //circle(cv_ptr->image, face_center+Point2f(ROI.boundingRect().x,ROI.boundingRect().y), 4, Scalar(255,255,255));
                                    
                                    face_area = contourArea(contours[0]);
                                    //sprintf(text, "%.4f, %.4f", face_area, head_area);
                                    //putText(cv_ptr->image, text, head_center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255) ,2);
                                    /*Mat face_show;
                                    temp_head.copyTo(face_show, face_mat);
                                    imshow("Face", face_show);*/
                                    if (face_area >= 0.4*head_area) {
                                        // Face is large enough -- half of the head
                                        face_found = true;
                                    }
                                }
                                
                                /*findContours(hair_mat.clone(), contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
                                if (contours.size() > 0) {
                                    std::sort(contours.begin(), contours.end(), compareContourAreas);
                                    Moments mu = moments(contours[0], true);
                                    hair_center = Point(mu.m10/mu.m00, mu.m01/mu.m00);
                                    //circle(cv_ptr->image, hair_center+Point2f(ROI.boundingRect().x,ROI.boundingRect().y), 4, Scalar(255,255,255));
                                    hair_area = contourArea(contours[0]);
                                }*/
                                
                                /*if (face_found) {
                                    Point2f head_vect = head_center - Point2f(u0, v0);
                                    Point2f face_vect = hair_center - face_center;
                                    double cross_product = head_vect.x*face_vect.y - face_vect.x*head_vect.y;
                                    if (face_area >= 0.8*hair_area) {
                                        sprintf(text, "%d", 0);
                                        putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255),2);
                                    }
                                    else if (face_area >= 0.3*hair_area) {
                                        if (cross_product<0) {
                                            // looking left, (CCW) +ve
                                            sprintf(text, "%d", 45);
                                            putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255),2);
                                        }
                                        else {
                                            // looking right, (CW) -ve
                                            sprintf(text, "-%d", 45);
                                            putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,255),2);
                                        }
                                    }
                                    else if (face_area >= 0.1*hair_area) {
                                        if (cross_product<0) {
                                            // looking left, (CCW) +ve
                                            sprintf(text, "%d", 90);
                                            putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255),2);
                                        }
                                        else {
                                            // looking right, (CW) -ve
                                            sprintf(text, "-%d", 90);
                                            putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,255),2);
                                        }
                                    }
                                    else {
                                        sprintf(text, ">%d", 90);
                                        putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255),2);
                                    }
                                }
                                else {
                                    // TODO
                                    sprintf(text, ">%d", 90);
                                    putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255),2);
                                }
                                */
                                //Mat face_show;
                                //temp_head.copyTo(face_show, face_mat);
                                //imshow("Face", head_hsv);
                                
                                
                                //rectangle(temp_head, ROI.boundingRect(), Scalar(255), -1);
                                //drawContours(temp_mask, contours_foreground, i, Scalar(255), CV_FILLED);
                                //temp_mask = temp_mask & temp_head;
                                //mask = mask | temp_mask;
                                point.x = head_center.x;
                                point.y = head_center.y;
                                point.z = (float) 1*face_found;
                                //point.z = (float)membershipValue;
                                detected_points.points.push_back(point);
                            }
                            else {
                                drawContours(cv_ptr->image, contours_foreground, i, Scalar(0,0,255), 1, CV_AA);        // Draw in red
                                sprintf(text, "%.2lf", diff_angle);
                                putText(cv_ptr->image, text, rect.center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,0),2);
                                sprintf(text, "%.2lf %.2lf", rect.angle, rect.size.width/rect.size.height);
                                putText(cv_ptr->image, text, Point(rect.center.x, rect.center.y+30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,255),2);
                                sprintf(text, "%.2lf", azimuth_angle);
                                putText(cv_ptr->image, text, Point(rect.center.x, rect.center.y+60), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(127,255,0),2);
                            }
                        }
                        
                    }
                    else {
                        // Supposed to be blimp. Draw for debug
                        drawContours(cv_ptr->image, contours_foreground, i, Scalar(255,0,0), 2, CV_AA);        // Draw in blue
                    }
                }
            }
            
            if (save_video)
                outputVideo << cv_ptr->image;
            image_pub_.publish(cv_ptr->toImageMsg());
            polygon.header.stamp = ros::Time::now();
            polygon.polygon = detected_points;
            human_pub_.publish(polygon);
            double end = ros::Time::now().toSec();
            //waitKey(1);
            //std::cout << end-begin << std::endl;
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
