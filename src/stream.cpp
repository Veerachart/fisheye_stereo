#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <Eigen/Core>


using namespace cv;
using namespace Eigen;

const double PI = 3.141592653589793;

class StreamRectifier{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::ImagePtr msg;
    Mat frame;
    std::string videoStreamAddress;
    VideoCapture vcap;
    int rect_size;
    Mat map_x, map_y;
    Mat dst;
    double theta, r, x, y, u, v, s, beta, psi;
    int i, j;
    Matrix3f R;
    Vector3f U, U_cam;
    
public:
    StreamRectifier(int argc, char** argv)
      : it_(nh_)
    {
        std::string camera = nh_.resolveName("camera");
        if(camera == "/left"){
            videoStreamAddress = std::string("rtsp://192.168.11.65/live2.sdp");
            image_pub_ = it_.advertise("/cam_left/rectified_video", 1);
        }
        else if(camera == "/right" ){
            videoStreamAddress = std::string("rtsp://192.168.11.79/live2.sdp");
            image_pub_ = it_.advertise("/cam_right/rectified_video", 1);
        }
        else{
            // Use default: left camera
            ROS_WARN("Wrong assignment to left or right camera --- use configuration for left camera");
            videoStreamAddress = std::string("rtsp://192.168.11.65/live2.sdp");
            image_pub_ = it_.advertise("/cam_left/rectified_video", 1);
        }
        
        cv_ptr = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
        
        rect_size = 720;
        map_x = Mat(rect_size,rect_size,CV_32FC1);
        map_y = Mat(rect_size,rect_size,CV_32FC1);
        dst = Mat(rect_size,rect_size,CV_32FC3);
        R << 0.9995, -0.0257, -0.0160, 0.0257, 0.9997, -0.0002, 0.0160, -0.0002, 0.9999;
        
        for(i = 0; i < rect_size; i++){
            for(j = 0; j < rect_size; j++){
                beta = (j-rect_size/2)/(rect_size/PI);
                psi = (i-rect_size/2)/(rect_size/PI);
                u = sin(psi);
                v = cos(psi)*sin(beta);
                s = cos(psi)*cos(beta);
                U << u, v, s;
                U_cam = R.transpose() * U;
                theta = acos(U_cam(2));
                r = 1.499236*theta + 0.045662*pow(theta,3) - 0.033468*pow(theta,5) + 0.010240*pow(theta,7) - 0.001877*pow(theta,9);
                x = r*U_cam(0)/sin(theta);
                y = r*U_cam(1)/sin(theta);
                map_x.at<float>(j,i) = 321.3305*x + 760.76;
                map_y.at<float>(j,i) = 321.2910*y + 770.55;
		    }
	    }
	    ROS_INFO("START");
	    if(!vcap.open(videoStreamAddress)) {
            std::cout << "Error opening video stream 1" << std::endl;
            //return -1;
        }
        ROS_INFO("Stream");
    }
    
    void stream(){
        ros::Time begin = ros::Time::now();
        if(!vcap.read(frame)) {
            std::cout << "No frame" << std::endl;
            return;
        }
        if( !frame.data )
          { return; }
        
        remap(frame, dst, map_x, map_y, CV_INTER_LINEAR, BORDER_TRANSPARENT, Scalar(0,0,0));
        
        for (int ang = 0; ang < rect_size; ang = ang+rect_size/12){
            line(dst, Point(0,ang), Point(rect_size,ang),CV_RGB(255,255,255));
            line(dst, Point(ang,0), Point(ang,rect_size),CV_RGB(255,255,255));
        }
        
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
        image_pub_.publish(msg);
        double loop_time = (ros::Time::now() - begin).toSec();
        ROS_INFO("%.6f", loop_time);
        //waitKey(1);
    }
    
    ~StreamRectifier()
    {
        vcap.release();
        destroyAllWindows();
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "streamer",ros::init_options::AnonymousName);
    std::string cam_arg = ros::names::remap("camera");
    if (cam_arg == "camera") {
        ROS_WARN("Selection 'camera' has not been remapped! Typical command-line usage:\n"
             "\t$ rosrun fisheye_stereo stream camera:=<left or right>");
        return -1;
    }
    else if ( (cam_arg != "/left") && (cam_arg != "/right") ){
        ROS_WARN("Selection 'camera' has been incorrectly remapped! Typical command-line usage:\n"
             "\t$ rosrun fisheye_stereo stream camera:=<left or right>");
        return -1;
    }
    ros::start();
    ros::Rate loop_rate(25);
    StreamRectifier streamer(argc, argv);

    while(ros::ok()) {
        streamer.stream();
        loop_rate.sleep();

    }
}
