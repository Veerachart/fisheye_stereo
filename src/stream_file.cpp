#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>

using namespace cv;
int main(int argc, char **argv){
    ros::init(argc, argv, "file_streamer",ros::init_options::AnonymousName);
    
    ros::start();
    ros::Rate loop_rate(15);
    
    VideoCapture player1;
    VideoCapture player2;
    
    if(!player1.open("/home/otalab/catkin_ws/src/fisheye_stereo/Exp_left.mp4") || !player2.open("/home/otalab/catkin_ws/src/fisheye_stereo/Exp_right.mp4")){
        std::cout << "Error opening video stream 1" << std::endl;
        return -1;
    }
    
    Mat frame1, frame2;
    
    player1.set(CV_CAP_PROP_POS_MSEC, 15930);
    player2.set(CV_CAP_PROP_POS_MSEC, 11570);
    
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher left_pub, right_pub;
    left_pub = it.advertise("/cam_left/raw_video", 1);
    right_pub = it.advertise("/cam_right/raw_video", 1);
    sensor_msgs::ImagePtr msg;
    
    while(ros::ok()) {
        if(!player1.read(frame1) || !player2.read(frame2)){
            break;
        }
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();
        left_pub.publish(msg);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame2).toImageMsg();
        right_pub.publish(msg);
        loop_rate.sleep();
    }
}
