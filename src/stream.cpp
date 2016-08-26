#include <ros/ros.h>
#include <iostream>
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
    double theta, r, x, y, u, v, s, beta, psi, mu, mv, u0, v0;
    double coeffs[5];
    int i, j;
    Matrix3f R;
    Vector3f U, U_cam;
    bool show;
    bool draw_line;
    
public:
    StreamRectifier()
      : it_(nh_)
    {
        std::string camera = nh_.resolveName("camera");
        if(camera == "/right"){
            videoStreamAddress = std::string("rtsp://192.168.11.65/live2.sdp");
            image_pub_ = it_.advertise("/cam_right/rectified_video", 1);
            R << 0.999669, 0.001851, -0.025645, -0.001851, 0.999998, 0.000024, 0.025645, -0.000024, 0.999671;
            coeffs[0] = 1.512327;
            coeffs[1] = 0.017717;
            coeffs[2] = -0.007934;
            coeffs[3] = 0.001223;
            coeffs[4] = -0.001235;
            mu = 321.3991;
            mv = 321.4652;
            u0 = 761.98;
            v0 = 772.98;
        }
        else if(camera == "/left" ){
            videoStreamAddress = std::string("rtsp://192.168.11.79/live2.sdp");
            image_pub_ = it_.advertise("/cam_left/rectified_video", 1);
            R << 0.999482, 0.028267, -0.015379, -0.028406, 0.999557, -0.008864, 0.015122, 0.009297, 0.999842;
            coeffs[0] = 1.499236;
            coeffs[1] = 0.045662;
            coeffs[2] = -0.033468;
            coeffs[3] = 0.010240;
            coeffs[4] = -0.001877;
            mu = 321.3305;
            mv = 321.2910;
            u0 = 760.76;
            v0 = 770.55;
        }
        else{
            // Use default: left camera
            ROS_WARN("Wrong assignment to left or right camera --- use configuration for left camera");
            videoStreamAddress = std::string("rtsp://192.168.11.79/live2.sdp");
            image_pub_ = it_.advertise("/cam_left/rectified_video", 1);
        }
        std::cout << ros::names::remap("show") << std::endl;
        if(ros::names::remap("show") == "/true")
            show = TRUE;
        else
            show = FALSE;
        
        cv_ptr = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
        
        rect_size = 720;
        map_x = Mat(rect_size,rect_size,CV_32FC1);
        map_y = Mat(rect_size,rect_size,CV_32FC1);
        dst = Mat(rect_size,rect_size,CV_32FC3);
        //R << 0.9995, -0.0257, -0.0160, 0.0257, 0.9997, -0.0002, 0.0160, -0.0002, 0.9999;
        
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
                r = coeffs[0]*theta + coeffs[1]*pow(theta,3) + coeffs[2]*pow(theta,5) + coeffs[3]*pow(theta,7) + coeffs[4]*pow(theta,9);
                //r = 1.499236*theta + 0.045662*pow(theta,3) - 0.033468*pow(theta,5) + 0.010240*pow(theta,7) - 0.001877*pow(theta,9);
                x = r*U_cam(0)/sin(theta);
                y = r*U_cam(1)/sin(theta);
                map_x.at<float>(j,i) = mu*x + u0;
                map_y.at<float>(j,i) = mv*y + v0;
		    }
	    }
	    ROS_INFO("START");
	    if(!vcap.open(videoStreamAddress)) {
            std::cout << "Error opening video stream 1" << std::endl;
            //return -1;
        }
        ROS_INFO("Stream");
        
        draw_line = FALSE;
    }
    
    void stream(){
        //ros::Time begin = ros::Time::now();
        if(!vcap.read(frame)) {
            std::cout << "No frame" << std::endl;
            return;
        }
        if( !frame.data )
          { return; }
        
        remap(frame, dst, map_x, map_y, CV_INTER_LINEAR, BORDER_TRANSPARENT, Scalar(0,0,0));
        
        if(draw_line){
            for (int ang = 0; ang < rect_size; ang = ang+rect_size/12){
                line(dst, Point(0,ang), Point(rect_size,ang),CV_RGB(255,255,255));
                line(dst, Point(ang,0), Point(ang,rect_size),CV_RGB(255,255,255));
            }
        }
        
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
        image_pub_.publish(msg);
        //double loop_time = (ros::Time::now() - begin).toSec();
        //ROS_INFO("%.6f", loop_time);
        if(show){
            imshow("Image",dst);
            if(waitKey(1) == 'l'){
                draw_line = !draw_line;
            }
        }
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
    StreamRectifier * streamer = new (std::nothrow) StreamRectifier;

    while(ros::ok()) {
        streamer->stream();
        loop_rate.sleep();

    }
    delete streamer;
}
