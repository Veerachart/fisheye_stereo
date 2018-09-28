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
#include <math.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>


using namespace cv;
using namespace Eigen;

const double PI = 3.141592653589793;

class StreamRectifier{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_, raw_pub_;
    //cv_bridge::CvImagePtr cv_ptr;
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
        if(camera == "/left"){
            //videoStreamAddress = std::string("rtsp://192.168.100.5/live3.sdp");
            videoStreamAddress = std::string("http://192.168.100.5/video4.mjpg");
            image_pub_ = it_.advertise("/cam_left/rectified_video", 1);
            raw_pub_ = it_.advertise("/cam_left/raw_video", 1);
            //R << 0.999228,  0.035960, -0.015825,
            //    -0.035960,  0.999353,  0.000285,
            //     0.015825,  0.000285,  0.999875;
            R << 0.999968,  0.007896, -0.000898,
                -0.007896,  0.999968,  0.000004,
                 0.000898,  0.000004,  1.000000;
            //R << 0.999601,  0.027689,  -0.004448, -0.027689,  0.999617, 0.000062, 0.004448,  0.000062,  0.999990;
            //R << 0.999992,  -0.003900, 0.000638, 0.003900,  0.999992, -0.000001,  -0.000638,  0.000001,  1.000000;
            //R << 0.998937, 0.035785, -0.030509, -0.035785, 0.999359, 0.000546, 0.030509, 0.000546, 0.999534;
            coeffs[0] = 1.492357;
            coeffs[1] = 0.013237;
            coeffs[2] = 0.007671;
            coeffs[3] = 0.001029;
            coeffs[4] = -0.003125;
            mu = 157.1979;
            mv = 157.1979;
            u0 = 385.21;
            v0 = 385.21;
        }
        else if(camera == "/right" ){
            //videoStreamAddress = std::string("rtsp://192.168.100.6/live3.sdp");
            videoStreamAddress = std::string("http://192.168.100.6/video4.mjpg");
            image_pub_ = it_.advertise("/cam_right/rectified_video", 1);
            raw_pub_ = it_.advertise("/cam_right/raw_video", 1);
            R << 0.998947, -0.041556,  0.019463,
                 0.041630,  0.999127,  -0.003437,
                -0.019303,  0.004244,  0.999805;
            //R <<  0.996727, -0.075889,  0.027869,
            //      0.075839,  0.997116,  0.002871,
            //     -0.028007, -0.000748,  0.999607;
            //R << 0.998324,  0.023180,  0.053027, -0.024059,  0.999583, 0.015998, -0.052634,  -0.017247,  0.998465;
            //R << 0.997059,  -0.075636, 0.012355, 0.075567,  0.997123, 0.005980,  -0.012772,  -0.005029,  0.999906;
            //R << 0.999744, 0.019123, -0.012076, -0.019195, 0.999798, -0.005883, 0.011961, 0.006114, 0.999910;
            coeffs[0] = 1.495884;
            coeffs[1] = 0.017052;
            coeffs[2] = 0.010611;
            coeffs[3] = 0.002062;
            coeffs[4] = -0.003934;
            mu = 156.4238;
            mv = 156.4208;
            u0 = 385.14;
            v0 = 385.32;
        }
        else{
            // Use default: left camera
            ROS_WARN("Wrong assignment to left or right camera --- use configuration for left camera");
            //videoStreamAddress = std::string("rtsp://192.168.11.85/live2.sdp");
            videoStreamAddress = std::string("/home/otalab/Videos/fps_test.mp4");
            image_pub_ = it_.advertise("/cam_left/rectified_video", 1);
            raw_pub_ = it_.advertise("/cam_left/raw_video", 1);
        }
        std::cout << ros::names::remap("show") << std::endl;
        if(ros::names::remap("show") == "/true")
            show = TRUE;
        else
            show = FALSE;
        
        //cv_ptr = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
        
        rect_size = 720;
        map_x = Mat(rect_size,rect_size,CV_32FC1);
        map_y = Mat(rect_size,rect_size,CV_32FC1);
        dst = Mat(rect_size,rect_size,CV_32FC3);
        //R << 0.9995, -0.0257, -0.0160, 0.0257, 0.9997, -0.0002, 0.0160, -0.0002, 0.9999;
        
        /*for(i = 0; i < rect_size; i++){
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
	    }*/
	    ROS_INFO("START");
	    if(!vcap.open(videoStreamAddress)) {
            std::cout << "Error opening video stream 1" << std::endl;
            //return -1;
        }
        ROS_INFO("Stream");
        
        draw_line = TRUE;
    }
    
    void stream(){
        //ros::Time begin = ros::Time::now();
        if(!vcap.read(frame)) {
            std::cout << "No frame" << std::endl;
            return;
        }
        if( !frame.data )
          { return; }
        
        //remap(frame, dst, map_x, map_y, CV_INTER_LINEAR, BORDER_TRANSPARENT, Scalar(0,0,0));
        //
        //if(draw_line){
        //    for (int ang = 0; ang < rect_size; ang = ang+rect_size/12){
        //        line(dst, Point(0,ang), Point(rect_size,ang),CV_RGB(255,255,255),2);
        //        line(dst, Point(ang,0), Point(ang,rect_size),CV_RGB(255,255,255),2);
        //    }
        //}
        
        //msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
        //image_pub_.publish(msg);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = ros::Time::now();
        raw_pub_.publish(msg);
        //double loop_time = (ros::Time::now() - begin).toSec();
        //ROS_INFO("%.6f", loop_time);
        if(show){
            //imshow("Image",frame);
            imshow("Image",dst);
            if(char(waitKey(10)) == 'l'){
                draw_line = !draw_line;
                std::cout << draw_line << std::endl;
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
        //return -1;
    }
    else if ( (cam_arg != "/left") && (cam_arg != "/right") ){
        ROS_WARN("Selection 'camera' has been incorrectly remapped! Typical command-line usage:\n"
             "\t$ rosrun fisheye_stereo stream camera:=<left or right>");
        //return -1;
    }
    ros::start();
    ros::Rate loop_rate(10);
    StreamRectifier * streamer = new (std::nothrow) StreamRectifier;

    while(ros::ok()) {
        streamer->stream();
        loop_rate.sleep();
    }
    delete streamer;
}
