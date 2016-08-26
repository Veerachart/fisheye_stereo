#include <ros/ros.h>
#include "fisheye_stereo/Stereo_pixels.h"
#include <math.h>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>

using namespace Eigen;

const double PI = 3.141592653589793;

class Triangulate{
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    tf::TransformBroadcaster br_;
    tf::Transform transform;
    tf::Quaternion q;
    float baseline;
    float x1, y1, x2, y2;
    float rect_size;
    float beta1, psi1, beta2, psi2, rho;
    
public:
    void triangulateCallback(const fisheye_stereo::Stereo_pixels::ConstPtr& msg){
        x1 = msg->x1;
        y1 = msg->y1;
        x2 = msg->x2;
        y2 = msg->y2;
        
        beta1 = (y1-rect_size/2)*PI/rect_size;
        psi1 = (x1-rect_size/2)*PI/rect_size;
        beta2 = (y2-rect_size/2)*PI/rect_size;
        psi2 = (x2-rect_size/2)*PI/rect_size;
        
        rho = baseline*cos(psi2)/sin(psi1-psi2);
        transform.setOrigin(tf::Vector3(rho*sin(psi1), rho*cos(psi1)*sin(beta1), rho*cos(psi1)*cos(beta1)));
        q.setRPY(0,0,0);
        transform.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "blimp"));
    }
    
    Triangulate(float b, int rect){
        sub_ = nh_.subscribe("pixels", 1, &Triangulate::triangulateCallback, this);
        rect_size = rect;
        baseline = b;
    }
};


    

int main(int argc, char **argv){
    ros::init(argc, argv, "triangulate");
    float baseline;
    int rect_size;
    if(argc == 3){
        baseline = atof(argv[1]);
        rect_size = atof(argv[2]);
    }
    else if(argc == 2){
        baseline = atof(argv[1]);
        rect_size = 720;
        ROS_INFO("Size of the image not specified. Use 720x720 pixels as default");
    }
    else{
        ROS_WARN("Please specify the length of the stereo's baseline.\nUSAGE: rosrun fisheye_stereo triangulate baseline:=<length in meters>");
    }
    Triangulate triangulator(baseline, rect_size);
    ros::spin();
}
