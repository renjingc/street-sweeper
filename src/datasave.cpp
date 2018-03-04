#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/core/core.hpp>
//#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <math.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>

#include "houghline.h"

using namespace std;
using namespace cv;
HoughLine hl("/home/ren/catkin_ws/src/car107/param.txt");
geometry_msgs::Twist cmd_vel;
nav_msgs::Odometry odom;
sensor_msgs::Imu imu;
Mat img,img_raw;
int frameCount=1;
ofstream outfile;
string fileAddr1="/home/ren/slam/car107_data/img33/data.txt";
string fileAddr="/home/ren/slam/car107_data/img33/";
ros::Time updated_;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
       img_raw=cv_bridge::toCvShare(msg, "bgr8")->image;
       double period = (ros::Time::now() - updated_).toSec();
       //更新时间
       updated_ = ros::Time::now();
       cout<<period<<endl;

        double vx=odom.twist.twist.linear.x;
        double vy=odom.twist.twist.linear.y;
        double vth=odom.twist.twist.angular.z;

        geometry_msgs::Quaternion q;
        q.w=imu.orientation.w;
        q.x=imu.orientation.x;
        q.y=imu.orientation.y;
        q.z=imu.orientation.z;

        geometry_msgs::Quaternion odomQ;
        odomQ.x=odom.pose.pose.orientation.x;
        odomQ.y=odom.pose.pose.orientation.y;
        odomQ.z=odom.pose.pose.orientation.z;
        odomQ.w=odom.pose.pose.orientation.w;

        double imuPTh=tf::getYaw(q);
        if(imuPTh>PI)
        {
            imuPTh=-2*PI+imuPTh;
        }
        else if(imuPTh<-PI)
        {
            imuPTh=2*PI+imuPTh;
        }
        double imuOVth=imu.angular_velocity.z;

        stringstream ss;
        ss<<setw(6)<<setfill('0')<<frameCount;
        string filePath=fileAddr+ss.str()+".jpg";


        stringstream vxSS,vySS,vthSS,imuPThSS,imuOVthSS;
        vxSS<<vx;
        vySS<<vy;
        vthSS<<vth;
        imuPThSS<<imuPTh;
        imuOVthSS<<imuOVth;

        stringstream timeSS;
        timeSS<<period;

        stringstream cmdXSS,cmdYSS,cmdThSS;
        cmdXSS<<cmd_vel.linear.x;
        cmdYSS<<cmd_vel.linear.y;
        cmdThSS<<cmd_vel.angular.z;

        string dataString=timeSS.str()+" "+vxSS.str()+" "+vySS.str()+" "+vthSS.str()+" "+
                                    imuPThSS.str()+" "+imuOVthSS.str()+" "+
                                    cmdXSS.str()+" "+cmdYSS.str()+" "+cmdThSS.str();

        imwrite(filePath,img_raw);
        outfile<<dataString<<endl;
        frameCount++;
        cout<<filePath<<endl;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void odomCallback(const nav_msgs::Odometry& odom_)
{
    odom=odom_;
}

void imuCallback(const sensor_msgs::Imu& imu_)
{
    imu=imu_;
}

void cmdCallback(const geometry_msgs::Twist& cmd_)
{
    cmd_vel=cmd_;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "datasave");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("ub31/image", 1, imageCallback);
    ros::Subscriber sub1 = nh.subscribe("/odom", 10, odomCallback);
    ros::Subscriber sub2 = nh.subscribe("/imu/data", 10, imuCallback);
    ros::Subscriber sub3 = nh.subscribe("/pad_teleop/cmd_vel", 10, cmdCallback);

    outfile.open(fileAddr1);
    ros::Rate r(55);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
