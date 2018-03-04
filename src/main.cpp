#include <ros/ros.h>
#include <iostream>
#include <string>
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

#include "utils.h"
#include "houghline.h"
#include "colorhistogram.h"
#include "hsvcalchistogram.h"
#include "location.h"

using namespace std;
using namespace cv;

//#define UNDISTORT_FUN 1

Mat img,img_raw;
geometry_msgs::Twist cmd_vel;
nav_msgs::Odometry odom;
sensor_msgs::Imu imu;
Location myLocation("/home/ren/catkin_ws/src/car107/param.txt");
Location* pLocation;
int num;
string outPath;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
       img_raw=cv_bridge::toCvShare(msg, "bgr8")->image;
       undistort(img_raw,													    //输入源图像
                      img,												                //存放矫正后图像
                      pLocation->intrinsic_Matrix,										//内参矩阵
                      pLocation->distortion_coeffs										//畸变矩阵
                      );
          imshow("img_raw",img_raw);
          waitKey(1);
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
void getInformation(Location* pLocation)
{
    pLocation->odom.vx=odom.twist.twist.linear.x;
    pLocation->odom.vy=odom.twist.twist.linear.y;
    pLocation->odom.vth=odom.twist.twist.angular.z;

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

    pLocation->imuP.th=tf::getYaw(q);
    if(pLocation->imuP.th>PI)
    {
        pLocation->imuP.th=-2*PI+pLocation->imuP.th;
    }
    else if(pLocation->imuP.th<-PI)
    {
        pLocation->imuP.th=2*PI+pLocation->imuP.th;
    }
    pLocation->imuO.vth=imu.angular_velocity.z;
    //cout<<myLocation.imuO.vth*RAD2DEG<<endl;
}
void initParam(ros::NodeHandle& nh)
{
    nh.param<int>("/car107/num",num,1);
    nh.param<string>("/car107/outPath",outPath,"/home/ren/slam/car107_data/img44Result");
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "car107");
  ros::NodeHandle nh;
  //initParam(nh);
  string outPath=argv[1];
  string testNum=argv[2];
  string outAddr=outPath+"/"+testNum;//strtool::numToString<int>(num);
  pLocation=new Location("/home/ren/catkin_ws/src/car107/param.txt",outAddr);

  startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("ub31/image", 1, imageCallback);
  ros::Subscriber sub1 = nh.subscribe("/odom", 10, odomCallback);
  ros::Subscriber sub2 = nh.subscribe("/imu/data", 10, imuCallback);
  ros::Subscriber sub3 = nh.subscribe("/pad_teleop/cmd_vel", 10, cmdCallback);

  ros::Rate r(50);
  while(ros::ok())
  {
       ros::spinOnce();
       getInformation(pLocation);

      if(!img.empty())
      {
            if(pLocation->hl->frameNum>5)
            {
                pLocation->updateAll(img);
            }
            pLocation->hl->frameNum++;
            char c=waitKey(1);
            if(c==32)
                waitKey(0);
            else
                waitKey(1);
      }
  }
}
