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

#include "houghline.h"
#include "colorhistogram.h"
#include "hsvcalchistogram.h"

using namespace std;
using namespace cv;

//#define UNDISTORT_FUN 1

Mat img,img_raw;
geometry_msgs::Twist cmd_vel;
nav_msgs::Odometry odom;
sensor_msgs::Imu imu;
//findLine::HoughLine hl(640,480,10,180,10);
findLine::HoughLine hl("/home/ren/catkin_ws/src/car107/param.txt");

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
       img_raw=cv_bridge::toCvShare(msg, "bgr8")->image;
       undistort(img_raw,													    //输入源图像
                      img,												                //存放矫正后图像
                      hl.intrinsic_Matrix,										//内参矩阵
                      hl.distortion_coeffs										//畸变矩阵
                      );
          //imshow("img_raw",img_raw);
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
    void　imageFilter(Mat src, Mat &dst)
    {
        Mat kernel = (Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
        cv::filter2D(src, dst, src.depth(), kernel);
    }
int main(int argc, char **argv)
{
  ros::init(argc, argv, "car107");
  ros::NodeHandle nh;

  startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("ub31/image", 1, imageCallback);
  image_transport::Publisher image_pub_;
  image_pub_ = it.advertise("/image_converter/output_video", 1);

  Mat frame,frame_grey;

  findLine::Position p;
  hl.loadCamera("/home/ren/catkin_ws/src/car107/calibrationdata/ost1.txt");

  ros::Subscriber sub1 = nh.subscribe("/odom", 10, odomCallback);
  ros::Subscriber sub2 = nh.subscribe("/imu/data", 10, imuCallback);
  ros::Subscriber sub3 = nh.subscribe("/pad_teleop/cmd_vel", 10, cmdCallback);

  ros::Rate r(50);
  while(ros::ok())
  {
       ros::spinOnce();
	  //cout<<"vx:"<<odom.twist.twist.linear.x<<" vy:"<<odom.twist.twist.linear.y<<" vth:"<<odom.twist.twist.angular.z<<endl;
	  //cout<<"imu vz:"<<imu.angular_velocity.z<<" imu orientation:"<<imu.orientation.w<<" "<<imu.orientation.x<<" "<<imu.orientation.y<<" "<<imu.orientation.z<<endl;
      hl.odom.vx=odom.twist.twist.linear.x;
      hl.odom.vy=odom.twist.twist.linear.y;
      hl.odom.vth=odom.twist.twist.angular.z;

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
      //hl.odomP.th=tf::getYaw(odomQ);
      hl.imuP.th=tf::getYaw(q);
      hl.imuO.vth=imu.angular_velocity.z;

      if(!img.empty())
      {
            Mat image1,image2;
            resize(img, frame, Size(640, 480));

            //直方图显示
            Mat frameReduce;
            frame.copyTo(frameReduce);

            cvtColor(frame, hl.gray, CV_BGR2GRAY);
            //bilateralFilter(image,image1,10, 10*2, 10/2);//双边滤波器

            imageFilter(hl.gray,image1);//锐化
            image1.copyTo(image2);

            Mat drawnLines(image1);
            Mat drawnLines1,drawnLines2;
            Mat image_gaussian;
            frame.copyTo(drawnLines1);
            frame.copyTo(drawnLines2);

            Mat contours;
            Canny(image1, contours, 5, 10); // Apply canny edge//可选canny算子
            Mat contoursInv;
            threshold(contours, contoursInv, 128, 255, cv::THRESH_BINARY_INV);//why

            //double_scale 越大检测越松
            //_sigma_scale 越大检测越松
            //_quant 越大检测越松
            //_ang_th 越大检测越松
            Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE, 1.5, 0.6, 4.0, 42.5, 0.0, 0.8);
            vector<Vec4f> lines_std;

           double start = double(getTickCount());
            // Detect the lines
            ls->detect(image1, lines_std);//这里把检测到的直线线段都存入了lines_std中，4个float的值，分别为起止点的坐标
            ls->drawSegments(drawnLines, lines_std);

            vector<Vec4f> out_lines;

             //寻找主线
            hl.updateMotion();
            Vec4f mainLine;
            vector<Vec3f> data=hl.Transform(lines_std);
            hl.TransformImage(data,image_gaussian);


            if(hl.frameNum>3)
            {
                Vec2f maxData;
                int findMainCount=0;
                bool ifFindMainLine=false;
                hl.getLBP();
                ifFindMainLine=hl.findMainLine(frame,image_gaussian,mainLine,maxData);
                //判断主线
                if(ifFindMainLine)
                {
                    out_lines.push_back(mainLine);
                }
                //寻找横线
                Vec4f secondLine;
                bool ifFindSecondLine;
                if(ifFindMainLine)
                {
                    ifFindSecondLine=hl.findSecondLine(image_gaussian,mainLine,secondLine,maxData);
                    if(ifFindSecondLine)
                    {
                            out_lines.push_back(secondLine);
                    }
                }
                //计算十字交叉点
                Point cross;
                bool ifFindCross;
                if(ifFindMainLine&&ifFindSecondLine)
                    ifFindCross=hl.getCrossPoint(mainLine,secondLine,cross);
                else
                    ifFindCross=false;
                //if(ifFindCross)
                    //ifFindCross=hl.judgeCrossPoint(frame,cross);
                //计算栅格坐标
                if(ifFindCross)
                {
                    hl.transformCross(cross);
                    hl.updateGrid();
                }
                //定位
                if(ifFindCross&&hl.ifRotate==0)
                {
                    if(!hl.ifInit)
                    {
                        hl.InitializeCoordinates(mainLine,secondLine);
                        hl.ifInit=true;
                    }
                    else
                    {
                        hl.updateCoordinatesWithCross(mainLine);
                    }
                }
                else if(ifFindMainLine&&hl.ifRotate==0)
                {
                    if(hl.ifInit)
                    {
                        hl.updateCoordinatesWithMainLine(mainLine);
                    }
                }
                else
                {
                    if(hl.ifInit)
                    {
                        hl.updateCoordinates();
                    }
                }
                if(ifFindMainLine)
                {
                    line(drawnLines2,Point(mainLine[0],mainLine[1]),Point(mainLine[2],mainLine[3]),Scalar(0,0,255),5);
                    line(drawnLines1,Point(mainLine[0],mainLine[1]),Point(mainLine[2],mainLine[3]),Scalar(0,0,255),5);
                }
                if(ifFindSecondLine)
                {
                    line(drawnLines2,Point(secondLine[0],secondLine[1]),Point(secondLine[2],secondLine[3]),Scalar(0,255,0),5);
                }
                if(ifFindCross)
                {
                    circle(drawnLines2,cross,5,Scalar(0,0,0),5);
                }
            }
          double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
            //cout<<duration_ms<<endl;
            /*
            Mat frame_gray,drawLines_gray,drawLines1_gray,drawLines2_gray,image_gaussian_graw;
            cvtColor(frame,frame_gray,CV_BGR2GRAY);
            cvtColor(drawnLines,drawLines_gray,CV_BGR2GRAY);
            cvtColor(drawnLines1,drawLines1_gray,CV_BGR2GRAY);
            cvtColor(drawnLines2,drawLines2_gray,CV_BGR2GRAY);*/

            //imshow("Canny",contoursInv);
            //imshow("drawnLines",drawnLines);
            //imshow("drawnLines1",drawnLines1);
            imshow("drawnLines2", drawnLines2);
            //imshow("frame",frame);

            /*
            imshow("drawnLines",drawLines_gray);
            imshow("drawnLines1",drawLines1_gray);
            imshow("drawnLines2", drawLines2_gray);
            imshow("frame",frame_gray);*/

            hl.frameNum++;
            char c=waitKey(1);
            if(c==32)
                waitKey(0);
            else
                waitKey(1);
      }
      //r.sleep();
  }
}
