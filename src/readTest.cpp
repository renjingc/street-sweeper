#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <fstream>
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

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>

#include "utils.h"
#include "houghline.h"
#include "colorhistogram.h"
#include "hsvcalchistogram.h"
#include "location.h"

using namespace std;
using namespace cv;

//#define UNDISTORT_FUN 1

//Mat img,img_raw;

vector<long double> timeVector;
vector<double> vxVector;
vector<double> vyVector;
vector<double> vthVector;
vector<double> imuThVector;
vector<double> imuOthVector;
vector<double> cmdXVector;
vector<double> cmdYVector;
vector<double> cmdThVector;
vector<string> imageFileNameVector;
vector<Mat> imageVector;

int dataLength=0;
long double lastTime=0.0;
long double curTime=0.0;
string outPath="/home/ren/slam/car107_data/img33Result";

void getFiles( string path, vector<string>& files )
{
        DIR *dir;
        struct dirent *ptr;
        char base[1000];

        if ((dir=opendir(path.c_str())) == NULL)
            {
            perror("Open dir error...");
                    exit(1);
            }

        while ((ptr=readdir(dir)) != NULL)
        {
            if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
                    continue;
            else if(ptr->d_type == 8)    ///file
            {
                vector<string> fileName;
                strtool::split(ptr->d_name,fileName,".");
                if(fileName.size()>=2&&fileName[1]=="jpg")
                {
                    files.push_back(ptr->d_name);
                }
            }
            else if(ptr->d_type == 10)    ///link file
                continue;
            else if(ptr->d_type == 4)    ///dir
            {
                files.push_back(ptr->d_name);
            }
        }
        closedir(dir);
        //排序，按从小到大排序
        sort(files.begin(), files.end());
}
void readImage(string fileAddr)
{
    int num=0;
    getFiles(fileAddr, imageFileNameVector);
    for(int i=0;i<imageFileNameVector.size();i++)
    {
        Mat image=imread(fileAddr+"/"+imageFileNameVector[i]);
        imageVector.push_back(image);
        cout<<"readImage:"<<i<<endl;
    }

}

void readData(string fileAddr)
{
    string fileName=fileAddr+"/data.txt";
    ifstream fin(fileName);
    string s;

    while(fin.good())
    {
        getline(fin, s);
        vector<string> lineData;
        strtool::split(s,lineData," ");
        if(lineData.size()>0&&lineData[0]!=" ")
        {
            long double time=strtool::stringToNum<double>(lineData[0]);
            double vx=strtool::stringToNum<double>(lineData[1]);
            double vy=strtool::stringToNum<double>(lineData[2]);
            double vth=strtool::stringToNum<double>(lineData[3]);
            double imuTh=strtool::stringToNum<double>(lineData[4]);
            double imuOth=strtool::stringToNum<double>(lineData[5]);
            double cmdX=strtool::stringToNum<double>(lineData[6]);
            double cmdY=strtool::stringToNum<double>(lineData[7]);
            double cmdTh=strtool::stringToNum<double>(lineData[8]);
            timeVector.push_back(time);
            vxVector.push_back(vx);
            vyVector.push_back(vy);
            vthVector.push_back(vth);
            imuThVector.push_back(imuTh);
            imuOthVector.push_back(imuOth);
            cmdXVector.push_back(cmdX);
            cmdYVector.push_back(cmdY);
            cmdThVector.push_back(cmdTh);

            dataLength++;
        }
    }
    cout<<dataLength<<endl;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "readTest");
    ros::NodeHandle nh;

    startWindowThread();
  if(argc<3)
  {
      cerr<<"argc<3!"<<endl;
      return 1;
  }
  string fileAddr=argv[1];
  string testNum=argv[2];


  readData(fileAddr);
  readImage(fileAddr);
  cout<<"read finish!"<<endl<<endl;

  string filePath=fileAddr+"/"+testNum;
  Location* pLocation;
  for(int k=0;k<strtool::stringToNum<int>(testNum);k++)
  {
      string outAddr=outPath+"/"+strtool::numToString<int>(k+1);
      pLocation=new Location("/home/ren/catkin_ws/src/car107/param.txt",outAddr);
      cout<<endl;
      for(int i=2220;i<dataLength;i++)
      {
               Mat img;
               double t = (double)getTickCount();
               undistort(imageVector[i],													    //输入源图像
                              img,                                                                          //存放矫正后图像
                              pLocation->intrinsic_Matrix,										//内参矩阵
                              pLocation->distortion_coeffs										//畸变矩阵
                              );
               pLocation->odom.vx=vxVector[i];
               pLocation->odom.vy=vyVector[i];
               pLocation->odom.vth=vthVector[i];
               pLocation->imuP.th=imuThVector[i];
               pLocation->imuO.vth=imuOthVector[i];

               curTime=timeVector[i];
               cout<<curTime<<" "<<lastTime<<endl;
               double period=curTime-lastTime;

                if(i>5)
                {
                    pLocation->updateAll(img,period);
                }
                t = ((double)getTickCount() - t)/getTickFrequency();
                //cout<<"t:"<<t<<endl;
                pLocation->hl->frameNum=i;
                lastTime=curTime;

                char c=waitKey(1);
                if(c==32)
                    waitKey(0);
                else
                    waitKey(1);
    }
  }
  cout<<"finish"<<endl;
}
