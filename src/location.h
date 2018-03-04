#ifndef LOCATION_H
#define LOCATION_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include "opencv2/core/core.hpp"
//#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"

//#include "opencv2/calib3d.hpp"
//#include "opencv2/xfeatures2d.hpp"
//#include "opencv2/contrib/contrib.hpp"
#include "opencv2/video/tracking.hpp"

#include "utils.h"
#include "lbp.h"
#include "colorhistogram.h"
#include "hsvcalchistogram.h"

#include <vector>
#include <algorithm>
#include <functional>
#include <fstream>
#include <sstream>
#include <math.h>

#include <ros/ros.h>

#include "houghline.h"

using namespace std;
using namespace cv;

class Odometry
{
public:
    double vx=0.0;
    double vy=0.0;
    double vth=0.0;
};
class Position
{
public:
    double x=0;
    double y=0;
    double th=0;
    Point currentCrossPoint;//栅格点坐标系，x正为前，y正为左
    int fourDirection=0;//0为向前，1为向右，2为向后，3向左
};
class Location
{
public:
    Location();
    Location(string fileName);
    Location(string fileName,string outDataPath);
    ~Location();

    bool setOutFile(string filePath);
    bool transformCross(Point cross);                                                                                        //转换交叉点
    bool loadCamera(string fileName);                                                                                      //加载相机参数
    bool loadParam(string fileName);

    bool InitializeCoordinates(Vec4f mainLine,Vec4f secondLine);                                         //初始化位姿
    bool updateFourDirection();
    bool updateGrid();                                                                                                                 //更新栅格地图
    bool updateCoordinatesWithCross(Vec4f mainLine);                                                        //更新位姿
    bool updateCoordinatesWithMainLine(Vec4f mainLine);
    bool updateCoordinates();

    bool updateOdomP();
    bool updateImuP();
    bool updateTime();
    bool updateTime(double _p);
    bool updateMotion();

    bool updateAll(Mat image);
    bool updateAll(Mat image,double _p);


    void kFilter_init();
    void kFilter_update(double period);

    bool ifInit;                                         //是否初始化

    Mat intrinsic_Matrix;                        //相机内参
    Mat distortion_coeffs;                      //相机畸变参数
    double fz;                                          //相机坐标系转换到物理坐标系的参数

    Position p;                                          //当前位姿
    Position lastP;                                    //上一时刻位姿
    Position initP;                                     //初始位姿

    Position odomP;                                //里程计位姿，只用了odom信息
    Position lastOdomP;                         //上一时刻里程计位姿

    Odometry imuO;                              //imu的角速度

    Position imuP;                                   //IMU位姿
    Position lastImuP;                             //上一时刻IMU位姿，通过odom的线速度和imu的角度算出来的位姿
    Position lastCrossImuP;                   //上一时刻有交叉点时的Imu位姿

    Odometry odom;                               //里程计速度

    ros::Time updated_;                          //上一时刻时间
    double period;                                   //间隔时间

    double errOdomTh;                         //初始里程计和参考x坐标系的转角
    double errImuTh;                             //初始IMU和参考x坐标系的转角

    double floorH;                                  //地板高度

    Point2d errPoint,lastErr;                 //图像上交叉点和摄像头中心的误差，和上一时刻误差

    ofstream outfile;                              //结果输出文件流
    ofstream logLineAndCrossFile;      //线和点中间调试数据文件
    ofstream logUpdateDataFile;         //更新位置调试数据文件
    ofstream logCrossDataFile;            //更新交叉点坐标调试数据文件

    string outFileName;                                                     //输出文件名
    string logLineAndCrossFileName;                             //输出文件名
    string logUpdateDataFileName;                               //输出文件名
    string logCrossDataFileName;                                    //输出文件名

    KalmanFilter kf;                                //角度kalman滤波器
    const int stateNum=2;                     //状态维度
    const int measureNum=2;              //观测唯独
    Mat state;                                          //state(x,y,detaX,detaY)
    Mat processNoise;                           //过程噪声
    Mat measurement;                          //测量噪声

    double filterAngle,filterAngle_v;     //滤波后角度和滤波后角速度
    Position filterPosition;                     //角度滤波后计算出的位置

    HoughLine* hl;
    double L,T;

    double errCurAndLastImuX;
    double errCurAndLastImuY;
    double errCurAndLastImuTh;

    string cameraFileName;
};

#endif // LOCATION_H
