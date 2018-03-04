#ifndef HOUGHLINE_H
#define HOUGHLINE_H

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

using namespace std;
using namespace cv;

#define DEG2RAD 0.017453293f    //角度转弧度系数
#define PI 3.1415926

#define TH 2

namespace findLine
{
    class Odometry
    {
    public:
        double vx;
        double vy;
        double vth;
    };
    class Position
    {
    public:
        double x;
        double y;
        double th;
        Point currentCrossPoint;//栅格点坐标系，x正为前，y正为左
        int fourDirection;//0为向前，1为向右，2为向后，3向左
    };

    class HoughLine
    {
    public:
        HoughLine(){}
        HoughLine(int _w,int _h, int _threshold,int _distThres,int _thetaThres);
        HoughLine(string paramFile);

        bool findLinesAndCross(Mat image,vector<Vec4f> out_lines,Point& cross,bool ifFindMainLine,bool ifFindSecondLine,bool ifFindCross);
        void imageFilter(Mat src,Mat& dst);
        bool pretreatment(Mat frame,Mat& dst);
        bool findLsd(Mat image,vector<Vec4f>& lines_std);
        void showLine(Mat image,vector<Vec4f> out_lines,Point& cross,bool ifFindMainLine,bool ifFindSecondLine,bool ifFindCross);

        Vec4f transformPolarToLine(float maxIDx, float maxIDy);                     //将极坐标转换为线的两个点
        Vec3f transformLineToPolar(Vec4f line);                                                  //将线的两个点转换为极坐标
        vector<Vec3f> Transform(vector<Vec4f> lines);                                      //将线段集合转换为霍夫空间的数据
        void TransformImage(vector<Vec3f> data,Mat& image_gaussian);      //转换为霍夫图像
        bool getLBP();

        bool findMainLine(Mat frame,Mat image_gaussian,Vec4f &mainLine);                             //寻找主线
        bool findSecondLine(Mat image_gaussian,Vec4f mainLine,Vec4f& secondLine);              //寻找横线
        bool judgeLine(Mat image,Vec4f line,bool mainOrSecond);                                                //判断线

        bool getCrossPoint(Vec4f mainLine,Vec4f secondLine,Point& cross);                                //获取交叉点坐标
        bool judgeCrossPoint(Mat image,Point cross);                                                                    //判断交叉点
        void getKb(Vec4f myLine,Vec2f& kb);                                                                                                                         //计算线的斜率和截距
        bool addLine(vector<Vec4f>& lines,float maxIDx,float maxIDy);

        bool transformCross(Point cross);                                                                                        //转换交叉点
        bool loadCamera(string fileName);                                                                                      //加载相机参数
        bool loadParam(string fileName);                                                                                        //加载参数

        bool InitializeCoordinates(Vec4f mainLine,Vec4f secondLine);                                         //初始化位姿
        bool updateGrid();                                                                                                                 //更新栅格地图
        bool updateCoordinatesWithCross(Vec4f mainLine);                                                        //更新位姿
        bool updateCoordinatesWithMainLine(Vec4f mainLine);
        bool updateCoordinates();

        bool updateOdomP();
        bool updateImuP();
        bool updateTime();
        bool updateMotion();


        void kFilter_init();
        void kFilter_update(double period);

        int threshold;                                     //霍夫统计个数阈值
        int w;                                                   //图像宽
        int h;                                                    //图像高
        int rows;                                              //霍夫空间高，即极距值
        int cols;                                               //霍夫空间宽，即极角值

        float lastMainMaxIDx;
        float lastMainMaxIDy;

        int thetaThres;                                     //搜索角度阈值
        int distThres;                                       //搜索极值阈值

        bool ifFollowFind;
        int ifRotate;                                         //是否旋转，0为未旋转，1为向右转，2为向左转
        int lastRotate;                                     //上次是否旋转
        int eddging;                                        //边界判断旋转参数

        int findRange;                                    //初次搜索角度范围
        int rotateRange;                                //跟踪时的搜索范围

        int frameNum;                                  //图像帧数
        bool ifInit;                                         //是否初始化

        int lineRange;                                      //线附近像素个数
        int crossRange;                                    //交叉点附近像素个数
        int mainAndSecondRoate;               //主线和横线角度差值

        Mat intrinsic_Matrix;                        //相机内参
        Mat distortion_coeffs;                      //相机畸变参数
        double fz;                                          //相机坐标系转换到物理坐标系的参数

        Mat constfeatureHist;                       //主线特征向量
        Mat constLBPfeatureHist;                //主线lbp特征向量
        int mainLineCount;

        Mat constCrossLBPfeatureHist;       //交叉点附近特征向量
        int crossCount;

        Mat constSecondfeatureHist;          //横线特征向量
        Mat constSecondLBPfeatureHist;   //横向lbp特征向量
        int secondLineCount;

        double maxScore;                            //直方图分类的最大值
        Mat lbp_image;                                 //lbp图
        Mat gray;                                           //灰度图

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

        ofstream outfile;                              //输出文件流
        string outFileName;                         //输出文件名

        KalmanFilter kf;                                //角度kalman滤波器
        const int stateNum=2;                     //状态维度
        const int measureNum=2;              //观测唯独
        Mat state;                                          //state(x,y,detaX,detaY)
        Mat processNoise;                           //过程噪声
        Mat measurement;                          //测量噪声

        double filterAngle,filterAngle_v;     //滤波后角度和滤波后角速度
        Position filterPosition;                     //角度滤波后计算出的位置


        double T;
        double L;
    };
}

#endif // HOUGHLINE2_H
