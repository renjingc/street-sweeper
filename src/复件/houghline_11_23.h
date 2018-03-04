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
        HoughLine();
        HoughLine(int _w,int _h, int _threshold,int _distThres,int _thetaThres);
        HoughLine(string paramFile);

        bool findLinesAndCross(Mat image,Vec4f& mainLine,Vec4f& secondLine,Point& cross,bool& ifFindMainLine,bool& ifFindSecondLine,bool& ifFindCrossn,int& fourDirectio);
        void imageFilter(Mat src,Mat& dst);
        bool pretreatment(Mat frame,Mat& dst);
        bool findLsd(Mat image,Mat& dst,vector<Vec4f>& lines_std);
        void showLine(Mat image,Vec4f mainLine,Vec4f secondLine,Point& cross,bool ifFindMainLine,bool ifFindSecondLine,bool ifFindCross);

        Vec4f transformPolarToLine(float maxIDx, float maxIDy);                     //将极坐标转换为线的两个点
        Vec3f transformLineToPolar(Vec4f line);                                                  //将线的两个点转换为极坐标
        vector<Vec3f> Transform(vector<Vec4f> lines);                                      //将线段集合转换为霍夫空间的数据
        void TransformImage(vector<Vec3f> data,Mat& image_gaussian);      //转换为霍夫图像
        bool getLBP();

        bool findMainLine(Mat frame,Mat image_gaussian,Vec4f &mainLine,int& ifRotate,int& fourDirection);                             //寻找主线
        bool findSecondLine(Mat image_gaussian,Vec4f mainLine,Vec4f& secondLine);              //寻找横线
        bool judgeLine(Mat image,Vec4f line,bool mainOrSecond);                                                //判断线

        bool getCrossPoint(Vec4f mainLine,Vec4f secondLine,Point& cross);                                //获取交叉点坐标
        bool judgeCrossPoint(Mat image,Point cross);                                                                    //判断交叉点
        void getKb(Vec4f myLine,Vec2f& kb);                                                                                                                         //计算线的斜率和截距
        bool addLine(vector<Vec4f>& lines,float maxIDx,float maxIDy);

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


        int lineRange;                                      //线附近像素个数
        int crossRange;                                    //交叉点附近像素个数
        int mainAndSecondRoate;               //主线和横线角度差值

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

    };
}

#endif // HOUGHLINE2_H
