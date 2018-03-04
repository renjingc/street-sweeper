#ifndef PATHCONTROL_H
#define PATHCONTROL_H
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <limits>

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/GetMap.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <stdlib.h>
#include <sstream>
#include <string>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>

#include "sensor.h"
#include "point.h"
#include "rangereading.h"
#include "rangesensor.h"
#include "odometrysensor.h"
#include "odometryreading.h"
#include "map.h"
#include "smmap.h"
#include "gridlinetraversal.h"
#include "nodeGridMap.h"
#include "findPath.h"

#define noGone          0
#define obstacle        10
#define hasGone         20
#define gate               25
#define nextRound    30
#define KNode           50

#define forward          0
#define right                1
#define Backward       2
#define left                  3

#define goStraight      0
#define goBack           1
#define turnLeft         -1
#define turnRight       1

class PathControl
{
public:
    PathControl();
    PathControl(int maxHeight_,int maxWidth_,double delta_,double vx_,double vAngle_,double angle_P_);
    ~PathControl();

    LMapping::IntPoint world2node(const LMapping::Point& p,
                                  const Position m_center,
                                  const double m_delta,
                                  const int m_sizeX2,
                                  const int m_sizeY2,
                                  int dir);

    int getWorldDirection() const { return worldDirection;}
    void setNode(int dir,int type);
    void setNextRound(const double *readings);
    //路径规划
    bool updatePlan(const tf::StampedTransform &currPose,
                    LMapping::OrientedPoint& laser_pose,
                    const double *readings,
                    geometry_msgs::Twist &cmd);//当前地图，当前位置，当前激光
    //沿边走
    bool alongTheEdge(const double *readings);//当前地图，当前位置，当前激光
    //遍历走
    bool traversing(const double *readings);//当前地图，当前位置，当前激光

    bool isAngleRange(double angle,int dir);
    geometry_msgs::Twist goTracking(int dir);
    geometry_msgs::Twist turn(int dir);
    geometry_msgs::Twist goDistance(double start,double end);
    nav_msgs::OccupancyGrid_<std::allocator<void> > showMap;     //栅格地图
    //设置前方障碍激光
    void setLaser(std::vector<double> laser_angles_);

    NodeGridMap* pnodeMap;

    bool getIsUpdateNodeMap() const{return isUpdateNodeMap;}
    void setIsUpdateNodeMap(bool isUpdateNodeMap_) {isUpdateNodeMap=isUpdateNodeMap_;}

private:
    bool isGoTracking;                             //是否是直走
    bool isFirst;                                       //第一次
    bool isFindRight;                              //在寻找右边沿
    bool isAlongTheEdge;                      //是否在进行沿边行走
    bool isTraversing;                             //是否在进行遍历行走
    bool findGate;                                  //是否法线Gate栅格
    bool isPause;
    bool isUpdateNodeMap;                 //更新地图
    bool startfind;                                   //开始寻找最近点标志
    bool isFinish;                                     //是否走完
    bool isFind;                                        //是否找到

    int goNearId;

    LMapping::IntPoint initPosition;        //初始位置
    LMapping::Point initPosition_d;         //初始地图坐标点


    //判断四个方向的障碍
    bool judgeAdjacent(const double *readings,int direction,int type);
    //判断是否停止旋转,达到目标旋转目标
    bool judgeRotateToAim();
    //判断目标和当前位置的方向
    int judgeAimRotate(int dx,int dy);
    //设置世界目标方向
    void setWorldAimDirection(int dir);
    //得到当前坐标
    bool getCurrPoint(const tf::StampedTransform &currPose);
    //得到每个点的激光位置
    void getBeamLaserPose(const double *readings,LMapping::OrientedPoint lp);
    //寻找最近的点和路径
    bool findNearstNode(vector<PATH::MapSearchNode>& result);

    double delta;
    int maxHeight;
    int maxWidth;
    bool got_map_;
    LMapping::IntPoint K;
    int goDirection;
    int worldDirection;                                //前为x+:0,右为y-:1,后为x-:2,左为y+:3
    int aimWorldDirection;                         //目标世界方向
    int aimDirection;                                   //设置方向

    double maxLaserAngle;
    double minLaserAngle;
    int minLaserIndex;
    int maxLaserIndex;
    std::vector<double> laser_angles_use_;//每个点激光角度
    unsigned int laser_beam_use_count_;//单次使用到的激光点数
    vector<LMapping::Point> beamLaser_poses;//每个激光点的位姿
    double maxfontRange_,minfontRange_;

    double angle_P;
    double vx,vAngle;
    double distUpdatePoint;                                    //距离点距离参数
    LMapping::OrientedPoint currPosition;           //当前车体世界坐标
    LMapping::IntPoint currPoint;                           //当前车体地图坐标
    LMapping::IntPoint lastPoint;                           //当前车体地图坐标
    geometry_msgs::Twist Last_cmd;
    vector<LMapping::IntPoint> historyPoint;      //历史走过的点

    vector<PATH::MapSearchNode> planPath;
};

double ran_gaussian(double sigma);
#endif // PATHCONTROL_H
