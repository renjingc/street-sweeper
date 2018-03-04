#ifndef LASERMAPPING_H
#define LASERMAPPING_H

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
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <math.h>
#include <boost/thread.hpp>

#include "sensor.h"
#include "point.h"
#include "rangereading.h"
#include "rangesensor.h"
#include "odometrysensor.h"
#include "odometryreading.h"
#include "map.h"
#include "smmap.h"
#include "gridlinetraversal.h"
#include "pathcontrol.h"

using namespace std;
#define GAUSSIAN 1

class LaserMapping
{
public:
    LaserMapping();
    LaserMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    LaserMapping(ros::NodeHandle& nh);
    LaserMapping(unsigned long int seed, unsigned long int max_duration_buffer);
    ~LaserMapping();

    void init();
    void startLiveSlam();
    void startReplay(const std::string & bag_fname, std::string scan_topic);
    void publishTransform();
    void publishCmd();

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res);
    bool mapCallback1(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res);
    void publishLoop1(double transform_publish_period);
    void publishLoop2(double transform_publish_period);
    bool loadParam(string fileName);
private:
    ros::NodeHandle node_;
    ros::NodeHandle private_nh_;
    ros::Publisher sst_;
    ros::Publisher sstm_;
    ros::ServiceServer ss_;
    ros::Publisher sst1_;
    ros::Publisher sstm1_;
    ros::ServiceServer ss1_;

    ros::Publisher cmd_;

    tf::TransformListener tf_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    tf::TransformBroadcaster* tfB_;

    bool do_reverse_range_;

    bool got_first_scan_;
    bool got_map_;
    nav_msgs::GetMap::Response map_,map1_;

    ros::Duration map_update_interval_;
    tf::Transform map_to_odom_;
    boost::mutex map_to_odom_mutex_;
    boost::mutex cmd_mutex_;
    boost::mutex map_mutex_,map_mutex1_;

    unsigned long int seed_;
    double transform_publish_period_;
    double tf_delay_;

    boost::thread* transform_thread_;
    boost::thread* cmd_thread_;

    std::string base_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    void updateMap(const sensor_msgs::LaserScan& scan);
    bool getOdomPose(LMapping::OrientedPoint& map_pose, const ros::Time& t);
    void updateNodeMap(const double *readings);

    bool initMapper(const sensor_msgs::LaserScan& scan);
    LMapping::RangeReading addScan(const sensor_msgs::LaserScan& scan, LMapping::OrientedPoint& map_pose);
    //void setSensorMap(const LMapping::SensorMap& smap);

    void invalidateActiveArea();
    void computeActiveArea(LMapping::ScanMap& map, const LMapping::OrientedPoint& p, const double* readings);
    double registerScan(LMapping::ScanMap& map, const LMapping::OrientedPoint& p, const double* readings);
    bool m_activeAreaComputed;

    //自定义的里程计数据
    LMapping::OdometrySensor* odom_sensor_;
    //自定义的激光数据格式
    LMapping::RangeSensor* range_laser_;//第一次采集激光的值
    //每个点激光角度
    std::vector<double> laser_angles_;
    std::vector<double> laser_angles_use_;
    //初始中心激光的位姿
    tf::Stamped<tf::Pose> centered_laser_pose_;

    unsigned int laser_beam_count_;//单次接收到的激光点数
    unsigned int laser_beam_use_count_;//单次使用到的激光点数
    int laser_count_;//接收到的激光次数
    int throttle_scans_;//是否每次激光都使用，1为每次激光都使用，默认１

    double maxLaserAngle;
    double minLaserAngle;
    double maxRange_;//激光点距离的最大值
    double maxUrange_;//激光点距离的最大值
    double minUrange_;//激光点距离的最小值
    int minLaserIndex;
    int maxLaserIndex;
    unsigned int m_initialBeamsSkip;
    bool m_generateMap;

    double xmin_;           //地图x最小值
    double ymin_;           //地图y最小值
    double xmax_;          //地图x最大值
    double ymax_;           //地图y最大值
    double xmin1_;           //地图x最小值
    double ymin1_;           //地图y最小值
    double xmax1_;          //地图x最大值
    double ymax1_;           //地图y最大值
    double delta_;           //分辨率
    double delta1_;         //分辨率1

    double min_angle;
    double max_angle;

    double occ_thresh_;

    double vx,vAngle;
    double angle_P;

    vector<LMapping::OrientedPoint> poseVector;
    vector<LMapping::RangeReading> laserVector;
    LMapping::IntPoint* m_linePoints;

    PathControl* pPathControl;
    LMapping::ScanMap* g_psmap;
};

#endif // LASERMAPPING_H
