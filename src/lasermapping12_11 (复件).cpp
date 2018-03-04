#include <iostream>
#include <time.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/MapMetaData.h>

#include "lasermapping.h"
#include "rangesensor.h"
using namespace std;

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

LaserMapping::LaserMapping():
    map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
    laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
    seed_ = time(NULL);
    init();
}

LaserMapping::LaserMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh):
    map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
    laser_count_(0),node_(nh), private_nh_(pnh), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
    seed_ = time(NULL);
    init();
}
LaserMapping::LaserMapping(long unsigned int seed, long unsigned int max_duration_buffer):
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL),
  seed_(seed), tf_(ros::Duration(max_duration_buffer))
{
    init();
}

void LaserMapping::init()
{
    tfB_ = new tf::TransformBroadcaster();
    ROS_ASSERT(tfB_);

    range_laser_=NULL;
    odom_sensor_ = NULL;

    got_first_scan_ = false;
    got_map_ = false;

    m_activeAreaComputed=false;
    m_initialBeamsSkip=0;

    if(!private_nh_.getParam("maxLaserAngle", maxLaserAngle))
      maxLaserAngle = 1;
    if(!private_nh_.getParam("minLaserAngle", minLaserAngle))
      minLaserAngle = -1;
    if(!private_nh_.getParam("throttle_scans", throttle_scans_))
      throttle_scans_ = 1;
    if(!private_nh_.getParam("base_frame", base_frame_))
      base_frame_ = "base_link";
    if(!private_nh_.getParam("map_frame", map_frame_))
      map_frame_ = "map";
    if(!private_nh_.getParam("odom_frame", odom_frame_))
      odom_frame_ = "odom";
    if(!private_nh_.getParam("laser_frame_", laser_frame_))
      laser_frame_ = "front_laser";

    private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

    maxUrange_ = 0.0;  maxRange_ = 0.0; //
    double tmp;
    if(!private_nh_.getParam("map_update_interval", tmp))
      tmp = 5.0;
    map_update_interval_.fromSec(tmp);
    if(!private_nh_.getParam("xmin", xmin_))
      xmin_ = -100.0;
    if(!private_nh_.getParam("ymin", ymin_))
      ymin_ = -100.0;
    if(!private_nh_.getParam("xmax", xmax_))
      xmax_ = 100.0;
    if(!private_nh_.getParam("ymax", ymax_))
      ymax_ = 100.0;
    if(!private_nh_.getParam("delta", delta_))
      delta_ = 0.6;
    if(!private_nh_.getParam("occ_thresh", occ_thresh_))
      occ_thresh_ = 0.25;
    if(!private_nh_.getParam("m_generateMap", m_generateMap))
        m_generateMap=false;
    if(!private_nh_.getParam("tf_delay", tf_delay_))
      tf_delay_ = transform_publish_period_;

    m_linePoints= new LMapping::IntPoint[20000];

}

void LaserMapping::startLiveSlam()
{
    //发布栅格地图
    sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    //发布栅格地图信息，包括地图载入时间,分辨率(m/cell),宽度,高度,原点,
    sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    //发布地图服务
    ss_ = node_.advertiseService("dynamic_map", &LaserMapping::mapCallback, this);
    //获取激光数据
    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "/front/scan", 5);
    //激光滤波器，获取tf和目标坐标系。
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
    //注册关联函数,laserCallback。Boost.Bind 支持最多九个参数
    //第一个参数为自身类,_1
    scan_filter_->registerCallback(boost::bind(&LaserMapping::laserCallback, this, _1));

    //用一个线程去发布,transform_publish_period_＝0.05,为0.05秒发布一次
    transform_thread_ = new boost::thread(boost::bind(&LaserMapping::publishLoop, this, transform_publish_period_));

}
bool LaserMapping::mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
    //当获取到地图时，将地图通过服务发出去
    //此锁为智能锁，作用域应该在函数内，函数调用完毕，锁会自动释放
    //锁上地图锁
    boost::mutex::scoped_lock map_lock (map_mutex_);
    if(got_map_ && map_.map.info.width && map_.map.info.height)
    {
      res = map_;
      return true;
    }
    else
      return false;
}

void LaserMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //激光次数相加
    laser_count_++;    //如果激光次数==0,则退出
    if((laser_count_ % throttle_scans_)!=0)
        return;

    static ros::Time last_map_update(0,0);

    //第一次获得激光后初始化地图
    if(!got_first_scan_)
    {
        if(!initMapper(*scan))
          return;
        got_first_scan_ = true;
    }

    //里程计位姿
    LMapping::OrientedPoint odom_pose;


    //增加激光，参数激光和里程计位姿
    LMapping::RangeReading curr_laser=addScan(*scan,odom_pose);
    {
        ROS_DEBUG("scan processed");
        cout<<"New Odometry Pose (reported from observation)= "
           << curr_laser.getPose().x << " "
           << curr_laser.getPose().y<< " "
           <<curr_laser.getPose().theta << endl;

        poseVector.push_back(curr_laser.getPose());
        laserVector.push_back(curr_laser);
        //不进行激光地图矫正
        LMapping::OrientedPoint mpose = odom_pose;//gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
        ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
        ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
        //ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

        tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
        tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

        //更新地图
        //map_update_interval_为设置的两帧激光的间隔时间,5
        if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
        {
              updateMap(*scan);
              last_map_update = scan->header.stamp;
              ROS_DEBUG("Updated the map");
        }
    }
}

LMapping::RangeReading LaserMapping::addScan(const sensor_msgs::LaserScan &scan, LMapping::OrientedPoint &map_pose)
{
    //获取位姿
    if(!getOdomPose(map_pose, scan.header.stamp))
       return false;
    if(scan.ranges.size() != laser_beam_count_)
      return false;

    double* ranges_double = new double[laser_beam_use_count_];
    if (do_reverse_range_)
    {
        ROS_DEBUG("Inverting scan");
        //int num_ranges = scan.ranges.size();
        for(int i=0; i < laser_beam_use_count_; i++)
        {
          // Must filter out short readings, because the mapper won't
          if(scan.ranges[maxLaserIndex - i] < scan.range_min)
                ranges_double[i] = (double)scan.range_max;
          else
                ranges_double[i] = (double)scan.ranges[maxLaserIndex - i];
        }
    }
    else
     {
        for(unsigned int i=0; i < laser_beam_use_count_; i++)
        {
          // Must filter out short readings, because the mapper won't
          if(scan.ranges[minLaserIndex+i] < scan.range_min)
             ranges_double[minLaserIndex+i] = (double)scan.range_max;
          else
             ranges_double[minLaserIndex+i] = (double)scan.ranges[minLaserIndex+i];
        }
     }

    LMapping::RangeReading reading(laser_beam_use_count_,
                                   ranges_double,
                                   range_laser_,
                                   scan.header.stamp.toSec());

    // ...but it deep copies them in RangeReading constructor, so we don't
    // need to keep our array around.
    //delete[] ranges_double;

    reading.setPose(map_pose);

    return reading;
}
/*
void LaserMapping::setSensorMap(const LMapping::SensorMap &smap)
{
    LMapping::SensorMap::const_iterator laser_it=smap.find(std::string("FLASER"));
    if (laser_it==smap.end()){
      std::cerr << "Attempting to load the new carmen log format" << endl;
      laser_it=smap.find(std::string("ROBOTLASER1"));
      assert(laser_it!=smap.end());
    }
    const LMapping::RangeSensor* rangeSensor=dynamic_cast<const LMapping::RangeSensor*>((laser_it->second));
    assert(rangeSensor && rangeSensor->beams().size());
    m_beams=static_cast<unsigned int>(rangeSensor->beams().size());
    angles=new double[rangeSensor->beams().size()];
    for (unsigned int i=0; i<m_beams; i++){
      angles[i]=rangeSensor->beams()[i].pose.theta;
    }
}*/

bool LaserMapping::initMapper(const sensor_msgs::LaserScan &scan)
{
    laser_frame_ = scan.header.frame_id;
    // Get the laser's pose, relative to base.
    // tf::Stamped为tf的标准类型，即将数据加上ros::Time stamp_和std::string frame_id_
    //ident为位姿，矩阵R和向量t
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    //设置ident单位矩阵
    ident.setIdentity();
    ident.frame_id_ = laser_frame_;
    ident.stamp_ = scan.header.stamp;
    try
    {
        //得到当前激光坐标
        tf_.transformPose(base_frame_, ident, laser_pose);
    }
    catch(tf::TransformException e)
    {
          ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
                   e.what());
          return false;
    }
    // 在激光器位置上方1m处创建一个点，并将其转换为激光坐标
    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose.getOrigin().z());

    //得到激光点的一个坐标
    tf::Stamped<tf::Vector3> up(v, scan.header.stamp,base_frame_);

    try
    {
          tf_.transformPoint(laser_frame_, up, up);
          ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
    }
    catch(tf::TransformException& e)
    {
          ROS_WARN("Unable to determine orientation of laser: %s",
                   e.what());
          return false;
    }
    if (fabs(fabs(up.z()) - 1) > 0.001)
    {
          ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                       up.z());
          return false;
    }
    //获取当前激光点个数
    laser_beam_count_ = scan.ranges.size();
    //计算中心激光的角度
    double angle_center = (scan.angle_min + scan.angle_max)/2;
    //得到当前激光坐标系下的中心激光位姿
    if (up.z() > 0)
    {
          do_reverse_range_ = scan.angle_min > scan.angle_max;
          centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
                                                                     tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
          //ROS_INFO("Laser is mounted upwards.");
    }
    else
    {
          do_reverse_range_ = scan.angle_min < scan.angle_max;
          centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(M_PI,0,-angle_center),
                                                                     tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
          //ROS_INFO("Laser is mounted upside down.");
    }
    //计算激光每个点的角度
    laser_angles_.resize(scan.ranges.size());
    // Make sure angles are started so that they are centered
    double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
    for(unsigned int i=0; i<scan.ranges.size(); ++i)
    {
          laser_angles_[i]=theta;
          theta += std::fabs(scan.angle_increment);
    }
    ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max,
              scan.angle_increment);
    ROS_DEBUG("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
              laser_angles_.back(), std::fabs(scan.angle_increment));

    bool findMin=false,findMax=false;
    for(int i=0;i<laser_angles_.size();i++)
    {
        if(laser_angles_[i]>minLaserAngle && findMin==false)
        {
            minLaserIndex=i;
            findMin=true;
        }
        if(laser_angles_[i]>maxLaserAngle && findMax==false)
        {
            maxLaserIndex=i;
            findMax=true;
        }
    }
    if(!findMin)
        minLaserIndex=0;
    if(!findMax)
        maxLaserIndex=laser_angles_.size();
    laser_beam_use_count_=maxLaserIndex-minLaserIndex+1;
    cout<<laser_beam_use_count_<<endl;
    laser_angles_use_.resize(laser_beam_use_count_);
    for(int i=minLaserIndex;i<=minLaserIndex;i++)
        laser_angles_use_[i-minLaserIndex]=laser_angles_[i];
    LMapping::OrientedPoint map_pose(0, 0, 0);
    // setting maxRange and maxUrange here so we can set a reasonable default
    ros::NodeHandle private_nh_("~");

    //设置激光测距的最大值
    if(!private_nh_.getParam("maxRange", maxRange_))
      maxRange_ = scan.range_max - 0.01;
    if(!private_nh_.getParam("maxUrange", maxUrange_))
      maxUrange_ = maxRange_;
    range_laser_ = new LMapping::RangeSensor("FLASER",
                                           laser_beam_use_count_,
                                           fabs(scan.angle_increment),
                                           map_pose,
                                           0.0,
                                           maxRange_);

    ROS_ASSERT(range_laser_);
    odom_sensor_ = new LMapping::OdometrySensor(odom_frame_);
    ROS_ASSERT(odom_sensor_);

    LMapping::OrientedPoint initialPose;
    if(!getOdomPose(initialPose, scan.header.stamp))
    {
        ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
        initialPose = LMapping::OrientedPoint(0.0, 0.0, 0.0);
    }
    ROS_INFO("Initialization complete");

    return true;
}

bool LaserMapping::getOdomPose(LMapping::OrientedPoint &map_pose, const ros::Time &t)
{
    //当前中心激光坐标的时间
    centered_laser_pose_.stamp_ = t;
    //中心激光坐标
    tf::Stamped<tf::Transform> odom_pose;
    try
    {
        //转换得到中心激光位姿，
        tf_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
    }
    catch(tf::TransformException e)
    {
      ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
      return false;
    }
    //获取中心激光的角度
    double yaw = tf::getYaw(odom_pose.getRotation());

    //获取中心激光的x,y,theta
    map_pose = LMapping::OrientedPoint(odom_pose.getOrigin().x(),
                                        odom_pose.getOrigin().y(),
                                        yaw);
    return true;
}

void LaserMapping::publishLoop(double transform_publish_period)
{
  if(transform_publish_period == 0)
    return;

  //频率为１/0.05,20hz
  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok())
  {
        //发布map到odom的坐标变换
        publishTransform();
        //延时
        r.sleep();
  }
}

void LaserMapping::publishTransform()
{
    //map_to_odom的互斥锁锁上
    map_to_odom_mutex_.lock();

    //当前时间+transform_publish_period_
    ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);

    //发布变换,StampedTransform(变换矩阵，当前时间，父坐标系，子坐标系)
    tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, odom_frame_));

    //map_to_odom的互斥锁解锁
    map_to_odom_mutex_.unlock();
}

LaserMapping::~LaserMapping()
{
    //如果发布线程
    if(transform_thread_)
    {
        //join阻塞线程,直到线程结束
        transform_thread_->join();
        //删除该线程
        delete transform_thread_;
    }
    delete [] m_linePoints;
    if(range_laser_)
      delete range_laser_;
    if(odom_sensor_)
      delete odom_sensor_;
    if (scan_filter_)
      delete scan_filter_;
    if (scan_filter_sub_)
      delete scan_filter_sub_;
}

void LaserMapping::updateMap(const sensor_msgs::LaserScan &scan)
{
    ROS_DEBUG("Update map");
    //地图的智能锁
    boost::mutex::scoped_lock map_lock (map_mutex_);
    /*
     * 粒子滤波获取最佳位置，
     * 这里直接使用odom位置
     */

    //如果时第一次设置map，即got_map_为false,设置map的各个参数
    if(!got_map_) {
      map_.map.info.resolution = delta_;
      map_.map.info.origin.position.x = 0.0;
      map_.map.info.origin.position.y = 0.0;
      map_.map.info.origin.position.z = 0.0;
      map_.map.info.origin.orientation.x = 0.0;
      map_.map.info.origin.orientation.y = 0.0;
      map_.map.info.origin.orientation.z = 0.0;
      map_.map.info.origin.orientation.w = 1.0;
    }

    //map的中心
    LMapping::Point center;
    center.x=(xmin_ + xmax_) / 2.0;
    center.y=(ymin_ + ymax_) / 2.0;

    //建立ScanMap类
    LMapping::ScanMap smap(center, xmin_, ymin_, xmax_, ymax_,
                                  delta_);

    cout<<"建立ScanMap类"<<endl;
    for(int i=poseVector.size()-1;i>=0;i--)
    {
        invalidateActiveArea();
        computeActiveArea(smap, poseVector[i], &(laserVector[i][0]));
        registerScan(smap, poseVector[i], &(laserVector[i][0]));
    }
    cout<<map_.map.info.width<<endl;
    cout<<map_.map.info.height<<endl;
    cout<< (unsigned int) smap.getMapSizeX()<<endl;
    cout<<(unsigned int) smap.getMapSizeY()<<endl;
    // the map may have expanded, so resize ros message as well
    if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY())
    {
      LMapping::Point wmin = smap.map2world(LMapping::IntPoint(0, 0));
      LMapping::Point wmax = smap.map2world(LMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
      xmin_ = wmin.x; ymin_ = wmin.y;
      xmax_ = wmax.x; ymax_ = wmax.y;

      ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
                xmin_, ymin_, xmax_, ymax_);

      map_.map.info.width = smap.getMapSizeX();
      map_.map.info.height = smap.getMapSizeY();
      map_.map.info.origin.position.x = xmin_;
      map_.map.info.origin.position.y = ymin_;
      cout<<"建立ScanMap类11"<<endl;
      cout<<map_.map.info.width<<" "<<map_.map.info.height<<endl;
      map_.map.data.resize(map_.map.info.width*map_.map.info.height);
      cout<<"建立ScanMap类12"<<endl;
      ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
    }
   cout<<"建立ScanMap类2"<<endl;
    for(int x=0; x < smap.getMapSizeX(); x++)
    {
      for(int y=0; y < smap.getMapSizeY(); y++)
      {
        /// @todo Sort out the unknown vs. free vs. obstacle thresholding
        LMapping::IntPoint p(x, y);
        double occ=smap.cell(p);
        assert(occ <= 1.0);
        //occ<0代表未知区域，
        if(occ < 0)
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 50;
        else if(occ > occ_thresh_)
        {
          //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
          //100为障碍
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
        }
        else
          //0为可同行区域
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
      }
    }
    got_map_ = true;

    //make sure to set the header information on the map
    map_.map.header.stamp = ros::Time::now();
    map_.map.header.frame_id = tf_.resolve( map_frame_ );

    sst_.publish(map_.map);
    sstm_.publish(map_.map.info);
}

void LaserMapping::invalidateActiveArea()
{
    m_activeAreaComputed=false;
}
void LaserMapping::computeActiveArea(LMapping::ScanMap &map, const LMapping::OrientedPoint &p, const double *readings)
{
    if (m_activeAreaComputed)
        return;
    LMapping::OrientedPoint lp=p;
    lp.x+=cos(p.theta)*range_laser_->getPose().x-sin(p.theta)*range_laser_->getPose().y;
    lp.y+=sin(p.theta)*range_laser_->getPose().x+cos(p.theta)*range_laser_->getPose().y;
    lp.theta+=range_laser_->getPose().theta;

    LMapping::IntPoint p0=map.world2map(lp);

    LMapping::Point min(map.map2world(0,0));
    LMapping::Point max(map.map2world(map.getMapSizeX()-1,map.getMapSizeY()-1));

    if (lp.x<min.x) min.x=lp.x;
    if (lp.y<min.y) min.y=lp.y;
    if (lp.x>max.x) max.x=lp.x;
    if (lp.y>max.y) max.y=lp.y;


    unsigned int m_laserBeams=laser_beam_use_count_;
    /*determine the size of the area*/
    double m_laserAngles[2048];
    memcpy(m_laserAngles, &(laser_angles_use_[0]), sizeof(double)*laser_beam_use_count_);
    const double * angle=m_laserAngles+m_initialBeamsSkip;
    for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++)
    {
        if (*r>maxRange_||*r==0.0||std::isnan(*r))
            continue;
        double d=*r>maxUrange_?maxUrange_:*r;
        LMapping::Point phit=lp;
        phit.x+=d*cos(lp.theta+*angle);
        phit.y+=d*sin(lp.theta+*angle);
        if (phit.x<min.x) min.x=phit.x;
        if (phit.y<min.y) min.y=phit.y;
        if (phit.x>max.x) max.x=phit.x;
        if (phit.y>max.y) max.y=phit.y;
    }
    double m_enlargeStep=10.;
    if ( !map.isInside(min)	|| !map.isInside(max))
    {
        LMapping::Point lmin(map.map2world(0,0));
        LMapping::Point lmax(map.map2world(map.getMapSizeX()-1,map.getMapSizeY()-1));
        //cerr << "CURRENT MAP " << lmin.x << " " << lmin.y << " " << lmax.x << " " << lmax.y << endl;
        //cerr << "BOUNDARY OVERRIDE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
        min.x=( min.x >= lmin.x )? lmin.x: min.x-m_enlargeStep;
        max.x=( max.x <= lmax.x )? lmax.x: max.x+m_enlargeStep;
        min.y=( min.y >= lmin.y )? lmin.y: min.y-m_enlargeStep;
        max.y=( max.y <= lmax.y )? lmax.y: max.y+m_enlargeStep;
        map.resize(min.x, min.y, max.x, max.y);
        //cerr << "RESIZE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
    }

    LMapping::HierarchicalArray2D<LMapping::PointAccumulator>::PointSet activeArea;
    /*determine the size of the area*/
    angle=m_laserAngles+m_initialBeamsSkip;

    for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++)
    {
        if (m_generateMap)
        {
            double d=*r;
            if (d>maxRange_||d==0.0||std::isnan(d))
                continue;
            if (d>maxUrange_)
                d=maxUrange_;
            LMapping::Point phit=lp+LMapping::Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
            LMapping::IntPoint p0=map.world2map(lp);
            LMapping::IntPoint p1=map.world2map(phit);

            //IntPoint linePoints[20000] ;
            LMapping::GridLineTraversalLine line;
            line.points=m_linePoints;
            LMapping::GridLineTraversal::gridLine(p0, p1, &line);
            for (int i=0; i<line.num_points-1; i++)
            {
                assert(map.isInside(m_linePoints[i]));
                activeArea.insert(map.storage().patchIndexes(m_linePoints[i]));
                assert(m_linePoints[i].x>=0 && m_linePoints[i].y>=0);
            }
            if (d<maxUrange_){
                LMapping::IntPoint cp=map.storage().patchIndexes(p1);
                assert(cp.x>=0 && cp.y>=0);
                activeArea.insert(cp);
            }
        }
        else
        {
            if (*r>maxRange_||*r>maxUrange_||*r==0.0||std::isnan(*r))
                continue;
            LMapping::Point phit=lp;
            phit.x+=*r*cos(lp.theta+*angle);
            phit.y+=*r*sin(lp.theta+*angle);
            LMapping::IntPoint p1=map.world2map(phit);
            assert(p1.x>=0 && p1.y>=0);
            LMapping::IntPoint cp=map.storage().patchIndexes(p1);
            assert(cp.x>=0 && cp.y>=0);
            activeArea.insert(cp);
        }
    }
    map.storage().setActiveArea(activeArea, true);
    m_activeAreaComputed=true;
}

double LaserMapping::registerScan(LMapping::ScanMap &map, const LMapping::OrientedPoint &p, const double *readings)
{
    if (!m_activeAreaComputed)
        computeActiveArea(map, p, readings);

    //this operation replicates the cells that will be changed in the registration operation
    map.storage().allocActiveArea();

    LMapping::OrientedPoint lp=p;
    lp.x+=cos(p.theta)*range_laser_->getPose().x-sin(p.theta)*range_laser_->getPose().y;
    lp.y+=sin(p.theta)*range_laser_->getPose().x+cos(p.theta)*range_laser_->getPose().y;
    lp.theta+=range_laser_->getPose().theta;
    LMapping::IntPoint p0=map.world2map(lp);

    unsigned int m_laserBeams=laser_beam_use_count_;
    /*determine the size of the area*/
    double m_laserAngles[2048];
    memcpy(m_laserAngles, &(laser_angles_use_[0]), sizeof(double)*laser_beam_use_count_);
    const double * angle=m_laserAngles+m_initialBeamsSkip;
    double esum=0;
    for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++)
    {
        if (m_generateMap)
        {
            double d=*r;
            if (d>maxRange_||d==0.0||std::isnan(d))
                continue;
            if (d>maxUrange_)
                d=maxUrange_;
            LMapping::Point phit=lp+LMapping::Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
            LMapping::IntPoint p1=map.world2map(phit);
            LMapping::GridLineTraversalLine line;
            line.points=m_linePoints;
            LMapping::GridLineTraversal::gridLine(p0, p1, &line);
            for (int i=0; i<line.num_points-1; i++)
            {
                LMapping::PointAccumulator& cell=map.cell(line.points[i]);
                double e=-cell.entropy();
                cell.update(false, LMapping::Point(0,0));
                e+=cell.entropy();
                esum+=e;
            }
            if (d<maxUrange_)
            {
                double e=-map.cell(p1).entropy();
                map.cell(p1).update(true, phit);
                e+=map.cell(p1).entropy();
                esum+=e;
            }
        }
        else
        {
            if (*r>maxUrange_||*r>maxUrange_||*r==0.0||std::isnan(*r))
                continue;
            LMapping::Point phit=lp;
            phit.x+=*r*cos(lp.theta+*angle);
            phit.y+=*r*sin(lp.theta+*angle);
            LMapping::IntPoint p1=map.world2map(phit);
            assert(p1.x>=0 && p1.y>=0);
            map.cell(p1).update(true,phit);
        }
    }
    return esum;
}
