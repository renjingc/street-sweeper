#include "pathcontrol.h"

using namespace std;
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
PathControl::PathControl()
{
    got_map_ = false;
    maxHeight=-1;
    maxWidth=-1;
    delta=0.6;
    angle_P=0.1;

    isFirst=true;
    isFindRight=false;
    isAlongTheEdge=false;
    worldDirection=0;
    aimWorldDirection=0;
    aimDirection=0;
    distUpdatePoint=delta/8;

    minLaserAngle=-0.3925;
    maxLaserAngle=0.3925;
    maxRange_=0.5;
}
PathControl::PathControl(int maxHeight_,int maxWidth_,double delta_,
                         double vx_,double vAngle_,double angle_P_):
    maxHeight(maxHeight_),maxWidth(maxWidth_),delta(delta_),vx(vx_),vAngle(vAngle_),angle_P(angle_P_)
{
    got_map_ = false;
    maxHeight=-1;
    maxWidth=-1;
    //angle_P=0.1;
    //vx=0.1;

    isFirst=true;
    isFindRight=false;
    isAlongTheEdge=false;
    worldDirection=0;
    isPause=false;
    aimWorldDirection=0;
    aimDirection=0;
    distUpdatePoint=delta/8;

    minLaserAngle=-0.3925;
    maxLaserAngle=0.3925;
    maxRange_=0.5;

}
PathControl::~PathControl()
{
    if(pnodeMap)
        delete pnodeMap;
}
LMapping::IntPoint PathControl::world2map(const LMapping::Point& p,
                                          const LMapping::Point m_center,
                                          const double m_delta,
                                          const int m_sizeX2,
                                          const int m_sizeY2,
                                          int dir) const
{
    //dir为世界坐标系下的方向
    //朝前走的时候,将右边的栅格作为当前栅格,即ｙ方向的负方向四舍五入-1,x方向上的向下取整
    if (dir==0)
        return LMapping::IntPoint( (int)floor((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2-1);
    //向右走的时候，将x方向负方向作为当前栅格，x方向四舍五入-1,y方向向上取整
    else if(dir==1)
        return LMapping::IntPoint( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2-1, (int)ceil((p.y-m_center.y)/m_delta)+m_sizeY2);
    //朝后走的时候,将ｙ方向的正方向作为当前栅格，即y方向四舍五入+1,x方向上的向上取整
    if (dir==2)
        return LMapping::IntPoint( (int)ceil((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2);
    //向左走的时候，将x方向正方向作为当前栅格，x方向四舍五入,y方向向下取整
    else if(dir==3)
        return LMapping::IntPoint( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)floor((p.y-m_center.y)/m_delta)+m_sizeY2-1);
}
LMapping::IntPoint PathControl::world2node(const LMapping::Point& p,
                                           const LMapping::Point m_center,
                                           const double m_delta,
                                           const int m_sizeX2,
                                           const int m_sizeY2,
                                           int dir) const
{
    double lastX;//=m_delta*(lastPoint.x-m_sizeX2)+m_center.x;
    double lastY;//=m_delta*(lastPoint.y-m_sizeY2)+m_center.y;

    if(!got_map_)
        return LMapping::IntPoint( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2);
    if(dir==0)
    {
        lastX=m_delta*(lastPoint.x+1-m_sizeX2)+m_center.x;
        lastY=m_delta*(lastPoint.y-m_sizeY2)+m_center.y;
        if(abs(currPosition.x-lastX)<distUpdatePoint)
            return LMapping::IntPoint( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2);
        else
            return lastPoint;
    }
    else if(dir==1)
    {
        lastX=m_delta*(lastPoint.x-m_sizeX2)+m_center.x;
        lastY=m_delta*(lastPoint.y-1-m_sizeY2)+m_center.y;
        if(abs(currPosition.y-lastY)<distUpdatePoint)
            return LMapping::IntPoint( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2);
        else
            return lastPoint;
    }
    else if(dir==2)
    {
        lastX=m_delta*(lastPoint.x-1-m_sizeX2)+m_center.x;
        lastY=m_delta*(lastPoint.y-m_sizeY2)+m_center.y;
        if(abs(currPosition.x-lastX)<distUpdatePoint)
            return LMapping::IntPoint( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2);
        else
            return lastPoint;
    }
    else if(dir==3)
    {
        lastX=m_delta*(lastPoint.x-m_sizeX2)+m_center.x;
        lastY=m_delta*(lastPoint.y+1-m_sizeY2)+m_center.y;
        if(abs(currPosition.y-lastY)<distUpdatePoint)
            return LMapping::IntPoint( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2);
        else
            return lastPoint;
    }
}
void PathControl::world2NodeMap()
{
}

bool PathControl::isAngleRange(double angle,int dir)
{
    double angleRange=M_PI/48;
    if(dir==0)
    {
        if(angle<=angleRange&&angle>=-angleRange)
            return true;
        else
            return false;
    }
    else if(dir==1)
    {
        if(angle<=-M_PI/2+angleRange&&angle>=-M_PI/2-angleRange)
            return true;
        else
            return false;
    }
    else if(dir==2)
    {
        if((angle>=M_PI-angleRange&&angle<=M_PI)
            ||(angle>=-M_PI&&angle<=-M_PI+angleRange))
            return true;
        else
            return false;
    }
    else if(dir==3)
    {
        if(angle<=M_PI/2+angleRange&&angle>=M_PI/2-angleRange)
            return true;
        else
            return false;
    }
}
geometry_msgs::Twist PathControl::goTracking(int dir)
{
    geometry_msgs::Twist tCmd;
    int deltaInt=static_cast<int>(delta*100);
    //y方向跟踪
    if(worldDirection%2)
    {
        int aim=round((currPosition.x*100)/deltaInt);
        double aimX=aim*delta;
        double error=aimX-currPosition.x;
        //向前走
        if(dir==goStraight)
        {
            tCmd.linear.x=vx;
        }
        //向后走
        else if(dir==goBack)
        {
            tCmd.linear.x=-vx;
        }
        if(worldDirection==1)
        {
            tCmd.angular.z=error*angle_P;
        }
        else if(worldDirection==3)
        {
            tCmd.angular.z=-error*angle_P;
        }
    }
    //x方向跟踪
    else
    {
        int aim=round((currPosition.y*100)/deltaInt);
        double aimY=aim*delta;
        double error=aimY-currPosition.y;
        //向前走
        if(dir==goStraight)
        {
            tCmd.linear.x=vx;
        }
        else if(dir==goBack)
        {
            tCmd.linear.x=-vx;
        }
        if(worldDirection==0)
        {
            tCmd.angular.z=error*angle_P;
        }
        else if(worldDirection==2)
        {
            tCmd.angular.z=-error*angle_P;
        }
    }
    return tCmd;
}
geometry_msgs::Twist PathControl::goDistance(double start,double end)
{
    geometry_msgs::Twist t;
    if(start<end)
    {

    }
}
bool PathControl::judgeRotateToAim()
{
    //
    if(isAngleRange(currPosition.theta,aimWorldDirection))
    {
        isGoTracking=true;
        worldDirection=aimWorldDirection;
        return true;
    }
    return false;
}
void PathControl::setWorldAimDirection(int dir)
{
    aimWorldDirection=dir+worldDirection;
    if(aimWorldDirection<0)
        aimWorldDirection=aimWorldDirection+4;
    else if(aimWorldDirection>3)
        aimWorldDirection=aimWorldDirection-4;
}
void PathControl::setMap(nav_msgs::OccupancyGrid_<std::allocator<void> >& map_)
{
    if(!got_map_)
    {
        map.info.resolution = map_.info.resolution;
        map.info.origin.position.x = 0.0;
        map.info.origin.position.y = 0.0;
        map.info.origin.position.z = 0.0;
        map.info.origin.orientation.x = 0.0;
        map.info.origin.orientation.y = 0.0;
        map.info.origin.orientation.z = 0.0;
        map.info.origin.orientation.w = 1.0;

        map.info.width=map_.info.width;
        map.info.height=map_.info.height;
        map.info.origin.position.x = map_.info.origin.position.x;
        map.info.origin.position.y = map_.info.origin.position.y;
        map.data.resize(map.info.width*map.info.height);

        showMap.info.resolution = map_.info.resolution;
        showMap.info.origin.position.x = 0.0;
        showMap.info.origin.position.y = 0.0;
        showMap.info.origin.position.z = 0.0;
        showMap.info.origin.orientation.x = 0.0;
        showMap.info.origin.orientation.y = 0.0;
        showMap.info.origin.orientation.z = 0.0;
        showMap.info.origin.orientation.w = 1.0;

        showMap.info.width=map_.info.width;
        showMap.info.height=map_.info.height;
        showMap.info.origin.position.x = map_.info.origin.position.x;
        showMap.info.origin.position.y = map_.info.origin.position.y;
        showMap.data.resize(showMap.info.width*showMap.info.height);
        got_map_=true;

        pnodeMap=new NodeGridMap(int(map_.info.width+1),(int)(map_.info.height+1),delta*10,delta*10);
    }
    if(map.info.width!=map_.info.width ||map.info.height!=map_.info.height)
    {

    }
    if(got_map_)
    {
        //cout<<pnodeMap->getXCount()<<" "<<pnodeMap->getYCount()<<endl;
        for(int x=0; x < map_.info.height; x++)
        {
          for(int y=0; y < map_.info.width; y++)
          {
              if(map_.data[MAP_IDX(map_.info.width, x, y)]==80)
              {
                  map.data[MAP_IDX(map_.info.width, x, y)]=obstacle;
                  showMap.data[MAP_IDX(map_.info.width, x, y)]=80;
                  if(pnodeMap->getCell(y,x)!=hasGone)
                    pnodeMap->setCell(y,x,obstacle);
                  if(pnodeMap->getCell(y+1,x)!=hasGone)
                    pnodeMap->setCell(y+1,x,obstacle);
                  if(pnodeMap->getCell(y,x+1)!=hasGone)
                    pnodeMap->setCell(y,x+1,obstacle);
                  if(pnodeMap->getCell(y+1,x+1)!=hasGone)
                    pnodeMap->setCell(y+1,x+1,obstacle);
              }
//              else
//              {
//                  pnodeMap->setCell(y,x,noGone);
//              }
          }
        }
    }
    //pnodeMap=new NodeGridMap(20,20,5.0,5.0);
}
void PathControl::getBeamLaserPose(const double *readings,LMapping::OrientedPoint lp)
{
    if(beamLaser_poses.size()>10)
        beamLaser_poses.clear();
    double m_laserAngles[2048];
    unsigned int m_laserBeams=laser_beam_use_count_;
    memcpy(m_laserAngles, &(laser_angles_use_[0]), sizeof(double)*laser_beam_use_count_);
    const double * angle=m_laserAngles;
    for (const double* r=readings; r<readings+m_laserBeams; r++, angle++)
    {
        if (*r>maxRange_||*r==0.0||std::isnan(*r))
            continue;
        //double d=*r>maxUrange_?maxUrange_:*r;
        double d=*r;
        LMapping::Point phit=lp;
        //得到每个点的世界坐标系，并得到当前最远最近的激光点x,y
        phit.x+=d*cos(lp.theta+*angle);
        phit.y+=d*sin(lp.theta+*angle);
        beamLaser_poses.push_back(phit);
    }
    //cout<<beamLaser_poses.size()<<endl;
    //cout<<"beamLaser_poses "<<beamLaser_poses.size()<<endl;
}

void PathControl::setLaser(std::vector<double> laser_angles_)

{    bool findMin=false,findMax=false;
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

     laser_angles_use_.resize(laser_beam_use_count_);
     for(int i=minLaserIndex;i<=maxLaserIndex;i++)
     {
         laser_angles_use_[i-minLaserIndex]=laser_angles_[i];
     }
}

bool PathControl::getCurrPoint(LMapping::ScanMap smap,const tf::StampedTransform &currPose)
{
    geometry_msgs::Quaternion q;
    q.x=currPose.getRotation().getX();
    q.y=currPose.getRotation().getY();
    q.z=currPose.getRotation().getZ();
    q.w=currPose.getRotation().getW();
    currPosition.x=currPose.getOrigin().x();
    currPosition.y=currPose.getOrigin().y();
    currPosition.theta=tf::getYaw(q);

    currPoint=world2node(currPosition,
                                           smap.getCenter(),
                                           smap.getDelta(),
                                           smap.getSizeX2(),
                                           smap.getSizeY2(),
                                           getWorldDirection());
    lastPoint=currPoint;
}

void PathControl::setNode(int dir, int type)
{
    if(type==obstacle||type==hasGone)
    {
        pnodeMap->setCell(currPoint.y,currPoint.x,type);
    }
    else if(type==nextRound||type==gate)
    {
        int direction=dir+worldDirection;
        direction=direction%4;
        if(direction==forward)
            pnodeMap->setCell(currPoint.y,currPoint.x+1,type);
        else if(direction==right)
            pnodeMap->setCell(currPoint.y-1,currPoint.x,type);
        else if(direction==Backward)
            pnodeMap->setCell(currPoint.y,currPoint.x-1,type);
        else if(direction==left)
            pnodeMap->setCell(currPoint.y+1,currPoint.x,type);
    }
}

void PathControl::setNextRound(const double *readings)
{
    if(!judgeAdjacent(readings,left,obstacle))
    {
        if(!judgeAdjacent(readings,left,nextRound)
                &&!judgeAdjacent(readings,left,hasGone)
                &&!judgeAdjacent(readings,left,KNode))
        {
            setNode(left,nextRound);
        }
        /*else if(judgeAdjacent(readings,left,nextRound))
        {
            setNode(left,gate);
        }*/
    }
    else
    {

    }
}

bool PathControl::updatePlan(LMapping::ScanMap smap,
                             nav_msgs::OccupancyGrid_<std::allocator<void> >& map_,
                             const tf::StampedTransform &currPose,
                             LMapping::OrientedPoint& laser_pose,
                             const double *readings,
                             geometry_msgs::Twist &cmd)
{
    getCurrPoint(smap,currPose);
    setMap(map_);
    judgeRotateToAim();
    getBeamLaserPose(readings,laser_pose);
    if(isGoTracking&&!isPause)
    {
        //将初始位置的左边为K点,并且只有当碰到边缘时
        if(isFirst&&isFindRight)
        {
            if(!judgeAdjacent(readings,left,obstacle))
            {
                K=currPoint;
                K.y++;
                pnodeMap->setCell(K.y,K.x,KNode);
                isFirst=false;
            }
        }
        //如果未碰到前侧障碍，则向右转
        if(!isFindRight)
        {
            //向前走
            isPause=false;
            //判断前方栅格是否是障碍
            if(judgeAdjacent(readings,forward,obstacle))
            {
                //寻找到障碍,设置找到右边缘
                isFindRight=true;
                //停止向前
                isGoTracking=false;
                //设置移动方向
                aimDirection=turnLeft;//向左转
                //设置目标方向
                setWorldAimDirection(aimDirection);
                return true;
            }
            else
            {
                goDirection=goStraight;
                //showMap.data[MAP_IDX(map_.info.width, currPoint.x, currPoint.y)] = 250;
            }
        }
        //找到边缘
        else
        {
            //沿边行走
            if(!isAlongTheEdge)
            {
                //如果返回真,则说明是在沿边行走
                isAlongTheEdge=alongTheEdge(smap,map_,readings);
                showMap.data[MAP_IDX(map_.info.width, currPoint.x, currPoint.y)] = 250;
                map.data[MAP_IDX(map_.info.width, currPoint.x, currPoint.y)] = hasGone;
                if(pnodeMap->getCell(currPoint.y,currPoint.x)!=KNode)
                    pnodeMap->setCell(currPoint.y,currPoint.x,hasGone);
                setNextRound(readings);
            }
            //遍历走
            else
            {
                traversing(smap,map_,readings);
            }
        }
        cmd=goTracking(goDirection);
    }
    else if(!isPause)
    {
        //旋转,aimDirection为喜欢转方向
        cmd=turn(aimDirection);
    }
    pnodeMap->show();

    return true;
}
geometry_msgs::Twist PathControl::turn(int dir)
{
    geometry_msgs::Twist tCmd;
    //dir为旋转方向,-1为向左转,1为向右转
    if(dir==turnLeft)
    {
        tCmd.linear.x=0.01;
        tCmd.angular.z=vAngle;
    }
    else if(dir==turnRight)
    {
        tCmd.linear.x=0.01;
        tCmd.angular.z=-vAngle;
    }
    return tCmd;
}

bool PathControl::judgeAdjacent(const double *readings, int direction,int type)
{
    if(direction==forward&&beamLaser_poses.size()>10)
        return true;

    direction+=worldDirection;
    direction=direction%4;
    if(direction==forward)
    {
        //if(map.data[MAP_IDX(map.info.width, currPoint.x+1, currPoint.y)]==type)
        if(pnodeMap->getCell(currPoint.y, currPoint.x+1)==type)
            return true;
        else
            return false;
    }
    else if(direction==right)
    {
        //if(map.data[MAP_IDX(map.info.width, currPoint.x, currPoint.y-1)]==type)
        if(pnodeMap->getCell(currPoint.y-1, currPoint.x)==type)
            return true;
        else
            return false;
    }
    else if(direction==Backward)
    {
        //if(map.data[MAP_IDX(map.info.width, currPoint.x-1, currPoint.y)]==type)
        if(pnodeMap->getCell(currPoint.y, currPoint.x-1)==type)
            return true;
        else
            return false;
    }
    else if(direction==left)
    {
        //if(map.data[MAP_IDX(map.info.width, currPoint.x, currPoint.y+1)]==type)
        if(pnodeMap->getCell(currPoint.y+1, currPoint.x)==type)
            return true;
        else
            return false;
    }
}

bool PathControl::alongTheEdge(LMapping::ScanMap smap,
                               nav_msgs::OccupancyGrid_<std::allocator<void> >& map_,
                               const double *readings)
{
    int choice=0;
    //如果右边无障碍，则向右转
    if(!judgeAdjacent(readings,right,obstacle)&&!judgeAdjacent(readings,right,hasGone))
    {
        isGoTracking=false;
        aimDirection=turnRight;
        setWorldAimDirection(aimDirection);
        choice=1;
    }
    //如果右方有障碍,前方无障碍，向前
    else if(!judgeAdjacent(readings,forward,obstacle)&&!judgeAdjacent(readings,forward,hasGone))
    {
        isGoTracking=true;
        aimDirection=0;
        goDirection=goStraight;
        choice=2;
    }
    //如果右方和前方都有障碍，则向左
    else if(!judgeAdjacent(readings,left,obstacle)&&!judgeAdjacent(readings,left,hasGone))
    {
        isGoTracking=false;
        aimDirection=0;
        aimDirection=turnLeft;
        setWorldAimDirection(aimDirection);
        choice=3;
    }
    else if(!judgeAdjacent(readings,left,obstacle))
    {
        isPause=true;
        isGoTracking=true;
        goDirection=goBack;
        choice=4;
    }
    cout<<worldDirection<<" "<<aimDirection<<" "<<aimWorldDirection<<" "<<currPoint.x<<" "<<currPoint.y<<" "<<choice<<endl;
}
bool PathControl::traversing(LMapping::ScanMap smap,
                             nav_msgs::OccupancyGrid_<std::allocator<void> >& map_,
                             const double *readings)
{

}
