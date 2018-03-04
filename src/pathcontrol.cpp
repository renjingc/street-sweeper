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
    isGoTracking=true;
    isUpdateNodeMap=true;
    isFind=false;
    isFinish=false;
    startfind=false;
    worldDirection=0;
    aimWorldDirection=0;
    aimDirection=0;
    goNearId=1;
    distUpdatePoint=delta/8;

    minLaserAngle=-0.2618;
    maxLaserAngle=0.2618;
    maxfontRange_=2*delta;
    minfontRange_=delta;
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
    isGoTracking=true;
    isUpdateNodeMap=true;
    isPause=false;
    isFind=false;
    isFinish=false;
    startfind=false;
    worldDirection=0;
    aimWorldDirection=0;
    aimDirection=0;
    goNearId=1;
    distUpdatePoint=delta/8;

    minLaserAngle=-0.2618;
    maxLaserAngle=0.2618;
    maxfontRange_=2*delta;
    minfontRange_=delta;
}
PathControl::~PathControl()
{
    if(pnodeMap)
        delete pnodeMap;
}

LMapping::IntPoint PathControl::world2node(const LMapping::Point& p,
                                           const Position m_center,
                                           const double m_delta,
                                           const int m_sizeX2,
                                           const int m_sizeY2,
                                           int dir)
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
        {
            return LMapping::IntPoint( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2);
            if(isGoTracking)
                isUpdateNodeMap=true;
        }
       else
            return lastPoint;
    }
    else if(dir==1)
    {
        lastX=m_delta*(lastPoint.x-m_sizeX2)+m_center.x;
        lastY=m_delta*(lastPoint.y-1-m_sizeY2)+m_center.y;
        if(abs(currPosition.y-lastY)<distUpdatePoint)
        {
            return LMapping::IntPoint( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2);
            if(isGoTracking)
                isUpdateNodeMap=true;
        }
       else
            return lastPoint;
    }
    else if(dir==2)
    {
        lastX=m_delta*(lastPoint.x-1-m_sizeX2)+m_center.x;
        lastY=m_delta*(lastPoint.y-m_sizeY2)+m_center.y;
        if(abs(currPosition.x-lastX)<distUpdatePoint)
        {
            return LMapping::IntPoint( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2);
            if(isGoTracking)
                isUpdateNodeMap=true;
        }
        else
            return lastPoint;
    }
    else if(dir==3)
    {
        lastX=m_delta*(lastPoint.x-m_sizeX2)+m_center.x;
        lastY=m_delta*(lastPoint.y+1-m_sizeY2)+m_center.y;
        if(abs(currPosition.y-lastY)<distUpdatePoint)
        {
            return LMapping::IntPoint( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2);

            if(isGoTracking)
                isUpdateNodeMap=true;
        }
        else
            return lastPoint;
    }
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
    tCmd.linear.x=0;
    tCmd.angular.z=0;
    if(!isPause)
    {
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
    }
    return tCmd;
}
geometry_msgs::Twist PathControl::goDistance(double start,double end)
{

}
bool PathControl::judgeRotateToAim()
{
    if(isAngleRange(currPosition.theta,aimWorldDirection))
    {
        isUpdateNodeMap=true;
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

void PathControl::getBeamLaserPose(const double *readings,LMapping::OrientedPoint lp)
{
    if(beamLaser_poses.size()>0)
        beamLaser_poses.clear();
    double m_laserAngles[2048];
    unsigned int m_laserBeams=laser_beam_use_count_;
    memcpy(m_laserAngles, &(laser_angles_use_[0]), sizeof(double)*laser_beam_use_count_);
    const double * angle=m_laserAngles;
    int i=0;
    for (const double* r=readings; r<readings+m_laserBeams; r++, angle++,i++)
    {
        if (*r>minfontRange_||*r==0.0||std::isnan(*r))
            continue;
        //double d=*r>maxUrange_?maxUrange_:*r;
        double d=*r;
        LMapping::Point phit=lp;
        //得到每个点的世界坐标系，并得到当前最远最近的激光点x,y
        phit.x+=d*cos(lp.theta+*angle);
        phit.y+=d*sin(lp.theta+*angle);
        beamLaser_poses.push_back(phit);
    }
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

bool PathControl::getCurrPoint(const tf::StampedTransform &currPose)
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
                                           pnodeMap->getOrigin(),
                                           pnodeMap->getCellHigh()/10.0,
                                           pnodeMap->getCenterX(),
                                           pnodeMap->getCenterY(),
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
bool PathControl::findNearstNode(vector<PATH::MapSearchNode>& result)
{
    PATH::Point end = PATH::breadthFirstSearch(pnodeMap, PATH::Point(currPoint.y, currPoint.x));
    //地图坐标点
    if(end.getX()==-1&&end.getY()==-1)
        return false;
    PATH::MapSearchNode nodeStart,nodeEnd;
    //全转换成地图坐标系的点
    nodeStart.x=currPoint.y;nodeStart.y=currPoint.x;
    nodeEnd.x=end.getX();nodeEnd.y=end.getY();
    result.clear();
    //寻找路径，result为地图坐标点
    PATH::getPath(pnodeMap, nodeStart, nodeEnd, result);
    cout<<result.size()<<endl;
    if(result.size()<=1)
        return false;
    return true;
}

int PathControl::judgeAimRotate(int dx,int dy)
{
    //如果相等，则返回０
    int aimWorldDir;
    if(dx==0&&dy==0)
        return -1;
    //目标点在当前点的世界坐标系的前方，则返回0
    else if(dx==1)
        aimWorldDir=0;
    //目标点在当前点的世界坐标系的后方，则返回2
    else if(dx==-1)
        aimWorldDir=2;
    //目标点在当前点的世界坐标系的左方，则返回3
    else if(dy==1)
        aimWorldDir=3;
    //目标点在当前点的世界坐标系的右方，则返回1
    else if(dy==-1)
        aimWorldDir=1;
    return aimWorldDir;
}

bool PathControl::updatePlan(const tf::StampedTransform &currPose,
                             LMapping::OrientedPoint& laser_pose,
                             const double *readings,
                             geometry_msgs::Twist &cmd)
{
    getCurrPoint(currPose);
    judgeRotateToAim();
    getBeamLaserPose(readings,laser_pose);
    if(isGoTracking)
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
            }
        }
        //找到边缘
        else
        {
            if(pnodeMap->getCell(currPoint.y,currPoint.x)!=KNode)
                pnodeMap->setCell(currPoint.y,currPoint.x,hasGone);
           setNextRound(readings);
            //沿边行走
            if(!isAlongTheEdge&&!startfind)
            {
                //如果返回真,则说明是在沿边行走
                isAlongTheEdge=alongTheEdge(readings);
                historyPoint.push_back(currPoint);

            }
            else if(startfind)
            {
                //寻找最近的点
                LMapping::IntPoint nearPoint;
                //如果未找到，isFind==false，开始寻找
                if(!isFind)
                {   //如果找到了,则isFind＝true，则得到了路径
                    isFind=findNearstNode(planPath);
                    if(isFind)
                    {
                        isPause=false;
                        goNearId=1;
                    }
                    //如果未找到，则说明无路可走，走完了
                    if(!isFind)
                    {
                        isFinish=true;
                        isPause=true;
                    }
                }
                else
                {

                    //如果当前点位置已到达预计点，则规划下一点
                    if(currPoint.x==planPath[goNearId].y&&currPoint.y==planPath[goNearId].x)
                    {
                        goNearId++;
                    }
                    //如果规划的Id已走完
                    if(goNearId==planPath.size())
                    {
                        startfind=false;
                        isFind=false;
                        goNearId=1;
                        isGoTracking=true;
                        aimDirection=0;
                    }
                    else
                    {
                        int errDir;
                        int dx=planPath[goNearId].y-currPoint.x;
                        int dy=planPath[goNearId].x-currPoint.y;
                        int aimWorldDir=judgeAimRotate(dx,dy);
                        //如果当前点和下一点的方向不同，设置旋转方向
                        if(aimWorldDir!=worldDirection)
                        {
                            errDir=aimWorldDir-worldDirection;
                            //向转
                            if(errDir==-3)
                            {
                                errDir=1;
                            }
                            //向左转
                            else if(errDir==3)
                            {
                                errDir=-1;
                            }
                            if(errDir==1)
                            {
                                //向右转
                                 aimDirection=turnRight;
                                 isGoTracking=false;
                            }
                            else if(errDir==-1)
                            {
                                //向左转
                                aimDirection=turnLeft;
                                isGoTracking=false;
                            }
                            else if(abs(errDir)==2)
                            {
                                //如果右边不是障碍
                                if(!judgeAdjacent(readings,right,obstacle))
                                {
                                    //向右转
                                    aimDirection=turnRight;
                                    isGoTracking=false;
                                }
                                else if(!judgeAdjacent(readings,left,obstacle))
                                {
                                    //向左转
                                    aimDirection=turnLeft;
                                    isGoTracking=false;
                                }
                                else
                                {
                                    isGoTracking=true;
                                    aimDirection=0;
                                    goDirection=goBack;
                                }
                            }
                            setWorldAimDirection(aimDirection);
                        }
                        //直走，直到下一目标点
                        else
                        {
                            isGoTracking=true;
                            aimDirection=0;
                            goDirection=goStraight;
                        }
                        cout<<"isGoTracking: "<<isGoTracking
                           <<" errDir: "<<errDir
                          <<" aimWorldDir: "<<aimWorldDir
                           <<" aimDirection: "<<aimDirection
                          <<" goDirection: "<<goDirection
                         <<" worldDirection: "<<worldDirection
                           <<" aimWorldDirection: "<<aimWorldDirection
                          <<" currPoint: "<<currPoint.x<<" "<<currPoint.y<<endl;
                    }
                }
            }
//            //遍历走
//            else
//            {
//                traversing(readings);
//            }
        }
        cmd=goTracking(goDirection);
    }
    else
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
    tCmd.linear.x=0.01;
    tCmd.angular.z=0;
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
    //如果是某种类型，则返回true。否则返回false
//    if(direction==forward&&beamLaser_poses.size()>10)
//        return true;

    direction+=worldDirection;
    direction=direction%4;
    if(direction==forward)
    {
        if(pnodeMap->getCell(currPoint.y, currPoint.x+1)==type)
            return true;
        else
            return false;
    }
    else if(direction==right)
    {
        if(pnodeMap->getCell(currPoint.y-1, currPoint.x)==type)
            return true;
        else
            return false;
    }
    else if(direction==Backward)
    {
        if(pnodeMap->getCell(currPoint.y, currPoint.x-1)==type)
            return true;
        else
            return false;
    }
    else if(direction==left)
    {
        if(pnodeMap->getCell(currPoint.y+1, currPoint.x)==type)
            return true;
        else
            return false;
    }
}

bool PathControl::alongTheEdge(const double *readings)
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
        aimDirection=turnLeft;
        setWorldAimDirection(aimDirection);
        choice=3;
    }
    else if(!judgeAdjacent(readings,Backward,obstacle))//&&!judgeAdjacent(readings,Backward,hasGone))
    {
        isPause=true;
        aimDirection=0;
        startfind=true;
        isGoTracking=true;
        //goDirection=goBack;
        choice=4;
    }
//    else
//    {
//        isPause=true;
//        startfind=true;
//    }
    //cout<<worldDirection<<" "<<aimDirection<<" "<<aimWorldDirection<<" "<<currPoint.x<<" "<<currPoint.y<<" "<<choice<<endl;
    return false;
}
bool PathControl::traversing(const double *readings)
{

}

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double ran_gaussian(double sigma)
{
  double x1, x2, w, r;
  do
  {
        do { r = drand48(); } while (r==0.0);
        x1 = 2.0 * r - 1.0;
        do { r = drand48(); } while (r==0.0);
        x2 = 2.0 * r - 1.0;
        w = x1*x1 + x2*x2;
  } while(w > 1.0 || w==0.0);

  return(sigma * x2 * sqrt(-2.0*log(w)/w));
}
