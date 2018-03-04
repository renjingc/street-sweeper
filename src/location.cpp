#include "location.h"

Location::Location()
{
    hl=new HoughLine();
    intrinsic_Matrix.create(3,3,CV_64F);
    distortion_coeffs.create(1, 5, CV_64F);

    state.create(stateNum, 1, CV_32FC1);
    processNoise.create(stateNum, stateNum, CV_32F);
    measurement = Mat::zeros(measureNum, measureNum, CV_32F);
    kFilter_init();

    ifInit=false;
    period=0.17;
    errOdomTh=0.0;
    errImuTh=0.0;
    filterAngle=0.0;
    filterAngle_v=0.0;
    errCurAndLastImuX=0.0;
    errCurAndLastImuY=0.0;
    errCurAndLastImuTh=0.0;

    outfile.open(outFileName);
}
Location::Location(string fileName)
{
    hl=new HoughLine();
    loadParam(fileName);
    hl->cols = 180;                                                                                                               //霍夫图像列数
    hl->rows = 2 * sqrt((hl->w / 2)*(hl->w / 2) + (hl->h / 2)*(hl->h / 2));                                                  //霍夫图像行数

    p.x=0;p.y=0;p.th=0;p.currentCrossPoint.x=0;p.currentCrossPoint.y=0;
    ifInit=false;
    intrinsic_Matrix.create(3,3,CV_64F);
    distortion_coeffs.create(1, 5, CV_64F);

    state.create(stateNum, 1, CV_32FC1);
    processNoise.create(stateNum, stateNum, CV_32F);
    measurement = Mat::zeros(measureNum, measureNum, CV_32F);
    kFilter_init();

    ifInit=false;
    period=0.17;
    errOdomTh=0.0;
    errImuTh=0.0;
    filterAngle=0.0;
    filterAngle_v=0.0;
    errCurAndLastImuX=0.0;
    errCurAndLastImuY=0.0;
    errCurAndLastImuTh=0.0;

    outfile.open(outFileName);
    loadCamera(cameraFileName);
}

Location::Location(string fileName,string outDataPath)
{
    hl=new HoughLine();
    loadParam(fileName);
    hl->cols = 180;                                                                                                               //霍夫图像列数
    hl->rows = 2 * sqrt((hl->w / 2)*(hl->w / 2) + (hl->h / 2)*(hl->h / 2));                                                  //霍夫图像行数

    p.x=0;p.y=0;p.th=0;p.currentCrossPoint.x=0;p.currentCrossPoint.y=0;
    ifInit=false;
    intrinsic_Matrix.create(3,3,CV_64F);
    distortion_coeffs.create(1, 5, CV_64F);

    state.create(stateNum, 1, CV_32FC1);
    processNoise.create(stateNum, stateNum, CV_32F);
    measurement = Mat::zeros(measureNum, measureNum, CV_32F);
    kFilter_init();

    ifInit=false;
    period=0.17;
    errOdomTh=0.0;
    errImuTh=0.0;
    filterAngle=0.0;
    filterAngle_v=0.0;
    errCurAndLastImuX=0.0;
    errCurAndLastImuY=0.0;
    errCurAndLastImuTh=0.0;

//    if(!strtool::mkpath(outDataPath,0777))
//    {
        strtool::mkpath(outDataPath,0777);
        outFileName=outDataPath+"/resultData.txt";
        logLineAndCrossFileName=outDataPath+"/logLineAndCross.txt";
        logUpdateDataFileName=outDataPath+"/logUpdateData.txt";
        logCrossDataFileName=outDataPath+"/logCrossData.txt";
        outfile.open(outFileName);
        logLineAndCrossFile.open(logLineAndCrossFileName);
        logUpdateDataFile.open(logUpdateDataFileName);
        logCrossDataFile.open(logCrossDataFileName);
//    }

    loadCamera(cameraFileName);
}
Location::~Location()
{
    outfile.close();
    logLineAndCrossFile.close();
    logUpdateDataFile.close();
    logCrossDataFile.close();

    delete hl;
}
bool Location::setOutFile(string filePath)
{
    outfile.close();
    logLineAndCrossFile.close();
    logUpdateDataFile.close();
    logCrossDataFile.close();

    if(!strtool::mkpath(filePath,0777))
    {
        outFileName=filePath+"/resultData.txt";
        logLineAndCrossFileName=filePath+"/logLineAndCross.txt";
        logUpdateDataFileName=filePath+"/logUpdateData.txt";
        logCrossDataFileName=filePath+"/logCrossData.txt";
        outfile.open(outFileName);
        logLineAndCrossFile.open(logLineAndCrossFileName);
        logUpdateDataFile.open(logUpdateDataFileName);
        logCrossDataFile.open(logCrossDataFileName);
    }
}

bool Location::loadParam(string fileName)
{
    ifstream conf_file(fileName);
    if (!conf_file.is_open())
    {
        cerr << "fail to load " << fileName << endl;
        exit(1);
    }
    string line;
    vector<int> data;
    while (conf_file.good())
    {
        getline(conf_file, line);
        istringstream line_s(line);
        string field;
        line_s >> field;
        if (field.compare("threshold1:")==0)
        {
            line_s>>hl->threshold1;
        }
        if (field.compare("threshold2:")==0)
        {
            line_s>>hl->threshold2;
        }
        else if (field.compare("w:")==0)
        {
            line_s>>hl->w;
        }
        else if (field.compare("h:")==0)
        {
            line_s>>hl->h;
        }
        else if (field.compare("thetaThres:")==0)
        {
            line_s>>hl->thetaThres;
        }
        else if (field.compare("distThres:")==0)
        {
            line_s>>hl->distThres;
        }
        else if (field.compare("eddging:")==0)
        {
            line_s>>hl->eddging;
        }
        else if (field.compare("findRange:")==0)
        {
            line_s>>hl->findRange;
        }
        else if (field.compare("rotateRange:")==0)
        {
            line_s>>hl->rotateRange;
        }
        else if (field.compare("lineRange:")==0)
        {
            line_s>>hl->lineRange;
        }
        else if (field.compare("crossRange:")==0)
        {
            line_s>>hl->crossRange;
        }
        else if (field.compare("floorH:")==0)
        {
            line_s>>floorH;
        }
        else if (field.compare("mainAndSecondRoate:")==0)
        {
            line_s>>hl->mainAndSecondRoate;
        }
        else if (field.compare("outFileName:")==0)
        {
            line_s>>outFileName;
        }
        else if(field.compare("L:")==0)
        {
            line_s>>L;
        }
        else if(field.compare("T:")==0)
        {
            line_s>>T;
        }
        else if(field.compare("fz:")==0)
        {
            line_s>>fz;
        }
        else if(field.compare("maxScore:")==0)
        {
            line_s>>hl->maxScore;
        }
        else if(field.compare("cameraFileName:")==0)
        {
            line_s>>cameraFileName;
        }
        else if(field.compare("alpha:")==0)
        {
            line_s>>hl->alpha;
        }
        else if(field.compare("topScore:")==0)
        {
            line_s>>hl->topScore;
        }
        else if(field.compare("crossMaxScore:")==0)
        {
            line_s>>hl->crossMaxScore;
        }
    }
    cout<<"load param:"<<fileName<<" finish"<<endl;
    return true;
}

bool Location::transformCross(Point cross)
{
    errPoint.x=(cross.x-hl->w/2)/(double)(intrinsic_Matrix.at<double>(0,0))*fz;
    errPoint.y=(cross.y-hl->h/2)/(double)(intrinsic_Matrix.at<double>(1,1))*fz;
    return true;
}
bool Location::loadCamera(string fileName)
{
    ifstream conf_file(fileName);
    if (!conf_file.is_open())
    {
        cerr << "fail to load " << fileName << endl;
        exit(1);
    }

    string line;
    vector<int> data;
    while (conf_file.good())
    {
        getline(conf_file, line);
        istringstream line_s(line);
        string field;
        line_s >> field;
        if (field.compare("camera_matrix:")==0)
        {
            double a;
            for (int i = 0; i < 9; i++)
            {
                line_s >> a;
                intrinsic_Matrix.at<double>(i/3,i%3)=a;
                cout<<(double)intrinsic_Matrix.at<double>(i/3,i%3)<<endl;
            }
        }
        if (field.compare("distortion:")==0)
        {
            double a;
            for (int i = 0; i < 5; i++)
            {
                line_s >> a;
                distortion_coeffs.at<double>(0,i)=a;
                cout<<(double)distortion_coeffs.at<double>(0,i)<<endl;
            }
        }
    }
    cout<<"load camera:"<<fileName<<" finish"<<endl;
    return true;
}

bool Location::InitializeCoordinates(Vec4f mainLine,Vec4f secondLine)
{
    //计算画面中心列与主线的角度，逆时针为正0~PI,顺时针为负-0~-PI
    Vec3f data1=hl->transformLineToPolar(mainLine);
    if(data1[0]<PI/2&&data1[0]>0)
        data1[0]=-data1[0];
    else if(data1[0]<PI&&data1[0]>PI/2)
        data1[0]=PI-data1[0];
    else
        data1[0]=data1[0];

    //车体初始角度
    initP.th=-data1[0];
    p.th=-data1[0];

    //imu和odom和实际车体角度的偏差
    errImuTh=imuP.th-p.th;
    imuP.th=imuP.th-errImuTh;
    if(imuP.th>PI)
    {
        imuP.th=-2*PI+imuP.th;
    }
    else if(imuP.th<-PI)
    {
        imuP.th=2*PI+imuP.th;
    }
    errOdomTh=odomP.th-p.th;
    odomP.th=p.th;

    //计算初始位置
    double x,y;
    x=(errPoint.y * cos(data1[0]) + errPoint.x * sin(data1[0]))/1000.0;
    y=(-errPoint.y * sin(data1[0]) + errPoint.x * cos(data1[0]))/1000.0;
    //初始化位置
    initP.x=x-L*cos(data1[0]);
    initP.y=y-L*sin(data1[0]);
    p.x=x-L*cos(data1[0]);
    p.y=y-L*sin(data1[0]);
    odomP.x=x-L*cos(data1[0]);
    odomP.y=y-L*sin(data1[0]);
    imuP.x=x-L*cos(data1[0]);
    imuP.y=y-L*sin(data1[0]);

    lastCrossImuP.x=imuP.x;
    lastCrossImuP.y=imuP.y;
    lastCrossImuP.th=imuP.th;

    //初始交叉点栅格位置
    initP.currentCrossPoint.x=0;
    initP.currentCrossPoint.y=0;
    p.currentCrossPoint.x=0;
    p.currentCrossPoint.y=0;

    //获取当前时间
    updated_ = ros::Time::now();

    cout<<initP.th<<endl;
    cout<<"currentCrossPoint: "<<p.fourDirection<<" "<<p.currentCrossPoint.x<<","<<p.currentCrossPoint.y<<endl;
}

bool Location::updateFourDirection()
{
    double thresTh=PI/4;

    errCurAndLastImuTh=imuP.th-lastCrossImuP.th;
    if((imuP.th>=0&&lastCrossImuP.th>=0)||(imuP.th<=0&&lastCrossImuP.th<=0))
        errCurAndLastImuTh=abs(imuP.th-lastCrossImuP.th);
    else if(imuP.th<0&&lastCrossImuP.th>0)
        errCurAndLastImuTh=-imuP.th+lastCrossImuP.th;
    else if(imuP.th>0&&lastCrossImuP.th<0)
        errCurAndLastImuTh=imuP.th-lastCrossImuP.th;
    if(errCurAndLastImuTh>PI)
    {
        errCurAndLastImuTh=abs(2*PI-errCurAndLastImuTh);
    }

    if(errCurAndLastImuTh>(thresTh-PI/32)&&errCurAndLastImuTh<PI)//(thresTh+PI/16))
    {
        string imuPString=strtool::numToString<int>(hl->frameNum)+" updateForth: "+strtool::numToString<int>(hl->frameNum)+" "+strtool::numToString<double>(errCurAndLastImuTh);
        logUpdateDataFile<<imuPString<<endl;

        if((imuP.th>=0&&lastCrossImuP.th>=0)||(imuP.th<=0&&lastCrossImuP.th<=0))
        {
            if(imuP.th>lastCrossImuP.th)
            {
                p.fourDirection--;//向左转
            }
            if(imuP.th<lastCrossImuP.th)
            {
                p.fourDirection++;//向右转
            }
        }
        else if(imuP.th<0&&lastCrossImuP.th>0)
        {
             p.fourDirection++;//向右转
        }
        else if(imuP.th>0&&lastCrossImuP.th<0)
        {
             p.fourDirection--;//向左转
        }
        if(p.fourDirection>3)
            p.fourDirection=0;
        else if(p.fourDirection<0)
            p.fourDirection=3;
    }

}

bool Location::updateGrid()
{
    //计算当前车体所在栅格位置点
    double errCrossX=(errPoint.y-lastErr.y)/1000;
    //cout<<errCrossX<<endl;
    //double errCrossY=errPoint.x-lastErr.x;
    errCurAndLastImuX=abs(imuP.x-lastCrossImuP.x);
    errCurAndLastImuY=abs(imuP.y-lastCrossImuP.y);
    errCurAndLastImuTh=abs(imuP.th-lastCrossImuP.th);
    if((imuP.th>=0&&lastCrossImuP.th>=0)||(imuP.th<=0&&lastCrossImuP.th<=0))
        errCurAndLastImuTh=abs(imuP.th-lastCrossImuP.th);
    else if(imuP.th<0&&lastCrossImuP.th>0)
        errCurAndLastImuTh=-imuP.th+lastCrossImuP.th;
    else if(imuP.th>0&&lastCrossImuP.th<0)
        errCurAndLastImuTh=imuP.th-lastCrossImuP.th;
    if(errCurAndLastImuTh>PI)
    {
        errCurAndLastImuTh=abs(2*PI-errCurAndLastImuTh);
    }

    double dist=sqrt(errCurAndLastImuX*errCurAndLastImuX+errCurAndLastImuY*errCurAndLastImuY);
    double thresX=floorH-(double)480/(double)(intrinsic_Matrix.at<double>(0,0))*fz/1000.0-0.08;
    double thresY=floorH-(double)480/(double)(intrinsic_Matrix.at<double>(0,0))*fz/1000.0-0.08;
    double thresTh1=PI/4,thresTh2=PI/2;

    string imuPString=strtool::numToString<int>(hl->frameNum)+"  errImuP: "+strtool::numToString<double>(errCurAndLastImuX)+" "+strtool::numToString<double>(errCurAndLastImuY)+" "+strtool::numToString<double>(errCurAndLastImuTh);
    logUpdateDataFile<<imuPString<<endl;

    if(errCurAndLastImuTh>(thresTh1-PI/32)&&errCurAndLastImuTh<(thresTh2+PI/32))//(thresTh+PI/16))
    {
        if((imuP.th>=0&&lastCrossImuP.th>=0)||(imuP.th<=0&&lastCrossImuP.th<=0))
        {
            if(imuP.th>lastCrossImuP.th)
            {
                p.fourDirection--;//向左转
            }
            if(imuP.th<lastCrossImuP.th)
            {
                p.fourDirection++;//向右转
            }
        }
        else if(imuP.th<0&&lastCrossImuP.th>0)
        {
             p.fourDirection++;//向右转
        }
        else if(imuP.th>0&&lastCrossImuP.th<0)
        {
             p.fourDirection--;//向左转
        }
        if(p.fourDirection>3)
            p.fourDirection=0;
        else if(p.fourDirection<0)
            p.fourDirection=3;
    }
    else if(errCurAndLastImuTh>(PI-thresTh1-PI/32)&&errCurAndLastImuTh<(PI+PI/32))
    {
        if((imuP.th>=0&&lastCrossImuP.th>=0)||(imuP.th<=0&&lastCrossImuP.th<=0))
        {
            if(imuP.th>lastCrossImuP.th)
            {
                p.fourDirection-=2;//向左转
            }
            if(imuP.th<lastCrossImuP.th)
            {
                p.fourDirection+=2;//向右转
            }
        }
        else if(imuP.th<0&&lastCrossImuP.th>0)
        {
             p.fourDirection+=2;//向右转
        }
        else if(imuP.th>0&&lastCrossImuP.th<0)
        {
             p.fourDirection-=2;//向左转
        }
        if(p.fourDirection>3)
            p.fourDirection=0;
        else if(p.fourDirection<0)
            p.fourDirection=3;
    }
    if(errCurAndLastImuX>thresX||errCurAndLastImuY>thresY||(errCurAndLastImuTh>(thresTh1-PI/32)&&errCurAndLastImuTh<PI))
    {
        string imuPString=strtool::numToString<int>(hl->frameNum)+" errXYThP: "+strtool::numToString<double>(errCurAndLastImuX)+" "+strtool::numToString<double>(errCurAndLastImuY)+" "+strtool::numToString<double>(errCurAndLastImuTh)
                +" "+strtool::numToString<double>(errPoint.y/1000)+" "+strtool::numToString<double>(lastErr.y/1000)+" "+strtool::numToString<double>(errCrossX);
        logUpdateDataFile<<imuPString<<endl;
        int count=0;
        if(errCurAndLastImuTh>(thresTh1-PI/32)&&errCurAndLastImuTh<PI)
        {
            count=1;
        }
        if(p.fourDirection==0)
        {
                errCurAndLastImuX-=errCrossX;
                while(errCurAndLastImuX>=(floorH-0.08))
                {
                    errCurAndLastImuX-=floorH;
                    count++;
                }
                p.currentCrossPoint.x+=count;
        }
        else if(p.fourDirection==1)
        {
            errCurAndLastImuY-=errCrossX;
            while(errCurAndLastImuY>=(floorH-0.08))
            {
                errCurAndLastImuY-=floorH;
                count++;
            }
            p.currentCrossPoint.y-=count;
        }
        if(p.fourDirection==2)
        {
            errCurAndLastImuX-=errCrossX;
            while(errCurAndLastImuX>=(floorH-0.08))
            {
                errCurAndLastImuX-=floorH;
                count++;
            }
            p.currentCrossPoint.x-=count;
        }
        else if(p.fourDirection==3)
        {
            errCurAndLastImuY-=errCrossX;
            while(errCurAndLastImuY>=(floorH-0.08))
            {
                errCurAndLastImuY-=floorH;
                count++;
            }
            p.currentCrossPoint.y+=count;
        }
        string crossString=strtool::numToString<int>(hl->frameNum)+" cross: "+strtool::numToString<int>(p.fourDirection)+" "+strtool::numToString<int>(p.currentCrossPoint.x)+" "+strtool::numToString<int>(p.currentCrossPoint.y);
        logUpdateDataFile<<crossString<<endl;
        logCrossDataFile<<crossString<<endl;

        cout<<"currentCrossPoint: "<<p.fourDirection<<" "<<p.currentCrossPoint.x<<","<<p.currentCrossPoint.y<<endl;
    }
    else
    {
    }
    lastErr.x=errPoint.x;
    lastErr.y=errPoint.y;

    lastCrossImuP.x=imuP.x;
    lastCrossImuP.y=imuP.y;
    lastCrossImuP.th=imuP.th;
}
bool Location::updateTime()
{
    //计算间隔时间
    period = (ros::Time::now() - updated_).toSec();
    //更新时间
    updated_ = ros::Time::now();
}

bool Location::updateTime(double _p)
{
    //计算间隔时间
    period = _p;
}

bool Location::updateImuP()
{
    //计算在当前坐标系下当前IMU的角度和使用IMU获得的里程计位置
    imuP.th=imuP.th-errImuTh;
    if(imuP.th>PI)
    {
        imuP.th=-2*PI+imuP.th;
    }
    else if(imuP.th<-PI)
    {
        imuP.th=2*PI+imuP.th;
    }

    double delta_Imux = (odom.vx * cos(imuP.th) - odom.vy * sin(imuP.th))*period;
    double delta_Imuy = (odom.vx * sin(imuP.th) + odom.vy * cos(imuP.th))*period;
    imuP.x+=delta_Imux;
    imuP.y+=delta_Imuy;
}
bool Location::updateOdomP()
{
    //计算在当前坐标系下当前里程计的角度和位置
    double delta_x = (odom.vx * cos(odomP.th) - odom.vy * sin(odomP.th))*period;
    double delta_y = (odom.vx * sin(odomP.th) + odom.vy * cos(odomP.th))*period;
    double delta_th = odom.vth*period;
    odomP.x+=delta_x;
    odomP.y+=delta_y;
    odomP.th+=delta_th;
    if(odomP.th>PI)
        odomP.th=-2*PI+odomP.th;
    else if(odomP.th<-PI)
        odomP.th=2*PI+odomP.th;
}
bool Location::updateMotion()
{
    updateOdomP();
    updateImuP();
}

bool Location::updateCoordinatesWithCross(Vec4f mainLine)
{
    //利用交叉点计算车体位姿
    double x,y;
    Vec3f data1=hl->transformLineToPolar(mainLine);
    if(data1[0]<PI/2&&data1[0]>0)
        data1[0]=-data1[0];
    else if(data1[0]<PI&&data1[0]>PI/2)
        data1[0]=PI-data1[0];
    else
        data1[0]=data1[0];
   double errTh;
   if(p.fourDirection==0)
   {
            p.th=-data1[0];
            errTh=-imuP.th;
    }
    if(p.fourDirection==1)
    {
            p.th=-data1[0]-PI/2;
            if(p.th>-PI)
                p.th=PI+(p.th-PI);

            errTh=-(imuP.th+PI/2);
    }
   if(p.fourDirection==2)
   {
            if(data1[0]>0)
                p.th=PI-data1[0];
            else if(data1[0]<0)
                p.th=-PI-data1[0];
            else
                p.th=PI;

            if(imuP.th>0)
                errTh=PI-imuP.th;
            else if(imuP.th<0)
                errTh=-PI-imuP.th;
    }
    if(p.fourDirection==3)
    {
            p.th=-data1[0]+PI/2;
            if(p.th>PI)
                p.th=-PI+(p.th-PI);

            errTh=-(imuP.th-PI/2);
    }
    x=(errPoint.y * cos(data1[0]) + errPoint.x * sin(data1[0]))/1000.0;
    y=(-errPoint.y * sin(data1[0]) + errPoint.x * cos(data1[0]))/1000.0;
    if(p.fourDirection==0)
    {
            p.x=p.currentCrossPoint.x*floorH+x;
            p.y=p.currentCrossPoint.y*floorH+y;
    }
    else if(p.fourDirection==1)
    {
            p.x=p.currentCrossPoint.x*floorH+y;
            p.y=p.currentCrossPoint.y*floorH-x;
    }
    else if(p.fourDirection==2)
    {
            p.x=p.currentCrossPoint.x*floorH-x;
            p.y=p.currentCrossPoint.y*floorH-y;
    }
    else if(p.fourDirection==3)
    {
            p.x=p.currentCrossPoint.x*floorH-y;
            p.y=p.currentCrossPoint.y*floorH+x;
    }

    p.x=p.x-L*cos(p.th);
    p.y=p.y-L*sin(p.th);

    kFilter_update(period);
    double errFilterTh;
    if(p.fourDirection==0)
    {
             errFilterTh=-filterAngle;
     }
     if(p.fourDirection==1)
     {
             errFilterTh=-(filterAngle+PI/2);
     }
    if(p.fourDirection==2)
    {
             if(filterAngle>0)
                 errFilterTh=PI-filterAngle;
             else if(filterAngle<0)
                 errFilterTh=-PI-filterAngle;
     }
     if(p.fourDirection==3)
     {
             errFilterTh=-(filterAngle-PI/2);
     }
    double filterX=(errPoint.y * cos(data1[0]) + errPoint.x * sin(data1[0]))/1000.0;
    double filterY=(-errPoint.y * sin(data1[0]) + errPoint.x * cos(data1[0]))/1000.0;
    if(p.fourDirection==0)
    {
            filterPosition.x=p.currentCrossPoint.x*floorH+filterX;
            filterPosition.y=p.currentCrossPoint.y*floorH+filterY;
    }
    else if(p.fourDirection==1)
    {
            filterPosition.x=p.currentCrossPoint.x*floorH+filterY;
            filterPosition.y=p.currentCrossPoint.y*floorH-filterX;
    }
    else if(p.fourDirection==2)
    {
            filterPosition.x=p.currentCrossPoint.x*floorH-filterX;
            filterPosition.y=p.currentCrossPoint.y*floorH-filterY;
    }
    else if(p.fourDirection==3)
    {
            filterPosition.x=p.currentCrossPoint.x*floorH-filterY;
            filterPosition.y=p.currentCrossPoint.y*floorH+filterX;
    }
    filterPosition.x=filterPosition.x-L*cos(filterAngle);
    filterPosition.y=filterPosition.y-L*sin(filterAngle);


    lastOdomP.x=odomP.x;
    lastOdomP.y=odomP.y;
    lastOdomP.th=odomP.th;

    lastImuP.x=imuP.x;
    lastImuP.y=imuP.y;
    lastImuP.th=imuP.th;

    lastP.x=p.x;
    lastP.y=p.y;
    lastP.th=p.th;
}
bool Location:: updateCoordinatesWithMainLine(Vec4f mainLine)
{
    //利用主线和里程计计算车体位姿
    Vec3f data1=hl->transformLineToPolar(mainLine);
     if(data1[0]<PI/2&&data1[0]>0)
         data1[0]=-data1[0];
     else if(data1[0]<PI&&data1[0]>PI/2)
         data1[0]=PI-data1[0];
     else
         data1[0]=data1[0];

    if(p.fourDirection==0)
    {
        p.th=-data1[0];
    }
    if(p.fourDirection==1)
    {
        p.th=-data1[0]-PI/2;
        if(p.th>-PI)
            p.th=PI+(p.th-PI);
    }
    if(p.fourDirection==2)
    {
          if(data1[0]>0)
              p.th=PI-data1[0];
          else if(data1[0]<0)
              p.th=-PI-data1[0];
          else
              p.th=PI;
    }
    if(p.fourDirection==3)
    {
        p.th=-data1[0]+PI/2;
        if(p.th>PI)
            p.th=-PI+(p.th-PI);
    }

    double delta_x1 = (odom.vx * cos(p.th) - odom.vy * sin(p.th))*period;
    double delta_y1= (odom.vx * sin(p.th) + odom.vy * cos(p.th))*period;

    p.x+=delta_x1;
    p.y+=delta_y1;

    kFilter_update(period);
    double deltafilter_x = (odom.vx * cos(filterAngle) - odom.vy * sin(filterAngle))*period;
    double deltafilter_y= (odom.vx * sin(filterAngle) + odom.vy * cos(filterAngle))*period;

    filterPosition.x+=deltafilter_x;
    filterPosition.y+=deltafilter_y;


    lastOdomP.x=odomP.x;
    lastOdomP.y=odomP.y;
    lastOdomP.th=odomP.th;

    lastImuP.x=imuP.x;
    lastImuP.y=imuP.y;
    lastImuP.th=imuP.th;

    lastP.x=p.x;
    lastP.y=p.y;
    lastP.th=p.th;
}
bool Location::updateCoordinates()
{
    //根据上一时刻和当前时刻的imu角度差，来累计计算当前车体的角度和位置
//    double errTh=imuP.th-lastImuP.th;
    double errTh=imuO.vth*period;
    /*if((imuP.th>=0&&lastImuP.th>=0)||(imuP.th<=0&&lastImuP.th<=0))
        errTh=abs(imuP.th-lastImuP.th);
    else if(imuP.th<0&&lastImuP.th>0)
        errTh=-imuP.th+lastImuP.th;
    else if(imuP.th>0&&lastImuP.th<0)
        errTh=imuP.th-lastImuP.th;
    if(errTh>PI)
    {
        errTh=abs(2*PI-errTh);
    }*/
    double delta_x1 = (odom.vx * cos(p.th) - odom.vy * sin(p.th))*period;
    double delta_y1= (odom.vx * sin(p.th) + odom.vy * cos(p.th))*period;

     p.x+=delta_x1;
     p.y+=delta_y1;
     p.th+=errTh;
     while(p.th>PI+0.1)
     {
         p.th-=PI;
     }
     while(p.th<(-PI-0.1))
     {
         p.th+=PI;
     }

     kFilter_update(period);
     double deltafilter_x = (odom.vx * cos(filterAngle) - odom.vy * sin(filterAngle))*period;
     double deltafilter_y= (odom.vx * sin(filterAngle) + odom.vy * cos(filterAngle))*period;

     filterPosition.x+=deltafilter_x;
     filterPosition.y+=deltafilter_y;



     lastOdomP.x=odomP.x;
     lastOdomP.y=odomP.y;
     lastOdomP.th=odomP.th;

     lastImuP.x=imuP.x;
     lastImuP.y=imuP.y;
     lastImuP.th=imuP.th;

     lastP.x=p.x;
     lastP.y=p.y;
     lastP.th=p.th;

}
void Location::kFilter_init()
{
    kf.init(stateNum,measureNum,0);

    setIdentity(kf.measurementMatrix);
    //!< process noise covariance matrix (Q)
    // wk 是过程噪声，并假定其符合均值为零，协方差矩阵为Qk(Q)的多元正态分布;
    setIdentity(kf.processNoiseCov, Scalar::all(1e-4));

    //vk 是观测噪声，其均值为零，协方差矩阵为Rk,且服从正态分布;
    setIdentity(kf.measurementNoiseCov, Scalar::all(1e-4));
    kf.measurementNoiseCov.at<float>(0,0)=0.003;
    kf.measurementNoiseCov.at<float>(1,1)=0.0004363323129985824;

    //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/  A代表F: transitionMatrix
    //预测估计协方差矩阵;
    setIdentity(kf.errorCovPost, Scalar::all(0.1));

    kf.transitionMatrix.at<float>(0,0)=1;
    kf.transitionMatrix.at<float>(0,1)=T;
    kf.transitionMatrix.at<float>(1,0)=0;
    kf.transitionMatrix.at<float>(0,1)=1;

    kf.statePost= Mat::zeros(stateNum, stateNum, CV_32F);
}
void Location::kFilter_update(double period)
{
    //1.kalman prediction
    Mat prediction = kf.predict();
    if((double)prediction.at<float>(0)>PI)
    {
        prediction.at<float>(0)=(double)(-2*PI+(double)prediction.at<float>(0));
    }
    else if((double)prediction.at<float>(0)<-PI)
    {
        prediction.at<float>(0)=(double)(2*PI+(double)prediction.at<float>(0));
    }
    filterAngle=(double)prediction.at<float>(0);
    filterAngle_v=(double)prediction.at<float>(1);

    kf.transitionMatrix.at<float>(0,1)=period;

    //2.update measurement
    Mat temp5=kf.measurementMatrix*kf.statePre;
    measurement.at<float>(0)= (float)p.th;
    measurement.at<float>(1) = (float)imuO.vth;
    if(abs(measurement.at<float>(0)-temp5.at<float>(0,0))>PI)
    {
        if(measurement.at<float>(0)<0)
        {
            measurement.at<float>(0)=2*PI+p.th;
        }
        else if(p.th>0)
        {
            measurement.at<float>(0)=-2*PI+p.th;
        }
    }

    //3.update
    kf.correct(measurement);
}

bool Location::updateAll(Mat image)
{
    Vec4f mainLine,secondLine;
    Point cross;
    bool ifFindMainLine,ifFindSecondLine,ifFindCross;
    updateTime();
    updateMotion();

    hl->findLinesAndCross(image,mainLine,secondLine,cross,imuO.vth*RAD2DEG*period,ifFindMainLine,ifFindSecondLine,ifFindCross);
    hl->showLine(image,mainLine,secondLine,cross,ifFindMainLine,ifFindSecondLine,ifFindCross);
    //计算栅格坐标
    if(ifFindCross)
    {
        transformCross(cross);
        if(ifInit)
        {
            updateGrid();
        }
    }
    //定位
    if(ifFindCross&&hl->ifRotate==0)
    {
        string lineAndCrossString=strtool::numToString<int>(hl->frameNum)+"  lineAndCross: "+strtool::numToString<float>(mainLine[0])+" "+strtool::numToString<float>(mainLine[1])+" "+strtool::numToString<float>(mainLine[2])+" "+strtool::numToString<float>(mainLine[3])
                                                +" "+strtool::numToString<float>(hl->curMainMaxIDx)+" "+strtool::numToString<float>(hl->lastMainMaxIDx)
                                                +" "+strtool::numToString<float>(secondLine[0])+" "+strtool::numToString<float>(secondLine[1])+" "+strtool::numToString<float>(secondLine[2])+" "+strtool::numToString<float>(secondLine[3])
                                                +" "+strtool::numToString<int>(cross.x)+" "+strtool::numToString<int>(cross.y)
                                                +" "+strtool::numToString<float>(hl->mainMax)+" "+strtool::numToString<float>(hl->secondMax);
        logLineAndCrossFile<<lineAndCrossString<<endl;

        if(!ifInit)
        {
            InitializeCoordinates(mainLine,secondLine);
            ifInit=true;

            string outString=strtool::numToString<int>(hl->frameNum)+"   init: "+strtool::numToString<double>(imuP.x)+" "+strtool::numToString<double>(imuP.y)+" "+strtool::numToString<double>(imuP.th)
                                           +" "+strtool::numToString<double>(p.x)+" "+strtool::numToString<double>(p.y)+" "+strtool::numToString<double>(p.th);
            outfile<<outString<<endl;
        }
        else
        {
            updateCoordinatesWithCross(mainLine);

            string outString=strtool::numToString<int>(hl->frameNum)+"   cross: "+strtool::numToString<double>(imuP.x)+" "+strtool::numToString<double>(imuP.y)+" "+strtool::numToString<double>(imuP.th)
                                           +" "+strtool::numToString<double>(p.x)+" "+strtool::numToString<double>(p.y)+" "+strtool::numToString<double>(p.th);
            outfile<<outString<<endl;
         }
     }
//     else if(ifFindMainLine&&hl->ifRotate==0)
//     {
//        if(ifInit)
//        {
//            updateCoordinatesWithMainLine(mainLine);

//            string lineAndCrossString=strtool::numToString<int>(hl->frameNum)+"  mainLine: "+strtool::numToString<float>(mainLine[0])+" "+strtool::numToString<float>(mainLine[1])+" "+strtool::numToString<float>(mainLine[2])+" "+strtool::numToString<float>(mainLine[3])
//                                                    +" "+strtool::numToString<float>(hl->curMainMaxIDx)+" "+strtool::numToString<float>(hl->lastMainMaxIDx)
//                                                    +" "+strtool::numToString<float>(hl->mainMax)+" "+strtool::numToString<float>(hl->secondMax);

//            string outString=strtool::numToString<int>(hl->frameNum)+"   main: "+strtool::numToString<double>(imuP.x)+" "+strtool::numToString<double>(imuP.y)+" "+strtool::numToString<double>(imuP.th)
//                                           +" "+strtool::numToString<double>(p.x)+" "+strtool::numToString<double>(p.y)+" "+strtool::numToString<double>(p.th);

//            logLineAndCrossFile<<lineAndCrossString<<endl;
//            outfile<<outString<<endl;
//         }
//      }
      else
      {
          if(ifInit)
          {
             updateCoordinates();

             string outString=strtool::numToString<int>(hl->frameNum)+"   imu: "+strtool::numToString<double>(imuP.x)+" "+strtool::numToString<double>(imuP.y)+" "+strtool::numToString<double>(imuP.th)
                                            +" "+strtool::numToString<double>(p.x)+" "+strtool::numToString<double>(p.y)+" "+strtool::numToString<double>(p.th);
              outfile<<outString<<endl;
          }
      }
}

bool Location::updateAll(Mat image,double _p)
{
    Vec4f mainLine,secondLine;
    Point cross;
    bool ifFindMainLine,ifFindSecondLine,ifFindCross;
    updateTime(_p);
    updateMotion();

    hl->findLinesAndCross(image,mainLine,secondLine,cross,imuO.vth*RAD2DEG*period,ifFindMainLine,ifFindSecondLine,ifFindCross);
    hl->showLine(image,mainLine,secondLine,cross,ifFindMainLine,ifFindSecondLine,ifFindCross);

    //计算栅格坐标
    if(ifFindCross)
    {
        transformCross(cross);
        if(ifInit)
        {
            updateGrid();
        }
    }
    //定位
    if(ifFindCross&&hl->ifRotate==0)
    {
        string lineAndCrossString=strtool::numToString<int>(hl->frameNum)+"  lineAndCross: "+strtool::numToString<float>(mainLine[0])+" "+strtool::numToString<float>(mainLine[1])+" "+strtool::numToString<float>(mainLine[2])+" "+strtool::numToString<float>(mainLine[3])
                                                +" "+strtool::numToString<float>(hl->curMainMaxIDx)+" "+strtool::numToString<float>(hl->lastMainMaxIDx)
                                                +" "+strtool::numToString<float>(secondLine[0])+" "+strtool::numToString<float>(secondLine[1])+" "+strtool::numToString<float>(secondLine[2])+" "+strtool::numToString<float>(secondLine[3])
                                                +" "+strtool::numToString<int>(cross.x)+" "+strtool::numToString<int>(cross.y)
                                                +" "+strtool::numToString<float>(hl->mainMax)+" "+strtool::numToString<float>(hl->secondMax);
        logLineAndCrossFile<<lineAndCrossString<<endl;

        if(!ifInit)
        {
            InitializeCoordinates(mainLine,secondLine);
            ifInit=true;

            string outString=strtool::numToString<int>(hl->frameNum)+"   init: "+strtool::numToString<double>(imuP.x)+" "+strtool::numToString<double>(imuP.y)+" "+strtool::numToString<double>(imuP.th)
                                           +" "+strtool::numToString<double>(p.x)+" "+strtool::numToString<double>(p.y)+" "+strtool::numToString<double>(p.th);
            outfile<<outString<<endl;
        }
        else
        {
            updateCoordinatesWithCross(mainLine);

            string outString=strtool::numToString<int>(hl->frameNum)+"   cross: "+strtool::numToString<double>(imuP.x)+" "+strtool::numToString<double>(imuP.y)+" "+strtool::numToString<double>(imuP.th)
                                           +" "+strtool::numToString<double>(p.x)+" "+strtool::numToString<double>(p.y)+" "+strtool::numToString<double>(p.th);
            outfile<<outString<<endl;
        }
     }
    /*
     else if(ifFindMainLine&&hl->ifRotate==0)
     {
        if(ifInit)
        {
            updateCoordinatesWithMainLine(mainLine);

            string lineAndCrossString=strtool::numToString<int>(hl->frameNum)+"  mainLine: "+strtool::numToString<float>(mainLine[0])+" "+strtool::numToString<float>(mainLine[1])+" "+strtool::numToString<float>(mainLine[2])+" "+strtool::numToString<float>(mainLine[3])
                                                    +" "+strtool::numToString<float>(hl->curMainMaxIDx)+" "+strtool::numToString<float>(hl->lastMainMaxIDx)
                                                    +" "+strtool::numToString<float>(hl->mainMax)+" "+strtool::numToString<float>(hl->secondMax);

            string outString=strtool::numToString<int>(hl->frameNum)+"   main: "+strtool::numToString<double>(imuP.x)+" "+strtool::numToString<double>(imuP.y)+" "+strtool::numToString<double>(imuP.th)
                                           +" "+strtool::numToString<double>(p.x)+" "+strtool::numToString<double>(p.y)+" "+strtool::numToString<double>(p.th);

            logLineAndCrossFile<<lineAndCrossString<<endl;
            outfile<<outString<<endl;
         }
      }*/
      else
      {
          if(ifInit)
          {
            updateCoordinates();

            string outString=strtool::numToString<int>(hl->frameNum)+"   imu: "+strtool::numToString<double>(imuP.x)+" "+strtool::numToString<double>(imuP.y)+" "+strtool::numToString<double>(imuP.th)
                                           +" "+strtool::numToString<double>(p.x)+" "+strtool::numToString<double>(p.y)+" "+strtool::numToString<double>(p.th);
             outfile<<outString<<endl;
          }
      }

}
