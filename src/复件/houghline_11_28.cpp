#include "houghline.h"

    HoughLine::HoughLine()
    {
        cols = 180;                                                                                                               //霍夫图像列数
        rows = 2 * sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));                                                  //霍夫图像行数

        frameNum=0;                                                                                                         //图像帧数
        ifRotate=0;                                                                                                               //是否旋转
        ifFollowFind=false;                                                                                                  //是否进入跟随

        lastMainMaxIDx=0;
        lastMainMaxIDy=0;
        curMainMaxIDx=0;
        curMainMaxIDy=0;

        mainMax=0;
        secondMax=0;

        mainLineCount=0;
        crossCount=0;
        secondLineCount=0;

        constfeatureHist.resize(121);
        constLBPfeatureHist.resize(59);

        constSecondfeatureHist.resize(121);
        constSecondLBPfeatureHist.resize(59);

        constCrossfeatureHist.resize(121);
        constCrossLBPfeatureHist.resize(59);

    }

    HoughLine::HoughLine(int _w,int _h, int _threshold1,int _threshold2,int _distThres=20,int _thetaThres=20):w(_w),h(_h),threshold1(_threshold1),threshold2(_threshold2),distThres(_distThres),thetaThres(_thetaThres)
    {
          findRange=40;
          rotateRange=20;
          eddging=20;

          lineRange=21;
          mainAndSecondRoate=85;

          cols = 180;                                                                                                               //霍夫图像列数
          rows = 2 * sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));                                                  //霍夫图像行数

          frameNum=0;                                                                                                         //图像帧数
          ifRotate=0;                                                                                                               //是否旋转
          ifFollowFind=false;                                                                                                  //是否进入跟随


          lastMainMaxIDx=0;
          lastMainMaxIDy=0;
          curMainMaxIDx=0;
          curMainMaxIDy=0;

          mainMax=0;
          secondMax=0;

          mainLineCount=0;
          crossCount=0;
          secondLineCount=0;

          constfeatureHist.resize(121);
          constLBPfeatureHist.resize(59);

          constSecondfeatureHist.resize(121);
          constSecondLBPfeatureHist.resize(59);

          constCrossfeatureHist.resize(121);
          constCrossLBPfeatureHist.resize(59);
    }

    HoughLine::HoughLine(string paramFile)
    {
        cols = 180;                                                                                                               //霍夫图像列数
        rows = 2 * sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));                                                  //霍夫图像行数

        frameNum=0;                                                                                                         //图像帧数
        ifRotate=0;                                                                                                               //是否旋转
        ifFollowFind=false;                                                                                                  //是否进入跟随

        lastMainMaxIDx=0;
        lastMainMaxIDy=0;
        curMainMaxIDx=0;
        curMainMaxIDy=0;

        mainMax=0;
        secondMax=0;

        mainLineCount=0;
        crossCount=0;
        secondLineCount=0;

        constfeatureHist.resize(121);
        constLBPfeatureHist.resize(59);

        constSecondfeatureHist.resize(121);
        constSecondLBPfeatureHist.resize(59);

        constCrossfeatureHist.resize(121);
        constCrossLBPfeatureHist.resize(59);

    }
    void HoughLine::imageFilter(Mat src, Mat &dst)
    {
        Mat kernel = (Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
        cv::filter2D(src, dst, src.depth(), kernel);
    }

    bool HoughLine::pretreatment(Mat frame,Mat& dst)
    {
        Mat image;
        resize(frame, frame, Size(640, 480));

        cvtColor(frame, gray, CV_BGR2GRAY);
        //bilateralFilter(image,image1,10, 10*2, 10/2);//双边滤波器

        imageFilter(gray,image);//锐化

        Mat contours;
        Canny(image, contours, 5, 10); // Apply canny edge//可选canny算子
        Mat contoursInv;
        cv::threshold(contours, contoursInv, 128, 255, cv::THRESH_BINARY_INV);//why
        image.copyTo(dst);
    }

    bool HoughLine::findLsd(Mat image,Mat& dst,vector<Vec4f> &lines_std)
    {
        image.copyTo(dst);
        Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE, 1.5, 0.6, 4.0, 42.5, 0.0, 0.8);
        // Detect the lines
        ls->detect(image, lines_std);//这里把检测到的直线线段都存入了lines_std中，4个float的值，分别为起止点的坐标
        ls->drawSegments(dst, lines_std);
    }
    void HoughLine::showLine(Mat image, Vec4f mainLine,Vec4f secondLine, Point &cross, bool ifFindMainLine, bool ifFindSecondLine, bool ifFindCross)
    {
        Mat imageShow;
        image.copyTo(imageShow);
        if(ifFindMainLine)
        {
            line(imageShow,Point(mainLine[0],mainLine[1]),Point(mainLine[2],mainLine[3]),Scalar(0,0,255),5);
        }
        if(ifFindSecondLine)
        {
            line(imageShow,Point(secondLine[0],secondLine[1]),Point(secondLine[2],secondLine[3]),Scalar(0,255,0),5);
        }
        if(ifFindCross)
        {
            circle(imageShow,cross,5,Scalar(0,0,0),5);
        }
        imshow("LineImage", imageShow);
    }

    bool HoughLine::findLinesAndCross(Mat image, Vec4f& mainLine,Vec4f& secondLine, Point &cross,double vth,
                                      bool& ifFindMainLine,bool& ifFindSecondLine,bool& ifFindCross,int& fourDirection)
    {
        Mat image1,image2,image_gaussian;
        vector<Vec4f> out_lines;
        vector<Vec4f> lines_std;
        ifFindMainLine=false;
        ifFindSecondLine=false;
        ifFindCross=false;
        pretreatment(image,image1);
        findLsd(image1,image2,lines_std);
        vector<Vec3f> data=Transform(lines_std);
        TransformImage(data,image_gaussian);

        getLBP();
        ifFindMainLine=findMainLine(image,image_gaussian,mainLine,vth,ifRotate,fourDirection);
        //寻找横线
        if(ifFindMainLine)
        {
            ifFindSecondLine=findSecondLine(image,image_gaussian,mainLine,secondLine);
        }
        //计算十字交叉点
        if(ifFindMainLine&&ifFindSecondLine)
            ifFindCross=getCrossPoint(mainLine,secondLine,cross);
        else
            ifFindCross=false;
        if(ifFindCross)
        {
            ifFindCross=judgeCrossPoint(image,cross);
        }
    }

    Vec4f HoughLine::transformPolarToLine(float maxIDx, float maxIDy)
    {
        //cout<<max<<endl;

        float x1, x2, y1, y2;
        float tempp = (maxIDx / 180.0 * PI), temppp = atan((float)h / w);
        if (tempp < temppp || tempp > PI - temppp)
        {
            y1 = -h / 2;
            y2 = h / 2;
            float temp = cos(maxIDx / 180.0 * PI);
            if (temp == 0)
                temp = 0.00001;
            x1 = (maxIDy - y1*sin(maxIDx / 180.0 * PI)) / temp;
            x2 = (maxIDy - y2*sin(maxIDx / 180.0 * PI)) / temp;
            x1=x1+w/2;
            x2=x2+w/2;
            y1=y1+h / 2;
            y2=y2+h / 2;
            if(x1>w||x1<0)
            {
                double k;
                if(x2!=x1)
                {
                    k=(y2-y1)/(x2-x1);
                }
                else
                {
                    k=(y2-y1)/(x2-x1+0.00001);
                }
                if(x1>w)
                {
                    x1=w;
                    y1=y2-(x2-x1)*k;
                }
                else if(x1<0)
                {
                    x1=0;
                    y1=y2-(x2-x1)*k;
                }
            }
            if(x2>w||x2<0)
            {
                double k;
                if(x2!=x1)
                {
                    k=(y2-y1)/(x2-x1);
                }
                else
                {
                    k=(y2-y1)/(x2-x1+0.00001);
                }
                if(x2>w)
                {
                    x2=w;
                    y2=y1+(x2-x1)*k;
                }
                else if(x2<0)
                {
                    x2=0;
                    y2=y1+(x2-x1)*k;
                }
            }
            //printf("(%.2f,%.2f) (%.2f,%.2f)\r\n", x1, y1, x2, y2);
        }
        else
        {
            x1 = -w / 2;
            x2 = w / 2;
            float temp = sin(maxIDx / 180.0 * PI);
            if (temp == 0)
                temp = 0.00001;
            y1 = (maxIDy - x1*cos(maxIDx / 180.0 * PI)) / temp;
            y2 = (maxIDy - x2*cos(maxIDx / 180.0 * PI)) / temp;
            x1=x1+w/2;
            x2=x2+w/2;
            y1=y1+h / 2;
            y2=y2+h / 2;
            if(y1<0||y1>h)
            {
                double k;
                if(x2!=x1)
                {
                    k=(y2-y1)/(x2-x1);
                }
                else
                {
                    k=(y2-y1)/(x2-x1+0.00001);
                }
                if(y1<0)
                {
                    y1=0;
                    x1=x2-(y2-y1)/k;
                }

                if(y1>h)
                {
                    y1=h;
                    x1=x2-(y2-y1)/k;
                }
            }
            if(y2<0||y2>h)
            {
                double k;
                if(x2!=x1)
                {
                    k=(y2-y1)/(x2-x1);
                }
                else
                {
                    k=(y2-y1)/(x2-x1+0.00001);
                }
                if(y2<0)
                {
                    y2=0;
                    x2=x1+(y2-y1)/k;
                }

                if(y2>h)
                {
                    y2=h;
                    x2=x1+(y2-y1)/k;
                }
            }
            //printf("(%.2f,%.2f) (%.2f,%.2f)\r\n", x1, y1, x2, y2);
        }
       //return Vec4f(x1 + w / 2, y1 + h / 2, x2 + w / 2, y2 + h / 2);
        return Vec4f(x1, y1, x2, y2);
    }

    bool HoughLine::addLine(vector<Vec4f>& lines, float maxIDx, float maxIDy)
    {
        //cout<<max<<endl;

        float x1, x2, y1, y2;
        float tempp = (maxIDx / 180.0 * PI), temppp = atan((float)h / w);
        if (tempp < temppp || tempp > PI - temppp)
        {
            y1 = -h / 2;
            y2 = h / 2;
            float temp = cos(maxIDx / 180.0 * PI);
            if (temp == 0)
                temp = 0.00001;
            x1 = (maxIDy - y1*sin(maxIDx / 180.0 * PI)) / temp;
            x2 = (maxIDy - y2*sin(maxIDx / 180.0 * PI)) / temp;
            //printf("(%.2f,%.2f) (%.2f,%.2f)\r\n", x1, y1, x2, y2);
        }
        else
        {
            x1 = -w / 2;
            x2 = w / 2;
            float temp = sin(maxIDx / 180.0 * PI);
            if (temp == 0)
                temp = 0.00001;
            y1 = (maxIDy - x1*cos(maxIDx / 180.0 * PI)) / temp;
            y2 = (maxIDy - x2*cos(maxIDx / 180.0 * PI)) / temp;
            //printf("(%.2f,%.2f) (%.2f,%.2f)\r\n", x1, y1, x2, y2);
        }
        lines.push_back(Vec4f(x1 + w / 2, y1 + h / 2, x2 + w / 2, y2 + h / 2));
    }

    Vec3f HoughLine::transformLineToPolar(Vec4f line)
    {
        //坐标系原点为图像中心，向右x增大，向下y增大
        float x1 = line[0] - w / 2;
        float x2 = line[2] - w / 2;
        float y1 = line[1] - h / 2;
        float y2 = line[3] - h / 2;
        float tempx = x1 - x2;
        float tempy = y1 - y2;
        if (tempy == 0)
                tempy = 0.000001;

        float theta = atan((x2 - x1) / tempy);
        if (theta < 0)
                theta += PI;
        float ro = x1*cos(theta) + y1*sin(theta);
        return Vec3f(theta, ro, sqrt(tempx*tempx + tempy*tempy));
    }

    vector<Vec3f> HoughLine::Transform(vector<Vec4f> lines)
    {
        vector<Vec3f> temp_vec;
        for (int i = 0; i < lines.size(); i++)
        {
                //坐标系原点为图像中心，向右x增大，向下y增大
                float x1 = lines[i][0] - w / 2;
                float x2 = lines[i][2] - w / 2;
                float y1 = lines[i][1] - h / 2;
                float y2 = lines[i][3] - h / 2;
                float tempx = x1 - x2;
                float tempy = y1 - y2;
                if (tempy == 0)
                        tempy = 0.000001;

                float theta = atan((x2 - x1) / tempy);
                if (theta < 0)
                        theta += PI;
                float ro = x1*cos(theta) + y1*sin(theta);
                temp_vec.push_back(Vec3f(theta, ro, sqrt(tempx*tempx + tempy*tempy)));
        }
        return temp_vec;
    }

    void HoughLine::TransformImage(vector<Vec3f> data,Mat& image_gaussian)
    {
        Mat image(rows, cols, CV_32F, Scalar(0));
        for (int i = 0; i < data.size(); i++)
        {
                image.at<float>((int)(data[i][1] + sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2))),(int)(data[i][0] / PI * 180)) += data[i][2];
        }
        //image.copyTo(image_gaussian);
        GaussianBlur(image, image_gaussian, Size(5, 5), 0, 0);
    }

    bool HoughLine::findMainLine(Mat frame,Mat image_gaussian,Vec4f &mainLine,double vth,int& ifRotate,int& fourDirection)
    {
        //搜第一跟线,即竖直的线
        //在上一时刻的MaxIDx,MaxIDy两边搜
        float max = 0;
        float maxIDx = 0, maxIDy = 0;
        float rotate=0;
        bool findMainLine=false;
        if(!ifFollowFind)
        {
            for (int col = cols-findRange; col < cols+findRange; col++)
            {
            	int i=col;
                if(i<0)
                    i+=cols;
                else if(i>=cols)
                    i-=cols;
                for (int row = 0; row < rows; row++)
                {
                    int j=row;
                    if(j<0)
                        j+=rows;
                    else if(j>=rows)
                        j-=rows;
                    if (max < image_gaussian.at<float>(j, i))
                    {
                        max = image_gaussian.at<float>(j, i);
                        maxIDx = i;//theta
                        maxIDy = j - sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));//RO
                    }
                }
            }
        }

        else
        {
            int minCol,maxCol;
            curMainMaxIDx=lastMainMaxIDx+vth;
            if(curMainMaxIDx>180)
                curMainMaxIDx=curMainMaxIDx-180;
            else if(curMainMaxIDx<0)
                curMainMaxIDx=curMainMaxIDx+180;
            curMainMaxIDy=lastMainMaxIDy;
            //cout<<"lastMainMaxIDy: "<<lastMainMaxIDx<<" vth: "<<static_cast<int>(vth)<<" curMainMaxIDx:"<<curMainMaxIDx<<endl;
            if(ifRotate==0)
            {
                minCol=static_cast<int>(curMainMaxIDx)-thetaThres;
                maxCol=static_cast<int>(curMainMaxIDx)+thetaThres;
            }
           else
           {
                minCol=static_cast<int>(curMainMaxIDx)-2*thetaThres;
                maxCol=static_cast<int>(curMainMaxIDx)+2*thetaThres;
            }

            int minRow,maxRow;
            minRow=0;
            maxRow=rows;

            //cout<<"minCol:"<<minCol<<" maxCol:"<<maxCol<<endl;
            for(int col=minCol;col<maxCol;col++)
            {
                int i=col;
                if(i<0)
                    i+=cols;
                else if(i>=cols)
                    i-=cols;
                for(int row=minRow;row<maxRow;row++)
                {
                    int j=row;
                    if(j<0)
                        j+=rows;
                    else if(j>=rows)
                        j-=rows;
                    if (max < image_gaussian.at<float>(j, i))
                    {
                        max = image_gaussian.at<float>(j, i);
                        maxIDx = i;//theta
                        maxIDy = j - sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));//RO
                    }
                }
            }
        }
        //cout<<"maxIDx: "<<maxIDx<<endl;
        if (max>threshold1)
        {
            mainLine=transformPolarToLine(maxIDx,maxIDy);
            bool ifFindMainLine=true;
            ifFindMainLine=judgeLine(frame,mainLine,0);
            if(ifFindMainLine)
            {
                lastMainMaxIDx=maxIDx;
                lastMainMaxIDy=maxIDy+ sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));
                if(mainLine[1]==0&&mainLine[3]==h&&(mainLine[0]>eddging&&mainLine[0]<(w-eddging))&&(mainLine[2]>eddging&&mainLine[2]<(w-eddging)))
                {
                    ifRotate=0;
                }
                //else if((mainLine[0]!=0||mainLine[2]==w)||(mainLine[0]==0||mainLine[2]==0))
                else if((mainLine[1]!=0)||(mainLine[0]>=(w-eddging)||mainLine[2]>=(w-eddging))||(mainLine[0]<=eddging||mainLine[2]<=eddging))
                {
                    if(ifRotate==0)
                    {
                        if(maxIDx>90)
                            lastMainMaxIDx=maxIDx-90;
                        else
                            lastMainMaxIDx=maxIDx+90;
                        lastMainMaxIDy=maxIDy+ sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));
                    }
                    ifRotate=4;
                    if(lastMainMaxIDx>0&&lastMainMaxIDx<90)
                    {
                        ifRotate=1;//向右转
                    }
                    else if(lastMainMaxIDx>90&&lastMainMaxIDx<180)
                    {
                        ifRotate=2;//向左转
                    }
                    else if(lastMainMaxIDx==90)
                    {
                        if(mainLine[0]>(w-eddging)&&mainLine[2]>(w-eddging))
                        {
                            ifRotate=2;//向左转
                        }
                        else if(mainLine[0]<eddging&&mainLine[2]<eddging)
                        {
                            ifRotate=1;//向右转
                        }
                    }
                }

                //cout<<"mainLine:"<<mainLine[0]<<" "<<mainLine[1]<<" "<<mainLine[2]<<" "<<mainLine[3]<<" "<<max<<endl;//" "<<abs(imuP.th-lastCrossImuP.th)<<endl;
                //cout<<ifRotate<<" "<<lastMainMaxIDx<<" "<<lastMainMaxIDy<<endl;
                //cout<<"x1:"<<mainLine[0]<<" y1:"<<mainLine[1]<<" x2:"<<mainLine[2]<<" y2:"<<mainLine[3]<<" maxIDx:"<<houghData[0]<<" ifRoate:"<<ifRoate<<endl;
                //cout<<"x1:"<<mainLine[0]<<" y1:"<<mainLine[1]<<" x2:"<<mainLine[2]<<" y2:"<<mainLine[3]<<" dir:"<<p.fourDirection<<" ifRoate:"<<ifRoate<<endl;
                findMainLine=true;
                mainLineCount++;

                if(!ifFollowFind)
                    ifFollowFind=true;

                mainMax=max;
            }
            else
            {
                findMainLine=false;
                lastMainMaxIDx=curMainMaxIDx;
                lastMainMaxIDy=curMainMaxIDy;
            }
        }
        else
        {
            findMainLine=false;
            lastMainMaxIDx=curMainMaxIDx;
            lastMainMaxIDy=curMainMaxIDy;
        }
        //imshow("image_gaussian",image_gaussian);
        return findMainLine;
    }
    bool HoughLine::getLBP()
    {
        int radius = 1;
        int neighbor = 8;
        Mat lbp_image16 = elbp(gray, radius, neighbor); //robust
        lbp_image16.convertTo(lbp_image,CV_8U);//是由于elbp返回的矩阵元素类型是16位的，通过convertTo函数转化为CV_8U就可以

    }

    bool HoughLine::judgeLine(Mat image, Vec4f line,bool mainOrSecond)
    {
         Mat query;
         double score;
         bool isLine=true;
         vector<Point> linePoints;
         int lineNum=(lineRange-1)/2;
         Vec2f kb;
         getKb(line,kb);
         vector<float> mainLbpFeatureHist,secondLbpFeatureHist;
         vector<float> mainFeatureHist,secondFeatureHist;
         vector<float> HS;
         mainLbpFeatureHist.resize(59);
         mainFeatureHist.resize(121);
         secondLbpFeatureHist.resize(59);
         secondFeatureHist.resize(121);

         if(mainOrSecond==0)
         {
             if(line[1]==0&&line[3]==image.rows)
             {
                for(int i=0;i<image.rows;i++)
                {
                    Point2f p;
                    p.y=i;
                    p.x=(((float)(i)-(float)kb[1])/(float)kb[0]);
                    if(p.x>lineNum&&p.x<(image.cols-lineNum))
                        linePoints.push_back(Point((int)p.x,int(p.y)));
                }
            }
            else if(line[0]==0&&line[2]==image.cols)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        if(mainOrSecond==1)
        {
             if(line[0]==0&&line[2]==image.cols)
             {
                for(int i=0;i<image.cols;i++)
                {
                    Point2f p;
                    p.x=i;
                    p.y=(float)(i)*(float)kb[0]+(float)kb[1];
                    if(p.y>lineNum&&p.y<(image.rows-lineNum))
                        linePoints.push_back(Point((int)p.x,int(p.y)));
                }
            }
            else if(line[1]==0&&line[3]==image.rows)
            {
                return false;
            }
            else
            {
                return true;
            }
            cout<<secondLineCount<<endl;
            if(linePoints.size()<image.rows)
            {
                return false;
            }
        }
        Mat lineImageLBP;
        Mat lineImage;

        if(linePoints.size()>0)
        {
            if(mainOrSecond==0)
            {
                lineImageLBP.create(linePoints.size(),lineRange,lbp_image.type());
                lineImage.create(linePoints.size(),lineRange,image.type());
                for(int i=0;i<linePoints.size();i++)
                {
                    int countX=0;
                    for(int j=linePoints[i].x-lineNum;j<=linePoints[i].x+lineNum;j++)
                    {
                            lineImageLBP.at<uchar>(i,countX)=lbp_image.at<uchar>(linePoints[i].y,j);
                            lineImage.at<Vec3b>(i,countX)[0]=image.at<Vec3b>(linePoints[i].y,j)[0];
                            lineImage.at<Vec3b>(i,countX)[1]=image.at<Vec3b>(linePoints[i].y,j)[1];
                            lineImage.at<Vec3b>(i,countX)[2]=image.at<Vec3b>(linePoints[i].y,j)[2];
                            countX++;
                    }
                }
                imshow("mainLbp",lineImageLBP);
                imshow("mainLineImage",lineImage);
            }
            else if(mainOrSecond==1)
            {
                lineImageLBP.create(lineRange,linePoints.size(),lbp_image.type());
                lineImage.create(lineRange,linePoints.size(),image.type());
                for(int i=0;i<linePoints.size();i++)
                {
                    int countY=0;
                    for(int j=linePoints[i].y-lineNum;j<=linePoints[i].y+lineNum;j++)
                    {
                            lineImageLBP.at<uchar>(countY,i)=lbp_image.at<uchar>(j,linePoints[i].x);
                            lineImage.at<Vec3b>(countY,i)[0]=image.at<Vec3b>(j,linePoints[i].x)[0];
                            lineImage.at<Vec3b>(countY,i)[1]=image.at<Vec3b>(j,linePoints[i].x)[1];
                            lineImage.at<Vec3b>(countY,i)[2]=image.at<Vec3b>(j,linePoints[i].x)[2];
                            countY++;
                    }
                }
                imshow("secondLbp",lineImageLBP);
                imshow("secondLineImage",lineImage);
            }
            int _grid_x=1;
            int _grid_y=1;
            int _neighbors=8;
            query = spatial_histogram(
                     lineImageLBP,
                     static_cast<int>(std::pow(2.0, static_cast<double>(_neighbors))),
                     _grid_x,
                     _grid_y,
                     true,
                     true);
             hsvHist.getHistogramHS(lineImage ,HS);
            //0为主线
            if(mainOrSecond==0)
            {
                if(mainLineCount<10)
                {
                    for(int i=0;i<59;i++)
                    {
                        constLBPfeatureHist[i]+=(float)query.at<float>(0,i);
                        constfeatureHist[i]+=(float)query.at<float>(0,i);
                    }
                    for(int i=0;i<62;i++)
                    {
                        constfeatureHist[i+59]+=HS[i];
                    }
                    if(mainLineCount==9)
                    {
                        for(int i=0;i<59;i++)
                        {
                            constLBPfeatureHist[i]/=10;
                            constfeatureHist[i]/=10;
                        }
                        for(int i=0;i<62;i++)
                        {
                            constfeatureHist[i+59]/=10;
                        }
                        float sum=0.0;
                        for(int i=0;i<constfeatureHist.size();i++)
                            sum+=(float)constfeatureHist[i];
                        for(int i=0;i<constfeatureHist.size();i++)
                            constfeatureHist[i]+=((float)(constfeatureHist[i])/sum);
                    }
                }
                else
                {
                    for(int i=0;i<59;i++)
                    {
                        mainLbpFeatureHist[i]=(float)query.at<float>(0,i);
                        mainFeatureHist[i]=(float)query.at<float>(0,i);
                    }
                    for(int i=0;i<62;i++)
                    {
                        mainFeatureHist[i+59]=HS[i];
                    }
                    float sum=0;
                    for(int i=0;i<mainFeatureHist.size();i++)
                        sum+=(float)(mainFeatureHist[i]);
                    for(int i=0;i<mainFeatureHist.size();i++)
                        mainFeatureHist[i]+=((float)(mainFeatureHist[i])/sum);

                    score=compareHist(constfeatureHist,mainFeatureHist,CV_COMP_CORREL);
                    if(score<maxScore)
                        isLine=false;
                    else
                    {
                        isLine=true;
                        if(score>topScore)
                        {
                            for(int i=0;i<121;i++)
                            {
                                //constfeatureHist.at<float>(i,0)=(1-alpha)*constfeatureHist.at<float>(i,0)
                                //        +alpha*(float)(mainFeatureHist.at<float>(i,0));
                            }
                        }
                    }
                }
            }

            else if(mainOrSecond==1)
            {
                if(secondLineCount<5)
                {
                    for(int i=0;i<59;i++)
                    {
                        constSecondLBPfeatureHist[i]+=(float)query.at<float>(0,i);
                        constSecondfeatureHist[i]+=(float)query.at<float>(0,i);
                    }
                    for(int i=0;i<62;i++)
                    {
                        constSecondfeatureHist[i+59]+=HS[i];
                    }
                    if(secondLineCount==4)
                    {
                        for(int i=0;i<59;i++)
                        {
                            constSecondLBPfeatureHist[i]/=4;
                            constSecondfeatureHist[i]/=4;
                        }
                        for(int i=0;i<62;i++)
                        {
                            constSecondfeatureHist[i+59]/=4;
                        }
                        float sum=0.0;
                        for(int i=0;i<constSecondfeatureHist.size();i++)
                            sum+=(float)constSecondfeatureHist[i];
                        for(int i=0;i<constSecondfeatureHist.size();i++)
                            constSecondfeatureHist[i]+=((float)(constSecondfeatureHist[i])/sum);
                    }
                }
                else
                {
                    for(int i=0;i<59;i++)
                    {
                        secondLbpFeatureHist[i]=(float)query.at<float>(0,i);
                        secondFeatureHist[i]=(float)query.at<float>(0,i);
                    }
                    for(int i=0;i<62;i++)
                    {
                        secondFeatureHist[i+59]=HS[i];
                    }
                    float sum=0;
                    for(int i=0;i<secondFeatureHist.size();i++)
                        sum+=(float)(secondFeatureHist[i]);
                    for(int i=0;i<secondFeatureHist.size();i++)
                        secondFeatureHist[i]+=((float)(secondFeatureHist[i])/sum);

                    score=compareHist(constSecondfeatureHist,secondFeatureHist,CV_COMP_CORREL);
                    if(score<maxScore)
                        isLine=false;
                    else
                    {
                        isLine=true;
                        if(score>topScore)
                        {
                            for(int i=0;i<121;i++)
                            {
                                //constSecondfeatureHist.at<float>(i,0)=(1-alpha)*constSecondfeatureHist.at<float>(i,0)
                                //        +alpha*(float)(secondFeatureHist.at<float>(i,0));
                            }
                        }
                    }
                    cout<<secondLineCount<<" second score: "<<score<<endl;
                }
            }
            lineImageLBP.release();
            lineImage.release();
        }
        else
        {
            isLine=true;
        }
        return isLine;
    }
    bool HoughLine::judgeCrossPoint(Mat image, Point cross)
    {
        Mat query;
        bool isCross=true;
        double score;
        int crossNum=(crossRange-1)/2;
        vector<float> HS;
        Mat crossImageLBP(crossRange,crossRange,lbp_image.type());
        Mat crossImage(crossRange,crossRange,image.type());
        vector<float> crossLbpFeatureHist;
        vector<float> crossFeatureHist;
        crossLbpFeatureHist.resize(59);
        crossFeatureHist.resize(121);

        if(cross.x>=crossNum&&cross.x<(image.cols-crossNum)&&cross.y>=crossNum&&cross.y<(image.rows-crossNum))
        {
            for(int i=0;i<crossRange;i++)
            {
                for(int j=0;j<crossRange;j++)
                {
                    crossImageLBP.at<uchar>(i,j)=lbp_image.at<uchar>(cross.y+(i-crossNum),cross.x+(j-crossNum));
                    crossImage.at<Vec3b>(i,j)[0]=image.at<Vec3b>(cross.y+(i-crossNum),cross.x+(j-crossNum))[0];
                    crossImage.at<Vec3b>(i,j)[1]=image.at<Vec3b>(cross.y+(i-crossNum),cross.x+(j-crossNum))[1];
                    crossImage.at<Vec3b>(i,j)[2]=image.at<Vec3b>(cross.y+(i-crossNum),cross.x+(j-crossNum))[2];
                }
            }
        }
        else
            return false;
        int _grid_x=1;
        int _grid_y=1;
        int _neighbors=8;
        query = spatial_histogram(
                 crossImageLBP, /* lbp_image */
                 static_cast<int>(std::pow(2.0, static_cast<double>(_neighbors))), /* number of possible patterns   */
                 _grid_x, /* grid size x */
                 _grid_y, /* grid size y */
                 true,/* normed histograms */
                 true);
        hsvHist.getHistogramHS(crossImage ,HS);
        if(crossCount<5)
        {
            for(int i=0;i<59;i++)
            {
                constCrossLBPfeatureHist[i]+=(float)query.at<float>(0,i);
                constCrossfeatureHist[i]+=(float)query.at<float>(0,i);
            }
            for(int i=0;i<62;i++)
            {
                constCrossfeatureHist[i+59]+=HS[i];
            }
            if(crossCount==4)
            {
                for(int i=0;i<59;i++)
                {
                    constCrossLBPfeatureHist[i]/=5;
                    constCrossfeatureHist[i]/=5;
                }
                for(int i=0;i<62;i++)
                {
                    constCrossfeatureHist[i+59]/=5;
                }
                float sum=0.0;
                for(int i=0;i<constCrossfeatureHist.size();i++)
                    sum+=(float)(constCrossfeatureHist[i]);
                for(int i=0;i<constCrossfeatureHist.size();i++)
                    constCrossfeatureHist[i]+=((float)(constCrossfeatureHist[i])/sum);
            }
        }
        else
        {
            for(int i=0;i<59;i++)
            {
                crossLbpFeatureHist[i]=(float)query.at<float>(0,i);
                crossFeatureHist[i]=(float)query.at<float>(0,i);
            }
            for(int i=0;i<62;i++)
            {
                crossFeatureHist[i+59]=HS[i];
            }
            float sum=0;
            for(int i=0;i<crossFeatureHist.size();i++)
                sum+=(float)(crossFeatureHist[i]);
            for(int i=0;i<crossFeatureHist.size();i++)
                crossFeatureHist[i]+=((float)(crossFeatureHist[i])/sum);

            score=compareHist(constCrossfeatureHist,crossFeatureHist,CV_COMP_CORREL);
            if(score<crossMaxScore)
                isCross=false;
            else
            {
                isCross=true;
                if(score>topScore)
                {
                    for(int i=0;i<121;i++)
                    {
                        //constCrossfeatureHist.at<float>(i,0)=(1-alpha)*constCrossfeatureHist.at<float>(i,0)
                        //        +alpha*(float)(crossFeatureHist.at<float>(i,0));
                    }
                }
            }
        }
//        cout<<"crossCount:"<<crossCount<<" score:"<<score<<endl;
        imshow("crossLBP",crossImageLBP);
        imshow("crossImage",crossImage);
        return isCross;
    }
    bool HoughLine::findSecondLine(Mat frame,Mat image_gaussian,Vec4f mainLine,Vec4f& secondLine)
    {
        int mainRoate;
        int mainP;
        Vec3f polar=transformLineToPolar(mainLine);
        mainP=(int)(polar[1] + sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2)));
        mainRoate=(int)(polar[0] / PI * 180);

        float max = 0;
        float maxIDx = mainRoate, maxIDy = mainP;
        bool findSecondLine=false;
        //再次搜索 隔绝之前搜到的角度70度以上

        for (int col = 0; col < cols; col++)
        {
            if (abs(col - mainRoate) < mainAndSecondRoate || abs(abs(col - mainRoate) - 180) < mainAndSecondRoate)
                continue;
            for (int row = 0; row < rows; row++)
              {
                    if (max < image_gaussian.at<float>(row, col))
                    {
                        max = image_gaussian.at<float>(row, col);
                        maxIDx = col;//theta
                        maxIDy = row - sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));//RO
                    }
                }
          }
        if(max>threshold2)
        {
               secondLine=transformPolarToLine(maxIDx,maxIDy);
               findSecondLine=true;
               secondLineCount++;

               secondMax=max;
//               if(judgeLine(frame,secondLine,1))
//               {
//                    findSecondLine=true;
//                    secondLineCount++;
//               }
//               else
//               {
//                   findSecondLine=false;
//               }
        }
        else
        {
            findSecondLine=false;
        }

        return findSecondLine;
    }
    void HoughLine::getKb(Vec4f myLine,Vec2f& kb)
    {
        kb[0]=(float)(myLine[3]-myLine[1])/(float)(myLine[2]-myLine[0]+0.0001);
        kb[1]=(float)myLine[1]-kb[0]*(float)myLine[0];
    }

    bool HoughLine::getCrossPoint(Vec4f mainLine, Vec4f secondLine, Point &cross)
    {
        Vec2f kb1,kb2;
        getKb(mainLine,kb1);
        getKb(secondLine,kb2);
        Vec2f crossD;
        crossD[0]=(float)(kb2[1]-kb1[1])/(float)(kb1[0]-kb2[0]+0.0001);
        if(mainLine[1]==0)
            crossD[1]=(float)kb2[0]*(float)crossD[0]+(float)kb2[1];
        else
            crossD[1]=(float)kb1[0]*(float)crossD[0]+(float)kb1[1];
        cross.x=(int)crossD[0];
        cross.y=(int)crossD[1];

        crossCount++;
        return true;
    }
