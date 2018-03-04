#include "houghline.h"

namespace findLine
{
    HoughLine::HoughLine(int _w,int _h, int _threshold,int _distThres=20,int _thetaThres=20):w(_w),h(_h),threshold(_threshold),distThres(_distThres),thetaThres(_thetaThres)
    {
          cols = 180;//列
          rows = 2 * sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));//行

          frameNum=0;
          ifRotate=false;
          ifInit=false;
          ifFollowFind=false;
          findRange=40;
          rotateRange=20;
          eddging=20;

          mainLineRange=21;
          secondLineRange=21;

          p.x=0;p.y=0;p.th=0;p.currentCrossPoint.x=0;p.currentCrossPoint.y=0;

          floorH=0.600;

          mainAndSecondRoate=85;
          intrinsic_Matrix.create(3,3,CV_64F);
          distortion_coeffs.create(1, 5, CV_64F);

          constfeatureHist.create(121,1,CV_32F);
          constLBPfeatureHist.create(59,1,CV_32F);

          constSecondfeatureHist.create(121,1,CV_32F);
          constSecondLBPfeatureHist.create(59,1,CV_32F);

          outfile.open("/home/ren/PycharmProjects/car107/pos15.txt");
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

    bool HoughLine::findMainLine(Mat image_gaussian,Vec4f &mainLine, Vec2f& houghData)
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
            int curMainMaxIDx=lastMainMaxIDx;//+kVth*odom.vth;
            int curMainMaxIDy=lastMainMaxIDy;//+kVy*odom.vy;

            if(ifRotate==0)
            {
                minCol=curMainMaxIDx-thetaThres;
                maxCol=curMainMaxIDx+thetaThres;
            }
           else
           {
                minCol=curMainMaxIDx-2*thetaThres;
                maxCol=curMainMaxIDx+2*thetaThres;
            }

            int minRow,maxRow;
            minRow=curMainMaxIDy-distThres;
            maxRow=curMainMaxIDy+distThres;

            //cout<<"minCol:"<<minCol<<" maxCol:"<<maxCol<<endl;
            for(int col=minCol;col<maxCol;col++)
            {
                int i=col;
                if(i<0)
                    i+=cols;
                else if(i>=cols)
                    i-=cols;
                for(int row=0;row<rows;row++)
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
        if (max>threshold)
        {
            lastMainMaxIDx=maxIDx;
            lastMainMaxIDy=maxIDy+ sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));

            mainLine=transformPolarToLine(maxIDx,maxIDy);

            houghData[0]=maxIDx;
            houghData[1]=maxIDy+ sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));

            //if((maxIDx<roateRange&&maxIDx>=0)||(maxIDx>(cols-roateRange)&&maxIDx<=cols))
           // if((mainLine[0]>(w/2-60)&&mainLine[0]<(w/2+60))&&(mainLine[2]>(w/2-60)&&mainLine[2]<(w/2+60)))
            //if((mainLine[0]>0&&mainLine[0]<w)&&(mainLine[2]>0&&mainLine[2]<w))
            if(mainLine[1]==0&&mainLine[3]==h&&(mainLine[0]>eddging&&mainLine[0]<(w-eddging))&&(mainLine[2]>eddging&&mainLine[2]<(w-eddging)))
            {
                lastMainMaxIDx=maxIDx;
                lastMainMaxIDy=maxIDy+ sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));

                lastRotate=ifRotate;
                ifRotate=0;
            }
            //else if((mainLine[0]!=0||mainLine[2]==w)||(mainLine[0]==0||mainLine[2]==0))
            else if((mainLine[1]!=0)||(mainLine[0]>(w-eddging)&&mainLine[2]>(w-eddging))||(mainLine[0]<eddging&&mainLine[2]<eddging))
            {
                if(ifRotate==0)
                {
                    if(maxIDx>90)
                        lastMainMaxIDx=maxIDx-90;
                    else
                        lastMainMaxIDx=maxIDx+90;
                    lastMainMaxIDy=maxIDy+ sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));
                }
                lastRotate=ifRotate;
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
            if(lastRotate==0&&ifRotate!=0)
            {
                if(ifRotate==1)
                {
                    p.fourDirection++;
                }
                else if(ifRotate==2)
                {
                     p.fourDirection--;
                }
                if(p.fourDirection>3)
                    p.fourDirection=0;
                else if(p.fourDirection<0)
                    p.fourDirection=3;
            }
            //cout<<"mainLine:"<<mainLine[0]<<" "<<mainLine[1]<<" "<<mainLine[2]<<" "<<mainLine[3]<<endl;
            //cout<<"x1:"<<mainLine[0]<<" y1:"<<mainLine[1]<<" x2:"<<mainLine[2]<<" y2:"<<mainLine[3]<<" maxIDx:"<<houghData[0]<<" ifRoate:"<<ifRoate<<endl;
            //cout<<"x1:"<<mainLine[0]<<" y1:"<<mainLine[1]<<" x2:"<<mainLine[2]<<" y2:"<<mainLine[3]<<" dir:"<<p.fourDirection<<" ifRoate:"<<ifRoate<<endl;
            findMainLine=true;

            if(!ifFollowFind)
                ifFollowFind=true;
        }
        else
        {
            findMainLine=false;
        }
        //imshow("image_gaussian",image_gaussian);
        return findMainLine;
    }

    bool HoughLine::judgeMainLine(Mat image, Vec4f mainLine)
    {
        vector<float> featureVector;
        Mat query;
        if(mainLine[0]==0&&mainLine[2]==image.cols)
        {
            vector<Point> mainLinePoints;
            Mat gray;
            int radius = 1;
            int neighbor = 8;
            Vec2f kb;
            getKb(mainLine,kb);
            for(int i=0;i<image.cols;i++)
            {
                    Point2f p;
                    p.x=i;
                    p.y=(float)(i)*(float)kb[0]+(float)kb[1];

                    mainLinePoints.push_back(Point((int)p.x,int(p.y)));
            }
            if(image.channels()==3)
            {
                cvtColor(image,gray,CV_BGR2GRAY);
                Mat lbp_image = elbp(gray, radius, neighbor); //robust
                Mat lbpImage8U;
                lbp_image.convertTo(lbpImage8U,CV_8U);//是由于elbp返回的矩阵元素类型是16位的，通过convertTo函数转化为CV_8U就可以
                Mat mainLineImageLBP(mainLineRange,lbpImage8U.cols,lbpImage8U.type(),Scalar(0));
                for(int i=0;i<mainLinePoints.size();i++)
                {
                    int countY=0;
                    for(int j=mainLinePoints[i].y-mainLineRange/2;j<mainLinePoints[i].y+mainLineRange/2;j++)
                    {
                        mainLineImageLBP.at<uchar>(countY,mainLinePoints[i].x)=lbpImage8U.at<uchar>(j,mainLinePoints[i].x);
                        countY++;
                    }
                }
                int _grid_x=1;
                int _grid_y=1;
                int _neighbors=8;
                query = spatial_histogram(
                         mainLineImageLBP, /* lbp_image */
                         static_cast<int>(std::pow(2.0, static_cast<double>(_neighbors))), /* number of possible patterns   */
                         _grid_x, /* grid size x */
                         _grid_y, /* grid size y */
                         true,/* normed histograms */
                         true /* uniform LBP*/);

                Mat mainLineImage(mainLineRange,image.cols,image.type(),Scalar(0,0,0));
                for(int i=0;i<mainLinePoints.size();i++)
                {
                    int countY=0;
                    for(int j=mainLinePoints[i].y-mainLineRange/2;j<mainLinePoints[i].y+mainLineRange/2;j++)
                    {
                        mainLineImage.at<Vec3b>(countY,mainLinePoints[i].x)[0]=image.at<Vec3b>(j,mainLinePoints[i].x)[0];
                        mainLineImage.at<Vec3b>(countY,mainLinePoints[i].x)[1]=image.at<Vec3b>(j,mainLinePoints[i].x)[1];
                        mainLineImage.at<Vec3b>(countY,mainLinePoints[i].x)[2]=image.at<Vec3b>(j,mainLinePoints[i].x)[2];
                        countY++;
                    }
                }
                HSVCalcHistogram hsvHist;
                Mat hsv;
                vector<float> HS;
                cvtColor(mainLineImage, hsv, CV_BGR2HSV);
                hsvHist.getHistogramHS(hsv,HS);

                for(int i=0;i<query.cols;i++)
                {
                    featureVector.push_back((float)query.at<float>(0,i));
                }
                for(int i=0;i<HS.size();i++)
                {
                    featureVector.push_back(HS[i]);
                }
                float sum=0.0;
                for(int i=0;i<featureVector.size();i++)
                    sum+=featureVector[i];
                for(int i=0;i<featureVector.size();i++)
                    featureVector[i]+=featureVector[i]/sum;

                //imshow("lbpImage8U",lbpImage8U);
                imshow("mainLineImageLBP",mainLineImageLBP);
                imshow("mainLineImage",mainLineImage);
                //waitKey(0);
            }
            else if(image.channels()==1)
            {
                image.copyTo(gray);
                Mat lbp_image = elbp(gray, radius, neighbor); //robust
                Mat lbpImage8U;
                lbp_image.convertTo(lbpImage8U,CV_8U);//是由于elbp返回的矩阵元素类型是16位的，通过convertTo函数转化为CV_8U就可以
                Mat mainLineImageLBP(mainLineRange,lbpImage8U.cols,lbpImage8U.type(),Scalar(0));
                for(int i=0;i<mainLinePoints.size();i++)
                {
                    int countY=0;
                    for(int j=mainLinePoints[i].y-mainLineRange/2;j<mainLinePoints[i].y+mainLineRange/2;j++)
                    {
                        mainLineImageLBP.at<uchar>(countY,mainLinePoints[i].x)=lbpImage8U.at<uchar>(j,mainLinePoints[i].x);
                        countY++;
                    }
                }
                int _grid_x=1;
                int _grid_y=1;
                int _neighbors=8;
                query = spatial_histogram(
                         mainLineImageLBP, /* lbp_image */
                         static_cast<int>(std::pow(2.0, static_cast<double>(_neighbors))), /* number of possible patterns   */
                         _grid_x, /* grid size x */
                         _grid_y, /* grid size y */
                         true,/* normed histograms */
                         true /* uniform LBP*/);

                for(int i=0;i<query.cols;i++)
                {
                    featureVector.push_back((float)query.at<float>(0,i));
                }
            }
        }
        if(mainLine[1]==0&&mainLine[3]==image.rows)
        {
            vector<Point> mainLinePoints;
            Mat gray;
            int radius = 1;
            int neighbor = 8;
            Vec2f kb;
            getKb(mainLine,kb);
            for(int i=0;i<image.rows;i++)
            {
                    Point2f p;
                    p.y=i;
                    p.x=((float)(i)-(float)kb[1])/(float)kb[0];

                    mainLinePoints.push_back(Point((int)p.x,int(p.y)));
            }
            if(image.channels()==3)
            {
                cvtColor(image,gray,CV_BGR2GRAY);
                Mat lbp_image = elbp(gray, radius, neighbor); //robust
                Mat lbpImage8U;
                lbp_image.convertTo(lbpImage8U,CV_8U);//是由于elbp返回的矩阵元素类型是16位的，通过convertTo函数转化为CV_8U就可以
                Mat mainLineImageLBP(image.rows,mainLineRange,lbpImage8U.type(),Scalar(0));
                for(int i=0;i<mainLinePoints.size();i++)
                {
                    int countX=0;
                    for(int j=mainLinePoints[i].x-mainLineRange/2;j<mainLinePoints[i].x+mainLineRange/2;j++)
                    {
                        mainLineImageLBP.at<uchar>(mainLinePoints[i].y,countX)=lbpImage8U.at<uchar>(mainLinePoints[i].y,j);
                        countX++;
                    }
                }
                int _grid_x=1;
                int _grid_y=1;
                int _neighbors=8;
                query = spatial_histogram(
                         mainLineImageLBP, /* lbp_image */
                         static_cast<int>(std::pow(2.0, static_cast<double>(_neighbors))), /* number of possible patterns   */
                         _grid_x, /* grid size x */
                         _grid_y, /* grid size y */
                         true,/* normed histograms */
                         true);

                Mat mainLineImage(image.rows,mainLineRange,image.type(),Scalar(0,0,0));
                for(int i=0;i<mainLinePoints.size();i++)
                {
                    int countX=0;
                    for(int j=mainLinePoints[i].x-mainLineRange/2;j<mainLinePoints[i].x+mainLineRange/2;j++)
                    {
                        mainLineImage.at<Vec3b>(mainLinePoints[i].y,countX)[0]=image.at<Vec3b>(mainLinePoints[i].y,j)[0];
                        mainLineImage.at<Vec3b>(mainLinePoints[i].y,countX)[1]=image.at<Vec3b>(mainLinePoints[i].y,j)[1];
                        mainLineImage.at<Vec3b>(mainLinePoints[i].y,countX)[2]=image.at<Vec3b>(mainLinePoints[i].y,j)[2];
                        countX++;
                    }
                }
                HSVCalcHistogram hsvHist;
                Mat hsv;
                vector<float> HS;
                cvtColor(mainLineImage, hsv, CV_BGR2HSV);
                hsvHist.getHistogramHS(hsv,HS);

                for(int i=0;i<query.cols;i++)
                {
                    featureVector.push_back((float)query.at<float>(0,i));
                }
                for(int i=0;i<HS.size();i++)
                {
                    featureVector.push_back(HS[i]);
                }
                float sum=0.0;
                for(int i=0;i<featureVector.size();i++)
                    sum+=featureVector[i];
                for(int i=0;i<featureVector.size();i++)
                    featureVector[i]+=featureVector[i]/sum;
                imshow("mainLineImageLBP",mainLineImageLBP);
                imshow("mainLineImage",mainLineImage);
               // waitKey(0);
            }
            else if(image.channels()==1)
            {
                image.copyTo(gray);
                Mat lbp_image = elbp(gray, radius, neighbor); //robust
                Mat lbpImage8U;
                lbp_image.convertTo(lbpImage8U,CV_8U);//是由于elbp返回的矩阵元素类型是16位的，通过convertTo函数转化为CV_8U就可以
                Mat mainLineImageLBP(image.rows,mainLineRange,lbpImage8U.type(),Scalar(0));
                for(int i=0;i<mainLinePoints.size();i++)
                {
                    int countX=0;
                    for(int j=mainLinePoints[i].x-mainLineRange/2;j<mainLinePoints[i].x+mainLineRange/2;j++)
                    {
                        mainLineImageLBP.at<uchar>(mainLinePoints[i].y,countX)=lbpImage8U.at<uchar>(mainLinePoints[i].y,j);
                        countX++;
                    }
                }
                int _grid_x=1;
                int _grid_y=1;
                int _neighbors=8;
                query = spatial_histogram(
                         mainLineImageLBP, /* lbp_image */
                         static_cast<int>(std::pow(2.0, static_cast<double>(_neighbors))), /* number of possible patterns   */
                         _grid_x, /* grid size x */
                         _grid_y, /* grid size y */
                         true,/* normed histograms */
                         true);

                for(int i=0;i<query.cols;i++)
                {
                    featureVector.push_back((float)query.at<float>(0,i));
                }
                imshow("lbpImage8U",lbpImage8U);
                imshow("mainLineImageLBP",mainLineImageLBP);
                //waitKey(0);
            }
        }

        Mat featureHist(featureVector.size(),1,CV_32F);
        Mat lbpFeatureHist(59,1,CV_32F);
        bool isMainLine=true;
        double score;
        if(frameNum==11)
        {
            for(int i=0;i<featureVector.size();i++)
            {
                constfeatureHist.at<float>(i,0)=featureVector[i];
            }
            for(int i=0;i<59;i++)
            {
                constLBPfeatureHist.at<float>(i,0)=query.at<float>(0,i);
            }
        }

        else if(frameNum>11)
        {
            for(int i=0;i<featureVector.size();i++)
            {
                featureHist.at<float>(i,0)=featureVector[i];
            }
            for(int i=0;i<59;i++)
            {
                lbpFeatureHist.at<float>(i,0)=query.at<float>(0,i);
            }
            score=compareHist(constLBPfeatureHist,lbpFeatureHist,CV_COMP_INTERSECT);
            if(score<0.8)
                isMainLine=false;
            else
                isMainLine=true;
        }
        //cout<<" main score:"<<score<<endl;
        return isMainLine;
    }
    bool HoughLine::findSecondLine(Mat image_gaussian,Vec4f mainLine,Vec4f& secondLine,Vec2f& houghData)
    {
        int mainRoate;
        int mainP;
        Vec3f polar=transformLineToPolar(mainLine);
        mainP=(int)(polar[1] + sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2)));
        mainRoate=(int)(polar[0] / PI * 180);

        float max2 = 0;
        float maxIDx2 = mainRoate, maxIDy2 = mainP;
        bool findSecondLine=false;
        //再次搜索 隔绝之前搜到的角度70度以上

        for (int col = 0; col < cols; col++)
        {
            if (abs(col - mainRoate) < mainAndSecondRoate || abs(abs(col - mainRoate) - 180) < mainAndSecondRoate)
                continue;
            for (int row = 0; row < rows; row++)
              {
                    if (max2 < image_gaussian.at<float>(row, col))
                    {
                        max2 = image_gaussian.at<float>(row, col);
                        maxIDx2 = col;//theta
                        maxIDy2 = row - sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));//RO
                    }
                }
          }
        if(max2>threshold)
        {
               secondLine=transformPolarToLine(maxIDx2,maxIDy2);
               houghData[0]=maxIDx2;
               houghData[1]=maxIDy2+ sqrt((w / 2)*(w / 2) + (h / 2)*(h / 2));
               findSecondLine=true;
        }
        else
        {
            findSecondLine=false;
        }

        return findSecondLine;
    }
    bool HoughLine::judgeSecondLine(Mat image, Vec4f secondLine)
    {
        vector<float> featureVector;
        Mat query;
        if(secondLine[0]==0&&secondLine[2]==image.cols)
        {
            vector<Point> secondLinePoints;
            Mat gray;
            int radius = 1;
            int neighbor = 8;
            Vec2f kb;
            getKb(secondLine,kb);
            for(int i=0;i<image.cols;i++)
            {
                    Point2f p;
                    p.x=i;
                    p.y=(float)(i)*(float)kb[0]+(float)kb[1];

                    secondLinePoints.push_back(Point((int)p.x,int(p.y)));
            }
            if(image.channels()==3)
            {
                cvtColor(image,gray,CV_BGR2GRAY);
                Mat lbp_image = elbp(gray, radius, neighbor); //robust
                Mat lbpImage8U;
                lbp_image.convertTo(lbpImage8U,CV_8U);//是由于elbp返回的矩阵元素类型是16位的，通过convertTo函数转化为CV_8U就可以
                Mat secondLineImageLBP(secondLineRange,lbpImage8U.cols,lbpImage8U.type(),Scalar(0));
                for(int i=0;i<secondLinePoints.size();i++)
                {
                    int countY=0;
                    for(int j=secondLinePoints[i].y-secondLineRange/2;j<secondLinePoints[i].y+secondLineRange/2;j++)
                    {
                        secondLineImageLBP.at<uchar>(countY,secondLinePoints[i].x)=lbpImage8U.at<uchar>(j,secondLinePoints[i].x);
                        countY++;
                    }
                }
                int _grid_x=1;
                int _grid_y=1;
                int _neighbors=8;
                query = spatial_histogram(
                         secondLineImageLBP, /* lbp_image */
                         static_cast<int>(std::pow(2.0, static_cast<double>(_neighbors))), /* number of possible patterns   */
                         _grid_x, /* grid size x */
                         _grid_y, /* grid size y */
                         true,/* normed histograms */
                         true /* uniform LBP*/);

                Mat secondLineImage(secondLineRange,image.cols,image.type(),Scalar(0,0,0));
                for(int i=0;i<secondLinePoints.size();i++)
                {
                    int countY=0;
                    for(int j=secondLinePoints[i].y-secondLineRange/2;j<secondLinePoints[i].y+secondLineRange/2;j++)
                    {
                        secondLineImage.at<Vec3b>(countY,secondLinePoints[i].x)[0]=image.at<Vec3b>(j,secondLinePoints[i].x)[0];
                        secondLineImage.at<Vec3b>(countY,secondLinePoints[i].x)[1]=image.at<Vec3b>(j,secondLinePoints[i].x)[1];
                        secondLineImage.at<Vec3b>(countY,secondLinePoints[i].x)[2]=image.at<Vec3b>(j,secondLinePoints[i].x)[2];
                        countY++;
                    }
                }
                HSVCalcHistogram hsvHist;
                Mat hsv;
                vector<float> HS;
                cvtColor(secondLineImage, hsv, CV_BGR2HSV);
                hsvHist.getHistogramHS(hsv,HS);

                for(int i=0;i<query.cols;i++)
                {
                    featureVector.push_back((float)query.at<float>(0,i));
                }
                for(int i=0;i<HS.size();i++)
                {
                    featureVector.push_back(HS[i]);
                }
                float sum=0.0;
                for(int i=0;i<featureVector.size();i++)
                    sum+=featureVector[i];
                for(int i=0;i<featureVector.size();i++)
                    featureVector[i]+=featureVector[i]/sum;
                //imshow("lbpImage8U",lbpImage8U);
                imshow("secondLineImageLBP",secondLineImageLBP);
                imshow("secondLineImage",secondLineImage);
                //waitKey(0);
            }
            else if(image.channels()==1)
            {
                image.copyTo(gray);
                Mat lbp_image = elbp(gray, radius, neighbor); //robust
                Mat lbpImage8U;
                lbp_image.convertTo(lbpImage8U,CV_8U);//是由于elbp返回的矩阵元素类型是16位的，通过convertTo函数转化为CV_8U就可以
                Mat secondLineImageLBP(secondLineRange,lbpImage8U.cols,lbpImage8U.type(),Scalar(0));
                for(int i=0;i<secondLinePoints.size();i++)
                {
                    int countY=0;
                    for(int j=secondLinePoints[i].y-secondLineRange/2;j<secondLinePoints[i].y+secondLineRange/2;j++)
                    {
                        secondLineImageLBP.at<uchar>(countY,secondLinePoints[i].x)=lbpImage8U.at<uchar>(j,secondLinePoints[i].x);
                        countY++;
                    }
                }
                int _grid_x=1;
                int _grid_y=1;
                int _neighbors=8;
                query = spatial_histogram(
                         secondLineImageLBP, /* lbp_image */
                         static_cast<int>(std::pow(2.0, static_cast<double>(_neighbors))), /* number of possible patterns   */
                         _grid_x, /* grid size x */
                         _grid_y, /* grid size y */
                         true,/* normed histograms */
                         true /* uniform LBP*/);

                for(int i=0;i<query.cols;i++)
                {
                    featureVector.push_back((float)query.at<float>(0,i));
                }
                imshow("secondLineImageLBP",secondLineImageLBP);
                //waitKey(0);
            }
        }
        if(secondLine[1]==0&&secondLine[3]==image.rows)
        {
            vector<Point> secondLinePoints;
            Mat gray;
            int radius = 1;
            int neighbor = 8;
            Vec2f kb;
            getKb(secondLine,kb);
            for(int i=0;i<image.rows;i++)
            {
                    Point2f p;
                    p.y=i;
                    p.x=((float)(i)-(float)kb[1])/(float)kb[0];

                    secondLinePoints.push_back(Point((int)p.x,int(p.y)));
            }
            if(image.channels()==3)
            {
                cvtColor(image,gray,CV_BGR2GRAY);
                Mat lbp_image = elbp(gray, radius, neighbor); //robust
                Mat lbpImage8U;
                lbp_image.convertTo(lbpImage8U,CV_8U);//是由于elbp返回的矩阵元素类型是16位的，通过convertTo函数转化为CV_8U就可以
                Mat secondLineImageLBP(image.rows,mainLineRange,lbpImage8U.type(),Scalar(0));
                for(int i=0;i<secondLinePoints.size();i++)
                {
                    int countX=0;
                    for(int j=secondLinePoints[i].x-mainLineRange/2;j<secondLinePoints[i].x+mainLineRange/2;j++)
                    {
                        secondLineImageLBP.at<uchar>(secondLinePoints[i].y,countX)=lbpImage8U.at<uchar>(secondLinePoints[i].y,j);
                        countX++;
                    }
                }
                int _grid_x=1;
                int _grid_y=1;
                int _neighbors=8;
                query = spatial_histogram(
                         secondLineImageLBP, /* lbp_image */
                         static_cast<int>(std::pow(2.0, static_cast<double>(_neighbors))), /* number of possible patterns   */
                         _grid_x, /* grid size x */
                         _grid_y, /* grid size y */
                         true,/* normed histograms */
                         true);

                Mat secondLineImage(image.rows,mainLineRange,image.type(),Scalar(0,0,0));
                for(int i=0;i<secondLinePoints.size();i++)
                {
                    int countX=0;
                    for(int j=secondLinePoints[i].x-mainLineRange/2;j<secondLinePoints[i].x+mainLineRange/2;j++)
                    {
                        secondLineImage.at<Vec3b>(secondLinePoints[i].y,countX)[0]=image.at<Vec3b>(secondLinePoints[i].y,j)[0];
                        secondLineImage.at<Vec3b>(secondLinePoints[i].y,countX)[1]=image.at<Vec3b>(secondLinePoints[i].y,j)[1];
                        secondLineImage.at<Vec3b>(secondLinePoints[i].y,countX)[2]=image.at<Vec3b>(secondLinePoints[i].y,j)[2];
                        countX++;
                    }
                }
                HSVCalcHistogram hsvHist;
                Mat hsv;
                vector<float> HS;
                cvtColor(secondLineImage, hsv, CV_BGR2HSV);
                hsvHist.getHistogramHS(hsv,HS);

                for(int i=0;i<query.cols;i++)
                {
                    featureVector.push_back((float)query.at<float>(0,i));
                }
                for(int i=0;i<HS.size();i++)
                {
                    featureVector.push_back(HS[i]);
                }
                float sum=0.0;
                for(int i=0;i<featureVector.size();i++)
                    sum+=featureVector[i];
                for(int i=0;i<featureVector.size();i++)
                    featureVector[i]+=featureVector[i]/sum;
                imshow("secondLineImageLBP",secondLineImageLBP);
                imshow("secondLineImage",secondLineImage);
                //waitKey(0);
            }
            else if(image.channels()==1)
            {
                image.copyTo(gray);
                Mat lbp_image = elbp(gray, radius, neighbor); //robust
                Mat lbpImage8U;
                lbp_image.convertTo(lbpImage8U,CV_8U);//是由于elbp返回的矩阵元素类型是16位的，通过convertTo函数转化为CV_8U就可以
                Mat secondLineImageLBP(image.rows,mainLineRange,lbpImage8U.type(),Scalar(0));
                for(int i=0;i<secondLinePoints.size();i++)
                {
                    int countX=0;
                    for(int j=secondLinePoints[i].x-mainLineRange/2;j<secondLinePoints[i].x+mainLineRange/2;j++)
                    {
                        secondLineImageLBP.at<uchar>(secondLinePoints[i].y,countX)=lbpImage8U.at<uchar>(secondLinePoints[i].y,j);
                        countX++;
                    }
                }
                int _grid_x=1;
                int _grid_y=1;
                int _neighbors=8;
                query = spatial_histogram(
                         secondLineImageLBP, /* lbp_image */
                         static_cast<int>(std::pow(2.0, static_cast<double>(_neighbors))), /* number of possible patterns   */
                         _grid_x, /* grid size x */
                         _grid_y, /* grid size y */
                         true,/* normed histograms */
                         true);

                for(int i=0;i<query.cols;i++)
                {
                    featureVector.push_back((float)query.at<float>(0,i));
                }
                imshow("secondLineImageLBP",secondLineImageLBP);
                //waitKey(0);
            }
        }

        Mat secondFeatureHist(featureVector.size(),1,CV_32F);
        Mat secondLbpFeatureHist(59,1,CV_32F);
        bool isSecondLine=true;
        double score;
        if(frameNum==11)
        {
            for(int i=0;i<featureVector.size();i++)
            {
                constSecondfeatureHist.at<float>(i,0)=featureVector[i];
            }
            for(int i=0;i<59;i++)
            {
                constSecondLBPfeatureHist.at<float>(i,0)=query.at<float>(0,i);
            }
        }

        else if(frameNum>11)
        {
            for(int i=0;i<featureVector.size();i++)
            {
                secondFeatureHist.at<float>(i,0)=featureVector[i];
            }
            for(int i=0;i<59;i++)
            {
                secondLbpFeatureHist.at<float>(i,0)=query.at<float>(0,i);
            }
            score=compareHist(constSecondLBPfeatureHist,secondLbpFeatureHist,CV_COMP_INTERSECT);
            if(score<0.8)
                isSecondLine=false;
            else
                isSecondLine=true;
        }
        //cout<<" second score:"<<score<<endl;
        return isSecondLine;
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

#if 0
         cout<<"mainLine:"<<mainLine[0]<<" "<<mainLine[1]<<" "<<mainLine[2]<<" "<<mainLine[3]<<endl;
         cout<<"secondLine:"<<secondLine[0]<<" "<<secondLine[1]<<" "<<secondLine[2]<<" "<<secondLine[3]<<endl;
         cout<<"kb:"<<kb1[0]<<" "<<kb1[1]<<" "<<kb2[0]<<" "<<kb2[1]<<endl;
          cout<<"cross:"<<cross.x<<" "<<cross.y<<endl;
          cout<<endl;
#endif
        return true;
    }
    bool HoughLine::transformCross(Point cross, double Z, Point2d &err)
    {
        Point errPoint;
        err.x=(w/2-cross.x)/631.748188*284.7765;
        err.y=(cross.y-h/2)/628.892345*284.7765;
        return true;
    }

    bool HoughLine::loadCamera(string fileName)
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
        cout<<"load "<<fileName<<" finish"<<endl;
        return true;
    }


    bool HoughLine::InitializeCoordinates(Point2d err,Vec4f mainLine,Vec4f secondLine)
    {

        Vec3f data1=transformLineToPolar(mainLine);
        //cout<<"theta1:"<<data1[0]<<endl;
        if(data1[0]<PI/2&&data1[0]>0)
            data1[0]=-data1[0];
        else if(data1[0]<PI&&data1[0]>PI/2)
            data1[0]=PI-data1[0];
        else
            data1[0]=data1[0];

        initP.th=-data1[0];
        p.th=-data1[0];

        errImuTh=imuP.th-p.th;
        errOdomTh=odomP.th-p.th;

        double x,y;
        x=(err.y * cos(data1[0]) - err.x * sin(data1[0]))/1000.0;
        y=(err.y * sin(data1[0]) + err.x * cos(data1[0]))/1000.0;
        initP.x=x;
        initP.y=y;

        p.x=x;
        p.y=y;

        odomP.x=x;
        odomP.y=y;
        imuP.x=x;
        imuP.y=y;

        initP.currentCrossPoint.x=0;
        initP.currentCrossPoint.y=0;

        p.currentCrossPoint.x=0;
        p.currentCrossPoint.y=0;

        updated_ = ros::Time::now();

        cout<<initP.th<<endl;
        //cout<<"currentCrossPoint:"<<p.currentCrossPoint.x<<","<<p.currentCrossPoint.y<<endl;
    }

    bool HoughLine::updateCoordinates(Point2d err,Point2d lastErr,Vec4f mainLine)
    {
        double period = (ros::Time::now() - updated_).toSec();
        updated_ = ros::Time::now();

        odomP.th=odomP.th-errOdomTh;

        double delta_x = (odom.vx * cos(odomP.th) - odom.vy * sin(odomP.th))*period;
        double delta_y = (odom.vx * sin(odomP.th) + odom.vy * cos(odomP.th))*period;
        double delta_th = odom.vth*period;

        odomP.x+=delta_x;
        odomP.y+=delta_y;
        odomP.th+=delta_th;

        imuP.th=imuP.th-errImuTh;

        if((abs(err.y-lastErr.y)>150&&ifRotate==0)||(abs(err.x-lastErr.x)>150))
        {
            if(p.fourDirection==0)
                p.currentCrossPoint.x++;
            else if(p.fourDirection==1)
                p.currentCrossPoint.y++;
            if(p.fourDirection==2)
                p.currentCrossPoint.x--;
            else if(p.fourDirection==3)
                p.currentCrossPoint.y--;
            //cout<<"currentCrossPoint:"<<p.currentCrossPoint.x<<","<<p.currentCrossPoint.y<<endl;
        }
        else
        {
        }

        double x,y;
        Vec3f data1=transformLineToPolar(mainLine);

        if(data1[0]<PI/2&&data1[0]>0)
            data1[0]=-data1[0];
        else if(data1[0]<PI&&data1[0]>PI/2)
            data1[0]=PI-data1[0];
        else
            data1[0]=data1[0];

#if TH==0
       p.th=imuP.th-errImuTh;
#elif TH==1
       p.th=odomP.th;
#elif TH==2
       double tempTh;
       if(p.fourDirection==0)
       {
                tempTh=data1[0];
                p.th=-data1[0];
        }
        if(p.fourDirection==1)
        {
                tempTh=data1[0]-PI/2;
                if(tempTh>-PI)
                    tempTh=PI+(tempTh-PI);

                p.th=-data1[0]-PI/2;
                if(p.th>-PI)
                    p.th=PI+(p.th-PI);
        }
       if(p.fourDirection==2)
       {
                if(data1[0]>0)
                    tempTh=-(PI-data1[0]);
                else if(data1[0]<0)
                    tempTh=PI+data1[0];
                else
                    tempTh=PI;

                  if(data1[0]>0)
                      p.th=PI-data1[0];
                  else if(data1[0]<0)
                      p.th=-PI-data1[0];
                  else
                      p.th=PI;
        }
        if(p.fourDirection==3)
        {
                tempTh=data1[0]+PI/2;
                if(tempTh>PI)
                    tempTh=-PI+(tempTh-PI);

                p.th=-data1[0]+PI/2;
                if(p.th>PI)
                    p.th=-PI+(p.th-PI);
        }
#endif
        x=(-err.y * cos(p.th) - err.x * sin(p.th))/1000.0;
        y=(-err.y * sin(p.th) + err.x * cos(p.th))/1000.0;

        p.x=p.currentCrossPoint.x*floorH-x;
        p.y=p.currentCrossPoint.y*floorH+y;

        lastImuP.th=imuP.th;

        stringstream xS,yS,thS,pointXS,pointYS,imuPS;
        xS<<p.x;
        yS<<p.y;
        thS<<p.th;
        pointXS<<p.currentCrossPoint.x;
        pointYS<<p.currentCrossPoint.y;
        imuPS<<(imuP.th-errImuTh);
        //string lineString=xS.str()+" "+yS.str()+" "+thS.str()+" "+pointXS.str()+" "+pointYS.str()+" "+imuPS.str();

        stringstream odomXS,odomYS;
        odomXS<<odomP.x;
        odomYS<<odomP.y;
        string lineString=odomXS.str()+" "+odomYS.str()+" "+imuPS.str();
        outfile<<lineString<<endl;
        cout<<odomP.x<<" "<<odomP.y<<" "<<imuP.th<<endl;

        lastOdomP.x=odomP.x;
        lastOdomP.y=odomP.y;
        lastOdomP.th=odomP.th;

        lastP.x=p.x;
        lastP.y=p.y;
        lastP.th=p.th;
    }
    bool HoughLine::updateCoordinates(Vec4f mainLine)
    {
        double period = (ros::Time::now() - updated_).toSec();
        updated_ = ros::Time::now();

        odomP.th=odomP.th-errOdomTh;
        imuP.th=imuP.th-errImuTh;

        double delta_x = (odom.vx * cos(imuP.th) - odom.vy * sin(imuP.th))*period;
        double delta_y = (odom.vx * sin(imuP.th) + odom.vy * cos(imuP.th))*period;
        double delta_th = odom.vth*period;

        odomP.x+=delta_x;
        odomP.y+=delta_y;

        Vec3f data1=transformLineToPolar(mainLine);

         if(data1[0]<PI/2&&data1[0]>0)
             data1[0]=-data1[0];
         else if(data1[0]<PI&&data1[0]>PI/2)
             data1[0]=PI-data1[0];
         else
             data1[0]=data1[0];

#if TH==0
        p.th=imuP.th-errImuTh;
#elif TH==1
        p.th=odomP.th;
#elif TH==2
        double tempTh;
        if(p.fourDirection==0)
        {
            tempTh=data1[0];
            p.th=-data1[0];
        }
        if(p.fourDirection==1)
        {
            tempTh=data1[0]-PI/2;
            if(tempTh>-PI)
                tempTh=PI+(tempTh-PI);

            p.th=-data1[0]-PI/2;
            if(p.th>-PI)
                p.th=PI+(p.th-PI);
        }
        if(p.fourDirection==2)
        {
            if(data1[0]>0)
                tempTh=-(PI-data1[0]);
            else if(data1[0]<0)
                tempTh=PI+data1[0];
            else
                tempTh=PI;

              if(data1[0]>0)
                  p.th=PI-data1[0];
              else if(data1[0]<0)
                  p.th=-PI-data1[0];
              else
                  p.th=PI;
        }
        if(p.fourDirection==3)
        {
            tempTh=data1[0]+PI/2;
            if(tempTh>PI)
                tempTh=-PI+(tempTh-PI);

            p.th=-data1[0]+PI/2;
            if(p.th>PI)
                p.th=-PI+(p.th-PI);
        }
 #endif


        double delta_x1 = (odom.vx * cos(p.th) - odom.vy * sin(p.th))*period;
        double delta_y1= (odom.vx * sin(p.th) + odom.vy * cos(p.th))*period;

        //p.x+=delta_x1;
        //p.y+=delta_y1;

        lastImuP.th=imuP.th;

        //cout<<p.x<<" "<<p.y<<" "<<p.th<<" "<<p.currentCrossPoint.x<<" "<<p.currentCrossPoint.y<<endl;
        stringstream xS,yS,thS,pointXS,pointYS,imuPS;
        xS<<p.x;
        yS<<p.y;
        thS<<p.th;
        pointXS<<p.currentCrossPoint.x;
        pointYS<<p.currentCrossPoint.y;
        imuPS<<(imuP.th-errImuTh);
        //string lineString=xS.str()+" "+yS.str()+" "+thS.str()+" "+pointXS.str()+" "+pointYS.str()+" "+imuPS.str();

        stringstream odomXS,odomYS;
        odomXS<<odomP.x;
        odomYS<<odomP.y;
        string lineString=odomXS.str()+" "+odomYS.str()+" "+imuPS.str();
        outfile<<lineString<<endl;
        cout<<odomP.x<<" "<<odomP.y<<" "<<imuP.th<<endl;
    }
    bool HoughLine::updateCoordinates()
    {
        double period = (ros::Time::now() - updated_).toSec();
        updated_ = ros::Time::now();

        imuP.th=imuP.th-errImuTh;
        double delta_x = (odom.vx * cos(imuP.th) - odom.vy * sin(imuP.th))*period;
        double delta_y = (odom.vx * sin(imuP.th) + odom.vy * cos(imuP.th))*period;

        odomP.x+=delta_x;
        odomP.y+=delta_y;

        double currentTh=imuP.th-lastImuP.th;
        lastImuP.th=imuP.th;
        double delta_x1 = (odom.vx * cos((currentTh-errImuTh)) - odom.vy * sin((currentTh-errImuTh)))*period;
        double delta_y1= (odom.vx * sin((currentTh-errImuTh)) + odom.vy * cos((currentTh-errImuTh)))*period;

         //p.x+=delta_x1;
         //p.y+=delta_y1;
         p.th+=currentTh;

         while(p.th>PI+0.1)
         {
             p.th-=PI;
         }
         while(p.th<(-PI-0.1))
         {
             p.th+=PI;
         }
         //cout<<p.x<<" "<<p.y<<" "<<p.th<<" "<<p.currentCrossPoint.x<<" "<<p.currentCrossPoint.y<<endl;
         stringstream xS,yS,thS,pointXS,pointYS,imuPS;
         xS<<p.x;
         yS<<p.y;
         thS<<p.th;
         pointXS<<p.currentCrossPoint.x;
         pointYS<<p.currentCrossPoint.y;
         imuPS<<(imuP.th-errImuTh);
         //string lineString=xS.str()+" "+yS.str()+" "+thS.str()+" "+pointXS.str()+" "+pointYS.str()+" "+imuPS.str();

         stringstream odomXS,odomYS;
         odomXS<<odomP.x;
         odomYS<<odomP.y;
         string lineString=odomXS.str()+" "+odomYS.str()+" "+imuPS.str();
         outfile<<lineString<<endl;
         cout<<odomP.x<<" "<<odomP.y<<" "<<imuP.th<<endl;
    }
}
