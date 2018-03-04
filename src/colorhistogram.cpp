#include "colorhistogram.h"

ColorHistogram::ColorHistogram()
{
    histSize[0]=histSize[1]=histSize[2]=256;
    hranges[0]=0.0;
    hranges[1]=256.0;

    ranges[0]=ranges[1]=ranges[2]=hranges;

    channels[0]=0;
    channels[1]=1;
    channels[2]=2;

    channels_b[0] = 0;
    channels_g[0] = 1;
    channels_r[0] = 2;

    channels_h[0] = 0;
    channels_s[0] = 1;
    channels_v[0] = 2;
}

vector<MatND> ColorHistogram::getRGBHistogram(const Mat &image)
{
    MatND hist_r,hist_g,hist_b;
    vector<MatND> histRGB;
    hranges[0]=0.0;
    hranges[1]=256.0;

    //R
    calcHist(&image,1,channels_r,Mat(),hist_r,1,histSize,ranges);//分别计算R，G，B的直方图分布

    //G
    calcHist(&image,1,channels_g,Mat(),hist_g,1,histSize,ranges);

    //B
    calcHist(&image,1,channels_b,Mat(),hist_b,1,histSize,ranges);


    histRGB.push_back(hist_r);
    histRGB.push_back(hist_g);
    histRGB.push_back(hist_b);

    return histRGB;
}

Mat ColorHistogram::getRGBHistogramImage(const Mat& image)
{
    vector<MatND> histRGB=getRGBHistogram(image);
    double max_val_r,max_val_g,max_val_b;

    minMaxLoc(histRGB[0],0,&max_val_r,0,0);//计算直方图中统计最大值
    minMaxLoc(histRGB[1],0,&max_val_g,0,0);
    minMaxLoc(histRGB[2],0,&max_val_b,0,0);

    Mat histImage(histSize[0],3*histSize[0],CV_8UC3,Scalar(0,0,0)); //定义一个显示直方图的图片，长256*3 高256

    for (int i =0;i<histSize[0];i++)
    {
        //R，G，B= i的统计值
        float binVal_r = histRGB[0].at<float>(i);
        float binVal_g = histRGB[1].at<float>(i);
        float binVal_b = histRGB[2].at<float>(i);

        //统一R，G，B统计值的大小，以高度的90%封顶
        int intensity_r = static_cast<int>(0.9*histSize[0]*binVal_r/max_val_r);

        int intensity_g = static_cast<int>(0.9*histSize[0]*binVal_g/max_val_g);

        int intensity_b = static_cast<int>(0.9*histSize[0]*binVal_b/max_val_b);

        //画出R，G，B的直方图直线
        line(histImage,Point(i,histImage.rows),Point(i,histImage.rows-intensity_r),Scalar(0,0,255));
        line(histImage,Point(i+histSize[0],histImage.rows),Point(i+histSize[0],histImage.rows-intensity_g),Scalar(0,255,0));
        line(histImage,Point(i+histSize[0]*2,histImage.rows),Point(i+histSize[0]*2,histImage.rows-intensity_b),Scalar(255,0,0));
    }

    return histImage;
}

Mat ColorHistogram::getRGBHistogramImage(const vector<MatND>& histRGB)
{
    double max_val_r,max_val_g,max_val_b;

    minMaxLoc(histRGB[0],0,&max_val_r,0,0);//计算直方图中统计最大值
    minMaxLoc(histRGB[1],0,&max_val_g,0,0);
    minMaxLoc(histRGB[2],0,&max_val_b,0,0);

    Mat histImage(histSize[0],3*histSize[0],CV_8UC3,Scalar(0,0,0)); //定义一个显示直方图的图片，长256*3 高256

    for (int i =0;i<histSize[0];i++)
    {
        //R，G，B= i的统计值
        float binVal_r = histRGB[0].at<float>(i);
        float binVal_g = histRGB[1].at<float>(i);
        float binVal_b = histRGB[2].at<float>(i);

        //统一R，G，B统计值的大小，以高度的90%封顶
        int intensity_r = static_cast<int>(0.9*histSize[0]*binVal_r/max_val_r);

        int intensity_g = static_cast<int>(0.9*histSize[0]*binVal_g/max_val_g);

        int intensity_b = static_cast<int>(0.9*histSize[0]*binVal_b/max_val_b);

        //画出R，G，B的直方图直线
        line(histImage,Point(i,histImage.rows),Point(i,histImage.rows-intensity_r),Scalar(0,0,255));
        line(histImage,Point(i+histSize[0],histImage.rows),Point(i+histSize[0],histImage.rows-intensity_g),Scalar(0,255,0));
        line(histImage,Point(i+histSize[0]*2,histImage.rows),Point(i+histSize[0]*2,histImage.rows-intensity_b),Scalar(255,0,0));
    }

    return histImage;
}

Mat ColorHistogram::getRGBHistogramData(const vector<MatND>& histRGB)
{
    double max_val_r,max_val_g,max_val_b;

    minMaxLoc(histRGB[0],0,&max_val_r,0,0);//计算直方图中统计最大值
    minMaxLoc(histRGB[1],0,&max_val_g,0,0);
    minMaxLoc(histRGB[2],0,&max_val_b,0,0);

    Mat histImage(1,3*histSize[0],CV_8U); //定义一个显示直方图的图片，长256*1 高256

    for (int i =0;i<histSize[0];i++)
    {
        //R，G，B= i的统计值
        float binVal_r = histRGB[0].at<float>(i);
        float binVal_g = histRGB[1].at<float>(i);
        float binVal_b = histRGB[2].at<float>(i);

        //统一R，G，B统计值的大小，以高度的90%封顶
        int intensity_r = static_cast<int>(0.9*histSize[0]*binVal_r/max_val_r);

        int intensity_g = static_cast<int>(0.9*histSize[0]*binVal_g/max_val_g);

        int intensity_b = static_cast<int>(0.9*histSize[0]*binVal_b/max_val_b);

        histImage.ptr<uchar>(0)[i]=intensity_r;
        histImage.ptr<uchar>(0)[i+histSize[0]]=intensity_g;
        histImage.ptr<uchar>(0)[i+histSize[0]*2]=intensity_b;
    }

    return histImage;
}

MatND ColorHistogram::getHistogram(const Mat &image)
{
    MatND hist;
    calcHist(&image,
             1,//仅使用一张图
             channels,//通道数量
             Mat(),
             hist,
             3,//返回三维直方图
             histSize,
             ranges
             );
    return hist;
}
/*
 *获取稀疏矩阵
 */
SparseMat ColorHistogram::getSparseHistogram(const Mat &image)
{
    SparseMat hist(3,histSize,CV_32F);
    calcHist(&image,
             1,
             channels,
             Mat(),
             hist,
             3,
             histSize,
             ranges
             );
    return hist;
}


vector<MatND> ColorHistogram::getHSVHistogram(const Mat &src)
{
    //CV_BGR2HSV＿FULL   实现的映射是 H * 255 / 360 --->H ,
    //所以利用_FULL 这个转换得到的H通道图像的范围为 0-255


    MatND hist_h,hist_s,hist_v;
    vector<MatND> histHSV;
    Mat hsv;
    cvtColor(src, hsv, CV_BGR2HSV);
    float hranges_h[]={0.0,180.0};
    float hranges_s[]={0.0,256.0};
    float hranges_v[]={0.0,256.0};
    const float* ranges_h=hranges_h;
    const float* ranges_s=hranges_s;
    const float* ranges_v=hranges_v;
    const float* ranges_hsv[]={ranges_h,ranges_s,ranges_v};
    //H
    calcHist(&hsv,1,channels_h,Mat(),hist_h,1,histSize,ranges_hsv);//分别计算H，S，V的直方图分布

    //S
    calcHist(&hsv,1,channels_s,Mat(),hist_s,1,histSize,ranges_hsv);

    //V
    calcHist(&hsv,1,channels_v,Mat(),hist_v,1,histSize,ranges_hsv);

    histHSV.push_back(hist_h);
    histHSV.push_back(hist_s);
    histHSV.push_back(hist_v);
    cout<<"hist_h:"<<hist_h.rows<<" "<<hist_h.cols<<endl;
    cout<<"hist_s:"<<hist_s.rows<<" "<<hist_s.cols<<endl;
    cout<<"hist_v:"<<hist_v.rows<<" "<<hist_v.cols<<endl;
    return histHSV;
}

Mat ColorHistogram::getHSVHistogramImage(const Mat &image)
{
    vector<MatND> histHSV=getHSVHistogram(image);
    double max_val_h,max_val_s,max_val_v;

    minMaxLoc(histHSV[0],0,&max_val_h,0,0);//计算直方图中统计最大值
    minMaxLoc(histHSV[1],0,&max_val_s,0,0);
    minMaxLoc(histHSV[2],0,&max_val_v,0,0);

    Mat histImage(histSize[0],3*histSize[0],CV_8UC3,Scalar(0,0,0)); //定义一个显示直方图的图片，长256*3 高256

    for (int i =0;i<histSize[0];i++)
    {
        //H，S，V= i的统计值
        float binVal_h = histHSV[0].at<float>(i);
        float binVal_s = histHSV[1].at<float>(i);
        float binVal_v = histHSV[2].at<float>(i);

        //统一H，S，V统计值的大小，以高度的90%封顶
        int intensity_h = static_cast<int>(0.9*histSize[0]*binVal_h/max_val_h);

        int intensity_s = static_cast<int>(0.9*histSize[0]*binVal_s/max_val_s);

        int intensity_v = static_cast<int>(0.9*histSize[0]*binVal_v/max_val_v);

        //画出H，S，V的直方图直线
        line(histImage,Point(i,histImage.rows),Point(i,histImage.rows-intensity_h),Scalar(0,0,255));
        line(histImage,Point(i+histSize[0],histImage.rows),Point(i+histSize[0],histImage.rows-intensity_s),Scalar(0,255,0));
        line(histImage,Point(i+histSize[0]*2,histImage.rows),Point(i+histSize[0]*2,histImage.rows-intensity_v),Scalar(255,0,0));
    }

    return histImage;
}


Mat ColorHistogram::getHSVHistogramImage(const vector<MatND> &histHSV)
{
    double max_val_h,max_val_s,max_val_v;

    minMaxLoc(histHSV[0],0,&max_val_h,0,0);//计算直方图中统计最大值
    minMaxLoc(histHSV[1],0,&max_val_s,0,0);
    minMaxLoc(histHSV[2],0,&max_val_v,0,0);

    Mat histImage(histSize[0],3*histSize[0],CV_8UC3,Scalar(0,0,0)); //定义一个显示直方图的图片，长256*3 高256

    for (int i =0;i<histSize[0];i++)
    {
        //H，S，V= i的统计值
        float binVal_h = histHSV[0].at<float>(i);
        float binVal_s = histHSV[1].at<float>(i);
        float binVal_v = histHSV[2].at<float>(i);

        //统一H，S，V统计值的大小，以高度的90%封顶
        int intensity_h = static_cast<int>(0.9*histSize[0]*binVal_h/max_val_h);

        int intensity_s = static_cast<int>(0.9*histSize[0]*binVal_s/max_val_s);

        int intensity_v = static_cast<int>(0.9*histSize[0]*binVal_v/max_val_v);

        //画出H，S，V的直方图直线
        line(histImage,Point(i,histImage.rows),Point(i,histImage.rows-intensity_h),Scalar(0,0,255));
        line(histImage,Point(i+histSize[0],histImage.rows),Point(i+histSize[0],histImage.rows-intensity_s),Scalar(0,255,0));
        line(histImage,Point(i+histSize[0]*2,histImage.rows),Point(i+histSize[0]*2,histImage.rows-intensity_v),Scalar(255,0,0));
    }

    return histImage;
}

Mat ColorHistogram::getHSVHistogramData(const vector<MatND> &histHSV)
{
    double max_val_h,max_val_s,max_val_v;

    minMaxLoc(histHSV[0],0,&max_val_h,0,0);//计算直方图中统计最大值
    minMaxLoc(histHSV[1],0,&max_val_s,0,0);
    minMaxLoc(histHSV[2],0,&max_val_v,0,0);

    Mat histImage(1,3*histSize[0],CV_8U); //定义一个显示直方图的图片，长256*3 高256

    for (int i =0;i<histSize[0];i++)
    {
        //H，S，V= i的统计值
        float binVal_h = histHSV[0].at<float>(i);
        float binVal_s = histHSV[1].at<float>(i);
        float binVal_v = histHSV[2].at<float>(i);

        //统一H，S，V统计值的大小，以高度的90%封顶
        int intensity_h = static_cast<int>(0.9*histSize[0]*binVal_h/max_val_h);

        int intensity_s = static_cast<int>(0.9*histSize[0]*binVal_s/max_val_s);

        int intensity_v = static_cast<int>(0.9*histSize[0]*binVal_v/max_val_v);

        histImage.ptr<uchar>(0)[i]=intensity_h;
        histImage.ptr<uchar>(0)[i+histSize[0]]=intensity_s;
        histImage.ptr<uchar>(0)[i+histSize[0]*2]=intensity_v;
    }

    return histImage;
}
Mat ColorHistogram::getHSVHistogramDataH(const vector<MatND> &histHSV)
{
    double max_val_h,max_val_s,max_val_v;

    minMaxLoc(histHSV[0],0,&max_val_h,0,0);//计算直方图中统计最大值
    minMaxLoc(histHSV[1],0,&max_val_s,0,0);
    minMaxLoc(histHSV[2],0,&max_val_v,0,0);

    Mat histImage(1,histSize[0],CV_8U); //定义一个显示直方图的图片，长256*1 高256

    for (int i =0;i<histSize[0];i++)
    {
        //H，S，V= i的统计值
        float binVal_h = histHSV[0].at<float>(i);
        float binVal_s = histHSV[1].at<float>(i);
        float binVal_v = histHSV[2].at<float>(i);

        //统一H，S，V统计值的大小，以高度的90%封顶
        int intensity_h = static_cast<int>(0.9*histSize[0]*binVal_h/max_val_h);

        int intensity_s = static_cast<int>(0.9*histSize[0]*binVal_s/max_val_s);

        int intensity_v = static_cast<int>(0.9*histSize[0]*binVal_v/max_val_v);

        histImage.ptr<uchar>(0)[i]=intensity_v;
    }

    return histImage;
}

Mat ColorHistogram::getHSVHistogramDataHS(const vector<MatND> &histHSV)
{
    double max_val_h,max_val_s,max_val_v;

    minMaxLoc(histHSV[0],0,&max_val_h,0,0);//计算直方图中统计最大值
    minMaxLoc(histHSV[1],0,&max_val_s,0,0);
    minMaxLoc(histHSV[2],0,&max_val_v,0,0);

    Mat histImage(1,histSize[0]*2,CV_8U); //定义一个显示直方图的图片，长256*2 高256

    for (int i =0;i<histSize[0];i++)
    {
        //H，S，V= i的统计值
        float binVal_h = histHSV[0].at<float>(i);
        float binVal_s = histHSV[1].at<float>(i);
        float binVal_v = histHSV[2].at<float>(i);

        //统一H，S，V统计值的大小，以高度的90%封顶
        int intensity_h = static_cast<int>(0.9*histSize[0]*binVal_h/max_val_h);

        int intensity_s = static_cast<int>(0.9*histSize[0]*binVal_s/max_val_s);

        int intensity_v = static_cast<int>(0.9*histSize[0]*binVal_v/max_val_v);

        histImage.ptr<uchar>(0)[i]=intensity_v;
        histImage.ptr<uchar>(0)[i+histSize[0]]=intensity_s;
    }

    return histImage;
}

Mat ColorHistogram::colorReduce(const Mat &image,int div)
{
#define ITERATOR_FUN

#ifdef ITERATOR_FUN
  int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
  // mask used to round the pixel value
  uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0

  Mat_<Vec3b>::const_iterator it= image.begin<Vec3b>();
  Mat_<Vec3b>::const_iterator itend= image.end<Vec3b>();

  // Set output image (always 1-channel)
  Mat result(image.rows,image.cols,image.type());
  Mat_<Vec3b>::iterator itr= result.begin<Vec3b>();

  for ( ; it!= itend; ++it, ++itr)
  {
    (*itr)[0]= ((*it)[0]&mask) + div/2;
    (*itr)[1]= ((*it)[1]&mask) + div/2;
    (*itr)[2]= ((*it)[2]&mask) + div/2;
  }
  return result;

#endif

#ifdef AT_FUN
  Mat result = image.clone();
  int rows = result.rows;
  int cols = result.cols;
  for(int i = 0;i < rows;i++)
  {
      for(int j = 0;j < cols;j++)
      {
          result.at<Vec3b>(i,j)[0] =  result.at<Vec3b>(i,j)[0]/div*div + div/2;
          result.at<Vec3b>(i,j)[1] =  result.at<Vec3b>(i,j)[1]/div*div + div/2;
          result.at<Vec3b>(i,j)[2] =  result.at<Vec3b>(i,j)[2]/div*div + div/2;
      }
  }
  return outputImage;

#endif

#ifdef PTR_FUN
  result = result.clone();
  int rows = result.rows;
  int cols = result.cols*result.channels();
  for(int i = 0;i < rows;i++)
  {
       uchar* data = result.ptr<uchar>(i);
       uchar* dataout = result.ptr<uchar>(i);
       for(int j = 0;j < cols;j++)
       {
          dataout[j] = dataout[j]/div*div + div/2;
       }
  }
  return result;
#endif
}
