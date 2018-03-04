#include "histogram1d.h"

Histogram1D::Histogram1D()
{
    histSize[0]=256;
    hranges[0]=0.0;
    hranges[1]=256.0;
    ranges[0]=hranges;
    channels[0]=0;
}

void Histogram1D::setHistSize(int size)
{
    histSize[0]=size;
}

int Histogram1D::getHistSize() const
{
    return histSize[0];
}

void Histogram1D::setHranges(float minValue, float maxValue)
{
    hranges[0]=minValue;
    hranges[1]=maxValue;
}

float* Histogram1D::getHranges()
{
    return hranges;
}

float Histogram1D::getMaxValue() const
{
    return hranges[1];
}
float Histogram1D::getMinValue() const
{
    return hranges[0];
}

void Histogram1D::setChannels(int channel)
{
    channels[0]=channel;
}
int Histogram1D::getChannels() const
{
    return channels[0];
}

MatND Histogram1D::getHistogram(const Mat &image)
{
    MatND hist;
    calcHist(&image,
             1,//计算单张图像的直方图
             channels,//通道数量
             Mat(),//不适用图像作为掩码
             hist,//返回的直方图
             1,//这是1D的直方图
             histSize,//项的数量
             ranges//像素值的范围
             );
    return hist;
}

Mat Histogram1D::getHistogramImage(const Mat &image)
{
    MatND hist=getHistogram(image);

    //获取最大最小值
    double maxValue=0;
    double minValue=0;
    minMaxLoc(hist,&minValue,&maxValue,0,0);
    //显示直方图的图像
    Mat histImg(histSize[0],histSize[0],CV_8U,Scalar(255));

    //设置最高点为nbins的%90
    int hpt=static_cast<int>(0.9*histSize[0]);

    //每个条目都绘制一条直线
    //显示图坐标
    /*
     * 0    |
     *      |
     *      |
     * 256  |____________
     *     0             256
     */
    for(int h=0;h<histSize[0];h++)
    {
        float binVal=hist.at<float>(h);
        int intensity=static_cast<int>(binVal*hpt/maxValue);//将数量转化为与最大值的比例的乘以0.9的256
        //两点之间绘制一条直线
        line(histImg,Point(h,histSize[0]),Point(h,histSize[0]-intensity),Scalar(0));//::all(0));
    }
    cout<<"minValue:"<<minValue<<endl;
    cout<<"maxValue:"<<maxValue<<endl;
    return histImg;
}

/*
 * 使用查表法对图像进行修改
 */
Mat Histogram1D::applyLooUp(const Mat &image,const MatND &lookup)
{
    Mat result;
    LUT(image,lookup,result);//对图像应用查表法以生成新图像
    return result;
}

/*
 * 使用像素重映射来提高对比度
 */
Mat Histogram1D::stretch(const Mat &image, int minValue)
{
    MatND hist=getHistogram(image);

    //忽略两端不足minValue的像素,再进行0-255的线性重映射

    //寻找直方图的最左端,即直方图的最小值
    int min=0;
    for(;min<histSize[0];min++)
    {
        //cout<<hist.at<float>(min)<<endl;
        if(hist.at<float>(min)>minValue)
            break;
    }
    //寻找直方图的最右端
    int max=histSize[0]-1;
    for(;max>0;max--)
    {
        if(hist.at<float>(max)>minValue)
            break;
    }

    //创建查找表
    int dim(256);
    MatND lookup(1,
               &dim,
               CV_8U);
    for(int i=0;i<256;i++)
    {
        //确保数值位于min和max之间
        if(i<min)
            lookup.at<uchar>(i)=0;
        else if(i>max)
            lookup.at<uchar>(i)=255;
        //线性映射
        else
            lookup.at<uchar>(i)=static_cast<uchar>(255.0*(i-min)/(max-min)+0.5);
    }
    Mat result;
    result=applyLooUp(image,lookup);
    return result;
}
/*
 * 利用直方图均衡化进行提高图像对比度
 */
Mat Histogram1D::equalize(const Mat &image)
{
    Mat result;
    equalizeHist(image,result);//用来使灰度图象直方图均衡化
    return result;
}
