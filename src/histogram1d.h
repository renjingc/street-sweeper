#ifndef HISTOGRAM1D_H
#define HISTOGRAM1D_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class Histogram1D
{
public:
    Histogram1D();
    void setHistSize(int size);
    int getHistSize() const;
    void setHranges(float minValue,float maxValue);
    float* getHranges();
    float getMinValue() const;
    float getMaxValue() const;
    void setChannels(int channel);
    int getChannels() const;
    MatND getHistogram(const Mat &image);//计算灰度直方图1D
    Mat getHistogramImage(const Mat &image);//计算1D直方图，并用图像显示
    Mat applyLooUp(const Mat &image, const MatND &lookup);
    Mat stretch(const Mat &image,int minValue=0);
    Mat equalize(const Mat &image);

private:
    int histSize[1];//项的数量
    float hranges[2];//像素的最小及最大值
    const float *ranges[1];
    int channels[1];//单通道


};

#endif // HISTOGRAM1D_H
