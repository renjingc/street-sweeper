#ifndef HSVCALCHISTOGRAM_H
#define HSVCALCHISTOGRAM_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include "opencv2/core/core.hpp"
//#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"

//#include "opencv2/calib3d.hpp"
//#include "opencv2/xfeatures2d.hpp"
//#include "opencv2/contrib/contrib.hpp"

using namespace std;
using namespace cv;

class HSVCalcHistogram
{
private:
    int histSize[3];         //直方图项的数量
    float hranges[2];        //h通道像素的最小和最大值
    float sranges[2];
    float vranges[2];
    const float *ranges[3];  //各通道的范围
    int channels[3];         //三个通道
    int dims;

public:
    HSVCalcHistogram(int hbins=30, int sbins=32, int vbins=32)
    {
        histSize[0]=hbins;
        histSize[1]=sbins;
        histSize[2]=vbins;
        hranges[0]=0; hranges[1]=180;
        sranges[0]=0; sranges[1]=256;
        vranges[0]=0; vranges[1]=256;
        ranges[0]=hranges;
        ranges[1]=sranges;
        ranges[2]=vranges;
        channels[0]=0;
        channels[1]=1;
        channels[2]=2;
        dims=3;
    }

    Mat getHistogram(const Mat &image);
    void getHistogramImage(const Mat &image);
    void getHistogramHS(const Mat image ,vector<float>& HS);
};
#endif // HSVCALCHISTOGRAM_H

