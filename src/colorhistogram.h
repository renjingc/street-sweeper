#ifndef COLORHISTOGRAM_H
#define COLORHISTOGRAM_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class ColorHistogram
{
public:
    ColorHistogram();
    MatND getHistogram(const Mat &image);
    //Mat getHistogramImage(const Mat &image);
    SparseMat getSparseHistogram(const Mat &image);
    MatND getabHistogram(const Mat &image);

    vector<MatND> getHSVHistogram(const Mat &src);
    Mat getHSVHistogramImage(const Mat &image);
    Mat getHSVHistogramImage(const vector<MatND>& histHSV);
    Mat getHSVHistogramData(const vector<MatND>& histHSV);
    Mat getHSVHistogramDataH(const vector<MatND> &histHSV);
    Mat getHSVHistogramDataHS(const vector<MatND> &histHSV);

    vector<MatND> getRGBHistogram(const Mat &image);
    Mat getRGBHistogramImage(const Mat& image);
    Mat getRGBHistogramImage(const vector<MatND>& histRGB);
    Mat getRGBHistogramData(const vector<MatND>& histRGB);

    Mat colorReduce(const Mat &image,int div);

private:
    int histSize[3];
    float hranges[2];
    const float* ranges[3];
    int channels[3];
    int channels_r[1],channels_g[1],channels_b[1];
    int channels_h[1],channels_s[1],channels_v[1];
};

#endif // COLORHISTOGRAM_H
