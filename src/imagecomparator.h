#ifndef IMAGECOMPARATOR_H
#define IMAGECOMPARATOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "histogram1d.h"
#include "colorhistogram.h"

using namespace std;
using namespace cv;

class ImageComparator
{
public:
    ImageComparator():div(32){
    }

    void setColorReduction(int factor);
    int getColorReduction() const;
    void setReferenceImage1D(const Mat & image);
    double compare1D(const Mat &image);

    void setReferenceImageHSV(const Mat & image);
    vector<double> compareHSV(const Mat &image);

    void setReferenceImageRGB(const Mat & image);
    vector<double> compareRGB(const Mat &image);

    void setReferenceImageRGB2(const Mat & image);
    double compareRGB2(const Mat &image);
private:
    Mat reference;
    Mat input;
    MatND refH,refS,refV;
    MatND inputH,inputS,inputV;
    MatND refR,refG,refB,refRGB;
    MatND inputR,inputG,inputB,inputRGB;

    ColorHistogram hist;
    int div;

};

#endif // IMAGECOMPARATOR_H
