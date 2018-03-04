#include "imagecomparator.h"

void ImageComparator::setColorReduction(int factor)
{
    div=factor;
}

int ImageComparator::getColorReduction() const
{
    return div;
}

void ImageComparator::setReferenceImage1D(const Mat &image)
{
    reference=hist.colorReduce(image,div);
    refH=hist.getHistogram(reference);
}

double ImageComparator::compare1D(const Mat &image)
{
   input=hist.colorReduce(image,div);
   inputH=hist.getHistogram(input);
   return compareHist(refH,inputH,CV_COMP_INTERSECT);//交叉测量法
}

void ImageComparator::setReferenceImageHSV(const Mat &image)
{
    vector<MatND> HSVHisto;
    reference=hist.colorReduce(image,div);
    HSVHisto=hist.getHSVHistogram(image);
    normalize(HSVHisto[0], HSVHisto[0], 0, 1, NORM_MINMAX, -1, Mat() );
    normalize(HSVHisto[1], HSVHisto[1], 0, 1, NORM_MINMAX, -1, Mat() );
    normalize(HSVHisto[2], HSVHisto[2], 0, 1, NORM_MINMAX, -1, Mat() );
    HSVHisto[2].copyTo(refH);
    HSVHisto[1].copyTo(refS);
    HSVHisto[0].copyTo(refV);
}

vector<double> ImageComparator::compareHSV(const Mat &image)
{
   vector<MatND> HSVHisto;
   vector<double> distHist;
   input=hist.colorReduce(image,div);
   HSVHisto=hist.getHSVHistogram(image);
   normalize(HSVHisto[0], HSVHisto[0], 0, 1, NORM_MINMAX, -1, Mat() );
   normalize(HSVHisto[1], HSVHisto[1], 0, 1, NORM_MINMAX, -1, Mat() );
   normalize(HSVHisto[2], HSVHisto[2], 0, 1, NORM_MINMAX, -1, Mat() );

   HSVHisto[2].copyTo(inputH);
   HSVHisto[1].copyTo(inputS);
   HSVHisto[0].copyTo(inputV);

   distHist.push_back(compareHist(refH,inputH,CV_COMP_INTERSECT));//交叉测量法
   distHist.push_back(compareHist(refS,inputS,CV_COMP_INTERSECT));
   distHist.push_back(compareHist(refV,inputV,CV_COMP_INTERSECT));
   return distHist;
}


void ImageComparator::setReferenceImageRGB(const Mat &image)
{
    vector<MatND> RGBHisto;
    reference=hist.colorReduce(image,div);
    RGBHisto=hist.getRGBHistogram(image);

    normalize(RGBHisto[0], RGBHisto[0], 0, 1, NORM_MINMAX, -1, Mat() );
    normalize(RGBHisto[1], RGBHisto[1], 0, 1, NORM_MINMAX, -1, Mat() );
    normalize(RGBHisto[2], RGBHisto[2], 0, 1, NORM_MINMAX, -1, Mat() );

    RGBHisto[0].copyTo(refR);
    RGBHisto[1].copyTo(refG);
    RGBHisto[2].copyTo(refB);
}

vector<double> ImageComparator::compareRGB(const Mat &image)
{
   vector<MatND> RGBHisto;
   vector<double> distHist;
   input=hist.colorReduce(image,div);
   RGBHisto=hist.getRGBHistogram(image);

   normalize(RGBHisto[0], RGBHisto[0], 0, 1, NORM_MINMAX, -1, Mat() );
   normalize(RGBHisto[1], RGBHisto[1], 0, 1, NORM_MINMAX, -1, Mat() );
   normalize(RGBHisto[2], RGBHisto[2], 0, 1, NORM_MINMAX, -1, Mat() );

   RGBHisto[0].copyTo(inputR);
   RGBHisto[1].copyTo(inputG);
   RGBHisto[2].copyTo(inputB);

   distHist.push_back(compareHist(refR,inputR,CV_COMP_INTERSECT));//交叉测量法
   distHist.push_back(compareHist(refR,inputG,CV_COMP_INTERSECT));
   distHist.push_back(compareHist(refR,inputB,CV_COMP_INTERSECT));

   return distHist;
}

void ImageComparator::setReferenceImageRGB2(const Mat &image)
{
    MatND RGBHisto;
    reference=hist.colorReduce(image,div);
    RGBHisto=hist.getHistogram(image);

    normalize(RGBHisto, RGBHisto, 0, 1, NORM_MINMAX, -1, Mat() );

    RGBHisto.copyTo(refRGB);
}

double ImageComparator::compareRGB2(const Mat &image)
{
   MatND RGBHisto;
   double distHist;
   input=hist.colorReduce(image,div);
   RGBHisto=hist.getHistogram(image);

   normalize(RGBHisto, RGBHisto, 0, 1, NORM_MINMAX, -1, Mat() );

   RGBHisto.copyTo(inputRGB);

   distHist=compareHist(refRGB,inputRGB,CV_COMP_INTERSECT);//交叉测量法

   return distHist;
}
