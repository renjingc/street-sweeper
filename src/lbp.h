#ifndef LBP_H
#define LBP_H

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


#include <vector>
#include <algorithm>
#include <functional>
#include <fstream>
#include <map>

using namespace std;
using namespace cv;

void LBP (IplImage *src,IplImage *dst);
void olbp(InputArray src, OutputArray dst);
Mat elbp(InputArray src, int radius, int neighbors);
Mat spatial_histogram(InputArray _src, int numPatterns,
                             int grid_x, int grid_y, bool normed,bool uniform);

void compute_uniformLBP(CvMat src,vector<double>& vec_out);
inline void ulbp_(InputArray _src, OutputArray _dst, int radius, int neighbors, vector<int> m_uniform);

#endif // LBP_H

