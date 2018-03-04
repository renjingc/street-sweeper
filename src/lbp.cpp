#include "lbp.h"

//基于旧版本的opencv的LBP算法opencv1.0
void LBP (IplImage *src,IplImage *dst)
{
    int tmp[8]={0};
    CvScalar s;

    IplImage * temp = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U,1);
    uchar *data=(uchar*)src->imageData;
    int step=src->widthStep;  // 图像位宽

    cout<<"step"<<step<<endl;

    for (int i=1;i<src->height-1;i++)
      for(int j=1;j<src->width-1;j++)
      {
          int sum=0;
          if(data[(i-1)*step+j-1]>data[i*step+j])  // 左上角
            tmp[0]=1;

          if(data[i*step+(j-1)]>data[i*step+j])   // 上方
            tmp[1]=1;

          if(data[(i+1)*step+(j-1)]>data[i*step+j])  // 右上角
            tmp[2]=1;

          if (data[(i+1)*step+j]>data[i*step+j])  // 右侧
            tmp[3]=1;

          if (data[(i+1)*step+(j+1)]>data[i*step+j])  // 右下角
            tmp[4]=1;

          if(data[i*step+(j+1)]>data[i*step+j])  // 下方
            tmp[5]=1;

          if(data[(i-1)*step+(j+1)]>data[i*step+j])  // 左下角
            tmp[6]=1;

          if(data[(i-1)*step+j]>data[i*step+j])  // 左侧
            tmp[7]=1;

          //计算LBP编码
            s.val[0]=(tmp[0]*1+tmp[1]*2+tmp[2]*4+tmp[3]*8+tmp[4]*16+tmp[5]*32+tmp[6]*64+tmp[7]*128);
            cvSet2D(dst,i,j,s);//写入LBP图像
      }
}
// src为输入图像，dst为输出图像，radius为半径，neighbor为计算当前点LBP所需的邻域像素点数，也就是样本点个数
template <typename _Tp> static // 模板函数，根据不同的原始数据类型得到不同的结果
inline void elbp_(InputArray _src, OutputArray _dst, int radius, int neighbors)
{
    //get matrices
    Mat src = _src.getMat();
    // allocate memory for result因此不用在外部给_dst分配内存空间，输出数据类型都是int
    _dst.create(src.rows-2*radius, src.cols-2*radius, CV_32SC1);
    Mat dst = _dst.getMat();
    // zero
    dst.setTo(0);
    for(int n=0; n<neighbors; n++)
    {
        // sample points 获取当前采样点
        float x = static_cast<float>(-radius) * sin(2.0*CV_PI*n/static_cast<float>(neighbors));
        float y = static_cast<float>(radius) * cos(2.0*CV_PI*n/static_cast<float>(neighbors));
        // relative indices 下取整和上取整
        int fx = static_cast<int>(floor(x)); // 向下取整
        int fy = static_cast<int>(floor(y));
        int cx = static_cast<int>(ceil(x));  // 向上取整
        int cy = static_cast<int>(ceil(y));
        // fractional part 小数部分
        float tx = x - fx;
        float ty = y - fy;
        // set interpolation weights 设置四个点的插值权重
        float w1 = (1 - tx) * (1 - ty);
        float w2 =      tx  * (1 - ty);
        float w3 = (1 - tx) *      ty;
        float w4 =      tx  *      ty;
        // iterate through your data 循环处理图像数据
        for(int i=radius; i < src.rows-radius;i++)
        {
            for(int j=radius;j < src.cols-radius;j++)
            {
                // calculate interpolated value 计算插值，t表示四个点的权重和
                float t = w1*src.at<_Tp>(i+fy,j+fx) +w2*src.at<_Tp>(i+fy,j+cx) +w3*src.at<_Tp>(i+cy,j+fx) +w4*src.at<_Tp>(i+cy,j+cx);
                // floating point precision, so check some machine-dependent epsilon
                // std::numeric_limits<float>::epsilon()=1.192092896e-07F
                // 当t>=src(i,j)的时候取1，并进行相应的移位
                dst.at<int>(i-radius,j-radius) += ((t > src.at<_Tp>(i,j)) ||
                            (std::abs(t-src.at<_Tp>(i,j)) < std::numeric_limits<float>::epsilon())) << n;
            }
        }
    }
}

// 外部接口，根据不同的数据类型调用模板函数
static void elbp(InputArray src, OutputArray dst, int radius, int neighbors)
{
    int type = src.type();
    switch (type) {
    case CV_8SC1:   elbp_<char>(src,dst, radius, neighbors); break;
    case CV_8UC1:   elbp_<unsigned char>(src, dst, radius, neighbors); break;
    case CV_16SC1:  elbp_<short>(src,dst, radius, neighbors); break;
    case CV_16UC1:  elbp_<unsigned short>(src,dst, radius, neighbors); break;
    case CV_32SC1:  elbp_<int>(src,dst, radius, neighbors); break;
    case CV_32FC1:  elbp_<float>(src,dst, radius, neighbors); break;
    case CV_64FC1:  elbp_<double>(src,dst, radius, neighbors); break;
    default:
        string error_msg = format("Using Circle Local Binary Patterns for feature extraction only works                                     on single-channel images (given %d). Please pass the image data as a grayscale image!", type);
        CV_Error(CV_StsNotImplemented, error_msg);
        break;
    }
}
Mat elbp(InputArray src, int radius, int neighbors)
{
    Mat dst;
    elbp(src, dst, radius, neighbors);
    return dst;
}
// 原始LBP算子只是计算8邻域内的局部二值模式
template <typename _Tp> static
void olbp_(InputArray _src, OutputArray _dst)
{
    // get matrices
    Mat src = _src.getMat();
    // allocate memory for result
    _dst.create(src.rows-2, src.cols-2, CV_8UC1);
    Mat dst = _dst.getMat();
    // zero the result matrix
    dst.setTo(0);
    // calculate patterns
    for(int i=1;i<src.rows-1;i++)
    {
        for(int j=1;j<src.cols-1;j++)
        {
            _Tp center = src.at<_Tp>(i,j);
            unsigned char code = 0;
            code |= (src.at<_Tp>(i-1,j-1) >= center) << 7;
            code |= (src.at<_Tp>(i-1,j) >= center) << 6;
            code |= (src.at<_Tp>(i-1,j+1) >= center) << 5;
            code |= (src.at<_Tp>(i,j+1) >= center) << 4;
            code |= (src.at<_Tp>(i+1,j+1) >= center) << 3;
            code |= (src.at<_Tp>(i+1,j) >= center) << 2;
            code |= (src.at<_Tp>(i+1,j-1) >= center) << 1;
            code |= (src.at<_Tp>(i,j-1) >= center) << 0;
            dst.at<unsigned char>(i-1,j-1) = code;
        }
    }
}
// 外部接口，根据不同的数据类型调用模板函数
void olbp(InputArray src, OutputArray dst)
{
    switch (src.getMat().type()) {
    case CV_8SC1:   olbp_<char>(src,dst); break;
    case CV_8UC1:   olbp_<unsigned char>(src,dst); break;
    case CV_16SC1:  olbp_<short>(src,dst); break;
    case CV_16UC1:  olbp_<unsigned short>(src,dst); break;
    case CV_32SC1:  olbp_<int>(src,dst); break;
    case CV_32FC1:  olbp_<float>(src,dst); break;
    case CV_64FC1:  olbp_<double>(src,dst); break;
    default:
        string error_msg = format("Using Original Local Binary Patterns for feature extraction only works on single-channel images (given %d). Please pass the image data as a grayscale image!", src.getMat().type());
        CV_Error(CV_StsNotImplemented, error_msg);
        break;
    }
}
Mat olbp(InputArray src)
{
    Mat dst;
    olbp(src, dst);
    return dst;
}
static Mat histc_(const Mat& src, int minVal = 0, int maxVal = 255, bool normed = false)
{
    Mat result;
    // Establish the number of bins.
    int histSize = maxVal - minVal + 1;
    // Set the ranges.
    float range[] = { minVal, maxVal + 1 };
    const float* histRange = { range };
    // calc histogram
    calcHist(&src, 1, 0, Mat(), result, 1, &histSize, &histRange, true, false);
    // normalize
    if (normed)
    {
        result /= src.total();
    }
    return result.reshape(1, 1);
}
Mat histc(InputArray _src, int minVal, int maxVal, bool normed)
{
    Mat src = _src.getMat();
    switch (src.type())
    {
        case CV_8SC1:
            return histc_(Mat_<float>(src), minVal, maxVal, normed);
            break;
        case CV_8UC1:
            return histc_(src, minVal, maxVal, normed);
            break;
        case CV_16SC1:
            return histc_(Mat_<float>(src), minVal, maxVal, normed);
            break;
        case CV_16UC1:
            return histc_(src, minVal, maxVal, normed);
            break;
        case CV_32SC1:
            return histc_(Mat_<float>(src), minVal, maxVal, normed);
            break;
        case CV_32FC1:
            return histc_(src, minVal, maxVal, normed);
            break;
        default:
            CV_Error(CV_StsUnmatchedFormats, "This type is not implemented yet.");
            break;
    }
    return Mat();
}
const int uniformLBP[58]={0,1,2,3,4,6,7,8,12,14,15,16,24,28,30,31,32,48,56,60,62,63,64,
            96,112,120,124,126,127,128,129,131,135,143,159,191,192,193,195,
            199,207,223,224,225,227,231,239,240,241,243,247,248,249,251,252,
            253,254,255};//unifor LBP算子01跳变次数小于等于2的值,即uniform pattern
Mat uniformHistc(InputArray _src)
{
    Mat src = _src.getMat();
    Mat dst(1,59,src.type());
    vector<float> hist;
    float noUniform=0.0;
    for(int i=0;i<src.cols;i++)
    {
        bool ifUniform=false;
        for(int j=0;j<58;j++)
        {
            if(i==uniformLBP[j])
            {
                ifUniform=true;
                break;
            }
        }
        if(ifUniform)
        {
            hist.push_back(src.at<float>(0,i));
        }
        else
        {
            noUniform+=src.at<float>(0,i);
            //cout<<i<<" "<<src.at<float>(0,i)<<endl;
        }
    }
    hist.push_back(noUniform);
    for(int i=0;i<59;i++)
    {
        dst.at<float>(0,i)=hist[i];
    }
    return dst;
}

// 计算LBPM的空间直方图分布，得到一个一维向量
// src为LBPM是通过olbp或者elbp计算得到的
// numPatterns为计算LBP的模式数目，一般为2的幂
// grid_x和grid_y分别为每行或每列的block个数
// normed为是否进行归一化处理
Mat spatial_histogram(InputArray _src, int numPatterns,
                             int grid_x, int grid_y, bool normed,bool uniform=false)
{
    Mat src = _src.getMat();
    // allocate memory for the spatial histogram为LBPH分配内存空间
    //if(uniform==false)
        Mat result = Mat::zeros(grid_x * grid_y, numPatterns, CV_32FC1);
    //else if(uniform==true)
    //{
        Mat resultTemp = Mat::zeros(grid_x * grid_y, 256, CV_32FC1);
        Mat resultUniform = Mat::zeros(1, 59*grid_x*grid_y, CV_32FC1);
    //}
    // return matrix with zeros if no data was given，如果没有输入数据，返回的是0
    if(src.empty())
        return result.reshape(1,1);
    // calculate LBP patch size block的尺寸
    int width = src.cols/grid_x;
    int height = src.rows/grid_y;
    // initial result_row 初始化结果行
    int resultRowIdx = 0;
    // iterate through grid
    for(int i = 0; i < grid_y; i++)
    {
        for(int j = 0; j < grid_x; j++)
         {
            // 获取指定区域
            Mat src_cell = Mat(src, Range(i*height,(i+1)*height), Range(j*width,(j+1)*width));
            // 计算指定区域的直方图
            if(uniform==false)
            {
                Mat cell_hist = histc(src_cell, 0, (numPatterns-1), true);
                // copy to the result matrix 将计算得到的结果拷贝到每一行
                Mat result_row = result.row(resultRowIdx);
                cell_hist.reshape(1,1).convertTo(result_row, CV_32FC1);
                // increase row count in result matrix
                resultRowIdx++;
            }
            else if(uniform==true)
            {
                Mat cell_hist = histc(src_cell, 0, (256-1), true);
                // copy to the result matrix 将计算得到的结果拷贝到每一行
                Mat result_row = resultTemp.row(resultRowIdx);
                cell_hist.reshape(1,1).convertTo(result_row, CV_32FC1);
                // increase row count in result matrix
                resultRowIdx++;
            }
        }
    }
    if(uniform)
    {
        Mat t=resultTemp.reshape(1,1);
        for(int i=0;i<t.cols;i++)
        {
            //cout<<t.at<float>(0,i)<<" ";
        }
        resultUniform=uniformHistc(t);
        return resultUniform;
    }
    else
        // return result as reshaped feature vector
        return result.reshape(1,1);
}

// 第一种方法提取整幅图像的ULBP特征
// 计算ULBP索引表
bool ulbpIndex(vector<int> &uniform_lbp)
 {
        uniform_lbp.clear();
        for (int i = 0; i < 256; ++i)
        {
            int data = i;
            int dataTmp1 = data, dataTmp2 = data;
            int jump = 0; int tmp = -1;


            for (int k = 0; k < 8; ++k)
            {
                dataTmp1 = data >> 1;
                dataTmp2 = dataTmp1 << 1;
                int t = data ^ dataTmp2;
                data = dataTmp1;
                assert(t == 0 || t == 1);
                if (tmp == -1 && k == 0)
                {
                    tmp = t;
                    continue;
                }
                if (tmp != t)
                {
                    jump++;
                    tmp = t;
                }
            }


            if (jump < 3)
            {
                uniform_lbp.push_back(i);
            }
        }
        if (uniform_lbp.size() != 58)
        {
            return false;
        }
        return true;
}
// ULBP特征提取
template <typename _Tp> static
inline void ulbp_(InputArray _src, OutputArray _dst, int radius, int neighbors, vector<int> m_uniform)
{
        if (neighbors != 8 || m_uniform.size() != 58)
        {
            cout << "neighbor must be 8! and uniform size be 58!\n";
            system("pause");
            exit(-1);
        }


        //get matrices
        Mat src = _src.getMat();
        // allocate memory for result
        _dst.create(src.rows-2*radius, src.cols-2*radius, CV_32SC1);
        Mat dst = _dst.getMat();
        // zero
        dst.setTo(0);
        for(int n=0; n<neighbors; n++)
        {
            // sample points
            float x = static_cast<float>(-radius) * sin(2.0*CV_PI*n/static_cast<float>(neighbors));
            float y = static_cast<float>(radius) * cos(2.0*CV_PI*n/static_cast<float>(neighbors));
            // relative indices
            int fx = static_cast<int>(floor(x));
            int fy = static_cast<int>(floor(y));
            int cx = static_cast<int>(ceil(x));
            int cy = static_cast<int>(ceil(y));
            // fractional part
            float ty = y - fy;
            float tx = x - fx;
            // set interpolation weights
            float w1 = (1 - tx) * (1 - ty);
            float w2 =      tx  * (1 - ty);
            float w3 = (1 - tx) *      ty;
            float w4 =      tx  *      ty;
            // iterate through your data
            for(int i=radius; i < src.rows-radius;i++) {
                for(int j=radius;j < src.cols-radius;j++) {
                    // calculate interpolated value
                    float t = w1*src.at<_Tp>(i+fy,j+fx) + w2*src.at<_Tp>(i+fy,j+cx) + w3*src.at<_Tp>(i+cy,j+fx) + w4*src.at<_Tp>(i+cy,j+cx);
                    // floating point precision, so check some machine-dependent epsilon
                    dst.at<int>(i-radius,j-radius) += ((t > src.at<_Tp>(i,j)) ||
                        (std::abs(t-src.at<_Tp>(i,j)) < std::numeric_limits<float>::epsilon()))
                        << n;
                }
            }
        }


        for (int i = 0; i < dst.rows; ++i)
        {
            for (int j = 0; j < dst.cols; ++j)
            {
                int data = dst.at<int>(i, j);
                vector<int>::iterator iter = find(m_uniform.begin(), m_uniform.end(), data);
                if (iter == m_uniform.end())
                {
                    dst.at<int>(i, j) = 0;
                }
                else
                {
                    int new_data = iter - m_uniform.begin() ;
                    dst.at<int>(i, j) = new_data + 1;
                }
            }
        }
}


void compute_uniformLBP(CvMat src,vector<double>& vec_out)
{
    map<int,int> myMap;
    vector<int> myVec;
    map<int,int>::iterator itMap;
    vector<int>::iterator itVec;
    int lastLBP=0;
    int sumLBP=0;
    int tmp[8]={0};
    uchar *data=src.data.ptr;
    int step=src.step;
    for (int i=1;i<src.height-1;i++)
        for(int j=1;j<src.width-1;j++)
        {
            int sum=0;
            if(data[(i-1)*step+j-1]>data[i*step+j])
                tmp[0]=1;
            else
                tmp[0]=0;
            if(data[i*step+(j-1)]>data[i*step+j])
                tmp[1]=1;
            else
                tmp[1]=0;
            if(data[(i+1)*step+(j-1)]>data[i*step+j])
                tmp[2]=1;
            else
                tmp[2]=0;
            if (data[(i+1)*step+j]>data[i*step+j])
                tmp[3]=1;
            else
                tmp[3]=0;
            if (data[(i+1)*step+(j+1)]>data[i*step+j])
                tmp[4]=1;
            else
                tmp[4]=0;
            if(data[i*step+(j+1)]>data[i*step+j])
                tmp[5]=1;
            else
                tmp[5]=0;
            if(data[(i-1)*step+(j+1)]>data[i*step+j])
                tmp[6]=1;
            else
                tmp[6]=0;
            if(data[(i-1)*step+j]>data[i*step+j])
                tmp[7]=1;
            else
                tmp[7]=0;
            //计算LBP编码
            sum=(tmp[0]*1+tmp[1]*2+tmp[2]*4+tmp[3]*8+tmp[4]*16+tmp[5]*32+tmp[6]*64+tmp[7]*128);
            myMap[sum]++;
        }
        for (int i=0; i<58; i++)
        {
            myVec.push_back(myMap[uniformLBP[i]]);
            sumLBP+=myMap[uniformLBP[i]];
            myMap.erase(uniformLBP[i]);
        }
        for (itMap=myMap.begin(); itMap!=myMap.end(); itMap++)
        {
            lastLBP += itMap->second;
        }//剩余的非uniform pattern归为一类
        sumLBP+=lastLBP;
        myVec.push_back(lastLBP);
        for (itVec=myVec.begin(); itVec!=myVec.end(); itVec++)
        {
            vec_out.push_back(*itVec/double(sumLBP));
        }//归一化处理，使用直方图进行表示，包含59个bins
        for(int i=0;i<vec_out.size();i++)
        {
            cout<<vec_out[i]<<" ";
        }
        cout<<endl;
}
