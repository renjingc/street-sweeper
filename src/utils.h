#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <sys/stat.h>
#include <string>
#include <sstream>

using namespace std;

namespace strtool
{

    //模板函数：将string类型变量转换为常用的数值类型（此方法具有普遍适用性）
    template <class Type>
    Type stringToNum(const string str)
    {
        istringstream iss(str);
        Type num;
        iss >> num;
        return num;
    }
    template <class Type>
    string numToString(const Type num)
    {
        stringstream ss;
        ss << num;
        return ss.str();
    }

    int mkpath(std::string s,mode_t mode);

    string trim(const string& str);

    int split(const string& str, vector<string>& ret_, string sep);

    string replace(const string& str, const string& src, const string& dest);

    /*
     * @function: 获取cate_dir目录下的所有文件名
     * @param: cate_dir - string类型
     * @result：vector<string>类型
    */
    vector<string> myGetFiles(string cate_dir);
}

#endif // UTILS_H

