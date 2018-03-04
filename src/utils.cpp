#include "utils.h"

namespace strtool
{

    int mkpath(std::string s,mode_t mode=0755)
    {
        size_t pre=0,pos;
        std::string dir;
        int mdret;

        if(s[s.size()-1]!='/'){
            // force trailing / so we can handle everything in loop
            s+='/';
        }

        while((pos=s.find_first_of('/',pre))!=std::string::npos){
            dir=s.substr(0,pos++);
            pre=pos;
            if(dir.size()==0) continue; // if leading / first time is 0 length
            if((mdret=::mkdir(dir.c_str(),mode)) && errno!=EEXIST){
                return mdret;
            }
        }
        return mdret;
    }

    string trim(const string& str)
    {
        string::size_type pos = str.find_first_not_of(' ');
        if (pos == string::npos)
        {
            return str;
        }
        string::size_type pos2 = str.find_last_not_of(' ');
        if (pos2 != string::npos)
        {
            return str.substr(pos, pos2 - pos + 1);
        }
        return str.substr(pos);
    }

    int split(const string& str, vector<string>& ret_, string sep = ",")
    {
        if (str.empty())
        {
            return 0;
        }

        string tmp;
        string::size_type pos_begin = str.find_first_not_of(sep);
        string::size_type comma_pos = 0;

        while (pos_begin != string::npos)
        {
            comma_pos = str.find(sep, pos_begin);
            if (comma_pos != string::npos)
            {
                tmp = str.substr(pos_begin, comma_pos - pos_begin);
                pos_begin = comma_pos + sep.length();
            }
            else
            {
                tmp = str.substr(pos_begin);
                pos_begin = comma_pos;
            }

            if (!tmp.empty())
            {
                ret_.push_back(tmp);
                tmp.clear();
            }
        }
        return 0;
    }

    string replace(const string& str, const string& src, const string& dest)
    {
        string ret;

        string::size_type pos_begin = 0;
        string::size_type pos       = str.find(src);
        while (pos != string::npos)
        {
            cout <<"replacexxx:" << pos_begin <<" " << pos <<"\n";
            ret.append(str.data() + pos_begin, pos - pos_begin);
            ret += dest;
            pos_begin = pos + 1;
            pos       = str.find(src, pos_begin);
        }
        if (pos_begin < str.length())
        {
            ret.append(str.begin() + pos_begin, str.end());
        }
        return ret;
    }

    /*
     * @function: 获取cate_dir目录下的所有文件名
     * @param: cate_dir - string类型
     * @result：vector<string>类型
    */
    vector<string> myGetFiles(string cate_dir)
    {
            vector<string> files;//存放文件名

    #ifdef WIN32
            _finddata_t file;
            long lf;
            //输入文件夹路径
            if ((lf=_findfirst(cate_dir.c_str(), &file)) == -1) {
                    cout<<cate_dir<<" not found!!!"<<endl;
            } else {
                    while(_findnext(lf, &file) == 0) {
                            //输出文件名
                            //cout<<file.name<<endl;
                            if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
                                    continue;
                            files.push_back(file.name);
                    }
            }
            _findclose(lf);
    #endif

    #ifdef linux
            DIR *dir;
            struct dirent *ptr;
            char base[1000];

            if ((dir=opendir(cate_dir.c_str())) == NULL)
            {
                    perror("Open dir error...");
                    exit(1);
            }

            while ((ptr=readdir(dir)) != NULL)
            {
                    if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
                            continue;
                    else if(ptr->d_type == 8)    ///file
                            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
                            files.push_back(ptr->d_name);
                    else if(ptr->d_type == 10)    ///link file
                            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
                            continue;
                    else if(ptr->d_type == 4)    ///dir
                    {
                            files.push_back(ptr->d_name);
                            /*
                            memset(base,'\0',sizeof(base));
                            strcpy(base,basePath);
                            strcat(base,"/");
                            strcat(base,ptr->d_nSame);
                            readFileList(base);
                            */
                    }
            }
            closedir(dir);
    #endif

            //排序，按从小到大排序
            sort(files.begin(), files.end());
            return files;
    }
}

