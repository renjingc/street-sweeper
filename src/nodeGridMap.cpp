#include "nodeGridMap.h"

#define DEFAULTWIDTH 20
#define DEFAULTHIGH 20
#define DEFAULTCELLWIDTH 0.5
#define DEFAULTCELLHIGH 0.5

#define PI 3.14159265

double NodeGridMap::power(double a) { //simple function to calculate power of the value
    double b = a * a;
    return b;
}

NodeGridMap::NodeGridMap()
{

}

NodeGridMap::NodeGridMap(double width, double high, double _cellWidth, double _cellHigh):
        xWidth(width),yHigh(high),cellWidth(_cellWidth),cellHigh(_cellHigh)
{
    origin.x=0.0;
    origin.x=0.0;
    origin.theta=0.0;
    xCount = (int)floor(double((double)xWidth / (double)cellWidth)) + 1;
    yCount = (int)floor(double((double)yHigh / (double)cellHigh)) + 1;
    center_x=(int)round(xCount/2);
    center_y=(int)round(yCount/2);
    cells = new int[xCount*yCount];
    for (int i = 0; i < xCount; i++)
    {
        for (int j = 0; j < yCount; j++)
            cells[i + j * xCount] = 0;
    }
}
NodeGridMap::NodeGridMap(const NodeGridMap& other)
{
    //*this=other;
    this->origin=other.origin;
    this->xCount=other.xCount;
    this->yCount=other.yCount;
    this->center_x=other.center_x;
    this->center_y=other.center_y;
    this->xWidth=other.xWidth;
    this->yHigh=other.yHigh;
    this->cellWidth=other.cellWidth;
    this->cellHigh=other.cellHigh;
    this->p=other.p;
    this->cells=other.cells;
}

NodeGridMap::NodeGridMap(int xCount_, int yCount_, double _cellWidth, double _cellHigh):
    xCount(xCount_),yCount(yCount_),cellWidth(_cellWidth),cellHigh(_cellHigh)
{
    origin.x=0.0;
    origin.x=0.0;
    origin.theta=0.0;
    center_x=(int)round(xCount/2);
    center_y=(int)round(yCount/2);
    xWidth=xCount*cellWidth;
    yHigh=yCount*cellHigh;
    cells = new int[xCount*yCount];
    for (int i = 0; i < xCount; i++)
    {
        for (int j = 0; j < yCount; j++)
            cells[i + j * xCount] = 0;
    }
}

//NodeMap::NodeMap() :xWidth(DEFAULTWIDTH), yHigh(DEFAULTHIGH), cellWidth(DEFAULTCELLWIDTH), cellHigh(DEFAULTCELLHIGH)
//{
//	/*
//	xCount = floor(double((double)xWidth / (double)cellWidth)) + 1;
//	yCount = floor(double((double)yHigh / (double)cellHigh)) + 1;
//	cells = new int[xCount*yCount];
//	for (int i = 0; i < xCount; i++) {
//		for (int j = 0; j < yCount; j++)
//			cells[i + j * xCount] = 0;
//	}*/
//}
NodeGridMap::~NodeGridMap()
{
	/*
    if(cells)
		delete cells;
		*/
}

void NodeGridMap::init(double width, double high, double _cellWidth, double _cellHigh)
{
    origin.x=0.0;
    origin.x=0.0;
    origin.theta=0.0;
	xWidth = width;
	yHigh = high;
	cellWidth = _cellWidth;
	cellHigh = _cellHigh;

	xCount = (int)floor(double((double)xWidth / (double)cellWidth)) + 1;
	yCount = (int)floor(double((double)yHigh / (double)cellHigh)) + 1;

    center_x=(int)round(xCount/2);
    center_y=(int)round(yCount/2);

//	if (cells)
//		delete cells;
	cells = new int[xCount*yCount];
	for (int i = 0; i < xCount; i++) {
		for (int j = 0; j < yCount; j++)
			cells[i + j * xCount] = 0;
	}
}
void NodeGridMap::init(int xCount_, double yCount_, double _cellWidth, double _cellHigh)
{
    origin.x=0.0;
    origin.x=0.0;
    origin.theta=0.0;
    center_x=(int)round(xCount/2);
    center_y=(int)round(yCount/2);
    xCount = xCount_;
    yCount = yCount_;
    cellWidth = _cellWidth;
    cellHigh = _cellHigh;
    xWidth=xCount*_cellWidth;
    yHigh=yCount*_cellHigh;

//	if (cells)
//		delete cells;
    cells = new int[xCount*yCount];
    for (int i = 0; i < xCount; i++) {
        for (int j = 0; j < yCount; j++)
            cells[i + j * xCount] = 0;
    }
}
int NodeGridMap::update(Position _p, bool isCross, vector<double> range)
{
	p = _p;
	if (p.x<0 || p.x>xWidth || p.y<0 || p.y>yHigh)
	{
		cout << "position is out" << endl;
		return 0;
	}
	if (isCross)
	{
		int x,y;
		if (p.theta < 0.6 || p.theta>2 * PI - 0.6)
		{
			y = (int)floor(p.y / cellHigh) + 2;
			x = (int)floor(p.x / cellWidth) + 1;
			if (y < yCount&&x<xCount)
			{
				cells[x + y*xCount] = 1;
			}
		}
		else if (p.theta > PI / 2 - 0.6 || p.theta>PI / 2 + 0.6)
		{
			y = (int)floor(p.y / cellHigh) + 1;
			x = (int)floor(p.x / cellWidth);
			if (y < yCount&&x<xCount)
			{
				cells[x + y*xCount] = 1;
			}
		}
		else if (p.theta > PI  - 0.6 || p.theta>PI  + 0.6)
		{
			y = (int)floor(p.y / cellHigh);
			x = (int)floor(p.x / cellWidth) + 1;
			if (y < yCount&&x<xCount)
			{
				cells[x + y*xCount] = 1;
			}
		}
		else if (p.theta > 3*PI / 2 - 0.6 || p.theta>3*PI / 2 + 0.6)
		{
			y = (int)floor(p.y / cellHigh) + 1;
			x = (int)floor(p.x / cellWidth) + 2;
			if (y < yCount&&x<xCount)
			{
				cells[x + y*xCount] = 1;
			}
		}
	}
	for (int i = 0; i < range.size(); i++)
	{

	}
}

void NodeGridMap::load(string fileName)
{
	ifstream conf_file(fileName);

	if (!conf_file.is_open())
	{
		cerr << "fail to load " << fileName << endl;
		exit(1);
	}

	string line;
	vector<int> data;
	while (conf_file.good())
	{
		getline(conf_file, line);
		istringstream line_s(line);
		string field;
		line_s >> field;
		if (field.compare("xCount:")==0)
		{
			line_s >> this->xCount;
		}
		else if (field.compare("yCount:") == 0)
		{
			line_s >> this->yCount;
		}
		else if (field.compare("cellWidth:") == 0)
		{
			line_s >> this->cellWidth;
		}
		else if (field.compare("cellHigh:") == 0)
		{
			line_s >> this->cellHigh;
		}
		else if (field.compare("data:") == 0)
		{
			int a,size;
			line_s >> size;
			for (int i = 0; i < size; i++)
			{
				line_s >> a;
				data.push_back(a);
			}
		}
	}
	conf_file.close();
	xWidth = xCount*cellWidth;
	yHigh = yCount*cellHigh;

	cells = new int[xCount*yCount];
	for (int i = 0; i < xCount; i++) 
	{
		for (int j = 0; j < yCount; j++)
			cells[i + j * xCount] = data[i+j*xCount];
	}

    origin.x=0.0;
    origin.x=0.0;
    origin.theta=0.0;
    center_x=(int)round(xCount/2);
    center_y=(int)round(yCount/2);
}

void NodeGridMap::show()
{
	if (xCount != 0 && yCount != 0)
	{
        cv::Point p0((int)(xWidth - 0*cellWidth), (int)(yHigh - 0*cellHigh)),p1((int)(xWidth - 1*cellWidth), (int)(yHigh - 0*cellHigh));//,p2(1,0);
        Mat image(Size((int)(xWidth + cellWidth), (int)(yHigh + cellHigh)), CV_8UC3);
        int radius=1;
        int thickness=1;
		for (int i = 0; i < xCount; i++)
		{
			for (int j = 0; j < yCount; j++)
			{
				Point p((int)(xWidth - i*cellWidth), (int)(yHigh - j*cellHigh));
                //未走的0,白色
                if (cells[i+j*xCount] == 0)
				{
                    circle(image, p, radius, Scalar(51,51,51),thickness);
				}
                //已走的，浅黄
                else if (cells[i + j*xCount] == 20)
				{
                    circle(image, p, radius, Scalar(0,0,255), thickness);
				}
                //障碍10,蓝色
                else if (cells[i + j*xCount] == 10)
				{
                    circle(image, p, radius, Scalar(255,0, 0), thickness);
				}
                //gate,红色
                else if (cells[i + j*xCount] == 25)
                {
                    circle(image, p, radius, Scalar(0,0,255), thickness);
                }
                //下一轮,绿色
                else if (cells[i + j*xCount] == 30)
                {
                    circle(image, p, radius, Scalar(0,255,0), thickness);
                }
                //K
                else if (cells[i + j*xCount] == 50)
                {
                    circle(image, p, radius, Scalar(147,20,255), thickness);
                }
			}
		}

		cv::imshow("Map", image);
        waitKey(1);
	}
}
double getDistance(Point p1, Point p2)
{
	return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}
int gXCount = 0, gYCount = 0;
double gXWidth = 0, gYHigh = 0;
double gCellWidth = 0, gCellHigh = 0;
void on_mouse(int event, int x, int y, int flags, void *ustc)
{
	static int count = 0;
	char temp[16];
	Point pt;
	int* cells = (int*)ustc;
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		pt = Point(x, y);
		Scalar color = CV_RGB(255, 0, 0);
		int minDist = 99999, minX = -1, minY = -1;

		for (int i = 0; i < gXCount; i++)
		{
			for (int j = 0; j < gYCount; j++)
			{
				Point p2((int)(gXWidth - i*gCellWidth), (int)(gYHigh - j*gCellHigh));
				double dist = getDistance(pt, p2);
				if (dist < minDist)
				{
					minDist = dist;
					minX = i;
					minY = j;
				}
			}
		}
		if (minX != -1 && minY != -1)
		{
			cells[minX + minY*gXCount] = 2;
		}
	}
	if (event == CV_EVENT_RBUTTONDOWN)
	{
		pt = Point(x, y);
		Scalar color = CV_RGB(255, 0, 0);
		int minDist = 99999, minX = -1, minY = -1;

		for (int i = 0; i < gXCount; i++)
		{
			for (int j = 0; j < gYCount; j++)
			{
				Point p2((int)(gXWidth - i*gCellWidth), (int)(gYHigh - j*gCellHigh));
				double dist = getDistance(pt, p2);
				if (dist < minDist)
				{
					minDist = dist;
					minX = i;
					minY = j;
				}
			}
		}
		if (minX != -1 && minY != -1)
		{
			cells[minX + minY*gXCount] = 0;
		}
	}
}
void NodeGridMap::writeConfig(int xCount, int yCount, double cellWidth, double cellHigh, string fileName)
{
	ofstream conf_file(fileName);
	string line; 
	line = "xCount: ";
	stringstream sxCount;
	sxCount << xCount;
	line = line + sxCount.str()+"\n";
	conf_file << line;

	line = "yCount: ";
	stringstream syCount;
	syCount << yCount;
	line = line + syCount.str()+"\n";
	conf_file << line;


	line = "cellWidth: ";
	stringstream scellWidth;
	scellWidth << cellWidth;
	line = line + scellWidth.str() + "\n";
	conf_file << line;

	line = "cellHigh: ";
	stringstream scellHigh;
	scellHigh << cellHigh;
	line = line + scellHigh.str() + "\n";
	conf_file << line;

	cells = new int[xCount*yCount];
	for (int i = 0; i < xCount; i++)
	{
		for (int j = 0; j < yCount; j++)
			cells[i + j * xCount] = 0;
	}

	xWidth = xCount*cellWidth;
	yHigh = yCount*cellHigh;

	gXCount = xCount;
	gYCount = yCount;
	gXWidth = xWidth;
	gYHigh = yHigh;
	gCellWidth = cellWidth;
	gCellHigh = cellHigh;

	while (1)
	{
		setMouseCallback("Map", on_mouse, (void*)cells);

		Mat image(Size((int)(xWidth + cellWidth), (int)(yHigh + cellHigh)), CV_8UC3);
		for (int i = 0; i < xCount; i++)
		{
			for (int j = 0; j < yCount; j++)
			{
				Point p((int)(xWidth - i*cellWidth), (int)(yHigh - j*cellHigh));
				if (cells[i + j*xCount] == 0)
				{
					circle(image, p, 3, Scalar(255, 0, 0), 3);
				}
				else if (cells[i + j*xCount] == 1)
				{
					circle(image, p, 3, Scalar(0, 0, 255), 3);
				}
				else if (cells[i + j*xCount] == 2)
				{
					circle(image, p, 3, Scalar(0, 0, 0), 3);
				}
			}
		}

		imshow("Map", image);
		char c = waitKey(33);
		if (c == 27)
			break;
	}
	line = "data: ";
	stringstream ssizeNum;
	int sizeNum = xCount*yCount;
	ssizeNum << sizeNum;
	line = line + ssizeNum.str();
	for (int i = 0; i < sizeNum; i++)
	{
		stringstream scell;
		scell << cells[i];
		line = line + " " + scell.str();
	}
	line = line + "\n";
	conf_file << line;
	conf_file.close();
}
