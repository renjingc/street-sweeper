#ifndef NODEGRIDMAP_H
#define NODEGRIDMAP_H

#include <iostream>
#include <vector>
#include <fstream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
class Position
{
public:
	double x, y;
	double theta;
};
class NodeGridMap
{
public:
    NodeGridMap(); // default constructor of the occupancy grid
    NodeGridMap(double width, double high, double _cellWidth, double _cellHigh); // grid defined by the user
    NodeGridMap(int xCount_, int yCount_, double _cellWidth, double _cellHigh);
    NodeGridMap(const NodeGridMap& other);
    ~NodeGridMap();
	double getXWidth() const{ return xWidth; }
	double getYHigh() const{ return yHigh; }
	void setXWidth(double _xWidth){ xWidth = _xWidth; }
	void setYHigh(double _yHigh){ yHigh = _yHigh; }

	double getCellWidth() const{ return cellWidth; }
	double getCellHigh() const{ return cellHigh; }
	void setCellWidth(double _cellWidth){ cellWidth = _cellWidth; }
	void setCellHigh(double _cellHigh){ cellHigh = _cellHigh; }

	int getXCount() const{ return xCount; }
	int getYCount() const{ return yCount; }
	void setXCount(int _xCount){ xCount = _xCount; }
	void setYCount(int _yCount){ yCount = _yCount; }

	Position getPosition() const{ return p; }
	void setPosition(Position _p){ p = _p; }

	int* getCells() const{ return cells; }
	void setCells(int* _cells){ cells = _cells; }
    void setCell(int x, int y, int data) { cells[x + y*xCount] = data; }
	int getCell(int x, int y) const{ return cells[x + y*xCount]; }
    void setCenterX(int x){center_x=x;}
    void setCenterY(int y){center_y=y;}
    int getCenterX()const{return center_x;}
    int getCenterY()const{return center_y;}
    Position getOrigin()const {return origin;}
    void setOrigin(Position p){origin=p;}

	void init(double width, double high, double _cellWidth, double _cellHigh);
    void init(int xCount_, double yCount_, double _cellWidth, double _cellHigh);
	int update(Position _p, bool isCross, vector<double> range);
	void show();
	void load(string fileName);
	void writeConfig(int xCount, int yCount, double cellWidth, double cellHigh, string fileName);//cellWidth，cellHigh为厘米单位，
	double power(double a);

private:
    Position origin;
    int center_x,center_y;
    double xWidth, yHigh;//设置覆盖区域大小，单位为米
    double cellWidth, cellHigh;//设置一个栅格的大小,单位为米
    int xCount, yCount;//这块区域节点数
    int *cells;//十字节点表,0为空，1为已走，2为障碍
    Position p;//当前位置

};

#endif
