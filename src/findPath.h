#include "stlastar.h" // See header for copyright and usage information

#include <vector>
#include <queue>
#include <iostream>
#include <stdio.h>
#include <math.h>

#include "nodeGridMap.h"

using namespace std;

namespace PATH
{
	//属性宏
#define SYNTHESIZE(varType, varName, funName) \
protected: \
     varType varName; \
public: \
	virtual varType get##funName(void) const { \
		return varName; \
	} \
	virtual void set##funName(varType var) { \
	varName = var; \
}
	/***************************
	*    Point类
	*    表示一个二维坐标
	****************************/
	class Point
	{
		SYNTHESIZE(int, _x, X);
		SYNTHESIZE(int, _y, Y);
	public:
		Point();
		Point(int x, int y);
		//获得曼哈顿距离
		int getManhattanDistance(const Point& t) const;
		//获得欧几里得距离
		double getEuclidDistance(const Point& t) const;
		bool operator== (const Point& t) const;
		bool operator< (const Point& t) const;
	};
	class MapSearchNode
	{
	public:
		int x;	 // the (x,y) positions of the node
		int y;

		MapSearchNode() { x = y = 0; }
		MapSearchNode(int px, int py) { x = px; y = py; }

		float GoalDistanceEstimate(MapSearchNode &nodeGoal);
		bool IsGoal(MapSearchNode &nodeGoal);
		bool GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node);
		float GetCost(MapSearchNode &successor);
		bool IsSameState(MapSearchNode &rhs);

		void PrintNodeInfo();
	};

	int GetMap(int x, int y);
        void getPath(NodeGridMap* map, MapSearchNode nodeStart, MapSearchNode nodeEnd, vector<MapSearchNode>& result);
        Point breadthFirstSearch(NodeGridMap* map, Point root);
}
