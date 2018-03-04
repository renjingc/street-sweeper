#define _CRT_SECURE_NO_WARNINGS
#include "findPath.h"


using namespace std;

namespace PATH
{
	NodeGridMap gmap;
    Point breadthFirstSearch(NodeGridMap* map, Point root)
	{
		std::queue<Point> nodeQueue;
		NodeGridMap setMap;
        setMap.init(map->getXCount(), map->getYCount(), map->getCellWidth(), map->getCellHigh());
		nodeQueue.push(root);
		Point node;
		Point end;
		end.setX(-1);
		end.setY(-1);
		int level = 1;
		setMap.setCell(root.getX(), root.getY(), level);
		while (!nodeQueue.empty())
		{
			node = nodeQueue.front();
			nodeQueue.pop();
			level++;
			if (node.getY() - 1 >= 0)
			{
				//上
                if (map->getCell(node.getX(), node.getY() - 1) != 30)
				{
					if (setMap.getCell(node.getX(), node.getY() - 1) == 0
                        && map->getCell(node.getX(), node.getY() - 1) != 10)
					{
						nodeQueue.push(Point(node.getX(), node.getY() - 1));
						setMap.setCell(node.getX(), node.getY() - 1, level);
					}
				}
				else
				{
					end.setX(node.getX());
					end.setY(node.getY() - 1);
					setMap.setCell(end.getX(), end.getY(), level);
					break;
				}
			}
            if (node.getX() + 1 < map->getXCount())
			{
				//右
                if (map->getCell(node.getX() + 1, node.getY()) != 30)
				{
					if (setMap.getCell(node.getX() + 1, node.getY()) == 0
                        && map->getCell(node.getX() + 1, node.getY()) != 10)
					{
						nodeQueue.push(Point(node.getX() + 1, node.getY()));
						setMap.setCell(node.getX() + 1, node.getY(), level);
					}
				}
				else
				{
					end.setX(node.getX() + 1);
					end.setY(node.getY());
					setMap.setCell(end.getX(), end.getY(), level);
					break;
				}

			}

            if (node.getY() + 1 < map->getYCount())
			{
				//下
                if (map->getCell(node.getX(), node.getY() + 1) != 30)
				{
					if (setMap.getCell(node.getX(), node.getY() + 1) == 0
                        && map->getCell(node.getX(), node.getY() + 1) != 10)
					{
						nodeQueue.push(Point(node.getX(), node.getY() + 1));
						setMap.setCell(node.getX(), node.getY() + 1, level);
					}
				}
				else
				{
					end.setX(node.getX());
					end.setY(node.getY() + 1);
					setMap.setCell(end.getX(), end.getY(), level);
					break;
				}
			}
			if (node.getX() - 1 >= 0)
			{
				//左
                if (map->getCell(node.getX() - 1, node.getY()) != 30)
				{
					if (setMap.getCell(node.getX() - 1, node.getY()) == 0
                        && map->getCell(node.getX() - 1, node.getY()) != 10)
					{
						nodeQueue.push(Point(node.getX() - 1, node.getY()));
						setMap.setCell(node.getX() - 1, node.getY(), level);
					}
				}
				else
				{
					end.setX(node.getX() - 1);
					end.setY(node.getY());
					setMap.setCell(end.getX(), end.getY(), level);
					break;
				}
			}
		}
		return end;
	}
	Point::Point()
	{
		_x = _y = 0;
	}
	Point::Point(int x, int y)
	{
		_x = x;
		_y = y;
	}
	int Point::getManhattanDistance(const Point& t) const
	{
		return std::abs(_x - t._x) + std::abs(_y - t._y);
	}
	double Point::getEuclidDistance(const Point& t) const
	{
		return sqrt((_x - t._x) * (_x - t._x) + (_y - t._y) * (_y - t._y));
	}
	bool Point::operator== (const Point& t) const
	{
		return _x == t._x && _y == t._y;
	}
	bool Point::operator< (const Point& t) const
	{
		if (_x == t._x)
		{
			return _y < t._y;
		}
		return _x < t._x;
	}
	int GetMap(int x, int y)
	{
		if( x < 0 ||x >= gmap.getXCount() ||
			y < 0 || y >= gmap.getYCount())
		{
			return 10;	 
		}

		return gmap.getCell(x,y);
	}

	bool MapSearchNode::IsSameState( MapSearchNode &rhs )
	{

		// same state in a maze search is simply when (x,y) are the same
		if( (x == rhs.x) &&
			(y == rhs.y) )
		{
			return true;
		}
		else
		{
			return false;
		}

	}

	void MapSearchNode::PrintNodeInfo()
	{
		char str[100];
		sprintf( str, "Node position : (%d,%d)\n", x,y );

		cout << str;
	}

	// Here's the heuristic function that estimates the distance from a Node
	// to the Goal. 

	float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
	{
		return fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y);	
	}

	bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
	{
		if( (x == nodeGoal.x) &&
			(y == nodeGoal.y) )
		{
			return true;
		}

		return false;
	}

	// This generates the successors to the given Node. It uses a helper function called
	// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
	// is done for each node internally, so here you just set the state information that
	// is specific to the application
	bool MapSearchNode::GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
	{

		int parent_x = -1; 
		int parent_y = -1; 

		if( parent_node )
		{
			parent_x = parent_node->x;
			parent_y = parent_node->y;
		}
	
		MapSearchNode NewNode;

		// push each possible move except allowing the search to go backwards

		if( (GetMap( x-1, y ) != 10) 
			&& !((parent_x == x-1) && (parent_y == y))
		  ) 
		{
			NewNode = MapSearchNode( x-1, y );
			astarsearch->AddSuccessor( NewNode );
		}	

		if( (GetMap( x, y-1 ) != 10) 
			&& !((parent_x == x) && (parent_y == y-1))
		  ) 
		{
			NewNode = MapSearchNode( x, y-1 );
			astarsearch->AddSuccessor( NewNode );
		}	

		if( (GetMap( x+1, y ) != 10)
			&& !((parent_x == x+1) && (parent_y == y))
		  ) 
		{
			NewNode = MapSearchNode( x+1, y );
			astarsearch->AddSuccessor( NewNode );
		}	

		
		if( (GetMap( x, y+1 ) != 10) 
			&& !((parent_x == x) && (parent_y == y+1))
			)
		{
			NewNode = MapSearchNode( x, y+1 );
			astarsearch->AddSuccessor( NewNode );
		}	

		return true;
	}

	// given this node, what does it cost to move to successor. In the case
	// of our map the answer is the map terrain value at this node since that is 
	// conceptually where we're moving

	float MapSearchNode::GetCost( MapSearchNode &successor )
	{
		return (float) GetMap( x, y );

	}
    void getPath(NodeGridMap* map,MapSearchNode nodeStart, MapSearchNode nodeEnd, vector<MapSearchNode>& result)
	{
        gmap.init(map->getXCount(), map->getYCount(), map->getCellWidth(), map->getCellHigh());
		//gmap = map;
        for (int i = 0; i < map->getXCount(); i++)
            for (int j = 0; j < map->getYCount(); j++)
                gmap.setCell(i,j,map->getCell(i,j));

		AStarSearch<MapSearchNode> astarsearch;
		astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd);
		unsigned int SearchState;
		unsigned int SearchSteps = 0;
		do
		{
			SearchState = astarsearch.SearchStep();
			SearchSteps++;
		} while (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);
		if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED)
		{
			//cout << "Search found goal state\n";
			MapSearchNode *node = astarsearch.GetSolutionStart();
			int steps = 0;
			//node->PrintNodeInfo();
			result.push_back(*node);
			for (;;)
			{
				node = astarsearch.GetSolutionNext();
				if (!node)
				{
					break;
				}
				//node->PrintNodeInfo();
				steps++;
				result.push_back(*node);
			};
			// Once you're done with the solution you can free the nodes up
			astarsearch.FreeSolutionNodes();

		}
		else if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED)
		{
			cout << "寻找停止,没找到目标\n";
		}

		// Display the number of loops the search went through

		astarsearch.EnsureMemoryFreed();
	}

}
