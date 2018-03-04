#define _CRT_SECURE_NO_WARNINGS
#include "findPath.h"
#include "nodeGridMap.h"

// Main
using namespace std;
using namespace cv;
int main(int argc, char *argv[])
{
    // Create a start state
    NodeGridMap* pnodeMap;
    pnodeMap->load("/home/ren/catkin_ws/src/car107/config.txt");
    PATH::MapSearchNode nodeStart;
    nodeStart.x = 1;// rand() % MAP_WIDTH;
    nodeStart.y = 1;// rand() % MAP_HEIGHT;

    // Define the goal state
    PATH::MapSearchNode nodeEnd;
    PATH::Point end = PATH::breadthFirstSearch(pnodeMap, PATH::Point(nodeStart.x, nodeStart.y));
    nodeEnd.x = end.getX();//rand()%MAP_WIDTH;
    nodeEnd.y = end.getY();
    std::cout << nodeStart.x << " " << nodeStart.y << " " << nodeEnd.x << " " << nodeEnd.y << std::endl;

    vector<PATH::MapSearchNode> result;
    getPath(pnodeMap, nodeStart, nodeEnd, result);

    for (int i = 0; i < result.size(); i++)
    {
        cout<<result[i].x<<" "<<result[i].y<<endl;
        pnodeMap->setCell(result[i].x, result[i].y, 20);
    }
    pnodeMap->show();
    waitKey(0);
    return 0;
}
