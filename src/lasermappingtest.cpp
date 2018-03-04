#include <ros/ros.h>

#include "lasermapping.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "laserMapping");
    LaserMapping lm;
    lm.startLiveSlam();
    ros::spin();
    return 0;
}
