//
// Created by lab306 on 2022/3/21.
//
#include "calib.h"




int main(int argc,char** argv)
{

    /*ros init*/
    ros::init(argc,argv,"calib");
    ROS_INFO("calib node start! ");
    Calib calib;

    calib.CalibLoopProcess();

    return 0;

}
