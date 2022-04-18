//
// Created by lab306 on 2022/1/17.
//
#include "pf.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"pf");
    ROS_INFO("The Particle Filter Node Start!");\
    PF pf(50);
    pf.ParticleFilterLoopProcess();
    return 0;
}


