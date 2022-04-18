//
// Created by lab306 on 2022/1/11.
//
#include "Lab306_robot.h"
#include "kvaser.h"
int  main(int argc,char **argv)
{

    /*ros init*/
    ros::init(argc,argv,"base_controller");
    ROS_INFO("base controller node start! ");
    Robot_start_object Robot_Control(true);
    //while(1);
    Robot_Control.ReadAndWriteLoopProcess();
    return 0;
}