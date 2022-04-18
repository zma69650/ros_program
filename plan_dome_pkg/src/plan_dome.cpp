//
// Created by lab306 on 2022/1/20.
//
#include "ros/ros.h"
#include "ros/ros.h"
#include "iostream"
#include "string"
#include "string.h"
#include "math.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_broadcaster.h"
#include "time.h"
#include "iostream"
#include "fstream"
#include "iostream"

int main(int argc,char** argv)
{
    const double _pi = 3.1415926535898;
    bool is_circle = false;

    ros::init(argc,argv,"plan_dome");
    ros::NodeHandle n("~");
    ros::Publisher PlanPub = n.advertise<geometry_msgs::Twist>("my_cmd_vel",50);

    ros::Publisher PlanPath = n.advertise<nav_msgs::Path>("plan_path",1, true);
    nav_msgs::Path path;
    ros::Rate loop_rate(10);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "odom";

    std::ofstream  data;
    data.open("/home/lab306/catkin_ws/src/robot_start/plan_path.txt",std::ios::trunc);

    float  vx=0.05;
    float  vy=0.0;
    float  w=0.1;
    float x=0;
    float y=0;
    float th=0;
    ros::Time lasttime,currenttime,starttime;
    lasttime = ros::Time::now();
    starttime = ros::Time::now();
    ROS_INFO("Plan Circle Start!");
    while (ros::ok())
    {
        currenttime = ros::Time::now();
        float  dt = (currenttime-lasttime).toSec();

        //publish plan
        geometry_msgs::Twist  circle;
        if(is_circle) {
            float _t = (currenttime-starttime).toSec();
            circle.linear.x = 0.1 * sin(0.1*_t);
            circle.linear.y = 0.1 * cos(0.1*_t);
            std::cout << "vx: "  << circle.linear.x << "  vy:  " << circle.linear.y << std::endl;
            circle.angular.z = 0.0;
        }
        else
        {
            float _t = (currenttime-starttime).toSec();
            circle.linear.x = 0.1 * cos(0.05*_t);
            circle.linear.y = -0.1 * sin(0.025*_t);
            circle.angular.z = -(0.025*cos(0.025*_t)*cos(0.05*_t)+0.05*sin(0.025*_t)*sin(0.05*_t))/(cos(0.05*_t)*cos(0.05*_t)+sin(0.025*_t)*sin(0.025*_t));
            std::cout << "vx: "  << circle.linear.x << "  vy:  " << circle.linear.y << "  vw:  " <<  circle.angular.z << std::endl;
            //circle.angular.z = 0.0;
        }
        PlanPub.publish(circle);
        double delta_x,delta_y,delta_th;
        //record plan
        if(is_circle){
            delta_x = (vx * cos(th) - vy * sin(th)) * dt;
            delta_y = (vx * sin(th) + vy * cos(th)) * dt;
            delta_th = w * dt;

            x+=delta_x;
            y+=delta_y;
            th+=delta_th;
        }
        else{
            delta_x = circle.linear.x*dt;
            delta_y = circle.linear.y*dt;
            delta_th = 0.0;

            x+=delta_x;
            y+=delta_y;
            th+=delta_th;
        }

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y=y;
        pose.pose.position.z=0.0;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(th);
        pose.pose.orientation.x = q.x;
        pose.pose.orientation.y = q.y;
        pose.pose.orientation.z = q.z;
        pose.pose.orientation.w = q.w;
        pose.header.stamp = currenttime;
        pose.header.frame_id = "odom";
        path.poses.push_back(pose);
        PlanPath.publish(path);
        //double t = currenttime.toNSec();
        size_t timestamp = time(NULL);
        data<<timestamp<< " " <<x<< " " <<y<< " " <<0.0<< " " <<q.x<< " " <<q.y<< " " <<q.z<< " " <<q.w<<std::endl;

        lasttime = currenttime;
        loop_rate.sleep();
        ros::spinOnce();

    }
    data.close();
    ROS_INFO("Path Write!");

    return 0;
}