//
// Created by lab306 on 2022/3/28.
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
int main(int argc, char** argv)
{


    //Markers test
    ros::init(argc,argv,"lines");
    ros::NodeHandle n;
    ros::Publisher markers_pub = n.advertise<visualization_msgs::Marker>("visualization_markers",10);

    ros::Rate r(30);
    visualization_msgs::Marker lines;
    float f =0.0;
    while(ros::ok())
    {

        lines.header.frame_id = "/odom";
        lines.header.stamp = ros::Time::now();
        lines.lifetime = ros::Duration();

        lines.ns = "double_lines";
        lines.action = visualization_msgs::Marker::ADD;
        lines.id = 0;

        lines.type = visualization_msgs::Marker::LINE_LIST;
        lines.pose.orientation.w=1.0;
        lines.scale.x = 0.1;
        lines.color.r = 1.0;
        lines.color.a = 1.0;

        geometry_msgs::Point p;
        p.x = 0;
        p.y = 1;
        p.z = 0;
        lines.points.push_back(p);
        p.x+=10.0;
        lines.points.push_back(p);
        markers_pub.publish(lines);

        geometry_msgs::Point p2;
        p2.x = 0;
        p2.y = -1;
        p2.z = 0;
        lines.points.push_back(p2);
        p2.x+=10.0;
        lines.points.push_back(p2);
        markers_pub.publish(lines);


        r.sleep();
        //f+=1.0;

    }





    //tf test
/*    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    ros::Publisher odom_pub;
    odom_pub = n.advertise<nav_msgs::Odometry>("od", 50);//modify
    ros::Publisher path_pub;
    path_pub = n.advertise<nav_msgs::Path>("ph",100,true);
    ros::Publisher path_pub2;
    path_pub2 = n.advertise<nav_msgs::Path>("path",100,true);
    ros::Rate r(10);

    tf::TransformBroadcaster broadcaster;

    float i=0.1;
    float j=0;
    float a=0.02;
    float b=0.01;
    bool odom_flag = true;
    bool path_flag = true;
    bool path2_flag = true;
    nav_msgs::Path odom_path;
    nav_msgs::Path odom_path2;
    while(ros::ok()){
        if(odom_flag)
        {
            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = "base_link";
            odom.child_frame_id = "laser";


            odom.pose.pose.position.x = i;
            odom.pose.pose.position.y = j;
            odom.pose.pose.position.z = 0.0;


            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);


            odom.pose.pose.orientation = odom_quat;


            odom.twist.twist.linear.x = 0;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = 0;


            odom_pub.publish(odom);
            ROS_INFO("odom publish");
        }
        if(path_flag)
        {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = i;
            pose_stamped.pose.position.y = j;

            geometry_msgs::Quaternion path_quat = tf::createQuaternionMsgFromYaw(0);
            pose_stamped.pose.orientation.x = path_quat.x;
            pose_stamped.pose.orientation.y = path_quat.y;
            pose_stamped.pose.orientation.z = path_quat.z;
            pose_stamped.pose.orientation.w = path_quat.w;

            pose_stamped.header.stamp = ros::Time::now();
            odom_path.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "odom";
            odom_path.header.frame_id = "base_link";

            odom_path.poses.push_back(pose_stamped);

            path_pub.publish(odom_path);
            ROS_INFO("path publish");
        }
        if(path2_flag)
        {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = -i;
            pose_stamped.pose.position.y = -j;

            geometry_msgs::Quaternion path_quat = tf::createQuaternionMsgFromYaw(0);
            pose_stamped.pose.orientation.x = path_quat.x;
            pose_stamped.pose.orientation.y = path_quat.y;
            pose_stamped.pose.orientation.z = path_quat.z;
            pose_stamped.pose.orientation.w = path_quat.w;

            pose_stamped.header.stamp = ros::Time::now();
            odom_path2.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "odom";
            odom_path2.header.frame_id = "base_link";

            odom_path2.poses.push_back(pose_stamped);

            path_pub2.publish(odom_path2);
            ROS_INFO("path2 publish");
        }


//发布坐标变换
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(
                                                               tf::Quaternion(0, 0, 0+i, 1), //四元数
                                                               tf::Vector3(-0.25, 0.0+j, 0.0)),
                                                       ros::Time::now(),
                                                       "base_link",
                                                       "laser"));
        i+=b;
        j+=a;

        if(j>=3||j<=-3)
            a=-a;
        if(i>=1||i<=-1)
            b=-b;

        r.sleep();
        ros::spinOnce();
    }*/
}