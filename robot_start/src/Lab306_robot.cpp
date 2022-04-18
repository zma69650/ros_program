//
// Created by lab306 on 2022/1/11.
//
#include "Lab306_robot.h"
#include "fstream"


const int period_ms = 100;
Robot_start_object::Robot_start_object(bool flag)
{
    //Get Luncher file define value
    ros::NodeHandle nh_private("~");

    nh_private.param<std::string>("robot_frame_id",this->robot_frame_id,"base_link");
    nh_private.param<std::string>("smoother_cmd_vel", this->smoother_cmd_vel,"/my_cmd_vel");


    nh_private.param<bool>("PubPath",this->mbIsPubPath,true);
    nh_private.param<bool>("PubOdom",this->mbIsPubOdom, true);
    nh_private.param<bool>("PubMarkers", this->mbIsPubMarkers, false);


    //set zero



    oV1 = oV2 = oV3 = oVw = oVx = oVy = 0;
    oX = oY = oTheta = 0;
    calib_matrix<<0.740641,-0.000814,0.230436,0.267997,0.890413,-0.146980,0.127247,-0.084093,0.839484;


    this->cmd_vel_sub = n.subscribe(smoother_cmd_vel,100,&Robot_start_object::cmd_velCallback,this);

    this->odom_pub = n.advertise<nav_msgs::Odometry>("odom",50);

    this->path_pubilsher = n.advertise<nav_msgs::Path>("trajectory",100, true);

    this->markers_pub = n.advertise<visualization_msgs::Marker>("visualization_markers",10);


    ros_kvaser = new Kvaser();
    ros_kvaser->canInit(0);

    //connect motor
    ros_kvaser->connectMotor(1);
    ros_kvaser->connectMotor(2);
    ros_kvaser->connectMotor(3);

    //Enable motor
    ros_kvaser->motorEnable(1);
    ros_kvaser->motorEnable(2);
    ros_kvaser->motorEnable(3);
    //SPEED MODE mode
    ros_kvaser->modeChoose(1,ros_kvaser->SPEED_MODE);
    ros_kvaser->modeChoose(2,ros_kvaser->SPEED_MODE);
    ros_kvaser->modeChoose(3,ros_kvaser->SPEED_MODE);

    ROS_INFO("Motor init success");


}

Robot_start_object::~Robot_start_object() {

    //release motor

    ros_kvaser->motorDisable(1);
    ros_kvaser->motorDisable(2);
    ros_kvaser->motorDisable(3);
    ros_kvaser->canRelease();

    //write path data
    ofstream data;
    data.open("/home/lab306/catkin_ws/src/robot_start/trajectory.txt",ios::trunc);
    for(int i=0;i<odom_path.poses.size();i++)
    {
        float x = odom_path.poses.at(i).pose.position.x;
        float y = odom_path.poses.at(i).pose.position.y;
        float z = odom_path.poses.at(i).pose.position.z;
        float qx = odom_path.poses.at(i).pose.orientation.x;
        float qy = odom_path.poses.at(i).pose.orientation.y;
        float qz = odom_path.poses.at(i).pose.orientation.z;
        float qw = odom_path.poses.at(i).pose.orientation.w;
        data <<mTimeStamp.at(i)<< " " <<x <<" " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
    }
    data.close();



}

void Robot_start_object::cmd_velCallback(const geometry_msgs::Twist &twist_aux) {

    //step1.
    mVx=twist_aux.linear.x;
    mVy=twist_aux.linear.y;
    mVw=twist_aux.angular.z;
    //std::cout << "vx:  "  << mVx << "  vy:  " << mVy << "  Vw:  " << mVw << std::endl;
    //step2. compute theta
    //compute_theta();
    //mTheta=oTheta;
    mTheta = 0;
    //step3. whold to wheel
    mV1 = -1*mVx*cos(mTheta)-mVy*sin(mTheta)+mL*mVw;
    mV2 = mVx*cos(mTheta-PI/3)+mVy*sin(mTheta-PI/3)+mL*mVw;
    mV3 = mVx*cos(mTheta+PI/3)+mVy*sin(mTheta+PI/3)+mL*mVw;
    /*mV1 = -1*mVx+mL*mVw;
    mV2 = mVx*cos(-PI/3)+mVy*sin(-PI/3)+mL*mVw;
    mV3 = mVx*cos(+PI/3)+mVy*sin(+PI/3)+mL*mVw;*/
    //mV1 = 0.5*mVx + mL*mVw;
    //mV2 = -0.5*mVx + 0.57735*mVy +mL*mVw;
    //mV3 = -0.5*mVx - 0.57735*mVy +mL*mVw;
    //std::cout << "v1:  "  << mV1 << "  v2:  " << mV2 << "  V3:  " << mV3 << std::endl;

    //ROS_INFO("plan V1:%f  V2:%f   V3:%f ",mV1,mV2,mV3);
    //step4. begin move


    ros_kvaser->speedMode(1,mV1*SPEEDX0);
    ros_kvaser->speedMode(2,mV2*SPEEDX0);
    ros_kvaser->speedMode(3,mV3*SPEEDX0);

    ros_kvaser->beginMovement(1);
    ros_kvaser->beginMovement(2);

    ros_kvaser->beginMovement(3);


}




bool Robot_start_object::ReadAndWriteLoopProcess() {
    ros::Rate loop_rate(20);//循环频率(20HZ)
    this->last_time = ros::Time::now();
    odom_path.header.stamp = last_time;
    odom_path.header.frame_id ="odom";
    //static ofstream odom_data("/home/lab306/Documents/odom.txt",ios::out);
    while (ros::ok())
    {
        this->current_time = ros::Time::now();
        this->mdt = (current_time - last_time).toSec();

        odom_publisher();
        //ROS_INFO("real 1: %f  2: %f  3:%f",oV1,oV2,oV3);
        this->last_time = current_time;
        loop_rate.sleep();
        ros::spinOnce();
    }
}

void Robot_start_object::compute_theta() {
    mTheta += oVw;
}

void Robot_start_object::update_odom() {



    Eigen::Matrix3d matrix;

    matrix << -cos(oTheta)     ,-sin(oTheta)       , mL,
               cos(oTheta-PI/3), sin(oTheta - PI/3), mL,
               cos(oTheta+PI/3), sin(oTheta + PI/3), mL;
    read_Velocity();
    Eigen::Vector3d V3;
    V3 << oV1, oV2, oV3;

    Eigen::Vector3d Vw = matrix.inverse()*V3;
    oVx = Vw(0);
    oVy = Vw(1);
    oVw = Vw(2);

    Eigen::Vector3d d(oVx*mdt,oVy*mdt,oVw*mdt);
    Eigen::Vector3d dd = calib_matrix*d;


    oX += d(0);
    oY += d(1);
    oTheta += d(2);

}

 void Robot_start_object::read_Velocity() {
    oV1 = ros_kvaser->getVelocity(1, 2048, 60)*mR;
    oV2 = ros_kvaser->getVelocity(2, 2048, 60)*mR*1.0125;
    oV3 = ros_kvaser->getVelocity(3, 2048, 60)*mR;
    //std::cout<<"1:"<<oV1<<"\n2:"<<oV2<<"\n3:"<<oV3<<std::endl;

}

void Robot_start_object::odom_publisher() {


    update_odom();

    if(mbIsPubOdom)
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        /*odom.header.frame_id = "odom_combined";
        odom.child_frame_id = "base_footprint";*/
        odom.header.frame_id = "base_link";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = oX;
        odom.pose.pose.position.y = oY;
        odom.pose.pose.position.z = 0.0;

        //ROS_INFO("oTheta:%f,oX:%f,oY:%f",oTheta,oX,oY);
       geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(oTheta);
//
//        //first ,we'll publish the transform over tf
//        geometry_msgs::TransformStamped odom_trans;
//        odom_trans.header.stamp = ros::Time::now();
//        odom_trans.header.frame_id = "odom";
//        odom_trans.child_frame_id = this->robot_frame_id;
//
//        odom_trans.transform.translation.x =oX;
//        odom_trans.transform.translation.y = oY;
//        odom_trans.transform.translation.z = 0.0;
//
//        odom_trans.transform.rotation = odom_quat;
//        odom_broadcaster.sendTransform(odom_trans);

        //next,we'll publish the odomtry message over ROS

        odom.pose.pose.orientation = odom_quat;


        odom.twist.twist.linear.x = oVx;
        odom.twist.twist.linear.y = oVy;
        odom.twist.twist.angular.z = oVw;


       /* if(this->oVx == 0)
        {
            memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2));
            memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
        }
        else
        {
            memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));
            memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
        }*/

        memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));
        memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
        //std::cout << "robot_start:   "<<odom.pose.pose.position.x << odom.pose.pose.position.y  << endl;
        odom_pub.publish(odom);
    }


    //finally, publish path
    if(mbIsPubPath)
    {

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = oX;
        pose_stamped.pose.position.y = oY;

        geometry_msgs::Quaternion path_quat = tf::createQuaternionMsgFromYaw(oTheta);
        pose_stamped.pose.orientation.x = path_quat.x;
        pose_stamped.pose.orientation.y = path_quat.y;
        pose_stamped.pose.orientation.z = path_quat.z;
        pose_stamped.pose.orientation.w = path_quat.w;

        pose_stamped.header.stamp = current_time;
        odom_path.header.stamp = current_time;

        //pose_stamped.header.frame_id = "odom_combined";
        //odom_path.header.frame_id = "odom_combined";
        pose_stamped.header.frame_id = "base_link";
        odom_path.header.frame_id = "base_link";
        odom_path.poses.push_back(pose_stamped);
        path_pubilsher.publish(odom_path);
        size_t t = time(NULL);
        mTimeStamp.push_back(t);
    }

    if(mbIsPubMarkers)
    {
        visualization_msgs::Marker lines;
        //lines.header.frame_id = "/odom_combined";
        lines.header.frame_id = "/base_link";
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
    }


}

void Robot_start_object::run_circle()
{

}