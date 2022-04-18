//
// Created by lab306 on 2022/3/21.
//

#ifndef ROBOT_START_CALIB_H
#define ROBOT_START_CALIB_H

#include "ros/ros.h"
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

#include "kvaser.h"

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "tf/transform_broadcaster.h"
#include "iostream"
#include "fstream"
#include "vector"
#include "time.h"
#include "mutex"
#include "thread"
#include <condition_variable>
using std::make_pair;

using namespace std;


class Calib{
public:
    Calib();
    ~Calib();
    void cmd_velCallback(const geometry_msgs::Twist &twist_aux);
    void CalibLoopProcess();
    void odom_Callback(const nav_msgs::Odometry &odom);
    void read_Velocity();

    void add_3row();
    void add_row();
    void ThreadGetOdom();
    void integration();

    void updateodom();
    bool solver();
    bool solver2();

private:
    ros::Time current_time,last_time;
    float mdt;
    //calibration varies
    float mL1,mL2,mL3;


    //world velocity
    float mVx,mVy,mVw;
    float mV1,mV2,mV3;
    float mTheta;

    float mL=0.26;
    float mR=0.0631;

    int solver_num;
    bool solver_begin;
    bool first_odom = true;

    std::mutex solver_mutex;

    std::mutex g_mutex;
    std::condition_variable g_cond;

    Eigen::Matrix<float,1,3> A_row;

    Eigen::Matrix<float,Eigen::Dynamic,3> A;
    Eigen::Matrix<float,Eigen::Dynamic,3> b;
    Eigen::Matrix<float,Eigen::Dynamic,3> A_A;
    Eigen::Matrix<float,Eigen::Dynamic,3> B_B;
    Eigen::Matrix<float,1,3> temp_b;

    Eigen::Matrix<float,Eigen::Dynamic,9> AA;
    Eigen::Matrix<float,3,1> d_b;
    Eigen::Matrix<float,Eigen::Dynamic,1> bb;

    vector<pair<float,Eigen::Matrix3f>> vJ;

    double w1,w2,w3;
    Eigen::Quaternionf last_q;
    Eigen::Quaternionf orin_q;
    nav_msgs::Odometry last_odom;
    float dx,dy,dtheta;
    float cur2orin_theta;

    float oTheta=0;

    string calib_odom,smoother_cmd_vel;
    //ros node define
    ros::NodeHandle n;

    ros::Subscriber cmd_vel_sub;
    ros::Subscriber odom_sub;


    Kvaser *ros_kvaser;

};


#endif //ROBOT_START_CALIB_H
