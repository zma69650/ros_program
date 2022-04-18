//
// Created by lab306 on 2022/1/11.
//

#ifndef SRC_LAB306_ROBOT_H
#define SRC_LAB306_ROBOT_H

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

#include "kvaser.h"

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "tf/transform_broadcaster.h"
#include "iostream"
#include "fstream"
#include "vector"
#include "time.h"
#include "visualization_msgs/Marker.h"
//#define SPEEDX0 8192*60/2/PI/0.063
#define SPEEDX0 1232198.75249

using namespace std;




const double odom_pose_covariance[36] = {5e-4, 0, 0, 0, 0, 0,
                                         0, 5e-4, 0, 0, 0, 0,
                                         0, 0, 1e6, 0, 0, 0,
                                         0, 0, 0, 1e6, 0, 0,
                                         0, 0, 0, 0, 1e6, 0,
                                         0, 0, 0, 0, 0, 1e3};
const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
                                          0, 1e-3, 1e-9, 0, 0, 0,
                                          0, 0, 1e6, 0, 0, 0,
                                          0, 0, 0, 1e6, 0, 0,
                                          0, 0, 0, 0, 1e6, 0,
                                          0, 0, 0, 0, 0, 1e-9};

const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
                                          0, 1e-3, 0, 0, 0, 0,
                                          0, 0, 1e6, 0, 0, 0,
                                          0, 0, 0, 1e6, 0, 0,
                                          0, 0, 0, 0, 1e6, 0,
                                          0, 0, 0, 0, 0, 1e3};
const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
                                           0, 1e-3, 1e-9, 0, 0, 0,
                                           0, 0, 1e6, 0, 0, 0,
                                           0, 0, 0, 1e6, 0, 0,
                                           0, 0, 0, 0, 1e6, 0,
                                           0, 0, 0, 0, 0, 1e-9};




class Robot_start_object
{
public:
    Robot_start_object(bool flag);
    ~Robot_start_object();

    void cmd_velCallback(const geometry_msgs::Twist &twist_aux);
    void odom_publisher();
    inline void compute_theta();

    inline void read_Velocity();
    void update_odom();

    bool ReadAndWriteLoopProcess();
    void run_circle();
public:
    //
    //rel wheel velocity

       //world velocity
    float mVx,mVy,mVw;
    float mV1,mV2,mV3;
    float mTheta;

      //robot model
    float mL=0.26;
    float mR=0.0631;

    //odom
    float oV1,oV2,oV3;
    float oVx,oVy,oVw;
    float oX,oY,oTheta;
    float mRealTheta;

    float mdt;

    Eigen::Matrix3d calib_matrix;

private:
    string robot_frame_id,smoother_cmd_vel;
    bool mbIsPubOdom,mbIsPubPath;
    bool mbIsPubMarkers;

    //ros node define
    ros::NodeHandle n;
    ros::Time current_time,last_time;




    ros::Subscriber cmd_vel_sub;
    ros::Publisher odom_pub;
    ros::Publisher path_pubilsher;
    ros::Publisher markers_pub;
    nav_msgs::Path odom_path;

    tf::TransformBroadcaster odom_broadcaster;

    Kvaser *ros_kvaser;

    vector<size_t> mTimeStamp;



};




#endif //SRC_LAB306_ROBOT_H
